"""Replay one bounded reference initialization from exact two-parent XML.

Read-only source diagnostic, never a selected map or independent turn repair.
This cannot substitute for the complete simultaneous source group solve.
"""
import argparse
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from scripts.solve_source_junction_group import written_long_seed
from scripts.rebuild_north_source_ports import fresh_seed
from scripts.gen_all import shp_source,_sha256
from scripts.build_source_geometry_candidate import dump
from spikes.measured_connector_caps import frames,composite_sources,needs_cap
from mapforge.validate.shp_boundary_fidelity import _origin,_project
from mapforge.ops.port_dependencies import PortDependencies
from mapforge.ops.long_source_seed import initialize,ReferenceInitializationError
from scripts.research_code_revision import bind_current_code
from scripts.fit_source_boundary_block import unchanged


def run(directory,output,rid,alternate_endpoint_seed=False,two_core=False,allow_open=False):
    directory=Path(directory).resolve();output=Path(output).resolve()
    output.mkdir(parents=True,exist_ok=False)
    info=json.loads((directory/'report.json').read_text(encoding='utf8'))
    path=Path(info['artifact']);base=Path(info['input'])
    if _sha256(path)!=info['sha256'] or _sha256(base)!=info['input_sha256']:
        raise ValueError('prepared or retained XML changed')
    frozen,code_changes=bind_current_code(info['source_hashes']);unchanged(frozen)
    root=ET.parse(path).getroot();old=ET.parse(base).getroot();graph=PortDependencies(root)
    parents={r['road'] for r in info['parents']}
    if rid not in info['affected_roads']:raise ValueError('explicit affected turn required')
    road=graph.roads[rid];a,b=frames(root,road);old_frames=frames(old,old.find(f"road[@id='{rid}']"))
    baseline=json.loads((base.parent/'report.json').read_text(encoding='utf8'))
    previous=next((r for r in baseline['connectors'] if r['road']==rid),None)
    if previous is None:previous=written_long_seed(old.find(f"road[@id='{rid}']"),old_frames)
    # Conservative diagnostic branch: explicit cap at every mutable source port.
    caps=tuple(needs_cap(f) or p.road in parents for f,p in zip((a,b),graph.connections[rid]))
    q,core,migration=fresh_seed(a,b,previous,old_frames,contact_caps=caps)
    if two_core:
        core=2;q=None
        migration=dict(method='explicit-two-core-long-reference-probe',caps=list(caps),
            no_generic_G2_existence_claim=True,minimum_reference_span_m=6.)
    source=shp_source();lat,lon=_origin(root)
    raw,support=composite_sources(root,road,source,lambda p:_project(p,lat,lon))
    bound={**frozen,**{str(p):_sha256(p) for p in (path,base,directory/'report.json',Path(__file__),
        ROOT/'mapforge/ops/long_source_seed.py',ROOT/'mapforge/ops/long_connector_chain.py')}}
    if alternate_endpoint_seed:
        # Different analytic Hermite seed, same fixed model and ALL constraints.
        if core!=3:raise ValueError('analytic endpoint seed requires three cores')
        q=None
    try:
        q,co,report=initialize(a,b,raw,parameters=q,core_count=core,max_iterations=40,contact_caps=caps,
            allow_open_reference=allow_open)
        report.update(parameters=q.tolist())
    except ReferenceInitializationError as exc:report=exc.report
    report.update(road=rid,prepared_input=str(path),prepared_input_sha256=info['sha256'],
        original_support=support,frames=[a,b],seed_migration=migration,
        alternate_endpoint_seed=alternate_endpoint_seed,input_hashes=bound,
        two_core_probe=two_core,
        code_changes_before_diagnosis=code_changes,
        simultaneous_optimization_started=False,source_changed=False,production_accepted=False)
    for file,sha in bound.items():
        if _sha256(Path(file))!=sha:raise ValueError('diagnostic bound input changed')
    dump(output/'report.json',report)
    print(report['status'],rid,flush=True)
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('directory');p.add_argument('output');p.add_argument('--road',required=True)
    p.add_argument('--alternate-endpoint-seed',action='store_true');p.add_argument('--two-core',action='store_true')
    p.add_argument('--allow-open',action='store_true');a=p.parse_args()
    if a.two_core and a.alternate_endpoint_seed:raise ValueError('select one explicit model probe')
    run(a.directory,a.output,a.road,a.alternate_endpoint_seed,a.two_core,a.allow_open);raise SystemExit(2)
