"""Bind a multi-parent trial to ALL explicit junction movements, without PASS reuse."""
import argparse
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from scripts.gen_all import _sha256
from scripts.build_source_geometry_candidate import dump
from scripts.research_code_revision import bind_current_code
from scripts.fit_source_boundary_block import unchanged
from mapforge.ops.port_dependencies import PortDependencies


def run(directory,output):
    directory=Path(directory).resolve();output=Path(output).resolve()
    trial=json.loads((directory/'report.json').read_text(encoding='utf8'))
    saved=json.loads((directory/'joint-state.json').read_text(encoding='utf8'))
    if not trial['artifact'] or not trial['simultaneous_parent_connector_variables']:
        raise ValueError('actual simultaneous group XML required')
    path=Path(trial['artifact']);base=Path(trial['input'])
    if _sha256(path)!=trial['sha256'] or _sha256(base)!=trial['input_sha256']:
        raise ValueError('actual trial or baseline changed')
    old=json.loads((base.parent/'report.json').read_text(encoding='utf8'))
    graph=PortDependencies(ET.parse(path).getroot());graph.validate_junction_table()
    oldgraph=PortDependencies(ET.parse(base).getroot());oldgraph.validate_junction_table()
    if graph.connections!=oldgraph.connections:raise ValueError('explicit traffic movements changed')
    hashes,changes=bind_current_code(trial['source_hashes']);unchanged(hashes)
    incoming={r['road']:r for r in trial['connectors']}
    required={rid for rid,ports in graph.connections.items() if any(p.road in trial['parent_components'] for p in ports)}
    if (set(incoming)!=required or set(trial['affected_roads'])!=required
            or len(incoming)!=len(trial['connectors'])):raise ValueError('incomplete shared group results')
    output.mkdir(parents=True,exist_ok=False);parent_states={}
    for row in saved['parents']:
        rid=row['road'];component=Path(trial['parent_components'][rid])
        state=json.loads((component/'shared-state.json').read_text(encoding='utf8'))
        if state['model_sha']!=row['model_sha']:raise ValueError('saved shared source model differs')
        state['coefficients']=row['coefficients'];state['source_hashes']=hashes
        dest=output/('north-state' if rid=='10' else 'parent-'+rid+'-state');dest.mkdir()
        dump(dest/'shared-state.json',state)
        dump(dest/'report.json',dict(status='READBACK_INPUT_NOT_ACCEPTANCE',artifact=str(path),sha256=trial['sha256']))
        hashes.update({str(p):_sha256(p) for p in (component/'shared-state.json',dest/'shared-state.json',dest/'report.json')})
        parent_states[rid]=str(dest)
    if set(parent_states)!=set(trial['parent_components']):raise ValueError('missing/duplicate parent state')
    hashes.update({str(p):_sha256(p) for p in (path,base,directory/'report.json',directory/'joint-state.json',Path(__file__))})
    packet=dict(status='REJECTED_PENDING_INDEPENDENT_READBACK',artifact=str(path),sha256=trial['sha256'],
        input=str(base),input_sha256=trial['input_sha256'],component=old['component'],
        parent_states=parent_states,north_component=parent_states['10'],source_hashes=hashes,
        connectors=[incoming.get(rid,dict(road=rid,unchanged_from_input=True)) for rid in sorted(graph.connections,key=int)],
        affected_roads=trial['affected_roads'],simultaneous_parent_connector_variables=True,
        no_previous_pass_verdict_reused=True,code_changes_before_independent_readback=changes,
        source_changed=False,production_accepted=False)
    unchanged(hashes);dump(output/'report.json',packet)
    print('ALL ORIGINAL JUNCTION MOVEMENTS',len(packet['connectors']),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('directory');p.add_argument('output');a=p.parse_args();run(a.directory,a.output)
