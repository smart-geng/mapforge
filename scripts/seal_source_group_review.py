"""Seal one multi-parent research trial with complete, same-XML evidence.

This seal has no production-acceptance branch. It refuses incomplete parent,
turn, source, consumer or rendering evidence rather than reusing old verdicts.
"""
import argparse
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from scripts.gen_all import _sha256
from scripts.build_source_geometry_candidate import dump
from scripts.fit_source_boundary_block import unchanged
from scripts.seal_parent_rebuild_diagnostic import require_same_artifact,require_inventory,interface_maxima
from scripts.seal_geometry_review import verify_full_source_budget
from mapforge.ops.port_dependencies import PortDependencies


def require_group_inventory(trial,packet,geometry,shape,parents,root):
    graph=PortDependencies(root);graph.validate_junction_table()
    parent_ids=set(trial['parent_components'])
    if len(parents)!=len(parent_ids) or {p['road'] for p in parents}!=parent_ids:
        raise ValueError('every source parent needs its own actual-file audit')
    affected={rid for rid,ports in graph.connections.items() if any(p.road in parent_ids for p in ports)}
    if (set(trial['affected_roads'])!=affected or len(trial['connectors'])!=len(affected)
            or {r['road'] for r in trial['connectors']}!=affected):
        raise ValueError('joint solve omitted or duplicated an incident turn')
    require_inventory(packet,geometry,shape)
    if {r['road'] for r in packet['connectors']}!=set(graph.connections):
        raise ValueError('independent audit must cover all explicit movements')


def run(directory,packet_name='review-packet'):
    directory=Path(directory).resolve();packet=(directory/packet_name).resolve()
    if packet.parent!=directory:raise ValueError('review packet must be an explicit child directory')
    paths=[directory/'report.json',packet/'report.json',packet/'independent-review/report.json',
        packet/'shape-review/report.json',packet/'shape-review/full-source-turn-budget.json',
        directory/'esmini-review/esmini-captures.json']
    trial,review,geometry,shape,budget,render=[json.loads(p.read_text(encoding='utf8')) for p in paths]
    parent_paths=[packet/('north-independent' if rid=='10' else 'parent-'+rid+'-independent')/'report.json'
        for rid in sorted(trial['parent_components'],key=int)]
    parents=[json.loads(p.read_text(encoding='utf8')) for p in parent_paths]
    artifact=Path(trial['artifact'])
    sha=require_same_artifact(artifact,[trial,review,geometry,shape,budget,render]+parents)
    if not trial['simultaneous_parent_connector_variables'] or not review['simultaneous_parent_connector_variables']:
        raise ValueError('actual shared parent and connector solve required')
    require_group_inventory(trial,review,geometry,shape,parents,ET.parse(artifact).getroot())
    for report,key in [(review,'source_hashes'),(geometry,'source_hashes'),(shape,'hashes')]+[(p,'source_hashes') for p in parents]:
        unchanged(report[key])
    failed=verify_full_source_budget(shape,budget,paths[3])
    retained=Path(trial['input'])
    if _sha256(retained)!=trial['input_sha256']:raise ValueError('retained research XML changed')
    shots=[]
    for row in render['shots']:
        p=(paths[-1].parent/row['file']).resolve()
        if p.parent!=paths[-1].parent or not p.is_file():raise ValueError('missing or escaped capture')
        shots.append(p)
    if len(shots)!=3 or len(set(shots))!=3:raise ValueError('three distinct actual simulator views required')
    figures=[packet/'independent-review/all-turns.png']+[p.parent/'actual-source.png' for p in parent_paths]
    if not all(p.is_file() for p in figures):raise ValueError('actual source overlay missing')
    result=dict(status='REJECTED_NOT_DELIVERY',artifact=str(artifact),sha256=sha,
        previous_research_artifact_retained=str(retained),previous_research_sha256=trial['input_sha256'],
        candidate_selected=False,previous_research_is_delivery=False,
        simultaneous_parent_connector_variables=True,variable_count=trial['variable_count'],
        independent_parent_variables=trial['independent_parent_variables'],affected_roads=trial['affected_roads'],
        independently_reviewed_turn_count=len(review['connectors']),construction=trial['final'],
        independent_geometry_failed_roads=geometry['geometry_failed_roads'],full_source_turn_failed_roads=failed,
        internal_edge_failed_roads=sorted({r['road'] for r in geometry['all_map_internal_edges']['failures']},key=int),
        internal_edge_maxima=geometry['all_map_internal_edges']['maxima'],
        consumer_interfaces_status=geometry['interfaces']['status'],
        consumer_interfaces_count=len(geometry['interfaces']['rows']),
        consumer_interfaces_failed_count=sum(r['status']!='PASS' for r in geometry['interfaces']['rows']),
        consumer_interfaces_maxima=interface_maxima(geometry['interfaces']),
        parents={p['road']:{k:p[k] for k in ('written_source','internal_edges','source_speed_entries_match',
            'source_speed_entry_count','source_speed_dynamics')} for p in parents},
        east_full_source_aggregate=geometry['east_full_source_aggregate'],xsd_pass=geometry['xsd_pass'],
        code_changes_before_independent_readback=review['code_changes_before_independent_readback'],
        source_priority_stage_ran=trial['source_priority_stage_ran'],source_changed=False,source_roles_expanded=False,
        source_speed_dynamics_accepted=False,whole_surface_accepted=False,movement_paths_validated=False,
        MAP_retested=False,production_accepted=False,rendered_images_are_automatic_acceptance=False,
        reason='Same-state research trial only; not approved as a complete source-faithful smooth map.',
        evidence_hashes={str(p):_sha256(p) for p in paths+parent_paths+shots+figures+[artifact,retained,Path(__file__)]})
    target=directory/'FINAL-REVIEW-STATUS.json'
    if target.exists():raise FileExistsError('preserve previous review seal')
    unchanged(result['evidence_hashes']);dump(target,result)
    print(result['status'],result['independent_geometry_failed_roads'],failed,flush=True)
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('directory');parser.add_argument('--packet',default='review-packet')
    args=parser.parse_args();run(args.directory,args.packet)
    raise SystemExit(2)
