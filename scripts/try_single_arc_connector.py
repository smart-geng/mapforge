"""Same parents/other turns; scoped alternate chart for an unresolved turn."""
import argparse
import json
import sys
from pathlib import Path
import xml.etree.ElementTree as ET
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from scripts.gen_all import shp_source,_sha256
from scripts.build_source_geometry_candidate import dump
from spikes.measured_connector_caps import frames,raw_curves,composite_sources
from spikes.one_arc_source_ribbon import fit
from spikes.trial_xml import replace_road,write_trial
from mapforge.validate.shp_boundary_fidelity import _origin,_project


def run(source,output,rid):
    source=Path(source).resolve();output=Path(output).resolve();output.mkdir(exist_ok=False)
    tree=ET.parse(source);root=tree.getroot();road=root.find(f"road[@id='{rid}']")
    before={r.get('id'):ET.tostring(r) for r in root.findall('road') if r.get('id')!=rid}
    src=shp_source();lat,lon=_origin(root);raw,identity=composite_sources(root,road,src,lambda p:_project(p,lat,lon))
    new,report=fit(road,*frames(root,road),raw)
    report.update(input=str(source),input_sha256=_sha256(source),source_support=identity)
    if new is not None:
        from scripts.review_measured_ribbon import fidelity,target_curves,minimum_width,sampled_strip_validity,stats,distances
        from scripts.review_source_connectors import source_ok,written_forward_minimum
        from scripts.internal_edge_jets import audit
        actual=target_curves(new,.05);errors=fidelity(raw,actual)
        sid=road.find(".//userData[@code='mapforge.source_lane']").get('value')
        via=raw_curves(src,sid,lambda p:_project(p,lat,lon))
        via_errors={k:stats(distances(v,actual[k])) for k,v in via.items()}
        report.update(written_source=errors,raw_via=via_errors,minimum_width_m=minimum_width(new),
            minimum_forward_factor=written_forward_minimum(new),strip=sampled_strip_validity(actual))
        replace_road(root,road,new);path=output/'node4-review.xodr';write_trial(tree,path)
        assert all(ET.tostring(root.find(f"road[@id='{k}']"))==v for k,v in before.items())
        from scripts.esmini_lane_interfaces import check
        interfaces=check(path,edges=True)
        port_rows=[r for r in interfaces['rows'] if r['road']==rid]
        report.update(artifact=str(path),sha256=_sha256(path),all_other_roads_unchanged=True,
            internal=audit(root),interfaces=interfaces,geometry_source_pass=source_ok([v for f in errors.values() for v in f.values()]+list(via_errors.values())),
            ports_pass=bool(port_rows) and all(r['status']=='PASS' for r in port_rows))
        report['status']=('COMPONENT_REVIEW_NOT_DELIVERY' if report['geometry_source_pass'] and report['ports_pass']
                          else 'REJECTED_REVIEW_NOT_DELIVERY')
    dump(output/'report.json',report);print(json.dumps({k:v for k,v in report.items() if k not in ('interfaces','internal','coefficients','knots')}),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('source');p.add_argument('output');p.add_argument('--road',default='122')
    a=p.parse_args();run(a.source,a.output,a.road)
