"""Bind a single-turn trial to the EXACT unchanged parents and all other roads.

Re-run the complete north/east readback, not just the solver's chosen turn.
Only geometry records may differ. This produces a review copy, never delivery.
"""
import argparse
import copy
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from scripts.gen_all import _sha256
from scripts.fit_source_boundary_block import unchanged
from scripts.build_source_geometry_candidate import dump
from spikes.trial_xml import write_trial


def nongeometry(road):
    road = copy.deepcopy(road)
    road.attrib.pop('length', None)
    road.remove(road.find('planView'))
    for group in road.findall('lanes'):
        for element in group.findall('laneOffset'):
            group.remove(element)
    for lane in road.findall('.//lane'):
        for element in lane.findall('width') + lane.findall('border'):
            lane.remove(element)
        # Candidate provenance may describe the new geometry; source identity,
        # speed and traffic connections are intentionally not stripped.
        for element in lane.findall("userData[@code='mapforge.provenance/v1']"):
            lane.remove(element)
    return ET.tostring(road)


def require_fixed_parent(before, after, changed):
    old = {r.get('id'): r for r in before.findall('road')}
    new = {r.get('id'): r for r in after.findall('road')}
    if old.keys() != new.keys() or not changed or not changed <= old.keys():
        raise ValueError('road identity inventory changed')
    for rid in old:
        if rid in changed:
            if old[rid].get('junction') == '-1':
                raise ValueError('parent replacement needs ALL dependency rebuilds')
            if nongeometry(old[rid]) != nongeometry(new[rid]):
                raise ValueError('non-geometric source identity/speed/link changed')
        elif ET.tostring(old[rid]) != ET.tostring(new[rid]):
            raise ValueError('fixed parent or unaffected turn changed')
    for tag in ('header', 'junction'):
        if [ET.tostring(e) for e in before.findall(tag)] != [ET.tostring(e) for e in after.findall(tag)]:
            raise ValueError('header/junction changed')


def run(baseline, candidate, output):
    baseline, candidate, output = map(lambda p: Path(p).resolve(), (baseline, candidate, output))
    parent = json.loads((baseline/'report.json').read_text(encoding='utf-8'))
    trial = json.loads((candidate/'report.json').read_text(encoding='utf-8'))
    if trial['input_sha256'] != parent['sha256']:
        raise ValueError('candidate did not use this exact parent state')
    for record in (parent, trial):
        if _sha256(Path(record['artifact'])) != record['sha256']:
            raise ValueError('input XML changed')
    from scripts.research_code_revision import bind_current_code
    hashes, changes = bind_current_code(parent['source_hashes'])
    unchanged(trial['source_hashes'])
    unchanged(trial['code_sha256'])
    old = ET.parse(parent['artifact']); new = ET.parse(trial['artifact'])
    replacements = {r['road']: r for r in trial['connectors']}
    if len(replacements) != len(trial['connectors']):
        raise ValueError('duplicate replacement')
    require_fixed_parent(old.getroot(), new.getroot(), set(replacements))
    if not replacements.keys() <= {r['road'] for r in parent['connectors']}:
        raise ValueError('replacement outside declared review set')
    output.mkdir(parents=True, exist_ok=False)
    path = output/'node4-review.xodr'
    write_trial(new, path)
    hashes.update(trial['code_sha256'])
    for p in (baseline/'report.json', candidate/'report.json', Path(trial['artifact']), Path(__file__)):
        hashes[str(p)] = _sha256(p)
    report = dict(parent)
    report.update(status='REVIEW_NOT_DELIVERY', artifact=str(path), sha256=_sha256(path),
        source_hashes=hashes, input=parent['artifact'], input_sha256=parent['sha256'],
        connectors=[replacements.get(r['road'], r) for r in parent['connectors']],
        replacement_roads=sorted(replacements), parents_and_other_roads_byte_identical=True,
        nongeometry_unchanged=True, production_accepted=False, input_code_revision_changes=changes)
    dump(output/'report.json', report)
    from scripts.review_asymmetric_dependents import run as review
    result = review(output)
    unchanged(hashes)
    return result


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('baseline'); p.add_argument('candidate'); p.add_argument('output')
    a = p.parse_args(); run(a.baseline, a.candidate, a.output)
