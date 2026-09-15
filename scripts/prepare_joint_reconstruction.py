"""Prepare immutable whole-road SHP reconstruction input; never export XODR."""
import argparse
import json
import sys
from collections import Counter
from pathlib import Path

from lxml import etree

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from mapforge.adapters.shp.profile_source import ProfileSource
from mapforge.ops.reconstruction_scope import prepare_scope, require_solver_input, whole_junction_scope
from mapforge.ops.source_domain import prepare_source_domain, require_domain_replay
from scripts.gen_all import _sha256


def prepare(path, output, roads=('10',), source_dir=None, profile='ibd-smarteditor-v1',
            source_junction=None, xodr_junction=None, *, whole_junction=False,
            chart_mode='line-only-v1', identity_mode='single-id-v1'):
    path, output = Path(path).resolve(), Path(output).resolve()
    if output.exists():
        raise FileExistsError('use a new preparation directory; do not overwrite evidence')
    if (source_junction is None) != (xodr_junction is None):
        raise ValueError('source and XODR junction identities must be explicitly bound together')
    if whole_junction and (source_junction is None or roads):
        raise ValueError('whole junction requires both identities and no manual road selection')
    source = ProfileSource(str(source_dir or ROOT / 'shp_0222-0326'), profile)
    # Freeze before loading geometry. Include every file of each mapped layer,
    # not a hardcoded partial list of SHP assets.
    files = {path, Path(source.p['_path']).resolve()}
    for spec in source.L.values():
        if isinstance(spec, dict) and spec.get('file'):
            base = source.dir / spec['file']
            files.update(p.resolve() for p in base.parent.glob(base.stem + '.*') if p.is_file())
    hashes = {str(p): _sha256(p) for p in sorted(files)}
    root = etree.parse(str(path)).getroot()
    complete = whole_junction_scope(root, xodr_junction) if whole_junction else None
    if complete:
        roads = complete['ordinary_roads']
    binding = ({'source_junction_id': str(source_junction), 'xodr_junction_id': str(xodr_junction)}
               if source_junction is not None else None)
    packet = prepare_scope(root, source, roads, binding, identity_mode)
    domain = (prepare_source_domain(root, source, packet, source_junction, xodr_junction, chart_mode)
              if source_junction is not None else None)
    # Admission is a separate state from export. Even fully admitted input is
    # still BLOCKED until a geometry solver and independent validation exist.
    try:
        require_solver_input(packet, root, source)
        admission_gate = 'ADMITTED_FOR_RESEARCH'
    except ValueError as exc:
        admission_gate = str(exc)
    source_unchanged = all(_sha256(Path(p)) == h for p, h in hashes.items())
    if not source_unchanged:
        raise RuntimeError('inputs changed during preparation; no usable packet written')
    run = {'status': 'BLOCKED', 'scope': 'input-preparation-only-no-xodr',
           'input': str(path), 'input_sha256': hashes[str(path)],
           'source_dir': str(source.dir.resolve()), 'profile': str(Path(source.p['_path']).resolve()),
           'input_files_sha256': hashes, 'source_and_input_unchanged': True,
           'source_admission': packet['source_admission'], 'admission_gate': admission_gate,
           'projection': {'profile_crs': source.p.get('crs'), 'absolute_verified': False,
                          'geometry': 'raw source coordinates; no coordinate transformation'},
           'counts': {'mutable_roads': len(packet['mutable_roads']),
                      'connectors': len(packet['connectors']),
                      'fixed_ports': len(packet['fixed_ports']),
                      'source_lanes': len(packet['observations']),
                      'physical_boundary_ids': len(packet['boundaries']),
                      'shared_boundary_ids': sum(len(v) > 1 for v in packet['boundary_owners'].values()),
                      'source_links': len(packet['source_links']),
                      'written_source_speed_mismatches': sum(u['written_source_speed_match'] is False
                                                             for u in packet['occurrences']),
                      'issues': dict(Counter(i['code'] for i in packet['issues']))},
           'geometry_solver_ran': False, 'export_allowed': False}
    if complete:
        if packet['fixed_ports'] or packet['connectors'] != complete['connectors']:
            raise ValueError('incomplete whole-junction dependency closure')
        run['whole_junction'] = complete
    if domain is not None and 'source_speed_intervals' in domain:
        run['source_speed_intervals'] = domain['source_speed_intervals']['counts']
        run['counts']['written_source_speed_mismatches'] = domain['source_speed_intervals']['counts'].get('MISMATCH', 0)
        run['counts']['source_speed_comparison_basis'] = 'full_original_segment_overlap_not_entire_chain_against_each_section'
    output.mkdir(parents=True)
    encoded = json.dumps(packet, ensure_ascii=False, indent=2, allow_nan=False)
    (output / 'reconstruction-input.json').write_text(encoded, encoding='utf-8')
    run['packet_file_sha256'] = _sha256(output / 'reconstruction-input.json')
    if domain is not None:
        (output/'source-domain.json').write_text(json.dumps(
            domain, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
        run['source_domain'] = {
            'file_sha256': _sha256(output/'source-domain.json'),
            'source_junction_id': str(source_junction), 'xodr_junction_id': str(xodr_junction),
            'movement_status': domain['comparison']['status'],
            'expected_movements': domain['comparison']['expected_count'],
            'written_movements': domain['comparison']['written_count'],
            'partition_status': domain['partition']['status'],
            'length_by_status_m': domain['partition']['length_by_status_m'],
            'shared_boundary_contact_binding': 'NOT_SOLVED'}
    (output / 'run.json').write_text(json.dumps(run, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps({k: run[k] for k in ('status', 'source_admission', 'counts', 'source_domain')
                      if k in run}, ensure_ascii=False, indent=2))
    return run


def verify_preparation(output):
    """Rebuild from unchanged raw inputs, not just trust serialized PASS strings."""
    output = Path(output)
    run = json.loads((output/'run.json').read_text(encoding='utf-8'))
    path = output/'reconstruction-input.json'
    if _sha256(path) != run['packet_file_sha256']:
        raise ValueError('prepared file hash mismatch')
    for filename, sha in run['input_files_sha256'].items():
        if _sha256(Path(filename)) != sha:
            raise ValueError('source or baseline changed: ' + filename)
    packet = json.loads(path.read_text(encoding='utf-8'))
    root = etree.parse(run['input']).getroot()
    if 'whole_junction' in run:
        whole = whole_junction_scope(root, run['whole_junction']['xodr_junction_id'])
        if (whole != run['whole_junction'] or whole['ordinary_roads'] != packet['mutable_roads']
                or whole['connectors'] != packet['connectors'] or packet['fixed_ports']):
            raise ValueError('incomplete whole-junction preparation')
    source = ProfileSource(run['source_dir'], run['profile'])
    rebuilt = prepare_scope(root, source, packet['mutable_roads'], packet.get('source_domain_binding'),
                            packet.get('identity_mode', 'single-id-v1'))
    if rebuilt['content_sha256'] != packet['content_sha256']:
        raise ValueError('prepared model differs from fresh original inputs')
    require_solver_input(packet, root, source)
    domain_result = None
    binding = packet.get('source_domain_binding')
    if binding is not None or 'source_domain' in run or (output/'source-domain.json').exists():
        if binding is None or 'source_domain' not in run:
            raise ValueError('source domain required by packet version or artifacts; missing binding/manifest')
        path = output/'source-domain.json'
        if _sha256(path) != run['source_domain']['file_sha256']:
            raise ValueError('source domain file hash mismatch')
        domain = json.loads(path.read_text(encoding='utf-8'))
        if (domain['inventory']['source_junction_id'] != run['source_domain']['source_junction_id']
                or domain['comparison']['xodr_junction_id'] != run['source_domain']['xodr_junction_id']):
            raise ValueError('source domain junction binding mismatch')
        if binding != {k: run['source_domain'][k] for k in ('source_junction_id', 'xodr_junction_id')}:
            raise ValueError('source domain binding differs from main packet')
        require_domain_replay(domain, root, source, packet)
        domain_result = {'fresh_read_match': True, 'movement_status': domain['comparison']['status'],
                         'partition_status': domain['partition']['status']}
    # Check again after fresh reads; this is revision integrity, not a signed
    # provenance authority or an OS-level snapshot of a concurrently edited FS.
    if any(_sha256(Path(f)) != h for f, h in run['input_files_sha256'].items()):
        raise ValueError('source changed during preparation verification')
    return {'source_admission': rebuilt['source_admission'], 'fresh_read_match': True,
            'status': 'BLOCKED', 'geometry_solver_ran': False, 'export_allowed': False,
            'source_domain': domain_result}


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('xodr', type=Path)
    p.add_argument('output', type=Path)
    p.add_argument('--road', action='append', dest='roads')
    p.add_argument('--whole-junction', action='store_true', help='collect all explicit parents and turns; no per-road subset')
    p.add_argument('--chart-mode', choices=['line-only-v1', 'exact-line-arc-v1'], default='line-only-v1')
    p.add_argument('--identity-mode', choices=['single-id-v1', 'source-chain-v1'], default='single-id-v1')
    p.add_argument('--source-dir', type=Path)
    p.add_argument('--profile', default='ibd-smarteditor-v1')
    p.add_argument('--source-junction', help='exact original SHP junction PID; no nearest-geometry guessing')
    p.add_argument('--xodr-junction', help='explicit matching XODR junction ID')
    p.add_argument('--verify', action='store_true', help='verify existing output by rereading original inputs; do not write')
    a = p.parse_args()
    if a.verify:
        result = verify_preparation(a.output)
        print(json.dumps(result, indent=2))
    else:
        prepare(a.xodr, a.output, a.roads if a.whole_junction else a.roads or ['10'], a.source_dir, a.profile,
                a.source_junction, a.xodr_junction, whole_junction=a.whole_junction,
                chart_mode=a.chart_mode, identity_mode=a.identity_mode)
    raise SystemExit(2)  # This command deliberately cannot claim a usable map.
