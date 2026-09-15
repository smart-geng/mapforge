"""Small, replayable edits on existing long cubic width intervals.

One handle owns ONE shared physical boundary. Its two lane widths receive
opposite additions. Reference line, other boundaries, ports and original data
are immutable. This first slice intentionally does not move junction mouths.
"""
from __future__ import annotations

import copy
import hashlib
import json
import math
import os
import threading
import uuid
import subprocess
import sys
from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np
from scipy.interpolate import BSpline
from pyproj import CRS, Transformer

from scripts.internal_edge_jets import states, audit, active


class Conflict(ValueError):
    pass


def digest(data):
    return hashlib.sha256(data).hexdigest()


def json_bytes(value):
    return json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False).encode('utf8')


def atomic(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + '.' + uuid.uuid4().hex + '.tmp')
    with tmp.open('xb') as stream:
        stream.write(data)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(tmp, path)


def parse(data):
    # First-slice admission is deliberately narrow; unsupported input fails.
    if b'<!DOCTYPE' in data.upper() or b'<!ENTITY' in data.upper():
        raise ValueError('DTD/entities are not accepted')
    root = ET.fromstring(data, parser=ET.XMLParser(target=ET.TreeBuilder(insert_comments=True)))
    if root.tag != 'OpenDRIVE':
        raise ValueError('Only unnamespaced OpenDRIVE is supported in this slice')
    for g in root.findall('road/planView/geometry'):
        if len(g) != 1 or g[0].tag not in ('line', 'arc', 'spiral'):
            raise ValueError('Unsupported reference geometry; will not silently drop it')
    return root


def lanes(sec):
    return {int(l.get('id')): l for l in sec.findall('left/lane') + sec.findall('right/lane')}


def intervals(road):
    secs = road.findall('lanes/laneSection')
    return [(s, float(s.get('s')), float(secs[i+1].get('s')) if i+1 < len(secs)
             else float(road.get('length'))) for i, s in enumerate(secs)]


def coefficients(w):
    return np.array([float(w.get(k, '0')) for k in 'abcd'])


def extrema(c, length):
    roots = np.polynomial.polynomial.polyroots([c[1], 2*c[2], 3*c[3]])
    xs = [0., length] + [float(r.real) for r in roots if abs(r.imag) < 1e-9 and 0 < r.real < length]
    return min(float(np.polynomial.polynomial.polyval(s, c)) for s in xs)


def shifted(elements, attr, station):
    if not elements:
        return np.zeros(4)
    item = active(elements, station, False, lambda e: float(e.get(attr)))
    poly = np.polynomial.Polynomial(coefficients(item))
    out = poly(np.polynomial.Polynomial([station-float(item.get(attr)),1])).coef
    return np.pad(out,(0,4-len(out)))


def regular_frame(road):
    """Exact cubic minimum of 1-k*t for every stacked edge of a constant-k road."""
    g = road.find('planView/geometry'); k = float(g[0].get('curvature','0'))
    if not k:
        return 1.
    minimum = 1.
    offsets = road.findall('lanes/laneOffset')
    for sec,lo,hi in intervals(road):
        cuts = sorted({lo,hi} | {float(o.get('s')) for o in offsets if lo<float(o.get('s'))<hi}
                      | {lo+float(w.get('sOffset')) for l in lanes(sec).values() for w in l.findall('width')
                         if lo<lo+float(w.get('sOffset'))<hi})
        for start,end in zip(cuts,cuts[1:]):
            for side,sign in [('left',1),('right',-1)]:
                t = shifted(offsets,'s',start)
                for lane in sorted(sec.findall(side+'/lane'),key=lambda l:abs(int(l.get('id')))):
                    if lane.findall('border'):
                        raise ValueError('Cannot certify a border-based offset frame in this slice')
                    for boundary in (t, t+sign*shifted(lane.findall('width'),'sOffset',start-lo)):
                        a = -k*boundary; a[0] += 1.
                        minimum = min(minimum,extrema(a,end-start))
                    t = t+sign*shifted(lane.findall('width'),'sOffset',start-lo)
    if minimum <= 1e-6:
        raise ValueError('拒绝：横向偏移坐标发生折叠或奇异')
    return minimum


def road_scene(road):
    edges, centers = [], []
    for sec, lo, hi in intervals(road):
        if hi <= lo:
            raise ValueError('Invalid laneSection interval')
        # Sampling is ONLY for display; never serialized as geometry.
        ss = np.linspace(lo, hi, max(2, int(math.ceil((hi-lo)/.8)) + 1))
        for lid, lane in lanes(sec).items():
            if lane.get('type') != 'driving':
                continue
            pair = np.array([states(road, lid, float(s), i == len(ss)-1) for i, s in enumerate(ss)])
            edges.extend([p.tolist() for p in (pair[:, 0, :2], pair[:, 1, :2])])
            centers.append(((pair[:, 0, :2] + pair[:, 1, :2])/2).tolist())
    return {'id': road.get('id'), 'junction': road.get('junction') != '-1',
            'auxiliary': road.get('name') == 'junction_paving', 'edges': edges, 'centers': centers}


def complexity(root):
    result = {}
    for tag, xpath in [('geometry','road/planView/geometry'), ('width','road/lanes/laneSection/*/lane/width'),
                       ('laneOffset','road/lanes/laneOffset'), ('laneSection','road/lanes/laneSection')]:
        result[tag] = len(root.findall(xpath))
    lengths = [float(g.get('length')) for g in root.findall('road/planView/geometry')]
    result['min_geometry_m'] = min(lengths)
    result['geometry_under_2m'] = sum(v < 2 for v in lengths)
    return result


class RepairModel:
    def __init__(self, data):
        self.data = data
        self.root = parse(data)
        self.base_hash = digest(data)
        self.handles = {}
        self.baseline_scene = [road_scene(r) for r in self.root.findall('road')]
        self.baseline_complexity = complexity(self.root)
        self._build_handles()

    def _build_handles(self):
        for road in self.root.findall('road'):
            # Fixed constant-curvature parent references: adding a C2 lateral
            # field preserves existing G2 joints; spiral/compound ports locked.
            gs = road.findall('planView/geometry')
            if road.get('junction') != '-1' or len(gs) != 1 or gs[0][0].tag not in ('line','arc'):
                continue
            secs = intervals(road)
            ids = sorted(set.intersection(*(set(lanes(s)) for s, _, _ in secs)))
            for lid in ids:
                neighbor = lid + (1 if lid > 0 else -1)
                if neighbor not in ids:
                    continue  # shared interior boundaries only, not an outer-edge guess
                affected = (lid, neighbor)
                valid = True
                for i, (sec, _, _) in enumerate(secs):
                    for ln in affected:
                        l = lanes(sec)[ln]
                        if l.get('type') != 'driving' or l.findall('border') or not l.findall('width'):
                            valid = False
                        if i < len(secs)-1:
                            successor = l.find('link/successor')
                            predecessor = lanes(secs[i+1][0])[ln].find('link/predecessor')
                            if successor is None or predecessor is None or int(successor.get('id')) != ln or int(predecessor.get('id')) != ln:
                                valid = False
                if not valid:
                    continue
                # Use EXISTING common width/section breakpoints: zero new XML records.
                sets = []
                for ln in affected:
                    sets.append({round(lo + float(w.get('sOffset')), 8)
                                 for sec, lo, _ in secs for w in lanes(sec)[ln].findall('width')})
                cuts = sorted(s for s in sets[0] & sets[1] if 5 < s < float(road.get('length'))-5)
                if len(cuts) < 5:
                    continue
                chosen = [cuts[int(round(i*(len(cuts)-1)/4))] for i in range(5)]
                if min(np.diff(chosen)) < 6:
                    continue
                # Use exact XML station values (rounded keys above only for matching).
                exact = [lo+float(w.get('sOffset')) for sec, lo, _ in secs for w in lanes(sec)[lid].findall('width')]
                chosen = [min(exact, key=lambda s: abs(s-x)) for x in chosen]
                b = BSpline.basis_element(chosen, extrapolate=False)
                at = (chosen[1]+chosen[3])/2
                scale = float(b(at))
                edge = states(road, lid, at, False)[1]
                ref = gs[0]
                heading = float(ref.get('hdg')) + (float(ref[0].get('curvature','0')) * at)
                key = f'{road.get("id")}:{lid}'
                self.handles[key] = {'id': key, 'road': road.get('id'), 'lane': lid,
                    'neighbor': neighbor, 'knots': chosen, 's': at, 'scale': scale,
                    'xy': list(edge[:2]), 'normal': [-math.sin(heading), math.cos(heading)],
                    'span_m': chosen[-1]-chosen[0], 'min_basis_span_m': min(np.diff(chosen)),
                    'kind': 'shared_boundary', 'locked': 'reference / outer edges / all ports / topology'}

    def compile(self, values):
        if set(values)-set(self.handles):
            raise ValueError('Unknown or locked handle')
        root = copy.deepcopy(self.root)
        touched = set()
        for key, delta in values.items():
            if not isinstance(delta, (float, int)) or not math.isfinite(delta) or abs(delta) > 2:
                raise ValueError('单次相对基线位移须为有限数且在 ±2m 内；不会自动扩大权限')
            if delta == 0:
                continue
            h = self.handles[key]
            road = next(r for r in root.findall('road') if r.get('id') == h['road'])
            b = BSpline.basis_element(h['knots'], extrapolate=False)
            for sec, lo, hi in intervals(road):
                for lid, sign in [(h['lane'], 1 if h['lane'] > 0 else -1),
                                  (h['neighbor'], -1 if h['lane'] > 0 else 1)]:
                    widths = lanes(sec)[lid].findall('width')
                    for i, w in enumerate(widths):
                        s = lo+float(w.get('sOffset'))
                        end = lo+float(widths[i+1].get('sOffset')) if i+1 < len(widths) else hi
                        mid = (s+end)/2
                        if not h['knots'][0] < mid < h['knots'][-1]:
                            continue
                        if any(s+1e-7 < k < end-1e-7 for k in h['knots']):
                            raise ValueError('Handle would create a new width interval')
                        # Derivatives taken inside this polynomial, then shifted
                        # back to its exact original sOffset (one-sided at knots).
                        c_mid = [float(b(mid, nu=j))/math.factorial(j) for j in range(4)]
                        poly = np.polynomial.Polynomial(c_mid)(np.polynomial.Polynomial([s-mid, 1]))
                        c = coefficients(w) + sign*delta*np.pad(poly.coef,(0,4-len(poly.coef)))/h['scale']
                        for attr, v in zip('abcd', c):
                            w.set(attr, format(float(v), '.17g'))
                        touched.add((h['road'], lo, lid, s))
        minima = []
        for road in root.findall('road'):
            for sec, lo, hi in intervals(road):
                for lid, lane in lanes(sec).items():
                    widths = lane.findall('width')
                    for i, w in enumerate(widths):
                        s = lo+float(w.get('sOffset'))
                        if (road.get('id'),lo,lid,s) not in touched:
                            continue
                        end = lo+float(widths[i+1].get('sOffset')) if i+1 < len(widths) else hi
                        m = extrema(coefficients(w), end-s)
                        if m < -1e-8:
                            raise ValueError(f'拒绝：road {road.get("id")} lane {lid} 在区间 {s:.2f}m 出现负宽 {m:.4f}m')
                        minima.append(m)
        # All non-width semantics must remain EXACT, including extensions/speeds.
        def strip_widths(r):
            r = copy.deepcopy(r)
            for w in r.findall('road/lanes/laneSection/*/lane/width'):
                for attr in 'abcd':
                    w.attrib.pop(attr, None)
            return ET.tostring(r)
        if strip_widths(root) != strip_widths(self.root) or complexity(root) != self.baseline_complexity:
            raise ValueError('Non-width semantics/complexity changed')
        # Exact cubic extrema, not sampled positivity or a dynamics certificate.
        changed = sorted({t[0] for t in touched})
        frame_minima = []
        for road in root.findall('road'):
            if road.get('id') not in changed:
                continue
            frame_minima.append(regular_frame(road))
        data = self.data if not touched else ET.tostring(root, encoding='utf-8', xml_declaration=True)
        return data, {'guard': 'LOCAL_WIDTH_EDIT_ONLY', 'changed_roads': changed,
                      'changed_width_records': len(touched), 'min_changed_width_m': min(minima) if minima else None,
                      'min_offset_frame_A': min(frame_minima) if frame_minima else None,
                      'new_geometry_records': 0, 'new_width_records': 0, 'ports': 'LOCKED',
                      'delivery': 'BLOCKED', 'source_fidelity': 'NOT_VALIDATED',
                      'dynamics': 'NOT_VALIDATED', 'absolute_crs': 'NOT_VALIDATED'}


class Project:
    def __init__(self, source, packet, run, directory):
        self.directory = Path(directory).resolve()
        self.directory.mkdir(parents=True, exist_ok=True)
        self.source = Path(source).resolve(); self.packet = Path(packet).resolve(); self.run = Path(run).resolve()
        self.run_data = json.loads(self.run.read_text(encoding='utf8'))
        self.binding = dict(self.run_data['input_files_sha256'])
        self.binding.update({str(self.packet): digest(self.packet.read_bytes()), str(self.run): digest(self.run.read_bytes())})
        repo = Path(__file__).resolve().parents[2]
        # Freeze the nine existing decisions too; this UI creates no new source-role decision.
        for relative in ('profiles/repair/node4-zero-width-source-roles-v1.yaml',
                         'profiles/repair/node4-east-zero-width-source-roles-v2.yaml',
                         'profiles/repair/node4-west-south-zero-width-source-roles-v1.yaml',
                         'mapforge/repair_web/model.py', 'scripts/internal_edge_jets.py',
                         'mapforge/validate/g11.py', 'mapforge/validate/smoothness.py'):
            path = repo/relative
            self.binding[str(path)] = digest(path.read_bytes())
        if self.run_data['input_sha256'] != digest(self.source.read_bytes()):
            raise ValueError('Source XODR differs from registered source packet')
        self.verify_sources()
        self.model = RepairModel(self.source.read_bytes())
        self.lock = threading.RLock()
        self.timeline = [{}]; self.cursor = 0; self.revision = uuid.uuid4().hex; self.saved_revision = None
        self.preview_cache = None
        self.saved_path = self.directory/'project.json'
        self.overlay = self._source_overlay()
        if self.saved_path.exists():
            self.reopen()

    def verify_sources(self):
        for path, sha in self.binding.items():
            if digest(Path(path).read_bytes()) != sha:
                raise Conflict('Source drift: '+path)

    def _source_overlay(self):
        root = self.model.root
        ref = root.findtext('header/geoReference')
        if not ref:
            raise ValueError('No coordinate transform, cannot overlay')
        off = root.find('header/offset')
        if off is not None and any(abs(float(off.get(k,'0'))) > 1e-12 for k in ('x','y','z','hdg')):
            raise ValueError('Header offset is not supported in this slice')
        proj = Transformer.from_crs(CRS.from_epsg(4326), CRS.from_proj4(ref), always_xy=True)
        packet = json.loads(self.packet.read_text(encoding='utf8'))
        def points(part):
            return [list(proj.transform(float(p[0]),float(p[1]))) for p in part]
        boundaries = [{'id': key, 'points':points(p)} for key,b in packet['boundaries'].items()
                      for rec in b['records'] for p in rec['parts'] if len(p)>1]
        paths = [{'id': key, 'points':points(o['lane_path'])} for key,o in packet['observations'].items() if len(o['lane_path'])>1]
        return {'boundaries':boundaries, 'paths':paths, 'frame':ref,
                'crs_notice':'沿用已登记 EQC 研究坐标作叠图；不是 WGS84/绝对位置验收',
                'transform_definition':proj.definition, 'alignment':'NONE; no best-fit registration'}

    def values(self):
        return self.timeline[self.cursor]

    def check_revision(self, revision):
        if revision != self.revision:
            raise Conflict('页面版本已过期，请重新载入；旧请求不会覆盖新版本')

    def snapshot(self):
        data, guard = self.model.compile(self.values())
        root = parse(data)
        return {'revision':self.revision, 'saved_revision':self.saved_revision, 'values':self.values(),
                'can_undo':self.cursor>0, 'can_redo':self.cursor+1<len(self.timeline),
                'sha256':digest(data), 'guard':guard, 'handles':list(self.model.handles.values()),
                'roads':[road_scene(r) for r in root.findall('road')], 'complexity':complexity(root)}

    def preview(self, revision, handle, value):
        self.check_revision(revision)
        vals = dict(self.values()); vals[handle] = value
        data, guard = self.model.compile(vals)
        token = uuid.uuid4().hex
        self.preview_cache = (token, revision, vals, data, guard)
        h = self.model.handles[handle]
        root = parse(data)
        return {'preview_id':token, 'revision':revision, 'guard':guard, 'value':value,
                'road':road_scene(next(r for r in root.findall('road') if r.get('id')==h['road'])),
                'sha256':digest(data)}

    def apply(self, revision, token):
        self.check_revision(revision)
        if not self.preview_cache or self.preview_cache[:2] != (token,revision):
            raise Conflict('预览已过期，请重新预览')
        self.verify_sources()
        vals = self.preview_cache[2]
        self.timeline = self.timeline[:self.cursor+1]+[vals]
        self.cursor += 1; self.revision = uuid.uuid4().hex; self.preview_cache = None
        return self.snapshot()

    def navigate(self, revision, delta):
        self.check_revision(revision)
        new = self.cursor+delta
        if not 0 <= new < len(self.timeline):
            raise Conflict('没有可撤销/重做的版本')
        self.cursor = new; self.revision = uuid.uuid4().hex; self.preview_cache = None
        return self.snapshot()

    def save(self, revision):
        self.check_revision(revision); self.verify_sources()
        data, guard = self.model.compile(self.values())
        packet = {'schema':'mapforge/local-repair/v1', 'revision':self.revision,
                  'binding':self.binding, 'timeline':self.timeline, 'cursor':self.cursor,
                  'xodr_sha256':digest(data), 'guard':guard, 'source_sha256':self.model.base_hash}
        atomic(self.saved_path,json_bytes(packet))
        self.saved_revision = self.revision
        return {'saved_revision':self.revision, 'path':str(self.saved_path), 'sha256':digest(data)}

    def reopen(self):
        self.verify_sources()
        packet = json.loads(self.saved_path.read_text(encoding='utf8'))
        if packet['binding'] != self.binding or packet['source_sha256'] != self.model.base_hash:
            raise Conflict('Saved project bindings differ from registered originals')
        # Validate the entire history so undo cannot inject unsupported state.
        for vals in packet['timeline']:
            self.model.compile(vals)
        vals = packet['timeline'][packet['cursor']]
        if digest(self.model.compile(vals)[0]) != packet['xodr_sha256']:
            raise Conflict('Saved project cannot replay byte-identically')
        self.timeline = packet['timeline']; self.cursor = packet['cursor']
        self.revision = uuid.uuid4().hex; self.saved_revision = self.revision; self.preview_cache = None
        return self.snapshot()

    def export(self, revision):
        self.check_revision(revision); self.verify_sources()
        data, guard = self.model.compile(self.values())
        export_id = uuid.uuid4().hex
        dest = self.directory/'exports'/export_id
        dest.mkdir(parents=True,exist_ok=False)
        path = dest/'candidate.xodr'
        atomic(path,data)
        # Audit the written bytes, never the preview geometry. Inherited failures
        # remain visible; this endpoint NEVER promotes a research map to delivery.
        root = parse(path.read_bytes())
        written = audit(root)
        baseline = audit(self.model.root)
        atomic(dest/'internal-edges.json',json_bytes(written))
        report = {'status':'BLOCKED_RESEARCH_CANDIDATE', 'revision':revision,
                  'xodr_sha256':digest(path.read_bytes()), 'source_sha256':self.model.base_hash,
                  'guard':guard, 'complexity_before':self.model.baseline_complexity,
                  'complexity_after':complexity(root), 'written_internal_edges':written['status'],
                  'written_internal_failures':len(written['failures']),
                  'baseline_internal_failures':len(baseline['failures']),
                  'source_fidelity':'NOT_VALIDATED', 'surface':'NOT_VALIDATED',
                  'dynamics':'NOT_VALIDATED', 'esmini':'NOT_RUN', 'absolute_crs':'NOT_VALIDATED',
                  'ids_topology_speed_extensions':'UNCHANGED', 'input_hashes':self.binding,
                  'edit_intents':self.values(), 'handles':list(self.model.handles.values()),
                  'coordinate_frame':self.overlay['frame'], 'source_projection':self.overlay['transform_definition'],
                  'undo_replay':'from immutable input; no resampling during compilation'}
        atomic(dest/'report.json',json_bytes(report))
        verifier = Path(__file__).resolve().parents[2]/'scripts/validate_repair_export.py'
        try:
            proc = subprocess.run([sys.executable,str(verifier),str(path),str(dest/'report.json'),
                                   str(dest/'consumer.json')],cwd=dest,capture_output=True,timeout=30)
            atomic(dest/'consumer.log',proc.stdout+proc.stderr)
            if proc.returncode == 0 and (dest/'consumer.json').exists():
                report['consumer'] = json.loads((dest/'consumer.json').read_text(encoding='utf8'))
                report['esmini'] = report['consumer']['esmini']
            else:
                report['esmini'] = 'VERIFIER_FAILED'
        except subprocess.TimeoutExpired:
            report['esmini'] = 'TIMEOUT'
        atomic(dest/'report.json',json_bytes(report))
        self.verify_sources()
        return {'export_id':export_id, 'path':str(path), 'report':report,
                'download':f'/api/exports/{export_id}/candidate.xodr'}
