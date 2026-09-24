"""Read-only v0.6 scope/source/anchor review. No fitter or XODR writer call."""
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from mapforge.repair_web.model import atomic, digest, json_bytes, parse
from scripts.check_outer_event_control import inputs
from scripts.check_outer_event_written import boundary_poly

START, END = 61.8028, 211.7074107
DEST = ROOT/'out/node4-split-event-scope-review-20260917'
PRIOR = ROOT/'out/node4-refinement-target-r2-20260917'


def source_at(traces, edge, station):
    rows = []
    for trace in traces:
        if trace['edge'] != edge: continue
        for a, b in zip(trace['st'][:-1], trace['st'][1:]):
            if a[0] <= station <= b[0]:
                slope = (b[1]-a[1])/(b[0]-a[0])
                rows.append(dict(key=trace['key'], record=trace['record'], part=trace['part'],
                    segment_s=[float(a[0]), float(b[0])],
                    t=float(a[1]+slope*(station-a[0])), slope=float(slope)))
    return rows


def anchor(road, traces, edge, station, left):
    c = boundary_poly(road, edge, station, left=left)
    jets = (c*np.array([1., 1., 2., 6.])).tolist()
    source = source_at(traces, edge, station)
    return dict(edge=edge, station=station, side='left' if left else 'right',
        actual_reference_jets=jets, source_segments=source,
        source_direction_disagreement=any(jets[1]*s['slope'] < 0 for s in source),
        source_point_error_m=None if not source else max(abs(jets[0]-s['t']) for s in source),
        source_slope_is_measurement=False, final_anchor_admitted=False)


def review(event, reference):
    root = parse(reference); road = root.find("road[@id='11']")
    if abs(float(road.get('length'))-END) > 1e-8: raise ValueError('Unexpected road length')
    if not any(abs(float(sec.get('s'))-START) < 1e-8 for sec in road.findall('lanes/laneSection')):
        raise ValueError('Proposed upstream boundary is not an existing semantic cut')
    connections = [dict(**c.attrib, lanes=[l.attrib for l in c.findall('laneLink')])
                   for c in root.findall('junction/connection') if c.get('incomingRoad') == '11']
    inventory = []
    for t in event.traces:
        lo, hi = float(t['st'][0, 0]), float(t['st'][-1, 0])
        overlap = [max(START, lo), min(END, hi)]
        inventory.append(dict(key=t['key'], edge=t['edge'], record=t['record'], part=t['part'],
            source_s=[lo, hi], proposed_overlap=overlap if overlap[1] > overlap[0] else None,
            outside_parent_support=[] if lo >= 0 and hi <= END else
                ([[lo, min(0., hi)]] if lo < 0 else [])+([[max(END, lo), hi]] if hi > END else [])))
    return dict(schema='mapforge/split-event-scope-review/v1', status='DESIGN_REVIEW_NOT_EXECUTION_AUTHORIZATION',
        reference_sha256=digest(reference), parent_road='11', analysis_domain=[0., END],
        proposed_edit_domain=[START, END], proposed_length_m=END-START,
        editable_physical_edges=[0, 1, 2, 3, 4], births=event.births,
        upstream_frozen_domain=[0., START], mouth_station=END,
        lane_dependency={'0': ['laneOffset', '-1'], '1': ['-1', '-2'],
                         '2': ['-2', '-3 after first birth'], '3': ['-3', '-4 after second birth'], '4': ['-4']},
        outer_envelope=[dict(edge=2, domain=[START, event.births[3]]),
                        dict(edge=3, domain=[event.births[3], event.births[4]]),
                        dict(edge=4, domain=[event.births[4], END])],
        connections=connections, connector_geometry_edit_allowed=False,
        source_inventory=inventory, source_traces=len(event.traces), movement_paths=len(event.paths),
        old_cut_anchors=[anchor(road, event.traces, edge, s, False)
                        for s, edge in ((100., 2), (157.6089, 3), (201.7074107, 2))],
        proposed_anchors=[anchor(road, event.traces, edge, s, side)
                          for s, side, edges in ((START, True, range(3)), (END, True, range(5))) for edge in edges],
        anchor_scope='Candidate endpoint conditions only; slopes of sparse source chords are not surveyed tangents',
        source_tail_policy='retain all raw source segments; explicitly audit ownership outside parent before any solve',
        optimizer_calls=0, new_xodr=False, map_accepted=False, web_changed=False,
        new_trial_registered=False)


def plot(event, reference, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    root = parse(reference); road = root.find("road[@id='11']")
    fig, axs = plt.subplots(2, 1, figsize=(13, 9), layout='constrained')
    for ax, xlim in zip(axs, ((0, 220), (120, 212))):
        ax.axvspan(START, END, color='#7db3ac', alpha=.15, label='proposed editable interval (NOT applied)')
        ax.axvspan(157.6089, 201.7074107, color='#ddaf70', alpha=.12, label='old tail-only interval')
        first = True
        for trace in event.traces:
            ax.plot(*trace['st'].T, color='#cb801a', lw=1.7, label='original SHP boundary' if first else None)
            first = False
        for edge in range(5):
            ss = np.linspace(event.births.get(edge, 0), END, 801)
            ax.plot(ss, [boundary_poly(road, edge, float(s))[0] for s in ss], color='#50647c', lw=1.1,
                    label='actual r6 XML (unmodified)' if edge == 0 else None)
        for label, s in (('birth -3', 131.0543), ('birth -4', 151.6089), ('locked mouth', END)):
            ax.axvline(s, color='#666666', lw=.8, ls=':'); ax.text(s, 7.6, label, rotation=90, va='top', fontsize=8)
        ax.set_xlim(*xlim); ax.set_ylim(-8.5, 8); ax.grid(alpha=.2)
        ax.set_xlabel('road11 reference station s (m)'); ax.set_ylabel('physical boundary t (m)')
    axs[0].legend(loc='lower left', fontsize=8)
    axs[0].set_title('Entire parent and proposed split-event scope (source tails remain visible)')
    axs[1].set_title('Two linked lane births + transition + downstream stabilization, not one point')
    fig.suptitle('v0.6 SCOPE REVIEW ONLY | no new curve, solver, XODR or Web change', fontsize=13)
    fig.savefig(path, dpi=140); plt.close(fig)


def main():
    if DEST.exists(): raise ValueError('Scope review exists; do not overwrite')
    b = json.loads((PRIOR/'necessary-condition-audit/binding.json').read_bytes())
    for p in (PRIOR/'necessary-condition-audit/conditions.json', Path(__file__).resolve(),
              ROOT/'tests/test_split_event_scope_review.py'):
        b[str(p)] = digest(p.read_bytes())
    def verify():
        for path, sha in b.items():
            if digest(Path(path).read_bytes()) != sha: raise ValueError('Frozen evidence drift: '+path)
    verify(); event, control = inputs(); report = review(event, control.reference); verify()
    DEST.mkdir(exist_ok=False)
    atomic(DEST/'binding.json', json_bytes(b)); atomic(DEST/'scope-review.json', json_bytes(report))
    plot(event, control.reference, DEST/'scope-review.png'); verify()
    atomic(DEST/'status.json', json_bytes(dict(status=report['status'], input_bindings=len(b), input_drift=0,
        optimizer_calls=0, new_xodr=False, new_trial_registered=False, map_accepted=False,
        artifact_sha256={p.name: digest(p.read_bytes()) for p in DEST.iterdir() if p.is_file()})))
    print(json.dumps(dict(proposed_scope=report['proposed_edit_domain'], source_traces=report['source_traces'],
        movement_paths=report['movement_paths'], connections=report['connections'], input_bindings=len(b),
        support_outside_parent=[t for t in report['source_inventory'] if t['outside_parent_support']])))


if __name__ == '__main__': main()
