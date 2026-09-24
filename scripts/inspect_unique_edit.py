"""Plot the saved S2 state against raw SHP/reference XML; no new candidate."""
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from mapforge.repair_web.model import atomic, json_bytes, digest
from mapforge.repair_web.outer_event_fairing import critical_parameters, world_geometry
from scripts.check_outer_event_control import inputs
from scripts.check_outer_event_written import boundary_poly


def main():
    trial = ROOT/'out/node4-unique-local-target-s2-20260915-r1'
    dest = trial/'inspection'
    if dest.exists(): raise ValueError('Inspection already exists; do not overwrite evidence')
    bindings = json.loads((trial/'binding.json').read_bytes())
    status = json.loads((trial/'status.json').read_bytes())
    for name, sha in status['artifact_sha256'].items(): bindings[str(trial/name)] = sha
    bindings[str(trial/'status.json')] = digest((trial/'status.json').read_bytes())
    bindings[str(Path(__file__).resolve())] = digest(Path(__file__).read_bytes())
    for path, sha in bindings.items():
        if digest(Path(path).read_bytes()) != sha: raise ValueError('Evidence drift: '+path)
    report = json.loads((trial/'evaluation.json').read_bytes())
    state = np.asarray(json.loads((trial/'state.json').read_bytes())['coefficients'])
    event, control = inputs()
    road = control.event.root.find("road[@id='11']")
    # The display baseline must be the actual r6 XML, not the older event root.
    from mapforge.repair_web.model import parse
    road = parse(control.reference).find("road[@id='11']")
    fig, axs = plt.subplots(2, 2, figsize=(15, 10), layout='constrained')
    colors = dict(source='#d18719', reference='#536b87', rejected='#c3374b')
    for i, ax in enumerate(axs.ravel()):
        ax.grid(alpha=.2); ax.set_xlabel('reference station s (m)')
        ax.set_xlim(event.start if i in (0, 3) else 150., event.end)
    labels = set()
    for trace in event.traces:
        for i in (0, 1):
            if i == 1 and trace['edge'] != 3: continue
            key = (i, 'source')
            axs.ravel()[i].plot(trace['st'][:, 0], trace['st'][:, 1], color=colors['source'], lw=2,
                               label='original SHP boundary' if key not in labels else None)
            labels.add(key)
    for edge, bs in event.splines.items():
        ss = np.linspace(min(bs.t), max(bs.t), 801)
        original = np.array([boundary_poly(road, edge, float(s))[0] for s in ss])
        trial_values = np.array([event.row(edge, float(s))@state for s in ss])
        for i in (0, 1):
            if i == 1 and edge != 3: continue
            ax = axs.ravel()[i]
            ax.plot(ss, original, color=colors['reference'], lw=2.3,
                    label='actual r6 XML' if edge == (0 if i == 0 else 3) else None)
            ax.plot(ss, trial_values, color=colors['rejected'], lw=1.1, ls='--',
                    label='REJECTED coefficients (not XML)' if edge == (0 if i == 0 else 3) else None)
    axs[0, 0].set_title('All physical boundaries: only edge 3 may change')
    axs[0, 1].set_title('Selected edge 3: a correct target point is not a good full shape')
    axs[0, 1].plot(178., report['target_t_m'], 'kx', ms=9, label='exact source target')
    for ax in axs[0]: ax.set_ylabel('lateral coordinate t (m)'); ax.legend(fontsize=8)
    ss = np.linspace(150., event.end, 1601)
    for state_name, color in (('reference', colors['reference']), ('rejected', colors['rejected'])):
        values = []
        for s in ss:
            if state_name == 'reference':
                c = boundary_poly(road, 3, float(s)); jets = [c[1], 2*c[2], 6*c[3]]
            else:
                jets = [event.row(3, float(s), j)@state for j in (1, 2, 3)]
            values.append(world_geometry(jets)[0][0])
        axs[1, 0].plot(ss, values, color=color, label='actual r6 XML' if state_name == 'reference' else 'rejected state')
    axs[1, 0].set_ylabel('world boundary curvature (1/m)')
    axs[1, 0].set_title('Edge 3 world curvature: peaks are checked algebraically')
    axs[1, 0].legend(fontsize=8)
    for trace, lo, hi, source in event.source_cells():
        if trace['edge'] != 3: continue
        ss = np.linspace(lo, hi, 20)
        truth = np.polynomial.polynomial.polyval(ss-lo, source)
        for label, color in (('reference', colors['reference']), ('rejected', colors['rejected'])):
            yy = np.array([boundary_poly(road, 3, float(s))[0] if label == 'reference' else event.row(3, float(s))@state for s in ss])
            key = (3, label)
            axs[1, 1].plot(ss, abs(yy-truth), color=color, lw=1.2, label=label if key not in labels else None)
            labels.add(key)
    axs[1, 1].axhline(.75, color='black', ls=':', label='unchanged L01 source gate 0.75m')
    axs[1, 1].set_ylabel('absolute source error (m)')
    axs[1, 1].set_title('Edge 3 source accuracy passes; shape checks still reject')
    axs[1, 1].legend(fontsize=8)
    fig.suptitle('S2 SINGLE FIXED-SCOPE TARGET | REJECTED STATE, NO NEW XODR\nOriginal SHP / actual r6 XML / saved coefficients; plot sampling is display-only', fontsize=13)
    dest.mkdir(exist_ok=False)
    fig.savefig(dest/'unique-target-review.png', dpi=150); plt.close(fig)
    witnesses = []
    for edge, bs in event.splines.items():
        cuts = sorted(set(bs.t))
        for lo, hi in zip(cuts, cuts[1:]):
            c = event.power(edge, lo)@state
            for index, us in enumerate(critical_parameters(c, hi-lo)):
                for u in us:
                    s = lo+u*(hi-lo)
                    jets = [np.polynomial.polynomial.polyval(s-lo, np.polynomial.polynomial.polyder(c, j)) for j in (1, 2, 3)]
                    witnesses.append(dict(edge=edge, metric=['curvature', 'curvature_rate'][index], station_m=s,
                                           value=float(world_geometry(jets)[0][index]), interval_m=[float(lo), float(hi)]))
    result = dict(schema='mapforge/unique-target-inspection/v1', state_is_xml=False, map_accepted=False,
                  metric_witnesses=[max((r for r in witnesses if r['edge'] == e and r['metric'] == metric), key=lambda r: abs(r['value']))
                                    for e in range(event.count+1) for metric in ('curvature', 'curvature_rate')],
                  source_worst=max(report['source']['rows'], key=lambda r: r['max_m']),
                  png_sha256=digest((dest/'unique-target-review.png').read_bytes()))
    for path, sha in bindings.items():
        if digest(Path(path).read_bytes()) != sha: raise ValueError('Evidence drift after plot: '+path)
    atomic(dest/'binding.json', json_bytes(bindings)); atomic(dest/'inspection.json', json_bytes(result))
    print(json.dumps(dict(edge3_witnesses=[r for r in result['metric_witnesses'] if r['edge'] == 3],
                          source_worst=result['source_worst'], output=str(dest/'unique-target-review.png'))))


if __name__ == '__main__': main()
