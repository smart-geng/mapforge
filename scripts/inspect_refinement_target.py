"""Read-only rendering of the saved R2 state; never solve, compile or fit."""
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from mapforge.repair_web.edit_scope import EditScopeRequest, ScopeHandle
from mapforge.repair_web.local_refinement import prepare_local_refinement, LO, HI, EDGE
from mapforge.repair_web.model import atomic, digest, json_bytes, parse
from mapforge.repair_web.outer_event_fairing import world_geometry, critical_parameters
from mapforge.repair_web.refinement_target import RefinementProblem
from scripts.check_outer_event_control import inputs, POINT
from scripts.check_outer_event_written import boundary_poly


def main():
    trial = ROOT/'out/node4-refinement-target-r2-20260917'
    dest = trial/'inspection'
    if dest.exists(): raise ValueError('Read existing inspection; do not overwrite')
    binding = json.loads((trial/'binding.json').read_bytes())
    summary = json.loads((trial/'status.json').read_bytes())
    for name, sha in summary['artifact_sha256'].items(): binding[str(trial/name)] = sha
    binding[str(trial/'status.json')] = digest((trial/'status.json').read_bytes())
    def verify():
        for path, sha in binding.items():
            if digest(Path(path).read_bytes()) != sha: raise ValueError('Evidence drift: '+path)
    verify()
    saved = json.loads((trial/'state.json').read_bytes())
    if saved['coefficients'] is None: raise ValueError('No saved nonzero state; inspect linear conflict report instead')
    state = np.asarray(saved['coefficients'])
    report = json.loads((trial/'evaluation.json').read_bytes())
    registration = json.loads((trial/'registration.json').read_bytes())
    event, control = inputs()
    model = prepare_local_refinement(event, control.reference,
        EditScopeRequest(control.reference_sha256, '11', (EDGE,), (LO, HI), (ScopeHandle(POINT),)))
    if model.contract_sha256 != saved['r1_contract_sha256']: raise ValueError('Contract drift')
    problem = RefinementProblem(event, control, model)  # builds affine cells only; no new target search
    actual = None
    if summary['new_xodr']:
        data = (trial/'candidate.xodr').read_bytes()
        if digest(data) != summary['xodr_sha256']: raise ValueError('Candidate SHA mismatch')
        actual = parse(data).find("road[@id='11']")
    def power(edge, s, current):
        if current and actual is not None: return boundary_poly(actual, edge, float(s))
        return model.power(edge, float(s), state if current else np.zeros(2))
    label = 'actual R2 candidate XML (NOT map approved)' if actual is not None else 'REJECTED R2 coefficients (NOT XML)'
    fig, axs = plt.subplots(2, 2, figsize=(15, 10), layout='constrained')
    colors = ['#52687e', '#bd3656']; labels = set()
    for i, ax in enumerate(axs.ravel()):
        ax.grid(alpha=.2); ax.set_xlabel('reference station s (m)')
        ax.set_xlim(event.start if i == 0 else 150., event.end)
        ax.axvspan(LO, HI, color='#4ca1a1', alpha=.06)
    for trace in event.traces:
        for i in (0, 1):
            if i == 1 and trace['edge'] != EDGE: continue
            axs.ravel()[i].plot(*trace['st'].T, color='#d18719', lw=2,
                label='original SHP boundary' if i not in labels else None)
            labels.add(i)
    for edge in range(5):
        ss = np.linspace(event.births.get(edge, event.start), event.end, 801)
        for current, color in enumerate(colors):
            yy = np.array([power(edge, s, current)[0] for s in ss])
            for i in (0, 1):
                if i == 1 and edge != EDGE: continue
                axs.ravel()[i].plot(ss, yy, color=color, lw=2.2 if not current else 1.2,
                    ls='-' if not current else '--',
                    label=('actual r6 reference XML' if not current else label) if edge == (0 if i == 0 else EDGE) else None)
    axs[0, 0].set_title('Five physical boundaries; only shared edge 3 can change')
    axs[0, 1].set_title('Full selected boundary, including before/after the target')
    axs[0, 1].plot(POINT.station, registration['source_t_m'], 'kx', ms=9, label='original source target')
    for ax in axs[0]: ax.set_ylabel('lateral coordinate t (m)'); ax.legend(fontsize=8)
    ss = np.linspace(150., event.end, 1601)
    for current, color in enumerate(colors):
        values = []
        for s in ss:
            c = power(EDGE, s, current)
            values.append(world_geometry([c[1], 2*c[2], 6*c[3]])[0][0])
        axs[1, 0].plot(ss, values, color=color, label='r6' if not current else 'saved R2')
    axs[1, 0].set_title('World boundary curvature (critical roots checked separately)')
    axs[1, 0].set_ylabel('curvature (1/m)'); axs[1, 0].legend()
    for trace, lo, hi, source in event.source_cells():
        if trace['edge'] != EDGE: continue
        ss = np.linspace(lo, hi, 25); truth = np.polynomial.polynomial.polyval(ss-lo, source)
        for current, color in enumerate(colors):
            yy = np.array([power(EDGE, s, current)[0] for s in ss])
            key = ('error', current)
            axs[1, 1].plot(ss, abs(yy-truth), color=color,
                label=('r6' if not current else 'saved R2') if key not in labels else None)
            labels.add(key)
    axs[1, 1].axhline(.75, color='black', ls=':', label='unchanged local source guard')
    axs[1, 1].set_title('Source error: local comparison, not whole-map accuracy')
    axs[1, 1].set_ylabel('absolute source error (m)'); axs[1, 1].legend(fontsize=8)
    fig.suptitle(f"R2 | {summary['status']} | NO MAP RELEASE\nOriginal SHP / actual r6 XML / saved R2; display sampling adds no geometry", fontsize=12)
    witnesses = []
    for cp in problem.spans:
        if cp['edge'] != EDGE: continue
        L = cp['hi']-cp['lo']; c = power(EDGE, cp['lo'], True)
        for index, us in enumerate(critical_parameters(c, L)):
            for u in us:
                d = u*L
                jets = [np.polynomial.polynomial.polyval(d, np.polynomial.polynomial.polyder(c, j)) for j in (1, 2, 3)]
                witnesses.append(dict(metric=['curvature', 'curvature_rate'][index], s=cp['lo']+d,
                                      value=float(world_geometry(jets)[0][index])))
    result = dict(status=summary['status'], map_accepted=False, actual_xml=actual is not None,
        witnesses=[max((w for w in witnesses if w['metric'] == m), key=lambda w: abs(w['value']))
                   for m in ('curvature', 'curvature_rate')],
        source_worst=max(report['guard_evaluation']['source']['rows'], key=lambda r: r['max_m'])
            if report.get('guard_evaluation') else None)
    verify(); dest.mkdir(exist_ok=False)
    fig.savefig(dest/'refinement-target-review.png', dpi=150); plt.close(fig)
    result['png_sha256'] = digest((dest/'refinement-target-review.png').read_bytes())
    verify(); atomic(dest/'binding.json', json_bytes(binding)); atomic(dest/'inspection.json', json_bytes(result))
    print(json.dumps(result))


if __name__ == '__main__': main()
