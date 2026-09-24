"""Scientific actual-XML review; refused source target stays visibly refused."""
import argparse
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from scripts.check_outer_event_control import inputs, REFERENCE
from scripts.check_outer_event_written import boundary_poly
from mapforge.repair_web.model import parse, digest


def main():
    p = argparse.ArgumentParser(); p.add_argument('directory', type=Path)
    dest = p.parse_args().directory.resolve()
    event, control = inputs()
    data = (dest/'candidate.xodr').read_bytes()
    report = json.loads((dest/'range.json').read_text(encoding='utf8'))
    guard = json.loads((dest/'preview.json').read_text(encoding='utf8'))['guard']
    roads = [next(r for r in parse(b).findall('road') if r.get('id') == '11') for b in (REFERENCE.read_bytes(), data)]
    fig, axes = plt.subplots(3, 1, figsize=(12, 11), layout='constrained')
    for trace in event.traces:
        for ax in axes[:2]:
            if ax is axes[1] and trace['edge'] != 3: continue
            ax.plot(trace['st'][:, 0], trace['st'][:, 1], color='#e1972d', lw=2)
    for road, color, label in zip(roads, ('#697889', '#165dd4'), ('r6 reference', 'written control test')):
        for edge in range(5):
            ss = np.linspace(max(150., event.births.get(edge, event.start)), 195., 601)
            ts = [boundary_poly(road, edge, s)[0] for s in ss]
            axes[0].plot(ss, ts, color=color, lw=1.3, label=label if edge == 0 else None)
            if edge == 3:
                axes[1].plot(ss, ts, color=color, lw=1.5, label=label)
                co = np.array([boundary_poly(road, edge, s) for s in ss])
                axes[2].plot(ss, 2*co[:, 2]/(1+co[:, 1]**2)**1.5, color=color, lw=1.5, label=label)
    axes[1].plot(control.point.station, report['source_t_m'], 'x', color='#c83836', ms=12, mew=2,
                 label='source target NOT reached / NOT applied')
    axes[1].plot(control.point.station, control.reference_t+guard['achieved_m'], 'o', color='#165dd4', ms=5)
    axes[0].set_title('All five shared boundaries: orange SHP / gray r6 / blue actual XML')
    axes[1].set_title('Third boundary bulge remains; this is an interaction test, NOT a repair')
    axes[2].set_title('Actual third-boundary curvature; no shape improvement claim')
    for ax in axes:
        ax.set_xlim(150, 195); ax.grid(alpha=.2); ax.legend(loc='best'); ax.set_xlabel('Reference station s (m)')
    # Use the actual local-t domain. Hard-coded world-coordinate limits hide
    # the very boundary/target being inspected in this rotated source chart.
    for ax in axes[:2]: ax.margins(y=.12)
    axes[0].set_ylabel('Lateral t (m)'); axes[1].set_ylabel('Lateral t (m)'); axes[2].set_ylabel('Curvature (1/m)')
    fig.suptitle('BLOCKED research test | requested source move %.6f m | verified lower end %.6f m\n'
                 'separate test requested %.9f m / written %.9f m | SHA %s'
                 % (report['source_point_delta_m'], report['verified_delta_m'][0], guard['requested_m'],
                    guard['achieved_m'], digest(data)[:16]), fontsize=10)
    fig.savefig(dest/'control-review.png', dpi=145)
    plt.close(fig)


if __name__ == '__main__': main()
