"""The births/deaths at the station of an event the window advances: each one's opening slope (own section alone and
through the window, as lane_birth_advance._opening_slope measures it) and the advance it would get alone, against the
station's steepest event, whose advance every lane born (dying) there takes (lane_birth_advance keeps one event per
station). cwd = code tree (with refit_birth_window). argv: raw dir, then 'name:road:s' ..."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path.cwd()))
from lxml import etree
from mapforge.ops import lane_birth_advance as BA
from mapforge.ops import lane_refit as LR
from mapforge.ops import refit_birth_window as BW
from mapforge.validate.shp_boundary_fidelity import _origin


def alone(slope):
    if slope is None or slope < BA.SLOPE_MIN:
        return "not advanced"
    return f"{min(max(slope / LR.KAPPA_CAP, BA.ADVANCE_MIN_M), BA.ADVANCE_MAX_M):.2f} m"


def fmt(x):
    return "-" if x is None else f"{x:.3f}"


raw_dir = Path(sys.argv[1])
src = LR._shp_source()
for spec in sys.argv[2:]:
    name, rid, s0 = spec.split(":")
    root = etree.parse(str(raw_dir / f"{name}.raw.xodr")).getroot()
    origin = _origin(root)
    road_el = next(r for r in root.findall("road") if r.get("id") == rid)
    road = LR._Road(road_el)

    def observe(side, i, k):
        try:
            return LR._source_observations(road, src, origin, side, i, k)
        except IndexError:
            return None
    ext = BW.observer(road, observe)
    print(f"== {name} road {rid} station s {float(s0):.2f}")
    for e in BA.events(road_el):
        if abs(e["s"] - float(s0)) < 0.05:
            own, win = BA._opening_slope(observe, e), BA._opening_slope(ext, e)
            print(f"   {e['kind']:5s} {e['side']:5s} lane {e['lane']:>3d}: slope own section {fmt(own)}, "
                  f"through the window {fmt(win)}; alone it would advance {alone(win)}")
