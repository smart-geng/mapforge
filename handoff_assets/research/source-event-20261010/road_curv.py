import sys, xml.etree.ElementTree as ET
sys.path.insert(0, ".")
from mapforge.validate.smoothness import edge_shape_quality
root = ET.parse(sys.argv[1]).getroot()
for rid in sys.argv[2].split(","):
    road = next(r for r in root.findall("road") if r.get("id") == rid)
    q = edge_shape_quality(road)
    print(f"  road {rid}: outer curvature max {q['outer_curvature_max']:.4f}  edge curvature max {q['edge_curvature_max']:.4f}  flips/100m {q['outer_curvature_flips_per_100m_max']:.2f}")
