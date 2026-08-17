import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "mapforge" / "adapters" / "v2xmap" / "asn"))


@pytest.fixture
def g8_policy():
    limits = {
        "source_to_target": {"median_max_m": 0.2, "p95_max_m": 0.3, "max_max_m": 0.5},
        "target_to_source": {"median_max_m": 0.2, "p95_max_m": 0.3, "max_max_m": 0.5},
        "endpoint_max_m": 0.2,
        "coverage_radius_m": 0.5,
        "source_coverage_min": 0.99,
        "target_coverage_min": 0.99,
        "length_ratio": {"min": 0.99, "max": 1.01},
        "stopline_delta_max_m": 0.2,
    }
    return {
        "schema": "mapforge/g8-policy/v1",
        "policy_id": "g8-test-v1",
        "version": "1.0",
        "lifecycle": "active",
        "applicability": {"target_format": "opendrive", "source_formats": ["synthetic"]},
        "sampling": {"step_m": 1.0, "zero_width_epsilon_m": 0.05,
                     "outlier_limit_per_direction": 5},
        "classes": {"test.measured": limits},
        "allowed_exclusions": [
            "zero-width", "median-non-driving", "paving-non-driving",
            "mirror-no-source-geometry", "inferred-connector-no-source-geometry",
            "source-polygon-paving", "inferred-paving",
        ],
    }


@pytest.fixture
def build_g8_case(tmp_path):
    def build(*, target_length=100.0, source_length=100.0, include_second=True,
              second_width=3.5, second_source="B", second_provenance=True):
        from mapforge.adapters.opendrive.writer import Lane, LaneSection, Road, XodrDoc
        from mapforge.validate.g8_model import make_manifest, source_lane

        doc = XodrDoc("g8-test", geo_reference="+proj=eqc +units=m")
        road = Road(1, name="test")
        road.add_geometry("line", 0.0, 0.0, 0.0, target_length)
        road.add_offset(0.0, 0.0)
        sec = LaneSection(0.0)
        prov_a = {"eligibility": "comparable", "role": "approach",
                  "status": "EXACT", "support_kind": "synthetic",
                  "policy_class": "test.measured", "travel_direction": "with_s"}
        a = Lane(-1, source_id="A", provenance=prov_a)
        a.add_width(3.5)
        sec.right.append(a)
        if include_second:
            prov_b = dict(prov_a) if second_provenance else None
            b = Lane(-2, source_id=second_source, provenance=prov_b)
            b.add_width(second_width)
            sec.right.append(b)
        road.sections.append(sec)
        doc.add_road(road)
        out = tmp_path / f"case-{target_length}-{source_length}-{include_second}-{second_width}.xodr"
        doc.write(out)

        sx = np.linspace(0.0, source_length, int(source_length) + 1)
        lanes = [source_lane(
            "A", np.column_stack([sx, np.full_like(sx, -1.75)]),
            owner={"format": "synthetic", "link": "road"}, role="approach",
            status="EXACT", support_kind="synthetic", policy_class="test.measured",
            travel_direction="with_s", stop_line={"availability": "anchor",
                                                     "coordinates": [[source_length, -1.75]]},
        )]
        if include_second and second_source == "B":
            lanes.append(source_lane(
                "B", np.column_stack([sx, np.full_like(sx, -5.25)]),
                owner={"format": "synthetic", "link": "road"}, role="approach",
                status="EXACT", support_kind="synthetic", policy_class="test.measured",
                travel_direction="with_s", stop_line={"availability": "anchor",
                                                         "coordinates": [[source_length, -5.25]]},
            ))
        manifest = make_manifest(
            source_format="synthetic", source_profile="test",
            comparison_crs={"id": "local-test", "units": "m", "origin": [0, 0]},
            lanes=lanes,
        )
        return out, manifest

    return build
