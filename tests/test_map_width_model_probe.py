"""Relaxed model probes must not be mistaken for geometry qualification."""
from types import SimpleNamespace
import numpy as np
from scripts.map_width_model_probe import vertex_only


def test_probe_keeps_all_raw_vertices_and_hard_structure_without_mutating_model():
    labels=[dict(kind='source-center',source='lane-a',s=s) for s in (0.,1.,2.)]
    labels += [dict(kind='whole-source-center',source='lane-a',s=[0.,2.]),dict(kind='width')]
    model=SimpleNamespace(labels=labels,C=np.arange(10).reshape(5,2),lower=np.arange(5),
                          E=np.eye(2),er=np.array([3.5,3.5]))
    subset,count=vertex_only(model,{'lane-a':np.array([[2.,4.],[0.,4.]])})
    assert count==2
    np.testing.assert_array_equal(subset.C,model.C[[0,2,4]])
    assert subset.E is model.E and subset.er is model.er
    assert len(model.C)==5 and len(model.labels)==5
