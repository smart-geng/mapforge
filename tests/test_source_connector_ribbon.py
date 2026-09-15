import numpy as np
import pytest
from pyclothoids import Clothoid
from mapforge.ops.source_connector_ribbon import fit_source_ribbon,fit_world_ribbon,reference_samples,minimum_forward_factor
from spikes.connector_cross_section import jet
from spikes.measured_connector_caps import sample
from mapforge.validate.smoothness import _edge_world_curvature


def lines():
    result=[];x=0.
    for length in (6.,12.,14.,12.,6.):
        result.append(Clothoid.StandardParams(x,0.,0.,0.,0.,length));x+=length
    return result


def test_fits_complete_source_on_fixed_basis_without_moving_end_jets():
    cls=lines();s=np.linspace(0,50,201);bulge=.2*np.sin(np.pi*s/50)**2
    raw={side:np.c_[s,bulge+y] for side,y in (('left',1.75),('right',-1.75),('center',0.))}
    ends={s:np.array([y,0.,0.]) for s,y in (('left',1.75),('right',-1.75))}
    before={k:v.copy() for k,v in raw.items()};kn,c=fit_source_ribbon(cls,ends,ends,raw)
    assert len(kn)==9 and min(np.diff(kn))==pytest.approx(3.)
    for side in ('left','right'):
        assert jet(c[side][0],0)==pytest.approx(ends[side],abs=1e-9)
        assert jet(c[side][-1],kn[-1]-kn[-2])==pytest.approx(ends[side],abs=1e-9)
        for i,L in enumerate(np.diff(kn)[:-1]):assert jet(c[side][i],L)==pytest.approx(jet(c[side][i+1],0),abs=1e-9)
    pts,jets,_=sample(cls,kn,c,.25)
    assert max(pts['center'][:,1])>.04
    assert max(abs(jets['left'][:,0]-jets['right'][:,0]-3.5))<1e-8
    assert all(np.array_equal(raw[k],v) for k,v in before.items())


def test_sampling_density_does_not_add_width_or_reference_variables():
    cls=lines();ends={s:np.array([y,0.,0.]) for s,y in (('left',2.),('right',-2.))}
    for n in (2,501):
        raw={side:np.c_[np.linspace(0,50,n),np.full(n,y)] for side,y in (('left',2.),('right',-2.),('center',0.))}
        kn,c=fit_source_ribbon(cls,ends,ends,raw)
        assert len(c['left'])==8 and len(cls)==5
        assert c['left'][:,0]==pytest.approx(2.,abs=1e-8)
    with pytest.raises(ValueError,match='at least 3m'):fit_source_ribbon([Clothoid.StandardParams(0,0,0,0,0,4)]*3,ends,ends,raw)


def test_crossing_core_lengths_cannot_switch_basis_topology():
    ends={s:np.array([y,0.,0.]) for s,y in (('left',2.),('right',-2.))}
    raw={s:np.c_[np.linspace(0,32,130),y+.2*np.sin(np.linspace(0,np.pi,130))**2]
         for s,y in (('left',2.),('right',-2.),('center',0.))}
    outputs=[]
    for perturb in (-1e-6,1e-6):
        cls=[];x=0.
        for length in (6.,10.+perturb,10.-perturb,6.):
            cls.append(Clothoid.StandardParams(x,0,0,0,0,length));x+=length
        kn,co=fit_source_ribbon(cls,ends,ends,raw)
        outputs.append((kn,co))
    assert np.max(abs(outputs[0][0]-outputs[1][0]))<3e-6
    assert np.max(abs(outputs[0][1]['left']-outputs[1][1]['left']))<1e-5


def test_world_g2_all_three_curves_without_forcing_flat_offset_slopes():
    cls=[];x=y=h=0.
    for k0,k1 in ((0.,.025),(.025,.025),(.025,0.)):
        c=Clothoid.StandardParams(x,y,h,k0,(k1-k0)/10.,10.)
        cls.append(c);x,y,h=c.XEnd,c.YEnd,c.ThetaEnd
    s=np.linspace(0,30,301);xy,normal=reference_samples(cls,s);bulge=.3*np.sin(np.pi*s/30)**2
    raw={k:xy+(bulge+t)[:,None]*normal for k,t in (('left',1.8),('right',-1.8),('center',0.))}
    ends={k:np.array([t,0.,.6*(np.pi/30)**2]) for k,t in (('left',1.8),('right',-1.8))}
    diagnostics={};kn,co=fit_world_ribbon(cls,ends,ends,raw,diagnostics)
    assert len(kn)-1==6 and len(cls)==3 and min(np.diff(kn))>=3.
    assert all(jet(co[k][0],0)==pytest.approx(v,abs=1e-9) for k,v in ends.items())
    slopes=[]
    for j,st in enumerate((10.,20.)):
        i=int(np.argmin(abs(kn-st)));u=kn[i]-kn[i-1]
        jets={k:(jet(co[k][i-1],u),jet(co[k][i],0)) for k in ('left','right')}
        jets['center']=tuple((a+b)/2 for a,b in zip(jets['left'],jets['right']))
        for a,b in jets.values():
            assert a[:2]==pytest.approx(b[:2],abs=1e-8)
            assert _edge_world_curvature(*a,cls[j].KappaEnd,cls[j].dk)==pytest.approx(
                _edge_world_curvature(*b,cls[j+1].KappaStart,cls[j+1].dk),abs=1e-8)
        slopes.append(abs(jets['center'][0][1]))
    assert max(slopes)>1e-3,str(diagnostics)
    assert diagnostics['jacobian_error']<1e-7
    assert diagnostics['bernstein_regularity_margin']>=-1e-8


def test_common_world_knots_are_not_round_tripped_via_normalized_station():
    cls=[];x=y=h=0.
    for length,k0,k1 in ((6.1231231,0.,.003), (7.81231238,.003,.004),(9.68374555123,.004,0.)):
        c=Clothoid.StandardParams(x,y,h,k0,(k1-k0)/length,length);cls.append(c)
        x,y,h=c.XEnd,c.YEnd,c.ThetaEnd
    L=sum(c.length for c in cls);s=np.linspace(0,L,130);xy,normal=reference_samples(cls,s)
    raw={k:xy+(t+.3*np.sin(np.pi*s/L)**2)[:,None]*normal for k,t in (('left',1.8),('right',-1.8),('center',0.))}
    ends={k:np.array([t,0.,0.]) for k,t in (('left',1.8),('right',-1.8))}
    kn,co=fit_world_ribbon(cls,ends,ends,raw)
    assert set(np.cumsum([c.length for c in cls]))<=set(kn)
    assert minimum_forward_factor(cls,kn,co)>.9
    import xml.etree.ElementTree as ET
    from spikes.measured_connector_caps import write_ribbon
    from scripts.internal_edge_jets import audit
    from scripts.review_source_connectors import written_forward_minimum
    road=ET.fromstring('<road id="1" junction="0"><planView/><lanes><laneSection s="0"><right><lane id="-1" type="driving"/></right></laneSection></lanes></road>')
    written=write_ribbon(road,cls,kn,co);root=ET.Element('OpenDRIVE');root.append(written)
    assert audit(root)['status']=='PASS'
    assert written_forward_minimum(written)==pytest.approx(minimum_forward_factor(cls,kn,co),abs=1e-10)
    from mapforge.ops.joint_connector_fit import basis_for,coefficients_to_basis
    joint_kn,joint_basis,_=basis_for(cls)
    assert np.array_equal(joint_kn,kn)
    values=coefficients_to_basis(kn,co,joint_basis).reshape(2,-1)
    for side,c in zip(('left','right'),values):
        for j,st in enumerate(kn[:-1]):
            for derivative in range(3):
                assert joint_basis(st/kn[-1],derivative)@c/kn[-1]**derivative==pytest.approx(
                    jet(co[side][j],0)[derivative],abs=1e-9)


def test_forward_factor_rejects_interior_fold_even_with_positive_width():
    cls=[Clothoid.StandardParams(0,0,0,.2,0,10)]
    # Positive constant width; both endpoints forward, middle folds backwards.
    co={'left':np.array([[1.,2.,-.2,0.]]),'right':np.array([[0.,2.,-.2,0.]])}
    assert minimum_forward_factor(cls,np.array([0.,10.]),co)==pytest.approx(-.2)
    import xml.etree.ElementTree as ET
    from spikes.measured_connector_caps import write_ribbon
    from scripts.review_source_connectors import written_forward_minimum
    from scripts.review_measured_ribbon import minimum_width
    road=ET.fromstring('<road id="1" junction="0"><planView/><lanes><laneSection s="0"><right><lane id="-1" type="driving"/></right></laneSection></lanes></road>')
    written=write_ribbon(road,cls,np.array([0.,10.]),co)
    assert minimum_width(written)==pytest.approx(1.)
    assert written_forward_minimum(written)==pytest.approx(-.2)
