"""One source-supported lateral transition between protected straight caps.

Research alternative to unconstrained few-clothoid fitting, not a road writer.
The two cap ranges are model hypotheses fitted from the original polyline;
two-point support in particular is NOT a surveyed straight-line certificate.
No source vertices, limits, topology or accepted XODR are modified.
"""
import numpy as np
from pyclothoids import Clothoid, SolveG2
from scipy.optimize import minimize

from spikes.sparse_path_model import observations, point_to_segments, sample, straight_candidate, verify


def straight_caps(raw, residual_m=.08, minimum_length_m=20.):
    """Longest low-residual prefix/suffix; reject overlap and missing support."""
    choices=[]
    for reverse in (False,True):
        points=raw[::-1] if reverse else raw
        selected=None
        for n in range(2,len(points)):
            line=straight_candidate(points[:n])
            if line is None:break
            errors=[line.Distance(float(x),float(y)) for x,y in points[:n]]
            if max(errors)>residual_m:break
            if line.length>=minimum_length_m:
                selected=(len(raw)-n if reverse else n-1,line,max(errors),n)
        if selected is None:raise ValueError('no long source-supported straight cap')
        choices.append(selected)
    if choices[0][0]>=choices[1][0]:raise ValueError('straight caps overlap; try single line first')
    return choices


def heading_extrema(curves):
    values=[]
    for c in curves:
        stations=[0.,c.length]
        if abs(c.dk)>1e-14 and 0 < -c.KappaStart/c.dk < c.length:
            stations.append(-c.KappaStart/c.dk)
        values.extend(c.Theta(s) for s in stations)
    return np.array(values)


def fit_transition(points,limits,speed_frontier=False,maxiter=150):
    """Line + 3 G2 spirals + line, four geometric variables, no tiny fallback.

    Variables: two straight-cap cut stations and two normal displacements.
    Cut stations can move at most 10m within the full original support.
    Straight tangents remain fixed; zero curvature at both cap interfaces.
    A heading envelope prohibits the optimizer's extra countersteer lobes.
    """
    limits.check();raw=np.array(points,float,copy=True);obs,arc=observations(raw,1.)
    try:caps=straight_caps(raw)
    except ValueError as e:return None,{'status':'REJECTED','reason':str(e),'trials':[]}
    i,a,ea,na=caps[0];j,back,eb,nb=caps[1]
    p0=np.array([a.XStart,a.YStart]);p1=np.array([back.XStart,back.YStart])
    h0=a.ThetaStart
    h1=h0+np.arctan2(np.sin(back.ThetaStart+np.pi-h0),np.cos(back.ThetaStart+np.pi-h0))
    d0=np.array([np.cos(h0),np.sin(h0)]);d1=np.array([np.cos(h1),np.sin(h1)])
    normal0=np.array([-d0[1],d0[0]]);normal1=np.array([-d1[1],d1[0]])
    shift=float((p1-p0)@normal0);sign=float(np.sign(shift))
    if abs(h1-h0)>.1 or abs(shift)<.25:
        return None,{'status':'REJECTED','reason':'not a near-parallel lateral-transition hypothesis','trials':[]}
    # A minimum 10m straight cap prevents dissolving real straights into a
    # whole-path S bend. This is the tested model budget, not a standard.
    bounds=[(max(10.,a.length-10),a.length+5),
            (max(10.,back.length-10),back.length+5),
            (-limits.source_m,limits.source_m),(-limits.source_m,limits.source_m)]
    z0=np.array([max(10.,a.length-5),max(10.,back.length-5),0.,0.])
    if not speed_frontier:
        z0=np.r_[z0,5.];bounds.append((0.,float(arc[-1])))
    source=raw-p0;data=obs-p0

    def build(z,world=False):
        start=normal0*z[2];end=p1-p0+normal1*z[3]
        q0=start+d0*z[0];q1=end-d1*z[1]
        middle=SolveG2(*q0,h0,0.,*q1,h1,0.)
        first=Clothoid.StandardParams(*start,h0,0.,0.,z[0])
        last=Clothoid.StandardParams(*q1,h1,0.,0.,z[1])
        cc=[first,*middle,last]
        if world:
            cc=[Clothoid.StandardParams(c.XStart+p0[0],c.YStart+p0[1],c.ThetaStart,
                                       c.KappaStart,c.dk,c.length) for c in cc]
        return cc

    cached={}
    def evaluate(z):
        key=tuple(z)
        if key not in cached:
            cc=build(z)
            forward=np.array([min(c.Distance(float(x),float(y)) for c in cc) for x,y in data])
            reverse=point_to_segments(sample(cc,per_segment=25),source)
            errors=np.r_[forward,reverse,abs(z[2:4])]
            kk=max(max(abs(c.KappaStart),abs(c.KappaEnd)) for c in cc)
            dk=max(abs(c.dk) for c in cc)
            supported=min(np.sqrt(limits.acceleration_mps2/max(kk,1e-15)),
                          np.cbrt(limits.jerk_mps3/max(dk,1e-15)))
            cached.clear();cached[key]=(cc,errors,kk,dk,supported)
        return cached[key]

    def constraints(z):
        cc,errors,kk,dk,supported=evaluate(z)
        # Fixed-size analytic envelope, including quadratic heading extrema.
        hs=sign*(heading_extrema(cc)-h0)
        heading_floor=min(0.,sign*(h1-h0))
        geometry=np.r_[min(hs)-heading_floor, .6-max(hs),
                       [c.length-limits.minimum_span_m for c in cc]]
        if speed_frontier:return np.r_[limits.source_m-.003-errors,geometry]
        return np.r_[z[-1]-errors,geometry,
                     limits.acceleration_mps2-kk*limits.speed_ms**2,
                     limits.jerk_mps3-dk*limits.speed_ms**3]

    answer=minimize(lambda z:-min(evaluate(z)[4],limits.speed_ms) if speed_frontier else z[-1],
                    z0,method='SLSQP',bounds=bounds,constraints=[{'type':'ineq','fun':constraints}],
                    options={'maxiter':maxiter,'ftol':1e-9})
    cc=build(answer.x,world=True);report=verify(cc,raw,limits)
    hs=sign*(heading_extrema(cc)-h0);floor=min(0.,sign*(h1-h0))
    shape_ok=bool(min(hs)>=floor-1e-8 and max(hs)<=.6+1e-8)
    if not shape_ok:report['status']='REJECTED'
    report.update(model_kind='line-3spirals-line',parameters=[list(c.Parameters) for c in cc],
                  formulation='protected-straights-speed-diagnostic' if speed_frontier else 'protected-straights-fixed-speed',
                  optimizer_success=bool(answer.success),optimizer_message=str(answer.message),
                  optimizer_iterations=int(answer.nit),geometry_variables=answer.x[:4].tolist(),
                  shape_preservation='fixed straight cap tangents and zero curvature; one bounded heading envelope',
                  heading_envelope_pass=shape_ok,
                  cap_hypotheses={'prefix_end_index':i,'suffix_start_index':j,
                      'raw_vertices':[na,nb],'line_residual_m':[ea,eb],
                      'cap_classification_tolerance_m':.08,'minimum_cap_support_m':20.,
                      'qualification':'polyline-supported model hypothesis, not surveyed straight geometry'})
    result={'status':report['status'],'trials':[report],'limits':limits.__dict__,
            'scope':'single path, source-supported straight-cap hypothesis; not multi-lane reconstruction or export'}
    return (cc if report['status']=='PATH_CANDIDATE' else None),result
