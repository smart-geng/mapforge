"""Alternative world-path model: few G2 clothoids, free span lengths.

Research only. The source is a PATH observation, not a physical boundary or
an OpenDRIVE lane midpoint by assumption. No road, width, topology or speed
is written here. A path candidate cannot approve a multi-lane map.
"""
from dataclasses import dataclass
import numpy as np
from pyclothoids import Clothoid
from scipy.optimize import minimize


@dataclass(frozen=True)
class Limits:
    speed_ms: float
    source_m: float = .35
    acceleration_mps2: float = 2.5
    jerk_mps3: float = 1.
    minimum_span_m: float = 5.

    def check(self):
        if not np.isfinite(list(self.__dict__.values())).all() or min(self.__dict__.values()) <= 0:
            raise ValueError('limits must be finite and positive')


def observations(points, step):
    if not np.isfinite(step) or step<=0:
        raise ValueError('positive finite sampling step required')
    g=np.asarray(points,float)
    if g.ndim!=2 or g.shape[1]!=2 or len(g)<2 or not np.isfinite(g).all():
        raise ValueError('finite ordered 2D source path required')
    s=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(g,axis=0),axis=1))]
    if s[-1]<1e-6 or np.any(np.diff(s)<1e-9):
        raise ValueError('duplicate/degenerate source vertices: resolve identities explicitly')
    q=np.unique(np.r_[s,np.arange(0,s[-1],step)])
    return np.c_[np.interp(q,s,g[:,0]),np.interp(q,s,g[:,1])],s


def chain(z, n, origin, heading):
    """One initial pose, lengths, and shared curvature nodes: G2 by construction."""
    x,y=origin+z[:2];h=heading+z[2]
    lengths=z[3:3+n];nodes=z[3+n:4+2*n]/100.
    out=[]
    for length,k0,k1 in zip(lengths,nodes[:-1],nodes[1:]):
        c=Clothoid.StandardParams(float(x),float(y),float(h),float(k0),float((k1-k0)/length),float(length))
        out.append(c);x,y,h=c.XEnd,c.YEnd,c.ThetaEnd
    return out


def sample(curves, step=.25, per_segment=None):
    parts=[]
    for c in curves:
        ss=np.linspace(0,c.length,per_segment) if per_segment else np.unique(np.r_[np.arange(0,c.length,step),c.length])
        parts.append(np.array([[c.X(float(s)),c.Y(float(s))] for s in ss]))
    return np.concatenate([p if i==0 else p[1:] for i,p in enumerate(parts)])


def point_to_segments(points, polyline):
    a=polyline[:-1];v=np.diff(polyline,axis=0);den=np.sum(v*v,axis=1)
    good=den>1e-15;a=a[good];v=v[good];den=den[good]
    if not len(a):raise ValueError('degenerate polyline')
    d=points[:,None,:]-a
    t=np.clip(np.einsum('nmi,mi->nm',d,v)/den,0,1)
    return np.linalg.norm(d-t[:,:,None]*v,axis=2).min(axis=1)


def straight_candidate(raw):
    """Arc-length-weighted TLS line, retaining the full projected extent.

    This is a candidate, not a declaration that the source is straight.
    Independent full-source and endpoint checks decide whether to use it.
    """
    obs,_=observations(raw,1.)
    mean=obs.mean(axis=0)
    direction=np.linalg.svd(obs-mean,full_matrices=False)[2][0]
    if np.dot(direction,raw[-1]-raw[0])<0:direction=-direction
    station=(np.asarray(raw)-mean)@direction
    if np.any(np.diff(station)<=0):return None
    start=mean+station.min()*direction
    return Clothoid.StandardParams(*start,float(np.arctan2(direction[1],direction[0])),
                                   0.,0.,float(np.ptp(station)))


def verify(curves, raw, limits):
    """Exact source-point distance and analytic dynamics; dense reverse check."""
    limits.check();raw=np.asarray(raw,float)
    obs,_=observations(raw,.1)
    direct=np.array([min(c.Distance(float(x),float(y)) for c in curves) for x,y in obs])
    back=point_to_segments(sample(curves,.1),np.asarray(raw))
    endpoints=[np.linalg.norm(np.array([curves[0].XStart,curves[0].YStart])-raw[0]),
               np.linalg.norm(np.array([curves[-1].XEnd,curves[-1].YEnd])-raw[-1])]
    ay=max(max(abs(c.KappaStart),abs(c.KappaEnd)) for c in curves)*limits.speed_ms**2
    jy=max(abs(c.dk) for c in curves)*limits.speed_ms**3
    kmax=max(max(abs(c.KappaStart),abs(c.KappaEnd)) for c in curves)
    dkmax=max(abs(c.dk) for c in curves)
    supported=min(np.sqrt(limits.acceleration_mps2/kmax) if kmax>0 else np.inf,
                  np.cbrt(limits.jerk_mps3/dkmax) if dkmax>0 else np.inf)
    joins=[{'position_m':float(np.hypot(a.XEnd-b.XStart,a.YEnd-b.YStart)),
            'heading_rad':float(abs(np.arctan2(np.sin(a.ThetaEnd-b.ThetaStart),np.cos(a.ThetaEnd-b.ThetaStart)))),
            'curvature_per_m':float(abs(a.KappaEnd-b.KappaStart))} for a,b in zip(curves,curves[1:])]
    # Relative to the source chord, prohibit loops/backtracking. In this
    # prototype heading(s) is quadratic, so test its vertex as well as ends.
    h=np.arctan2(*(raw[-1]-raw[0])[::-1]);heading_values=[]
    for c in curves:
        ss=[0,c.length]
        if abs(c.dk)>1e-14 and 0 < -c.KappaStart/c.dk < c.length:ss.append(-c.KappaStart/c.dk)
        heading_values.extend(c.Theta(float(s))-h for s in ss)
    monotone=bool(max(abs(v) for v in heading_values)<np.pi/2)
    connected=all(j['position_m']<=1e-8 and j['heading_rad']<=1e-10 and
                  j['curvature_per_m']<=1e-10 for j in joins)
    # Distance to a closed set is 1-Lipschitz. Each source/curve arc interval
    # is covered by endpoint samples at spacing <= step, so d_max+step/2
    # bounds the entire interval (plus numerical distance margin). This
    # certifies the supplied polyline, NOT unknown surveyed geometry between
    # sparse original vertices. Refinement adds no curve parameters.
    step=.1;forward_upper=float(max(direct)+step/2+1e-8)
    reverse_upper=float(max(back)+step/2+1e-8)
    margin=limits.source_m-max(max(direct),max(back))
    if margin>0 and max(forward_upper,reverse_upper)>limits.source_m:
        refined=max(.0005,min(.1,.8*margin))
        # Bounded work in this research verifier; a budget exhaustion rejects
        # the candidate rather than treating a finite-point check as proof.
        if max(sum(c.length for c in curves),observations(raw,1.)[1][-1])/refined<500000:
            step=refined;obs,_=observations(raw,step)
            direct=np.array([min(c.Distance(float(x),float(y)) for c in curves) for x,y in obs])
            back=point_to_segments(sample(curves,step),raw)
            forward_upper=float(max(direct)+step/2+1e-8)
            reverse_upper=float(max(back)+step/2+1e-8)
    certified=bool(connected and max(forward_upper,reverse_upper)<=limits.source_m)
    report={'source_to_curve_max_m':float(max(direct)), 'curve_to_source_sampled_max_m':float(max(back)),
        'endpoint_max_m':float(max(endpoints)),'acceleration_mps2':float(ay),'jerk_mps3':float(jy),
        'requested_speed_kmh':limits.speed_ms*3.6,
        'supported_speed_kmh':float(supported*3.6) if np.isfinite(supported) else None,
        'primitive_count':len(curves),'lengths_m':[float(c.length) for c in curves],
        'analytic_g2_joins':joins,'monotone_in_source_chord':monotone,
        'g2_connected':connected,
        'verification_scope':'1-Lipschitz tube bounds against the supplied polyline; numerical distance routine, not surveyed geometry proof',
        'source_tube_upper_bound_m':forward_upper,'reverse_tube_upper_bound_m':reverse_upper,
        'tube_certified':certified,'certificate_step_m':step,'numerical_distance_margin_m':1e-8,
        'shape_preservation':'not certified by source tube or G2 alone',
        'reverse_verification_step_m':step,'raw_vertex_count':len(raw),
        'source_geometry_role':'ordered-path-observation; not a certified lane midpoint'}
    report['status']='PATH_CANDIDATE' if (max(direct)<=limits.source_m+1e-6 and
        max(back)<=limits.source_m+1e-6 and max(endpoints)<=limits.source_m+1e-6 and
        ay<=limits.acceleration_mps2+1e-6 and jy<=limits.jerk_mps3+1e-6 and monotone and connected and certified and
        min(c.length for c in curves)>=limits.minimum_span_m-1e-7) else 'REJECTED'
    return report


def fit(points, limits, counts=(1,3,5), maxiter=200, speed_frontier=False):
    """Lexicographic search over requested counts; no dense fallback.

    Default: within each count minimize the maximum fitting residual with
    dynamics held hard. Optional speed_frontier holds the source tube hard
    and seeks the highest supported speed <= requested. This is DIAGNOSTIC:
    final acceptance ALWAYS reuses the original requested speed in Limits.
    SLSQP failure is local/model failure, NOT global infeasibility.
    The independent verifier decides; raw arrays remain unchanged.
    """
    limits.check();raw=np.array(points,float,copy=True);obs,arc=observations(raw,1.)
    if counts not in ((1,), (1,3), (1,3,5), (3,), (5,)):
        raise ValueError('research model supports only bounded 1/3/5 search')
    origin=raw[0];h=np.arctan2(*(raw[-1]-raw[0])[::-1]);length=arc[-1]
    local=raw-origin
    # All optimizer coordinates are local to avoid geographic magnitude loss.
    data=obs-origin;source=raw-origin
    maxk=.15 if speed_frontier else limits.acceleration_mps2/limits.speed_ms**2
    maxdk=limits.jerk_mps3/limits.speed_ms**3
    trials=[]
    if counts[0]==1:
        line=straight_candidate(raw)
        if line is not None:
            report=verify([line],raw,limits)
            report.update(model_kind='line',parameters=[list(line.Parameters)],
                          formulation='line-first-verified-proposal')
            trials.append(report)
            if report['status']=='PATH_CANDIDATE':
                return [line],{'status':'PATH_CANDIDATE','trials':trials,'limits':limits.__dict__,
                              'scope':'single line path only; no width/graph/road/export approval'}
    for n in counts:
        if length<n*limits.minimum_span_m:
            trials.append({'count':n,'status':'REJECTED','reason':'insufficient length for span budget'})
            continue
        z0=np.r_[0.,0.,0.,np.full(n,length/n),np.zeros(n+1),limits.speed_ms*.6 if speed_frontier else 5.]
        bounds=[(-limits.source_m,limits.source_m)]*2+[(-.6,.6)]+[(limits.minimum_span_m,length*1.1)]*n
        bounds += [(-100*maxk,100*maxk)]*(n+1)+[(.1,limits.speed_ms) if speed_frontier else (0.,max(length,10.))]
        cached={}
        def evaluate(z):
            key=tuple(z)
            if key not in cached:
                cc=chain(z,n,np.zeros(2),h)
                # Optimizer reverse samples use fixed per-primitive count,
                # so the constraint vector cannot change during a solve.
                sampled=sample(cc,per_segment=25)
                direct=np.array([min(c.Distance(float(x),float(y)) for c in cc) for x,y in data])
                reverse=point_to_segments(sampled,source)
                ends=np.array([np.linalg.norm(z[:2]),np.linalg.norm(sampled[-1]-local[-1])])
                cached.clear();cached[key]=(cc,np.r_[direct,reverse,ends])
            return cached[key]
        def inequalities(z):
            cc,errors=evaluate(z)
            lens=z[3:3+n];knots=z[3+n:4+2*n]/100.
            if speed_frontier:
                v=z[-1]
                return np.r_[limits.source_m-.003-errors,
                    limits.acceleration_mps2-knots*v*v,limits.acceleration_mps2+knots*v*v,
                    limits.jerk_mps3-np.diff(knots)/lens*v**3,
                    limits.jerk_mps3+np.diff(knots)/lens*v**3,
                    np.sum(lens)-.9*length,1.1*length-np.sum(lens)]
            return np.r_[z[-1]-errors,(maxdk*lens-np.diff(knots))*100,
                         (maxdk*lens+np.diff(knots))*100,
                         np.sum(lens)-.9*length,1.1*length-np.sum(lens)]
        answer=minimize(lambda z:-z[-1] if speed_frontier else z[-1],z0,method='SLSQP',bounds=bounds,
            constraints=[{'type':'ineq','fun':inequalities}],
            options={'maxiter':maxiter,'ftol':1e-9})
        # Even unsuccessful optimizer output remains a diagnostic, never an
        # unchecked XODR. Exact verification can still recognize a feasible
        # incumbent; optimality is not claimed.
        cc=chain(answer.x,n,origin,h);report=verify(cc,raw,limits)
        report.update(model_kind='free-g2-clothoids',optimizer_success=bool(answer.success),optimizer_message=str(answer.message),
                      optimizer_iterations=int(answer.nit),
                      formulation='max-supported-speed-diagnostic' if speed_frontier else 'fixed-speed-minimax-error',
                      optimizer_phase_value=float(answer.x[-1]),
                      parameters=[list(c.Parameters) for c in cc])
        trials.append(report)
        if report['status']=='PATH_CANDIDATE':
            return cc,{'status':'PATH_CANDIDATE','trials':trials,'limits':limits.__dict__,
                       'scope':'single source path only; no width/graph/road/export approval'}
    return None,{'status':'REJECTED','trials':trials,'limits':limits.__dict__,
                 'scope':'no verified result from tested models/initialization; not global impossibility'}
