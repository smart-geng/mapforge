"""Whole-connector source-fitted cubic cross sections on a fixed long basis.

Both measured boundaries AND the measured center drive one constrained least
squares system. At most 8 width spans are fixed by long primitives, never by source
sample count. Reference primitives are neither added nor shortened here.
"""
from math import factorial,comb

import numpy as np
from scipy.interpolate import BSpline
from scipy.linalg import null_space
from scipy.optimize import minimize,least_squares


def reference_samples(cls, station):
    cuts=np.r_[0.,np.cumsum([c.length for c in cls])]
    ss=np.asarray(station,float);idx=np.clip(np.searchsorted(cuts,ss,side='right')-1,0,len(cls)-1)
    xy=np.empty((len(ss),2));heading=np.empty(len(ss))
    for i,c in enumerate(cls):
        mask=idx==i;u=np.clip(ss[mask]-cuts[i],0,c.length)
        xy[mask]=np.array([[c.X(float(s)),c.Y(float(s))] for s in u]).reshape(-1,2)
        heading[mask]=c.ThetaStart+c.KappaStart*u+.5*c.dk*u*u
    return xy,np.c_[-np.sin(heading),np.cos(heading)]


def project_observations(cls, points):
    """Initialization correspondence only; actual world errors gate the file."""
    length=sum(c.length for c in cls);ss=np.linspace(0.,length,max(2,int(np.ceil(length/.35))+1))
    xy,_=reference_samples(cls,ss);v=np.diff(xy,axis=0);den=np.sum(v*v,axis=1)
    if min(den)<1e-15:raise ValueError('degenerate reference correspondence')
    points=np.asarray(points,float)
    if points.ndim!=2 or points.shape[1]!=2 or len(points)<2 or not np.isfinite(points).all():
        raise ValueError('finite complete original observations required')
    d=points[:,None,:]-xy[:-1];q=np.clip(np.einsum('nmi,mi->nm',d,v)/den,0,1)
    dist=np.sum((d-q[:,:,None]*v)**2,axis=2);i=np.argmin(dist,axis=1)
    station=ss[i]+q[np.arange(len(points)),i]*np.diff(ss)[i]
    center,normal=reference_samples(cls,station)
    return station,np.sum((points-center)*normal,axis=1)


def fit_source_ribbon(cls, start_jets, end_jets, raw, spans=8):
    """Solve exact C2/end/flat-sharpness constraints, then whole-source LSQ.

    Width positivity and bidirectional world error remain explicit outer
    acceptance checks; an LSQ result alone never approves a connector.
    """
    if spans!=8 or not 3<=len(cls)<=5:raise ValueError('explicit at-most-eight-span representation required')
    if set(raw)!={'left','right','center'}:raise ValueError('both source sides and center required')
    length=float(sum(c.length for c in cls))
    lens=np.array([c.length for c in cls]);ref=np.r_[0.,np.cumsum(lens)]
    # A fixed topological index is essential. Picking the longest core at
    # every evaluation changes the model discontinuously when lengths cross
    # (often at the symmetric seed), destroying optimizer derivatives.
    middle=len(cls)//2
    physical_breaks=np.sort(np.r_[ref,lens[0]/2,length-lens[-1]/2,(ref[middle]+ref[middle+1])/2])
    if not np.isfinite(length) or min(np.diff(physical_breaks))<3.-1e-8:
        raise ValueError('source-fitted width intervals must be at least 3m')
    # Reuse the fixed fairness-family layout: one global freedom beyond the
    # endpoint-determined family. Knots follow structural sharpness changes,
    # not measured vertices, avoiding clustered constraints in a single cubic.
    breaks=physical_breaks/length;knots=np.r_[[0.]*4,breaks[1:-1],[1.]*4]
    n=len(knots)-4;basis=BSpline(knots,np.eye(n),3,extrapolate=False)
    jumps=np.cumsum([c.length for c in cls])[:-1]/length
    E=np.array([basis(s,d) for s in (0.,1.) for d in range(3)]+[basis(s,1) for s in jumps])
    row_norm=np.linalg.norm(E,axis=1);scaled=E/row_norm[:,None]
    Z=null_space(scaled);parts={};targets={}
    for side in ('left','right'):
        rhs=np.r_[np.asarray(start_jets[side])*[1.,length,length**2],
                  np.asarray(end_jets[side])*[1.,length,length**2],np.zeros(len(jumps))]
        parts[side]=np.linalg.lstsq(scaled,rhs/row_norm,rcond=None)[0];targets[side]=rhs
        if max(abs(E@parts[side]-rhs))>1e-7:raise ValueError('inconsistent exact cross-section constraints')
    dim=Z.shape[1];design=[];values=[]
    for field in ('left','right','center'):
        ss,t=project_observations(cls,raw[field]);B=basis(ss/length)
        left_weight,right_weight={'left':(1.,0.),'right':(0.,1.),'center':(.5,.5)}[field]
        A=np.c_[left_weight*(B@Z),right_weight*(B@Z)]
        target=t-B@(left_weight*parts['left']+right_weight*parts['right'])
        w=1./np.sqrt(len(ss));design.append(w*A);values.append(w*target)
    # Weak physical fairness regularization, integrated on fixed intervals.
    nodes,weights=np.polynomial.legendre.leggauss(3)
    for a,b in zip(breaks[:-1],breaks[1:]):
        ss=(a+b)/2+(b-a)*nodes/2
        B=basis(ss,2)/length**2*np.sqrt(1e-3*length*(b-a)/2*weights)[:,None]
        zero=np.zeros((len(ss),dim))
        for side in ('left','right'):
            design.append(np.c_[B@Z,zero] if side=='left' else np.c_[zero,B@Z]);values.append(-B@parts[side])
    beta=np.linalg.lstsq(np.vstack(design),np.concatenate(values),rcond=None)[0] if dim else np.zeros(0)
    coefficients={}
    for side,offset in (('left',0),('right',dim)):
        c=parts[side]+Z@beta[offset:offset+dim]
        residual=(E@c-targets[side])/np.r_[np.tile([1.,length,length**2],2),np.full(len(jumps),length)]
        if max(abs(residual))>1e-9:raise ValueError('physical end constraints inaccurate')
        coefficients[side]=np.array([[basis(s,j)@c/(length**j*factorial(j)) for j in range(4)] for s in breaks[:-1]])
    return physical_breaks,coefficients


def fit_world_ribbon(cls, start_jets, end_jets, raw, diagnostics=None):
    """Source fit with WORLD G2 instead of unnecessarily flat transverse jets.

    At a reference sharpness jump, C1 transverse offsets need
      [t''] = -[k'] * t*t'/(1-k*t).
    Requiring the two edge tangents to agree there makes their arithmetic
    center satisfy the same world-G2 equation. This does NOT require t'=0.
    Exact end jets and the same structural intervals are retained. No extra
    reference/width interval is introduced; C1 knots admit the required t'' jump.
    """
    kn,flat=fit_source_ribbon(cls,start_jets,end_jets,raw)
    L=kn[-1];breaks=kn/L;cuts=np.cumsum([c.length for c in cls])[:-1]/L
    # Use the exact floating representation stored in the knot vector.
    # Recomputing a mathematically equal station can land one ULP to its left
    # and incorrectly erase the one-sided second-derivative jump.
    refs=np.array([breaks[np.argmin(abs(breaks-t))] for t in cuts])
    interior=[]
    for t in breaks[1:-1]:interior.extend([t]*(2 if np.min(abs(refs-t))<1e-12 else 1))
    knots=np.r_[[0.]*4,interior,[1.]*4];n=len(knots)-4
    B=BSpline(knots,np.eye(n),3,extrapolate=False)
    E=np.array([B(t,d) for t in (0.,1.) for d in range(3)])
    norms=np.linalg.norm(E,axis=1);EE=E/norms[:,None];Z=null_space(EE);dim=Z.shape[1]
    particular=[]
    for side in ('left','right'):
        rhs=np.r_[np.asarray(start_jets[side])*[1,L,L**2],np.asarray(end_jets[side])*[1,L,L**2]]
        particular.append(np.linalg.lstsq(EE,rhs/norms,rcond=None)[0])
    # Exact same-curve basis transfer at Greville sites, only an NLP seed.
    sites=np.array([np.mean(knots[i+1:i+4]) for i in range(n)])
    s=sites*L;idx=np.clip(np.searchsorted(kn,s,side='right')-1,0,len(kn)-2);u=s-kn[idx]
    seed=[]
    for i,side in enumerate(('left','right')):
        a,b,c,d=flat[side][idx].T;value=a+u*(b+u*(c+u*d))
        coeff=np.linalg.solve(B(sites),value);seed.extend(Z.T@(coeff-particular[i]))
    matrices=[];targets=[]
    for field,wl,wr in (('left',1.,0.),('right',0.,1.),('center',.5,.5)):
        ss,t=project_observations(cls,raw[field]);basis=B(ss/L);w=1/np.sqrt(len(ss))
        matrices.append(w*np.c_[wl*(basis@Z),wr*(basis@Z)])
        targets.append(w*(t-basis@(wl*particular[0]+wr*particular[1])))
    A=np.vstack(matrices);y=np.concatenate(targets)
    # Fixed Gauss quadrature: weak curvature regularity, not extra data knots.
    nodes,weights=np.polynomial.legendre.leggauss(3)
    for lo,hi in zip(breaks[:-1],breaks[1:]):
        tt=(lo+hi)/2+(hi-lo)*nodes/2
        D=B(tt,2)/L**2*np.sqrt(1e-3*L*(hi-lo)/2*weights)[:,None];zero=np.zeros((3,dim))
        A=np.vstack([A,np.c_[D@Z,zero],np.c_[zero,D@Z]])
        y=np.r_[y,-D@particular[0],-D@particular[1]]
    H=A.T@A;g=A.T@y
    # On each fixed span, widths are cubic and forward factors quartic.
    # Degree elevation to Bernstein degree 8 provides continuous sufficient
    # bounds without adding any geometric knot or fitting unknown.
    q=np.linspace(0.,1.,9)
    bern=np.array([[comb(8,j)*t**j*(1-t)**(8-j) for j in range(9)] for t in q])
    matrices=[];constants=[]
    for lo,hi in zip(breaks[:-1],breaks[1:]):
        ss=(lo+(hi-lo)*q)*L;refidx=min(len(cls)-1,int(np.searchsorted(np.r_[0.,np.cumsum([c.length for c in cls])],(lo+hi)*L/2,side='right')-1))
        c=cls[refidx];start=sum(v.length for v in cls[:refidx]);kk=c.KappaStart+c.dk*(ss-start)
        V=B(ss/L);forward=np.linalg.solve(bern,-kk[:,None]*V);width=np.linalg.solve(bern,V)
        for i in (0,1):
            M=forward@Z;zero=np.zeros_like(M)
            matrices.append(np.c_[M,zero] if i==0 else np.c_[zero,M])
            constants.append(.9+forward@particular[i])
        matrices.append(np.c_[width@Z,-width@Z]);constants.append(width@(particular[0]-particular[1])-.1)
    G=np.vstack(matrices);margin=np.concatenate(constants)
    def regularity(v):return G@v+margin
    B0=B(refs);B1=B(refs,1)/L
    D2=(B(refs,2)-B(np.nextafter(refs,-np.inf),2))/L**2
    k=np.array([c.KappaEnd for c in cls[:-1]]);dj=np.diff([c.dk for c in cls])
    def coefficients(v):return [particular[i]+Z@v[i*dim:(i+1)*dim] for i in (0,1)]
    def constraints(v,jac=False):
        cc=coefficients(v);res=[];jacrows=[];slopes=[];derivatives=[]
        for i,c in enumerate(cc):
            t=B0@c;dt=B1@c;den=1-k*t
            if np.min(abs(den))<1e-10:raise ValueError('singular world-G2 join')
            q=D2@c+dj*t*dt/den
            J=(D2+dj[:,None]*(dt[:,None]/den[:,None]**2*B0+(t/den)[:,None]*B1))@Z
            zero=np.zeros_like(J);res.extend(20*q)
            jacrows.append(20*(np.c_[J,zero] if i==0 else np.c_[zero,J]))
            slopes.append(dt/den)
            derivatives.append((B1/den[:,None]+(k*dt/den**2)[:,None]*B0)@Z)
        res.extend(slopes[0]-slopes[1]);jacrows.append(np.c_[derivatives[0],-derivatives[1]])
        return np.vstack(jacrows) if jac else np.array(res)
    # Whiten the least-squares metric. Raw coefficient scales can make SQP
    # declare convergence after a negligible step with a nonzero tangent
    # gradient, especially on almost constant-width turns.
    eigen,U=np.linalg.eigh(H)
    if eigen[0]<=max(eigen[-1]*1e-12,1e-16):raise ValueError('unobservable source cross-section freedom')
    T=U/np.sqrt(eigen);origin=np.linalg.solve(H,g)
    z0=np.sqrt(eigen)*(U.T@(np.array(seed)-origin))
    result=minimize(lambda z:.5*z@z,z0,jac=lambda z:z,method='SLSQP',
        constraints=[dict(type='eq',fun=lambda z:constraints(origin+T@z),
                          jac=lambda z:constraints(origin+T@z,True)@T),
                     dict(type='ineq',fun=lambda z:regularity(origin+T@z),jac=lambda z:G@T)],
        options=dict(maxiter=80,ftol=1e-12))
    proposed=origin+T@result.x
    # Nearly parallel/constant-width data can make an equality row dependent.
    # A rank-robust least-squares restoration offers another start, but its
    # penalty is NEVER acceptance: every unscaled constraint is checked below.
    restored=least_squares(lambda z:np.r_[z,1e5*constraints(origin+T@z),1e5*np.minimum(regularity(origin+T@z),0.)],result.x,
        jac=lambda z:np.vstack([np.eye(len(z)),1e5*constraints(origin+T@z,True)@T,
                              1e5*(regularity(origin+T@z)<0)[:,None]*(G@T)]),
        max_nfev=80,ftol=1e-10,xtol=1e-10,gtol=1e-10)
    feasible=[v for v in (np.array(seed),proposed,origin+T@restored.x) if max(abs(constraints(v)))<=1e-8]
    if not feasible:raise ValueError('source world-G2 cross-section did not converge')
    fully_feasible=[v for v in feasible if min(regularity(v))>=-1e-8]
    v=min(fully_feasible or feasible,key=lambda q:np.linalg.norm(A@q-y))
    if diagnostics is not None:
        finite_jac=np.column_stack([(constraints(v+np.eye(len(v))[i]*1e-5)-constraints(v-np.eye(len(v))[i]*1e-5))/2e-5 for i in range(len(v))])
        diagnostics.update(success=bool(result.success),message=str(result.message),iterations=int(result.nit),
            dimension=len(v),constraint_rank=int(np.linalg.matrix_rank(constraints(v,True))),
            constraint_count=len(constraints(v)),residual=float(max(abs(constraints(v)))),
            fit_before=float(np.linalg.norm(A@seed-y)),fit_after=float(np.linalg.norm(A@v-y)),
            projected_gradient=float(np.linalg.norm(null_space(constraints(v,True)).T@(H@v-g))),
            jacobian_error=float(np.max(abs(finite_jac-constraints(v,True)))),
            jump_basis_norms=np.linalg.norm(D2,axis=1).tolist(),
            constraint_singular_values=np.linalg.svd(constraints(v,True),compute_uv=False).tolist(),
            bernstein_regularity_margin=float(min(regularity(v))),
            fallback_to_feasible_seed=bool(np.array_equal(v,np.array(seed))))
    if max(abs(constraints(v)))>1e-8:raise ValueError('source world-G2 cross-section did not converge')
    result_co={}
    for side,c in zip(('left','right'),coefficients(v)):
        result_co[side]=np.array([[B(t,j)@c/(L**j*factorial(j)) for j in range(4)] for t in breaks[:-1]])
    return kn,result_co


def minimum_forward_factor(cls,kn,co):
    """Exact quartic extrema of 1-k(s)t(s); positive widths alone miss folds."""
    refs=np.r_[0.,np.cumsum([c.length for c in cls])]
    cuts=np.unique(np.r_[kn,refs]);minimum=float('inf')
    for lo,hi in zip(cuts[:-1],cuts[1:]):
        i=min(len(cls)-1,max(0,int(np.searchsorted(refs,lo,side='right')-1)))
        j=min(len(kn)-2,max(0,int(np.searchsorted(kn,lo,side='right')-1)))
        c=cls[i];k=c.KappaStart+c.dk*(lo-refs[i]);u=lo-kn[j]
        for side in ('left','right'):
            a,b,cc,d=co[side][j]
            t=np.array([a+u*(b+u*(cc+u*d)),b+u*(2*cc+3*d*u),cc+3*d*u,d])
            p=-np.polynomial.polynomial.polymul([k,c.dk],t);p[0]+=1.
            roots=np.polynomial.polynomial.polyroots(np.polynomial.polynomial.polyder(p))
            q=[0.,hi-lo]+[float(r.real) for r in roots if abs(r.imag)<1e-9 and 0<r.real<hi-lo]
            minimum=min(minimum,float(np.min(np.polynomial.polynomial.polyval(q,p))))
    return minimum
