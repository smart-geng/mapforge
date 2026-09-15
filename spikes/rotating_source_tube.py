"""Whole raw MAP polyline tubes in a continuously rotating Line chart.

Chart rotation is around the original junction end; source coordinates are
immutable. Long cubic bases and source identities remain fixed. All Bernstein
witness inequalities remain separate, including tied active witnesses;
certificate subdivision never adds a geometric variable. End support is
checked in world distance.
"""
import numpy as np


class RotatingSourceTube:
    def __init__(self, model, source, tolerance=.35,source_widths=None):
        self.model=model;self.tolerance=tolerance;self.widths=source_widths or {}
        g=model.original.find('planView/geometry')
        if len(model.original.findall('planView/geometry'))!=1 or g.find('line') is None:
            raise ValueError('rotating source tube requires a single Line chart')
        self.heading=float(g.get('hdg'));self.length=float(model.original.get('length'))
        self.anchor=np.array([float(g.get('x')),float(g.get('y'))])+self.length*self.tangent(0.)
        self.sources={}
        for (side,si,lid),outer in model.owner.items():
            lane=model.original.findall('lanes/laneSection')[si].find(f"{side}/lane[@id='{lid}']")
            ud=lane.find("userData[@code='mapforge.source_lane']")
            if ud is None:continue
            sid=ud.get('value');ids=model.ids[(side,si)];rank=ids.index(lid)
            inner=model.owner[(side,si,ids[rank-1])] if rank else 0
            if sid not in self.sources:
                raw=np.asarray(source.lane_center_geometry(sid),float)
                if raw.ndim!=2 or raw.shape[1]!=2 or len(raw)<2 or not np.isfinite(raw).all():
                    raise ValueError('missing finite original lane support')
                station=(raw-self.anchor)@self.tangent(0.)
                order=np.arange(len(raw))
                if np.all(np.diff(station)<0):order=order[::-1]
                if np.any(np.diff(station[order])<=1e-7):raise ValueError(f'folded original source {sid}')
                self.sources[sid]={'xy':raw[order].copy(),'original_vertex_indices':order.tolist(),'spans':[]}
            self.sources[sid]['spans'].append((model.starts[si],model.ends[si],inner,outer))
        if not self.sources:raise ValueError('rotating source tube has no raw identities')
        self.labels=[]
        for sid,record in self.sources.items():
            spans=sorted(record['spans'])
            if any(b[0]>a[1]+1e-7 for a,b in zip(spans,spans[1:])):
                raise ValueError(f'disconnected same-identity support {sid}')
            for i in range(len(record['xy'])-1):
                original_segment=min(record['original_vertex_indices'][i:i+2])
                self.labels.extend([dict(kind='rotating-source',source=sid,segment=original_segment,sign=sign) for sign in (1,-1)])
            self.labels.extend([dict(kind='source-domain',source=sid,vertex=i) for i in record['original_vertex_indices']])

    def tangent(self,angle):
        h=self.heading+angle
        return np.array([np.cos(h),np.sin(h)])

    def row(self,span,s,der=0,width=False):
        m=self.model;r=np.zeros(np.shape(s)+(m.nvar,))
        sign=1 if m.families[span[3]].side=='left' else -1
        weights=(-sign,sign) if width else (.5,.5)
        for fi,weight in zip(span[2:],weights):
            f=m.families[fi];r[...,f.columns]+=weight*f.basis(s,nu=der)
        return r

    def evaluate(self,x,angle,jac=False):
        if not np.isfinite(angle) or not np.isfinite(x).all():raise ValueError('finite rotating source state required')
        m=self.model;e=self.tangent(angle);n=np.array([-e[1],e[0]])
        values=[];rows=[];angle_rows=[];labels=[]
        for sid,record in self.sources.items():
            xy=record['xy'];delta=xy-self.anchor
            st=np.c_[self.length+delta@e,delta@n]
            if np.any(np.diff(st[:,0])<=1e-7):
                raise ValueError(f'axis folds original source {sid}')
            spans=record['spans'];station_derivative=delta@n;transverse_derivative=-delta@e
            for segment,((sa,ta),(sb,tb)) in enumerate(zip(st[:-1],st[1:])):
                covered=False
                dsa,dsb=station_derivative[segment:segment+2];dta,dtb=transverse_derivative[segment:segment+2]
                slope=(tb-ta)/(sb-sa);dslope=((dtb-dta)*(sb-sa)-(tb-ta)*(dsb-dsa))/(sb-sa)**2
                for span in spans:
                    lo=max(sa,span[0]);hi=min(sb,span[1])
                    if hi<lo:continue
                    covered=True
                    # Fixed certificate cells, not a source-dependent moving
                    # grid. Only the two clipped raw-segment ends move.
                    knots=np.unique(np.r_[span[0],span[1],np.arange(span[0],span[1],2.),
                        np.concatenate([m.families[k].knots for k in span[2:]])])
                    cuts=np.unique(np.r_[lo,hi,knots[(knots>lo)&(knots<hi)]])
                    if len(cuts)==1:cuts=np.repeat(cuts,2)
                    dc=np.zeros(len(cuts));dc[0]=dsa if sa>=span[0] else 0.;dc[-1]=dsb if sb<=span[1] else 0.
                    a,b=cuts[:-1],cuts[1:];da,db=dc[:-1],dc[1:];length=b-a;dl=db-da
                    p,q=self.row(span,a),self.row(span,b);dp,dq=self.row(span,a,1),self.row(span,b,1)
                    bs=np.stack([p,p+length[:,None]*dp/3,q-length[:,None]*dq/3,q],axis=1).reshape(-1,m.nvar)
                    stations=np.linspace(a,b,4,axis=1);dstation=np.linspace(da,db,4,axis=1)
                    target=ta+(stations-sa)*slope
                    err=bs@x-target.ravel()
                    values.extend(err/self.tolerance-1);values.extend(-err/self.tolerance-1)
                    base_labels=[dict(kind='rotating-source',source=sid,segment=min(record['original_vertex_indices'][segment:segment+2]),
                                      interval=[float(a[i]),float(b[i])],witness=j)
                                 for i in range(len(a)) for j in range(4)]
                    labels.extend([dict(l,sign=sign) for sign in (1,-1) for l in base_labels])
                    if jac:
                        ca,cb=dp@x,dq@x;dca=self.row(span,a,2)@x;dcb=self.row(span,b,2)@x
                        dcurve=np.stack([ca*da,ca*da+dl*ca/3+length*dca*da/3,
                                         cb*db-dl*cb/3-length*dcb*db/3,cb*db],axis=1)
                        dtarget=dta+(stations-sa)*dslope+(dstation-dsa)*slope
                        de=(dcurve-dtarget).ravel()/self.tolerance
                        rows.extend(bs/self.tolerance);rows.extend(-bs/self.tolerance)
                        angle_rows.extend(de);angle_rows.extend(-de)
                    if self.widths.get(sid) is not None:
                        wp,wq=self.row(span,a,width=True),self.row(span,b,width=True)
                        wdp,wdq=self.row(span,a,1,True),self.row(span,b,1,True)
                        wb=np.stack([wp,wp+length[:,None]*wdp/3,wq-length[:,None]*wdq/3,wq],axis=1).reshape(-1,m.nvar)
                        difference=wb@x-float(self.widths[sid])
                        values.extend((difference-1e-6)/self.tolerance);values.extend((-difference-1e-6)/self.tolerance)
                        labels.extend([dict(l,kind='explicit-source-width',sign=sign) for sign in (1,-1) for l in base_labels])
                        if jac:
                            ca,cb=wdp@x,wdq@x;dca=self.row(span,a,2,True)@x;dcb=self.row(span,b,2,True)@x
                            dw=np.stack([ca*da,ca*da+dl*ca/3+length*dca*da/3,
                                         cb*db-dl*cb/3-length*dcb*db/3,cb*db],axis=1).ravel()/self.tolerance
                            rows.extend(wb/self.tolerance);rows.extend(-wb/self.tolerance)
                            angle_rows.extend(dw);angle_rows.extend(-dw)
                if not covered:
                    # A whole raw segment outside support is covered by the
                    # endpoint checks below only if it lies before/after the
                    # entire same-identity chain, never in an internal hole.
                    if sa<max(s[1] for s in spans) and sb>min(s[0] for s in spans):
                        raise ValueError(f'uncovered internal source interval {sid}')
            for index,point in enumerate(st):
                span=min(spans,key=lambda v:max(v[0]-point[0],0.,point[0]-v[1]))
                s=float(np.clip(point[0],span[0],span[1]));r=self.row(span,s)
                # All vertices use a continuous world-distance expression,
                # even at a chart end. Never switch from inactive -1 to an
                # already nonzero residual at an end crossing.
                transverse=r@x-point[1];distance=np.hypot(s-point[0],transverse)
                values.append(distance/self.tolerance-1.)
                labels.append(dict(kind='source-domain',source=sid,vertex=record['original_vertex_indices'][index]))
                if jac:
                    ds=station_derivative[index] if span[0]<point[0]<span[1] else 0.
                    dt=self.row(span,s,1)@x*ds-transverse_derivative[index]
                    denom=max(distance,1e-15)*self.tolerance
                    rows.append(r*transverse/denom)
                    angle_rows.append(((s-point[0])*(ds-station_derivative[index])+transverse*dt)/denom)
        v=np.asarray(values)
        self.labels=labels
        if not jac:return v
        return v,np.asarray(rows),np.asarray(angle_rows)
