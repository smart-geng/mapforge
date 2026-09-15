"""Source-native cubic boundaries on ONE exact Line/Arc reference primitive.

All physical boundary identities/contact roles are inherited from reviewed
source inputs. A shared global knot layout prevents independently phased
boundary knots from creating tiny width intervals. This block alone is NOT
a whole-road compiler: source-end support and incident connectors remain gates.
"""
import copy
from math import factorial

import numpy as np
from scipy.interpolate import BSpline
from scipy.linalg import null_space
from scipy.optimize import linprog

from mapforge.ops.arc_source_chart import ArcChart, ArcSourceTrace
from spikes.source_contact_fit import SourceBoundaryBlock, SourceFamily, exact_difference_max, polynomial_bernstein


class ArcSourceBoundaryBlock(SourceBoundaryBlock):
    def __init__(self, original, heading_delta=0., curvature=0., structural_stations=(),
                 *, origin_delta=(0., 0.), fixed_layout=None, knot_displacements=None):
        if original.degree != 3 or getattr(original, 'reference_curvature', 0.) != 0:
            raise ValueError('construct from original source-native cubic Line block, not fitted geometry')
        if not np.isfinite([heading_delta, curvature]).all() or abs(heading_delta) > .1 or abs(curvature) > .004:
            raise ValueError('outside declared research chart search box')
        origin_delta = np.asarray(origin_delta, float)
        if origin_delta.shape != (2,) or not np.isfinite(origin_delta).all() or np.max(abs(origin_delta)) > 2.:
            raise ValueError('finite reference-origin displacement in declared +/-2m research box required')
        if fixed_layout is None and knot_displacements is not None:
            raise ValueError('movable knots require a predeclared fixed discrete layout')
        if fixed_layout is not None and len(structural_stations):
            raise ValueError('cannot add structural stations inside a fixed-layout evaluation')
        self.__dict__.update(copy.deepcopy(original.__dict__))
        e = np.array(original.chart['tangent']); n = np.array([-e[1], e[0]])
        self.axis = ArcChart(tuple(np.asarray(original.chart['origin'])+origin_delta), float(np.arctan2(e[1], e[0])+heading_delta), float(curvature))
        self.reference_curvature = float(curvature); self.heading_delta = float(heading_delta)
        # Reversed raw source order can be a strided view; deepcopy makes it
        # contiguous. Use the same arithmetic/layout in both cases so a
        # coordinate trial cannot acquire 1-ulp source drift from BLAS paths.
        self.source_xy = {k:np.asarray(original.chart['origin'])+np.ascontiguousarray(r)@np.stack([e,n])
                          for k,r in original.raw.items()}
        self.traces = {k:ArcSourceTrace(self.axis, xy) for k,xy in self.source_xy.items()}
        self.raw = {k:t.st.copy() for k,t in self.traces.items()}
        tangent = np.array([np.cos(self.axis.heading), np.sin(self.axis.heading)])
        self.origin_delta = origin_delta.tolist()
        self.chart = dict(origin=list(self.axis.origin), tangent=tangent.tolist(),
                          length=original.chart['length'], curvature=curvature)
        self.endpoint_groups = []
        for group in original.endpoint_groups:
            new = []
            for key, station in group:
                old = original.raw[key]
                i = int(np.argmin(abs(old[:,0]-station)))
                if i not in (0,len(old)-1) or abs(old[i,0]-station)>1e-7:
                    raise ValueError('source contact must refer to an original endpoint')
                new.append((key, float(self.raw[key][i,0])))
            if np.ptp([s for _,s in new])>1e-7:
                raise ValueError('physical source contact changed under exact re-projection')
            self.endpoint_groups.append(new)
        if fixed_layout is not None:
            self.export_structural_stations = []
            self.global_breaks, cuts = fixed_layout.rebuild(self, knot_displacements)
            self._install_families(original, cuts)
            self._build_constraints()
            self.fixed_layout = fixed_layout
            return
        events=np.unique([g[0][1] for g in self.endpoint_groups if len(g)>2])
        a=min(r[0,0] for k,r in self.raw.items() if k in self.owner)
        b=max(r[-1,0] for k,r in self.raw.items() if k in self.owner)
        extra=np.asarray(structural_stations,float)
        if not np.isfinite(extra).all() or np.any((extra<=a)|(extra>=b)):
            raise ValueError('structural stations must lie inside complete source support')
        required=np.unique(np.r_[a,events,extra,b])
        self.export_structural_stations=extra.tolist()
        if np.min(np.diff(required))<self.min_span-1e-7:
            raise ValueError('events require a sub-budget independent span; no short fallback')
        self.global_breaks=np.unique(np.concatenate([np.linspace(lo,hi,max(1,int((hi-lo)/self.min_span))+1)
                                                     for lo,hi in zip(required[:-1],required[1:])]))
        # Oblique source end cuts differ slightly between lane boundaries.
        # Remove a redundant GLOBAL interior knot when a clipped end would
        # be short. Never insert a per-family knot or move a source event.
        domains=[(min(self.raw[k][0,0] for k in f.features),max(self.raw[k][-1,0] for k in f.features))
                 for f in original.families]
        def family_cuts(lo,hi):
            interior=self.global_breaks[(self.global_breaks>lo+1e-7)&(self.global_breaks<hi-1e-7)]
            # A direction's transverse export start can lie centimetres
            # inside an oblique boundary cap. It is a section event, not an
            # extra degree of freedom in that cap's existing cubic. The
            # family continues through it with the SAME polynomial.
            interior=np.array([v for v in interior if not (
                len(extra) and np.min(abs(extra-v))<1e-7 and
                min(v-lo,hi-v)<self.min_span-1e-7)])
            return np.r_[lo,interior,hi]
        while True:
            remove=None
            for lo,hi in domains:
                cuts=family_cuts(lo,hi)
                short=np.flatnonzero(np.diff(cuts)<self.min_span-1e-7)
                if not len(short):continue
                i=int(short[0])
                candidates=[v for v in cuts[i:i+2] if lo+1e-7<v<hi-1e-7 and np.min(abs(required-v))>1e-7]
                if not candidates:
                    raise ValueError('required source events/support leave a short independent span')
                remove=candidates[0];break
            if remove is None:break
            self.global_breaks=self.global_breaks[abs(self.global_breaks-remove)>1e-7]
        cuts = []
        for old in original.families:
            lo=min(self.raw[k][0,0] for k in old.features);hi=max(self.raw[k][-1,0] for k in old.features)
            breaks=family_cuts(lo,hi)
            if np.min(np.diff(breaks))<self.min_span-1e-7:
                raise ValueError('source support clips a long span below its budget; no silent extrapolation')
            cuts.append(breaks)
        self._install_families(original, cuts)
        self._build_constraints()

    def _install_families(self, original, cuts):
        self.families=[];self.nvar=0
        for old, breaks in zip(original.families, cuts):
            lo,hi=breaks[0],breaks[-1]
            knots=np.r_[[lo]*4,breaks[1:-1],[hi]*4];size=len(knots)-4
            self.families.append(SourceFamily(old.features,knots,BSpline(knots,np.eye(size),3,extrapolate=False),
                                               slice(self.nvar,self.nvar+size)))
            self.nvar+=size

    def contact_station(self, xy):
        return float(self.axis.project(np.asarray(xy))[0])

    def source_cells(self, feature, raw, breaks):
        for lo,hi,va,vb,error in self.traces[feature].cells(raw[0,0],raw[-1,0],breaks):
            yield lo,hi,np.linspace(va,vb,4),error

    def source_error(self, curve, feature, raw):
        worst=0.
        for lo,hi,va,vb,error in self.traces[feature].cells(raw[0,0],raw[-1,0],curve.t):
            exact=exact_difference_max(curve,np.array([[lo,va],[hi,vb]]))
            worst=max(worst,exact+error)
        return float(worst)

    def _build_constraints(self):
        super()._build_constraints()
        # Every exported offset must remain on the regular side of the arc,
        # across complete spans, not just at sampled lane centers.
        rows=[];lower=[];labels=[]
        if self.reference_curvature==0:return
        for f in self.families:
            key=f.features[0]
            for a,b in zip(np.unique(f.knots)[:-1],np.unique(f.knots)[1:]):
                for row in polynomial_bernstein(lambda s,d=0:self.expression(key,s,d),a,b,3):
                    # Keep the source-model slack in metres, not a mixture of
                    # transverse metres and dimensionless chart regularity.
                    rows.append(-np.sign(self.reference_curvature)*row);lower.append(-.8/abs(self.reference_curvature))
                    labels.append(dict(kind='regular-reference-chart',features=f.features,span=[float(a),float(b)]))
        self.C=np.vstack([self.C,rows]);self.lower=np.r_[self.lower,lower];self.labels.extend(labels)

    def necessary_source_preflight(self):
        raise ValueError('Line-only source relaxation is not valid in an Arc chart')

    def original_vertex_preflight(self):
        """Necessary original-boundary vertices/C2 only, no enclosure hulls.

        Diagnostic coefficients can violate source/width/dynamics and must
        never go to coefficients()/production writing. A conflict here is
        independent of conservative source-cell remainder bounds, but is
        still conditional on this reference/basis/event layout.
        """
        rows=[];targets=[];labels=[]
        for key in self.owner:
            for i,(s,t) in enumerate(self.raw[key]):
                rows.append(self.expression(key,s));targets.append(t)
                labels.append(dict(feature=key,original_vertex_index=self.source_vertex_indices[key][i],station_m=float(s)))
        return self._sampled_phase(rows,targets,labels)

    def sampled_source_preflight(self, step=.5):
        """Necessary exact source-point/width constraints, without hull bounds.

        Unlike vertex-only checking, includes original straight segments and
        physical-center observations, and sampled widths. No dynamics; a
        positive slack is conditional on this cubic/reference/contact model.
        """
        if not np.isfinite(step) or not 0<step<=1:raise ValueError('source witness step must be in (0,1]')
        rows=[];targets=[];labels=[];width_rows=[]
        def observe(feature,a,b,expr):
            for lo,hi,va,vb,_ in self.traces[feature].cells(a,b,[],step):
                for s,t in ((lo,va),(hi,vb)):
                    rows.append(expr(s));targets.append(t);labels.append(dict(feature=feature,station_m=s))
        for key in self.owner:
            raw=self.raw[key];observe(key,raw[0,0],raw[-1,0],lambda s,key=key:self.expression(key,s))
        for sid,(left,right,a,b,sign) in self.lane_pairs.items():
            for s in np.r_[np.arange(a,b,step),b]:
                width_rows.append(sign*(self.expression(left,s)-self.expression(right,s)))
            if sid in self.movement_observations:continue
            raw=self.raw['lane:'+sid];lf,rf=[self.families[self.owner[k]] for k in (left,right)]
            ca=max(raw[0,0],lf.knots[0],rf.knots[0]);cb=min(raw[-1,0],lf.knots[-1],rf.knots[-1])
            observe('lane:'+sid,ca,cb,lambda s,l=left,r=right:.5*(self.expression(l,s)+self.expression(r,s)))
        x,report=self._sampled_phase(rows,targets,labels,width_rows)
        report.update(status='CONDITIONAL_SAMPLED_CONFLICT' if report['additional_source_slack_m']>1e-7 else 'NECESSARY_SAMPLES_ONLY',
                      source_witness_step_m=step,physical_center_observations_included=True,
                      original_boundary_and_center_witnesses=len(rows),width_witnesses=len(width_rows),dynamics_included=False)
        report.pop('source_boundary_vertices')
        return x,report

    def _sampled_phase(self,rows,targets,labels,width_rows=()):
        A=np.array(rows);y=np.array(targets);Z=null_space(self.E) if len(self.E) else np.eye(self.nvar)
        D=np.vstack([A,-A])@Z;rhs=np.r_[y+self.source_tol,-y+self.source_tol]
        full_labels=[dict(l,sign=sign) for sign in (1,-1) for l in labels]
        if len(width_rows):
            D=np.vstack([D,-np.asarray(width_rows)@Z]);rhs=np.r_[rhs,np.zeros(len(width_rows))]
            full_labels.extend(dict(kind='width-witness',index=i) for i in range(len(width_rows)))
        matrix=np.c_[D,-np.ones(len(D))]
        answer=linprog(np.r_[np.zeros(Z.shape[1]),1.],A_ub=matrix,b_ub=rhs,
                       bounds=[(None,None)]*Z.shape[1]+[(0,None)],method='highs')
        if not answer.success or answer.x is None or not np.isfinite(answer.x).all():
            raise ValueError('original-vertex necessary LP did not return a finite solution')
        x=Z@answer.x[:-1];slack=float(answer.x[-1]);weights=-answer.ineqlin.marginals
        primal=float(max(0.,np.max(matrix@answer.x-rhs),
                         np.max(abs(self.E@x),initial=0.)))
        dual=float(np.max(abs(D.T@weights),initial=0.))
        gap=float(abs(answer.fun-rhs@answer.ineqlin.marginals))
        if max(primal,dual,gap)>1e-6 or not np.isfinite([primal,dual,gap]).all():
            raise ValueError('original-vertex LP residuals not validated')
        return x,dict(status='CONDITIONAL_VERTEX_CONFLICT' if slack>1e-7 else 'NECESSARY_VERTICES_ONLY',
                      additional_source_slack_m=slack,primal_residual=primal,dual_residual=dual,duality_gap=gap,
                      uses_source_interpolation_enclosure=False,source_boundary_vertices=len(rows),
                      export_allowed=False,global_impossibility_proven=False,
                      dual_support=[dict(full_labels[i],weight=float(weights[i]))
                                    for i in np.argsort(-weights) if weights[i]>1e-7])

    def coefficients(self, x):
        """Exact long cubic powers; no approximation or point-based writing.

        A future complete source-domain compiler may difference adjacent
        entries into XODR width. This table is not a routable road/map.
        """
        x=np.asarray(x,float)
        if x.shape!=(self.nvar,) or not np.isfinite(x).all():
            raise ValueError('finite correctly sized shared boundary coefficients required')
        if max(np.max(self.lower-self.C@x),np.max(abs(self.E@x),initial=0.))>1e-6:
            raise ValueError('do not compile a rejected source/contact/width state')
        result=[]
        for i,f in enumerate(self.families):
            curve=BSpline(f.knots,x[f.columns],3)
            for a,b in zip(np.unique(f.knots)[:-1],np.unique(f.knots)[1:]):
                result.append(dict(family=i,features=f.features,s=float(a),length=float(b-a),
                                   abcd=[float(curve(a,j)/factorial(j)) for j in range(4)]))
        return result

    def audit(self, x):
        report=super().audit(x)
        report.update(scope='source-native cubic Arc block; not complete road/incident connectors',
                      source_error_metric='whole original segment same-normal enclosure, Euclidean upper bound',
                      source_error_is_upper_bound=True,reference_curvature=self.reference_curvature,
                      xodr_generated=False,independent_xodr_validation_ran=False)
        return report

    def describe(self):
        result=super().describe()
        result.update(reference_primitive='line' if self.reference_curvature==0 else 'arc',
                      reference_primitive_count=1,heading_delta_rad=self.heading_delta,
                      reference_curvature=self.reference_curvature,global_breaks_m=self.global_breaks.tolist(),
                      boundary_powers_exactly_compilable=True,whole_road_compiler_complete=False)
        return result
