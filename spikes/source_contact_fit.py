"""Source-native shared physical-boundary block (research, no XODR writer).

Degree-two source contacts form long families; branching contacts share C2
constraints. Source record cuts do not introduce spline knots. A Line chart is
a coordinate system, not the old road's geometry truth or accepted crop extent.
"""
from dataclasses import dataclass
from math import comb, factorial

import numpy as np
from scipy.interpolate import BSpline
from scipy.linalg import null_space
from scipy.optimize import linprog

from mapforge.ops.port_dependencies import revision
from mapforge.ops.reconstruction_scope import digest
from mapforge.ops.source_domain import linear_chart
from mapforge.ops.source_roles import require_body, replay_source_role_packet
from mapforge.ops.physical_continuations import compile_physical_continuations
from mapforge.validate.shp_boundary_fidelity import _project
from spikes.road_boundary_family import _convex_qp, world_kinematics


def cubic_bernstein(row, a, b):
    """Exact cubic Bernstein controls on [a,b], no sampling approximation."""
    return np.array([row(a), row(a)+(b-a)*row(a, 1)/3,
                     row(b)-(b-a)*row(b, 1)/3, row(b)])


def polynomial_bernstein(row, a, b, degree=3):
    """Polynomial certificate on an existing span, never a new curve span."""
    if degree == 3:
        return cubic_bernstein(row, a, b)
    if degree != 5 or not b > a:
        raise ValueError('bounded research basis supports cubic/quintic only')
    power = [row(a, j)*(b-a)**j/factorial(j) for j in range(degree+1)]
    return np.array([sum(comb(i,j)/comb(degree,j)*power[j] for j in range(i+1))
                     for i in range(degree+1)])


def exact_difference_max(curve, raw):
    """Max same-chart source deviation, including every linear source piece."""
    worst = 0.
    for (a, va), (b, vb) in zip(raw[:-1], raw[1:]):
        cuts = np.unique(np.r_[a, curve.t[(curve.t > a)&(curve.t < b)], b])
        slope = (vb-va)/(b-a)
        for lo, hi in zip(cuts[:-1], cuts[1:]):
            c = np.array([curve(lo,j)/factorial(j) for j in range(curve.k+1)])
            c[0] -= va+slope*(lo-a); c[1] -= slope
            roots = np.polynomial.polynomial.polyroots(np.arange(1,len(c))*c[1:])
            ss = [0., hi-lo] + [float(r.real) for r in roots if abs(r.imag)<1e-8 and 0<r.real<hi-lo]
            worst = max(worst, float(np.max(abs(np.polynomial.polynomial.polyval(ss, c)))))
    return worst


@dataclass
class SourceFamily:
    features: list
    knots: np.ndarray
    basis: BSpline
    columns: slice


class SourceBoundaryBlock:
    def _source_chart(self, road):
        return linear_chart(road)

    def __init__(self, root, scope, domain, contacts, roles, road='10', min_span=15., source_tol=.35, degree=3):
        for packet in (scope, domain, contacts, roles): require_body(packet)
        if revision(root) != scope['base_revision']:
            raise ValueError('stale baseline for source-native boundary block')
        fresh = replay_source_role_packet(scope, domain, contacts, roles)
        if fresh['content_sha256'] != roles['content_sha256'] or not roles['role_binding_complete']:
            raise ValueError('source role decision has unresolved or altered bindings')
        if not np.isfinite([min_span, source_tol]).all() or min_span < 15 or not 0 < source_tol <= .35:
            raise ValueError('do not relax the long-span/source budget')
        if degree not in (3,5): raise ValueError('only cubic or quintic research basis supported')
        self.degree = degree
        self.reference_curvature = 0.
        if road not in scope['mutable_roads']: raise ValueError('road outside declared mutable scope')
        self.road = road; self.scope = scope; self.roles = roles; self.contacts = contacts
        self.physical_graph = compile_physical_continuations(contacts,roles)
        self.chart = self._source_chart(root.find("road[@id='"+road+"']"))
        self.min_span = min_span; self.source_tol = source_tol
        self.source_ids = sorted(s for s, owners in contacts['source_bindings'].items() if 'road:'+road in owners)
        pr = domain['partition']['projection']; self.features = {}; self.raw = {}; self.source_vertex_indices = {}
        tangent = np.asarray(self.chart['tangent']); normal = np.array([-tangent[1], tangent[0]])
        for key, f in domain['partition']['features'].items():
            if not any(s in self.source_ids for s in f['source_lane_ids']): continue
            if len(f['parts']) != 1: raise ValueError('explicit multipart chart required')
            xy = _project(np.array(f['parts'][0]['raw_vertices']), pr['lat_0'], pr['lon_0'])
            delta = xy-self.chart['origin']; st = np.c_[delta@tangent, delta@normal]
            indices=np.arange(len(st))
            if np.all(np.diff(st[:, 0]) < -1e-9): st = st[::-1];indices=indices[::-1]
            if len(st)<2 or not np.all(np.diff(st[:, 0]) > 1e-9):
                raise ValueError('folded or repeated-station source chart; no sorting or point deletion')
            self.features[key] = f; self.raw[key] = st; self.source_vertex_indices[key] = indices.tolist()
        boundaries = sorted(k for k in self.features if k.startswith('boundary:'))
        parent = {k:k for k in boundaries}
        def find(k):
            while parent[k] != k: k = parent[k]
            return k
        self.endpoint_groups = []
        for group in contacts['contact_groups']:
            members = [contacts['boundary_endpoints'][k] for k in group['members']
                       if contacts['boundary_endpoints'][k]['feature'] in parent]
            if len(members)<2: continue
            xy = np.array([m['xy'] for m in members])
            if np.max(np.linalg.norm(xy-xy[0], axis=1)) > 1e-7:
                raise ValueError('noncoincident source contact requires movable event station model')
            items = [(m['feature'], float((np.asarray(m['xy'])-self.chart['origin'])@tangent)) for m in members]
            if len({k for k, s in items}) != len(items): raise ValueError('self-contact chart not supported')
            self.endpoint_groups.append(items)
            if len(items)==2:
                (ka, sa), (kb, sb) = items
                spans = sorted([self.raw[ka][[0,-1],0], self.raw[kb][[0,-1],0]], key=lambda x:x[0])
                if abs(spans[0][1]-spans[1][0]) > 1e-7:
                    raise ValueError('source continuation overlaps or reverses in chart')
                a,b=find(ka),find(kb);parent[max(a,b)]=min(a,b)
        groups = {}
        for key in boundaries: groups.setdefault(find(key), []).append(key)
        self.families=[];self.owner={}; self.nvar=0
        for members in sorted(groups.values()):
            lo = min(self.raw[k][0,0] for k in members); hi=max(self.raw[k][-1,0] for k in members)
            if hi-lo < min_span: raise ValueError('source family shorter than independent-span budget')
            breaks = np.linspace(lo, hi, max(1,int((hi-lo)/min_span))+1)
            # Degree elevation must not silently demand C4 at internal joins.
            # Multiplicity degree-2 retains exactly C2 on the SAME long spans.
            knots=np.r_[[lo]*(degree+1), np.repeat(breaks[1:-1],degree-2), [hi]*(degree+1)];n=len(knots)-degree-1
            f=SourceFamily(members,knots,BSpline(knots,np.eye(n),degree,extrapolate=False),slice(self.nvar,self.nvar+n))
            for key in members: self.owner[key]=len(self.families)
            self.families.append(f);self.nvar+=n
        self._build_constraints()

    def expression(self, feature, s, derivative=0):
        f=self.families[self.owner[feature]];a,b=f.knots[[0,-1]]
        if s<a-1e-7 or s>b+1e-7: raise ValueError('no extrapolation beyond full source family')
        row=np.zeros(self.nvar);row[f.columns]=f.basis(np.clip(s,a,b),derivative)
        return row

    def source_cells(self, feature, raw, breaks):
        """Exact straight-chart source cells; subclasses enclose curved charts."""
        for (a,va),(b,vb) in zip(raw[:-1],raw[1:]):
            cuts=np.unique(np.r_[a,np.arange(a,b,2.),breaks[(breaks>a)&(breaks<b)],b])
            for lo,hi in zip(cuts[:-1],cuts[1:]):
                targets=np.interp(np.linspace(lo,hi,self.degree+1),[a,b],[va,vb])
                yield lo,hi,targets,0.

    def source_error(self, curve, feature, raw):
        return exact_difference_max(curve, raw)

    def contact_station(self, xy):
        return float((np.asarray(xy)-self.chart['origin'])@self.chart['tangent'])

    def _build_constraints(self):
        A=[];y=[];E=[];C=[];lower=[];labels=[];eq_labels=[]
        self.lane_pairs={};self.uncovered_centers=[];self.movement_observations=[]
        def bound(row, lo, label): C.append(row);lower.append(lo);labels.append(label)
        def tube(rows, targets, label, enclosure_error=0.):
            allowance=self.source_tol-enclosure_error
            if not np.isfinite(allowance) or allowance<=0:
                raise ValueError('source enclosure exhausts total fidelity budget')
            for row,v in zip(rows,targets):
                bound(row,v-allowance,dict(label,bound='lower'))
                bound(-row,-v-allowance,dict(label,bound='upper'))
        def add_source(feature, raw, expression, breaks):
            # Bernstein bounds over every source segment AND spline interval
            # constrain the entire curve, including all original vertices.
            for lo,hi,targets,enclosure_error in self.source_cells(feature,raw,breaks):
                # Cells tighten a certificate only; never add curve variables.
                rows=polynomial_bernstein(expression,lo,hi,self.degree)
                tube(rows,targets,{'kind':'source','feature':feature,'span':[float(lo),float(hi)]},enclosure_error)
                A.extend(rows);y.extend(targets)
        for feature,fi in self.owner.items():
            add_source(feature,self.raw[feature],lambda s,d=0,k=feature:self.expression(k,s,d),self.families[fi].knots)
        for relation in self.physical_graph['relations']:
            p,q=[self.contacts['boundary_endpoints'][k] for k in relation['endpoints']]
            k0,k=p['feature'],q['feature']
            if k0 not in self.owner or k not in self.owner or self.owner[k0]==self.owner[k]:continue
            s0,s=[self.contact_station(e['xy']) for e in (p,q)]
            for der in range(3):
                E.append((self.expression(k,s,der)-self.expression(k0,s0,der))*20**der)
                eq_labels.append({'features':[k0,k],'stations':[s0,s],'derivative':der,
                                  'physical_relation':relation['role']})
        for sid in self.source_ids:
            relations=self.scope['observations'][sid]['boundary_relations']
            byside={r['declared_side']:'boundary:'+r['boundary_key'] for r in relations}
            if len(relations)!=2 or set(byside)!={'left','right'}: raise ValueError('source lane side relation incomplete')
            left,right=byside['left'],byside['right']
            lf,rf=self.families[self.owner[left]],self.families[self.owner[right]]
            raw=self.raw['lane:'+sid]
            # Width applies on the complete common SOURCE boundary support,
            # not the cropped legacy section. Outside is explicitly pending.
            a=max(self.raw[left][0,0],self.raw[right][0,0]);b=min(self.raw[left][-1,0],self.raw[right][-1,0])
            if b<=a:raise ValueError('source boundaries have no common chart support')
            mid=(a+b)/2
            sign=np.sign(np.interp(mid,self.raw[left][:,0],self.raw[left][:,1])-np.interp(mid,self.raw[right][:,0],self.raw[right][:,1]))
            if not sign:raise ValueError('source lane collapses throughout chart')
            cuts=np.unique(np.r_[a,b,lf.knots[(lf.knots>a)&(lf.knots<b)],rf.knots[(rf.knots>a)&(rf.knots<b)]])
            width=lambda s,d=0:sign*(self.expression(left,s,d)-self.expression(right,s,d))
            for lo,hi in zip(cuts[:-1],cuts[1:]):
                for row in polynomial_bernstein(width,lo,hi,self.degree):bound(row,0.,{'kind':'width','source_lane':sid,'span':[float(lo),float(hi)]})
            self.lane_pairs[sid]=(left,right,float(a),float(b),float(sign))
            if self.roles['feature_roles']['lane:'+sid]['role']=='movement_path_observation':
                self.movement_observations.append(sid);continue
            ca=max(raw[0,0],lf.knots[0],rf.knots[0]);cb=min(raw[-1,0],lf.knots[-1],rf.knots[-1])
            if ca>raw[0,0]+1e-8 or cb<raw[-1,0]-1e-8:
                self.uncovered_centers.append({'source_lane':sid,'full_source_s':[float(raw[0,0]),float(raw[-1,0])],
                                               'supported_s':[float(ca),float(cb)],'raw_retained':True})
            if cb<=ca:raise ValueError('source center has no family support')
        # Original record endpoints need not be transverse cuts. Resolve
        # center observations on their source-proven CONTINUOUS path, then
        # account for exterior tails in the SAME Euclidean source budget.
        from mapforge.ops.source_center_support import center_support_plan,source_slice
        self.legacy_uncovered_centers=self.uncovered_centers
        self.center_support=center_support_plan(self)
        self.uncovered_centers=self.center_support['unresolved']
        for d in self.center_support['pieces']:
            left,right=d['left'],d['right']
            lf,rf=[self.families[self.owner[k]] for k in (left,right)]
            center=lambda s,j=0:.5*(self.expression(left,s,j)+self.expression(right,s,j))
            raw=source_slice(self,d['feature'],d['a'],d['b'])
            add_source(d['feature'],raw,center,np.unique(np.r_[lf.knots,rf.knots]))
        for d in self.center_support['endpoint_caps']:
            s=d['host_station'];row=.5*(self.expression(d['left'],s)+self.expression(d['right'],s))
            for witness in d['witnesses']:
                target=witness['transverse_target_m'];allowance=witness['transverse_allowance_m']
                label=dict(kind='source-endpoint-cap',source_lane=d['source_lane'],host_station=s,
                           source_station=witness['source_station'],total_tolerance_m=self.source_tol)
                bound(row,target-allowance,dict(label,bound='lower'))
                bound(-row,-target-allowance,dict(label,bound='upper'))
                A.append(row);y.append(target)
        # Curvature-rate regularization affects the SAME shared variables as
        # source/width/contact constraints. It does not certify dynamics.
        from mapforge.ops.source_transition_domains import source_transition_domains
        self.transition_coverage=source_transition_domains(self)
        for d in self.transition_coverage['intervals']:
            left,right,a,b,sign=[d[k] for k in ('left','right','a','b','sign')]
            lf,rf=[self.families[self.owner[k]] for k in (left,right)]
            cuts=np.unique(np.r_[a,b,lf.knots[(lf.knots>a)&(lf.knots<b)],rf.knots[(rf.knots>a)&(rf.knots<b)]])
            width=lambda s,j=0:sign*(self.expression(left,s,j)-self.expression(right,s,j))
            for lo,hi in zip(cuts[:-1],cuts[1:]):
                for row in polynomial_bernstein(width,lo,hi,self.degree):
                    bound(row,0.,dict(kind='transition-width',source_lanes=d['source_lanes'],span=[float(lo),float(hi)]))
        for f in self.families:
            for a,b in zip(np.unique(f.knots)[:-1],np.unique(f.knots)[1:]):
                # Gauss integrates squared cubic/quintic third derivatives
                # exactly. A single midpoint would miss quintic oscillations.
                nodes,weights=np.polynomial.legendre.leggauss(3)
                for s,w in zip((nodes+1)*(b-a)/2+a,weights*(b-a)/2):
                    row=np.zeros(self.nvar);row[f.columns]=f.basis(s,3)
                    A.append(row*np.sqrt(1e6*w));y.append(0.)
        self.A=np.asarray(A);self.y=np.asarray(y);self.E=np.asarray(E).reshape(-1,self.nvar)
        self.C=np.asarray(C);self.lower=np.asarray(lower);self.labels=labels;self.equality_labels=eq_labels

    def solve(self):
        Z=null_space(self.E) if len(self.E) else np.eye(self.nvar)
        D=self.C@Z
        phase=linprog(np.r_[np.zeros(Z.shape[1]),1.],A_ub=np.c_[-D,-np.ones(len(D))],
                      b_ub=-self.lower,bounds=[(None,None)]*Z.shape[1]+[(0,None)],method='highs')
        if not phase.success: return None,{'status':'ERROR','message':str(phase.message)}
        x=Z@phase.x[:-1];weights=-phase.ineqlin.marginals
        proof={'status':'FEASIBLE' if phase.fun<1e-7 else 'INFEASIBLE',
               'minimum_uniform_constraint_slack_m':float(phase.fun),
               'meaning':'chosen fixed-Line-chart and degree-'+str(self.degree)+' long basis; not global impossibility',
               'dual_support':[dict(self.labels[i],dual_weight=float(weights[i])) for i in np.argsort(-weights) if weights[i]>1e-7]}
        if proof['status']!='FEASIBLE':return None,proof
        H=self.A.T@self.A+np.eye(self.nvar)*1e-10;g=self.A.T@self.y
        candidate,qp=_convex_qp(H,g,self.E,np.zeros(len(self.E)),self.C,self.lower,x)
        proof['optimization']=qp
        if candidate is None:proof['status']='OPTIMIZATION_FAILED'
        return candidate,proof

    def necessary_source_preflight(self):
        """Necessary sampled boundary constraints, WITHOUT center/width rows.

        A rejection here cannot be attributed to conservative Bernstein hulls.
        This is still conditional on the chosen basis/chart/contact stations.
        A pass is not an all-point certificate and is never used to export.
        """
        rows=[];targets=[];labels=[]
        for k in self.owner:
            raw=self.raw[k];ss=np.unique(np.r_[raw[:,0],np.arange(raw[0,0],raw[-1,0],.5)])
            for s in ss:
                rows.append(self.expression(k,s));targets.append(np.interp(s,raw[:,0],raw[:,1]))
                labels.append({'feature':k,'s':float(s)})
        A=np.array(rows);y=np.array(targets);Z=null_space(self.E) if len(self.E) else np.eye(self.nvar)
        D=np.vstack([A,-A])@Z;rhs=np.r_[y+self.source_tol,-y+self.source_tol]
        answer=linprog(np.r_[np.zeros(Z.shape[1]),1.],A_ub=np.c_[D,-np.ones(len(D))],b_ub=rhs,
                       bounds=[(None,None)]*Z.shape[1]+[(0,None)],method='highs')
        if not answer.success:raise ValueError('necessary source diagnostic solver failed')
        weights=-answer.ineqlin.marginals
        return Z@answer.x[:-1],{'status':'FEASIBLE_NECESSARY_ONLY' if answer.fun<1e-7 else 'INFEASIBLE_CHOSEN_BASIS',
            'minimum_additional_source_slack_m':float(answer.fun),'sampled_boundary_points':len(A),
            'includes_width_center_or_dynamics_constraints':False,'all_point_acceptance':False,
            'dual_support':[dict(labels[i%len(A)],dual_weight=float(weights[i])) for i in np.argsort(-weights) if weights[i]>1e-7]}

    def audit(self, x):
        x=np.asarray(x,float)
        if x.shape!=(self.nvar,) or not np.isfinite(x).all():raise ValueError('invalid boundary state')
        curves=[BSpline(f.knots,x[f.columns],self.degree,extrapolate=False) for f in self.families]
        errors={k:self.source_error(curves[fi],k,self.raw[k]) for k,fi in self.owner.items()}
        curvature=[];width_min=[];centers=[]
        for sid,(left,right,a,b,sign) in self.lane_pairs.items():
            lc,rc=curves[self.owner[left]],curves[self.owner[right]]
            cuts=np.unique(np.r_[a,b,lc.t[(lc.t>a)&(lc.t<b)],rc.t[(rc.t>a)&(rc.t<b)]])
            for lo,hi in zip(cuts[:-1],cuts[1:]):
                # True polynomial width minimum (not only Bernstein bound).
                coeff=sign*np.array([(lc(lo,j)-rc(lo,j))/factorial(j) for j in range(self.degree+1)])
                roots=np.polynomial.polynomial.polyroots(np.arange(1,len(coeff))*coeff[1:])
                query=[0.,hi-lo]+[float(r.real) for r in roots if abs(r.imag)<1e-8 and 0<r.real<hi-lo]
                width_min.append(float(min(np.polynomial.polynomial.polyval(query,coeff))))
                ss=np.unique(np.r_[np.nextafter(lo,hi),np.arange(lo,hi,.02),np.nextafter(hi,lo)])
                ss=np.clip(ss,np.nextafter(lo,hi),np.nextafter(hi,lo))
                jets=.5*np.stack([lc(ss,j)+rc(ss,j) for j in range(4)],axis=-1)
                speed=self.scope['observations'][sid].get('source_max_speed_kmh')
                if speed is None or speed<=0:raise ValueError('source speed required; no fallback or lowering')
                values=abs(world_kinematics(jets,self.reference_curvature,0.))*[ (speed/3.6)**2, (speed/3.6)**3]
                for column,name in enumerate(('ay_mps2','jerk_mps3')):
                    i=int(np.argmax(values[:,column]));curvature.append({'source_lane':sid,'metric':name,'value':float(values[i,column]),
                                                                         's':float(ss[i]),'source_speed_kmh':speed})
            raw=self.raw['lane:'+sid];inside=(raw[:,0]>=max(lc.t[0],rc.t[0]))&(raw[:,0]<=min(lc.t[-1],rc.t[-1]))
            ss=raw[inside,0];error=abs(.5*(lc(ss)+rc(ss))-raw[inside,1])
            centers.append({'source_lane':sid,'role':self.roles['feature_roles']['lane:'+sid]['role'],
                            'raw_vertices':len(raw),'evaluated_vertices':len(ss),'vertex_max_m':float(max(error)) if len(error) else None,
                            'complete_geometry_fidelity_validated':False})
        from spikes.shared_boundary_dynamics import audit_transition_bands
        transition_audit=audit_transition_bands(self,x)
        from mapforge.ops.source_center_support import audit_center_support
        center_audit=audit_center_support(self,x,self.center_support)
        curvature.extend(transition_audit['rows'])
        if transition_audit['exact_width_min_m'] is not None:width_min.append(transition_audit['exact_width_min_m'])
        inequality=float(max(0.,np.max(self.lower-self.C@x)));eq=float(np.max(abs(self.E@x))) if len(self.E) else 0.
        return {'status':'BLOCKED','export_allowed':False,
                'scope':'shared north physical-boundary block; incident connectors and final reference not solved',
                'boundary_same_chart_max_m':max(errors.values()),'boundary_errors_m':errors,
                'exact_width_min_m':min(width_min),'inequality_violation':inequality,'scaled_C2_residual':eq,
                'center_observations':centers,
                'full_source_center_support':center_audit,
                'physical_midpoint_dynamics_worst':[max((c for c in curvature if c['metric']==name),key=lambda c:c['value'])
                                                    for name in ('ay_mps2','jerk_mps3')],
                'movement_paths_validated':False,'uncovered_center_support':self.uncovered_centers,
                'transition_band_audit':transition_audit,
                'independent_xodr_validation_ran':False}

    def describe(self):
        return {'road':self.road,'variables':self.nvar,'source_lanes':len(self.source_ids),
                'polynomial_degree':self.degree,'direct_xodr_width_representation':self.degree==3,
                'source_boundary_features':len(self.owner),'long_families':len(self.families),
                'families':[{'features':f.features,'knots':f.knots.tolist(),
                             'column_range':[f.columns.start,f.columns.stop]} for f in self.families],
                'minimum_independent_span_m':min(float(min(np.diff(np.unique(f.knots)))) for f in self.families),
                'chart':self.chart,'source_role_sha256':self.roles['content_sha256'],
                'physical_continuation_sha256':self.physical_graph['content_sha256'],
                'constraints':len(self.C),'C2_contact_equalities':len(self.E),
                'movement_observations':self.movement_observations,
                'uncovered_center_support':self.uncovered_centers,
                'center_support_plan':self.center_support,
                'legacy_unsupported_center_records':self.legacy_uncovered_centers,
                'source_record_cuts_are_knots':False,'certificate_partition_max_m':2.,
                'certificate_partition_adds_geometry_variables':False,'incident_connectors_jointly_solved':False}
