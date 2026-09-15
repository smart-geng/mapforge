"""Original-source cross-section evidence for fixed mouth-window anchors.

For a single straight reference, projected source segment intersections are
exact (no nearest-vertex station approximation). Source centers stay centers;
both boundaries retain their transverse order. No raw vertices are edited.
"""
import math
import numpy as np
from spikes.road_boundary_family import ordered_source_boundary
from mapforge.validate.smoothness import lane_edges_kinematics_at


def project_curve(raw,road,project):
    g=road.find('planView/geometry')
    if len(road.findall('planView/geometry'))!=1 or g.find('line') is None:
        raise ValueError('anchor evidence requires a straight frame')
    h=float(g.get('hdg'));d=np.asarray(project(raw))-np.array([float(g.get('x')),float(g.get('y'))])
    st=d@np.array([[math.cos(h),-math.sin(h)],[math.sin(h),math.cos(h)]])
    if st[-1,0]<st[0,0]:st=st[::-1]
    if np.any(np.diff(st[:,0]) < -1e-5):raise ValueError('backtracking raw geometry; cannot sort into a fake curve')
    return st[np.r_[True,np.diff(st[:,0])>1e-8]]


def observations(road,src,project,station):
    sec=max((s for s in road.findall('lanes/laneSection') if float(s.get('s'))<=station),key=lambda e:float(e.get('s')))
    rows=[]
    for side in ('left','right'):
        edges=lane_edges_kinematics_at(road,station,side)
        for i,lane in enumerate(sorted(sec.findall(side+'/lane'),key=lambda e:abs(int(e.get('id'))))):
            ud=lane.find("userData[@code='mapforge.source_lane']")
            if ud is None:continue
            sid=ud.get('value');rec=src.lane(sid)
            candidates=[project_curve(raw,road,project) for raw in src.lane_boundary_geometries(sid)]
            low=ordered_source_boundary(candidates,select_high=False)
            high=ordered_source_boundary(candidates,select_high=True)
            center=project_curve(rec.geometry,road,project)
            target=sorted([edges[i][0],edges[i+1][0]])
            for field,curve,t in [('low',low,target[0]),('high',high,target[1]),('center',center,sum(target)/2)]:
                if not curve[0,0]<=station<=curve[-1,0]:
                    rows.append({'source':sid,'field':field,'supported':False});continue
                value=float(np.interp(station,curve[:,0],curve[:,1]))
                rows.append({'source':sid,'field':field,'supported':True,'source_t':value,'target_t':t,'error_m':abs(value-t)})
    return rows
