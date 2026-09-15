"""Original physical boundaries vs written edges around node4's north split.

The common chart comes from the baseline straight planView. Source IDs come
from the raw SHP and are not selected by distance to an output curve.
"""
import argparse
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from lxml import etree

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from scripts.gen_all import shp_source
from mapforge.validate.shp_boundary_fidelity import _origin,_project
from mapforge.validate.smoothness import sample_road_ref,lane_edges_at


def main(before, after, output, after_title='After: source fork + explicit zero width'):
    trees=[etree.parse(str(p)) for p in (before,after)]
    roads=[next(r for r in t.findall('road') if r.get('id')=='10') for t in trees]
    geom=roads[0].find('planView/geometry')
    if geom.find('line') is None or len(roads[0].findall('planView/geometry'))!=1:
        raise ValueError('review chart must be a single known straight axis')
    origin=np.array([float(geom.get('x')),float(geom.get('y'))]); h=float(geom.get('hdg'))
    rotation=np.array([[np.cos(h),-np.sin(h)],[np.sin(h),np.cos(h)]])
    lat,lon=_origin(trees[0].getroot()); source=shp_source()
    def chart(points):return (np.asarray(points)-origin)@rotation
    sids=['2023081117282865661','2023081117282868432','2023081117282871225',
          '2023081117251325687','2023081117251327605','2023081117251329691','2023081117251332825',
          '2023081117193436879','2023081117193439629','2023081117193442066','2023081117193445295']
    fig,axes=plt.subplots(2,1,figsize=(15,8),sharex=True,sharey=True,layout='constrained')
    summaries=[]
    for ax,road,title,color in zip(axes,roads,('Input candidate (not accepted)',after_title),('#ce4b4b','#155bb5')):
        for sid in sids:
            for boundary in source.lane_boundary_geometries(sid):
                b=chart(_project(boundary,lat,lon));ax.plot(b[:,0],b[:,1],color='#373737',lw=2,alpha=.6)
        xy,stations,heads=sample_road_ref(road,.1); normals=np.c_[-np.sin(heads),np.cos(heads)]
        sections=road.findall('lanes/laneSection'); records=[]
        for i,section in enumerate(sections):
            a=float(section.get('s'));b=float(sections[i+1].get('s')) if i+1<len(sections) else float(road.get('length'))
            indices=np.flatnonzero((stations>=a)&(stations<b-1e-7))
            if len(indices)<2:continue
            offsets=np.array([lane_edges_at(road,float(stations[j]),'right') for j in indices])
            for k in range(offsets.shape[1]):
                line=chart(xy[indices]+normals[indices]*offsets[:,k,None])
                ax.plot(line[:,0],line[:,1],color=color,lw=1.6,linestyle='--')
            if 40.<a<50.:
                for lane in section.findall('right/lane'):
                    ud=lane.find("userData[@code='mapforge.source_lane']")
                    records.append({'s':a,'lane':lane.get('id'),'source':ud.get('value'),
                        'start_width_m':float(lane.find('width').get('a')),
                        'predecessor':lane.find('link/predecessor').get('id') if lane.find('link/predecessor') is not None else None})
        summaries.append({'title':title,'written_split':records})
        ax.axvline(41.3849,color='#aa7b00',lw=1);ax.text(42.,-9.,'source Link transition',color='#805a00')
        ax.set(title=title,ylabel='transverse t (m)',xlim=(15,80),ylim=(-10,10));ax.grid(alpha=.2)
        ax.plot([],[],color='#373737',lw=2,label='Original SHP physical boundaries')
        ax.plot([],[],color=color,linestyle='--',label='Written XODR right-side edges');ax.legend(loc='upper left')
    axes[-1].set_xlabel('common reference station (m)')
    fig.suptitle('node4 north split: source identity and boundary shape (not an acceptance PASS)')
    output.parent.mkdir(parents=True,exist_ok=True);fig.savefig(output,dpi=150);plt.close(fig)
    output.with_suffix('.json').write_text(json.dumps({'before':str(before),'after':str(after),
        'source_ids':sids,'comparisons':summaries,'scope':'local raw boundary overlay, not complete-map validation'},indent=2),encoding='utf-8')


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('before',type=Path);p.add_argument('after',type=Path);p.add_argument('output',type=Path)
    a=p.parse_args();main(a.before,a.after,a.output)
