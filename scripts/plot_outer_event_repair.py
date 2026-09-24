"""Scientific source/actual-XML comparison; no generated or retouched imagery."""
import json
import argparse
import math
import sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from xml.etree import ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from mapforge.repair_web.model import intervals,lanes,digest
from scripts.internal_edge_jets import states
from pyproj import Transformer


def main():
    parser=argparse.ArgumentParser();parser.add_argument('directory',type=Path)
    directory=parser.parse_args().directory.resolve()
    root=ET.parse(directory/'candidate.xodr').getroot()
    old=ET.parse(ROOT/'out/node4-all-source-tail-readback-v171/node4-review.xodr').getroot()
    packet=json.loads((ROOT/'out/node4-global-model-preflight-20260914/final-input/reconstruction-input.json').read_text(encoding='utf8'))
    scope=json.loads((directory/'scope.json').read_text(encoding='utf8'))
    proj=Transformer.from_crs(4326,root.findtext('header/geoReference'),always_xy=True)
    fig,axes=plt.subplots(3,1,figsize=(15,11),layout='constrained',gridspec_kw={'height_ratios':[1,1,1.4]})
    for ax,rr,title in zip(axes[:2],(old,root),('Before: original SHP + archived XODR','After: same original SHP + final written candidate')):
        road=next(r for r in rr.findall('road') if r.get('id')=='11')
        seen=set()
        for item in scope['boundaries']:
            k=item['key']
            if k in seen:continue
            seen.add(k)
            for rec in packet['boundaries'][k]['records']:
                for part in rec['parts']:
                    xy=np.asarray([proj.transform(*p[:2]) for p in part]);ax.plot(xy[:,0],xy[:,1],color='#dd8e24',lw=2,alpha=.8)
        for sec,lo,hi in intervals(road):
            if hi<95 or lo>207:continue
            for lid in lanes(sec):
                ss=np.linspace(max(95,lo),min(207,hi),max(2,int((hi-lo)/.2)))
                xy=np.array([states(road,lid,s,s==hi) for s in ss])
                for edge in (0,1):ax.plot(xy[:,edge,0],xy[:,edge,1],color='#256dc2',lw=1.2)
        ax.set_xlim(-145,-35);ax.set_ylim(-16,4);ax.set_aspect('equal');ax.set_title(title);ax.grid(alpha=.2)
    for rr,color,name in [(old,'#d05d56','Before outer edge'),(root,'#276fc4','After outer edge')]:
        road=next(r for r in rr.findall('road') if r.get('id')=='11')
        for sec,lo,hi in intervals(road):
            if hi<=100 or lo>=201.7074107:continue
            lid=min(lanes(sec));ss=np.linspace(max(lo,100),min(hi,201.7074107),101)
            k=[states(road,lid,s,i==len(ss)-1)[1][3] for i,s in enumerate(ss)]
            axes[2].plot(ss,k,color=color,label=name)
    handles,labels=axes[2].get_legend_handles_labels();unique=dict(zip(labels,handles))
    axes[2].legend(unique.values(),unique.keys());axes[2].set_xlabel('Reference station s (m)')
    axes[2].set_ylabel('Outer-edge curvature (1/m)');axes[2].grid(alpha=.2)
    fig.suptitle('L01 west split: orange = original SHP; blue = written XODR; G2 joins repaired, DYNAMICS FAIL\n'
                 +'XODR SHA256 '+digest((directory/'candidate.xodr').read_bytes()),fontsize=10)
    fig.savefig(directory/'source-before-after.png',dpi=150)
    plt.close(fig)
    fig,ax=plt.subplots(figsize=(12,10),layout='constrained')
    for rr,color,lw in ((old,'#9a9a9a',.5),(root,'#256dc2',.7)):
        for road in rr.findall('road'):
            if road.get('name')=='junction_paving':continue
            for sec,lo,hi in intervals(road):
                for lid,lane in lanes(sec).items():
                    if lane.get('type')!='driving':continue
                    ss=np.linspace(lo,hi,max(2,math.ceil((hi-lo)/1.)+1))
                    pair=np.array([states(road,lid,s,i==len(ss)-1) for i,s in enumerate(ss)])
                    for edge in (0,1):ax.plot(pair[:,edge,0],pair[:,edge,1],color=color,lw=lw)
    ax.set_aspect('equal');ax.grid(alpha=.2);ax.set_title('Whole written map: gray before / blue candidate\nWest event only; rest unchanged. NOT ACCEPTED.')
    fig.savefig(directory/'whole-map.png',dpi=130);plt.close(fig)


if __name__=='__main__':main()
