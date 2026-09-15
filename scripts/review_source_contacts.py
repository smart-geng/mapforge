"""Scientific source-evidence figure; not an XODR rendering or an acceptance."""
import argparse
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]; sys.path.insert(0,str(ROOT))
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from mapforge.ops.reconstruction_scope import digest
from mapforge.validate.shp_boundary_fidelity import _project


def render(directory, output):
    directory,output=Path(directory),Path(output)
    if output.exists(): raise FileExistsError('preserve previous evidence figure')
    model=json.loads((directory/'contact-model.json').read_text(encoding='utf-8'))
    domain=json.loads((directory/'input/source-domain.json').read_text(encoding='utf-8'))
    for packet in (model,domain):
        if digest({k:v for k,v in packet.items() if k!='content_sha256'})!=packet['content_sha256']:
            raise ValueError('source evidence content changed')
    if model['source_domain_sha256']!=domain['content_sha256']: raise ValueError('source evidence binding mismatch')
    pr=domain['partition']['projection']; lines=[]
    for key,f in domain['partition']['features'].items():
        if not any('road:10' in model['source_bindings'].get(s,[]) for s in f['source_lane_ids']):continue
        for p in f['parts']:
            lines.append((_project(np.array(p['raw_vertices']),pr['lat_0'],pr['lon_0']),f['kind']))
    conflicts=model['role_conflicts']
    fig,axes=plt.subplots(1,len(conflicts)+1,figsize=(5*(len(conflicts)+1),7),squeeze=False)
    axes=axes[0]
    for ax in axes:
        for xy,kind in lines:
            ax.plot(xy[:,0],xy[:,1],color='#2471a3' if kind=='physical_boundary' else '#d17b00',
                    ls='-' if kind=='physical_boundary' else '--',lw=1,alpha=.7)
        ax.set_aspect('equal');ax.grid(alpha=.15);ax.set_xlabel('local x (m)');ax.set_ylabel('local y (m)')
    axes[0].set_title('Complete original north-side supports\nNOT a fitted road',fontsize=11)
    for ax,c in zip(axes[1:],conflicts):
        path=np.array(c['source_path_xy']); boundary=np.array(c['collapsed_boundary_xy']); midpoint=(path+boundary)/2
        radius=model['policy']['source_error_budget_m']
        ax.add_patch(Circle(path,radius,color='#d17b00',alpha=.25))
        ax.add_patch(Circle(boundary,radius,color='#2471a3',alpha=.25))
        ax.plot(*path,marker='o',color='#d17b00',markersize=7,label='Original lane-path tip')
        ax.plot(*boundary,marker='x',color='#2471a3',markersize=9,mew=2,label='Original zero-width boundary tip')
        ax.annotate('',xy=boundary,xytext=path,arrowprops={'arrowstyle':'<->','color':'#bb2958'})
        ax.text(midpoint[0],midpoint[1]+.2,'%.3f m'%c['gap_m'],ha='center',color='#bb2958')
        ax.set_xlim(midpoint[0]-3.7,midpoint[0]+3.7);ax.set_ylim(midpoint[1]-3.7,midpoint[1]+3.7)
        ax.set_title('Original lane '+c['source_lane_id']+'\nDeclared start width = 0',fontsize=10)
        axes[0].plot(*midpoint,'o',color='#bb2958',markersize=6)
    if len(conflicts):
        handles,labels=axes[1].get_legend_handles_labels();fig.legend(handles,labels,loc='lower center',ncol=2)
    fig.suptitle('Source role conflict / BLOCKED / No new XODR\n'
                 'Circles: 0.35 m source bounds, conditional on BOTH observations representing the SAME physical endpoint',fontsize=11)
    fig.tight_layout(rect=(0,.07,1,.9));output.parent.mkdir(parents=True,exist_ok=True)
    fig.savefig(output,dpi=150);plt.close(fig)
    return {'output':str(output.resolve()),'geometry_solver_ran':False,'source_contact_model_sha256':model['content_sha256']}


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('directory');p.add_argument('output');a=p.parse_args()
    print(json.dumps(render(a.directory,a.output),indent=2))
