"""Headless, no-vehicle esmini diagnostic images bound to one XODR hash.

Rendering is evidence for human inspection, never an automatic acceptance.
The source XODR and formal delivery assets are not modified.
"""
import argparse
import hashlib
import json
import sys
import tempfile
from pathlib import Path

from PIL import Image, ImageDraw

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from scripts.visual_sweep import capture


def run(source,output):
    source=source.resolve();digest=hashlib.sha256(source.read_bytes()).hexdigest()
    output.mkdir(parents=True,exist_ok=True);tiles=[];shots=[]
    cameras=(('whole','0,-50,600,1.5708,1.45'),
             ('junction','0,-20,85,1.5708,1.55'),
             ('oblique','40,-60,50,2.2,0.6'))
    with tempfile.TemporaryDirectory(prefix='mapforge_geometry_review_') as tmp:
        for label,camera in cameras:
            args=['--density','0','--ground_plane','off','--camera_mode','custom_fixed',
                  '--custom_fixed_camera',camera]
            frame=capture(source,args,Path(tmp))
            if frame is None:raise RuntimeError('no screenshot: '+label)
            with Image.open(frame) as raw:
                original=raw.convert('RGB');original.save(output/(label+'.png'))
                tile=original.resize((960,540))
            caption=Image.new('RGB',(960,580),'#202020');caption.paste(tile,(0,40))
            ImageDraw.Draw(caption).text((12,10),'BLOCKED DIAGNOSTIC | '+label+' | SHA '+digest[:16],fill='white')
            tiles.append(caption);shots.append({'label':label,'file':label+'.png','arguments':args})
    sheet=Image.new('RGB',(960*len(tiles),580))
    for i,tile in enumerate(tiles):sheet.paste(tile,(960*i,0))
    sheet.save(output/'esmini-three-views.png')
    if hashlib.sha256(source.read_bytes()).hexdigest()!=digest:raise RuntimeError('XODR changed during capture')
    report={'artifact':str(source),'sha256':digest,'status':'BLOCKED','shots':shots,
            'scope':'render evidence only; no automated visual or controller approval'}
    (output/'esmini-captures.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(output/'esmini-three-views.png')


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('source',type=Path);p.add_argument('output',type=Path)
    a=p.parse_args();run(a.source,a.output)
