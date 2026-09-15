"""Run the registered node4 local repair slice; never expose a public listener."""
import argparse
import secrets
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))

from mapforge.repair_web.model import Project, atomic, json_bytes
from mapforge.repair_web.app import create_app


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port',type=int,default=8765)
    parser.add_argument('--project',type=Path,default=ROOT/'out/node4-local-repair-20260915/project')
    args = parser.parse_args()
    if not 1024 <= args.port <= 65535:
        parser.error('port must be 1024..65535')
    directory = args.project.resolve()
    if not directory.is_relative_to(ROOT/'out'):
        parser.error('project directory must be below workspace out/')
    project = Project(ROOT/'out/node4-all-source-tail-readback-v171/node4-review.xodr',
                      ROOT/'out/node4-global-model-preflight-20260914/final-input/reconstruction-input.json',
                      ROOT/'out/node4-global-model-preflight-20260914/final-input/run.json',directory)
    token = secrets.token_urlsafe(32)
    url = f'http://127.0.0.1:{args.port}/#token={token}'
    atomic(directory/'session.json',json_bytes({'url':url,'scope':'loopback-only','port':args.port}))
    print(url,flush=True)
    import uvicorn
    uvicorn.run(create_app(project,token,args.port),host='127.0.0.1',port=args.port,access_log=False)


if __name__ == '__main__':
    main()
