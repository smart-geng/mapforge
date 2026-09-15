"""Token-gated loopback server. No arbitrary filesystem or shell endpoints."""
from pathlib import Path
import re
import secrets
import math

import numpy as np

from fastapi import FastAPI, Request
from fastapi.responses import FileResponse, JSONResponse

from .model import Project, Conflict, parse, intervals
from scripts.internal_edge_jets import states

STATIC = Path(__file__).with_name('static')


def selection_geometry(project: Project, data: bytes = None):
    """Presentation-only picking lines, owned by the existing shared-edge handles.

    Sample the actual compiled XML, not a nearest-neighbour match to source lines.
    These display samples never become OpenDRIVE geometry records.
    """
    root = parse(data if data is not None else project.model.compile(project.values())[0])
    roads = {r.get('id'): r for r in root.findall('road')}
    result = {}
    for key, handle in project.model.handles.items():
        road = roads[handle['road']]
        lines = []
        for _, lo, hi in intervals(road):
            a, b = max(lo, handle['knots'][0]), min(hi, handle['knots'][-1])
            if b <= a:
                continue
            ss = np.linspace(a, b, max(2, math.ceil((b-a)/.8)+1))
            lines.append([list(states(road, handle['lane'], float(s), s == hi)[1][:2]) for s in ss])
        result[key] = {'lines': lines, 'ends': [lines[0][0], lines[-1][-1]],
                       'scope': 'existing_shared_boundary_support'}
    return result


def create_app(project: Project, token: str, port: int):
    app = FastAPI(docs_url=None, redoc_url=None, openapi_url=None)
    host = f'127.0.0.1:{port}'
    origin = f'http://{host}'

    def decorated(state):
        state['selection'] = selection_geometry(project)
        return state

    @app.middleware('http')
    async def protect(request: Request, call_next):
        if request.headers.get('host') != host:
            return JSONResponse({'detail':'Host rejected'},status_code=403)
        if request.headers.get('origin') not in (None, origin):
            return JSONResponse({'detail':'Origin rejected'},status_code=403)
        if request.url.path.startswith('/api/'):
            expected = 'Bearer '+token
            if not secrets.compare_digest(request.headers.get('authorization',''),expected):
                return JSONResponse({'detail':'Session token required'},status_code=403)
            if request.method != 'GET' and request.headers.get('x-mapforge-csrf') != token:
                return JSONResponse({'detail':'CSRF token required'},status_code=403)
            if request.method == 'POST':
                if not request.headers.get('content-type','').startswith('application/json'):
                    return JSONResponse({'detail':'JSON only'},status_code=415)
                if int(request.headers.get('content-length','0')) > 8192:
                    return JSONResponse({'detail':'Request too large'},status_code=413)
                if len(await request.body()) > 8192:
                    return JSONResponse({'detail':'Request too large'},status_code=413)
        response = await call_next(request)
        response.headers['Cache-Control'] = 'no-store'
        response.headers['X-Content-Type-Options'] = 'nosniff'
        response.headers['Referrer-Policy'] = 'no-referrer'
        response.headers['Content-Security-Policy'] = "default-src 'self'; script-src 'self'; style-src 'self'; img-src 'self' data:; connect-src 'self'; frame-ancestors 'none'; base-uri 'none'"
        return response

    @app.exception_handler(Conflict)
    async def conflict(request, exc):
        return JSONResponse({'detail':str(exc)},status_code=409)

    @app.exception_handler(ValueError)
    async def invalid(request, exc):
        return JSONResponse({'detail':str(exc)},status_code=422)

    @app.get('/')
    def index():
        return FileResponse(STATIC/'index.html')

    @app.get('/assets/{name}')
    def asset(name: str):
        if name not in ('app.js','style.css'):
            return JSONResponse({'detail':'Not found'},status_code=404)
        return FileResponse(STATIC/name)

    @app.get('/api/project')
    def snapshot():
        with project.lock:
            return {'name':'node4 · SHP 原件叠图', 'state':decorated(project.snapshot()),
                    'source':project.overlay, 'baseline':project.model.baseline_scene,
                    'source_sha256':project.model.base_hash, 'workspace':str(project.directory),
                    'notice':'失败基线的受限修形台；不是已修复地图。口部、拓扑、参考轴、外缘锁定。'}

    @app.post('/api/action/{action}')
    def action(action: str, body: dict):
        with project.lock:
            revision = body.get('revision')
            if action == 'preview':
                if not isinstance(body.get('handle'),str) or isinstance(body.get('value'),bool):
                    raise ValueError('Invalid handle/value')
                result = project.preview(revision, body['handle'],body.get('value'))
                result['selection'] = selection_geometry(project, project.preview_cache[3])
                return result
            if action == 'apply':
                return decorated(project.apply(revision,body.get('preview_id')))
            if action in ('undo','redo'):
                return decorated(project.navigate(revision,-1 if action=='undo' else 1))
            if action == 'save':
                return project.save(revision)
            if action == 'reopen':
                project.check_revision(revision)
                if not project.saved_path.exists():
                    raise ValueError('尚未保存项目')
                return decorated(project.reopen())
            if action == 'export':
                return project.export(revision)
            raise ValueError('Unknown action')

    @app.get('/api/exports/{export_id}/candidate.xodr')
    def download(export_id: str):
        if not re.fullmatch('[a-f0-9]{32}',export_id):
            return JSONResponse({'detail':'Invalid id'},status_code=404)
        path = project.directory/'exports'/export_id/'candidate.xodr'
        if not path.is_file() or not path.with_name('report.json').is_file():
            return JSONResponse({'detail':'Export not completed'},status_code=404)
        return FileResponse(path,media_type='application/xml',filename='node4-repair-CANDIDATE.xodr')

    return app
