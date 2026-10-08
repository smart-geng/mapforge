"""Run the source workbench without changing the registered conversion CLI."""
import argparse
import json
import os
from pathlib import Path
import secrets
import socket
import webbrowser


def main():
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description="离线地图工程工作台（源工程开发版）")
    parser.add_argument("--source", type=Path, default=root / "shp_0222-0326")
    parser.add_argument("--profile", type=Path, default=root / "profiles/shp/ibd-smarteditor-v1.yaml")
    parser.add_argument("--workspace", type=Path, default=root / "out/workbench/projects")
    parser.add_argument("--port", type=int, default=8766)
    parser.add_argument("--open", action="store_true", help="打开本机默认浏览器")
    parser.add_argument("--surface-review", type=Path, action="append", default=[],
                        help="登记一份完整研究运行用于只读铺面诊断，不接受候选或放行交付")
    args = parser.parse_args()
    if not 1024 <= args.port <= 65535:
        parser.error("port 必须为 1024..65535")
    from .app import create_app
    from .sources import SourceCatalog
    from .store import ProjectStore
    import uvicorn
    print("正在建立只读源索引并核对原件哈希…", flush=True)
    catalog = SourceCatalog(args.source.resolve(), args.profile.resolve())
    store = ProjectStore(args.workspace.resolve())
    token = secrets.token_urlsafe(32)
    # Reserve the port and validate registered evidence before replacing the
    # session entry. A failed launch must not overwrite a live server's token.
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listener:
        if hasattr(socket, "SO_EXCLUSIVEADDRUSE"):
            listener.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
        listener.bind(("127.0.0.1", args.port))
        app = create_app(store, catalog, token, args.port, surface_reviews=args.surface_review)
        url = f"http://127.0.0.1:{args.port}/#token={token}"
        session = args.workspace.resolve().parent / f"session-{args.port}.json"
        session.parent.mkdir(parents=True, exist_ok=True)
        temporary = session.with_name("." + session.name + "." + secrets.token_hex(8) + ".tmp")
        try:
            temporary.write_text(json.dumps({"url": url, "scope": "loopback-only", "port": args.port},
                                            ensure_ascii=False), encoding="utf-8")
            os.replace(temporary, session)
        finally:
            temporary.unlink(missing_ok=True)
        print(url, flush=True)
        if args.open:
            webbrowser.open(url)
        config = uvicorn.Config(app, host="127.0.0.1", port=args.port, access_log=False)
        uvicorn.Server(config).run(sockets=[listener])


if __name__ == "__main__":
    main()
