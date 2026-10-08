"""Failed launches cannot replace an existing session entry or open a dead URL."""
import socket
import sys

import pytest

from mapforge.workbench import __main__ as launch
from mapforge.workbench import app, sources, store


@pytest.fixture
def setup(tmp_path, monkeypatch):
    with socket.socket() as reserved:
        reserved.bind(("127.0.0.1", 0))
        port = reserved.getsockname()[1]
    workspace = tmp_path/"projects"
    entry = tmp_path/f"session-{port}.json"
    entry.write_bytes(b"previous server entry")
    opened=[]
    monkeypatch.setattr(sys,"argv",["workbench","--workspace",str(workspace),"--port",str(port),"--open"])
    monkeypatch.setattr(sources,"SourceCatalog",lambda *args:object())
    monkeypatch.setattr(store,"ProjectStore",lambda *args:object())
    monkeypatch.setattr(launch.webbrowser,"open",opened.append)
    return entry,port,opened


def test_invalid_registration_does_not_publish_session(setup, monkeypatch):
    entry,_,opened=setup
    def reject(*args,**kwargs):
        raise ValueError("invalid registered diagnostic")
    monkeypatch.setattr(app,"create_app",reject)
    with pytest.raises(ValueError,match="diagnostic"):
        launch.main()
    assert entry.read_bytes()==b"previous server entry" and opened==[]


def test_busy_port_does_not_publish_session_or_open_browser(setup, monkeypatch):
    entry,port,opened=setup
    with socket.socket() as other:
        other.bind(("127.0.0.1",port)); other.listen(1)
        with pytest.raises(OSError):
            launch.main()
    assert entry.read_bytes()==b"previous server entry" and opened==[]


def test_reserved_socket_is_handed_to_server_and_entry_is_complete(setup, monkeypatch):
    import json
    import uvicorn
    entry,port,opened=setup; calls=[]
    monkeypatch.setattr(app,"create_app",lambda *args,**kwargs:object())
    def run(server,sockets):
        assert len(sockets)==1 and sockets[0].getsockname()==("127.0.0.1",port)
        calls.append(sockets[0].fileno())
        value=json.loads(entry.read_bytes())
        assert value["port"]==port and value["scope"]=="loopback-only"
        assert value["url"]==opened[0]
    monkeypatch.setattr(uvicorn.Server,"run",run)
    launch.main()
    assert len(calls)==1 and not list(entry.parent.glob("*.tmp"))
