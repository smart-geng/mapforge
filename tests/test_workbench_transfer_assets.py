from test_workbench_project_relocation import case
from test_workbench_transfer_api import _client

def test_package_hash_asset_is_served(case):
    with _client(case.store, case.old_raw, case.old_profile) as client:
        index = client.get("/")
        assert index.status_code == 200
        assert '/assets/sha256.js' in index.text
        response = client.get("/assets/sha256.js")
        assert response.status_code == 200
        assert "window.PackageHash" in response.text
