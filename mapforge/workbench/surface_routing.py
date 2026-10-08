"""Select a server-owned implementation for the one registered SHP source.

Routing is not capability authorization: each selected service independently
checks the complete source identity, current implementation and project state.
"""
from __future__ import annotations


JUNCTION_ID = "2023062110304177600"


def is_surface_project(project):
    return (isinstance(project, dict)
            and isinstance(project.get("source_snapshot"), dict)
            and project["source_snapshot"].get("junction_id") == JUNCTION_ID)


def validation_module(project):
    if is_surface_project(project):
        from . import surface_validation
        return surface_validation
    from . import validation
    return validation


class CheckingRouter:
    """Keep job proofs in their owning checking service across HTTP requests."""

    def __init__(self, store, jobs, *, source_dir, profile_path):
        from .checking import CheckingService
        from .surface_checking import SurfaceCheckingService
        self.store = store
        paths = {"source_dir": source_dir, "profile_path": profile_path}
        self.boundary = CheckingService(store, jobs, **paths)
        self.surface = SurfaceCheckingService(store, jobs, **paths)

    def _service(self, project_id):
        return (self.surface if is_surface_project(self.store.load(project_id))
                else self.boundary)

    def start(self, project_id, base_revision, request_id):
        return self._service(project_id).start(project_id, base_revision, request_id)

    def attach(self, project_id, base_revision, command_id, job_id):
        return self._service(project_id).attach(project_id, base_revision, command_id, job_id)
