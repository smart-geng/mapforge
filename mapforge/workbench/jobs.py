"""Owned subprocess jobs. Results never mutate a project automatically."""
from __future__ import annotations

import copy
import json
import multiprocessing as mp
import os
from pathlib import Path
import re
import threading
import time
import uuid

from .contracts import canonical_bytes, digest, require_json

ARCHIVE_SCHEMA = "mapforge-workbench-job-v1"
_ARCHIVE_FIELDS = {"job_id", "project_id", "operation", "base_revision", "content_hash", "input_hash",
                   "state", "started_at", "finished_at", "worker_pid", "result", "error", "events"}
_TERMINAL = {"succeeded", "failed", "cancelled", "timed_out"}
_OPERATIONS = {"source_check", "compile", "compile_surface", "validate"}


def _worker(connection, operation, payload):
    try:
        if operation == "source_check":
            from .sources import verify_source_snapshot
            result = verify_source_snapshot(payload["snapshot"], payload["source_dir"],
                                            payload.get("profile_path"))
        elif operation == "compile":
            from .compiler import compile_request
            # Business rejection is a completed compiler evaluation, not a
            # worker crash. Publishing artifacts never accepts the candidate.
            result = compile_request(payload)
        elif operation == "compile_surface":
            # The adapter calls the owned runner in this worker. Its Windows
            # Job handle closes with this process, including on cancellation.
            # Do not add a subprocess wrapper around that handle's owner.
            from .surface_compiler import compile_request
            result = compile_request(payload)
        elif operation == "validate":
            from .validation import validate_request
            result = validate_request(payload)
        else:
            raise ValueError("Unsupported background operation")
        connection.send({"state": "succeeded", "result": result})
    except Exception as exc:
        connection.send({"state": "failed", "error": f"{type(exc).__name__}: {exc}"})
    finally:
        connection.close()


class JobManager:
    """Spawn rather than fork: converter module state is never shared.

    Each job binds project/revision/content and an independent request id. A
    cancelled request remains cancelled even when its pipe contains a result.
    Polling a terminal result is read-only; accepting a candidate is separate.
    """

    def __init__(self, *, max_running=2, timeout_s=300, context=None, archive_dir=None):
        if isinstance(max_running, bool) or not isinstance(max_running, int) or max_running < 1:
            raise ValueError("max_running must be a positive integer")
        if isinstance(timeout_s, bool) or not isinstance(timeout_s, (int, float)) or not 0 < timeout_s < float("inf"):
            raise ValueError("timeout_s must be finite and positive")
        self.context = context or mp.get_context("spawn")
        self.max_running = max_running
        self.timeout_s = timeout_s
        self._jobs = {}
        self._requests = {}
        self._lock = threading.RLock()
        self._closed = False
        self.archive_dir = Path(archive_dir).resolve() if archive_dir is not None else None
        self.archive_issues = []
        if self.archive_dir is not None:
            self.archive_dir.mkdir(parents=True, exist_ok=True)
            for path in sorted(self.archive_dir.glob("*.json")):
                if not re.fullmatch(r"[0-9a-f]{32}\.json", path.name):
                    continue
                try:
                    job = self._read_archive(path.stem)
                    self._jobs[job["job_id"]] = job
                    self._requests[job["_request_key"]] = job["job_id"]
                except (OSError, ValueError, TypeError, KeyError) as exc:
                    self.archive_issues.append({"job_id": path.stem, "code": "archive-unavailable",
                                                "message": str(exc)})

    @staticmethod
    def _read_envelope(path):
        if path.is_symlink():
            raise ValueError("Job archive cannot be a symbolic link")
        value = json.loads(path.read_text(encoding="utf-8"))
        if (not isinstance(value, dict) or value.get("schema") != ARCHIVE_SCHEMA
                or not isinstance(value.get("payload"), dict)
                or value.get("sha256") != digest(value["payload"])):
            raise ValueError("Job archive checksum or schema is invalid")
        return value["payload"]

    @staticmethod
    def _envelope(payload):
        return canonical_bytes({"schema": ARCHIVE_SCHEMA, "payload": payload, "sha256": digest(payload)})

    def _read_archive(self, job_id):
        record = self._read_envelope(self.archive_dir / (job_id + ".json"))
        required = {"job_id", "project_id", "operation", "base_revision", "content_hash", "input_hash",
                    "request_key", "state", "started_at"}
        if (not required <= set(record) or set(record) - (_ARCHIVE_FIELDS | {"request_key"})
                or record["job_id"] != job_id
                or not isinstance(record["project_id"], str)
                or record["operation"] not in _OPERATIONS
                or isinstance(record["base_revision"], bool) or not isinstance(record["base_revision"], int)
                or record["base_revision"] < 0
                or any(not isinstance(record[key], str) or not re.fullmatch(r"[0-9a-f]{64}", record[key])
                       for key in ("input_hash", "content_hash", "request_key"))
                or record["state"] not in _TERMINAL | {"starting", "running", "cancelling"}):
            raise ValueError("Job archive fields are invalid")
        if record["state"] == "succeeded" and "result" not in record:
            raise ValueError("Successful job archive has no result")
        job = {key: value for key, value in record.items() if key in _ARCHIVE_FIELDS}
        job.update(_signature=record["input_hash"], _request_key=record["request_key"], _historical=True,
                   archive={"state": "persisted", "recorded_state": record["state"]})
        if record["state"] not in _TERMINAL:
            job.update(state="orphaned", recorded_state=record["state"],
                       error="Previous session execution is unconfirmed; this service has no owned process handle. "
                             "It will neither restart this request nor terminate the recorded PID.")
        return job

    def _archive(self, job):
        events = job.setdefault("events", [])
        if not events or events[-1]["state"] != job["state"]:
            events.append({"state": job["state"], "at": time.time()})
        if self.archive_dir is None:
            job["archive"] = {"state": "disabled", "recorded_state": None}
            return True
        previous = job.get("archive", {}).get("recorded_state")
        path = self.archive_dir / (job["job_id"] + ".json")
        temp = self.archive_dir / ("." + job["job_id"] + "." + uuid.uuid4().hex + ".tmp")
        try:
            if path.is_symlink():
                raise ValueError("Job archive cannot be a symbolic link")
            record = {key: value for key, value in job.items() if key in _ARCHIVE_FIELDS}
            record["request_key"] = job["_request_key"]
            with temp.open("xb") as stream:
                stream.write(self._envelope(record))
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temp, path)
            job["archive"] = {"state": "persisted", "recorded_state": job["state"]}
            return True
        except (OSError, ValueError, TypeError) as exc:
            job["archive"] = {"state": "failed", "recorded_state": previous,
                              "error": f"{type(exc).__name__}: {exc}"}
            return False
        finally:
            try:
                temp.unlink(missing_ok=True)
            except OSError:
                pass

    def _claim_request(self, job):
        """An exclusive, immutable claim prevents duplicate starts across services.

        A crash during this tiny claim leaves the request unavailable, never
        eligible for automatic restart. No process starts until the complete job
        starting record has also been written atomically.
        """
        if self.archive_dir is None:
            return None
        path = self.archive_dir / ("request-" + job["_request_key"] + ".json")
        claim = {"job_id": job["job_id"], "project_id": job["project_id"], "input_hash": job["input_hash"],
                 "request_key": job["_request_key"]}
        try:
            with path.open("xb") as stream:
                stream.write(self._envelope(claim))
                stream.flush()
                os.fsync(stream.fileno())
            return None
        except FileExistsError:
            try:
                existing = self._read_envelope(path)
                if (set(existing) != set(claim) or existing["project_id"] != claim["project_id"]
                        or existing["request_key"] != claim["request_key"]
                        or existing["input_hash"] != claim["input_hash"]):
                    raise ValueError("request_id already belongs to a different input")
                if not isinstance(existing["job_id"], str) or not re.fullmatch(r"[0-9a-f]{32}", existing["job_id"]):
                    raise ValueError("Invalid archived job identity")
                return self._read_archive(existing["job_id"])
            except (OSError, ValueError, TypeError, KeyError) as exc:
                raise ValueError("Previous request is reserved but unavailable or different; "
                                 "it cannot be restarted automatically") from exc

    def _update(self, job):
        if job.get("_historical"):
            return
        if job["state"] != "running":
            return
        process, pipe = job["_process"], job["_pipe"]
        try:
            available = pipe.poll()
        except (EOFError, OSError):
            # Windows PeekNamedPipe raises BrokenPipeError after os._exit or a
            # crashed worker, rather than reporting an empty pipe.
            job.update(state="failed", error="Worker pipe closed without a complete result")
            self._finish(job)
            return
        if available:
            try:
                message = pipe.recv()
            except (EOFError, OSError):
                message = {"state": "failed", "error": "Worker exited without a complete result"}
            if (not isinstance(message, dict) or message.get("state") not in {"succeeded", "failed"}
                    or (message.get("state") == "succeeded" and "result" not in message)
                    or (message.get("state") == "failed" and not isinstance(message.get("error"), str))):
                message = {"state": "failed", "error": "Worker returned an invalid result envelope"}
            # Worker output cannot overwrite job ownership, request fingerprints
            # or process handles, even if a future worker implementation errs.
            message = {key: value for key, value in message.items() if key in {"state", "result", "error"}}
            job.update(message)
            self._finish(job)
        elif not process.is_alive():
            job.update(state="failed", error=f"Worker exited without a result ({process.exitcode})")
            self._finish(job)
        elif time.monotonic() - job["_started"] > self.timeout_s:
            job.update(state="timed_out", error="Background task exceeded its time limit")
            self._finish(job, terminate=True)

    def _finish(self, job, *, terminate=False):
        process = job["_process"]
        if terminate and process.is_alive():
            process.terminate()
        process.join(timeout=0.2)
        if process.is_alive():
            process.terminate()
            process.join(timeout=1)
        job["_pipe"].close()
        job["finished_at"] = time.time()
        self._archive(job)

    @staticmethod
    def _public(job, revision=None, content_hash=None):
        result = {k: copy.deepcopy(v) for k, v in job.items() if not k.startswith("_")}
        result["historical"] = bool(job.get("_historical"))
        result["execution_status_known"] = job["state"] != "orphaned"
        result["stale"] = (result["historical"] or (revision is not None and revision != job["base_revision"]) or
                           (content_hash is not None and content_hash != job["content_hash"]))
        return result

    def start(self, project, operation, payload, request_id):
        if operation not in _OPERATIONS:
            raise ValueError("Unsupported background operation")
        if not isinstance(request_id, str) or not 1 <= len(request_id) <= 128:
            raise ValueError("A bounded request_id is required")
        require_json(payload)
        payload = copy.deepcopy(payload)
        key = digest({"project_id": project["project_id"], "request_id": request_id})
        input_hash = digest({"operation": operation, "payload": payload,
                             "project_id": project["project_id"], "revision": project["revision"],
                             "content_hash": project["content_hash"]})
        signature = input_hash
        with self._lock:
            if self._closed:
                raise ValueError("Background manager is closed")
            if key in self._requests:
                existing = self._jobs[self._requests[key]]
                if existing["_signature"] != signature:
                    raise ValueError("request_id already belongs to a different input")
                self._update(existing)
                return self._public(existing, project["revision"], project["content_hash"])
            for job in self._jobs.values():
                self._update(job)
            if sum(j["state"] == "running" for j in self._jobs.values()) >= self.max_running:
                raise ValueError("Background workers are busy; retry when a task finishes")
            job = dict(job_id=uuid.uuid4().hex, project_id=project["project_id"], operation=operation,
                       base_revision=project["revision"], content_hash=project["content_hash"],
                       input_hash=input_hash,
                       state="starting", started_at=time.time(), _started=time.monotonic(),
                       _signature=signature, _request_key=key, _historical=False)
            historical = self._claim_request(job)
            if historical is not None:
                self._jobs[historical["job_id"]] = historical
                self._requests[key] = historical["job_id"]
                return self._public(historical, project["revision"], project["content_hash"])
            self._jobs[job["job_id"]] = job
            self._requests[key] = job["job_id"]
            if not self._archive(job):
                job.update(state="failed", error="Task was not started because its starting archive could not be saved",
                           finished_at=time.time())
                return self._public(job)
            receiver, sender = self.context.Pipe(duplex=False)
            process = self.context.Process(target=_worker, args=(sender, operation, copy.deepcopy(payload)),
                                           daemon=True)
            job.update(_process=process, _pipe=receiver)
            try:
                process.start()
            except Exception as exc:
                receiver.close()
                sender.close()
                job.update(state="failed", error=f"Worker could not start: {type(exc).__name__}: {exc}",
                           finished_at=time.time())
                self._archive(job)
                return self._public(job)
            sender.close()
            job.update(state="running", worker_pid=process.pid)
            if not self._archive(job):
                job.update(state="failed", error="Owned worker stopped because its running archive could not be saved")
                self._finish(job, terminate=True)
            return self._public(job)

    def get(self, job_id, *, project_id, revision=None, content_hash=None):
        with self._lock:
            job = self._jobs.get(job_id)
            if job is None or job["project_id"] != project_id:
                raise KeyError("Task does not belong to this project")
            self._update(job)
            return self._public(job, revision, content_hash)

    def list(self, *, project_id, revision=None, content_hash=None):
        """Recover visible tasks without taking ownership of historical PIDs.

        Refresh only this project's owned processes. Return detached public
        records newest first, using the same stale rules as an individual poll.
        """
        with self._lock:
            selected = [job for job in self._jobs.values() if job["project_id"] == project_id]
            for job in selected:
                self._update(job)
            selected.sort(key=lambda job: (job["started_at"], job["job_id"]), reverse=True)
            return [self._public(job, revision, content_hash) for job in selected]

    def cancel(self, job_id, *, project_id):
        with self._lock:
            job = self._jobs.get(job_id)
            if job is None or job["project_id"] != project_id:
                raise KeyError("Task does not belong to this project")
            if job.get("_historical"):
                raise ValueError("Historical tasks are read-only; this service does not own their processes")
            if job["state"] == "running":
                job["state"] = "cancelling"
                self._archive(job)
                job["state"] = "cancelled"
                self._finish(job, terminate=True)
            return self._public(job)

    def retry_archive(self, job_id, *, project_id):
        with self._lock:
            job = self._jobs.get(job_id)
            if job is None or job["project_id"] != project_id:
                raise KeyError("Task does not belong to this project")
            if job.get("_historical") or job["state"] not in _TERMINAL:
                raise ValueError("Only owned terminal task archives can be retried")
            self._archive(job)
            return self._public(job)

    def close(self):
        with self._lock:
            self._closed = True
            for job in self._jobs.values():
                if job["state"] == "running" and not job.get("_historical"):
                    job["state"] = "cancelling"
                    self._archive(job)
                    job["state"] = "cancelled"
                    self._finish(job, terminate=True)
