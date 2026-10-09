"""Saved drafts may be uncompiled; accepted results must remain fully bound."""
from __future__ import annotations

import copy

from .contracts import StoreValidation, context_for, validate_context


def validate_project_context(project: dict) -> dict:
    """Accept only Store.create's exact empty compiler/policy state.

    This exception belongs to project snapshots, including previous.json.
    Candidate and validation contexts retain validate_context's four hashes.
    No compiler, policy, source or profile fingerprint is invented here.
    """
    value = project["context"]
    if value == context_for(project["source_snapshot"]):
        if (project["candidate"] is not None or project["validation"] is not None
                or project["candidate_history"] or project["validation_history"]
                or project["capabilities"]):
            raise StoreValidation("未绑定编译器和策略的源工程不能包含编辑能力、候选或检查")
        return copy.deepcopy(value)
    return validate_context(value)
