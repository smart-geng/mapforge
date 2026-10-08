"""Strict admission for newly recovered guided/source candidates only.

This module defines no quality thresholds and does not rank candidates. It uses
the existing mouth_frame_align.pick_from decisions, rejects unavailable evidence,
and leaves the caller to rank the surviving candidates by its unchanged rules.
The caller enables this only for a formerly skipped connector with guided source
evidence; pure geometry calls cannot claim G8 admission through this helper.
"""
from __future__ import annotations

from collections.abc import Mapping
import math
from numbers import Real


def _measured(value):
    return (isinstance(value, Real) and not isinstance(value, bool)
            and math.isfinite(float(value)) and value >= 0)


def recovery_score_evidence(entry, *, turn_required: bool, turn_end_zone: bool, aligned_frame: bool):
    """Check score evidence before pick_from performs numeric ranking.

    Missing/NaN evidence must be rejected before math.ceil or implicit defaults
    in the legacy ranking can hide it. This checks availability, never thresholds.
    """
    reasons = []
    fidelity = entry[3] if isinstance(entry[3], Mapping) else {}
    fields = ["median_m", "p95_m", "reverse_median_m", "reverse_p95_m", "coverage"]
    if aligned_frame:
        fields.append("edge_join_jump_per_m")
    if turn_required:
        fields += (["counter_mid_per_m", "counter_ends_per_m"] if turn_end_zone
                   else ["counter_curvature_per_m"])
    for key in fields:
        if not _measured(fidelity.get(key)):
            reasons.append("unmeasured:" + key)
    if _measured(fidelity.get("coverage")) and fidelity["coverage"] > 1:
        reasons.append("invalid:coverage")
    if not _measured(entry[4]):
        reasons.append("unmeasured:lane_centre_jump")
    shape = entry[0][6]
    contacts = shape.get("end_curvature", {}) if isinstance(shape, Mapping) else {}
    if not isinstance(contacts, Mapping):
        contacts = {}
    for end in ("start", "end"):
        contact = contacts.get(end)
        residuals = contact.get("edge_t2_residual") if isinstance(contact, Mapping) else None
        if (not isinstance(residuals, (list, tuple)) or len(residuals) != 2
                or any(not isinstance(v, Real) or isinstance(v, bool)
                       or not math.isfinite(float(v)) for v in residuals)):
            reasons.append("unmeasured:edge_residual_" + end)
    return reasons


def filter_recovery_candidates(scored, flags, *, recovery_required: bool,
                               full_g8_checked: bool, turn_required: bool,
                               turn_end_zone: bool):
    """Return (eligible scored entries, rejection records), preserving order.

    ``scored`` and ``flags`` are the original align_tree.score / pick_from values.
    ``full_g8_checked`` must mean the existing pick checked both directions and
    coverage (currently its match_end_curvature branch). ``turn_required`` is
    monotone_turns and sign != 0: null turn measurements for a straight connector
    are then legitimately inapplicable. No flag or measurement is invented.

    A healthy legacy path returns the original list object, without validation.
    New recovery requires explicit admission for G8, reverse steering, centre
    jumps and mouth-edge residuals. Numeric checks only establish availability;
    pass/fail against the unchanged thresholds comes exclusively from flags.
    """
    if not recovery_required:
        return scored, []
    eligible, rejected = [], []
    for entry in scored:
        name = entry[0][0]
        checks = flags.get(name) if isinstance(flags, Mapping) else None
        reasons = []
        if not full_g8_checked:
            reasons.append("incomplete:bidirectional_g8_and_coverage")
        if not isinstance(checks, Mapping):
            reasons.append("missing:pick_flags")
            checks = {}
        for key, expected in (("within_g8", True), ("counter_curvature", False),
                              ("lane_centre_jump", False), ("edge_residual", False)):
            if key not in checks:
                reasons.append("missing:" + key)
            elif type(checks[key]) is not bool:
                reasons.append("invalid:" + key)
            elif checks[key] is not expected:
                reasons.append(key)

        fidelity = entry[3] if isinstance(entry[3], Mapping) else {}
        for key in ("median_m", "p95_m", "reverse_median_m", "reverse_p95_m", "coverage"):
            if not _measured(fidelity.get(key)):
                reasons.append("unmeasured:" + key)
        if _measured(fidelity.get("coverage")) and fidelity["coverage"] > 1:
            reasons.append("invalid:coverage")
        if not _measured(entry[4]):
            reasons.append("unmeasured:lane_centre_jump")
        if not _measured(checks.get("edge_residual_per_m")):
            reasons.append("unmeasured:edge_residual")
        shape = entry[0][6]
        contacts = shape.get("end_curvature", {}) if isinstance(shape, Mapping) else {}
        if not isinstance(contacts, Mapping):
            contacts = {}
        for end in ("start", "end"):
            contact = contacts.get(end)
            residuals = contact.get("edge_t2_residual") if isinstance(contact, Mapping) else None
            if (not isinstance(residuals, (list, tuple)) or len(residuals) != 2
                    or any(not isinstance(v, Real) or isinstance(v, bool)
                           or not math.isfinite(float(v)) for v in residuals)):
                reasons.append("unmeasured:edge_residual_" + end)
        if turn_required:
            fields = (("counter_mid_per_m", "counter_ends_per_m") if turn_end_zone
                      else ("counter_curvature_per_m",))
            for key in fields:
                if not _measured(checks.get(key)):
                    reasons.append("unmeasured:" + key)
        if reasons:
            rejected.append({"candidate": name, "reasons": reasons, "checks": dict(checks)})
        else:
            eligible.append(entry)
    return eligible, rejected
