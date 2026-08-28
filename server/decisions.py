"""Severity -> lockout / door_action mapping, per v2 contract.

See doc/inference_api_v2_contract.md "normative severity to lockout mapping".
"""

from __future__ import annotations

import os
from dataclasses import dataclass

# Cat IDs that the prey_v3 / cat_id_v2 heads emit (train_prey_v3.CAT_MAP order).
CAT_LABELS = ("mazge", "benis")

SEVERITY_LOCKOUT_S: dict[str, int] = {
    "none": 0,
    "low": 30,
    "medium": 120,
    "high": 600,
    "critical": 1800,
}

# Threshold above which we treat cat_id softmax as a confident identification.
CAT_CONF_THRESHOLD = float(os.environ.get("MAZGE_CAT_CONF_THRESHOLD", "0.70"))

# prey_score thresholds for severity bucketing. Env-overridable so a model swap
# ships its calibrated threshold via server.env, atomically with MAZGE_PREY_ONNX.
# combined_v1 (2026-08 retrain) wants MAZGE_PREY_DETECT=0.76 (keeps FN=0, cuts FP).
PREY_HIGH = float(os.environ.get("MAZGE_PREY_HIGH", "0.90"))
PREY_MEDIUM = float(os.environ.get("MAZGE_PREY_DETECT", "0.50"))


@dataclass(frozen=True)
class Decision:
    detected: bool
    prey_score: float
    cat_recognized: bool
    cat_id: str
    cat_confidence: float
    severity: str
    lockout_seconds: int
    door_action: str
    should_continue_burst: bool
    reason: str


def decide(prey_score: float, cat_logits_softmax: list[float]) -> Decision:
    """Pure function: map model outputs to a v2 decision.

    Invariant from the contract: door_action != "allow" when cat_recognized is false.
    """
    if cat_logits_softmax:
        idx = max(range(len(cat_logits_softmax)), key=cat_logits_softmax.__getitem__)
        cat_conf = float(cat_logits_softmax[idx])
        cat_id = CAT_LABELS[idx] if idx < len(CAT_LABELS) else "unknown"
        cat_recognized = cat_conf >= CAT_CONF_THRESHOLD
    else:
        cat_conf = 0.0
        cat_id = "unknown"
        cat_recognized = False

    if not cat_recognized:
        # No cat / unsure cat -> safe default: keep door closed.
        return Decision(
            detected=False,
            prey_score=float(prey_score),
            cat_recognized=False,
            cat_id=cat_id,
            cat_confidence=cat_conf,
            severity="medium",
            lockout_seconds=SEVERITY_LOCKOUT_S["medium"],
            door_action="deny",
            should_continue_burst=True,
            reason="no_cat_recognized",
        )

    if prey_score >= PREY_HIGH:
        sev = "high"
        action = "deny"
        cont = False
        reason = "high_confidence_prey"
        detected = True
    elif prey_score >= PREY_MEDIUM:
        sev = "medium"
        action = "deny"
        cont = True
        reason = "uncertain_need_more_frames"
        detected = True
    else:
        sev = "none"
        action = "allow"
        cont = False
        reason = "high_confidence_non_prey"
        detected = False

    return Decision(
        detected=detected,
        prey_score=float(prey_score),
        cat_recognized=True,
        cat_id=cat_id,
        cat_confidence=cat_conf,
        severity=sev,
        lockout_seconds=SEVERITY_LOCKOUT_S[sev],
        door_action=action,
        should_continue_burst=cont,
        reason=reason,
    )
