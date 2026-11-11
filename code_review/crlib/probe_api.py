"""API for reproduction probes.

A probe is a small function that re-creates one reviewed defect against the
*current* code and reports whether it still reproduces. Probes are what make
a finding "live": fix the code and the probe's verdict flips on the next
report build, however the code was restructured.

Usage, inside ``code_review/probes/probe_<area>.py``::

    from crlib.probe_api import probe, reproduced, not_reproduced

    @probe("SM-001", deps=["navi_bot/state_machine.py"])
    def delivery_goal_is_published():
        ...
        if delivery_goal_seen:
            return not_reproduced("delivery goal (2.0, 8.0) was published")
        return reproduced("only the pickup goal was ever published")

``deps`` lists the repo-relative files whose content the verdict depends on;
the report re-runs a probe only when one of them (or the probe file) changes.
An exception escaping a probe is reported as a probe *error*, never as
"fixed" - a probe that can no longer run needs a human look.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable


@dataclass
class Verdict:
    """Outcome of one probe run."""

    reproduced: bool
    detail: str
    data: dict[str, Any] = field(default_factory=dict)


@dataclass
class ProbeSpec:
    """A registered probe: the finding it backs, its callable and its inputs."""

    finding_id: str
    func: Callable[[], Verdict]
    deps: tuple[str, ...]


REGISTRY: dict[str, ProbeSpec] = {}


def probe(finding_id: str, *, deps: list[str]) -> Callable[
    [Callable[[], Verdict]], Callable[[], Verdict]
]:
    """Register ``func`` as the reproduction probe for ``finding_id``."""

    def register(func: Callable[[], Verdict]) -> Callable[[], Verdict]:
        if finding_id in REGISTRY:
            raise ValueError(f"duplicate probe for finding {finding_id}")
        REGISTRY[finding_id] = ProbeSpec(finding_id, func, tuple(deps))
        return func

    return register


def reproduced(detail: str, **data: Any) -> Verdict:
    """The defect is still present; ``detail`` states what was observed."""
    return Verdict(True, detail, data)


def not_reproduced(detail: str, **data: Any) -> Verdict:
    """The defect no longer reproduces; ``detail`` states what was observed."""
    return Verdict(False, detail, data)
