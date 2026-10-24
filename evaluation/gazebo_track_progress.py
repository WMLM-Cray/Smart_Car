#!/usr/bin/env python3
"""Observational progress audit: does the driven path advance *along* the track?

The production instrument (`track_eval.py`, teammate repo, unmodified) judges two things:
a cone hit and "further than OFF_TRACK = 1.60 m from the nearest cone". Neither fires when
the car leaves one arm of the track, crosses the cone-free gap between two cones, and joins
the *other* arm: every sample of such a traversal stays close to some cone, and few are close
to a cone it is not driving past. `laps` also stays plausible, because the lap counter only
sees the start line. So `clean` cannot express "the car cut across".

This module measures that traversal directly. Each sample is projected onto the track
centreline; the projection gives an arc length `s` on the closed loop. Consecutive samples
are then compared: `dxy` is how far the car actually moved, `ds_track` is how far the
projection moved *along the track*. Driving the track normally keeps the two equal. Crossing
from one arm to the other moves the car a short distance while its projection has to walk all
the way around one end of the loop, so `ds_track` jumps by metres while `dxy` stays small.

Purely an observation. It feeds no accept/reject condition, and the `clean` definition, the
cone/off-track thresholds and every existing verdict are untouched. See
`PREREGISTRATION_cap_ladder_v3_20261005.md`.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

TRACK_CUT_GAIN_M = 0.30
TRACK_CUT_EVENT_CAP = 50

# Largest advance along the track one sample spacing may be credited with, in metres. A
# single-valued nearest-point projection has to jump somewhere on a closed centreline, and the
# only place it can is the start-line seam, where it wraps `total -> 0`: every finished lap
# ends with one such step. On the validation tracks the seam reads 35.4-36.3 m while the car
# moved 0.03 m, and no car covers that in the 0.4-0.8 s between logged samples. Real traversals
# in this suite reach 10.4 m, so this bound separates them from the wrap without touching any
# genuine reading. It never exceeds `total/2` either: past half a loop the fold has no
# resolution left, since a longer traversal is indistinguishable from its complement.
TRACK_CUT_STEP_BUDGET_M = 12.0


def load_centerline(path):
    """Closed-loop centreline as (N, 2); segments numbered i -> i+1 with wrap-around."""
    points = np.asarray(np.load(Path(path), allow_pickle=False), dtype=float)
    if points.ndim != 2 or points.shape[1] < 2:
        raise ValueError('centerline must be (N, 2), got %r' % (points.shape,))
    points = points[:, :2]
    if len(points) < 3:
        raise ValueError('centerline needs at least 3 points, got %d' % len(points))
    return points


def centerline_arc_lengths(points):
    """Cumulative arc length at each vertex; the closing segment closes the loop."""
    steps = np.hypot(*np.diff(np.vstack([points, points[:1]]), axis=0).T)
    return np.concatenate([[0.0], np.cumsum(steps[:-1])]), float(steps.sum())


def project_to_centerline(points, xy):
    """Nearest point on the loop for every sample -> (arc length s, distance to centreline).

    Nearest point on segment i is `points[i] + t * edge`; its arc length is
    `arc[i] + t * |edge|`, which may equal `total` and is folded back to 0 so that the
    closing segment and segment 0 agree about where the loop starts.
    """
    xy = np.asarray(xy, dtype=float).reshape(-1, 2)
    starts = points
    edges = np.roll(points, -1, axis=0) - points
    lengths = np.hypot(edges[:, 0], edges[:, 1])
    arc, total = centerline_arc_lengths(points)
    safe = np.where(lengths > 0.0, lengths, 1.0)
    tangent = edges / safe[:, None]
    delta = xy[:, None, :] - starts[None, :, :]
    along = np.einsum('nse,se->ns', delta, tangent)
    t = np.clip(along / safe[None, :], 0.0, 1.0)
    closest = starts[None, :, :] + t[:, :, None] * edges[None, :, :]
    distance = np.hypot(*(xy[:, None, :] - closest).transpose(2, 0, 1))
    index = np.argmin(distance, axis=1)
    rows = np.arange(len(xy))
    s = arc[index] + t[rows, index] * lengths[index]
    return np.mod(s, total), distance[rows, index]


def loop_delta_s(previous, current, total):
    """Signed shortest arc from `previous` to `current` on a loop of length `total`."""
    raw = np.asarray(current) - np.asarray(previous)
    return (raw + total / 2.0) % total - total / 2.0


def _arc_hop(raw, dxy, budget, total):
    """How far along the loop the projection moved, with unmeasurable readings marked so.

    `raw = s[i+1] - s[i]` from a nearest-point projection, and the unsplit value is unusable
    for one reason: a single-valued projection must jump *somewhere*, and on a closed
    centreline the only place it can is the start-line seam, where it wraps `total -> 0`. Read
    literally that invents a traversal every time a lap ends, so the arc is taken short way
    round -- the fold into (-total/2, +total/2].

    The fold is right even for a genuine infield traversal, where it *is* the skipped
    distance. What it cannot survive is being read as an advance of half a loop or more in a
    single sample spacing: the fold's own horizon is `total/2`, so a reading that still lands
    on it after folding is the projection failing to choose a side, not the car driving there.
    Such steps are marked ambiguous and excluded, which is also why `budget` never exceeds
    `total/2` -- beyond that the metric has no resolution left to report with.

    The two candidates are the fold and its complement: the two readings of the same wrap.
    The shorter one is taken when it fits inside the budget, because the car takes the short
    way round unless it has driven the long way.

    Returns the signed hop plus the ambiguous mask.
    """
    raw, dxy = np.asarray(raw, dtype=float), np.asarray(dxy, dtype=float)
    folded = (raw + total / 2.0) % total - total / 2.0
    other = folded - np.sign(folded) * total
    folded_ok, other_ok = np.abs(folded) <= budget, np.abs(other) <= budget
    hop = np.where(folded_ok, folded, other)
    return np.where(folded_ok | other_ok, hop, 0.0), ~folded_ok & ~other_ok


def edge_gains(centerline, path):
    """Gain of every consecutive sample pair, in path order, ignoring any dead band.

    Exposed so a caller can *place* the dead band from data instead of assuming it. Returns
    `(gains, ambiguous)`: `gains[i] = |ds_track| - dxy` for the step i -> i+1, and
    `ambiguous[i]` marks the steps whose arc the projection cannot be trusted to have
    measured (`_arc_hop`). Ambiguous steps get a gain of exactly zero, so neither the count
    nor the reported maximum can be inflated by a reading that is known to be unreliable.
    """
    points = np.asarray(centerline, dtype=float).reshape(-1, 2)
    samples = np.asarray(path, dtype=float).reshape(-1, 3)
    if len(samples) < 2:
        return np.zeros(0), np.zeros(0, dtype=bool)
    s, _ = project_to_centerline(points, samples[:, 1:3])
    _, total = centerline_arc_lengths(points)
    dxy = np.hypot(*np.diff(samples[:, 1:3], axis=0).T)
    budget = min(TRACK_CUT_STEP_BUDGET_M, total / 2.0)
    hop, ambiguous = _arc_hop(np.diff(s), dxy, budget, total)
    return np.where(ambiguous, 0.0, np.abs(hop) - dxy), ambiguous


def track_cut_summary(centerline, path, gain_m=TRACK_CUT_GAIN_M, max_events=TRACK_CUT_EVENT_CAP):
    """Compare |ds_track| against dxy sample by sample; report the traversals that overshoot.

    Both quantities are computed between the *same* consecutive path samples, so the two
    sample spacings cancel: a window of one stance phase cannot invent a gain, because a
    stationary car moves neither its position nor its projection. Steps whose arc is
    ambiguous are not reported as cuts -- they are counted in `progress_metrics` instead.
    """
    samples = np.asarray(path, dtype=float).reshape(-1, 3)
    if len(samples) < 2:
        return []
    gains, ambiguous = edge_gains(centerline, samples)
    dxy = np.hypot(*np.diff(samples[:, 1:3], axis=0).T)
    ds_track = gains + dxy
    events = []
    for i in np.flatnonzero((gains > gain_m) & ~ambiguous):
        if len(events) >= max_events:
            break
        events.append(dict(
            sim_s=float(samples[i, 0]),
            x=float(samples[i, 1]),
            y=float(samples[i, 2]),
            dxy=round(float(dxy[i]), 3),
            ds_track=round(float(ds_track[i]), 3),
        ))
    return events


def progress_metrics(events, ambiguous=None, total=None):
    """`max_cut_gain_m` over every event, not only over the recorded subset.

    `ambiguous` counts the steps whose arc the projection could not measure (it had lost track
    of which way round the loop the car went, in both directions). They are excluded from
    `track_cut_count` and from `max_cut_gain_m`, and surfaced here so a zero count reads as "no
    cut" rather than "not looked at". `max_detectable_cut_m` is half the loop: an arc longer
    than that folds to its complement, so a traversal skipping more than half the track is
    reported at its complement instead of its true length.
    """
    gains = [event['ds_track'] - event['dxy'] for event in events]
    return dict(track_cut_count=len(events),
                max_cut_gain_m=round(max(gains), 3) if gains else 0.0,
                ambiguous_edge_count=int(np.count_nonzero(ambiguous)) if ambiguous is not None else 0,
                max_detectable_cut_m=round(total / 2.0, 3) if total else None)


def audit_path_samples(centerline, path, gain_m=TRACK_CUT_GAIN_M, max_events=TRACK_CUT_EVENT_CAP):
    """The `track_cut_count` / `max_cut_gain_m` pair that `audit_report` publishes."""
    events = track_cut_summary(centerline, path, gain_m, max_events)
    samples = np.asarray(path, dtype=float).reshape(-1, 3)
    ambiguous, total = None, None
    if len(samples) >= 2:
        points = np.asarray(centerline, dtype=float).reshape(-1, 2)
        _, total = centerline_arc_lengths(points)
        ambiguous = edge_gains(centerline, samples)[1]
    return progress_metrics(events, ambiguous, total)


def audit_track_eval(track_eval_json, centerline_npy):
    """Read-only audit of a finished run's `track_eval.json`."""
    report = json.loads(Path(track_eval_json).read_text(encoding='utf-8'))
    return audit_path_samples(load_centerline(centerline_npy), report.get('path') or [])


def resolve_centerline(track_dir):
    """Where a track's closed centreline lives.

    `centerline.npy` is not among the runner's `REQUIRED_TRACK_FILES` (it is a sibling of the
    track directories in `runs/gen_suite_20260926/`), so the file sits beside the track
    directory rather than inside it. A copy of the track (`runs/<run>/track/`) therefore has
    no centreline of its own, which is why the audit is fed the *source* track directory.
    """
    track_dir = Path(track_dir)
    for candidate in (track_dir / 'centerline.npy', track_dir.parent / 'centerline.npy'):
        if candidate.is_file():
            return candidate
    raise FileNotFoundError('no centerline.npy beside %s' % track_dir)


def audit_report(out, track_dir, seconds, gain_m=TRACK_CUT_GAIN_M):
    """The `track_progress_audit` block the runner writes into `report.json`.

    Never raises for a missing or malformed artefact: an observation that cannot be taken is
    reported as such, and must not be able to flip a run's verdict.
    """
    block = dict(gain_threshold_m=gain_m, centerline=None,
                 dt_track_eval_s=round(seconds / 4.0, 3),
                 track_cut_events=[], track_cut_count=0, max_cut_gain_m=0.0, error=None)
    try:
        centerline = resolve_centerline(track_dir)
        block['centerline'] = str(centerline)
        report = json.loads(Path(out / 'track_eval.json').read_text(encoding='utf-8'))
        samples = np.asarray(report.get('path') or [], dtype=float).reshape(-1, 3)
        block['sample_count'] = int(len(samples))
        points = load_centerline(centerline)
        events = track_cut_summary(points, samples, gain_m)
        _, total = centerline_arc_lengths(points)
        ambiguous = edge_gains(points, samples)[1] if len(samples) >= 2 else None
        block.update(progress_metrics(events, ambiguous, total))
        block['track_cut_events'] = events
    except BaseException as exc:                                    # noqa: BLE001
        block['error'] = '%s: %s' % (type(exc).__name__, exc)
    return block


def path_motion(path):
    """Stationary fraction and longest stuck stretch, on the same decimated path."""
    samples = np.asarray(path, dtype=float).reshape(-1, 3)
    if len(samples) < 2:
        return dict(static_fraction=0.0, stuck_s=0.0, static_s=0.0, span_s=0.0)
    dxy = np.hypot(*np.diff(samples[:, 1:3], axis=0).T)
    dt = np.diff(samples[:, 0])
    still = (dxy <= 0.02) | (dt <= 0.0)
    longest, current = 0.0, 0.0
    for flag, step in zip(still, dt):
        current = current + step if flag and step > 0 else 0.0
        longest = max(longest, current)
    static_s = float(np.sum(dt[still & (dt > 0)]))
    span = float(samples[-1, 0] - samples[0, 0])
    return dict(static_fraction=round(float(np.mean(still)), 4),
                stuck_s=round(longest, 1), static_s=round(static_s, 1),
                span_s=round(span, 1))
