"""Unit tests for the shortcut ruler (`gazebo_track_progress`).

The ruler exists because `track_eval.py` can only see cone hits and distance to the nearest
cone, so a car that crosses the infield through the gap between two cones satisfies every
existing criterion. These tests pin the three ways that ruler can lie: it must read zero on
a car that follows the centreline, it must read one event on a car that cuts across the gap,
and it must read zero at the start-line seam, where a single-valued projection has to jump.
"""

import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import gazebo_track_progress as progress

# Two straight arms, 2.4 m apart, joined by a 1.2 m half-circle at the top and by the
# closing straight at the bottom: a hairpin whose two arms are `GAP_M` apart but 9.885 m
# apart along the loop, which is the geometry a shortcut exploits.
ARM_LENGTH_M = 8.0
GAP_M = 2.4
TURN_RADIUS_M = GAP_M / 2.0
STEP_M = 0.2


def hairpin_centerline(arm_length=ARM_LENGTH_M):
    """Closed centreline of the two-arm hairpin, `STEP_M` per segment."""
    right = np.arange(0.0, arm_length, STEP_M)
    angle = np.arange(1, 14) * (np.pi / 13.0)
    turn = np.column_stack([TURN_RADIUS_M - TURN_RADIUS_M * np.cos(angle),
                            arm_length + TURN_RADIUS_M * np.sin(angle)])
    down = right[::-1]
    arm_a = np.column_stack([np.zeros_like(right), right])
    arm_b = np.column_stack([np.full_like(down, GAP_M), down])
    return np.vstack([arm_a, turn, arm_b])


def timed(points, dt=0.5, start=0.0):
    """`(sim_s, x, y)` samples, the shape `track_eval.json` records."""
    points = np.asarray(points, dtype=float)
    return np.column_stack([start + dt * np.arange(len(points)), points])


def expected_hop(centerline, from_xy, to_xy):
    """The arc the projection measures between two samples, worked out independently of
    `_arc_hop`: take the raw difference of the two projections and read it short way round
    the loop, falling back to the complement when the fold is past the step budget."""
    points = np.asarray(centerline, dtype=float)
    _, total = progress.centerline_arc_lengths(points)
    s0 = progress.project_to_centerline(points, np.asarray([from_xy], dtype=float))[0][0]
    s1 = progress.project_to_centerline(points, np.asarray([to_xy], dtype=float))[0][0]
    folded = (s1 - s0 + total / 2.0) % total - total / 2.0
    other = folded - np.sign(folded) * total
    budget = min(progress.TRACK_CUT_STEP_BUDGET_M, total / 2.0)
    if abs(folded) <= budget:
        return folded
    if abs(other) <= budget:
        return other
    return None


def test_following_the_centreline_reports_no_shortcut():
    """A car on the racing line gains no arc it did not drive: zero events, zero gain."""
    centerline = hairpin_centerline()
    events = progress.track_cut_summary(centerline, timed(centerline))
    metrics = progress.audit_path_samples(centerline, timed(centerline))
    assert events == []
    assert metrics['track_cut_count'] == 0
    assert metrics['max_cut_gain_m'] == 0.0


def test_crossing_the_gap_reports_exactly_one_shortcut():
    """0.2 m steps up one arm, one 2.4 m step to the other arm, 0.2 m steps back down.

    The crossing step buys 9.885 m of arc with 2.4 m of driving, so it must be reported
    exactly once, with the gain and the two spacings recorded as measured.
    """
    centerline = hairpin_centerline()
    up = np.column_stack([np.zeros_like(np.arange(0.0, 4.0, STEP_M)),
                          np.arange(0.0, 4.0, STEP_M)])
    cross = np.array([[0.0, 4.0], [GAP_M, 4.0]])
    down = np.column_stack([np.full_like(np.arange(3.8, 0.0, -STEP_M), GAP_M),
                            np.arange(3.8, 0.0, -STEP_M)])
    path = timed(np.vstack([up, cross, down]))

    events = progress.track_cut_summary(centerline, path)
    metrics = progress.audit_path_samples(centerline, path)

    assert len(events) == 1
    event = events[0]
    hop = expected_hop(centerline, [0.0, 4.0], [GAP_M, 4.0])
    assert hop is not None and abs(hop) > 8.0, 'the two arms must be far apart along the loop'
    assert event['dxy'] == 2.4
    assert event['ds_track'] == round(abs(hop), 3)
    assert abs((event['ds_track'] - event['dxy']) - (abs(hop) - 2.4)) < 1e-9
    assert metrics['track_cut_count'] == 1
    assert metrics['max_cut_gain_m'] == round(abs(hop) - 2.4, 3)
    assert metrics['ambiguous_edge_count'] == 0
    assert event['x'] == 0.0 and event['y'] == 4.0


def test_loop_seam_produces_no_false_event():
    """The projection wraps `total -> 0` once per lap; that wrap is not a traversal.

    The samples below step across the seam in 0.025 m increments, i.e. every one of them is
    read by the projection as the loop restarting. Read literally that is a 9.885 m advance
    on a 0.025 m step; read short way round it is a 0.025 m advance, which is the truth.
    """
    centerline = hairpin_centerline()
    seam = np.array([[GAP_M, 0.0]])
    before = np.column_stack([np.linspace(GAP_M, GAP_M + 0.25, 11), np.zeros(11)])
    after = np.column_stack([np.zeros(11), np.linspace(0.0, 0.25, 11)])
    path = timed(np.vstack([before, seam, after]))

    events = progress.track_cut_summary(centerline, path)
    metrics = progress.audit_path_samples(centerline, path)

    assert events == []
    assert metrics['track_cut_count'] == 0
    assert metrics['max_cut_gain_m'] == 0.0
    assert metrics['ambiguous_edge_count'] == 0
    # The seam is real: without the fold the same samples measure a 9.885 m jump.
    assert metrics['max_detectable_cut_m'] == round(progress.centerline_arc_lengths(centerline)[1] / 2.0, 3)


def test_step_beyond_the_budget_is_ambiguous_not_a_shortcut():
    """The same crossing on a longer hairpin: past `TRACK_CUT_STEP_BUDGET_M` the reading is
    excluded from the count rather than reported as an even bigger shortcut."""
    long_arm_m = 20.0
    centerline = hairpin_centerline(arm_length=long_arm_m)
    total = progress.centerline_arc_lengths(centerline)[1]
    assert total / 2.0 > progress.TRACK_CUT_STEP_BUDGET_M
    # Mid-arm on a long hairpin: the two arms are 27.4 m apart along the loop, and the way
    # back round is 19.4 m, so neither reading fits inside the 12 m budget.
    mid = long_arm_m / 2.0 + 3.0
    path = timed(np.array([[0.0, mid], [GAP_M, mid]]))

    assert expected_hop(centerline, [0.0, mid], [GAP_M, mid]) is None
    events = progress.track_cut_summary(centerline, path)
    metrics = progress.audit_path_samples(centerline, path)
    assert events == []
    assert metrics['track_cut_count'] == 0
    assert metrics['max_cut_gain_m'] == 0.0
    assert metrics['ambiguous_edge_count'] == 1
    # The loop is long enough that the 12 m budget, not the loop, is the binding limit.
    assert metrics['max_detectable_cut_m'] == round(total / 2.0, 3)
    assert metrics['max_detectable_cut_m'] > progress.TRACK_CUT_STEP_BUDGET_M
