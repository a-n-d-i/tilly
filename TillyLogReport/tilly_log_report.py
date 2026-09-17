#!/usr/bin/env python3
"""
tilly_log_report.py
========================

Filters ArduPilot dataflash logs (.bin), finds points of interest
(course-change events + largest steering PID errors) and produces:

  1. A filtered, still-valid .bin log:
     - Around every finding, a window of +/- WINDOW_MIN minutes is kept
     - Everything in between is removed (to save disk space)
     - FMT/FMTU/UNIT/MULT/PARM/MSG/MODE/EV/ERR/CMD/VER etc. are ALWAYS kept
       (format definitions, parameter changes, mode changes, text messages)

  2. A single self-contained HTML report (images embedded as base64, no
     external dependencies) with:
     - A table of all findings (course changes + top-N PID errors)
     - Per finding: heuristic sea-state estimate + a note on any large
       deviation between compass heading and course over ground (COG)
     - Plots per finding:
         a) Yaw / Heel (Roll) / Speed on one chart (fixed scales: heel
            +/-40 deg, speed 0-7 kn)
         b) "PID review" view (modelled on ArduPilot's PID Review tool)
         c) Local XY view (meters) of course over ground (COG) vs.
            compass heading (Yaw), makes set/leeway visually apparent
     - An overview map (lat/lon, with an OpenStreetMap tile basemap when
       available) with all findings marked

GPS position errors are rejected in two layers (robust median-distance
check + sequential speed-jump check) since this is a small sailboat and no
fix should imply an implausible jump - see --max-distance-km/--max-speed-kn.

Course-change events within --course-merge-sec of each other are
accumulated into a single finding, clearly noted in the report.

PID errors that fall within the course-change context window
(--course-exclude-sec, default = --window-min) are ignored, since they are
expected to occur during an intentional maneuver.

Findings are ONLY considered while the vehicle was in GUIDED or AUTO mode
(configurable via --modes). The current flight mode is tracked automatically
by pymavlink from the log's MODE messages (mlog.flightmode).

Speed is reported in knots (kn), converted from the log's native m/s.

Requires: pymavlink, numpy, matplotlib
    pip install pymavlink numpy matplotlib --break-system-packages

Example:
    python3 ardupilot_log_report.py mission.bin --outdir out/

Key options:
    --window-min FLOAT         Time window around each finding (default 2.0 = -2 to +2 min)
    --top-n INT                 Number of largest PID errors (default 10)
    --pid-msgtype STR           Name of the steering PID log message (default PIDS)
    --modes STR                 Comma-separated list of allowed flight modes for
                                 finding selection (default GUIDED,AUTO)
    --course-exclude-sec FLOAT  Distance to a course change that excludes a PID
                                 error (default: same as --window-min, i.e. the
                                 whole course-change context window)
    --min-separation-sec FLOAT  Minimum spacing between selected top-N PID findings
    --deviation-threshold-deg   Mean heading/COG deviation above which a
                                 warning is shown (default 15 deg)

Note on sea-state estimate: purely heuristic, derived from roll (heel)
spread and zero-crossing period - not a calibrated sea-state model, just a
rough indicator for log analysis.
"""

import argparse
import base64
import gzip
import bisect
import io
import math
import os
import re
import sys
from dataclasses import dataclass, field
from datetime import datetime, timezone
from html import escape as h

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from pymavlink import mavutil

# ==========================================================================
# PID/filter tuning analysis engine (formerly a separate shared module;
# now inlined here so this is a single self-contained file). Covers:
# parameter-change/segment detection, sustained-error / actuator-
# saturation / step-wind-up / P-I-FF-balance / speed-dependence checks,
# and the suggestion-generation engine.
# ==========================================================================

# --------------------------------------------------------------------------
# Parameter lists shared with pid_video_overlay.py for visual/behavioral
# consistency between the two tools.
# --------------------------------------------------------------------------

DEFAULT_DISPLAY_PARAMS = [
    "ACRO_TURN_RATE",
    "ATC_STR_ACC_MAX",
    "ATC_STR_ANG_P",
    "ATC_STR_DEC_MAX",
    "ATC_STR_RAT_D",
    "ATC_STR_RAT_D_FF",
    "ATC_STR_RAT_FF",
    "ATC_STR_RAT_FLTD",
    "ATC_STR_RAT_FLTE",
    "ATC_STR_RAT_FLTT",
    "ATC_STR_RAT_I",
    "ATC_STR_RAT_IMAX",
    "ATC_STR_RAT_MAX",
    "ATC_STR_RAT_P",
    "ATC_STR_RAT_PDMX",
    "ATC_STR_RAT_SMAX",
    "INS_GYR_CAL",
    "LOG_REPLAY",
    "TURN_RADIUS",
]

# Which ArduPilot parameter prefix the tuning-suggestion engine reads,
# guessed from the PID log message type. Override with --param-prefix if
# your loop differs (e.g. a non-Rover PID message).
PARAM_PREFIX_GUESS = {
    "PIDS": "ATC_STR_RAT_",   # Rover steering rate loop
    "PIDA": "ATC_SPEED_",     # Rover speed/throttle loop (approx, varies by version)
    "PIDR": "ATC_RAT_RLL_",
    "PIDP": "ATC_RAT_PIT_",
    "PIDY": "ATC_RAT_YAW_",
}

FILTER_SUFFIXES = ["FLTT", "FLTE", "FLTD"]

# Suffixes considered when detecting a mid-log tuning change under a given
# prefix - anything else (ACRO_TURN_RATE, INS_GYR_CAL, ...) isn't part of
# this loop's own gains/filters, so a change there doesn't split the
# analysis into segments.
TUNE_SUFFIXES = ["P", "I", "D", "D_FF", "FF", "FLTT", "FLTE", "FLTD",
                  "IMAX", "MAX", "PDMX", "SMAX"]


def _format_param_value(v):
    """Shared display formatting - matches pid_video_overlay.py's
    _format_param_value for visual consistency across all the tools."""
    if v is None or (isinstance(v, float) and np.isnan(v)):
        return "n/a"
    if abs(v - round(v)) < 1e-9:
        return str(int(round(v)))
    return f"{v:.4f}".rstrip("0").rstrip(".")


# --------------------------------------------------------------------------
# Parameter history / mid-log tuning-change segmentation
# --------------------------------------------------------------------------

def param_value_at(parm_history, name, t):
    """Latest value of parameter `name` at or before time t (None if it
    never appears in the log before t). parm_history is {name: [(t, value),
    ...]} sorted by t."""
    series = parm_history.get(name)
    if not series:
        return None
    lo, hi = 0, len(series)
    while lo < hi:
        mid = (lo + hi) // 2
        if series[mid][0] <= t:
            lo = mid + 1
        else:
            hi = mid
    return series[lo - 1][1] if lo > 0 else None


def find_param_change_points(parm_history, prefix, t_min, t_max):
    """Every point within [t_min, t_max] where one of this loop's own
    tuning parameters (see TUNE_SUFFIXES) actually changed value (not just
    re-logged the same value, which ArduPilot sometimes does). Returns a
    sorted list of (t, name, old_value, new_value)."""
    changes = []
    for suffix in TUNE_SUFFIXES:
        series = parm_history.get(prefix + suffix)
        if not series:
            continue
        prev_val = None
        for t, v in series:
            if t <= t_min:
                prev_val = v
                continue
            if t > t_max:
                break
            if prev_val is not None and v != prev_val:
                changes.append((t, prefix + suffix, prev_val, v))
            prev_val = v
    changes.sort(key=lambda c: c[0])
    return changes


def build_tuning_segments(changes, t_min, t_max):
    """Splits [t_min, t_max] into segments bounded by every detected
    tuning-parameter change, so each segment has a genuinely constant tune
    throughout it. With no changes, this is just one segment covering the
    whole range - identical to a single combined analysis. Mixing data
    from before and after a gain change into one aggregate would blend two
    different tunes together and could hide a fix (or a regression) that
    already happened - this is the whole reason this module exists."""
    boundaries = sorted(set([t_min] + [c[0] for c in changes] + [t_max]))
    segments = []
    for i in range(len(boundaries) - 1):
        start, end = boundaries[i], boundaries[i + 1]
        if end <= start:
            continue
        changes_here = [c for c in changes if c[0] == start] if i > 0 else []
        segments.append({"start": start, "end": end, "changes": changes_here})
    return segments


# --------------------------------------------------------------------------
# Time-domain checks a pure error/oscillation spectrum can't see:
#   1. A prolonged Tar/Act gap (sustained bias, not oscillation) - often
#      caused by the I-term pinned at IMAX (out of authority), or by the
#      actuator being physically saturated / mechanically stuck (neither
#      of which any PID gain change can fix).
#   2. Sluggish response right after a course change because the I-term
#      wound up during the PREVIOUS maneuver and has to unwind (decay)
#      before it can help with the new one.
#   3. How much of the steady-state effort I is doing vs. P/FF - a high I
#      share is both an undertuned-FF symptom and the direct cause of (2).
#   4. Whether tracking error depends on boat speed (informational only -
#      Rover's steering rate controller has no gain scheduling).
# --------------------------------------------------------------------------

def find_runs(mask, t, gap_merge_s=1.0):
    """Contiguous True-runs in `mask` (aligned to `t`), merging runs
    separated by a gap of at most gap_merge_s (so a single dropped sample
    doesn't split one real sustained-error episode into two). Returns a
    list of (start_idx, end_idx) inclusive."""
    idx = np.where(mask)[0]
    if len(idx) == 0:
        return []
    runs = []
    start = idx[0]
    prev = idx[0]
    for i in idx[1:]:
        if t[i] - t[prev] > gap_merge_s:
            runs.append((start, prev))
            start = i
        prev = i
    runs.append((start, prev))
    return runs


def analyze_sustained_error(t, err, i_term, imax, threshold_percentile=90,
                             min_duration_s=1.5, gap_merge_s=1.0):
    """Finds the longest episode where |err| stayed persistently large (not
    just one spike) within this data, and checks whether the I-term was
    pinned near IMAX during it - a saturated I-term simply cannot close a
    steady bias any faster, no matter what the gains are set to."""
    abs_err = np.abs(err)
    valid = ~np.isnan(abs_err)
    if valid.sum() < 10:
        return None
    threshold = float(np.percentile(abs_err[valid], threshold_percentile))
    if threshold <= 0:
        return None

    runs = find_runs(abs_err >= threshold, t, gap_merge_s)
    episodes = []
    for s, e in runs:
        duration = t[e] - t[s]
        if duration < min_duration_s:
            continue
        seg_err = abs_err[s:e + 1]
        seg_i = i_term[s:e + 1]
        i_valid = seg_i[~np.isnan(seg_i)]
        i_frac_of_imax = i_saturated_frac = None
        if imax is not None and imax > 0 and len(i_valid):
            i_frac_of_imax = float(np.mean(np.abs(i_valid)) / imax)
            i_saturated_frac = float(np.mean(np.abs(i_valid) >= 0.95 * imax))
        episodes.append({
            "t_start": float(t[s]), "duration": float(duration),
            "mean_abs_err": float(np.mean(seg_err)), "max_abs_err": float(np.max(seg_err)),
            "i_frac_of_imax": i_frac_of_imax, "i_saturated_frac": i_saturated_frac,
        })
    if not episodes:
        return None
    return max(episodes, key=lambda ep: ep["duration"] * ep["mean_abs_err"])


def diagnose_sustained_error_cause(t, act, rcou_t, rcou_pwm, sustained,
                                    rc_min_us=1000.0, rc_max_us=2000.0, pwm_margin_us=5.0):
    """Given a sustained-error episode, checks RCOU and Act during that
    window against their own whole-window baselines to tell physical
    actuator saturation (RCOU pinned at its travel limit) apart from
    mechanical stiction (RCOU actively varying but Act barely moves) -
    neither of which any PID gain change can fix."""
    if sustained is None or rcou_t is None or len(rcou_t) < 10:
        return None
    t0, t1 = sustained["t_start"], sustained["t_start"] + sustained["duration"]
    rmask = (rcou_t >= t0) & (rcou_t <= t1)
    amask = (t >= t0) & (t <= t1)
    if rmask.sum() < 3 or amask.sum() < 3:
        return None

    pwm_win = rcou_pwm[rmask]
    act_win = act[amask]
    act_win = act_win[~np.isnan(act_win)]
    if len(act_win) < 3:
        return None

    near_min = np.abs(pwm_win - rc_min_us) <= pwm_margin_us
    near_max = np.abs(pwm_win - rc_max_us) <= pwm_margin_us
    saturated_frac = float(np.mean(near_min | near_max))

    rcou_std_win = float(np.std(pwm_win))
    rcou_std_all = float(np.std(rcou_pwm)) + 1e-9
    act_all = act[~np.isnan(act)]
    act_std_win = float(np.std(act_win))
    act_std_all = float(np.std(act_all)) + 1e-9 if len(act_all) else 1e-9

    verdict = "unclear"
    if saturated_frac > 0.6:
        verdict = "actuator_saturated"
    elif (rcou_std_win > 0.3 * rcou_std_all) and (act_std_win < 0.3 * act_std_all):
        verdict = "possible_stiction"

    return {
        "saturated_frac": saturated_frac,
        "rcou_std_win": rcou_std_win, "rcou_std_all": rcou_std_all,
        "act_std_win": act_std_win, "act_std_all": act_std_all,
        "verdict": verdict,
    }


def find_step_events(t, tar, min_gap_s=1.0):
    """Detects 'course change'-like step commands in Tar via a robust
    (median + MAD) outlier threshold on its sample-to-sample jumps, so it
    adapts to whatever units/scale this PID loop's Tar happens to use
    rather than assuming a fixed step size. Nearby jump samples (within
    min_gap_s) are merged into one event."""
    dtar = np.diff(tar)
    valid = ~np.isnan(dtar)
    if valid.sum() < 10:
        return []
    med = np.median(dtar[valid])
    mad = np.median(np.abs(dtar[valid] - med)) + 1e-9
    threshold = max(abs(med) + 6 * mad, 1e-6)
    idx = np.where(np.abs(dtar) > threshold)[0]
    if len(idx) == 0:
        return []
    events = []
    group = [idx[0]]
    for i in idx[1:]:
        if t[i] - t[group[-1]] <= min_gap_s:
            group.append(i)
        else:
            events.append(group)
            group = [i]
    events.append(group)

    results = []
    for g in events:
        start_i, end_i = g[0], g[-1] + 1
        if end_i >= len(tar):
            continue
        results.append({
            "step_time": float(t[end_i]),
            "step_size": float(tar[end_i] - tar[start_i]),
        })
    return results


def analyze_step_windup(t, tar, err, i_term, imax, events,
                         pre_window_s=1.0, window_s=15.0, progress_frac=0.63):
    """For every detected course-change step, checks whether the I-term
    already held a large value OPPOSING the new command right before the
    step (residual wind-up from the previous maneuver) and measures how
    long the response took to close most of the initial gap (time to
    within (1-progress_frac) of the initial post-step error - i.e. roughly
    one time-constant's worth of progress by default, since the first 10%
    of any decay closes almost immediately regardless of how slow the
    overall settle is, and is a poor discriminator). A high-I loop that
    decays slowly shows up here as: large opposing I-before + long onset
    lag - even though nothing in the steady-state error looks
    'oscillatory'."""
    results = []
    for ev in events:
        step_time, step_size = ev["step_time"], ev["step_size"]
        if abs(step_size) < 1e-9:
            continue
        pre_mask = (t >= step_time - pre_window_s) & (t < step_time)
        i_pre = i_term[pre_mask]
        i_pre = i_pre[~np.isnan(i_pre)]
        i_before = float(np.mean(i_pre)) if len(i_pre) else None

        post_mask = (t >= step_time) & (t <= step_time + window_s)
        tt, ee = t[post_mask], err[post_mask]
        if len(tt) < 3:
            continue
        err0 = ee[0] if not np.isnan(ee[0]) else np.nanmean(ee[:3])
        if err0 is None or np.isnan(err0) or err0 == 0:
            continue
        target_level = err0 * (1 - progress_frac)
        progressed = np.where(np.abs(ee) <= np.abs(target_level))[0]
        lag_s = float(tt[progressed[0]] - step_time) if len(progressed) else None

        opposing = (i_before is not None and imax is not None and imax > 0
                    and np.sign(i_before) != np.sign(step_size)
                    and abs(i_before) >= 0.3 * imax)

        results.append({
            "step_time": step_time, "step_size": step_size,
            "i_before": i_before,
            "i_before_frac_imax": (abs(i_before) / imax
                                    if i_before is not None and imax else None),
            "lag_s": lag_s, "opposing": opposing,
        })
    return results


def analyze_pid_balance(p_term, i_term, ff_term):
    """Mean |output| share of P vs. I vs. FF - a high I share means I is
    doing steady-state work that feed-forward should mostly be doing, and
    is also the direct cause of slow-to-unwind wind-up on reversals."""
    def mabs(a):
        a = a[~np.isnan(a)]
        return float(np.mean(np.abs(a))) if len(a) else 0.0

    mean_p, mean_i, mean_ff = mabs(p_term), mabs(i_term), mabs(ff_term)
    total = mean_p + mean_i + mean_ff
    if total <= 0:
        return None
    return {
        "mean_abs_p": mean_p, "mean_abs_i": mean_i, "mean_abs_ff": mean_ff,
        "i_share": mean_i / total, "p_share": mean_p / total, "ff_share": mean_ff / total,
    }


def analyze_speed_dependence_pairs(speed, aerr, n_bins=3):
    """Speed/|error| relationship from pre-matched (speed, |err|) sample
    pairs. Informational only: a fixed-gain PID has no way to compensate
    for this itself."""
    valid = ~np.isnan(speed) & ~np.isnan(aerr)
    speed, aerr = speed[valid], aerr[valid]
    if len(speed) < 10 or np.std(speed) < 1e-6:
        return None
    edges = np.quantile(speed, np.linspace(0, 1, n_bins + 1))
    edges[-1] += 1e-6
    bins = []
    for i in range(n_bins):
        m = (speed >= edges[i]) & (speed < edges[i + 1])
        if m.sum() < 3:
            continue
        bins.append({
            "speed_lo": float(edges[i]), "speed_hi": float(edges[i + 1]),
            "rms_err": float(np.sqrt(np.mean(aerr[m] ** 2))), "n": int(m.sum()),
        })
    correlation = float(np.corrcoef(speed, aerr)[0, 1]) if len(speed) > 3 else None
    return {"bins": bins, "correlation": correlation}


# --------------------------------------------------------------------------
# Suggestion engine - the ONE place that turns measured symptoms into
# concrete parameter suggestions, kept separate from the checks that feed
# it so a threshold change only needs to happen in one place.
# --------------------------------------------------------------------------

def generate_suggestions(cur, prefix, metrics, sustained, cause, step_windup, balance):
    """
    cur: callable, cur(suffix) -> current value of prefix+suffix (or None)
    metrics: dict with rms_error, dominant_osc_hz, osc_power_frac,
             noise_floor_hz, err_n_seg (from the oscillation/noise Welch analysis)
    sustained: result of analyze_sustained_error(), or None
    cause: result of diagnose_sustained_error_cause(), or None
    step_windup: result of analyze_step_windup() (a list, possibly empty)
    balance: result of analyze_pid_balance(), or None

    Returns a list of (full_param_name, current, suggested, reason) tuples.
    `current`/`suggested` may be None when the parameter's value couldn't be
    found in the log (ArduPilot only logs a parameter in PARM messages once
    it's been changed away from its firmware default - a loop still running
    on stock defaults simply never gets logged, even though it has a real
    running value). The problem is still real and still reported in that
    case; the reasoning text switches to relative language ("reduce by
    ~15% from whatever it's currently set to") instead of a computed
    absolute number, and says explicitly that the current value wasn't
    found so it's obvious why there's no before/after pair to show.

    Heuristic, not an autotuner - change one parameter (or one tightly-
    coupled pair, e.g. P+D together) at a time and re-log before making the
    next change.
    """
    suggestions = []
    rms_error = metrics["rms_error"]
    dominant_osc_hz = metrics["dominant_osc_hz"]
    osc_power_frac = metrics["osc_power_frac"]
    noise_floor_hz = metrics["noise_floor_hz"]
    err_n_seg = metrics["err_n_seg"]

    P, I, D, FF = cur("P"), cur("I"), cur("D"), cur("FF")
    FLTD, IMAX = cur("FLTD"), cur("IMAX")

    def scaled(current, factor):
        """Absolute new value if current is known, else None - the caller
        always still appends the suggestion, just phrases the reasoning
        relatively when this comes back None."""
        return round(current * factor, 4) if current is not None else None

    def not_found_note(name):
        return (f" (current {name} wasn't found in this log's parameters - it's "
                f"probably still at its firmware default, which ArduPilot only logs "
                f"once a parameter is changed away from default. Check your parameter "
                f"file/GCS for the actual value.)")

    # --- Oscillation vs. noise ---
    if err_n_seg > 0 and dominant_osc_hz > 0 and osc_power_frac > 0.25:
        if noise_floor_hz and dominant_osc_hz >= noise_floor_hz * 0.7:
            new_fltd = (round(min(FLTD, max(2.0, noise_floor_hz * 0.5)), 1) if FLTD is not None
                        else round(max(2.0, noise_floor_hz * 0.5), 1))
            suggestions.append((
                prefix + "FLTD", FLTD, new_fltd,
                f"Dominant tracking-error oscillation at {dominant_osc_hz:.2f} Hz sits "
                f"inside the measured gyro noise band (noise floor ~{noise_floor_hz:.1f} Hz). "
                f"Lower the D-term filter cutoff before touching gains - looks like D is "
                f"amplifying sensor noise, not fighting a real disturbance."
                + ("" if FLTD is not None else not_found_note(prefix + "FLTD")
                   + " Value shown above is an absolute target cutoff, not a delta.")
            ))
        else:
            suggestions.append((
                prefix + "P", P, scaled(P, 0.85),
                f"Sustained {dominant_osc_hz:.2f} Hz oscillation in tracking error "
                f"({osc_power_frac*100:.0f}% of error spectral power), below the noise "
                f"band - looks like genuine control-loop instability. Reduce P ~15% and "
                f"re-log before further changes."
                + ("" if P is not None else not_found_note(prefix + "P"))
            ))
            if D is None or D > 0:
                suggestions.append((
                    prefix + "D", D, scaled(D, 0.85),
                    "Reduce D alongside P when the oscillation is genuine instability "
                    "rather than noise amplification."
                    + ("" if D is not None else not_found_note(prefix + "D"))
                ))
    elif err_n_seg > 0 and not np.isnan(rms_error) and rms_error > 0 and osc_power_frac <= 0.25:
        suggestions.append((
            prefix + "P", P, scaled(P, 1.10),
            f"No significant oscillation detected (dominant peak carries only "
            f"{osc_power_frac*100:.0f}% of error spectral power) but tracking error RMS "
            f"is {rms_error:.3f}. Loop looks under-driven/sluggish rather than unstable - "
            f"try raising P ~10%."
            + ("" if P is not None else not_found_note(prefix + "P"))
        ))

    # --- Filter cutoffs vs. noise floor (needs a known current value to
    # judge whether a change is even warranted, unlike the checks above
    # where the problem itself is evidence enough) ---
    if noise_floor_hz:
        control_bandwidth_hz = max(dominant_osc_hz, 0.2)
        suggested_cutoff = round(max(control_bandwidth_hz * 3, min(noise_floor_hz * 0.5, 10.0)), 1)
        for suffix in FILTER_SUFFIXES:
            name = prefix + suffix
            if any(s[0] == name for s in suggestions):
                continue  # already covered above (e.g. FLTD from the oscillation check)
            current = cur(suffix)
            if current is not None and abs(current - suggested_cutoff) / max(current, 0.1) > 0.25:
                suggestions.append((
                    name, current, suggested_cutoff,
                    f"Gyro noise floor ~{noise_floor_hz:.1f} Hz, estimated control bandwidth "
                    f"~{control_bandwidth_hz:.2f} Hz. Cutoff should sit between these - current "
                    f"value is {'higher' if current > suggested_cutoff else 'lower'} than that band."
                ))

    # --- Prolonged Tar/Act gap: was the I-term simply out of authority? ---
    if sustained is not None:
        actuator_explains_it = cause is not None and cause["verdict"] in ("actuator_saturated", "possible_stiction")
        if sustained["i_saturated_frac"] is not None and not actuator_explains_it:
            if sustained["i_saturated_frac"] > 0.6:
                suggestions.append((
                    prefix + "IMAX", IMAX, scaled(IMAX, 1.3),
                    f"Tracking error stayed above the 90th-percentile level for "
                    f"{sustained['duration']:.1f}s (mean |err| {sustained['mean_abs_err']:.3f}, "
                    f"peak {sustained['max_abs_err']:.3f}), and the I-term sat within 5% of IMAX for "
                    f"{sustained['i_saturated_frac']*100:.0f}% of that time, with no sign of a "
                    f"physical actuator limit during that window - raising IMAX looks like the "
                    f"right lever here."
                    + ("" if IMAX is not None else not_found_note(prefix + "IMAX"))
                ))
            elif sustained["i_frac_of_imax"] is not None and sustained["i_frac_of_imax"] < 0.5:
                suggestions.append((
                    prefix + "I", I, scaled(I, 1.2),
                    f"Tracking error stayed above the 90th-percentile level for "
                    f"{sustained['duration']:.1f}s without the I-term approaching IMAX "
                    f"(averaged {sustained['i_frac_of_imax']*100:.0f}% of it) - I looks too low to "
                    f"close a sustained bias in reasonable time. Consider raising I moderately."
                    + ("" if I is not None else not_found_note(prefix + "I"))
                ))
        elif sustained["i_saturated_frac"] is None and not actuator_explains_it and sustained["duration"] >= 1.5:
            # Genuinely can't tell if I/IMAX explains this without a logged
            # IMAX value - still worth flagging that the episode happened.
            suggestions.append((
                prefix + "IMAX", None, None,
                f"Tracking error stayed above the 90th-percentile level for "
                f"{sustained['duration']:.1f}s (mean |err| {sustained['mean_abs_err']:.3f}, "
                f"peak {sustained['max_abs_err']:.3f}) starting at t={sustained['t_start']:.1f}s, "
                f"with no sign of a physical actuator limit - but {prefix}IMAX wasn't found in "
                f"this log's parameters, so it's not possible to tell whether I ran out of "
                f"authority to close the gap. Check {prefix}IMAX and {prefix}I in your "
                f"parameter file."
            ))

    # --- Slow response to course changes: I-term wound up the wrong way ---
    opposing_events = [e for e in step_windup if e["opposing"] and e["lag_s"] is not None]
    other_events = [e for e in step_windup if not e["opposing"] and e["lag_s"] is not None]
    if len(opposing_events) >= 2:
        med_lag_opp = float(np.median([e["lag_s"] for e in opposing_events]))
        med_lag_other = float(np.median([e["lag_s"] for e in other_events])) if other_events else None
        if med_lag_other is None or med_lag_opp > med_lag_other * 1.5:
            mean_i_frac = float(np.mean([e["i_before_frac_imax"] for e in opposing_events
                                          if e["i_before_frac_imax"] is not None]))
            suggestions.append((
                prefix + "I", I, scaled(I, 0.8),
                f"{len(opposing_events)} of {len(step_windup)} detected course-change steps had "
                f"the I-term already holding {mean_i_frac*100:.0f}% of IMAX in the OPPOSITE "
                f"direction from the new command beforehand (residual wind-up from the previous "
                f"maneuver) - median response-onset lag for those was {med_lag_opp:.2f}s"
                + (f" vs {med_lag_other:.2f}s for other steps" if med_lag_other is not None else "")
                + ". A high I value that decays slowly delays every direction reversal like this - "
                  "try reducing I ~20% and re-log; also consider whether IMAX is unnecessarily high."
                + ("" if I is not None else not_found_note(prefix + "I"))
            ))

    # --- P/I/FF balance ---
    if balance is not None and balance["i_share"] > 0.40:
        bump = 1.2 if FF else 1.5  # FF None or 0 both treated as "basically no FF"
        suggestions.append((
            prefix + "FF", FF, (round(max(FF * bump, 0.05), 4) if FF is not None else None),
            f"I-term output averaged {balance['i_share']*100:.0f}% of total P+I+FF effort "
            f"(P {balance['p_share']*100:.0f}%, FF {balance['ff_share']*100:.0f}%) - I is doing "
            f"steady-state work that feed-forward should mostly be doing. Raising FF reduces "
            f"reliance on I, which also reduces how much I there is left to wind up and unwind "
            f"on every direction reversal (see the step-response findings)."
            + ("" if FF is not None else not_found_note(prefix + "FF"))
        ))

    return suggestions

# ==========================================================================


try:
    import contextily as cx
    HAS_CONTEXTILY = True
except Exception:
    HAS_CONTEXTILY = False

# --------------------------------------------------------------------------
# Constants
# --------------------------------------------------------------------------

MPS_TO_KN = 1.9438444924406  # 1 m/s in knots

# Message types that are ALWAYS kept (formats, parameters, events, ...)
ALWAYS_KEEP_TYPES = {
    "FMT", "FMTU", "UNIT", "MULT",   # format/unit definitions (keeps log parseable)
    "PARM",                          # parameter changes
    "MSG", "STATUSTEXT",             # text messages (incl. course-change events)
    "MODE",                          # flight/drive mode changes
    "EV", "ERR",                     # events / error codes
    "CMD",                           # mission commands
    "VER", "MAVLINK_VERSION",        # version info
    "ORGN",                          # origin
}

# Regex for course-change STATUSTEXT, e.g. "SRC=250/1:Course -1 deg, Heading + 330"
# Tolerant of: optional prefix before "Course", optional +/- sign with an
# optional space before the number (ArduPilot sometimes emits "+ 330").
COURSE_CHANGE_RE = re.compile(
    r"course[:\s]*([+-]?)\s*(\d+(?:\.\d+)?)\s*deg[,;]?\s*"
    r"heading[:\s]*([+-]?)\s*(\d+(?:\.\d+)?)",
    re.IGNORECASE,
)


def _signed_float(sign, number):
    sign = (sign or "").strip()
    return float(f"{sign}{number}") if sign == "-" else float(number)

# --------------------------------------------------------------------------
# Vendored html2canvas 1.4.1 (MIT license, https://html2canvas.hertzen.com),
# gzip-compressed and base64-encoded, so the "export finding as image"
# button works with no external/CDN dependency - the report stays a single
# self-contained file, consistent with the rest of this tool's design.
# --------------------------------------------------------------------------
_HTML2CANVAS_JS_GZ_B64 = (
    "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"
    "WeUM6Rr0wqq0iKKRnLW/Qeo1u796Qz8o0H51Q9etSp9UFHpFcL5acVtToDSKslbl85e1qko0QFqrjrCsKdVtckOKjw5+TXKgwKKKF6Tq4EsOqaKXi1q7RRu18r7SZrRplItIF7s9VAJKkd/nLalObrUis/bvcd2X2kEO8a549CwNyblRdOE1qxrqXzxE+Bvl8DOWJ6R0"
    "gN915fCGfLeKhna4Q4wq9Ifv0J4hvv7HF8VNLTwebHb9ojjDOm5IU6frSlV23j8QuQb9aZEqfLwDekqm0bNFT3W7vYwlvaJn7Qn4S6Ud7veDnhvlK+WsTC/0cl87O6SfZbuE91tydkU7F2WmnT3Qy5vy1HXU5HKi9dY0gCsp9rqmwNW3PPr1FR9dZ+KLUgb3C8oPdpmg"
    "3/FFeXl8+Vq+lq/la/lavpav5Wv5Wi6vrJXuLfv9gFSj4axai1jcbvxI64a8b9W+4fuhVXskllG+0ORNslGTT6wa+HntUks5bmD7mVI3yL5cirCzR+SHUj5W2jna6pbrpL1NW3r5mLSztHVRPrbaW7R1U65r7QxttcvHWnuTtrRyXWnv0NZrv9d+r/1+Rb+q0tZp03r1"
    "417Lf5GSmvY5N3f/GpGt4HcaiI9tON8fibIK/O2aFQ9ESAODsR5yCmk67VUMtsqEfNOKUDXIdx5czZC+VlKIaRXXqwdV5fspxk3PnHKg7LslK24H3sfluXHIy0tyuGMZrLTfro2UH4PSQQz8a/lavpb//8srckjI3Weys4Xa5YAc6cUdUuKq644UNbKhyyOtvkO+G6V1"
    "R0UZ6F+4qqt6Q+6sk21yhfc1J0dBvV1qkI83tt9Rl0s7Si0cz74jGxd2+8lW6dGJi1N+f6ORc4XUaVh10pocERf/QIhC3fd9TenTog5w8gPl91nyS3p7izrxeCpfeOO3iHKB58MpUeL7l3V5VedgXZFXfc6uU9Jdzngyoddas+bfr8E0uq225EeogvcrJfCs+X2p8y4H"
    "ygpehrrtf/cy6fp7E3KG91Q33vtVVPty7Z8y2boJxhvqXTrTONVuaWr7gT+eUSovKAca8u1nbTX8ekmUIj2v2nqicjODvgA+PbBkhRS/0UNdNrRfLU/FHVqrRuBLb2mC/g/WL8R3f6y3f2jK38qfce+fA2UvUz632iAX1fKOdv6A3ze11jZ4AvtF0pmED5TnZ7Qa126U"
    "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"
    "cQ//P0Hc6PMhCAMA"
)


def _get_html2canvas_js():
    """Decompresses the vendored html2canvas library. Computed once and
    cached - it's ~200KB of JS, no need to redo the work per report."""
    if not hasattr(_get_html2canvas_js, "_cache"):
        _get_html2canvas_js._cache = gzip.decompress(
            base64.b64decode(_HTML2CANVAS_JS_GZ_B64)
        ).decode("utf-8")
    return _get_html2canvas_js._cache


SEA_STATE_BANDS = [
    (1.5, "calm / smooth (SS 0-1)"),
    (4.0, "slight (SS 2-3)"),
    (8.0, "moderate (SS 3-4)"),
    (15.0, "rough (SS 5)"),
    (math.inf, "very rough (SS 6+)"),
]


# --------------------------------------------------------------------------
# Data structures
# --------------------------------------------------------------------------

@dataclass
class Finding:
    kind: str          # "course_change" | "pid_error"
    t: float            # unix timestamp
    label: str
    detail: str
    value: float = 0.0     # abs(PIDS.Err) for pid_error, else 0
    course: float = None
    heading: float = None
    merged_count: int = 1               # >1 if this represents several accumulated course-change events
    merged_sources: list = field(default_factory=list)  # [(t, course, heading, raw_text), ...] of the merged events
    highlight_span_s: float = None      # if set, shade [t - highlight_span_s, t] in this finding's plots (e.g. a gust's rise window)


@dataclass
class LogData:
    att: list = field(default_factory=list)   # (t, roll, pitch, yaw)
    gps: list = field(default_factory=list)   # (t, spd_mps, gcrs, lat, lon)
    pids: list = field(default_factory=list)  # (t, tar, act, err, p, i, d, ff)
    all_gps: list = field(default_factory=list)  # full (filtered) GPS track
    rcou: list = field(default_factory=list)  # (t, pwm_us) - ram/steering servo output channel
    gyro: list = field(default_factory=list)  # (t, rate) - single-axis gyro, for noise-floor analysis


# --------------------------------------------------------------------------
# Ram position simulation
#
# Derives an ESTIMATED ram position from the RCOU PWM ArduPilot sends to the
# actuator's ESP32 position controller, by replaying the exact same control
# logic offline: RC pulse -> target position -> proportional speed request
# (with the firmware's minimum-move-speed floor) -> accel-ramped speed ->
# integrated position. This is NOT measured telemetry - the ESP32's real
# ADC position feedback never reaches the ArduPilot log, so this assumes an
# ideal actuator that tracks the commanded ESC speed exactly (no slippage,
# backlash, or hydraulic lag beyond the accel-ramp model). Constants below
# mirror the ESP32 sketch; override via CLI flags if your calibration
# differs.
# --------------------------------------------------------------------------

@dataclass
class RamParams:
    rc_min_us: float = 1000.0
    rc_max_us: float = 2000.0
    travel_mm: float = 250.0
    max_speed_mms: float = 50.0
    max_accel_mmss: float = 250.0
    deadband_mm: float = 2.0
    kp: float = 5.0
    min_move_speed_mms: float = 20.0   # firmware's speed floor once moving


def rc_to_target_mm(pulse_us, p: RamParams):
    pulse_us = min(max(pulse_us, p.rc_min_us), p.rc_max_us)
    return (pulse_us - p.rc_min_us) * p.travel_mm / (p.rc_max_us - p.rc_min_us)


def simulate_ram_position(rcou_slice, p: RamParams):
    """Replays the ESP32 controller loop over the (t, pwm_us) samples in
    rcou_slice (sorted by time). Returns a list of
    (t, pos_mm, speed_mms, target_mm). The ram is assumed to start already
    tracking the first commanded target (pos = target, speed = 0), since
    the true starting position is unknown - this avoids a spurious
    warm-up transient at the start of a short window."""
    if len(rcou_slice) < 2:
        return []
    rcou_slice = sorted(rcou_slice, key=lambda r: r[0])
    t_prev, pwm0 = rcou_slice[0]
    pos = rc_to_target_mm(pwm0, p)
    speed = 0.0
    out = [(t_prev, pos, speed, pos)]

    for t, pwm in rcou_slice[1:]:
        dt = t - t_prev
        if dt <= 0:
            continue
        target = rc_to_target_mm(pwm, p)
        error = target - pos

        if abs(error) <= p.deadband_mm:
            desired_speed = 0.0
        else:
            desired_speed = max(-p.max_speed_mms, min(error * p.kp, p.max_speed_mms))
            if desired_speed < 0:
                desired_speed = max(-p.max_speed_mms, min(desired_speed, -p.min_move_speed_mms))
            elif desired_speed > 0:
                desired_speed = min(p.max_speed_mms, max(desired_speed, p.min_move_speed_mms))

        max_dv = p.max_accel_mmss * dt
        speed += max(-max_dv, min(desired_speed - speed, max_dv))
        pos = min(max(pos + speed * dt, 0.0), p.travel_mm)

        out.append((t, pos, speed, target))
        t_prev = t

    return out


# --------------------------------------------------------------------------
# Helper functions
# --------------------------------------------------------------------------

def get_text(msg):
    """Extract text from MSG/STATUSTEXT messages, tolerant of field naming."""
    for field_name in ("Message", "text", "Text"):
        if hasattr(msg, field_name):
            val = getattr(msg, field_name)
            if val:
                return val.strip() if isinstance(val, str) else str(val)
    return None


def wrap180(deg):
    """Normalise an angle to (-180, 180]."""
    return (deg + 180.0) % 360.0 - 180.0


def interp_angle_deg(t_query, t_known, angle_known_deg):
    """Wrap-safe 1D interpolation of angles via sin/cos (avoids the 0/360 jump)."""
    if len(t_known) == 0:
        return np.full(len(t_query), np.nan)
    rad = np.radians(angle_known_deg)
    sin_i = np.interp(t_query, t_known, np.sin(rad))
    cos_i = np.interp(t_query, t_known, np.cos(rad))
    return np.degrees(np.arctan2(sin_i, cos_i))


def build_windows(times, half_width_sec):
    """Build merged time windows [t-h, t+h] around every given timestamp."""
    if not times:
        return []
    intervals = sorted((t - half_width_sec, t + half_width_sec) for t in times)
    merged = [list(intervals[0])]
    for s, e in intervals[1:]:
        if s <= merged[-1][1]:
            merged[-1][1] = max(merged[-1][1], e)
        else:
            merged.append([s, e])
    return [tuple(m) for m in merged]


class WindowIndex:
    """Fast test for 'does time t fall inside one of the merged windows'."""

    def __init__(self, merged_windows):
        self.windows = merged_windows
        self.starts = [w[0] for w in merged_windows]

    def contains(self, t):
        if not self.windows:
            return False
        i = bisect.bisect_right(self.starts, t) - 1
        if i < 0:
            return False
        return self.windows[i][0] <= t <= self.windows[i][1]


def fig_to_base64(fig):
    buf = io.BytesIO()
    fig.savefig(buf, format="png", dpi=110, bbox_inches="tight")
    plt.close(fig)
    buf.seek(0)
    return base64.b64encode(buf.read()).decode("ascii")


def fmt_time(t):
    return datetime.fromtimestamp(t, tz=timezone.utc).strftime("%Y-%m-%d %H:%M:%S UTC")


def format_duration(seconds):
    """Human-readable duration, e.g. 45s / 4m 12s / 1h 03m."""
    seconds = abs(seconds)
    if seconds < 60:
        return f"{seconds:.0f}s"
    m, s = divmod(int(round(seconds)), 60)
    if m < 60:
        return f"{m}m {s:02d}s"
    h, m = divmod(m, 60)
    return f"{h}h {m:02d}m"


def haversine_m(lat1, lon1, lat2, lon2):
    """Great-circle distance in meters between two lat/lon points."""
    r = 6371000.0
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = (math.sin(dphi / 2) ** 2
         + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2)
    return 2 * r * math.asin(min(1.0, math.sqrt(a)))


# --------------------------------------------------------------------------
# Pass 1: read log, identify findings
# --------------------------------------------------------------------------

def is_valid_fix(msg):
    """Basic sanity check for a GPS message: has a non-(0,0) position and,
    if a fix-status field is present, reports at least a 3D fix. Filters
    out pre-lock / no-fix garbage positions before they can poison the
    outlier detector."""
    lat = getattr(msg, "Lat", None)
    lng = getattr(msg, "Lng", None)
    if not lat or not lng:
        return False, None, None
    status = getattr(msg, "Status", None)
    if status is not None and status < 3:
        return False, None, None
    return True, lat, lng


def scan_log(path, pid_msgtype, allowed_modes, gyro_field="GyrZ", gyro_instance=0, ram_channel="C1"):
    """Reads the log once from start to end and collects course-change events
    as well as all steering-PID error values (for the top-N selection), plus
    every valid raw GPS position (used afterwards to compute a robust median
    position for outlier rejection in pass 2).

    Only events that occur while the vehicle was in one of `allowed_modes`
    (e.g. GUIDED, AUTO) are considered. The current mode is tracked
    automatically by pymavlink from the log's MODE messages
    (mlog.flightmode).

    Also collects the full-log parameter change history (every PARM
    message, regardless of mode/window) so that findings can later show
    "what was the parameter set to at this point in time" - parameters can
    change mid-log during a tuning session, so this isn't just a boot-time
    snapshot. See param_value_at().

    ALSO collects full-log (mode-gated, but NOT window-gated) PIDS/IMU/
    RCOU/GPS-speed streams for the tuning-suggestion engine. This is
    deliberately separate from filter_and_collect()'s pass 2, which only
    gathers data inside +/- WINDOW_MIN of a finding (course-change or
    top-N PID error) - that's correct for the filtered-log output and the
    per-finding plots, but would badly under-cover the tuning analysis:
    a chronic low-level oscillation or a sustained actuator-saturation
    episode that never happens to be a top-N spike or a course change
    would simply never be seen. The tuning engine needs everything."""
    mlog = mavutil.mavlink_connection(path, dialect="ardupilotmega")

    course_events = []
    pid_samples = []   # (t, err)
    gps_positions = []  # (t, lat, lon) - all valid fixes, for the median anchor
    parm_history = {}   # name -> [(t, value), ...] sorted by t
    tune_pids = []       # (t, tar, act, err, p, i, d, ff) - mode-gated, whole log
    tune_gyro = []       # (t, rate) - mode-gated, whole log
    tune_rcou = []       # (t, pwm_us) - mode-gated, whole log
    tune_gps_speed = []  # (t, speed_mps) - mode-gated, whole log
    tune_att = []        # (t, roll, pitch, yaw) - mode-gated, whole log
    t_min, t_max = None, None
    n_msgs = 0
    mode_segments = []  # (mode, t_start) for the report summary
    last_mode = None
    allowed_modes_upper = {m.upper() for m in allowed_modes}

    while True:
        msg = mlog.recv_match(blocking=False)
        if msg is None:
            break
        n_msgs += 1
        mtype = msg.get_type()
        t = getattr(msg, "_timestamp", None)
        if t is None:
            continue
        if t_min is None or t < t_min:
            t_min = t
        if t_max is None or t > t_max:
            t_max = t

        current_mode = (getattr(mlog, "flightmode", None) or "UNKNOWN").upper()
        if current_mode != last_mode:
            mode_segments.append((current_mode, t))
            last_mode = current_mode

        in_scope = current_mode in allowed_modes_upper

        if mtype == "GPS":
            valid, lat, lng = is_valid_fix(msg)
            if valid:
                gps_positions.append((t, lat, lng))
            if in_scope:
                spd = getattr(msg, "Spd", None)
                if spd is not None:
                    tune_gps_speed.append((t, float(spd)))

        if mtype == "PARM":
            name = getattr(msg, "Name", None)
            value = getattr(msg, "Value", None)
            if name is not None and value is not None:
                parm_history.setdefault(name, []).append((t, float(value)))

        if mtype in ("MSG", "STATUSTEXT"):
            text = get_text(msg)
            if text:
                m = COURSE_CHANGE_RE.search(text)
                if m and in_scope:
                    course_events.append(
                        Finding(
                            kind="course_change",
                            t=t,
                            label=f"Course change: {text}",
                            detail=text,
                            course=_signed_float(m.group(1), m.group(2)),
                            heading=_signed_float(m.group(3), m.group(4)),
                        )
                    )
        elif mtype == pid_msgtype:
            err = getattr(msg, "Err", None)
            if err is not None and in_scope:
                pid_samples.append((t, float(err)))
            tar = getattr(msg, "Tar", None)
            act = getattr(msg, "Act", None)
            if in_scope and tar is not None and act is not None and err is not None:
                tune_pids.append((
                    t, float(tar), float(act), float(err),
                    float(getattr(msg, "P", np.nan)), float(getattr(msg, "I", np.nan)),
                    float(getattr(msg, "D", np.nan)), float(getattr(msg, "FF", np.nan)),
                ))
        elif mtype == "IMU":
            if in_scope and getattr(msg, "I", 0) == gyro_instance and hasattr(msg, gyro_field):
                tune_gyro.append((t, float(getattr(msg, gyro_field))))
        elif mtype == "RCOU":
            if in_scope:
                val = getattr(msg, ram_channel, None)
                if val is not None:
                    tune_rcou.append((t, float(val)))
        elif mtype == "ATT":
            if in_scope:
                tune_att.append((
                    t, getattr(msg, "Roll", np.nan), getattr(msg, "Pitch", np.nan),
                    getattr(msg, "Yaw", np.nan),
                ))

    for name in parm_history:
        parm_history[name].sort(key=lambda tv: tv[0])

    # Safety net: pymavlink's own internal bookkeeping (mlog.params) is
    # built from the exact same PARM messages just iterated above, so it
    # should normally be redundant - but if any parameter ends up known to
    # pymavlink and missing from our own per-message capture for any
    # reason, fall back to it rather than silently having no value at all.
    # We can't know exactly when a fallback value took effect (we only
    # have pymavlink's final/latest-seen value for it), so it's recorded
    # as active from the very start of the log.
    mlog_params = dict(getattr(mlog, "params", None) or {})
    n_before_fallback = len(parm_history)
    recovered = []
    for name, value in mlog_params.items():
        if name not in parm_history:
            parm_history[name] = [(t_min if t_min is not None else 0.0, float(value))]
            recovered.append(name)
    if recovered:
        print(f"      Note: {len(recovered)} parameter(s) known to pymavlink but not seen as "
              f"individual PARM messages during the scan were recovered via mlog.params: "
              f"{', '.join(sorted(recovered)[:10])}"
              f"{', ...' if len(recovered) > 10 else ''}", file=sys.stderr)

    return (course_events, pid_samples, gps_positions, t_min, t_max, n_msgs,
            mode_segments, parm_history, tune_pids, tune_gyro, tune_rcou, tune_gps_speed, tune_att)


def build_param_grid_html(parm_history, display_params, t, n_cols=3):
    """Small HTML box/grid of current parameter values at time t, shown at
    the top of every finding - mirrors the "current parameters" text
    overlay in pid_video_overlay.py, but as a static HTML grid instead of a
    rendered-to-video image."""
    cells = []
    for name in display_params:
        value = param_value_at(parm_history, name, t)
        cells.append(
            f'<div><span class="pname">{h(name)}</span> = {_format_param_value(value)}</div>'
        )
    return (
        f'<div class="param-grid" style="grid-template-columns: repeat({max(1, n_cols)}, auto);">'
        + "".join(cells) + "</div>"
    )


def select_top_pid_errors(pid_samples, course_events, top_n,
                           course_exclude_sec, min_separation_sec):
    """Selects the top_n largest |Err| values that were not caused by a
    course change (proximity to a course-change event), with a minimum
    spacing between them (to avoid picking the same spike multiple times)."""
    course_times = sorted(f.t for f in course_events)

    def near_course_change(t):
        if not course_times:
            return False
        i = bisect.bisect_left(course_times, t)
        for j in (i - 1, i):
            if 0 <= j < len(course_times):
                if abs(course_times[j] - t) <= course_exclude_sec:
                    return True
        return False

    candidates = [
        (t, err) for (t, err) in pid_samples if not near_course_change(t)
    ]
    candidates.sort(key=lambda x: abs(x[1]), reverse=True)

    selected = []
    selected_times = []
    for t, err in candidates:
        if any(abs(t - st) < min_separation_sec for st in selected_times):
            continue
        selected.append(
            Finding(
                kind="pid_error",
                t=t,
                label=f"PID error |Err|={abs(err):.3f}",
                detail=f"Steering PID Err = {err:.4f} at {fmt_time(t)}",
                value=abs(err),
            )
        )
        selected_times.append(t)
        if len(selected) >= top_n:
            break

    selected.sort(key=lambda f: f.t)
    return selected


def median_position(gps_positions):
    """Robust anchor position (median lat/lon) computed from ALL valid GPS
    fixes across the whole log. Used as a sanity-check center: any fix far
    away from this median is almost certainly a gross GPS error, regardless
    of what the (possibly also-bad) previous fix looked like."""
    if not gps_positions:
        return None, None
    lats = np.array([p[1] for p in gps_positions])
    lons = np.array([p[2] for p in gps_positions])
    return float(np.median(lats)), float(np.median(lons))


def merge_course_events(course_events, gap_sec=10.0):
    """Accumulates consecutive course-change events into a single finding
    when they occur within `gap_sec` seconds of each other (chained: each
    event is compared to the previous one in the same group, so a rapid
    burst of small corrections collapses into one entry instead of
    cluttering the report). The merged finding's `course` is the SUM of the
    individual course-change deltas (the net accumulated turn during the
    burst), and `heading` is the LAST event's heading (the final state
    reached)."""
    if not course_events:
        return []
    events = sorted(course_events, key=lambda f: f.t)
    groups = [[events[0]]]
    for f in events[1:]:
        if f.t - groups[-1][-1].t <= gap_sec:
            groups[-1].append(f)
        else:
            groups.append([f])

    merged = []
    for g in groups:
        if len(g) == 1:
            merged.append(g[0])
            continue
        last = g[-1]
        total_course = sum((e.course or 0.0) for e in g)
        merged.append(
            Finding(
                kind="course_change",
                t=last.t,
                label=f"Course change (merged x{len(g)}, accumulated): {last.detail}",
                detail=last.detail,
                course=total_course,
                heading=last.heading,
                merged_count=len(g),
                merged_sources=[(e.t, e.course, e.heading, e.detail) for e in g],
            )
        )
    return merged


# --------------------------------------------------------------------------
# Pass 2: filter + collect raw data for plots + write filtered log
# --------------------------------------------------------------------------

def filter_and_collect(path, out_bin_path, win_index, pid_msgtype, max_speed_kn,
                        median_lat, median_lon, max_distance_km, ram_channel_field,
                        gyro_field="GyrZ", gyro_instance=0):
    """Second pass: writes the filtered log and collects raw data for plots.

    GPS position errors are rejected in two layers:
      1. Any fix farther than `max_distance_km` from the robust median
         position (computed in pass 1 across the whole log) is rejected
         outright - this catches gross errors (e.g. a pre-lock/no-fix
         position) regardless of what came right before it, so a single
         bad point at the start of the log can no longer poison layer 2.
      2. Among the remaining fixes, any fix implying an unrealistic jump
         in speed (> max_speed_kn) from the previous *accepted* fix is
         also rejected - catches shorter-range glitches/multipath.
    Rejected fixes are dropped entirely: not written to the filtered log,
    not used in any plot or the overview map.
    """
    mlog = mavutil.mavlink_connection(path, dialect="ardupilotmega")
    data = LogData()
    have_median = median_lat is not None and median_lon is not None
    max_distance_m = max_distance_km * 1000.0

    n_in, n_out = 0, 0
    n_skip_nobuf = 0
    n_gps_total = 0
    n_gps_rejected_far = 0
    n_gps_rejected_jump = 0
    last_good_fix = None  # (t, lat, lon)

    with open(out_bin_path, "wb") as out:
        while True:
            msg = mlog.recv_match(blocking=False)
            if msg is None:
                break
            n_in += 1
            mtype = msg.get_type()
            t = getattr(msg, "_timestamp", None)

            gps_plausible = True
            if mtype == "GPS" and t is not None:
                valid, lat, lng = is_valid_fix(msg)
                if valid:
                    n_gps_total += 1
                    # layer 1: gross-error check against the robust median position
                    if have_median:
                        dist_from_median_m = haversine_m(median_lat, median_lon, lat, lng)
                        if dist_from_median_m > max_distance_m:
                            gps_plausible = False
                            n_gps_rejected_far += 1
                    # layer 2: sequential speed-jump check against the last accepted fix
                    if gps_plausible and last_good_fix is not None:
                        dt = t - last_good_fix[0]
                        if dt > 0:
                            dist_m = haversine_m(last_good_fix[1], last_good_fix[2], lat, lng)
                            implied_speed_kn = (dist_m / dt) * MPS_TO_KN
                            if implied_speed_kn > max_speed_kn:
                                gps_plausible = False
                                n_gps_rejected_jump += 1
                    if gps_plausible:
                        last_good_fix = (t, lat, lng)
                else:
                    gps_plausible = False

            keep = (mtype in ALWAYS_KEEP_TYPES) or (
                t is not None and win_index.contains(t)
            )
            if mtype == "GPS":
                keep = keep and gps_plausible

            if keep:
                try:
                    buf = msg.get_msgbuf()
                    if buf:
                        out.write(buf)
                        n_out += 1
                except Exception:
                    n_skip_nobuf += 1

            if t is None:
                continue

            # collect raw data for plots, only inside the windows
            if win_index.contains(t):
                if mtype == "ATT":
                    data.att.append((
                        t,
                        getattr(msg, "Roll", np.nan),
                        getattr(msg, "Pitch", np.nan),
                        getattr(msg, "Yaw", np.nan),
                    ))
                elif mtype == "GPS" and gps_plausible:
                    data.gps.append((
                        t,
                        getattr(msg, "Spd", np.nan),  # m/s, native log unit
                        getattr(msg, "GCrs", np.nan),
                        getattr(msg, "Lat", np.nan),
                        getattr(msg, "Lng", np.nan),
                    ))
                elif mtype == pid_msgtype:
                    data.pids.append((
                        t,
                        getattr(msg, "Tar", np.nan),
                        getattr(msg, "Act", np.nan),
                        getattr(msg, "Err", np.nan),
                        getattr(msg, "P", np.nan),
                        getattr(msg, "I", np.nan),
                        getattr(msg, "D", np.nan),
                        getattr(msg, "FF", np.nan),
                    ))
                elif mtype == "RCOU":
                    pwm = getattr(msg, ram_channel_field, None)
                    if pwm is not None:
                        data.rcou.append((t, float(pwm)))
                elif mtype == "IMU":
                    if getattr(msg, "I", 0) == gyro_instance:
                        rate = getattr(msg, gyro_field, None)
                        if rate is not None:
                            data.gyro.append((t, float(rate)))

            # for the overview map: full (filtered) GPS track
            if mtype == "GPS" and t is not None and gps_plausible:
                lat = getattr(msg, "Lat", None)
                lng = getattr(msg, "Lng", None)
                if lat and lng:
                    data.all_gps.append((t, lat, lng))

    return data, n_in, n_out, n_skip_nobuf, n_gps_total, n_gps_rejected_far, n_gps_rejected_jump


def slice_window(records, t_center, half_width):
    lo, hi = t_center - half_width, t_center + half_width
    return [r for r in records if lo <= r[0] <= hi]


# --------------------------------------------------------------------------
# Per-finding analysis: sea-state estimate + heading/COG deviation
# --------------------------------------------------------------------------

def estimate_sea_state(att_slice):
    if len(att_slice) < 5:
        return "not enough data", None, None
    t = np.array([r[0] for r in att_slice])
    roll = np.array([r[1] for r in att_slice], dtype=float)
    roll = roll[~np.isnan(roll)]
    if len(roll) < 5:
        return "not enough data", None, None

    roll_std = float(np.std(roll))

    # rough roll period via zero-crossings of the detrended curve
    detrended = roll - np.mean(roll)
    signs = np.sign(detrended)
    signs[signs == 0] = 1
    crossings = np.where(np.diff(signs) != 0)[0]
    period = None
    if len(crossings) >= 2 and len(t) == len(roll):
        span = t[crossings[-1]] - t[crossings[0]]
        n_half_periods = len(crossings) - 1
        if n_half_periods > 0:
            period = 2.0 * span / n_half_periods

    label = next(txt for limit, txt in SEA_STATE_BANDS if roll_std < limit)
    return label, roll_std, period


def find_good_examples(tune_att, tune_pids, t_min, t_max, window_s, max_examples=5,
                        min_coverage_frac=0.9, min_activity_frac=0.15, edge_margin_s=None):
    """Finds the best-tracking example for each distinct sea state actually
    experienced in the log - a positive counterpart to the course-change/
    PID-error findings, which are all "here's a problem" by construction.
    Scans the whole log (mode-gated tune_att/tune_pids from scan_log(), NOT
    the window-gated per-finding data - same reasoning as the tuning
    engine: a calm, well-tracked stretch is exactly the kind of thing that
    would never show up as a course-change or a top-N error, so it would
    never be seen otherwise) in non-overlapping windows of `window_s`
    seconds, computes the sea state and RMS tracking error for each, and
    keeps the lowest-error window per sea-state label.

    Three checks specifically guard against a window looking spuriously
    "good" for reasons that have nothing to do with actually tracking well:
      - Edge margin: windows overlapping the first/last edge_margin_s
        seconds of the scanned range are skipped outright (default: one
        full window_s at each end). The very start/end of a log is
        commonly a transition in/out of the mode being scanned (just
        armed, about to disarm) with a smaller RMS error only because
        there's barely any real data or command yet, not because the
        tune is good - this is the direct, blunt fix for that.
      - Coverage: the window's actual data has to span at least
        min_coverage_frac of the window's nominal duration for BOTH
        attitude and PID data - catches the same problem at an internal
        mode-gap, not just the log's true edges.
      - Activity: the steering command (Tar) has to show at least
        min_activity_frac of the whole log's typical command activity -
        a window that's mostly idle (even if not fully) drags its own RMS
        down with near-zero-error idle samples and would otherwise look
        artificially "perfect" for having little real tracking happening.

    Returns a list of (t_center, sea_label, rms_err, roll_std, period)
    tuples, at most one per sea-state band actually seen (so at most
    len(SEA_STATE_BANDS) == 5), ordered calmest-to-roughest."""
    if t_max <= t_min or window_s <= 0:
        return []
    if edge_margin_s is None:
        edge_margin_s = window_s

    # Reference activity level for the whole log, used to reject
    # near-idle windows below - a single global number, computed once.
    all_tar = np.array([r[1] for r in tune_pids], dtype=float)
    all_tar = all_tar[~np.isnan(all_tar)]
    global_activity = float(np.mean(np.abs(all_tar))) if len(all_tar) else 0.0
    min_activity = global_activity * min_activity_frac

    best = {}  # sea_label -> (rms_err, t_center, roll_std, period)
    n_windows = int((t_max - t_min) // window_s) + 1
    for i in range(n_windows):
        w0 = t_min + i * window_s
        w1 = min(w0 + window_s, t_max)
        if w1 - w0 < window_s * 0.95:
            continue  # nominal window itself truncated (scan range ends mid-window)
        if w0 < t_min + edge_margin_s or w1 > t_max - edge_margin_s:
            continue  # too close to the actual start/end of the scanned range

        att_slice = [r for r in tune_att if w0 <= r[0] < w1]
        if not att_slice:
            continue
        att_times = [r[0] for r in att_slice]
        if (att_times[-1] - att_times[0]) < window_s * min_coverage_frac:
            continue  # attitude data doesn't actually cover this window
        sea_label, roll_std, period = estimate_sea_state(att_slice)
        if sea_label == "not enough data":
            continue

        pids_slice = [r for r in tune_pids if w0 <= r[0] < w1]
        if len(pids_slice) < 20:
            continue
        pids_times = [r[0] for r in pids_slice]
        if (pids_times[-1] - pids_times[0]) < window_s * min_coverage_frac:
            continue  # PID data doesn't actually cover this window either

        tar = np.array([r[1] for r in pids_slice], dtype=float)
        tar = tar[~np.isnan(tar)]
        if len(tar) == 0 or (global_activity > 0 and np.mean(np.abs(tar)) < min_activity):
            continue  # loop was essentially idle here - not a real tracking example

        err = np.array([r[3] for r in pids_slice], dtype=float)
        err = err[~np.isnan(err)]
        if len(err) < 20:
            continue
        rms_err = float(np.sqrt(np.mean(err ** 2)))
        t_center = (w0 + w1) / 2.0
        if sea_label not in best or rms_err < best[sea_label][0]:
            best[sea_label] = (rms_err, t_center, roll_std, period)

    order = [txt for _, txt in SEA_STATE_BANDS]
    results = [
        (best[label][1], label, best[label][0], best[label][2], best[label][3])
        for label in order if label in best
    ]
    return results[:max_examples]


def find_gust_events(tune_att, tune_gps_speed, course_events, t_min, t_max,
                      gust_window_s=8.0, smooth_s=2.0, exclude_sec=60.0,
                      min_separation_sec=120.0, max_examples=5, dt=1.0,
                      min_roll_deg=5.0, min_speed_mps=1.0,
                      min_duration_s=30.0, sustain_frac=0.5):
    """Finds moments of a sudden SIMULTANEOUS increase in heel angle and
    boat speed - the classic signature of a wind gust hitting - while
    excluding anything too close to a course-change event, since turning
    the boat also changes both heel and speed and would otherwise get
    mistaken for a gust.

    Method: both signals are resampled onto a common dt-second grid,
    lightly smoothed to suppress single-sample noise, then a "how much did
    this rise over the last gust_window_s seconds" delta is computed for
    each. Only points where BOTH deltas are positive count as a candidate
    (a real gust raises both at once; a point where only one rises is more
    likely noise, current/wave action, or something else). Candidates are
    ranked by their combined z-scored rise (so a large heel increase with
    a middling speed increase can still outrank a huge heel increase with
    no speed change at all) and greedily selected at least
    min_separation_sec apart, strongest first.

    Only candidates with at least min_roll_deg AND min_speed_mps of rise
    (in addition to the positive-and-away-from-course-changes conditions
    above) qualify - if the boat genuinely never saw more than a couple of
    real gusts, this returns fewer than max_examples rather than padding
    the report out with ordinary noise dressed up as a "gust".

    A candidate also has to actually LAST: for min_duration_s after the
    rise completes, both heel and speed must stay at least sustain_frac of
    the way from their pre-rise baseline to their peak - a brief spike
    that immediately falls back down (a wave slap, a wake crossing, a
    single noisy sample that survived smoothing) doesn't count as a gust
    just because the rise itself looked sharp. A candidate too close to
    the end of the available data to confirm this is discarded rather
    than assumed to qualify.

    Returns up to max_examples (t_peak, delta_roll_deg, delta_speed_mps)
    tuples, strongest gust first."""
    if not tune_att or not tune_gps_speed or t_max <= t_min:
        return []

    grid = np.arange(t_min, t_max, dt)
    if len(grid) < 20:
        return []

    att_t = np.array([r[0] for r in tune_att])
    att_roll = np.array([r[1] for r in tune_att], dtype=float)
    valid = ~np.isnan(att_roll)
    att_t, att_roll = att_t[valid], att_roll[valid]
    if len(att_t) < 10:
        return []
    roll_g = np.interp(grid, att_t, np.abs(att_roll))

    spd_t = np.array([r[0] for r in tune_gps_speed])
    spd_v = np.array([r[1] for r in tune_gps_speed], dtype=float)
    valid = ~np.isnan(spd_v)
    spd_t, spd_v = spd_t[valid], spd_v[valid]
    if len(spd_t) < 10:
        return []
    speed_g = np.interp(grid, spd_t, spd_v)

    smooth_n = max(1, int(round(smooth_s / dt)))
    if smooth_n > 1:
        kernel = np.ones(smooth_n) / smooth_n
        roll_g = np.convolve(roll_g, kernel, mode="same")
        speed_g = np.convolve(speed_g, kernel, mode="same")

    win_n = max(1, int(round(gust_window_s / dt)))
    if win_n >= len(grid):
        return []
    d_roll = np.full(len(grid), np.nan)
    d_speed = np.full(len(grid), np.nan)
    d_roll[win_n:] = roll_g[win_n:] - roll_g[:-win_n]
    d_speed[win_n:] = speed_g[win_n:] - speed_g[:-win_n]

    valid = ~np.isnan(d_roll) & ~np.isnan(d_speed)
    if valid.sum() < 10:
        return []

    def zscore(a):
        m, s = np.nanmean(a), np.nanstd(a)
        return (a - m) / s if s > 1e-9 else np.zeros_like(a)

    z_roll = zscore(d_roll)
    z_speed = zscore(d_speed)
    score = np.where((d_roll >= min_roll_deg) & (d_speed >= min_speed_mps),
                      z_roll + z_speed, -np.inf)

    if course_events:
        ce_t = np.array([e.t for e in course_events])
        near_course = np.array([np.any(np.abs(ce_t - t) < exclude_sec) for t in grid])
        score = np.where(near_course, -np.inf, score)

    # Persistence: the rise has to actually last, not just spike and fall
    # straight back - require both signals to stay at least sustain_frac
    # of the way from baseline to peak for min_duration_s afterward.
    dur_n = max(1, int(round(min_duration_s / dt)))
    for i in range(len(grid)):
        if not np.isfinite(score[i]):
            continue
        if i + dur_n >= len(grid):
            score[i] = -np.inf  # too close to the end of data to confirm duration
            continue
        baseline_roll = roll_g[i - win_n]
        baseline_speed = speed_g[i - win_n]
        roll_floor = baseline_roll + sustain_frac * d_roll[i]
        speed_floor = baseline_speed + sustain_frac * d_speed[i]
        if (np.min(roll_g[i:i + dur_n]) < roll_floor
                or np.min(speed_g[i:i + dur_n]) < speed_floor):
            score[i] = -np.inf  # fell back down too soon - a spike, not a gust

    order = np.argsort(score)[::-1]
    selected = []
    for i in order:
        if not np.isfinite(score[i]):
            break
        t_peak = grid[i]
        if any(abs(t_peak - s[0]) < min_separation_sec for s in selected):
            continue
        selected.append((t_peak, float(d_roll[i]), float(d_speed[i]), float(score[i])))
        if len(selected) >= max_examples:
            break

    return [(t, dr, ds) for t, dr, ds, _ in selected]


def heading_cog_deviation(att_slice, gps_slice, speed_threshold_mps=0.3):
    if not att_slice or not gps_slice:
        return None
    t_att = np.array([r[0] for r in att_slice])
    yaw = np.array([r[3] for r in att_slice], dtype=float)

    gps_arr = [r for r in gps_slice if (r[1] or 0) >= speed_threshold_mps]
    if not gps_arr:
        return None
    t_gps = np.array([r[0] for r in gps_arr])
    gcrs = np.array([r[2] for r in gps_arr], dtype=float)

    yaw_at_gps = interp_angle_deg(t_gps, t_att, yaw)
    diff = wrap180(gcrs - yaw_at_gps)
    diff = diff[~np.isnan(diff)]
    if len(diff) == 0:
        return None
    return {
        "mean_abs": float(np.mean(np.abs(diff))),
        "max_abs": float(np.max(np.abs(diff))),
        "n_samples": len(diff),
    }


# --------------------------------------------------------------------------
# Steering controller (PIDS Tar vs Act) time-domain response analysis
#
# A cross-correlation lag estimate per finding - robust to the fact that
# Tar is a continuously time-varying commanded rate/output, not a clean
# step, so per-step rise/settle times aren't well defined here.
# --------------------------------------------------------------------------

def _resample_uniform(t, y, dt):
    """Interpolates y(t) onto a uniform grid with spacing dt. Returns
    (grid, y_uniform) or (None, None) if there isn't enough data."""
    t = np.asarray(t, dtype=float)
    y = np.asarray(y, dtype=float)
    mask = ~np.isnan(y)
    t, y = t[mask], y[mask]
    if len(t) < 5 or t[-1] <= t[0]:
        return None, None
    grid = np.arange(t[0], t[-1], dt)
    if len(grid) < 5:
        return None, None
    return grid, np.interp(grid, t, y)


def estimate_tar_act_lag(pids_slice, dt=0.05, max_lag_sec=5.0):
    """Cross-correlation lag estimate between Tar (reference) and Act
    (response): finds the lag (>= 0, Act following Tar) that maximizes
    correlation between the two mean-removed, uniformly resampled signals.
    Returns lag in seconds, or None if there isn't enough data."""
    if len(pids_slice) < 5:
        return None
    t = [r[0] for r in pids_slice]
    tar = [r[1] for r in pids_slice]
    act = [r[2] for r in pids_slice]
    grid, tar_u = _resample_uniform(t, tar, dt)
    if grid is None:
        return None
    _, act_u = _resample_uniform(t, act, dt)
    if act_u is None or len(act_u) != len(tar_u):
        # re-resample both against the same grid explicitly to be safe
        t_arr = np.array(t)
        act_arr = np.array(act, dtype=float)
        mask = ~np.isnan(act_arr)
        if mask.sum() < 5:
            return None
        act_u = np.interp(grid, t_arr[mask], act_arr[mask])

    tar_u = tar_u - np.mean(tar_u)
    act_u = act_u - np.mean(act_u)
    if np.allclose(tar_u, 0) or np.allclose(act_u, 0):
        return None

    max_lag = max(1, int(max_lag_sec / dt))
    corr = np.correlate(act_u, tar_u, mode="full")
    lags = np.arange(-len(tar_u) + 1, len(tar_u))
    valid = (lags >= 0) & (lags <= max_lag)
    if not np.any(valid):
        return None
    idx = np.argmax(corr[valid])
    return float(lags[valid][idx]) * dt


def native_dt(t):
    """Median timestamp spacing - the data's own native sample rate, so
    resampling neither throws away resolution nor invents it."""
    if len(t) < 2:
        return 0.02
    dt = float(np.median(np.diff(t)))
    return dt if dt > 0 else 0.02


def choose_nperseg(n_samples, target_segments=32, overlap=0.5,
                    min_nperseg=128, max_nperseg=8192):
    """Picks a Welch segment length that uses as much of the available
    data as feasible: as many averaged (variance-reducing) segments as the
    data supports, up to `target_segments`, without going below
    min_nperseg (frequency resolution) or above max_nperseg (diminishing
    returns, memory). More data (a longer segment of log) -> more Welch
    segments, not a longer segment - extra segments reduce estimate
    variance, which is what we actually want."""
    step_frac = 1.0 - overlap
    nperseg = int(n_samples / (target_segments * step_frac + 1))
    return max(min_nperseg, min(max_nperseg, nperseg))


def welch_psd(y_uniform, nperseg, overlap=0.5):
    """Averaged periodogram (Hann-windowed, overlapping segments) of one
    long uniformly-sampled signal. Returns (psd, n_segments)."""
    n = len(y_uniform)
    if n < nperseg:
        return None, 0
    window = np.hanning(nperseg)
    step = max(1, int(nperseg * (1 - overlap)))
    psd_sum = np.zeros(nperseg // 2 + 1)
    n_seg = 0
    y = y_uniform - np.mean(y_uniform)
    for start in range(0, n - nperseg + 1, step):
        seg = y[start:start + nperseg] * window
        psd_sum += np.abs(np.fft.rfft(seg)) ** 2
        n_seg += 1
    return psd_sum / n_seg, n_seg


# --------------------------------------------------------------------------
# Time-domain checks (sustained error, actuator saturation/stiction,
# step-windup, P/I/FF balance, speed dependence) and the suggestion engine
# live in the inlined "PID/filter tuning analysis engine" section above.
# --------------------------------------------------------------------------

def analyze_pid_tuning(t, tar, act, err, i_term, p_term, ff_term, gyro_t, gyro_y,
                        rcou_t, rcou_pwm, gps_t, gps_speed, parm_history, t_ref, param_prefix,
                        target_segments=32, rc_min_us=1000.0, rc_max_us=2000.0,
                        pwm_margin_us=5.0, speed_bins=3):
    """PID/filter tuning-suggestion engine for ONE tuning segment (a
    stretch of the log where every relevant gain/filter stayed constant -
    see build_tuning_segments()). Unlike the per-finding-window approach
    this replaced, the arrays passed in here cover the FULL log for this
    segment (mode-gated, but not window-gated) - see scan_log()'s
    tune_pids/tune_gyro/tune_rcou/tune_gps_speed collection - so a chronic
    problem that never happens to be a top-N error spike or a course
    change still gets seen. Symptoms measured in the data (oscillation
    frequency + how much of it overlaps sensor noise, tracking error
    level, saturation, wind-up, P/I/FF balance, speed-dependence) are
    mapped to the standard ArduPilot tuning-guide response. This is a
    tuning aid, not an autotuner - change one parameter (or one tightly-
    coupled pair) at a time and re-log.

    Returns (metrics_dict, suggestions, err_freqs, err_psd, gyro_freqs,
    gyro_psd, sustained, cause, step_windup, balance, speed_dep).
    suggestions is a list of (param_name, current, suggested, reason)
    tuples; may be empty if nothing stood out."""

    def cur(suffix):
        return param_value_at(parm_history, param_prefix + suffix, t_ref)

    err_dt = native_dt(t)
    _, err_u = _resample_uniform(t, err, err_dt)
    err_nperseg = choose_nperseg(len(err_u), target_segments) if err_u is not None else 0
    err_freqs = np.fft.rfftfreq(err_nperseg, d=err_dt) if err_nperseg else None
    err_psd, err_n_seg = welch_psd(err_u, err_nperseg) if err_u is not None and err_nperseg else (None, 0)

    dominant_osc_hz, osc_power_frac = 0.0, 0.0
    if err_psd is not None:
        mask = err_freqs > 0.05  # ignore near-DC (heading changes, not oscillation)
        if mask.any() and err_psd[mask].sum() > 0:
            i = np.argmax(err_psd[mask])
            dominant_osc_hz = float(err_freqs[mask][i])
            osc_power_frac = float(err_psd[mask][i] / err_psd[mask].sum())

    gyro_freqs = gyro_psd = None
    gyro_n_seg = 0
    noise_floor_hz = None
    if len(gyro_t) > 10:
        gyro_dt = native_dt(gyro_t)
        _, gyro_u = _resample_uniform(gyro_t, gyro_y, gyro_dt)
        if gyro_u is not None:
            gyro_nperseg = choose_nperseg(len(gyro_u), target_segments)
            gyro_psd, gyro_n_seg = welch_psd(gyro_u, gyro_nperseg)
            gyro_freqs = np.fft.rfftfreq(gyro_nperseg, d=gyro_dt)
            if gyro_n_seg:
                tail = gyro_psd[gyro_freqs > gyro_freqs.max() * 0.3]
                tail_median = np.median(tail) if tail.size else np.median(gyro_psd)
                above = np.where(gyro_psd < tail_median * 3)[0]
                noise_floor_hz = float(gyro_freqs[above[0]]) if len(above) else float(gyro_freqs[-1])

    valid_err = err[~np.isnan(err)]
    rms_error = float(np.sqrt(np.mean(valid_err ** 2))) if len(valid_err) else float("nan")

    core_suffixes = ["P", "I", "D", "FF", "IMAX", "FLTD", "FLTT", "FLTE", "MAX"]
    current_values = {suf: cur(suf) for suf in core_suffixes}

    metrics = {
        "rms_error": rms_error,
        "dominant_osc_hz": dominant_osc_hz,
        "osc_power_frac": osc_power_frac,
        "noise_floor_hz": noise_floor_hz,
        "err_n_seg": err_n_seg,
        "gyro_n_seg": gyro_n_seg,
        "current_values": current_values,
        "params_found": any(v is not None for v in current_values.values()),
    }

    imax = cur("IMAX")
    sustained = analyze_sustained_error(t, err, i_term, imax)
    step_events = find_step_events(t, tar)
    step_windup = analyze_step_windup(t, tar, err, i_term, imax, step_events)
    cause = diagnose_sustained_error_cause(t, act, rcou_t, rcou_pwm, sustained,
                                            rc_min_us, rc_max_us, pwm_margin_us)
    balance = analyze_pid_balance(p_term, i_term, ff_term)

    speed_dep = None
    if len(gps_t) > 10:
        abs_err_at_gps = np.interp(gps_t, t, np.abs(err))
        speed_dep = analyze_speed_dependence_pairs(gps_speed, abs_err_at_gps, speed_bins)

    suggestions = generate_suggestions(cur, param_prefix, metrics, sustained, cause, step_windup, balance)

    return (metrics, suggestions, err_freqs, err_psd, gyro_freqs, gyro_psd,
            sustained, cause, step_windup, balance, speed_dep)


def plot_tuning_diagnostics(metrics, err_freqs, err_psd, gyro_freqs, gyro_psd):
    """Two-panel diagnostic plot backing the tuning suggestions: the
    tracking-error spectrum (with the detected dominant-oscillation peak
    marked) and the gyro noise spectrum (with the detected noise floor
    marked). Purely diagnostic - not a calibrated system-ID result."""
    fig, (ax_err, ax_gyro) = plt.subplots(2, 1, figsize=(8, 6))

    if err_freqs is not None and metrics["err_n_seg"] > 0:
        mask = err_freqs > 0
        ax_err.semilogy(err_freqs[mask], err_psd[mask], color="tab:red", linewidth=1.2)
        if metrics["dominant_osc_hz"] > 0:
            ax_err.axvline(metrics["dominant_osc_hz"], color="black", linestyle="--", linewidth=1,
                            label=f"dominant peak {metrics['dominant_osc_hz']:.2f} Hz")
            ax_err.legend(fontsize=8)
        ax_err.set_title(f"Tracking-error spectrum (aggregate, {metrics['err_n_seg']} segments)")
    else:
        ax_err.text(0.5, 0.5, "not enough data", ha="center", va="center", transform=ax_err.transAxes)
        ax_err.set_title("Tracking-error spectrum - not enough data")
    ax_err.set_ylabel("PSD")
    ax_err.grid(alpha=0.3, which="both")

    if gyro_freqs is not None and metrics["gyro_n_seg"] > 0:
        mask = gyro_freqs > 0
        ax_gyro.semilogy(gyro_freqs[mask], gyro_psd[mask], color="tab:blue", linewidth=1.2)
        if metrics["noise_floor_hz"]:
            ax_gyro.axvline(metrics["noise_floor_hz"], color="black", linestyle="--", linewidth=1,
                             label=f"noise floor {metrics['noise_floor_hz']:.1f} Hz")
            ax_gyro.legend(fontsize=8)
        ax_gyro.set_title(f"Gyro noise spectrum (aggregate, {metrics['gyro_n_seg']} segments)")
    else:
        ax_gyro.text(0.5, 0.5, "not enough IMU data", ha="center", va="center", transform=ax_gyro.transAxes)
        ax_gyro.set_title("Gyro noise spectrum - not enough data")
    ax_gyro.set_ylabel("PSD")
    ax_gyro.set_xlabel("Frequency [Hz]")
    ax_gyro.grid(alpha=0.3, which="both")

    fig.tight_layout()
    return fig_to_base64(fig)


def _time_axis(records_t, t0):
    return [(t - t0) for t in records_t]


def mark_course_events(ax, window_events, t0, course_index, show_label=True):
    """Draws a vertical dotted line for every INDIVIDUAL course-change
    event within this plot's time window - not the merged/combined
    finding, every raw event - so a burst of small corrections that got
    accumulated into one finding for the report's top-N selection still
    shows exactly where and how large each individual turn actually was.
    `course_index` is a {event_t: global_number} map (see build_course_index())
    so the same event carries a consistent #N label across every plot it
    appears in, for cross-referencing. Label text is only drawn when
    show_label is True - for a multi-panel figure, pass True for one panel
    and False for the rest so the lines stay aligned without tripling up
    the text."""
    if not window_events:
        return
    trans = ax.get_xaxis_transform()  # x in data units, y in axes-fraction [0,1]
    for e in window_events:
        xpos = e.t - t0
        ax.axvline(xpos, color="tab:purple", linestyle=":", linewidth=1.0, alpha=0.65, zorder=1)
        if show_label:
            idx = course_index.get(e.t, "?")
            course_str = f"{e.course:+.0f}\u00b0" if e.course is not None else "?"
            ax.text(xpos, 0.98, f"#{idx} {course_str}", transform=trans,
                    rotation=90, va="top", ha="right", fontsize=6.5, color="tab:purple")


def build_course_index(course_events):
    """Global {event_t: N} numbering (1-based, time order) across every
    INDIVIDUAL course-change event in the whole log - used so the same
    event shows the same #N wherever it's marked, letting a reader
    cross-reference the same turn between different findings' plots."""
    return {e.t: i + 1 for i, e in enumerate(sorted(course_events, key=lambda e: e.t))}


def mark_highlight_span(ax, highlight_span_s, label=None, show_label=True):
    """Shades [-highlight_span_s, 0] (relative to t0, i.e. right up to the
    finding's own reference time) - used for a gust's rise window, so the
    period the heel/speed increase was actually measured over is visible
    at a glance instead of just the single peak instant marked by the red
    dashed line. A no-op if highlight_span_s is falsy, so callers can pass
    it unconditionally."""
    if not highlight_span_s:
        return
    ax.axvspan(-highlight_span_s, 0, color="gold", alpha=0.20, zorder=0)
    if show_label and label:
        trans = ax.get_xaxis_transform()
        ax.text(-highlight_span_s / 2.0, 0.98, label, transform=trans,
                ha="center", va="top", fontsize=7, color="#8a6d00")


def plot_yaw_heel_speed(att_slice, gps_slice, t0, window_course_events=None, course_index=None,
                         highlight_span_s=None):
    fig, ax1 = plt.subplots(figsize=(9, 3.6))
    ax2 = ax1.twinx()
    ax3 = ax1.twinx()
    ax3.spines["right"].set_position(("outward", 55))

    if att_slice:
        t_att = _time_axis([r[0] for r in att_slice], t0)
        yaw = [r[3] for r in att_slice]
        heel = [r[1] for r in att_slice]
        l1, = ax1.plot(t_att, yaw, color="tab:blue", linewidth=1.0, label="Yaw (heading) [deg]")
        l2, = ax2.plot(t_att, heel, color="tab:orange", linewidth=1.0, label="Heel (roll) [deg]")
    else:
        l1 = l2 = None

    if gps_slice:
        t_gps = _time_axis([r[0] for r in gps_slice], t0)
        spd_kn = [(r[1] or 0.0) * MPS_TO_KN for r in gps_slice]
        l3, = ax3.plot(t_gps, spd_kn, color="tab:green", linewidth=1.0, label="Speed [kn]")
    else:
        l3 = None

    ax1.axvline(0, color="red", linestyle="--", linewidth=1)
    if window_course_events:
        mark_course_events(ax1, window_course_events, t0, course_index)
    mark_highlight_span(ax1, highlight_span_s, label="Gust rise")
    ax1.set_xlabel("Time relative to finding [s]")
    ax1.set_ylabel("Yaw [deg]", color="tab:blue")
    ax2.set_ylabel("Heel [deg]", color="tab:orange")
    ax3.set_ylabel("Speed [kn]", color="tab:green")
    ax2.set_ylim(-40, 40)
    ax3.set_ylim(0, 7)
    ax1.grid(alpha=0.3)

    lines = [l for l in (l1, l2, l3) if l is not None]
    if lines:
        ax1.legend(lines, [ln.get_label() for ln in lines], loc="upper right", fontsize=8)
    ax1.set_title("Yaw / Heel / Speed (one chart, different scales)")
    return fig_to_base64(fig)


def plot_pid_review(pids_slice, t0, window_course_events=None, course_index=None, highlight_span_s=None):
    """Modelled on ArduPilot's 'PID Review' tool in Mission Planner: top
    target/actual with shaded error, middle P/I/D/FF, bottom error over
    time with rolling RMS."""
    fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True,
                              gridspec_kw={"height_ratios": [1.1, 1.1, 1]})
    ax_ta, ax_terms, ax_err = axes

    if pids_slice:
        t = np.array(_time_axis([r[0] for r in pids_slice], t0))
        tar = np.array([r[1] for r in pids_slice], dtype=float)
        act = np.array([r[2] for r in pids_slice], dtype=float)
        err = np.array([r[3] for r in pids_slice], dtype=float)
        p = np.array([r[4] for r in pids_slice], dtype=float)
        i = np.array([r[5] for r in pids_slice], dtype=float)
        d = np.array([r[6] for r in pids_slice], dtype=float)
        ff = np.array([r[7] for r in pids_slice], dtype=float)

        ax_ta.plot(t, tar, label="Tar", linewidth=1.2)
        ax_ta.plot(t, act, label="Act", linewidth=1.0, alpha=0.85)
        ax_ta.fill_between(t, tar, act, color="grey", alpha=0.25, label="|Tar-Act|")
        ax_ta.axvline(0, color="red", linestyle="--", linewidth=1)
        if window_course_events:
            mark_course_events(ax_ta, window_course_events, t0, course_index, show_label=True)
        mark_highlight_span(ax_ta, highlight_span_s, label="Gust rise", show_label=True)
        ax_ta.set_ylabel("Steering value")
        ax_ta.legend(loc="upper right", fontsize=8)
        ax_ta.set_title("Target/actual + error band")

        ax_terms.plot(t, p, label="P", linewidth=1.0)
        ax_terms.plot(t, i, label="I", linewidth=1.0)
        ax_terms.plot(t, d, label="D", linewidth=1.0)
        ax_terms.plot(t, ff, label="FF", linewidth=1.0)
        ax_terms.axvline(0, color="red", linestyle="--", linewidth=1)
        if window_course_events:
            mark_course_events(ax_terms, window_course_events, t0, course_index, show_label=False)
        mark_highlight_span(ax_terms, highlight_span_s, show_label=False)
        ax_terms.set_ylabel("PID terms")
        ax_terms.legend(loc="upper right", fontsize=8)

        window = max(3, len(err) // 50)
        if len(err) >= window:
            rms = np.sqrt(
                np.convolve(err ** 2, np.ones(window) / window, mode="same")
            )
        else:
            rms = np.abs(err)
        ax_err.plot(t, err, color="tab:red", linewidth=0.8, alpha=0.6, label="Err")
        ax_err.plot(t, rms, color="black", linewidth=1.2, label="rolling RMS(Err)")
        ax_err.axvline(0, color="red", linestyle="--", linewidth=1)
        if window_course_events:
            mark_course_events(ax_err, window_course_events, t0, course_index, show_label=False)
        mark_highlight_span(ax_err, highlight_span_s, show_label=False)
        ax_err.set_ylabel("Err")
        ax_err.set_xlabel("Time relative to finding [s]")
        ax_err.legend(loc="upper right", fontsize=8)

    for a in axes:
        a.grid(alpha=0.3)
    fig.suptitle("PID review")
    fig.tight_layout()
    return fig_to_base64(fig)


def plot_ram_position(rcou_slice, ram_params, t0, window_course_events=None, course_index=None,
                       highlight_span_s=None):
    """Commanded target vs. simulated ram position (same scale, mm) plus
    simulated speed (different scale, mm/s). This is a DERIVED/ESTIMATED
    position, not measured - see simulate_ram_position() docstring."""
    fig, ax1 = plt.subplots(figsize=(9, 3.6))
    ax2 = ax1.twinx()

    sim = simulate_ram_position(rcou_slice, ram_params)
    if len(sim) < 2:
        ax1.text(0.5, 0.5, "not enough RCOU data for ram simulation",
                  ha="center", va="center", transform=ax1.transAxes)
        ax1.set_title("Ram position (simulated - not enough data)")
        return fig_to_base64(fig)

    t = _time_axis([r[0] for r in sim], t0)
    pos = [r[1] for r in sim]
    speed = [r[2] for r in sim]
    target = [r[3] for r in sim]

    l1, = ax1.plot(t, target, color="tab:grey", linewidth=1.0, linestyle="--",
                    label="Commanded target [mm]")
    l2, = ax1.plot(t, pos, color="tab:purple", linewidth=1.4,
                    label="Simulated ram position [mm]")
    l3, = ax2.plot(t, speed, color="tab:cyan", linewidth=0.9, alpha=0.8,
                    label="Simulated speed [mm/s]")

    ax1.axhline(0, color="black", linewidth=0.5, alpha=0.3)
    ax1.axhline(ram_params.travel_mm, color="black", linewidth=0.5, alpha=0.3)
    ax1.axvline(0, color="red", linestyle="--", linewidth=1)
    if window_course_events:
        mark_course_events(ax1, window_course_events, t0, course_index)
    mark_highlight_span(ax1, highlight_span_s, label="Gust rise")
    ax1.set_xlabel("Time relative to finding [s]")
    ax1.set_ylabel("Position [mm]")
    ax1.set_ylim(-10, ram_params.travel_mm + 10)
    ax2.set_ylabel("Speed [mm/s]", color="tab:cyan")
    ax2.set_ylim(-ram_params.max_speed_mms * 1.1, ram_params.max_speed_mms * 1.1)
    ax1.grid(alpha=0.3)
    ax1.legend([l1, l2, l3], [ln.get_label() for ln in (l1, l2, l3)],
               loc="upper right", fontsize=8)
    ax1.set_title("Ram position (simulated from RCOU + controller model - not measured)")
    return fig_to_base64(fig)


def latlon_to_local_xy(lat, lon, lat0, lon0):
    """Simple flat (equirectangular) projection to local meter coordinates,
    accurate enough for the short distances within a finding's time window."""
    m_per_deg_lat = 110540.0
    m_per_deg_lon = 111320.0 * math.cos(math.radians(lat0))
    x = (lon - lon0) * m_per_deg_lon   # east [m]
    y = (lat - lat0) * m_per_deg_lat   # north [m]
    return x, y


def plot_cog_xy(gps_slice, att_slice, t0, window_course_events=None, course_index=None,
                 highlight_span_s=None):
    """Local XY view (meters, east/north) of the track within the window,
    with arrows for course over ground (COG, blue) and compass heading
    (Yaw, orange) at several points along the track -> makes set/leeway
    visually apparent."""
    fig, ax = plt.subplots(figsize=(6.5, 6.5))

    valid = [r for r in gps_slice if r[3] and r[4]]
    if len(valid) < 2:
        ax.text(0.5, 0.5, "not enough GPS data", ha="center", va="center",
                transform=ax.transAxes)
        ax.set_title("XY: course over ground vs. compass heading")
        return fig_to_base64(fig)

    lat0, lon0 = valid[0][3], valid[0][4]
    ts = np.array([r[0] for r in valid])
    gcrs = np.array([r[2] for r in valid], dtype=float)
    xs, ys = [], []
    for _, _, _, lat, lon in valid:
        x, y = latlon_to_local_xy(lat, lon, lat0, lon0)
        xs.append(x)
        ys.append(y)
    xs, ys = np.array(xs), np.array(ys)

    sc = ax.scatter(xs, ys, c=(ts - t0), cmap="viridis", s=18, zorder=3)
    ax.plot(xs, ys, color="grey", linewidth=0.8, alpha=0.6, zorder=2)

    if highlight_span_s:
        # No shaded background to draw here (this is a spatial, not time,
        # plot) - highlight the corresponding stretch of TRACK instead, so
        # the segment the gust rise was measured over is visible.
        span_mask = (ts >= t0 - highlight_span_s) & (ts <= t0)
        if span_mask.sum() >= 2:
            ax.plot(xs[span_mask], ys[span_mask], color="gold", linewidth=4.0,
                    alpha=0.6, zorder=4, solid_capstyle="round")

    idx0 = int(np.argmin(np.abs(ts - t0)))
    ax.scatter([xs[idx0]], [ys[idx0]], color="red", s=110, marker="*",
               zorder=6, label="Finding")

    if window_course_events:
        # No time axis here to draw a vertical line on - mark each
        # INDIVIDUAL course-change event's approximate track position
        # instead, with the same #N/magnitude label used everywhere else.
        for e in window_course_events:
            j = int(np.argmin(np.abs(ts - e.t)))
            idx = (course_index or {}).get(e.t, "?")
            course_str = f"{e.course:+.0f}\u00b0" if e.course is not None else "?"
            ax.scatter([xs[j]], [ys[j]], color="tab:purple", s=55, marker="D", zorder=7)
            ax.annotate(f"#{idx} {course_str}", (xs[j], ys[j]),
                        textcoords="offset points", xytext=(5, 5),
                        fontsize=7, color="tab:purple")

    span = max(xs.max() - xs.min(), ys.max() - ys.min(), 1.0)
    arrow_len = 0.12 * span
    step = max(1, len(xs) // 15)

    yaw_at = None
    if att_slice:
        t_att = np.array([r[0] for r in att_slice])
        yaw = np.array([r[3] for r in att_slice], dtype=float)
        yaw_at = interp_angle_deg(ts, t_att, yaw)

    for i in range(0, len(xs), step):
        u = math.sin(math.radians(gcrs[i]))
        v = math.cos(math.radians(gcrs[i]))
        ax.annotate("", xy=(xs[i] + u * arrow_len, ys[i] + v * arrow_len),
                    xytext=(xs[i], ys[i]),
                    arrowprops=dict(arrowstyle="->", color="tab:blue", lw=1.3))
        if yaw_at is not None and not np.isnan(yaw_at[i]):
            u2 = math.sin(math.radians(yaw_at[i]))
            v2 = math.cos(math.radians(yaw_at[i]))
            ax.annotate("", xy=(xs[i] + u2 * arrow_len * 0.8, ys[i] + v2 * arrow_len * 0.8),
                        xytext=(xs[i], ys[i]),
                        arrowprops=dict(arrowstyle="->", color="tab:orange", lw=1.1))

    from matplotlib.lines import Line2D
    legend_elems = [
        Line2D([0], [0], color="tab:blue", lw=1.3, label="Course over ground (COG)"),
        Line2D([0], [0], color="tab:orange", lw=1.1, label="Compass heading (Yaw)"),
        Line2D([0], [0], marker="*", color="red", lw=0, label="Finding", markersize=10),
    ]
    if window_course_events:
        legend_elems.append(
            Line2D([0], [0], marker="D", color="tab:purple", lw=0,
                   label="Course-change event", markersize=7)
        )
    if highlight_span_s:
        legend_elems.append(
            Line2D([0], [0], color="gold", lw=4, alpha=0.6, label="Gust rise period")
        )
    ax.legend(handles=legend_elems, loc="best", fontsize=8)

    ax.set_xlabel("East [m]")
    ax.set_ylabel("North [m]")
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_title("XY: course over ground (COG) vs. compass heading")
    ax.grid(alpha=0.3)
    cbar = fig.colorbar(sc, ax=ax)
    cbar.set_label("Time relative to finding [s]")
    return fig_to_base64(fig)


def plot_overview_map(all_gps, findings, use_basemap=True):
    """Overview map: full course (kept track segments) with every finding
    marked and numbered, matching the numbering in the findings table.
    Draws a real OpenStreetMap tile basemap in the background when
    `contextily` is available and tiles can be fetched (needs internet
    access at runtime); otherwise falls back to a plain plot."""
    if not all_gps:
        return None
    fig, ax = plt.subplots(figsize=(8, 8))
    lat = np.array([r[1] for r in all_gps])
    lon = np.array([r[2] for r in all_gps])

    # Pad the extent a bit so points/arrows near the edge aren't clipped
    # and the basemap has some margin around the track.
    lat_pad = max((lat.max() - lat.min()) * 0.15, 0.0015)
    lon_pad = max((lon.max() - lon.min()) * 0.15, 0.0015)
    ax.set_xlim(lon.min() - lon_pad, lon.max() + lon_pad)
    ax.set_ylim(lat.min() - lat_pad, lat.max() + lat_pad)

    track_color = "tab:purple"
    ax.plot(lon, lat, color=track_color, linewidth=2.0, alpha=0.9, zorder=3,
            solid_capstyle="round")

    if findings:
        gt = np.array([r[0] for r in all_gps])
        for idx, f in enumerate(findings):
            i = int(np.argmin(np.abs(gt - f.t)))
            _, flat, flon = all_gps[i]
            if f.kind == "course_change":
                color, marker = "tab:blue", "^"
            elif f.kind == "good_example":
                color, marker = "tab:green", "*"
            elif f.kind == "gust":
                color, marker = "tab:orange", "s"
            else:
                color, marker = "tab:red", "o"
            ax.scatter([flon], [flat], color=color, s=100, marker=marker,
                       edgecolor="white", linewidth=1.0, zorder=5)
            ax.annotate(str(idx + 1), (flon, flat), textcoords="offset points",
                        xytext=(6, 6), fontsize=8, fontweight="bold", zorder=6,
                        color="black",
                        bbox=dict(boxstyle="round,pad=0.15", fc="white",
                                  ec="none", alpha=0.75))

    basemap_added = False
    if use_basemap and HAS_CONTEXTILY:
        try:
            cx.add_basemap(ax, crs="EPSG:4326",
                            source=cx.providers.OpenStreetMap.Mapnik,
                            attribution_size=6)
            basemap_added = True
        except Exception as e:
            print(f"      Warning: could not fetch OSM basemap tiles ({e}); "
                  f"falling back to a plain plot.", file=sys.stderr)
            ax.grid(alpha=0.3)
    else:
        ax.grid(alpha=0.3)

    from matplotlib.lines import Line2D
    legend_elems = [
        Line2D([0], [0], color=track_color, lw=2.0, label="Course (kept track segments)"),
        Line2D([0], [0], marker="^", color="tab:blue", lw=0, markersize=9,
               label="Course change", markeredgecolor="white"),
        Line2D([0], [0], marker="o", color="tab:red", lw=0, markersize=9,
               label="PID error", markeredgecolor="white"),
        Line2D([0], [0], marker="*", color="tab:green", lw=0, markersize=12,
               label="Good example", markeredgecolor="white"),
        Line2D([0], [0], marker="s", color="tab:orange", lw=0, markersize=9,
               label="Gust", markeredgecolor="white"),
    ]
    ax.legend(handles=legend_elems, loc="best", fontsize=8, framealpha=0.85)

    ax.set_xlabel("Lon")
    ax.set_ylabel("Lat")
    title = "Overview: course & findings\n(numbers match the findings table)"
    if use_basemap and not basemap_added:
        title += "\n(basemap unavailable - showing plain plot)"
    ax.set_title(title)
    ax.set_aspect("equal", adjustable="datalim")
    fig.tight_layout()
    return fig_to_base64(fig)


# --------------------------------------------------------------------------
# HTML report
# --------------------------------------------------------------------------

def build_html_report(path, findings, per_finding, overview_img,
                       parm_history, display_params, param_columns,
                       tuning_segments,
                       stats, args):
    def finding_summary(f):
        """Short, clean summary used both in the table and the section
        header - never shows the raw STATUSTEXT."""
        if f.kind == "course_change":
            suffix = f" (accumulated x{f.merged_count})" if f.merged_count > 1 else ""
            return f"{f.course:+.1f}&deg; &rarr; {f.heading:.1f}&deg;{suffix}"
        if f.kind == "good_example":
            return f"RMS |err| = {f.value:.4f}"
        if f.kind == "gust":
            return f.label.replace("Gust: ", "")
        return f"|Err| = {f.value:.4f}"

    def finding_type_label(f):
        if f.kind == "course_change":
            return "Course change"
        if f.kind == "good_example":
            return "Good example"
        if f.kind == "gust":
            return "Gust"
        return "PID error"

    rows = []
    for idx, f in enumerate(findings):
        type_label = finding_type_label(f)
        rows.append(
            f"<tr><td><a href='#f{idx}'>{idx+1}</a></td>"
            f"<td>{h(type_label)}</td>"
            f"<td>{h(fmt_time(f.t))}</td>"
            f"<td>{finding_summary(f)}</td></tr>"
        )

    sections = []
    for idx, f in enumerate(findings):
        pf = per_finding[idx]
        sea_label, roll_std, period = pf["sea_state"]
        dev = pf["deviation"]

        dev_html = "<p><em>Not enough GPS/Yaw data for a deviation estimate.</em></p>"
        if dev is not None:
            warn = ""
            if dev["mean_abs"] >= args.deviation_threshold_deg:
                warn = (f"<p style='color:#b00'><strong>Warning:</strong> mean deviation "
                        f"between compass heading and COG of {dev['mean_abs']:.1f}&deg; "
                        f"exceeds the threshold of {args.deviation_threshold_deg}&deg; "
                        f"(possible: current/set, leeway, compass error).</p>")
            dev_html = (
                f"<p>Compass heading vs. course over ground (COG): "
                f"mean dev. {dev['mean_abs']:.1f}&deg;, max {dev['max_abs']:.1f}&deg; "
                f"({dev['n_samples']} samples).</p>{warn}"
            )

        sea_html = f"<p>Sea-state estimate (heuristic): <strong>{h(sea_label)}</strong>"
        if roll_std is not None:
            sea_html += f" (roll spread {roll_std:.2f}&deg;"
            if period:
                sea_html += f", roll period ~{period:.1f}s"
            sea_html += ")"
        sea_html += "</p>"

        lag_s = pf.get("tar_act_lag_s")
        if lag_s is not None:
            lag_html = (f"<p>Estimated response lag, Tar&rarr;Act "
                        f"(cross-correlation): <strong>{lag_s*1000:.0f} ms</strong></p>")
        else:
            lag_html = "<p><em>Not enough PIDS data for a response-lag estimate.</em></p>"

        extra = ""
        if f.kind == "course_change":
            label = "Accumulated course change" if f.merged_count > 1 else "Course change"
            extra = f"<p>{label}: {f.course:+.1f}&deg;, new heading: {f.heading:.1f}&deg;</p>"
        elif f.kind == "good_example":
            extra = (f"<p style='color:#0a0'><strong>Good example</strong> - best-tracking "
                     f"window found for this sea state. {h(f.detail)}</p>")
        elif f.kind == "gust":
            extra = (f"<p style='color:#b6720a'><strong>Gust</strong> - {h(f.detail)}</p>")
        else:
            extra = f"<p>|Err| = {f.value:.4f}</p>"

        if idx > 0:
            gap = f.t - findings[idx - 1].t
            time_note = f"<p class=\"time-note\">{format_duration(gap)} after last error/course change</p>"
        else:
            gap = f.t - stats["t_min"]
            time_note = f"<p class=\"time-note\">First finding in this log ({format_duration(gap)} after log start)</p>"

        type_label = finding_type_label(f)
        param_grid_html = build_param_grid_html(parm_history, display_params, f.t, param_columns)

        sections.append(f"""
        <section id="f{idx}" class="finding">
          <button class="export-btn export-ignore"
                  onclick="exportSectionAsImage('f{idx}', 'finding_{idx+1}.png', this)">
            Export as image
          </button>
          <button class="export-btn export-ignore"
                  onclick="exportSectionAsImageCompact('f{idx}', 'finding_{idx+1}_small.jpg', this)">
            Export (&lt;400KB)
          </button>
          <h2>#{idx+1} &ndash; {h(type_label)}
              &nbsp;<span class="ts">{h(fmt_time(f.t))}</span></h2>
          {param_grid_html}
          {time_note}
          {extra}
          {sea_html}
          {dev_html}
          <div class="plots">
            <img src="data:image/png;base64,{pf['img_yaw_heel_speed']}" alt="Yaw Heel Speed">
            <img src="data:image/png;base64,{pf['img_pid_review']}" alt="PID review">
            {lag_html}
            <img src="data:image/png;base64,{pf['img_cog_xy']}" alt="COG XY">
            <img src="data:image/png;base64,{pf['img_ram_position']}" alt="Ram position">
          </div>
        </section>""")

    overview_html = ""
    if overview_img:
        overview_html = f"""
        <section>
          <h2>Overview</h2>
          <img src="data:image/png;base64,{overview_img}" alt="Overview map" style="max-width:700px;">
        </section>"""

    def render_tuning_segment_html(seg, heading):
        tuning_metrics = seg["metrics"]
        tuning_suggestions = seg["suggestions"]
        tuning_plot_img = seg["plot_img"]
        tuning_sustained = seg["sustained"]
        tuning_cause = seg["cause"]
        tuning_step_windup = seg["step_windup"]
        tuning_balance = seg["balance"]
        tuning_speed_dep = seg["speed_dep"]
        param_prefix = seg["prefix"]

        rows_html = ""
        if tuning_suggestions:
            for name, cur_val, new_val, reason in tuning_suggestions:
                cur_s = "n/a" if cur_val is None else f"{cur_val:.5g}"
                new_s = "n/a" if new_val is None else f"{new_val:.5g}"
                rows_html += (
                    f"<tr><td><code>{h(name)}</code></td><td>{cur_s}</td>"
                    f"<td>{new_s}</td><td>{h(reason)}</td></tr>"
                )
            suggestions_html = f"""
            <table>
            <tr><th>Parameter</th><th>Current</th><th>Suggested</th><th>Reasoning</th></tr>
            {rows_html}
            </table>
            <p class="time-note">Heuristic, not an autotuner - change one parameter (or one
            tightly-coupled pair, e.g. P+D together) at a time and re-log before making the
            next change.</p>"""
        else:
            suggestions_html = ("<p><em>No strong symptoms detected across the findings in this "
                                 "segment - current tune looks reasonable for what was logged.</em></p>")

        cv = tuning_metrics["current_values"]
        cv_html = ", ".join(f"<code>{h(k)}</code>={_format_param_value(v)}" for k, v in cv.items())
        noise_floor_str = ("n/a" if tuning_metrics["noise_floor_hz"] is None
                            else f"{tuning_metrics['noise_floor_hz']:.2f} Hz")
        diagnostics_html = f"""
        <p class="time-note">
          Raw numbers this segment's suggestions (or lack of them) were computed from -
          dominant oscillation {tuning_metrics['dominant_osc_hz']:.2f} Hz
          ({tuning_metrics['osc_power_frac']*100:.0f}% of error spectral power),
          gyro noise floor {noise_floor_str}, RMS tracking error {tuning_metrics['rms_error']:.4f},
          {tuning_metrics['err_n_seg']} error / {tuning_metrics['gyro_n_seg']} gyro Welch segments.
          Parameter values used: {cv_html}.
        </p>"""
        if not tuning_metrics["params_found"]:
            diagnostics_html += (
                f"<p style='color:#b00'><strong>Warning:</strong> none of this segment's "
                f"<code>{h(param_prefix)}*</code> parameters were found in the log's recorded "
                f"parameters, so every suggestion check above was skipped for this segment "
                f"(they all require a current value to compare against). Check "
                f"<code>--param-prefix</code> matches your PID loop's actual parameter names - "
                f"it was guessed as <code>{h(param_prefix)}</code> from "
                f"<code>--pid-msgtype {h(args.pid_msgtype)}</code>.</p>"
            )

        plot_html = ""
        if tuning_plot_img:
            plot_html = (f'<img src="data:image/png;base64,{tuning_plot_img}" '
                         f'alt="Tuning diagnostics" style="max-width:700px;">')

        sustained_html = "<p><em>No episode of persistently large |error| found.</em></p>"
        if tuning_sustained is not None:
            s = tuning_sustained
            sustained_html = (
                f"<p>Longest/worst sustained-error episode: <strong>{s['duration']:.1f}s</strong> "
                f"starting at t={s['t_start']:.1f}s (mean |err| {s['mean_abs_err']:.3f}, "
                f"peak {s['max_abs_err']:.3f}).</p>"
            )
            if s["i_frac_of_imax"] is not None:
                sustained_html += (
                    f"<p>I-term during episode: averaged {s['i_frac_of_imax']*100:.0f}% of IMAX, "
                    f"pinned within 5% of IMAX for {s['i_saturated_frac']*100:.0f}% of it.</p>"
                )
            if tuning_cause is not None:
                if tuning_cause["verdict"] == "actuator_saturated":
                    sustained_html += (
                        f"<p style='color:#b00'><strong>Cause:</strong> actuator output was pinned "
                        f"at its travel limit {tuning_cause['saturated_frac']*100:.0f}% of the "
                        f"episode - this is a PHYSICAL limit, not a PID gain problem.</p>"
                    )
                elif tuning_cause["verdict"] == "possible_stiction":
                    sustained_html += (
                        f"<p style='color:#b00'><strong>Cause:</strong> actuator was actively "
                        f"commanded (RCOU std {tuning_cause['rcou_std_win']:.0f} vs "
                        f"{tuning_cause['rcou_std_all']:.0f} baseline) but Act barely moved "
                        f"(std {tuning_cause['act_std_win']:.3f} vs {tuning_cause['act_std_all']:.3f} "
                        f"baseline) - looks like mechanical stiction/backlash, not a PID problem.</p>"
                    )

        windup_html = "<p><em>No clear step-like course-change commands detected.</em></p>"
        if tuning_step_windup:
            n_opp = sum(1 for e in tuning_step_windup if e["opposing"])
            lags = [e["lag_s"] for e in tuning_step_windup if e["lag_s"] is not None]
            windup_html = (
                f"<p>{len(tuning_step_windup)} step events detected, <strong>{n_opp}</strong> with "
                f"the I-term already opposing the new command beforehand."
            )
            if lags:
                windup_html += (f" Response-onset lag (median/max): "
                                 f"{np.median(lags):.2f}s / {np.max(lags):.2f}s.")
            windup_html += "</p>"

        balance_html = "<p><em>Not enough data for a P/I/FF balance estimate.</em></p>"
        if tuning_balance is not None:
            b = tuning_balance
            balance_html = (
                f"<p>Mean |output| share: P {b['p_share']*100:.0f}%, I {b['i_share']*100:.0f}%, "
                f"FF {b['ff_share']*100:.0f}%.</p>"
            )

        speed_html = "<p><em>No GPS speed data available.</em></p>"
        if tuning_speed_dep is not None:
            rows = "".join(
                f"<tr><td>{bn['speed_lo']:.1f}&ndash;{bn['speed_hi']:.1f} m/s</td>"
                f"<td>{bn['rms_err']:.3f}</td><td>{bn['n']}</td></tr>"
                for bn in tuning_speed_dep["bins"]
            )
            corr = tuning_speed_dep["correlation"]
            corr_note = ""
            if corr is not None:
                trend = ("worse at higher speed" if corr > 0.2 else
                         "worse at lower speed" if corr < -0.2 else "no strong trend")
                corr_note = f"<p>Correlation(speed, |err|): {corr:+.2f} ({trend}).</p>"
            speed_html = (f"<table><tr><th>Speed bucket</th><th>RMS |err|</th><th>N</th></tr>"
                          f"{rows}</table>{corr_note}"
                          f"<p class='time-note'>Informational only - a fixed-gain PID has no way "
                          f"to compensate for this itself.</p>")

        return f"""
        <section>
          <h2>{heading}</h2>
          <p>Uses the WHOLE log's data for this segment (mode-gated to {h(args.modes)}, but not
          restricted to the findings below) - a chronic problem that never happens to be a
          course-change or a top-N error spike is still caught here. RMS tracking error:
          {tuning_metrics['rms_error']:.4f}.</p>
          {suggestions_html}
          {diagnostics_html}
          {plot_html}
          <h3>Sustained tracking error (Tar/Act gap held over time)</h3>
          {sustained_html}
          <h3>Step-response / I wind-up (course-change responsiveness)</h3>
          {windup_html}
          <h3>P / I / FF balance</h3>
          {balance_html}
          <h3>Speed-dependent tracking error</h3>
          {speed_html}
        </section>"""

    tuning_html = ""
    if tuning_segments:
        if len(tuning_segments) == 1:
            tuning_html = render_tuning_segment_html(
                tuning_segments[0], "PID / filter tuning suggestions"
            )
        else:
            change_rows = "".join(
                f"<li>t={c[0]:.1f}s: <code>{h(c[1])}</code> "
                f"{_format_param_value(c[2])} &rarr; {_format_param_value(c[3])}</li>"
                for seg in tuning_segments for c in seg["changes"]
            )
            intro_html = f"""
            <section>
              <h2>PID / filter tuning suggestions</h2>
              <p>Tuning parameters changed mid-log, so the analysis below is split into
              {len(tuning_segments)} segments - each one only uses data and parameter values
              from while that particular tune was actually active, instead of mixing data
              from before and after a gain change into one misleading average.</p>
              <ul>{change_rows}</ul>
            </section>"""
            tuning_html = intro_html + "".join(
                render_tuning_segment_html(
                    seg,
                    f"Segment {i+1}/{len(tuning_segments)}: "
                    f"{fmt_time(seg['start'])} &ndash; {fmt_time(seg['end'])} "
                    f"({format_duration(seg['end'] - seg['start'])})"
                )
                for i, seg in enumerate(tuning_segments)
            )

    html = f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>ArduPilot Log Report &ndash; {h(os.path.basename(path))}</title>
<style>
  body {{ font-family: -apple-system, Segoe UI, Roboto, Arial, sans-serif; margin: 2rem; color: #222; }}
  h1 {{ margin-bottom: 0.2rem; }}
  .meta {{ color: #555; font-size: 0.9rem; margin-bottom: 1.5rem; }}
  table {{ border-collapse: collapse; margin-bottom: 2rem; }}
  th, td {{ border: 1px solid #ccc; padding: 4px 10px; font-size: 0.9rem; text-align: left; }}
  th {{ background: #f0f0f0; }}
  section.finding {{ border-top: 2px solid #ddd; padding-top: 1rem; margin-top: 2rem; }}
  .ts {{ font-weight: normal; color: #666; font-size: 0.85rem; }}
  .time-note {{ color: #777; font-size: 0.85rem; font-style: italic; margin: 0.2rem 0 0.6rem 0; }}
  .plots img {{ max-width: 100%; display: block; margin-bottom: 1rem; border: 1px solid #ddd; }}
  code {{ background: #f0f0f0; padding: 1px 4px; }}
  .param-grid {{
    display: grid;
    gap: 2px 20px;
    background: rgba(0,0,0,0.035);
    border: 1px solid #e0e0e0;
    border-radius: 4px;
    padding: 6px 12px;
    font-family: Consolas, Menlo, monospace;
    font-size: 0.76rem;
    color: #444;
    margin: 0.3rem 0 0.8rem 0;
    width: fit-content;
  }}
  .param-grid .pname {{ color: #888; }}
  .export-btn {{
    float: right;
    font-size: 0.78rem;
    padding: 3px 10px;
    margin-top: 2px;
    margin-left: 6px;
    border: 1px solid #bbb;
    border-radius: 4px;
    background: #f7f7f7;
    color: #333;
    cursor: pointer;
  }}
  .export-btn:hover {{ background: #ececec; }}
  .export-btn:disabled {{ opacity: 0.6; cursor: default; }}
</style>
</head>
<body>
<h1>ArduPilot Log Report</h1>
<div class="meta">
  Source: <code>{h(os.path.basename(path))}</code><br>
  Time range: {h(fmt_time(stats['t_min']))} &ndash; {h(fmt_time(stats['t_max']))}<br>
  Messages read: {stats['n_in']:,} &nbsp;|&nbsp; written (filtered): {stats['n_out']:,}
  ({stats['reduction_pct']:.1f}% reduction)<br>
  Window per finding: &plusmn;{args.window_min:.1f} min &nbsp;|&nbsp;
  Top-N PID errors: {args.top_n} &nbsp;|&nbsp;
  Course changes found: {stats['n_course']}<br>
  Flight modes considered: <strong>{h(stats['modes'])}</strong>
  (findings outside these modes are ignored)<br>
  GPS fixes rejected as implausible: {stats['n_gps_rejected']}/{stats['n_gps_total']}
  ({stats['n_gps_rejected_far']} too far from median position &gt; {stats['max_distance_km']:.0f} km,
  {stats['n_gps_rejected_jump']} speed jumps &gt; {stats['max_speed_kn']:.1f} kn)
</div>

<h2>Findings</h2>
<table>
<tr><th>#</th><th>Type</th><th>Time</th><th>Detail</th></tr>
{''.join(rows)}
</table>

{overview_html}

{tuning_html}

{''.join(sections)}

<script>
{_get_html2canvas_js()}
</script>
<script>
function exportSectionAsImage(elementId, filename, btnEl) {{
  var el = document.getElementById(elementId);
  if (!el) return;
  var originalText = btnEl ? btnEl.textContent : null;
  if (btnEl) {{ btnEl.textContent = 'Exporting...'; btnEl.disabled = true; }}
  html2canvas(el, {{
    backgroundColor: '#ffffff',
    scale: 2,
    ignoreElements: function(node) {{
      return node.classList && node.classList.contains('export-ignore');
    }}
  }}).then(function(canvas) {{
    var link = document.createElement('a');
    link.download = filename;
    link.href = canvas.toDataURL('image/png');
    link.click();
  }}).catch(function(err) {{
    alert('Export failed: ' + err);
  }}).finally(function() {{
    if (btnEl) {{ btnEl.textContent = originalText; btnEl.disabled = false; }}
  }});
}}

function _dataUrlBytes(dataUrl) {{
  var b64 = dataUrl.substring(dataUrl.indexOf(',') + 1);
  var padding = b64.endsWith('==') ? 2 : (b64.endsWith('=') ? 1 : 0);
  return Math.floor(b64.length * 3 / 4) - padding;
}}

function exportSectionAsImageCompact(elementId, filename, btnEl, maxBytes) {{
  maxBytes = maxBytes || 400 * 1024;
  var el = document.getElementById(elementId);
  if (!el) return;
  var originalText = btnEl ? btnEl.textContent : null;
  if (btnEl) {{ btnEl.textContent = 'Compressing...'; btnEl.disabled = true; }}

  function ignoreExport(node) {{
    return node.classList && node.classList.contains('export-ignore');
  }}

  // Re-encoding a canvas at a different JPEG quality is cheap (no DOM
  // re-render); only re-rendering at a smaller scale is expensive - so
  // try every quality step on one render before falling back to a
  // smaller scale and repeating.
  function bestAtScale(scale) {{
    return html2canvas(el, {{backgroundColor: '#ffffff', scale: scale, ignoreElements: ignoreExport}})
      .then(function(canvas) {{
        var qualities = [0.9, 0.75, 0.6, 0.45, 0.3, 0.15];
        var best = null;
        for (var i = 0; i < qualities.length; i++) {{
          var url = canvas.toDataURL('image/jpeg', qualities[i]);
          var bytes = _dataUrlBytes(url);
          best = {{url: url, bytes: bytes}};
          if (bytes <= maxBytes) return best;
        }}
        return best;  // smallest we could get at this scale, still over budget
      }});
  }}

  var scales = [2, 1.5, 1, 0.75, 0.5];
  var i = 0;
  function tryNext() {{
    var scale = scales[i++];
    var isLastScale = (i >= scales.length);
    bestAtScale(scale).then(function(result) {{
      if (result.bytes <= maxBytes || isLastScale) {{
        var link = document.createElement('a');
        link.download = filename;
        link.href = result.url;
        link.click();
        if (result.bytes > maxBytes) {{
          alert('Could not get this section under ' + Math.round(maxBytes/1024) +
                'KB even at reduced resolution/quality - downloaded the smallest ' +
                'version achieved instead (' + Math.round(result.bytes/1024) + 'KB).');
        }}
        if (btnEl) {{ btnEl.textContent = originalText; btnEl.disabled = false; }}
      }} else {{
        tryNext();
      }}
    }}).catch(function(err) {{
      alert('Export failed: ' + err);
      if (btnEl) {{ btnEl.textContent = originalText; btnEl.disabled = false; }}
    }});
  }}
  tryNext();
}}
</script>

</body>
</html>"""
    return html


# --------------------------------------------------------------------------
# Main
# --------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(
        description="Filters ArduPilot .bin logs and generates an HTML report.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    ap.add_argument("logfile", help="Path to the ArduPilot .bin dataflash log")
    ap.add_argument("--outdir", default=None,
                     help="Output directory (default: same directory as logfile)")
    ap.add_argument("--window-min", type=float, default=2.0,
                     help="Time window in minutes around each finding (default 2.0 = -2 to +2 min)")
    ap.add_argument("--top-n", type=int, default=10,
                     help="Number of largest PID error findings (default 10)")
    ap.add_argument("--pid-msgtype", default="PIDS",
                     help="Name of the steering PID log message (default PIDS)")
    ap.add_argument("--modes", default="GUIDED,AUTO",
                     help="Comma-separated list of allowed flight modes for "
                          "finding selection (default GUIDED,AUTO)")
    ap.add_argument("--course-exclude-sec", type=float, default=None,
                     help="Distance to a course change that excludes a PID error "
                          "(default: same as --window-min, i.e. a PID error inside "
                          "the course-change context window is ignored)")
    ap.add_argument("--min-separation-sec", type=float, default=60.0,
                     help="Minimum spacing between selected PID error findings (default 60s)")
    ap.add_argument("--deviation-threshold-deg", type=float, default=15.0,
                     help="Threshold for the heading/COG deviation warning (default 15 deg)")
    ap.add_argument("--no-basemap", action="store_true",
                     help="Disable fetching an OpenStreetMap tile basemap for the "
                          "overview map (no internet access needed, plain plot instead)")
    ap.add_argument("--max-speed-kn", type=float, default=20.0,
                     help="Maximum plausible speed over ground in knots. A GPS fix "
                          "implying a higher speed jump from the previous valid fix "
                          "is treated as a position error and discarded entirely "
                          "(default 20 kn, generous for a small sailboat)")
    ap.add_argument("--course-merge-sec", type=float, default=10.0,
                     help="Course-change events within this many seconds of each other "
                          "are accumulated into a single finding (default 10s, 0 disables)")
    ap.add_argument("--max-distance-km", type=float, default=50.0,
                     help="Maximum plausible distance in km from the robust median "
                          "GPS position of the whole log. Fixes farther away are "
                          "gross errors and are discarded outright (default 50 km)")
    ap.add_argument("--max-good-examples", type=int, default=5,
                     help="Max number of 'good example' findings to add - the best-"
                          "tracking window found for each distinct sea state actually "
                          "experienced in the log, as a positive counterpart to the "
                          "course-change/PID-error findings (default 5, i.e. one per "
                          "sea-state band)")
    ap.add_argument("--good-example-min-coverage", type=float, default=0.9,
                     help="A candidate window for a 'good example' needs attitude and PID "
                          "data spanning at least this fraction of the window's nominal "
                          "duration - rejects windows at the very start/end of the log (or "
                          "around a mode gap) where the data is actually sparse/incomplete "
                          "(default 0.9)")
    ap.add_argument("--good-example-min-activity", type=float, default=0.15,
                     help="A candidate window needs mean |commanded steering rate| at least "
                          "this fraction of the whole log's typical level - rejects windows "
                          "where the loop was essentially idle (e.g. right after arming) and "
                          "would otherwise look artificially 'perfect' for having nothing "
                          "real to track (default 0.15)")
    ap.add_argument("--good-example-edge-margin-min", type=float, default=None,
                     help="Skip 'good example' candidate windows within this many minutes of "
                          "the actual start/end of the scanned (mode-gated) log range - the "
                          "log's true edges are commonly a transition in/out of the scanned "
                          "mode and often look spuriously 'good' from incomplete data, not "
                          "genuinely good tracking (default: one full candidate window's "
                          "width at each end, i.e. 2*--window-min)")
    ap.add_argument("--no-good-examples", action="store_true",
                     help="Disable the 'good example' findings")
    ap.add_argument("--max-gust-examples", type=int, default=5,
                     help="Max number of gust findings to add - moments of a sudden "
                          "simultaneous rise in heel angle and boat speed, away from any "
                          "course change (default 5)")
    ap.add_argument("--gust-window-sec", type=float, default=8.0,
                     help="Time window over which the heel/speed rise is measured for gust "
                          "detection (default 8s)")
    ap.add_argument("--gust-exclude-sec", type=float, default=60.0,
                     help="A candidate gust within this many seconds of a course-change "
                          "event is discarded - turning also raises heel and speed and "
                          "would otherwise be mistaken for a gust (default 60s)")
    ap.add_argument("--gust-min-roll-deg", type=float, default=5.0,
                     help="Minimum heel-angle rise (over --gust-window-sec) to count as a "
                          "gust candidate at all, not just noise (default 5 deg)")
    ap.add_argument("--gust-min-speed-mps", type=float, default=1.0,
                     help="Minimum speed rise (over --gust-window-sec) to count as a gust "
                          "candidate at all, not just noise (default 1.0 m/s)")
    ap.add_argument("--gust-min-duration-sec", type=float, default=30.0,
                     help="A gust candidate has to stay elevated for at least this long "
                          "after its rise completes, not just spike and fall straight back "
                          "(default 30s)")
    ap.add_argument("--gust-sustain-frac", type=float, default=0.5,
                     help="How elevated heel/speed must stay during --gust-min-duration-sec, "
                          "as a fraction of the full rise from baseline to peak (default 0.5, "
                          "i.e. must stay at least halfway up)")
    ap.add_argument("--no-gust-examples", action="store_true",
                     help="Disable the gust findings")
    ram_group = ap.add_argument_group(
        "ram position simulation",
        "Derives an estimated ram position from the RCOU channel driving the "
        "actuator's ESP32 position controller, by replaying that controller's "
        "logic offline. NOT measured telemetry - see plot_ram_position() docs. "
        "Defaults mirror the ESP32 sketch's constants.",
    )
    ram_group.add_argument("--ram-channel", default="C1",
                            help="RCOU field name for the ram/steering output channel "
                                 "(default C1)")
    ram_group.add_argument("--ram-rc-min-us", type=float, default=1000.0,
                            help="RC_MIN_US from the controller (default 1000)")
    ram_group.add_argument("--ram-rc-max-us", type=float, default=2000.0,
                            help="RC_MAX_US from the controller (default 2000)")
    ram_group.add_argument("--ram-travel-mm", type=float, default=250.0,
                            help="TRAVEL_MM from the controller (default 250)")
    ram_group.add_argument("--ram-max-speed-mms", type=float, default=50.0,
                            help="MAX_SPEED_MMS from the controller (default 50)")
    ram_group.add_argument("--ram-max-accel-mmss", type=float, default=250.0,
                            help="MAX_ACCEL_MMSS from the controller (default 250)")
    ram_group.add_argument("--ram-deadband-mm", type=float, default=2.0,
                            help="POS_DEADBAND_MM from the controller (default 2.0)")
    ram_group.add_argument("--ram-kp", type=float, default=5.0,
                            help="kP from the controller's proportional speed request "
                                 "(default 5.0)")
    ram_group.add_argument("--ram-min-speed-mms", type=float, default=20.0,
                            help="Firmware's minimum-move-speed floor once moving "
                                 "(default 20.0)")

    tune_group = ap.add_argument_group(
        "parameter grid & tuning suggestions",
        "Small 'current parameters' grid shown at the top of every finding, and "
        "an aggregate PID/filter tuning-suggestion section - see analyze_pid_tuning() "
        "docs. Heuristic, not an autotuner.",
    )
    tune_group.add_argument("--params", default=",".join(DEFAULT_DISPLAY_PARAMS),
                             help="Comma-separated ArduPilot parameter names shown in the "
                                  "per-finding grid (default: see DEFAULT_DISPLAY_PARAMS)")
    tune_group.add_argument("--param-columns", type=int, default=3,
                             help="Number of columns in the per-finding parameter grid (default 3)")
    tune_group.add_argument("--param-prefix", default=None,
                             help="Parameter-name prefix for the tuning-suggestion engine "
                                  "(default: guessed from --pid-msgtype)")
    tune_group.add_argument("--gyro-field", default="GyrZ",
                             help="IMU field used for the gyro noise-floor analysis "
                                  "(default GyrZ - yaw axis, relevant for steering)")
    tune_group.add_argument("--gyro-instance", type=int, default=0,
                             help="IMU instance index for the gyro noise-floor analysis (default 0)")
    tune_group.add_argument("--pwm-margin-us", type=float, default=5.0,
                             help="How close to --ram-rc-min-us/--ram-rc-max-us counts as "
                                  "'pinned' for the actuator-saturation/stiction check (default 5)")
    tune_group.add_argument("--speed-bins", type=int, default=3,
                             help="Number of GPS-speed buckets for the speed-dependence check (default 3)")
    tune_group.add_argument("--target-segments", type=int, default=32,
                             help="Target number of averaged Welch periodogram segments per tuning "
                                  "segment - more data (a longer stretch of matching tune) buys more "
                                  "segments (lower-variance estimate) rather than a longer segment, "
                                  "up to this target (default 32)")
    tune_group.add_argument("--min-tuning-segment-min", type=float, default=5.0,
                             help="Skip tuning segments (stretches between parameter changes) shorter "
                                  "than this many minutes - too little data for a reliable spectral/"
                                  "statistical read, and would otherwise just add noisy near-empty "
                                  "sections to the report (default 5.0)")
    tune_group.add_argument("--no-tuning-suggestions", action="store_true",
                             help="Disable the aggregate PID/filter tuning-suggestion section")
    args = ap.parse_args()

    if not os.path.isfile(args.logfile):
        print(f"File not found: {args.logfile}", file=sys.stderr)
        sys.exit(1)

    outdir = args.outdir or os.path.dirname(os.path.abspath(args.logfile)) or "."
    os.makedirs(outdir, exist_ok=True)
    base = os.path.splitext(os.path.basename(args.logfile))[0]
    out_bin = os.path.join(outdir, f"{base}_filtered.bin")
    out_html = os.path.join(outdir, f"{base}_report.html")

    allowed_modes = [m.strip() for m in args.modes.split(",") if m.strip()]
    print(f"[1/4] Scanning log: {args.logfile} (modes only: {', '.join(allowed_modes)})")
    (course_events, pid_samples, gps_positions, t_min, t_max, n_in,
     mode_segments, parm_history, tune_pids, tune_gyro, tune_rcou, tune_gps_speed, tune_att) = scan_log(
        args.logfile, args.pid_msgtype, allowed_modes,
        gyro_field=args.gyro_field, gyro_instance=args.gyro_instance, ram_channel=args.ram_channel,
    )
    print(f"      -> {len(course_events)} course-change events, "
          f"{len(pid_samples)} PID samples (only in {', '.join(allowed_modes)}), "
          f"{n_in:,} messages total")
    print(f"      -> tuning-engine data (whole log, mode-gated): {len(tune_pids)} {args.pid_msgtype} "
          f"samples, {len(tune_gyro)} IMU samples, {len(tune_rcou)} RCOU samples, "
          f"{len(tune_gps_speed)} GPS samples")
    _prefix_check = args.param_prefix or PARAM_PREFIX_GUESS.get(args.pid_msgtype, "")
    if _prefix_check:
        _check_names = [_prefix_check + s for s in
                         ["P", "I", "D", "FF", "IMAX", "FLTD", "FLTT", "FLTE", "MAX"]]
        _found = [n for n in _check_names if n in parm_history]
        print(f"      -> {len(parm_history)} distinct parameters found in this log's PARM "
              f"history; {len(_found)}/{len(_check_names)} of {_prefix_check}* found: "
              f"{', '.join(_found) if _found else '(none)'}")
    if mode_segments:
        mode_summary = ", ".join(f"{m}@{fmt_time(t)}" for m, t in mode_segments[:12])
        print(f"      Mode history (first changes): {mode_summary}")

    median_lat, median_lon = median_position(gps_positions)
    if median_lat is not None:
        print(f"      -> robust median GPS position: {median_lat:.5f}, {median_lon:.5f} "
              f"(from {len(gps_positions)} valid fixes)")
    else:
        print("      -> no valid GPS fixes found in the log; skipping distance-based "
              "outlier rejection")

    course_exclude_sec = (
        args.course_exclude_sec if args.course_exclude_sec is not None
        else args.window_min * 60.0
    )
    top_pid = select_top_pid_errors(
        pid_samples, course_events, args.top_n,
        course_exclude_sec, args.min_separation_sec,
    )
    print(f"      -> {len(top_pid)} largest PID errors selected "
          f"(none within {course_exclude_sec:.0f}s of a course change)")

    merged_course_events = merge_course_events(course_events, args.course_merge_sec)
    n_merged = sum(1 for f in merged_course_events if f.merged_count > 1)
    if n_merged:
        print(f"      -> {len(course_events)} course-change events accumulated into "
              f"{len(merged_course_events)} findings ({n_merged} of them merged, "
              f"within {args.course_merge_sec:.0f}s of each other)")

    findings = sorted(merged_course_events + top_pid, key=lambda f: f.t)
    if not findings:
        print("No findings (neither course changes nor PID errors) - aborting.")
        sys.exit(0)

    half_width = args.window_min * 60.0

    if not args.no_good_examples:
        good_examples = find_good_examples(tune_att, tune_pids, t_min, t_max, 2 * half_width,
                                            max_examples=args.max_good_examples,
                                            min_coverage_frac=args.good_example_min_coverage,
                                            min_activity_frac=args.good_example_min_activity,
                                            edge_margin_s=(args.good_example_edge_margin_min * 60.0
                                                           if args.good_example_edge_margin_min is not None
                                                           else None))
        if good_examples:
            print(f"      -> {len(good_examples)} good-tracking example(s) found (one per sea "
                  f"state experienced): "
                  + ", ".join(f"{lbl} (RMS {rms:.3f})" for _, lbl, rms, _, _ in good_examples))
            for t_center, sea_label, rms_err, roll_std, period in good_examples:
                findings.append(Finding(
                    kind="good_example",
                    t=t_center,
                    label=f"Good example: {sea_label}",
                    detail=f"Best-tracking window found for {sea_label} sea state "
                           f"(RMS |err| {rms_err:.4f})",
                    value=rms_err,
                ))
            findings.sort(key=lambda f: f.t)
        else:
            print("      -> no good-tracking example found (not enough attitude/PID data "
                  "to assess any sea state)")

    if not args.no_gust_examples:
        gust_events = find_gust_events(
            tune_att, tune_gps_speed, course_events, t_min, t_max,
            gust_window_s=args.gust_window_sec, exclude_sec=args.gust_exclude_sec,
            min_separation_sec=args.min_separation_sec, max_examples=args.max_gust_examples,
            min_roll_deg=args.gust_min_roll_deg, min_speed_mps=args.gust_min_speed_mps,
            min_duration_s=args.gust_min_duration_sec, sustain_frac=args.gust_sustain_frac,
        )
        if gust_events:
            print(f"      -> {len(gust_events)} gust example(s) found (sudden heel+speed rise, "
                  f"not near a course change): "
                  + ", ".join(f"+{dr:.1f}deg/+{ds:.1f}m/s" for _, dr, ds in gust_events))
            for t_peak, d_roll, d_speed in gust_events:
                findings.append(Finding(
                    kind="gust",
                    t=t_peak,
                    label=f"Gust: heel +{d_roll:.1f}\u00b0, speed +{d_speed:.1f} m/s",
                    detail=f"Sudden increase in heel (+{d_roll:.1f}\u00b0) and speed "
                           f"(+{d_speed:.1f} m/s) over ~{args.gust_window_sec:.0f}s, not "
                           f"near a course change",
                    value=d_speed,
                    highlight_span_s=args.gust_window_sec,
                ))
            findings.sort(key=lambda f: f.t)
        else:
            print("      -> no gust example found (no simultaneous heel+speed rise detected "
                  "away from course changes)")

    windows = build_windows([f.t for f in findings], half_width)
    win_index = WindowIndex(windows)

    print(f"[2/4] Filtering log & writing {out_bin}")
    data, n_in2, n_out, n_skip, n_gps_total, n_gps_rejected_far, n_gps_rejected_jump = filter_and_collect(
        args.logfile, out_bin, win_index, args.pid_msgtype, args.max_speed_kn,
        median_lat, median_lon, args.max_distance_km, args.ram_channel,
        gyro_field=args.gyro_field, gyro_instance=args.gyro_instance,
    )
    reduction_pct = 100.0 * (1.0 - n_out / n_in2) if n_in2 else 0.0
    print(f"      -> {n_out:,}/{n_in2:,} messages written "
          f"({reduction_pct:.1f}% reduction), {n_skip} skipped (no re-encodable buffer)")
    if n_gps_total:
        n_gps_rejected = n_gps_rejected_far + n_gps_rejected_jump
        print(f"      -> {n_gps_rejected}/{n_gps_total} GPS fixes rejected: "
              f"{n_gps_rejected_far} too far from the median position "
              f"(> {args.max_distance_km:.0f} km), {n_gps_rejected_jump} implausible "
              f"speed jumps (> {args.max_speed_kn:.1f} kn)")

    ram_params = RamParams(
        rc_min_us=args.ram_rc_min_us, rc_max_us=args.ram_rc_max_us,
        travel_mm=args.ram_travel_mm, max_speed_mms=args.ram_max_speed_mms,
        max_accel_mmss=args.ram_max_accel_mmss, deadband_mm=args.ram_deadband_mm,
        kp=args.ram_kp, min_move_speed_mms=args.ram_min_speed_mms,
    )

    print("[3/4] Generating plots & analysis per finding")
    course_index = build_course_index(course_events)
    per_finding = []
    for f in findings:
        att_slice = slice_window(data.att, f.t, half_width)
        gps_slice = slice_window(data.gps, f.t, half_width)
        pids_slice = slice_window(data.pids, f.t, half_width)
        rcou_slice = slice_window(data.rcou, f.t, half_width)
        window_course_events = [e for e in course_events if abs(e.t - f.t) <= half_width]

        sea_state = estimate_sea_state(att_slice)
        dev = heading_cog_deviation(att_slice, gps_slice)
        lag_s = estimate_tar_act_lag(pids_slice)

        per_finding.append({
            "sea_state": sea_state,
            "deviation": dev,
            "tar_act_lag_s": lag_s,
            "img_yaw_heel_speed": plot_yaw_heel_speed(att_slice, gps_slice, f.t,
                                                       window_course_events, course_index,
                                                       f.highlight_span_s),
            "img_pid_review": plot_pid_review(pids_slice, f.t,
                                               window_course_events, course_index,
                                               f.highlight_span_s),
            "img_cog_xy": plot_cog_xy(gps_slice, att_slice, f.t,
                                       window_course_events, course_index,
                                       f.highlight_span_s),
            "img_ram_position": plot_ram_position(rcou_slice, ram_params, f.t,
                                                   window_course_events, course_index,
                                                   f.highlight_span_s),
        })

    overview_img = plot_overview_map(data.all_gps, findings, use_basemap=not args.no_basemap)

    # Full-log (mode-gated, NOT window-gated) arrays for the tuning engine -
    # see scan_log()'s docstring for why this has to be separate from the
    # per-finding data above: a chronic problem that's never a top-N error
    # spike or a course change would otherwise never be seen at all.
    tp_t = np.array([r[0] for r in tune_pids])
    tp_tar = np.array([r[1] for r in tune_pids], dtype=float)
    tp_act = np.array([r[2] for r in tune_pids], dtype=float)
    tp_err = np.array([r[3] for r in tune_pids], dtype=float)
    tp_p = np.array([r[4] for r in tune_pids], dtype=float)
    tp_i = np.array([r[5] for r in tune_pids], dtype=float)
    tp_ff = np.array([r[7] for r in tune_pids], dtype=float)
    tg_t = np.array([r[0] for r in tune_gyro])
    tg_y = np.array([r[1] for r in tune_gyro], dtype=float)
    tr_t = np.array([r[0] for r in tune_rcou])
    tr_pwm = np.array([r[1] for r in tune_rcou], dtype=float)
    tgps_t = np.array([r[0] for r in tune_gps_speed])
    tgps_spd = np.array([r[1] for r in tune_gps_speed], dtype=float)

    tuning_segments = []  # list of dicts, one per detected tuning segment
    if not args.no_tuning_suggestions:
        param_prefix = args.param_prefix or PARAM_PREFIX_GUESS.get(args.pid_msgtype, "")
        if not param_prefix:
            print(f"      Warning: no known parameter prefix for {args.pid_msgtype}; "
                  f"pass --param-prefix explicitly to get tuning suggestions.", file=sys.stderr)

        changes = find_param_change_points(parm_history, param_prefix, t_min, t_max)
        all_segments = build_tuning_segments(changes, t_min, t_max)
        min_segment_s = args.min_tuning_segment_min * 60.0
        segments = [s for s in all_segments if (s["end"] - s["start"]) >= min_segment_s]
        n_skipped = len(all_segments) - len(segments)
        if len(all_segments) > 1:
            print(f"      -> {len(changes)} tuning parameter change(s) found - splitting the "
                  f"tuning analysis into {len(all_segments)} segments so each one only sees the "
                  f"gains that were actually active during it")
        if n_skipped:
            print(f"      -> skipping {n_skipped} segment(s) shorter than "
                  f"{args.min_tuning_segment_min:.1f} min (not enough data for a reliable read): "
                  + ", ".join(f"{s['start']:.0f}-{s['end']:.0f}s ({(s['end']-s['start']):.0f}s)"
                              for s in all_segments if (s["end"] - s["start"]) < min_segment_s))
        if not segments:
            print("      -> no tuning segment long enough to analyze "
                  f"(need >= {args.min_tuning_segment_min:.1f} min)")
        print("      -> estimating PID/filter tuning suggestions across the whole log (Welch)")

        for seg in segments:
            s0, s1 = seg["start"], seg["end"]
            pm = (tp_t >= s0) & (tp_t <= s1)
            gm = (tg_t >= s0) & (tg_t <= s1)
            rm = (tr_t >= s0) & (tr_t <= s1)
            gpm = (tgps_t >= s0) & (tgps_t <= s1)

            # Use the segment's MIDPOINT, not its start, to resolve current
            # parameter values: a segment's start is often the exact
            # instant of a real parameter change (correct - the new value
            # takes effect right there), but the very first segment starts
            # at the log's absolute first message, which can be a moment
            # BEFORE the boot parameter dump finishes writing - querying
            # "what was P set to" at that literal instant would wrongly
            # come back empty even though it's been constant for the
            # entire rest of the (possibly hours-long) segment.
            param_ref_t = (s0 + s1) / 2.0

            (tuning_metrics, tuning_suggestions, err_freqs, err_psd, gyro_freqs, gyro_psd,
             tuning_sustained, tuning_cause, tuning_step_windup, tuning_balance,
             tuning_speed_dep) = analyze_pid_tuning(
                tp_t[pm], tp_tar[pm], tp_act[pm], tp_err[pm], tp_i[pm], tp_p[pm], tp_ff[pm],
                tg_t[gm], tg_y[gm], tr_t[rm], tr_pwm[rm], tgps_t[gpm], tgps_spd[gpm],
                parm_history, param_ref_t, param_prefix, target_segments=args.target_segments,
                rc_min_us=args.ram_rc_min_us, rc_max_us=args.ram_rc_max_us,
                pwm_margin_us=args.pwm_margin_us, speed_bins=args.speed_bins,
            )
            tuning_plot_img = plot_tuning_diagnostics(tuning_metrics, err_freqs, err_psd, gyro_freqs, gyro_psd)
            print(f"         segment t={s0:.0f}..{s1:.0f}s: {int(pm.sum())} {args.pid_msgtype} "
                  f"samples ({tuning_metrics['err_n_seg']} Welch segments), "
                  f"{int(gm.sum())} IMU samples ({tuning_metrics['gyro_n_seg']} Welch segments), "
                  f"{len(tuning_suggestions)} suggestion(s)")
            noise_floor_str = ("n/a" if tuning_metrics["noise_floor_hz"] is None
                                else f"{tuning_metrics['noise_floor_hz']:.2f} Hz")
            print(f"           osc: {tuning_metrics['dominant_osc_hz']:.2f} Hz "
                  f"({tuning_metrics['osc_power_frac']*100:.0f}% of spectral power), "
                  f"noise floor: {noise_floor_str}, RMS err: {tuning_metrics['rms_error']:.4f}")
            cv = tuning_metrics["current_values"]
            print(f"           {param_prefix}* values used: "
                  + ", ".join(f"{k}={'n/a' if v is None else round(v, 5)}" for k, v in cv.items()))
            if not tuning_metrics["params_found"]:
                print(f"           WARNING: none of {param_prefix}{{{','.join(cv.keys())}}} were found "
                      f"in this log's parameters - suggestions for this segment will all be empty. "
                      f"Check --param-prefix (guessed '{param_prefix}' from --pid-msgtype "
                      f"'{args.pid_msgtype}').", file=sys.stderr)
            tuning_segments.append({
                "start": s0, "end": s1, "changes": seg["changes"], "prefix": param_prefix,
                "metrics": tuning_metrics, "suggestions": tuning_suggestions,
                "plot_img": tuning_plot_img, "sustained": tuning_sustained,
                "cause": tuning_cause, "step_windup": tuning_step_windup,
                "balance": tuning_balance, "speed_dep": tuning_speed_dep,
            })

    print(f"[4/4] Writing report: {out_html}")
    stats = {
        "t_min": t_min, "t_max": t_max,
        "n_in": n_in2, "n_out": n_out,
        "reduction_pct": reduction_pct,
        "n_course": len(course_events),
        "modes": ", ".join(allowed_modes),
        "n_gps_total": n_gps_total,
        "n_gps_rejected": n_gps_rejected,
        "n_gps_rejected_far": n_gps_rejected_far,
        "n_gps_rejected_jump": n_gps_rejected_jump,
        "max_speed_kn": args.max_speed_kn,
        "max_distance_km": args.max_distance_km,
    }
    display_params = [p.strip() for p in args.params.split(",") if p.strip()]
    html = build_html_report(args.logfile, findings, per_finding, overview_img,
                              parm_history, display_params, args.param_columns,
                              tuning_segments,
                              stats, args)
    with open(out_html, "w", encoding="utf-8") as fh:
        fh.write(html)

    print("\nDone.")
    print(f"  Filtered log: {out_bin}")
    print(f"  HTML report:  {out_html}")


if __name__ == "__main__":
    main()
