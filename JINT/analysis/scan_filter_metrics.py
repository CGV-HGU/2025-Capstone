"""Compute paired temporal-filter metrics from replayed scan arrays."""

import numpy as np


RANGE_MIN, RANGE_MAX = 0.1, 10.0


def valid_ranges(values):
    return np.isfinite(values) & (values >= RANGE_MIN) & (values <= RANGE_MAX)


def flicker_and_delta(raw, filtered):
    raw_valid, filtered_valid = valid_ranges(raw), valid_ranges(filtered)
    raw_flips = raw_valid[1:] != raw_valid[:-1]
    filtered_flips = filtered_valid[1:] != filtered_valid[:-1]
    common = raw_valid[1:] & raw_valid[:-1] & filtered_valid[1:] & filtered_valid[:-1]
    raw_delta = np.abs(raw[1:][common] - raw[:-1][common])
    filtered_delta = np.abs(filtered[1:][common] - filtered[:-1][common])
    return {
        "raw_validity_flips": int(raw_flips.sum()),
        "filtered_validity_flips": int(filtered_flips.sum()),
        "all_channel_transitions": int(raw_flips.size),
        "common_valid_transitions": int(common.sum()),
        "raw_mean_abs_range_step_m": np.mean(raw_delta) if raw_delta.size else "",
        "filtered_mean_abs_range_step_m": np.mean(filtered_delta) if filtered_delta.size else "",
        "raw_valid_fraction": np.mean(raw_valid),
        "filtered_valid_fraction": np.mean(filtered_valid),
    }


def onset_timing(raw, filtered, stamps):
    """Count withheld onsets and measure the delay to next-frame confirmation."""
    raw_valid, filtered_valid = valid_ranges(raw), valid_ranges(filtered)
    withheld = ~raw_valid[:-2] & raw_valid[1:-1] & ~filtered_valid[1:-1]
    confirmed = withheld & raw_valid[2:] & filtered_valid[2:]
    isolated = withheld & ~raw_valid[2:] & ~filtered_valid[2:]
    intervals_ms = np.diff(stamps[1:]) / 1e6
    delays_ms = intervals_ms[np.where(confirmed)[0]]
    return {
        "withheld_onsets": int(withheld.sum()),
        "withheld_single_frame_onsets": int(isolated.sum()),
        "confirmed_next_frame_onsets": int(confirmed.sum()),
        "confirmation_delay_median_ms": np.median(delays_ms) if delays_ms.size else "",
        "confirmation_delay_p95_ms": np.percentile(delays_ms, 95) if delays_ms.size else "",
    }
