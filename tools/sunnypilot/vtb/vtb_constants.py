"""Centralized VTB offline-analysis constants — pure literals, no heavy imports.

Single source of truth for the thresholds shared across the VTB tools
(fit_steer_inertia, analyze_shadow, live_watch, transcribe_events, logio). Kept
deliberately import-light (stdlib only — no cereal / opendbc / raylib) so headless
tools like live_watch can import it without a display or car toolchain on PATH.

NOTE: DT_LAT_CTRL is intentionally NOT here — it derives from opendbc
`CarControllerParams.STEER_STEP` + `common.realtime.DT_CTRL`, which would drag opendbc
onto this pure-constants path. It stays defined once in fit_steer_inertia.py.

The VTB_* fit/FF thresholds mirror the DevUI's
selfdrive/ui/sunnypilot/onroad/developer_ui/vtb_fit.py — if you retune them there,
mirror the change here (that module can't be imported headless: it pulls in pyray).
"""

# steering-torque thresholds (mirror opendbc coop_steering.py)
DEADZONE_NM = 0.5              # STEER_OVERRIDE_MIN_TORQUE — hands-off torque gate
ALPHA_FLOOR = 5.0             # rad/s^2 — min |alpha| excitation for a usable legacy-ID sample
DEFAULT_RC = 0.04            # STEER_ALPHA_FILTER_RC — causal-alpha LPF time constant
GAP_S = 0.05                # carState spacing above this starts a new contiguous run

# inertia-J literature plausibility band (kg*m^2)
J_LIT_LO, J_LIT_HI = 0.05, 0.15

# fit / FF gates
VTB_FIT_SAMPLES = 200        # fit's "n < 200 -> INSUFFICIENT" gate (= 2.0 s at 100 Hz)
VTB_FF_LIMIT = 2.5          # Nm — inertia FF clamp (STEER_INERTIA_TORQUE_LIMIT)

# transcribe FF-engaged Schmitt trigger + deadzone-guard canary epsilon
FF_ON_NM = 0.05             # |tauInertia| above this -> ff_engaged
FF_OFF_NM = 0.01            # |tauInertia| below this -> ff_idle (hysteresis)
GUARD_EPS = 1e-6           # a deadzone-guard violation is |tauInertia| > this while in-deadzone

# Single local rlog root: all pulled drives live under realdata (the legacy secondary
# shadow store was consolidated into it on 2026-06-25). A 1-tuple so the resolver loops
# stay unchanged. (Fixes the stale 2-tuple that lingered in fit.)
LOCAL_LOG_ROOTS = ("~/.comma/media/0/realdata",)
