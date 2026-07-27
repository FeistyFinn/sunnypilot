#!/usr/bin/env python3
"""
Offscreen preview harness for the VTB inertia-comp Developer UI readouts.

Renders the sunnypilot onroad Developer UI (bottom bar + right panel) with synthetic
carState / carStateSP.coopSteering data and exports a PNG per scenario, so the VTB
inertia-comp elements can be reviewed without a comma device.

The repo's "UI not local-mac-runnable" note is about the scons font-atlas build, not
rendering: generate the atlases once with
    .venv/bin/python selfdrive/assets/fonts/process.py
then run this from the repo root:
    PYTHONPATH=$(pwd) .venv/bin/python tools/sunnypilot/vtb/preview_devui.py
    WINDOWED=1 PYTHONPATH=$(pwd) .venv/bin/python tools/sunnypilot/vtb/preview_devui.py   # live window

PNGs land in notes/preview/.
"""
import os

import pyray as rl

# Mirror the comma-three (big) screen and skip auto-scaling so the screenshot is full-res.
os.environ.setdefault("BIG", "1")
os.environ.setdefault("SCALE", "1.0")

WINDOWED = os.getenv("WINDOWED", "0") == "1"
if not WINDOWED:
  rl.set_config_flags(rl.ConfigFlags.FLAG_WINDOW_HIDDEN)
  os.environ["OFFSCREEN"] = "1"  # raylib without an FPS limit (must be set before importing gui_app)

from cereal import log
from openpilot.system.ui.lib.application import gui_app
from openpilot.selfdrive.ui.ui_state import ui_state, device
from openpilot.selfdrive.ui.sunnypilot.onroad.developer_ui import DeveloperUiRenderer, DeveloperUiState

OUT_DIR = "notes/preview"

# Keep builder Events alive: capnp readers alias the builder's backing memory.
_KEEPALIVE: list = []


class _Lookup:
  """Minimal stand-in for SubMaster.valid / .recv_frame / .alive (dict-with-default)."""
  def __init__(self, d: dict, default):
    self._d = d
    self._default = default

  def __getitem__(self, key):
    return self._d.get(key, self._default)


class FakeSubMaster:
  """Feeds the DeveloperUiRenderer fixed messages, no msgq/zmq involved."""
  def __init__(self, msgs: dict, valid: dict):
    self._msgs = msgs
    self.valid = _Lookup(valid, False)
    self.alive = _Lookup({}, True)
    self.updated = _Lookup({}, True)
    self.frame = 1000
    self.recv_frame = _Lookup({}, 1000)  # all >> started_frame so _render proceeds

  def __getitem__(self, key):
    return self._msgs[key]

  def update(self, timeout: int = 0) -> None:
    pass


def _reader(service: str):
  """Build a log.Event for one service (like messaging.new_message, but without msgq).

  Returns the retained builder Event + the sub-struct builder to populate."""
  evt = log.Event.new_message()
  evt.init(service)
  _KEEPALIVE.append(evt)
  return evt, getattr(evt, service)


def build_messages(coop: dict) -> tuple[dict, dict]:
  msgs, valid = {}, {}

  evt, cs = _reader("carState")
  cs.vEgo = 18.0
  cs.aEgo = 0.3
  cs.steeringAngleDeg = -4.5
  cs.steeringRateDeg = 12.0
  cs.steeringTorqueEps = 8.0
  cs.steeringTorque = coop.get("steeringTorque", 0.0)
  cs.steeringPressed = coop.get("steeringPressed", False)
  msgs["carState"] = evt.as_reader().carState

  evt, ctl = _reader("controlsState")
  ctl.curvature = 0.010
  ctl.desiredCurvature = 0.012
  angle = ctl.lateralControlState.init("angleState")  # Tesla is angle control
  angle.steeringAngleDeg = -4.0
  msgs["controlsState"] = evt.as_reader().controlsState

  evt, ccs = _reader("carControl")
  ccs.latActive = coop.get("latActive", True)
  msgs["carControl"] = evt.as_reader().carControl

  evt, rs = _reader("radarState")
  rs.leadOne.status = False
  msgs["radarState"] = evt.as_reader().radarState

  evt, lp = _reader("liveParameters")
  lp.roll = 0.0
  msgs["liveParameters"] = evt.as_reader().liveParameters
  valid["liveParameters"] = True

  # Present but invalid -> renderer skips these branches.
  for svc in ("liveTorqueParameters", "gpsLocation", "gpsLocationExternal"):
    evt, _ = _reader(svc)
    msgs[svc] = getattr(evt.as_reader(), svc)
    valid[svc] = False

  evt, csp = _reader("carStateSP")
  c = csp.coopSteering
  c.coopActive = coop.get("coopActive", False)
  c.inertiaCompActive = coop.get("inertiaCompActive", False)
  c.shadowActive = coop.get("shadowActive", False)
  c.alphaFilt = coop.get("alphaFilt", 0.0)
  c.tauInertia = coop.get("tauInertia", 0.0)
  c.tauIntent = coop.get("tauIntent", 0.0)
  c.inertiaJUsed = coop.get("inertiaJUsed", 0.08)
  c.angleOverride = coop.get("angleOverride", 0.0)
  msgs["carStateSP"] = evt.as_reader().carStateSP
  valid["carStateSP"] = True

  return msgs, valid


# Scenario coop-state overrides + the fit-meter sample count to display (fit_n).
SCENARIOS = {
  "baseline_no_coop": {"coopActive": False, "fit_n": 0},
  "shadow_gathering": {"coopActive": True, "shadowActive": True, "steeringPressed": False,
                       "steeringTorque": 0.1, "alphaFilt": 7.5, "tauInertia": 0.6, "inertiaJUsed": 0.08,
                       "fit_n": 120},
  "shadow_fit_ready": {"coopActive": True, "shadowActive": True, "steeringPressed": False,
                       "steeringTorque": 0.1, "alphaFilt": 8.0, "tauInertia": 0.64, "inertiaJUsed": 0.08,
                       "fit_n": 200},
  "live_saturated": {"coopActive": True, "inertiaCompActive": True, "steeringPressed": False,
                     "steeringTorque": 0.2, "alphaFilt": 40.0, "tauInertia": 2.45, "inertiaJUsed": 0.14,
                     "fit_n": 60},
  "live_sign_error": {"coopActive": True, "inertiaCompActive": True, "steeringPressed": False,
                      "steeringTorque": 0.2, "alphaFilt": 8.0, "tauInertia": -0.6, "inertiaJUsed": 0.08,
                      "fit_n": 80},
}


def _draw(renderer, rect) -> None:
  rl.begin_drawing()
  rl.clear_background(rl.Color(40, 44, 52, 255))  # neutral grey so the bottom bar + colors read
  renderer.render(rect)
  rl.end_drawing()


def main() -> int:
  os.makedirs(OUT_DIR, exist_ok=True)
  gui_app.init_window("vtb devui preview")

  # Freeze frame-time so the FIT accumulator only reflects the injected count (deterministic shots).
  rl.get_frame_time = lambda: 0.0

  ui_state.is_metric = True
  ui_state.started = True
  ui_state.started_frame = 0
  device._awake = True

  rect = rl.Rectangle(0, 0, gui_app.width, gui_app.height)

  for mode in (DeveloperUiState.BOTH, DeveloperUiState.BOTTOM):
    ui_state.developer_ui = mode
    tag = "both" if mode == DeveloperUiState.BOTH else "bottom"
    for name, sc in SCENARIOS.items():
      # Fresh renderer per scenario so the sticky coop gate + fit accumulator start clean.
      renderer = DeveloperUiRenderer()
      ui_state.sm = FakeSubMaster(*build_messages(sc))

      _draw(renderer, rect)                                  # 1st frame runs the session reset
      renderer.vtb_fit_elem._seconds = sc["fit_n"] / 100.0 + 1e-6  # exact FIT count for the shot
      for _ in range(2):
        _draw(renderer, rect)

      out = f"{OUT_DIR}/vtb_{name}_{tag}.png"
      rl.take_screenshot(out)
      print(f"wrote {out}")

  gui_app.close()
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
