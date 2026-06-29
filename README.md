# sunnypilot — Cooperative Steering (VTB)

> **Nudge the steering wheel without disengaging.** Cooperative steering for angle-control Teslas, at low speed.

This is a [sunnypilot](https://github.com/sunnypilot/sunnypilot) fork that ports dzid26's **Virtual
Torque Blending (VTB)**. On angle-control Teslas, openpilot tells the EPS *where to put the wheel* —
there's no torque knob to push against, so any driver input normally has to either be ignored or kill
lateral control. VTB reads the driver's torque on the steering column, turns it into a **bounded
steering-angle offset**, and **adds it on top of openpilot's commanded angle**. You can nudge the wheel
to reposition in a lane, ease around an obstacle, or tighten a slow turn — and lateral stays engaged.
The mapping comes from the vehicle model, so the same effort buys a big motion in a parking lot and a
subtle one on the highway, with no gain tables to maintain.

## ✨ What this fork adds

- **Cooperative Steering (VTB)** — driver torque → bounded steering-angle offset blended onto
  openpilot's path; lateral stays active for nudges below the hands-on / hard-yank limits.
- **Speed-aware stiffness, for free** — the torque-to-angle gain falls out of the bicycle model
  (`κ = a_lat / v²`), so authority shrinks ≈ 1/v² as you speed up. No separate low/high-speed modes.
- **MADS N-finger engagement** — toggle steering engagement by tapping the infotainment screen with a
  configurable number of fingers (3 / 4 / 5), with a fix for the finger-count desync that caused
  "TAKE CONTROL IMMEDIATELY" storms.
- **Cooperative longitudinal** — a tap on the accelerator doesn't kill ACC; longitudinal blends the
  same way steering does (brake still disengages).
- **Shadow-mode inertia compensation** — a steering-inertia feed-forward term is computed and logged
  every frame for offline tuning, but never steers (yet). Offline tooling under `tools/sunnypilot/vtb/`.

## ✅ Requirements

- A **comma 3X** running **AGNOS ≥ 18.4**.
- An **angle-control Tesla** (Model 3 / Model Y). The feature is Tesla-only and no-ops on other brands.

## 🚀 Enabling it

1. **Settings → Vehicle → Tesla → Cooperative Steering** (the toggle is editable **offroad only**).
2. Optionally set **MADS Toggle Touch Points** (3 / 4 / 5 fingers; default 5).
3. The setting is read at car-init, so it takes effect on the **next ignition / drive cycle**.

Full enable steps, the algorithm deep-dive, the safety model, and the tuning guide:

- **In-repo reference:** [`docs/COOPERATIVE_STEERING.md`](docs/COOPERATIVE_STEERING.md)
- **Project wiki:** <https://github.com/FeistyFinn/sunnypilot/wiki>

## ⚠️ Safety

**VTB is steering actuation. This is alpha research software, and you are the last line of defense.**
Every command is bounded by the Tesla **panda safety model**, which keeps three independent disengage
paths live (EPS hands-on level, a hard-yank torsion-bar torque limit, and an EPS angle-rate fault) and
clamps the commanded angle and its rate to the safety envelope. Read the
[Safety Model](https://github.com/FeistyFinn/sunnypilot/wiki/Safety-Model) wiki page and
[`docs/SAFETY.md`](docs/SAFETY.md) before driving, and validate the build in an empty lot first.

## 📚 Documentation

| Where | What |
| --- | --- |
| This README | Orientation — what the fork is and how to turn it on. |
| [`docs/COOPERATIVE_STEERING.md`](docs/COOPERATIVE_STEERING.md) | Canonical in-repo technical reference (physics, integration, safety, params, tuning). |
| [Project wiki](https://github.com/FeistyFinn/sunnypilot/wiki) | Expanded user + developer guide, MADS details, and FAQ / troubleshooting. |

---

# Built on sunnypilot

The sections below are from upstream sunnypilot, which this fork is built on.

![](https://user-images.githubusercontent.com/47793918/233812617-beab2e71-57b9-479e-8bff-c3931347ca40.png)

## 🌞 What is sunnypilot?
[sunnypilot](https://github.com/sunnyhaibin/sunnypilot) is a fork of comma.ai's openpilot, an open source driver assistance system. sunnypilot offers the user a unique driving experience for over 300+ supported car makes and models with modified behaviors of driving assist engagements. sunnypilot complies with comma.ai's safety rules as accurately as possible.

## 💭 Join our Community Forum
Join the official sunnypilot community forum to stay up to date with all the latest features and be a part of shaping the future of sunnypilot!
* https://community.sunnypilot.ai/

## Documentation
https://docs.sunnypilot.ai/ is your one stop shop for everything from features to installation to FAQ about the sunnypilot

## 🚘 Running on a dedicated device in a car
First, check out this list of items you'll need to [get started](https://community.sunnypilot.ai/t/getting-started-using-sunnypilot-in-your-supported-car/251).

## Installation
This fork is installed by URL / branch, not from sunnypilot's recommended-branch list — see the wiki
[Overview & Enable](https://github.com/FeistyFinn/sunnypilot/wiki/Overview-and-Enable) page. For the
base sunnypilot setup, refer to the community forum for [installation instructions](https://community.sunnypilot.ai/t/read-before-installing-sunnypilot/254), as well as a complete list of [Recommended Branch Installations](https://community.sunnypilot.ai/t/recommended-branch-installations/235).

## 🎆 Pull Requests
We welcome both pull requests and issues on GitHub. Bug fixes are encouraged.

Pull requests should be against the `vtb-port` branch.

## 📊 User Data

By default, sunnypilot uploads the driving data to comma servers. You can also access your data through [comma connect](https://connect.comma.ai/).

sunnypilot is open source software. The user is free to disable data collection if they wish to do so.

sunnypilot logs the road-facing camera, CAN, GPS, IMU, magnetometer, thermal sensors, crashes, and operating system logs.
The driver-facing camera and microphone are only logged if you explicitly opt-in in settings.

By using this software, you understand that use of this software or its related services will generate certain types of user data, which may be logged and stored at the sole discretion of comma. By accepting this agreement, you grant an irrevocable, perpetual, worldwide right to comma for the use of this data.

## Licensing

sunnypilot is released under the [MIT License](LICENSE). This repository includes original work as well as significant portions of code derived from [openpilot by comma.ai](https://github.com/commaai/openpilot), which is also released under the MIT license with additional disclaimers.

The original openpilot license notice, including comma.ai’s indemnification and alpha software disclaimer, is reproduced below as required:

> openpilot is released under the MIT license. Some parts of the software are released under other licenses as specified.
>
> Any user of this software shall indemnify and hold harmless Comma.ai, Inc. and its directors, officers, employees, agents, stockholders, affiliates, subcontractors and customers from and against all allegations, claims, actions, suits, demands, damages, liabilities, obligations, losses, settlements, judgments, costs and expenses (including without limitation attorneys’ fees and costs) which arise out of, relate to or result from any use of this software by user.
>
> **THIS IS ALPHA QUALITY SOFTWARE FOR RESEARCH PURPOSES ONLY. THIS IS NOT A PRODUCT.
> YOU ARE RESPONSIBLE FOR COMPLYING WITH LOCAL LAWS AND REGULATIONS.
> NO WARRANTY EXPRESSED OR IMPLIED.**

For full license terms, please see the [`LICENSE`](LICENSE) file.

## 💰 Support sunnypilot
If you find any of the features useful, consider becoming a [sponsor on GitHub](https://github.com/sponsors/sunnyhaibin) to support future feature development and improvements.


By becoming a sponsor, you will gain access to exclusive content, early access to new features, and the opportunity to directly influence the project's development.


<h3>GitHub Sponsor</h3>

<a href="https://github.com/sponsors/sunnyhaibin">
  <img src="https://user-images.githubusercontent.com/47793918/244135584-9800acbd-69fd-4b2b-bec9-e5fa2d85c817.png" alt="Become a Sponsor" width="300" style="max-width: 100%; height: auto;">
</a>
<br>

<h3>PayPal</h3>

<a href="https://paypal.me/sunnyhaibin0850" target="_blank">
<img src="https://www.paypalobjects.com/en_US/i/btn/btn_donateCC_LG.gif" alt="PayPal this" title="PayPal - The safer, easier way to pay online!" border="0" />
</a>
<br></br>

Your continuous love and support are greatly appreciated! Enjoy 🥰

<span>-</span> Jason, Founder of sunnypilot
