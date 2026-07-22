# Vehicle Swept Path — SketchUp Extension

Interactively simulate and visualize **vehicle swept paths** (turning envelopes
and tire tracks) for road-layout design, directly on SketchUp's ground plane.
Drop in a design vehicle, then "drive" it around your layout with the arrow keys
or a control panel and watch the swept area develop in real time.

Works on a flat 2D plane (z = 0), which is the standard setting for road
geometry, intersection, roundabout, and parking-layout checks.

![concept](docs-not-included) <!-- placeholder; no binary assets shipped -->

## Features

- **Kinematic bicycle model** for the lead unit — exact circular arcs, steering
  limited by each vehicle's minimum turning radius.
- **Articulated vehicles** — a chain of towed units (car + trailer, semi,
  B-double) with correct **off-tracking** and **jackknife-on-reverse** behavior,
  derived from a non-holonomic (no side-slip) hitch constraint.
- **Built-in design vehicles** (AASHTO-style, dimensions in metres):
  passenger car (P), single-unit truck (SU-30), city bus (BUS-40),
  car + utility trailer, semi-trailer (WB-50), and double-trailer (WB-67D).
- **Live visualization**:
  - body **swept envelope** (traces of all body corners),
  - **wheel tracks** (tire contact paths — shows the critical inner rear wheel),
  - **ghost footprints** dropped at a chosen spacing along the path.
- **Interactive control** three ways: arrow keys in the model, the HTML control
  panel, or an interactive click-to-place / click-to-aim setup.
- **Live readout**: distance travelled, current turn radius, and swept width.
- **Commit to model** — bakes the traces and footprints into a named group with
  a translucent footprint material, so it becomes part of your drawing.

## Install

### Option A — as an `.rbz` (recommended)

1. Build the package (see below) to get `vehicle_swept_path.rbz`, or zip it
   yourself.
2. In SketchUp: **Extensions ▸ Extension Manager ▸ Install Extension**, choose
   the `.rbz`, and confirm.

To build the `.rbz` from this repo:

```bash
./build.sh            # produces vehicle_swept_path.rbz
```

An `.rbz` is simply a ZIP whose root contains `swept_path.rb` and the
`swept_path/` folder.

### Option B — copy into the Plugins folder

Copy **both** `swept_path.rb` and the `swept_path/` folder into your SketchUp
Plugins folder:

- **Windows:** `%AppData%\SketchUp\SketchUp 20XX\SketchUp\Plugins`
- **macOS:** `~/Library/Application Support/SketchUp 20XX/SketchUp/Plugins`

Then restart SketchUp.

Requires SketchUp 2017 or newer (uses the `UI::HtmlDialog` API).

## Usage

1. **Extensions ▸ Vehicle Swept Path ▸ Open Control Panel.** This opens the
   panel and activates the Swept Path tool.
2. Pick a **vehicle** from the dropdown.
3. **Place it:** click once in the model for the start point (the lead unit's
   rear axle), then click again to aim the initial heading.
4. **Drive:**

   | Input | Action |
   |-------|--------|
   | ▲ / ▼ Up / Down arrow | drive forward / backward one step |
   | ◀ / ▶ Left / Right arrow | steer left / right |
   | Space | center the steering |
   | Esc | clear and re-arm placement |

   The same actions are available as buttons in the control panel, plus a
   steering slider and a step-size field.
5. Toggle **body envelope / wheel tracks / ghost footprints** and adjust ghost
   spacing in the *Visualization* section.
6. **Reset path** re-runs from the start point; **Clear all** removes the
   vehicle; **Commit to model** bakes the current result into a group.

## How it works

All simulation runs in metres on the z = 0 plane; results are converted to
SketchUp's internal units only when drawing.

- **Lead unit** — kinematic bicycle model. For a step `ds` at steering angle
  `δ` with wheelbase `L`, the heading changes by `ds·tan(δ)/L` and the rear
  axle follows the exact arc of radius `R = L/tan(δ)`.
- **Towed unit** — the hitch point is rigidly attached to the unit in front.
  Enforcing zero lateral velocity at the towed axle gives
  `dθ_trailer = (ΔH · n̂) / d`, where `ΔH` is the hitch's displacement, `n̂` is
  the trailer's left-normal, and `d` is the drawbar length. Chaining this rule
  supports any number of trailers.

Motion is integrated in 0.1 m sub-steps for accuracy, and a footprint is
recorded at each sub-step so the traced envelope stays smooth through curves.

## Project layout

```
swept_path.rb              # extension loader (registers with SketchUp)
swept_path/
  main.rb                  # entry point: App singleton, menus
  presets.rb               # design-vehicle dimensions
  vehicle.rb               # kinematics: Unit + Vehicle, Util helpers
  simulation.rb            # state, recording, drawing, commit-to-model
  tool.rb                  # interactive Tool (mouse + arrow keys)
  dialog.rb                # HtmlDialog control panel + callbacks
  html/panel.html          # the control-panel UI
```

## Notes & limitations

- Flat 2D plane only (by design — matches how swept-path checks are done).
- The swept envelope is drawn from body-corner and wheel-contact traces plus
  ghost footprints, which is the standard swept-path deliverable. It does not
  compute a single filled Boolean-union polygon of the whole envelope.
- Trailer dynamics are quasi-static (kinematic), appropriate for low-speed
  maneuvering analysis — not a dynamic (speed/tire-force) simulation.
