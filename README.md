# Synthetic Data Generation

Scripts for generating synthetic training data by capturing image sequences from an Isaac Sim scene across variations in color, lighting, object position, and object scale, with a utility to compile those sequences into videos.

Original albedo color of the bottle is #D9D9D9. The original color tint is #FFFFFF.

---

## Setup

### Prerequisites

- **NVIDIA Isaac Sim 5.1** — required to run all capture scripts
- **Python 3.8+** — required to run `create_video.py` outside Isaac Sim
- **ffmpeg** — required by `create_video.py` to produce MP4s and GIFs

Install ffmpeg on Ubuntu/Debian:
```bash
sudo apt install ffmpeg
```

### Virtual Environment (for `create_video.py`)

The capture scripts run inside Isaac Sim's built-in Python environment and **cannot** be run from a virtual environment. Only `create_video.py` is run from the terminal, and it uses only Python standard library modules. Set up a virtual environment to keep the project isolated:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

`requirements.txt` includes `usd-core`, which provides the `pxr` Python bindings for working with USD files outside of Isaac Sim. Deactivate when done:
```bash
deactivate
```

---

## Running the Scripts

### Capture scripts (Isaac Sim Script Editor)

`color_variation_capture.py`, `light_position_capture.py`, `object_position_capture.py`, `object_scale_capture.py`, and `test_single_capture.py` must be run from within Isaac Sim, **not** from the terminal.

1. Open Isaac Sim 5.1 and load the USD scene (`sim/sim-xr2-bin.usd` or the relevant file).
2. Open **Window → Script Editor**.
3. Open or paste the desired script.
4. Click **Run** (or press **Ctrl+Enter**).
5. Captured images are saved to the corresponding subdirectory under `results/`.

> **Note:** Each script has a `Configuration` section near the top. Verify that `CAMERA_PRIM_PATH`, any object/light prim paths, and `OUTPUT_DIR` match your scene before running.

Output is saved to the following directories under `results/`:

| Script | Output directory |
|---|---|
| `test_single_capture.py` | `results/med_color/` |
| `color_variation_capture.py` | `results/med_color/` |
| `light_position_capture.py` | `results/light_position/` |
| `object_position_capture.py` | `results/object_position/` |
| `object_scale_capture.py` | `results/object_scale/` |

### `create_video.py`

Converts a captured image sequence into an MP4 and/or GIF. Run from the terminal with the virtual environment active:

```bash
source .venv/bin/activate
python3 sim/scripts/create_video.py
```

By default it reads from `results/med_color/`. Use `--help` for available options:

```bash
python3 sim/scripts/create_video.py --help
```
