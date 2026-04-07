# Generating Variation GIFs with Isaac Sim + Replicator

This guide shows how to generate many variations of a single USD scene in NVIDIA Isaac Sim using the Replicator extensions, then convert the saved frames into GIFs or MP4s for presentation.

Use this when your USD scene is already open in Isaac Sim and you are ready to paste a Python script into the Script Editor.

**Quick overview**
- **Goal:** produce separate visual sequences (GIFs) that each illustrate varying a single parameter (lighting, object position, material color).
- **Approach:** run three separate replicator runs — one focused on lighting, one on the object pose, and one on color — saving frames to separate folders. Convert those frames to GIF or MP4 with `ffmpeg`.

**Prerequisites**
- **Isaac Sim:** running and your USD scene loaded.
- **Extensions enabled:** `omni.replicator.core` and `omni.syntheticdata` (enable via Window → Extensions).
- **Know your target prim path:** replace `"/World/TargetObject"` in the examples with the real prim path of the object you want to vary.
- **Script Editor:** open (Window → Script Editor) so you can paste and run the provided Python code.

**High-level recipe**
1. Create (or reuse) a camera and render product.
2. For each parameter you want to visualize, enable only the corresponding randomizer in a new replicator layer and run `rep.orchestrator.run(...)` for N frames.
3. Repeat for other parameters, saving each run to its own folder.
4. Convert the saved frames into GIFs or MP4s with `ffmpeg` for presentation.

**Example: full script (paste into Script Editor)**

Replace the `target_prim` string below with your object prim path. Each section writes frames to a different subfolder in `output_root` so you can make separate GIFs.

```python
import omni.replicator.core as rep

# --- User settings ---
target_prim = "/World/TargetObject"   # <- CHANGE this to your object prim path
output_root = "/home/user/sdg_output" # <- CHANGE to a path writable by Isaac Sim
frame_count = 60
resolution = (1280, 720)

# --- Camera / render product ---
camera = rep.create.camera(position=(0, -1.0, 0.8), look_at=(0, 0, 0))
render_product = rep.create.render_product(camera, resolution)

# Utility to prepare writer for each run
def make_writer(subdir):
    writer = rep.WriterRegistry.get("BasicWriter")
    writer.initialize(output_dir=f"{output_root}/{subdir}", rgb=True, depth=False, semantic_segmentation=False)
    writer.attach([render_product])
    return writer


# -------------------------
# 1) Lighting variations
# -------------------------
writer = make_writer("light_variations")
with rep.new_layer():
    # Create a single randomized light per generated frame
    rep.create.light(
        light_type="Sphere",
        intensity=rep.distribution.uniform(500, 3000),
        temperature=rep.distribution.uniform(3000, 9000),
        position=rep.distribution.uniform((-2, -2, 2), (2, 2, 4)),
    )
rep.orchestrator.run(num_frames=frame_count)


# -------------------------
# 2) Object position / pose variations
# -------------------------
writer = make_writer("pose_variations")
target_obj = rep.get.prims(path_pattern=target_prim)
with rep.new_layer():
    rep.modify.pose(
        target_obj,
        position=rep.distribution.uniform((-0.3, -0.3, 0), (0.3, 0.3, 0.2)),
        rotation=rep.distribution.uniform((0, 0, 0), (0, 360, 0)),
    )
rep.orchestrator.run(num_frames=frame_count)


# -------------------------
# 3) Object color / material variations
# -------------------------
writer = make_writer("color_variations")
with rep.new_layer():
    rep.modify.attribute(
        target_obj,
        attribute="primvars:displayColor",
        value=rep.distribution.uniform((0, 0, 0), (1, 1, 1)),
    )
rep.orchestrator.run(num_frames=frame_count)

print("Finished generating frames. Convert the images to GIFs using ffmpeg (see README).")
```

Notes about the script
- **Separate runs**: we run `rep.orchestrator.run(...)` three times, each with only the focused randomizer active; this keeps each output folder specific to one parameter.
- **Adjust `frame_count`** to control GIF length; `60` at 15 fps gives a 4-second GIF.
- **Determinism**: the script uses random distributions; you can repeat runs to get different samples. If you need exact reproducibility across runs, add a seed if your installation's Replicator exposes RNG seeding (check documentation).

Converting frames to GIF or MP4

After each run you will have a folder like `output_root/light_variations` containing image files (check the writer output structure; images are typically written directly into the output folder or a named subfolder). Use `ffmpeg` to build gifs or mp4s.

- Create a GIF (lossy, widely supported):

```bash
ffmpeg -framerate 15 -pattern_type glob -i '/home/user/sdg_output/light_variations/*.png' \
  -vf "scale=1280:-1:flags=lanczos" -loop 0 light_variations.gif
```

- Create a high-quality MP4 (recommended for presentation slides):

```bash
ffmpeg -framerate 15 -pattern_type glob -i '/home/user/sdg_output/light_variations/*.png' \
  -c:v libx264 -pix_fmt yuv420p -crf 18 -vf "scale=1280:-1" light_variations.mp4
```

Tips for useful presentation GIFs
- **One-parameter-per-gif**: show clarity by varying only one thing at a time.
- **Frame count**: 40–90 frames is a reasonable range for short GIFs (3–6 seconds at 15–20 fps).
- **Use MP4 for longer sequences**: GIFs get large — prefer MP4 for long demos.
- **Smooth transitions**: if you want smooth, deterministic sweeps (e.g., a light moving along a line or color transitioning across a gradient), ask me and I will generate a scripted sweep example that sets attribute values explicitly per frame (rather than Replicator's random distributions).
- **Clutter & realism**: try randomizing small additional factors (slight camera jitter, small amount of scene clutter) to show robustness, but keep presentation runs focused.

Troubleshooting
- **No images written:** check the writer `output_dir` permissions and confirm the `BasicWriter` was attached to your render product.
- **Images are empty/black:** confirm camera position and that the render product references the expected camera. Also check that lights are present (or increase intensity).
- **Too many files:** run with smaller `frame_count` or clean the output folder between tests.

Next steps I can help with
- Tailor the example script to your exact USD prim path(s) and camera placements.
- Create deterministic sweeps for smooth animations (I will produce a per-frame modification script and loop to capture each frame).
- Add annotators and save semantic/instance masks or create COCO-style annotations.

If you want, tell me the exact prim path for your target object (copy it from the Stage tree) and I will generate a fully-tailored script that does a smooth color sweep, a deterministic light sweep, and a pose sweep suitable for GIFs.

---
File added to repository: `REPLICATOR_VARIATIONS.md`
