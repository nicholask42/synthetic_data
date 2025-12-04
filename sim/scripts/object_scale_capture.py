"""
object_scale_capture.py

Isaac Sim 5.1 Script Editor compatible script for capturing images with object scale variations.

Usage:
1. Open your USD scene in Isaac Sim 5.1
2. Open Window → Script Editor
3. Copy and paste this ENTIRE script
4. Click "Run" or press Ctrl+Enter
5. Check the output directory for the captured images

This script:
- Scales an object from 1.0 to 2.0 and back to 1.0
- Captures an RGB image at each scale
- Reverts the object scale back to the original when done
- Saves images that can be stitched together into a video with seamless looping
"""

# ----------------------
# Configuration
# ----------------------
CAMERA_PRIM_PATH = "/World/rsd455/RSD455/Camera_OmniVision_OV9782_Color"
OBJECT_PRIM_PATH = "/World/med_bottle"
OUTPUT_DIR = "/home/nick/git/synthetic_data/results/object_scale"
RESOLUTION = (1280, 720)

# Object scale settings
NUM_SCALE_STEPS = 240  # Number of different scales to capture
MIN_SCALE = 1.0  # Minimum scale factor
MAX_SCALE = 2.0  # Maximum scale factor

# ----------------------
# Imports
# ----------------------
import omni.replicator.core as rep
from omni.isaac.core.utils.stage import get_current_stage
import omni.usd
from pxr import Sdf, Gf, UsdGeom
import os
import glob
from pathlib import Path
import asyncio
import omni.kit.app
import math

# ----------------------
# Helper Functions
# ----------------------
def get_object_scale(stage, object_path):
    """Get the current scale of an object."""
    try:
        object_prim = stage.GetPrimAtPath(object_path)
        
        if object_prim.IsValid():
            xformable = UsdGeom.Xformable(object_prim)
            # Find the scale operation
            for xform_op in xformable.GetOrderedXformOps():
                if xform_op.GetOpType() == UsdGeom.XformOp.TypeScale:
                    return xform_op.Get()
            
            print(f"Warning: scale operation not found at {object_path}")
        else:
            print(f"Warning: Object prim not found at {object_path}")
    except Exception as e:
        print(f"Error getting object scale: {e}")
        import traceback
        traceback.print_exc()
    
    return None


def set_object_scale(stage, object_path, scale):
    """Set the scale of an object."""
    try:
        object_prim = stage.GetPrimAtPath(object_path)
        
        if object_prim.IsValid():
            xformable = UsdGeom.Xformable(object_prim)
            
            # Find the scale operation
            scale_op = None
            for xform_op in xformable.GetOrderedXformOps():
                if xform_op.GetOpType() == UsdGeom.XformOp.TypeScale:
                    scale_op = xform_op
                    break
            
            if scale_op:
                # Set uniform scale (same value for x, y, z)
                scale_op.Set(Gf.Vec3f(scale, scale, scale))
                return True
            else:
                print(f"Error: scale operation not found at {object_path}")
        else:
            print(f"Error: Object prim not found at {object_path}")
    except Exception as e:
        print(f"Error setting object scale: {e}")
        import traceback
        traceback.print_exc()
    
    return False


def generate_scale_sequence(num_steps, min_scale, max_scale):
    """
    Generate a scale sequence that goes from min to max and back to min seamlessly.
    
    Args:
        num_steps: Number of scale values to generate
        min_scale: Minimum scale factor
        max_scale: Maximum scale factor
    
    Returns:
        List of scale values
    """
    scales = []
    
    # Use a sine wave to create smooth scaling from min to max and back to min
    # This creates a seamless loop when the video repeats
    for i in range(num_steps):
        # Map i to angle from 0 to 2*pi (full sine wave cycle)
        angle = 2 * math.pi * i / num_steps
        
        # Use sine wave: starts at 0, goes to 1 at pi/2, back to 0 at pi, to -1 at 3pi/2, back to 0 at 2pi
        # But we want to start at min, go to max, and back to min
        # So we use: min + (max-min) * (1 - cos(angle)) / 2
        # This makes it start at min when angle=0, max at angle=pi, and back to min at angle=2pi
        sine_value = (1 - math.cos(angle)) / 2  # Oscillates from 0 to 1 and back to 0
        scale = min_scale + (max_scale - min_scale) * sine_value
        
        scales.append(scale)
    
    return scales


# ----------------------
# Main Script
# ----------------------
def main():
    """Capture images with object scale variations."""
    
    print("=" * 60)
    print("Isaac Sim 5.1 - Object Scale Variation Capture")
    print("=" * 60)
    
    # Disable capture on play (required for manual step-based capture)
    print("Configuring orchestrator...")
    rep.orchestrator.set_capture_on_play(False)
    print(f"  Capture on play disabled")
    
    # Set DLSS to Quality mode for best results
    import carb.settings
    carb.settings.get_settings().set("/rtx/post/dlss/execMode", 2)
    print(f"  DLSS set to Quality mode")
    
    # Ensure output directory exists
    output_path = Path(OUTPUT_DIR)
    output_path.mkdir(parents=True, exist_ok=True)
    print(f"Output directory: {OUTPUT_DIR}")
    
    # Get the stage and verify camera exists
    stage = get_current_stage()
    camera_prim = stage.GetPrimAtPath(CAMERA_PRIM_PATH)
    
    if not camera_prim.IsValid():
        print(f"ERROR: Camera not found at path: {CAMERA_PRIM_PATH}")
        return
    
    print(f"Camera found: {CAMERA_PRIM_PATH}")
    
    # Verify object exists
    object_prim = stage.GetPrimAtPath(OBJECT_PRIM_PATH)
    if not object_prim.IsValid():
        print(f"ERROR: Object not found at path: {OBJECT_PRIM_PATH}")
        return
    
    print(f"Object found: {OBJECT_PRIM_PATH}")
    
    # Get and store original scale
    original_scale = get_object_scale(stage, OBJECT_PRIM_PATH)
    print(f"Original object scale: {original_scale}")
    
    if original_scale is None:
        print("ERROR: Could not get original object scale")
        return
    
    # Create a render product for the camera
    print("\nSetting up camera render product...")
    render_product = rep.create.render_product(
        CAMERA_PRIM_PATH,
        resolution=RESOLUTION,
        name="object_scale_rp"
    )
    
    # Create a basic writer to save RGB images
    print("Setting up writer...")
    writer = rep.writers.get("BasicWriter")
    writer.initialize(
        output_dir=OUTPUT_DIR,
        rgb=True,
        colorize_instance_segmentation=False,
        colorize_semantic_segmentation=False
    )
    writer.attach([render_product])
    print(f"Writer attached to render product")
    
    # Generate scale sequence
    print(f"\nGenerating scale sequence with {NUM_SCALE_STEPS} steps...")
    print(f"  Scale range: {MIN_SCALE} to {MAX_SCALE}")
    print(f"  Pattern: smoothly scales up and back down for seamless looping")
    
    scales = generate_scale_sequence(NUM_SCALE_STEPS, MIN_SCALE, MAX_SCALE)
    
    print(f"Scale sequence generated: {len(scales)} scales")
    print(f"  First scale: {scales[0]:.3f}")
    print(f"  Mid scale: {scales[len(scales)//2]:.3f}")
    print(f"  Last scale: {scales[-1]:.3f}")
    
    # Trigger capture loop
    print("\nStarting object scale variation capture...")
    
    async def capture_scale_variations():
        """Async function to capture images at each object scale."""
        # Give the app a few updates to ensure everything is ready
        print("Initializing render...")
        for i in range(5):
            await omni.kit.app.get_app().next_update_async()
        
        # Iterate through scales and capture
        for idx, scale in enumerate(scales):
            print(f"\n[{idx+1}/{len(scales)}] Setting scale to {scale:.3f}")
            
            # Set the new scale
            success = set_object_scale(stage, OBJECT_PRIM_PATH, scale)
            if not success:
                print(f"  WARNING: Failed to set object scale")
            
            # Give a moment for the scale to update
            for i in range(3):
                await omni.kit.app.get_app().next_update_async()
            
            # Capture the frame
            print(f"  Capturing frame {idx+1}...")
            try:
                await rep.orchestrator.step_async(rt_subframes=4, delta_time=0.0, pause_timeline=True)
                print(f"  ✓ Frame {idx+1} captured")
            except Exception as e:
                print(f"  ERROR during capture: {e}")
                continue
        
        # Revert to original scale
        print(f"\n{'='*60}")
        print("Reverting to original scale...")
        if original_scale:
            set_object_scale(stage, OBJECT_PRIM_PATH, original_scale[0])  # Use first component of Vec3
            print(f"Original scale restored: {original_scale}")
        else:
            print("WARNING: Could not restore original scale (was not captured)")
        
        # Wait for all data to be written to disk
        print("\nWaiting for all data to be written to disk...")
        try:
            await rep.orchestrator.wait_until_complete_async()
            print("All writes complete")
        except Exception as e:
            print(f"ERROR during wait_until_complete: {e}")
        
        # Additional wait to ensure filesystem sync
        await asyncio.sleep(1.0)
        
        # Verify the output
        print("\n" + "=" * 60)
        print("Verifying output...")
        print("=" * 60)
        
        all_images = glob.glob(os.path.join(OUTPUT_DIR, "**/*.png"), recursive=True)
        print(f"Total PNG images in output directory: {len(all_images)}")
        
        if len(all_images) > 0:
            print(f"\n✓ SUCCESS: {len(all_images)} images captured!")
            print(f"\nRecent image files:")
            for img in sorted(all_images)[-10:]:  # Show last 10
                file_size = os.path.getsize(img)
                rel_path = os.path.relpath(img, OUTPUT_DIR)
                print(f"  - {rel_path} ({file_size:,} bytes)")
            
            print(f"\nNext steps:")
            print(f"1. Review images in: {OUTPUT_DIR}")
            print(f"2. Create video using: python3 sim/scripts/create_video.py {OUTPUT_DIR}")
        else:
            print("✗ FAILED: No images were created.")
        
        print("=" * 60)
        
        # Clean up
        writer.detach()
        render_product.destroy()
    
    # Run the async capture
    asyncio.ensure_future(capture_scale_variations())
    
    print("\nCapture task scheduled. Please wait...")
    print(f"This will capture {NUM_SCALE_STEPS} images...")

# Run the main function
if __name__ == "__main__":
    main()
