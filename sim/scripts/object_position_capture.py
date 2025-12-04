"""
object_position_capture.py

Isaac Sim 5.1 Script Editor compatible script for capturing images with object position variations.

Usage:
1. Open your USD scene in Isaac Sim 5.1
2. Open Window → Script Editor
3. Copy and paste this ENTIRE script
4. Click "Run" or press Ctrl+Enter
5. Check the output directory for the captured images

This script:
- Moves an object in a circular path on the XY plane around its current position
- Captures an RGB image at each position
- Reverts the object position back to the original when done
- Saves images that can be stitched together into a video with seamless looping
"""

# ----------------------
# Configuration
# ----------------------
CAMERA_PRIM_PATH = "/World/rsd455/RSD455/Camera_OmniVision_OV9782_Color"
OBJECT_PRIM_PATH = "/World/med_bottle"
OUTPUT_DIR = "/home/nick/git/synthetic_data/results/object_position"
RESOLUTION = (1280, 720)

# Object position settings
NUM_POSITION_STEPS = 240  # Number of different positions to capture
CIRCLE_RADIUS = 0.1  # Radius of circular path in meters
# Object moves in a circle around its current position

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
def get_object_position(stage, object_path):
    """Get the current position of an object."""
    try:
        object_prim = stage.GetPrimAtPath(object_path)
        
        if object_prim.IsValid():
            xformable = UsdGeom.Xformable(object_prim)
            translate_op = xformable.GetOrderedXformOps()[0]  # Get first xform op (translate)
            
            if translate_op:
                return translate_op.Get()
            else:
                print(f"Warning: translate operation not found at {object_path}")
        else:
            print(f"Warning: Object prim not found at {object_path}")
    except Exception as e:
        print(f"Error getting object position: {e}")
        import traceback
        traceback.print_exc()
    
    return None


def set_object_position(stage, object_path, position):
    """Set the position of an object."""
    try:
        object_prim = stage.GetPrimAtPath(object_path)
        
        if object_prim.IsValid():
            xformable = UsdGeom.Xformable(object_prim)
            translate_op = xformable.GetOrderedXformOps()[0]  # Get first xform op (translate)
            
            if translate_op:
                translate_op.Set(Gf.Vec3d(position[0], position[1], position[2]))
                return True
            else:
                print(f"Error: translate operation not found at {object_path}")
        else:
            print(f"Error: Object prim not found at {object_path}")
    except Exception as e:
        print(f"Error setting object position: {e}")
        import traceback
        traceback.print_exc()
    
    return False


def generate_circular_path(num_steps, radius, center_x, center_y, start_angle_deg=0):
    """
    Generate positions along a circular path that loops seamlessly.
    
    Args:
        num_steps: Number of positions to generate
        radius: Radius of the circle
        center_x: X coordinate of circle center (current object X position)
        center_y: Y coordinate of circle center (current object Y position)
        start_angle_deg: Starting angle in degrees (0 = right side at x=center_x+radius)
    
    Returns:
        List of (x, y) tuples representing positions on the circle
    """
    positions = []
    
    # Convert start angle to radians
    start_angle = math.radians(start_angle_deg)
    
    # Generate positions around the circle
    # Use num_steps without adding 1 so the last position is different from first
    # This creates a seamless loop when the video repeats
    for i in range(num_steps):
        # Calculate angle for this step (counterclockwise from start_angle)
        angle = start_angle + (2 * math.pi * i / num_steps)
        
        # Calculate position on circle
        x = center_x + radius * math.cos(angle)
        y = center_y + radius * math.sin(angle)
        
        positions.append((x, y))
    
    return positions


# ----------------------
# Main Script
# ----------------------
def main():
    """Capture images with object position variations."""
    
    print("=" * 60)
    print("Isaac Sim 5.1 - Object Position Variation Capture")
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
    
    # Get and store original position
    original_position = get_object_position(stage, OBJECT_PRIM_PATH)
    print(f"Original object position: {original_position}")
    
    if original_position is None:
        print("ERROR: Could not get original object position")
        return
    
    # Store the Z coordinate to keep it constant
    original_z = original_position[2]
    
    # Use current position as the center of the circle
    center_x = original_position[0]
    center_y = original_position[1]
    
    # Create a render product for the camera
    print("\nSetting up camera render product...")
    render_product = rep.create.render_product(
        CAMERA_PRIM_PATH,
        resolution=RESOLUTION,
        name="object_position_rp"
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
    
    # Generate circular path
    print(f"\nGenerating circular path with {NUM_POSITION_STEPS} positions...")
    print(f"  Radius: {CIRCLE_RADIUS}m")
    print(f"  Center: ({center_x:.3f}, {center_y:.3f}) - current object position")
    print(f"  Starting position: ({center_x + CIRCLE_RADIUS:.3f}, {center_y:.3f}) - right side of circle")
    print(f"  Z coordinate: {original_z} (constant)")
    
    xy_positions = generate_circular_path(
        NUM_POSITION_STEPS,
        CIRCLE_RADIUS,
        center_x,
        center_y,
        start_angle_deg=0  # Start at right side (x=center_x+radius, y=center_y)
    )
    
    print(f"Circular path generated: {len(xy_positions)} positions")
    print(f"  First position: ({xy_positions[0][0]:.3f}, {xy_positions[0][1]:.3f})")
    print(f"  Last position: ({xy_positions[-1][0]:.3f}, {xy_positions[-1][1]:.3f})")
    
    # Trigger capture loop
    print("\nStarting object position variation capture...")
    
    async def capture_position_variations():
        """Async function to capture images at each object position."""
        # Give the app a few updates to ensure everything is ready
        print("Initializing render...")
        for i in range(5):
            await omni.kit.app.get_app().next_update_async()
        
        # Iterate through positions and capture
        for idx, (x, y) in enumerate(xy_positions):
            position = (x, y, original_z)
            print(f"\n[{idx+1}/{len(xy_positions)}] Moving object to ({x:.3f}, {y:.3f}, {original_z:.3f})")
            
            # Set the new position
            success = set_object_position(stage, OBJECT_PRIM_PATH, position)
            if not success:
                print(f"  WARNING: Failed to set object position")
            
            # Give a moment for the position to update
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
        
        # Revert to original position
        print(f"\n{'='*60}")
        print("Reverting to original position...")
        if original_position:
            set_object_position(stage, OBJECT_PRIM_PATH, original_position)
            print(f"Original position restored: {original_position}")
        else:
            print("WARNING: Could not restore original position (was not captured)")
        
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
    asyncio.ensure_future(capture_position_variations())
    
    print("\nCapture task scheduled. Please wait...")
    print(f"This will capture {NUM_POSITION_STEPS} images...")

# Run the main function
if __name__ == "__main__":
    main()
