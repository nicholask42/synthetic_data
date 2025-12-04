"""
color_variation_capture.py

Isaac Sim 5.1 Script Editor compatible script for capturing images with color variations.

Usage:
1. Open your USD scene in Isaac Sim 5.1
2. Open Window → Script Editor
3. Copy and paste this ENTIRE script
4. Click "Run" or press Ctrl+Enter
5. Check the output directory for the captured images

This script:
- Iterates through a range of colors for the specified material
- Captures an RGB image at each color variation
- Reverts the color back to the original when done
- Saves images that can be stitched together into a video
"""

# ----------------------
# Configuration
# ----------------------
CAMERA_PRIM_PATH = "/World/rsd455/RSD455/Camera_OmniVision_OV9782_Color"
MATERIAL_PRIM_PATH = "/World/med_bottle/WhitePackerBottle_A01_PR_NVD_01/Looks/Plastic_Glossy_White_A"
OUTPUT_DIR = "/home/nick/git/synthetic_data/results/med_color"
RESOLUTION = (1280, 720)

# Color variation settings
NUM_COLOR_STEPS = 240  # Number of different colors to capture
COLOR_MODE = "hue_sweep"  # Options: "hue_sweep", "rainbow", "custom"

# ----------------------
# Imports
# ----------------------
import omni.replicator.core as rep
from omni.isaac.core.utils.stage import get_current_stage
import omni.usd
from pxr import Sdf, UsdShade, Gf
import os
import glob
from pathlib import Path
import asyncio
import omni.kit.app
import colorsys

# ----------------------
# Helper Functions
# ----------------------
def get_material_color(stage, material_path):
    """Get the current color of a material using direct USD path."""
    try:
        # Direct path to shader
        shader_path = material_path + "/Shader"
        shader_prim = stage.GetPrimAtPath(shader_path)
        
        if shader_prim.IsValid():
            shader_obj = UsdShade.Shader(shader_prim)
            color_input = shader_obj.GetInput("diffuse_tint")
            
            if color_input:
                return color_input.Get()
            else:
                print(f"Warning: diffuse_tint not found at {shader_path}")
        else:
            print(f"Warning: Shader prim not found at {shader_path}")
    except Exception as e:
        print(f"Error getting material color: {e}")
        import traceback
        traceback.print_exc()
    
    return None


def set_material_color(stage, material_path, color):
    """Set the color of a material using direct USD path."""
    try:
        # Direct path to shader
        shader_path = material_path + "/Shader"
        shader_prim = stage.GetPrimAtPath(shader_path)
        
        if shader_prim.IsValid():
            shader_obj = UsdShade.Shader(shader_prim)
            color_input = shader_obj.GetInput("diffuse_tint")
            
            if color_input:
                color_input.Set(Gf.Vec3f(color[0], color[1], color[2]))
                return True
            else:
                print(f"Error: diffuse_tint input not found at {shader_path}")
        else:
            print(f"Error: Shader prim not found at {shader_path}")
    except Exception as e:
        print(f"Error setting material color: {e}")
        import traceback
        traceback.print_exc()
    
    return False


def generate_color_range(num_steps, mode="hue_sweep"):
    """Generate a range of colors that loops seamlessly."""
    colors = []
    
    if mode == "hue_sweep":
        # Sweep through hue while keeping saturation and value constant
        # Use num_steps without adding 1 so the last color is different from first
        # This creates a seamless loop when the video repeats
        for i in range(num_steps):
            hue = i / num_steps  # 0 to (num_steps-1)/num_steps, never reaches 1.0
            rgb = colorsys.hsv_to_rgb(hue, 0.8, 0.9)  # High saturation, high value
            colors.append(rgb)
    
    elif mode == "rainbow":
        # Classic rainbow colors that loop seamlessly
        for i in range(num_steps):
            hue = i / num_steps
            rgb = colorsys.hsv_to_rgb(hue, 1.0, 1.0)
            colors.append(rgb)
    
    else:
        # Default to hue sweep
        for i in range(num_steps):
            hue = i / num_steps
            rgb = colorsys.hsv_to_rgb(hue, 0.8, 0.9)
            colors.append(rgb)
    
    return colors


# ----------------------
# Main Script
# ----------------------
def main():
    """Capture images with color variations."""
    
    print("=" * 60)
    print("Isaac Sim 5.1 - Color Variation Capture")
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
    
    # Verify material exists
    material_prim = stage.GetPrimAtPath(MATERIAL_PRIM_PATH)
    if not material_prim.IsValid():
        print(f"ERROR: Material not found at path: {MATERIAL_PRIM_PATH}")
        return
    
    print(f"Material found: {MATERIAL_PRIM_PATH}")
    
    # Get and store original color
    original_color = get_material_color(stage, MATERIAL_PRIM_PATH)
    print(f"Original material color: {original_color}")
    
    # Create a render product for the camera
    print("\nSetting up camera render product...")
    render_product = rep.create.render_product(
        CAMERA_PRIM_PATH,
        resolution=RESOLUTION,
        name="color_variation_rp"
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
    
    # Generate color range
    print(f"\nGenerating {NUM_COLOR_STEPS} color variations ({COLOR_MODE})...")
    colors = generate_color_range(NUM_COLOR_STEPS, COLOR_MODE)
    print(f"Color range generated: {len(colors)} colors")
    
    # Trigger capture loop
    print("\nStarting color variation capture...")
    
    async def capture_color_variations():
        """Async function to capture images at each color variation."""
        # Give the app a few updates to ensure everything is ready
        print("Initializing render...")
        for i in range(5):
            await omni.kit.app.get_app().next_update_async()
        
        # Iterate through colors and capture
        for idx, color in enumerate(colors):
            print(f"\n[{idx+1}/{len(colors)}] Setting color to RGB({color[0]:.2f}, {color[1]:.2f}, {color[2]:.2f})")
            
            # Set the new color
            success = set_material_color(stage, MATERIAL_PRIM_PATH, color)
            if not success:
                print(f"  WARNING: Failed to set material color")
            
            # Give a moment for the material to update
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
        
        # Revert to original color
        print(f"\n{'='*60}")
        print("Reverting to original color...")
        if original_color:
            set_material_color(stage, MATERIAL_PRIM_PATH, original_color)
            print(f"Original color restored: {original_color}")
        else:
            print("WARNING: Could not restore original color (was not captured)")
        
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
            print(f"2. Create video using create_video.py")
        else:
            print("✗ FAILED: No images were created.")
        
        print("=" * 60)
        
        # Clean up
        writer.detach()
        render_product.destroy()
    
    # Run the async capture
    asyncio.ensure_future(capture_color_variations())
    
    print("\nCapture task scheduled. Please wait...")
    print(f"This will capture {NUM_COLOR_STEPS} images...")

# Run the main function
if __name__ == "__main__":
    main()
