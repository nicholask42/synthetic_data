"""
test_single_capture.py

Isaac Sim 5.1 Script Editor compatible script for testing single image capture.

Usage:
1. Open your USD scene in Isaac Sim 5.1
2. Open Window → Script Editor
3. Copy and paste this ENTIRE script
4. Click "Run" or press Ctrl+Enter
5. Check the output directory for the captured image

This script:
- Captures a single RGB image from the specified camera
- Saves it to the output directory
- Tests the camera and writer setup before implementing color variations
"""

# ----------------------
# Configuration
# ----------------------
CAMERA_PRIM_PATH = "/World/rsd455/RSD455/Camera_OmniVision_OV9782_Color"
OUTPUT_DIR = "/home/nick/git/synthetic_data/results/med_color"
RESOLUTION = (1280, 720)

# ----------------------
# Imports
# ----------------------
import omni.replicator.core as rep
from omni.isaac.core.utils.stage import get_current_stage
import omni.usd
import os
import time
import glob
from pathlib import Path
import asyncio
import omni.kit.app

# ----------------------
# Main Script
# ----------------------
def main():
    """Capture a single test image from the camera."""
    
    print("=" * 60)
    print("Isaac Sim 5.1 - Single Image Capture Test")
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
        print("Please check the camera path in your scene.")
        return
    
    print(f"Camera found: {CAMERA_PRIM_PATH}")
    print(f"Resolution: {RESOLUTION[0]}x{RESOLUTION[1]}")
    
    # Create a render product for the camera
    print("\nSetting up camera render product...")
    render_product = rep.create.render_product(
        CAMERA_PRIM_PATH,
        resolution=RESOLUTION,
        name="test_camera_rp"
    )
    
    print(f"Render product created: {render_product}")
    print(f"Render product path: {render_product.path if hasattr(render_product, 'path') else 'N/A'}")
    print(f"Render product resolution: {render_product.resolution if hasattr(render_product, 'resolution') else 'N/A'}")
    
    # Create a basic writer to save RGB images
    print("Setting up writer...")
    writer = rep.writers.get("BasicWriter")
    print(f"Writer object: {writer}")
    print(f"Writer type: {type(writer)}")
    
    writer.initialize(
        output_dir=OUTPUT_DIR,
        rgb=True,
        colorize_instance_segmentation=False,
        colorize_semantic_segmentation=False
    )
    print(f"Writer initialized with output_dir: {OUTPUT_DIR}")
    
    # Attach the render product to the writer
    print(f"Attaching render product: {render_product}")
    writer.attach([render_product])
    print(f"Writer attached to render product")
    print(f"Writer backend: {writer._backend_type if hasattr(writer, '_backend_type') else 'Unknown'}")
    print(f"Writer annotators: {writer._annotators if hasattr(writer, '_annotators') else 'Unknown'}")
    
    # Count existing images before capture
    existing_images = glob.glob(os.path.join(OUTPUT_DIR, "**/*.png"), recursive=True)
    num_existing = len(existing_images)
    print(f"Existing images in output directory: {num_existing}")
    
    # Trigger capture using async method
    print("\nCapturing image...")
    
    async def capture_and_verify():
        """Async function to capture a single frame and verify output."""
        # Give the app a few updates to ensure everything is ready
        print("\nInitializing render...")
        for i in range(5):
            await omni.kit.app.get_app().next_update_async()
        
        # Trigger ONE capture step directly (don't call run() first)
        print("Triggering single replicator step...")
        try:
            # Call step_async directly without run() to get exactly one capture
            await rep.orchestrator.step_async(rt_subframes=4, delta_time=0.0, pause_timeline=True)
            print(f"Step completed successfully")
        except Exception as e:
            print(f"ERROR during step: {e}")
            import traceback
            traceback.print_exc()
            return
        
        # Wait for all data to be written to disk (CRITICAL!)
        print("Waiting for data to be written to disk...")
        try:
            await rep.orchestrator.wait_until_complete_async()
            print("Write complete signal received")
        except Exception as e:
            print(f"ERROR during wait_until_complete: {e}")
            import traceback
            traceback.print_exc()
        
        # Additional wait to ensure filesystem sync
        print("Allowing filesystem sync...")
        await asyncio.sleep(1.0)
        
        # Verify the output
        print("\n" + "=" * 60)
        print("Verifying output...")
        print("=" * 60)
        
        # Check for any files created
        all_files_recursive = []
        for root, dirs, files in os.walk(OUTPUT_DIR):
            for file in files:
                all_files_recursive.append(os.path.join(root, file))
        
        print(f"Total files in output directory (recursive): {len(all_files_recursive)}")
        
        new_images = glob.glob(os.path.join(OUTPUT_DIR, "**/*.png"), recursive=True)
        num_new = len(new_images)
        
        print(f"PNG images found: {num_new}")
        
        if num_new > num_existing:
            print(f"✓ SUCCESS: {num_new - num_existing} new image(s) captured!")
            print(f"\nNew image files:")
            for img in sorted(new_images):
                if img not in existing_images:
                    file_size = os.path.getsize(img)
                    rel_path = os.path.relpath(img, OUTPUT_DIR)
                    print(f"  - {rel_path} ({file_size:,} bytes)")
        else:
            print("✗ FAILED: No new images were created.")
            print("\nTroubleshooting:")
            print("1. Check if the camera is visible in the viewport")
            print("2. Verify the scene has geometry visible to the camera")
            print("3. Ensure the camera path is correct")
            
            # Show all files that were created
            if all_files_recursive:
                print(f"\nAll files found in output directory:")
                for file_path in all_files_recursive:
                    rel_path = os.path.relpath(file_path, OUTPUT_DIR)
                    file_size = os.path.getsize(file_path)
                    print(f"  - {rel_path} ({file_size:,} bytes)")
            
            # Check directory structure
            print(f"\nDirectory structure of {OUTPUT_DIR}:")
            for root, dirs, files in os.walk(OUTPUT_DIR):
                level = root.replace(OUTPUT_DIR, '').count(os.sep)
                indent = ' ' * 2 * level
                print(f'{indent}{os.path.basename(root)}/')
                subindent = ' ' * 2 * (level + 1)
                for file in files:
                    print(f'{subindent}{file}')
            
            # Additional diagnostic info
            print(f"\nDiagnostic information:")
            print(f"  Writer is valid: {writer is not None}")
            print(f"  Render product is valid: {render_product is not None}")
            print(f"  Output directory exists: {os.path.exists(OUTPUT_DIR)}")
            print(f"  Output directory is writable: {os.access(OUTPUT_DIR, os.W_OK)}")
        
        print("=" * 60)
        
        # Clean up
        writer.detach()
        render_product.destroy()
    
    # Run the async capture
    asyncio.ensure_future(capture_and_verify())
    
    print("Capture task scheduled. Please wait...")

# Run the main function
if __name__ == "__main__":
    main()
