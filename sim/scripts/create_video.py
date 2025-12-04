#!/usr/bin/env python3
"""
create_video.py

Converts a sequence of PNG images into an MP4 video or GIF using ffmpeg.

Usage:
    python create_video.py

This script will:
1. Find all PNG images in the med_color results directory
2. Create a high-quality MP4 video with smooth playback
3. Optionally create an optimized GIF

Requirements:
    - ffmpeg must be installed (sudo apt install ffmpeg)
"""

import os
import subprocess
import glob
from pathlib import Path


# Configuration
INPUT_DIR = "/home/nick/git/synthetic_data/results/med_color"
OUTPUT_DIR = "/home/nick/git/synthetic_data/results"
VIDEO_NAME = "med_color_variation"
FRAMERATE = 15  # fps - lower = slower/smoother, higher = faster


def find_images(directory):
    """Find all PNG images in the directory and return sorted list."""
    # Try different possible patterns
    patterns = [
        os.path.join(directory, "rgb_*.png"),
        os.path.join(directory, "rgb", "*.png"),
        os.path.join(directory, "*.png"),
    ]
    
    for pattern in patterns:
        images = sorted(glob.glob(pattern))
        if images:
            print(f"Found {len(images)} images matching pattern: {pattern}")
            return images
    
    return []


def create_mp4(input_pattern, output_path, framerate=15):
    """Create high-quality MP4 video from image sequence."""
    print(f"\nCreating MP4 video: {output_path}")
    
    cmd = [
        "ffmpeg",
        "-y",  # Overwrite output file
        "-framerate", str(framerate),
        "-pattern_type", "glob",
        "-i", input_pattern,
        "-c:v", "libx264",
        "-pix_fmt", "yuv420p",
        "-crf", "18",  # Quality (lower = better, 18 is visually lossless)
        "-preset", "slow",  # Slower encoding = better compression
        output_path
    ]
    
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, check=True)
        print(f"✓ MP4 created successfully: {output_path}")
        return True
    except subprocess.CalledProcessError as e:
        print(f"✗ Error creating MP4: {e}")
        print(f"stderr: {e.stderr}")
        return False


def create_gif(input_pattern, output_path, framerate=15):
    """Create optimized GIF with palette for better colors."""
    print(f"\nCreating GIF: {output_path}")
    
    palette_path = "/tmp/palette.png"
    
    # Step 1: Generate palette
    palette_cmd = [
        "ffmpeg",
        "-y",
        "-framerate", str(framerate),
        "-pattern_type", "glob",
        "-i", input_pattern,
        "-vf", "palettegen",
        palette_path
    ]
    
    try:
        subprocess.run(palette_cmd, capture_output=True, text=True, check=True)
        print("  Generated color palette")
    except subprocess.CalledProcessError as e:
        print(f"✗ Error generating palette: {e.stderr}")
        return False
    
    # Step 2: Create GIF using palette
    gif_cmd = [
        "ffmpeg",
        "-y",
        "-framerate", str(framerate),
        "-pattern_type", "glob",
        "-i", input_pattern,
        "-i", palette_path,
        "-lavfi", "paletteuse",
        "-loop", "0",  # Loop forever
        output_path
    ]
    
    try:
        subprocess.run(gif_cmd, capture_output=True, text=True, check=True)
        print(f"✓ GIF created successfully: {output_path}")
        
        # Clean up palette
        if os.path.exists(palette_path):
            os.remove(palette_path)
        
        return True
    except subprocess.CalledProcessError as e:
        print(f"✗ Error creating GIF: {e.stderr}")
        return False


def main():
    print("=" * 70)
    print("Med Color Video Generator")
    print("=" * 70)
    
    # Check if ffmpeg is available
    try:
        subprocess.run(["ffmpeg", "-version"], capture_output=True, check=True)
    except (subprocess.CalledProcessError, FileNotFoundError):
        print("✗ Error: ffmpeg not found. Install it with: sudo apt install ffmpeg")
        return
    
    # Find images
    images = find_images(INPUT_DIR)
    
    if not images:
        print(f"✗ No images found in {INPUT_DIR}")
        print("  Run the med_color_capture.py script in Isaac Sim first.")
        return
    
    print(f"Found {len(images)} images")
    
    # Determine input pattern based on where images are
    if "rgb_" in images[0]:
        # Images are directly in INPUT_DIR
        input_pattern = os.path.join(INPUT_DIR, "rgb_*.png")
    else:
        # Images are in rgb/ subdirectory
        input_pattern = os.path.join(INPUT_DIR, "rgb", "*.png")
    
    # Create output directory if needed
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    
    # Generate MP4
    mp4_path = os.path.join(OUTPUT_DIR, f"{VIDEO_NAME}.mp4")
    mp4_success = create_mp4(input_pattern, mp4_path, FRAMERATE)
    
    # Generate GIF
    gif_path = os.path.join(OUTPUT_DIR, f"{VIDEO_NAME}.gif")
    gif_success = create_gif(input_pattern, gif_path, FRAMERATE)
    
    # Summary
    print("\n" + "=" * 70)
    print("SUMMARY")
    print("=" * 70)
    print(f"Input images: {len(images)} frames")
    print(f"Frame rate: {FRAMERATE} fps")
    print(f"Duration: {len(images) / FRAMERATE:.1f} seconds")
    
    if mp4_success:
        mp4_size = os.path.getsize(mp4_path) / 1024 / 1024
        print(f"\n✓ MP4: {mp4_path} ({mp4_size:.2f} MB)")
    
    if gif_success:
        gif_size = os.path.getsize(gif_path) / 1024 / 1024
        print(f"✓ GIF: {gif_path} ({gif_size:.2f} MB)")
    
    print("\nTip: For smoother/slower playback, decrease FRAMERATE in this script")
    print("     For faster playback, increase FRAMERATE")
    print("=" * 70)


if __name__ == "__main__":
    main()
