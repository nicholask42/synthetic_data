#!/usr/bin/env python3
"""
create_video.py

Converts a sequence of PNG images into an MP4 video or GIF using ffmpeg.

Usage:
    python3 sim/scripts/create_video.py  (from repo root)
    OR
    python3 create_video.py  (from sim/scripts directory)

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
import argparse


# Configuration - use relative paths from script location
SCRIPT_DIR = Path(__file__).parent.resolve()
REPO_ROOT = SCRIPT_DIR.parent.parent
DEFAULT_INPUT_DIR = REPO_ROOT / "results" / "med_color"

# Output directories for videos and gifs
VIDEO_OUTPUT_DIR = REPO_ROOT / "results" / "videos"
GIF_OUTPUT_DIR = REPO_ROOT / "results" / "gifs"
DEFAULT_FRAMERATE = 10  # fps - lower = slower/smoother, higher = faster


def find_images(directory):
    """Find all PNG images in the directory and return sorted list."""
    directory = Path(directory)
    
    # Try different possible patterns
    patterns = [
        directory / "rgb_*.png",
        directory / "rgb" / "*.png",
        directory / "*.png",
    ]
    
    for pattern in patterns:
        images = sorted(glob.glob(str(pattern)))
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
    parser = argparse.ArgumentParser(description="Convert a sequence of PNG images into an MP4 video and GIF using ffmpeg.")
    parser.add_argument("input_dir", type=str, nargs="?", default=str(DEFAULT_INPUT_DIR), help="Directory containing PNG images (default: results/med_color)")
    parser.add_argument("--framerate", type=int, default=DEFAULT_FRAMERATE, help="Frame rate for video and GIF (default: 10)")
    args = parser.parse_args()

    input_dir = Path(args.input_dir).resolve()
    framerate = args.framerate
    folder_name = input_dir.name

    print("=" * 70)
    print(f"Video Generator for: {folder_name}")
    print("=" * 70)
    print(f"Script directory: {SCRIPT_DIR}")
    print(f"Repository root: {REPO_ROOT}")
    print(f"Input directory: {input_dir}")
    print(f"Video output directory: {VIDEO_OUTPUT_DIR}")
    print(f"GIF output directory: {GIF_OUTPUT_DIR}")
    print()
    
    # Check if ffmpeg is available
    try:
        subprocess.run(["ffmpeg", "-version"], capture_output=True, check=True)
    except (subprocess.CalledProcessError, FileNotFoundError):
        print("✗ Error: ffmpeg not found. Install it with: sudo apt install ffmpeg")
        return
    
    # Find images
    images = find_images(input_dir)
    
    if not images:
        print(f"✗ No images found in {input_dir}")
        print("  Make sure the directory contains PNG images.")
        return
    
    print(f"Found {len(images)} images")
    
    # Determine input pattern based on where images are
    first_image = Path(images[0])
    if first_image.parent == input_dir:
        # Images are directly in input_dir
        input_pattern = str(input_dir / "rgb_*.png")
        if not glob.glob(input_pattern):
            input_pattern = str(input_dir / "*.png")
    else:
        # Images are in rgb/ subdirectory
        input_pattern = str(input_dir / "rgb" / "*.png")
    
    # Create output directories if needed
    VIDEO_OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    GIF_OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    # Generate MP4
    mp4_path = str(VIDEO_OUTPUT_DIR / f"{folder_name}.mp4")
    mp4_success = create_mp4(input_pattern, mp4_path, framerate)

    # Generate GIF
    gif_path = str(GIF_OUTPUT_DIR / f"{folder_name}.gif")
    gif_success = create_gif(input_pattern, gif_path, framerate)
    
    # Summary
    print("\n" + "=" * 70)
    print("SUMMARY")
    print("=" * 70)
    print(f"Input images: {len(images)} frames")
    print(f"Frame rate: {framerate} fps")
    print(f"Duration: {len(images) / framerate:.1f} seconds")
    
    if mp4_success:
        mp4_size = os.path.getsize(mp4_path) / 1024 / 1024
        print(f"\n✓ MP4: {mp4_path} ({mp4_size:.2f} MB)")
    
    if gif_success:
        gif_size = os.path.getsize(gif_path) / 1024 / 1024
        print(f"✓ GIF: {gif_path} ({gif_size:.2f} MB)")
    
    print("\nTip: For smoother/slower playback, decrease --framerate")
    print("     For faster playback, increase --framerate")
    print("=" * 70)


if __name__ == "__main__":
    main()
