"""
EPC901 Polar Image Reconstruction

Each saved frame is one radial slice (342 8-bit pixels along the radius,
stride-3 subsampled from 1024 — every 3rd pixel was transmitted over BLE).
Stacking frames as the sensor rotates produces a 360° polar image.

At 60Hz rotation and ~0.712ms/frame (512µs readout at 2Msps + overhead):
  - 108 frames span ~7.7 rotations  (same as before — BLE subsampling is
    applied after capture, so frame count and rotation geometry are unchanged)
  - ~14 unique angular positions → ~25.9° angular resolution
  - ~7-8 samples per position averaged for noise reduction

Frames are mapped evenly 0°→360° regardless of rotation count.
Pixel indices correspond to physical positions 0, 3, 6, ..., 1023 on the sensor.

Usage:
    python3 plot_polar.py
    python3 plot_polar.py --frames_dir frames --output polar.png
"""

import numpy as np
import matplotlib.pyplot as plt
import os
import argparse

# Must match PIXEL_STRIDE and PIXELS_PER_FRAME in save_frames.py / main.c
PIXELS_PER_FRAME  = 1024
PIXEL_STRIDE      = 3
BLE_PIXELS        = (PIXELS_PER_FRAME + PIXEL_STRIDE - 1) // PIXEL_STRIDE  # 342


def load_frames(frame_dir):
    """Load all .npy frame files from directory."""
    files = sorted([f for f in os.listdir(frame_dir) if f.endswith('.npy')])
    if not files:
        raise ValueError(f"No .npy frames found in '{frame_dir}'")

    frames = []
    for f in files:
        frame = np.load(os.path.join(frame_dir, f))
        # Handle both old 10-bit (uint16) and new 8-bit (uint8) frames
        if frame.dtype == np.uint16:
            frame = (frame >> 2).astype(np.uint8)
        frames.append(frame)

    n_px = frames[0].shape[0]
    print(f"Loaded {len(frames)} frames, {n_px} pixels each "
          f"(dtype={frames[0].dtype}).")

    if n_px == BLE_PIXELS:
        print(f"  Stride-{PIXEL_STRIDE} subsampled frames detected "
              f"({BLE_PIXELS} px → maps to sensor positions 0,{PIXEL_STRIDE},...,"
              f"{(BLE_PIXELS-1)*PIXEL_STRIDE}).")
    elif n_px == PIXELS_PER_FRAME:
        print(f"  Full-resolution frames detected ({PIXELS_PER_FRAME} px).")
    else:
        print(f"  Warning: unexpected frame size ({n_px} px).")

    return np.stack(frames, axis=0)   # shape: (num_frames, num_pixels)


def build_polar_image(frames):
    """
    Convert (num_frames, num_pixels) → Cartesian image via polar mapping.

    Each frame is one radial slice:
      - Angle = frame index mapped evenly to 0°..360°
      - Radius = pixel index mapped to physical sensor position

    For stride-3 subsampled frames (342 px), pixel i maps to physical
    sensor position i * PIXEL_STRIDE. The polar image is built at full
    sensor resolution (2048 × 2048) with subsampled pixel positions.

    Multiple frames at the same angle (from multiple rotations) are averaged.
    """
    num_frames, num_pixels = frames.shape

    # Physical radius span of this frame set
    if num_pixels == BLE_PIXELS:
        # Map subsampled index → physical pixel index (0, 3, 6, ..., 1020)
        physical_indices = np.arange(num_pixels) * PIXEL_STRIDE
        max_radius = PIXELS_PER_FRAME
    else:
        physical_indices = np.arange(num_pixels)
        max_radius = num_pixels

    img_size = max_radius * 2
    cx, cy   = img_size // 2, img_size // 2

    cartesian = np.zeros((img_size, img_size), dtype=np.float32)
    count     = np.zeros((img_size, img_size), dtype=np.float32)

    angles = np.linspace(0, 2 * np.pi, num_frames, endpoint=False)

    for frame_idx, angle in enumerate(angles):
        xs = (cx + physical_indices * np.cos(angle)).astype(int)
        ys = (cy + physical_indices * np.sin(angle)).astype(int)

        valid = (xs >= 0) & (xs < img_size) & (ys >= 0) & (ys < img_size)
        np.add.at(cartesian, (ys[valid], xs[valid]), frames[frame_idx, valid])
        np.add.at(count,     (ys[valid], xs[valid]), 1)

    # Average overlapping pixels (from multiple rotations)
    mask = count > 0
    cartesian[mask] /= count[mask]

    return cartesian


def plot_results(frames, cartesian, output_path):
    """Plot raw frame stack and polar reconstruction side by side."""
    num_frames, num_pixels = frames.shape
    is_subsampled = (num_pixels == BLE_PIXELS)

    fig, axes = plt.subplots(1, 2, figsize=(16, 7))

    # --- Left: raw frame stack ---
    ax1 = axes[0]
    im1 = ax1.imshow(frames, cmap='gray', aspect='auto',
                     vmin=np.percentile(frames, 2),
                     vmax=np.percentile(frames, 98))
    plt.colorbar(im1, ax=ax1, label='Intensity (8-bit)')
    ax1.set_xlabel(f'Pixel (radius){" — stride-3 subsampled" if is_subsampled else ""}')
    ax1.set_ylabel('Frame index')
    stride_note = f"stride={PIXEL_STRIDE}, {num_pixels}/{PIXELS_PER_FRAME} px" \
                  if is_subsampled else f"{num_pixels} px"
    ax1.set_title(f'Raw Frames — {num_frames} frames × {num_pixels} pixels\n'
                  f'({stride_note}, ~{360/num_frames:.1f}° spacing if evenly distributed)')

    # --- Right: polar reconstruction ---
    ax2 = axes[1]
    valid_pixels = cartesian[cartesian > 0]
    vmin = np.percentile(valid_pixels, 2)  if len(valid_pixels) else 0
    vmax = np.percentile(valid_pixels, 98) if len(valid_pixels) else 255
    im2 = ax2.imshow(cartesian, cmap='gray', vmin=vmin, vmax=vmax)
    plt.colorbar(im2, ax=ax2, label='Intensity (8-bit, averaged)')
    ax2.set_title('Polar Reconstruction — top-down view\n'
                  f'{num_frames} frames mapped evenly 0°→360°'
                  f'{" (stride-3 BLE)" if is_subsampled else ""}')
    ax2.axis('off')

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"Saved to {output_path}")
    plt.show()


def main():
    parser = argparse.ArgumentParser(description='EPC901 polar image reconstruction')
    parser.add_argument('--frames_dir', type=str, default='frames',
                        help='Directory containing frame_XXXXX.npy files')
    parser.add_argument('--output', type=str, default='polar.png',
                        help='Output image filename')
    args = parser.parse_args()

    if not os.path.exists(args.frames_dir):
        print(f"Error: frames directory '{args.frames_dir}' not found.")
        print("Run save_frames.py first to capture frames.")
        return

    frames    = load_frames(args.frames_dir)
    cartesian = build_polar_image(frames)
    plot_results(frames, cartesian, args.output)


if __name__ == '__main__':
    main()