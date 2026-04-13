#!/usr/bin/env python3
"""
GoPro IMU Extractor
===================
Extracts 6-DOF IMU data (accelerometer + gyroscope) from GoPro MP4 files
and outputs Ego4D-style CSV files at ~200Hz.

Also strips audio from videos and optionally clips them into segments.

Usage:
    python extract.py                        # Process all MP4s in ./input
    python extract.py --clip 180             # Also clip into 180-second segments
    python extract.py --input /path/to/vids  # Custom input directory

Output format (Ego4D 6-DOF standard):
    timestamp, accl_x, accl_y, accl_z, gyro_x, gyro_y, gyro_z
    - Accelerometer in m/s², Gyroscope in rad/s
    - ~200 Hz sampling rate
    - Timestamps in seconds, starting from 0.0

Compatible with: Ego4D, Epic-Kitchens, Assembly101, Project Aria pipelines.
"""

import argparse
import csv
import os
import shutil
import struct
import subprocess
import sys
import tempfile


# ---------------------------------------------------------------------------
# GPMF Parsing (GoPro Metadata Format)
# ---------------------------------------------------------------------------

def _check_dependencies():
    """Verify required packages are installed."""
    try:
        from gopro_overlay.gpmf.gpmf import GPMD  # noqa: F401
    except ImportError:
        print("ERROR: 'gopro-overlay' package is not installed.")
        print("       Run: pip install gopro-overlay")
        sys.exit(1)

    # Check ffmpeg
    try:
        subprocess.run(["ffmpeg", "-version"], capture_output=True, check=True)
    except FileNotFoundError:
        print("ERROR: 'ffmpeg' is not installed or not in PATH.")
        print("       Install it: https://ffmpeg.org/download.html")
        sys.exit(1)


def _extract_gpmf_binary(mp4_path: str) -> bytes:
    """Extract raw GPMF telemetry binary from a GoPro MP4 using gopro-extract."""
    extract_script = shutil.which("gopro-extract.py")
    if not extract_script:
        # Fallback: look in user's local bin
        fallback = os.path.expanduser("~/.local/bin/gopro-extract.py")
        if os.path.exists(fallback):
            extract_script = fallback
        else:
            print("ERROR: 'gopro-extract.py' not found. Install gopro-overlay.")
            sys.exit(1)

    tmp = tempfile.NamedTemporaryFile(suffix=".bin", delete=False)
    tmp.close()
    try:
        subprocess.run(
            [sys.executable, extract_script, mp4_path, tmp.name],
            check=True, capture_output=True,
        )
        with open(tmp.name, "rb") as f:
            return f.read()
    finally:
        if os.path.exists(tmp.name):
            os.unlink(tmp.name)


def _get_scale(stream_items: list):
    """Extract SCAL (scale factor) from a GPMF stream."""
    for item in stream_items:
        if item.fourcc == "SCAL":
            raw = item.rawdata
            tc = item.type_char
            if tc == "s":
                vals = struct.unpack(f">{len(raw) // 2}h", raw)
            elif tc == "l":
                vals = struct.unpack(f">{len(raw) // 4}l", raw)
            elif tc == "f":
                vals = struct.unpack(f">{len(raw) // 4}f", raw)
            else:
                return 1
            non_zero = [v for v in vals if v != 0]
            if len(non_zero) == 1:
                return non_zero[0]
            elif len(non_zero) > 1:
                return list(vals)
            return 1
    return 1


def _parse_samples(item, n_components: int, scale):
    """Parse raw GPMF sensor data into scaled float rows."""
    raw = item.rawdata
    tc = item.type_char
    sample_bytes = item.size
    n_samples = item.repeat

    rows = []
    for i in range(n_samples):
        offset = i * sample_bytes
        if offset + sample_bytes > len(raw):
            break
        if tc == "s":
            vals = struct.unpack(f">{n_components}h", raw[offset : offset + n_components * 2])
        elif tc == "f":
            vals = struct.unpack(f">{n_components}f", raw[offset : offset + n_components * 4])
        elif tc == "l":
            vals = struct.unpack(f">{n_components}l", raw[offset : offset + n_components * 4])
        else:
            continue

        if isinstance(scale, list):
            scaled = [v / s if s != 0 else v for v, s in zip(vals, scale)]
        else:
            scaled = [v / scale if scale != 0 else v for v in vals]
        rows.append(scaled)

    return rows


def _get_timestamp_us(stream_items: list):
    """Get STMP (timestamp in microseconds) from a GPMF stream."""
    for item in stream_items:
        if item.fourcc == "STMP":
            raw = item.rawdata
            if len(raw) >= 8:
                return struct.unpack(">Q", raw[:8])[0]
            elif len(raw) >= 4:
                return struct.unpack(">L", raw[:4])[0]
    return None


# ---------------------------------------------------------------------------
# Core Extraction
# ---------------------------------------------------------------------------

def extract_imu(mp4_path: str) -> tuple[list, list]:
    """
    Extract accelerometer and gyroscope data from a GoPro MP4.

    Returns:
        (accl_data, gyro_data) — each is a list of [timestamp_sec, x, y, z]
    """
    from gopro_overlay.gpmf.gpmf import GPMD

    gpmf_data = _extract_gpmf_binary(mp4_path)
    items = list(GPMD.parse(gpmf_data))

    accl_data = []
    gyro_data = []

    for devc in items:
        if devc.fourcc != "DEVC":
            continue
        for stream in devc.items:
            if stream.fourcc != "STRM":
                continue

            stream_items = list(stream.items)
            scale = _get_scale(stream_items)
            stmp = _get_timestamp_us(stream_items)

            for sub in stream_items:
                if sub.fourcc in ("ACCL", "GYRO"):
                    rows = _parse_samples(sub, 3, scale)
                    n = len(rows)
                    base_us = stmp if stmp is not None else 0
                    dt_us = (1_000_000.0 / 30.0) / max(n, 1)

                    target = accl_data if sub.fourcc == "ACCL" else gyro_data
                    for i, row in enumerate(rows):
                        t_sec = (base_us + i * dt_us) / 1_000_000.0
                        target.append([t_sec] + row)

    return accl_data, gyro_data


def write_ego4d_csv(path: str, accl: list, gyro: list, t_offset: float = 0.0) -> int:
    """
    Write Ego4D-style 6-DOF IMU CSV.

    Timestamps are rebased so the first sample starts at 0.0.
    Returns the number of samples written.
    """
    n = min(len(accl), len(gyro))
    if n == 0:
        return 0

    base_t = accl[0][0] if t_offset == 0.0 else t_offset

    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["timestamp", "accl_x", "accl_y", "accl_z", "gyro_x", "gyro_y", "gyro_z"])
        for i in range(n):
            a, g = accl[i], gyro[i]
            t = a[0] - base_t
            writer.writerow([
                f"{t:.6f}",
                f"{a[1]:.6f}", f"{a[2]:.6f}", f"{a[3]:.6f}",
                f"{g[1]:.6f}", f"{g[2]:.6f}", f"{g[3]:.6f}",
            ])
    return n


# ---------------------------------------------------------------------------
# Video Processing (strip audio, clip)
# ---------------------------------------------------------------------------

def get_video_duration(mp4_path: str) -> float:
    """Get video duration in seconds using ffprobe."""
    result = subprocess.run(
        ["ffprobe", "-v", "error", "-show_entries", "format=duration",
         "-of", "default=noprint_wrappers=1:nokey=1", mp4_path],
        capture_output=True, text=True,
    )
    return float(result.stdout.strip())


def strip_audio(input_path: str, output_path: str):
    """Remove audio track from video (stream copy, no re-encoding)."""
    subprocess.run(
        ["ffmpeg", "-y", "-i", input_path, "-an", "-c:v", "copy", output_path],
        capture_output=True, check=True,
    )


def clip_video(input_path: str, output_path: str, start_sec: float, duration_sec: float):
    """Extract a clip from a video (stream copy)."""
    cmd = ["ffmpeg", "-y", "-i", input_path, "-ss", str(start_sec), "-c:v", "copy"]
    if duration_sec is not None:
        cmd += ["-t", str(duration_sec)]
    cmd.append(output_path)
    subprocess.run(cmd, capture_output=True, check=True)


def slice_imu(accl: list, gyro: list, start_sec: float, end_sec: float) -> tuple[list, list]:
    """Slice IMU data to a time range (using original timestamps before rebasing)."""
    # The raw timestamps from extract_imu are absolute (not rebased).
    # We need to convert start/end relative to the video into absolute timestamps.
    if not accl:
        return [], []
    video_t0 = accl[0][0]
    abs_start = video_t0 + start_sec
    abs_end = video_t0 + end_sec

    sliced_accl = [row for row in accl if abs_start <= row[0] < abs_end]
    sliced_gyro = [row for row in gyro if abs_start <= row[0] < abs_end]
    return sliced_accl, sliced_gyro


# ---------------------------------------------------------------------------
# Main Pipeline
# ---------------------------------------------------------------------------

def process_all(input_dir: str, output_dir: str, clip_duration: float | None = None):
    """
    Process all GoPro MP4 files in input_dir.

    For each video:
      1. Strip audio -> output video
      2. Extract IMU -> Ego4D CSV
      3. (Optional) Clip into segments with matching IMU CSVs
    """
    # Discover MP4 files
    mp4_files = sorted([
        f for f in os.listdir(input_dir)
        if f.upper().endswith(".MP4")
    ])

    if not mp4_files:
        print(f"No MP4 files found in: {input_dir}")
        print("Place your GoPro videos in the 'input' folder and run again.")
        return

    print(f"\nFound {len(mp4_files)} video(s) in: {input_dir}")
    print(f"Output directory: {output_dir}\n")

    for mp4_name in mp4_files:
        mp4_path = os.path.join(input_dir, mp4_name)
        basename = os.path.splitext(mp4_name)[0]

        print(f"{'=' * 60}")
        print(f"Processing: {mp4_name}")
        print(f"{'=' * 60}")

        # --- Step 1: Extract IMU ---
        print("  [1/3] Extracting IMU telemetry...")
        accl, gyro = extract_imu(mp4_path)
        n_samples = min(len(accl), len(gyro))
        if n_samples == 0:
            print("  WARNING: No IMU data found. Skipping this file.")
            continue
        duration = accl[-1][0] - accl[0][0]
        hz = n_samples / duration if duration > 0 else 0
        print(f"         {n_samples:,} samples | {duration:.1f}s | ~{hz:.0f} Hz")

        # --- Step 2: Strip audio ---
        print("  [2/3] Stripping audio...")
        noaudio_path = os.path.join(output_dir, f"{basename}_noaudio.mp4")
        strip_audio(mp4_path, noaudio_path)
        print(f"         -> {os.path.basename(noaudio_path)}")

        # --- Step 3: Output (full or clipped) ---
        if clip_duration is None:
            # No clipping — write full video + full IMU
            print("  [3/3] Writing full IMU CSV...")
            imu_path = os.path.join(output_dir, f"{basename}_imu_200hz.csv")
            n = write_ego4d_csv(imu_path, accl, gyro)
            print(f"         -> {os.path.basename(imu_path)} ({n:,} samples)")
        else:
            # Clip into segments
            total_dur = get_video_duration(mp4_path)
            n_clips = int(total_dur // clip_duration) + (1 if total_dur % clip_duration > 0 else 0)
            print(f"  [3/3] Clipping into {n_clips} segment(s) of {clip_duration}s...")

            for clip_idx in range(n_clips):
                start = clip_idx * clip_duration
                end = min(start + clip_duration, total_dur)
                actual_dur = end - start

                start_label = int(start)
                end_label = int(end)
                clip_tag = f"clip{clip_idx + 1:02d}_{start_label}s_{end_label}s"

                # Clip video
                clip_path = os.path.join(output_dir, f"{basename}_{clip_tag}.mp4")
                clip_video(noaudio_path, clip_path, start, actual_dur)

                # Slice IMU
                sliced_accl, sliced_gyro = slice_imu(accl, gyro, start, end)
                imu_clip_path = os.path.join(output_dir, f"{basename}_{clip_tag}_imu_200hz.csv")
                # Rebase timestamps: the slice's first sample should map to ~0.0
                video_t0 = accl[0][0]
                n_written = write_ego4d_csv(imu_clip_path, sliced_accl, sliced_gyro, t_offset=video_t0 + start)

                print(f"         {clip_tag}: video + IMU ({n_written:,} samples)")

            # Remove the full noaudio file since we have clips
            os.remove(noaudio_path)

        print()

    print(f"{'=' * 60}")
    print("All done!")
    print(f"Output files are in: {output_dir}")
    print(f"{'=' * 60}")


def main():
    parser = argparse.ArgumentParser(
        description="Extract 6-DOF IMU data from GoPro videos (Ego4D format).",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python extract.py                          # Process all MP4s in ./input
  python extract.py --clip 180               # Clip into 3-minute segments
  python extract.py --input /path/to/videos  # Custom input path
  python extract.py --no-confirm             # Skip the deletion confirmation

Output format (Ego4D 6-DOF):
  timestamp, accl_x, accl_y, accl_z, gyro_x, gyro_y, gyro_z
  Units: m/s² (accel), rad/s (gyro) | Rate: ~200 Hz
        """,
    )
    parser.add_argument(
        "--input", "-i",
        default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "input"),
        help="Directory containing GoPro MP4 files (default: ./input)",
    )
    parser.add_argument(
        "--output", "-o",
        default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "output"),
        help="Directory for processed output (default: ./output)",
    )
    parser.add_argument(
        "--clip", "-c",
        type=float, default=None,
        metavar="SECONDS",
        help="Clip videos into segments of this duration (e.g., --clip 180 for 3 min)",
    )
    parser.add_argument(
        "--no-confirm",
        action="store_true",
        help="Skip confirmation prompt before deleting the output folder",
    )

    args = parser.parse_args()

    _check_dependencies()

    # Validate input
    if not os.path.isdir(args.input):
        print(f"ERROR: Input directory not found: {args.input}")
        print("       Create it and place your GoPro MP4 files inside.")
        sys.exit(1)

    # Warn about output deletion
    if os.path.exists(args.output):
        existing = [f for f in os.listdir(args.output) if not f.startswith(".")]
        if existing:
            print(f"WARNING: Output folder '{args.output}' is not empty.")
            print("         All existing files will be DELETED before processing.\n")
            if not args.no_confirm:
                response = input("Continue? [y/N]: ").strip().lower()
                if response != "y":
                    print("Aborted.")
                    sys.exit(0)
            for f in existing:
                p = os.path.join(args.output, f)
                if os.path.isdir(p):
                    shutil.rmtree(p)
                else:
                    os.remove(p)

    os.makedirs(args.output, exist_ok=True)

    process_all(args.input, args.output, clip_duration=args.clip)


if __name__ == "__main__":
    main()
