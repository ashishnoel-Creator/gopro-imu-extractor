#!/usr/bin/env python3
"""
GoPro IMU Extractor
===================
Extracts 6-DOF IMU data (accelerometer + gyroscope) from GoPro MP4 files
and outputs Ego4D-style CSV files at ~200Hz.

Works on BOTH older GoPros (Hero 5-10) and newer GoPros (Hero 11/12/13)
where the gyroscope is sampled at ~1600 Hz vs the accelerometer at ~200 Hz.

Also strips audio from videos and optionally clips them into segments.

Usage:
    python extract.py                        # Process all MP4s in ./input
    python extract.py --clip 180             # Also clip into 180-second segments
    python extract.py --input /path/to/vids  # Custom input directory

Output format (Ego4D 6-DOF standard):
    timestamp, accl_x, accl_y, accl_z, gyro_x, gyro_y, gyro_z
    - Accelerometer in m/s², Gyroscope in rad/s
    - ~200 Hz sampling rate (aligned to accelerometer timeline)
    - Timestamps in seconds, starting from 0.0

Compatible with: Ego4D, Epic-Kitchens, Assembly101, Project Aria pipelines.

Dependencies: ffmpeg (in PATH). No Python packages required beyond stdlib.
"""

import argparse
import csv
import math
import os
import shutil
import struct
import subprocess
import sys
import tempfile
from bisect import bisect_left


# ---------------------------------------------------------------------------
# GPMF Parsing (custom parser - works on all GoPro generations)
# ---------------------------------------------------------------------------

GPMD_HEADER = struct.Struct('>4sBBH')  # fourcc, type_char, size, repeat


def _pad4(n: int) -> int:
    return (n + 3) & ~3


def _check_dependencies():
    """Verify ffmpeg/ffprobe are available."""
    for tool in ("ffmpeg", "ffprobe"):
        try:
            subprocess.run([tool, "-version"], capture_output=True, check=True)
        except (FileNotFoundError, subprocess.CalledProcessError):
            print(f"ERROR: '{tool}' is not installed or not in PATH.")
            print("       Install it: https://ffmpeg.org/download.html")
            sys.exit(1)


def _extract_gpmf_binary(mp4_path: str) -> bytes:
    """
    Extract raw GPMF telemetry binary from a GoPro MP4 using ffmpeg.
    Locates the GPMF data stream (codec tag 'gpmd') and copies it out.
    """
    probe = subprocess.run(
        ["ffprobe", "-v", "error", "-select_streams", "d",
         "-show_entries", "stream=index,codec_tag_string",
         "-of", "default=noprint_wrappers=1", mp4_path],
        capture_output=True, text=True, check=True,
    )
    stream_idx = None
    cur_idx = None
    for line in probe.stdout.splitlines():
        line = line.strip()
        if line.startswith("index="):
            cur_idx = int(line.split("=", 1)[1])
        elif line.startswith("codec_tag_string=") and "gpmd" in line:
            stream_idx = cur_idx
            break
    if stream_idx is None:
        raise RuntimeError(f"No GPMF (gpmd) data stream found in {mp4_path}")

    tmp = tempfile.NamedTemporaryFile(suffix=".bin", delete=False)
    tmp.close()
    try:
        subprocess.run(
            ["ffmpeg", "-y", "-i", mp4_path,
             "-codec", "copy", "-map", f"0:{stream_idx}",
             "-f", "rawvideo", tmp.name],
            capture_output=True, check=True,
        )
        with open(tmp.name, "rb") as f:
            return f.read()
    finally:
        if os.path.exists(tmp.name):
            os.unlink(tmp.name)


def _walk_nested(data: bytes):
    """Walk nested items of a DEVC payload, yielding each STRM as a list of items."""
    offset = 0
    n = len(data)
    while offset < n:
        if offset + 8 > n:
            return
        try:
            fourcc, type_char, size, repeat = GPMD_HEADER.unpack_from(data, offset)
            fourcc_s = fourcc.decode('ascii')
        except (UnicodeDecodeError, struct.error):
            return
        payload_len = size * repeat
        padded = _pad4(payload_len)
        payload = data[offset + 8 : offset + 8 + payload_len]

        if fourcc_s == 'STRM' and type_char == 0:
            items = []
            sub_off = 0
            while sub_off < len(payload):
                if sub_off + 8 > len(payload):
                    break
                try:
                    f2, t2, s2, r2 = GPMD_HEADER.unpack_from(payload, sub_off)
                    f2s = f2.decode('ascii')
                except (UnicodeDecodeError, struct.error):
                    break
                p2_len = s2 * r2
                p2_pad = _pad4(p2_len)
                p2 = payload[sub_off + 8 : sub_off + 8 + p2_len]
                items.append((f2s, t2, s2, r2, p2))
                sub_off += 8 + p2_pad
            yield items

        offset += 8 + padded


def _find_devc_streams(data: bytes):
    """
    Iterate STRM blocks across all DEVC packets.
    Yields (devc_index, items) tuples.

    Uses DEVC marker search rather than strict byte-stride traversal so that
    fixed-stride zero-padded layouts (newer GoPros) also work.
    """
    positions = []
    off = 0
    while True:
        p = data.find(b'DEVC', off)
        if p == -1:
            break
        positions.append(p)
        off = p + 4
    positions.append(len(data))

    devc_idx = 0
    for i in range(len(positions) - 1):
        start = positions[i]
        try:
            fourcc, _tc, sz, rep = GPMD_HEADER.unpack_from(data, start)
            if fourcc != b'DEVC':
                continue
            payload_len = sz * rep
            payload = data[start + 8 : start + 8 + payload_len]
            for items in _walk_nested(payload):
                yield devc_idx, items
            devc_idx += 1
        except struct.error:
            continue


def _get_scale(items):
    for f, tc, _sz, _rep, payload in items:
        if f == 'SCAL':
            tcc = chr(tc)
            if tcc == 's':
                vals = struct.unpack(f'>{len(payload)//2}h', payload[:(len(payload)//2)*2])
            elif tcc == 'l':
                vals = struct.unpack(f'>{len(payload)//4}l', payload[:(len(payload)//4)*4])
            elif tcc == 'f':
                vals = struct.unpack(f'>{len(payload)//4}f', payload[:(len(payload)//4)*4])
            else:
                return 1
            non_zero = [v for v in vals if v != 0]
            if len(non_zero) == 1:
                return non_zero[0]
            elif len(non_zero) > 1:
                return list(vals)
    return 1


def _get_stmp(items):
    for f, _tc, _sz, _rep, payload in items:
        if f == 'STMP' and len(payload) >= 8:
            return struct.unpack('>Q', payload[:8])[0]
    return None


def _parse_3axis(payload, type_char, size, repeat, scale):
    rows = []
    tc = chr(type_char)
    for i in range(repeat):
        off = i * size
        if off + size > len(payload):
            break
        if tc == 's':
            vals = struct.unpack('>3h', payload[off:off+6])
        elif tc == 'f':
            vals = struct.unpack('>3f', payload[off:off+12])
        elif tc == 'l':
            vals = struct.unpack('>3l', payload[off:off+12])
        else:
            continue
        if isinstance(scale, list):
            scaled = [v / s if s != 0 else v for v, s in zip(vals, scale)]
        else:
            scaled = [v / scale if scale != 0 else v for v in vals]
        rows.append(scaled)
    return rows


# ---------------------------------------------------------------------------
# Core Extraction
# ---------------------------------------------------------------------------

def extract_imu(mp4_path: str):
    """
    Extract accelerometer and gyroscope data from a GoPro MP4.
    Returns (accl_data, gyro_data) where each is a list of [t_sec, x, y, z].
    Timestamps are absolute (relative to start of GPMF stream).
    """
    gpmf = _extract_gpmf_binary(mp4_path)

    accl_data = []
    gyro_data = []

    for devc_idx, items in _find_devc_streams(gpmf):
        scale = _get_scale(items)
        stmp = _get_stmp(items)
        # Each DEVC packet covers ~1 second of video.
        base_us = stmp if stmp is not None else devc_idx * 1_000_000

        for f, tc, sz, rep, payload in items:
            if f not in ('ACCL', 'GYRO'):
                continue
            rows = _parse_3axis(payload, tc, sz, rep, scale)
            n = len(rows)
            if n == 0:
                continue
            dt_us = 1_000_000.0 / n  # 1 second per DEVC packet
            target = accl_data if f == 'ACCL' else gyro_data
            for i, row in enumerate(rows):
                t_sec = (base_us + i * dt_us) / 1_000_000.0
                target.append([t_sec] + row)

    accl_data.sort(key=lambda r: r[0])
    gyro_data.sort(key=lambda r: r[0])
    return accl_data, gyro_data


def align_gyro_to_accl(accl, gyro):
    """
    Resample gyro onto the accelerometer timestamps using linear interpolation.
    Handles both older GoPros (gyro rate ~= accl rate) and newer GoPros
    (gyro at ~1600 Hz, accl at ~200 Hz).

    Returns a list of gyro rows [t, gx, gy, gz] aligned 1:1 with accl.
    """
    if not accl or not gyro:
        return []

    gyro_t = [g[0] for g in gyro]
    aligned = []
    for a in accl:
        t = a[0]
        idx = bisect_left(gyro_t, t)
        if idx <= 0:
            g0 = gyro[0]
            aligned.append([t, g0[1], g0[2], g0[3]])
            continue
        if idx >= len(gyro_t):
            g0 = gyro[-1]
            aligned.append([t, g0[1], g0[2], g0[3]])
            continue
        t0, t1 = gyro_t[idx - 1], gyro_t[idx]
        if t1 == t0:
            g0 = gyro[idx]
            aligned.append([t, g0[1], g0[2], g0[3]])
            continue
        f = (t - t0) / (t1 - t0)
        g0 = gyro[idx - 1]
        g1 = gyro[idx]
        aligned.append([t,
                        g0[1] + f * (g1[1] - g0[1]),
                        g0[2] + f * (g1[2] - g0[2]),
                        g0[3] + f * (g1[3] - g0[3])])
    return aligned


def write_ego4d_csv(path, accl, gyro_aligned, t_offset=None):
    """
    Write Ego4D-style 6-DOF IMU CSV. Timestamps rebased so first sample = 0.0.
    `gyro_aligned` must be 1:1 with `accl` (use align_gyro_to_accl first).
    """
    n = min(len(accl), len(gyro_aligned))
    if n == 0:
        return 0
    base_t = accl[0][0] if t_offset is None else t_offset
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["timestamp", "accl_x", "accl_y", "accl_z",
                    "gyro_x", "gyro_y", "gyro_z"])
        for i in range(n):
            a = accl[i]
            g = gyro_aligned[i]
            t = a[0] - base_t
            w.writerow([f"{t:.6f}",
                        f"{a[1]:.6f}", f"{a[2]:.6f}", f"{a[3]:.6f}",
                        f"{g[1]:.6f}", f"{g[2]:.6f}", f"{g[3]:.6f}"])
    return n


# ---------------------------------------------------------------------------
# Sanity checks
# ---------------------------------------------------------------------------

def sanity_check(accl, gyro):
    """Print warnings about anomalies in the IMU data."""
    warnings = []
    if not accl or not gyro:
        print("         ⚠  Empty IMU data!")
        return

    dur_a = accl[-1][0] - accl[0][0]
    dur_g = gyro[-1][0] - gyro[0][0]
    hz_a = len(accl) / dur_a if dur_a > 0 else 0
    hz_g = len(gyro) / dur_g if dur_g > 0 else 0

    if hz_a < 100 or hz_a > 500:
        warnings.append(f"Unusual accelerometer rate: {hz_a:.0f} Hz (expected ~200)")
    if hz_g < 100 or hz_g > 2500:
        warnings.append(f"Unusual gyroscope rate: {hz_g:.0f} Hz")

    mags = [math.sqrt(a[1]**2 + a[2]**2 + a[3]**2) for a in accl[:1000]]
    if mags:
        avg_mag = sum(mags) / len(mags)
        if avg_mag < 5 or avg_mag > 20:
            warnings.append(f"Abnormal accel magnitude: {avg_mag:.2f} (expected ~9.8 m/s²)")

    bad = False
    for data, name in ((accl, "accl"), (gyro, "gyro")):
        for row in data:
            for v in row[1:]:
                if not math.isfinite(v):
                    warnings.append(f"NaN/Inf detected in {name}")
                    bad = True
                    break
            if bad:
                break
        if bad:
            break

    max_gap = 0.0
    for i in range(1, len(accl)):
        d = accl[i][0] - accl[i-1][0]
        if d > max_gap:
            max_gap = d
    expected_dt = 1.0 / hz_a if hz_a > 0 else 0
    if expected_dt and max_gap > expected_dt * 10:
        warnings.append(f"Large time gap in accel: {max_gap*1000:.1f} ms")

    if warnings:
        for w in warnings:
            print(f"         ⚠  {w}")
    else:
        print(f"         ✓  Sanity checks passed (accl ~{hz_a:.0f}Hz, gyro ~{hz_g:.0f}Hz)")


# ---------------------------------------------------------------------------
# Video Processing
# ---------------------------------------------------------------------------

def get_video_duration(mp4_path):
    result = subprocess.run(
        ["ffprobe", "-v", "error", "-show_entries", "format=duration",
         "-of", "default=noprint_wrappers=1:nokey=1", mp4_path],
        capture_output=True, text=True,
    )
    return float(result.stdout.strip())


def strip_audio(input_path, output_path):
    subprocess.run(
        ["ffmpeg", "-y", "-i", input_path, "-an", "-c:v", "copy", output_path],
        capture_output=True, check=True,
    )


def clip_video(input_path, output_path, start_sec, duration_sec):
    cmd = ["ffmpeg", "-y", "-i", input_path, "-ss", str(start_sec), "-c:v", "copy"]
    if duration_sec is not None:
        cmd += ["-t", str(duration_sec)]
    cmd.append(output_path)
    subprocess.run(cmd, capture_output=True, check=True)


def slice_imu(accl, gyro_aligned, start_sec, end_sec):
    """Slice paired (accl, gyro_aligned) data to a time range relative to video start."""
    if not accl:
        return [], []
    video_t0 = accl[0][0]
    abs_start = video_t0 + start_sec
    abs_end = video_t0 + end_sec
    sa, sg = [], []
    for i, a in enumerate(accl):
        if abs_start <= a[0] < abs_end:
            sa.append(a)
            sg.append(gyro_aligned[i])
    return sa, sg


# ---------------------------------------------------------------------------
# Main Pipeline
# ---------------------------------------------------------------------------

def process_all(input_dir, output_dir, clip_duration=None):
    mp4_files = sorted([f for f in os.listdir(input_dir) if f.upper().endswith(".MP4")])
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

        print("  [1/3] Extracting IMU telemetry...")
        accl, gyro = extract_imu(mp4_path)
        if not accl or not gyro:
            print("  WARNING: No IMU data found. Skipping this file.")
            continue
        dur_a = accl[-1][0] - accl[0][0]
        dur_g = gyro[-1][0] - gyro[0][0]
        hz_a = len(accl) / dur_a if dur_a > 0 else 0
        hz_g = len(gyro) / dur_g if dur_g > 0 else 0
        print(f"         Raw: {len(accl):,} accl @ {hz_a:.0f}Hz | {len(gyro):,} gyro @ {hz_g:.0f}Hz")

        gyro_aligned = align_gyro_to_accl(accl, gyro)
        sanity_check(accl, gyro)

        print("  [2/3] Stripping audio...")
        noaudio_path = os.path.join(output_dir, f"{basename}_noaudio.mp4")
        strip_audio(mp4_path, noaudio_path)
        print(f"         -> {os.path.basename(noaudio_path)}")

        if clip_duration is None:
            print("  [3/3] Writing IMU CSV...")
            imu_path = os.path.join(output_dir, f"{basename}_imu_200hz.csv")
            n = write_ego4d_csv(imu_path, accl, gyro_aligned)
            print(f"         -> {os.path.basename(imu_path)} ({n:,} samples)")
        else:
            total_dur = get_video_duration(mp4_path)
            n_clips = int(total_dur // clip_duration) + (1 if total_dur % clip_duration > 0 else 0)
            print(f"  [3/3] Clipping into {n_clips} segment(s) of {clip_duration}s...")
            for clip_idx in range(n_clips):
                start = clip_idx * clip_duration
                end = min(start + clip_duration, total_dur)
                actual_dur = end - start
                clip_tag = f"clip{clip_idx + 1:02d}_{int(start)}s_{int(end)}s"
                clip_path = os.path.join(output_dir, f"{basename}_{clip_tag}.mp4")
                clip_video(noaudio_path, clip_path, start, actual_dur)
                sliced_accl, sliced_gyro = slice_imu(accl, gyro_aligned, start, end)
                imu_clip_path = os.path.join(output_dir, f"{basename}_{clip_tag}_imu_200hz.csv")
                video_t0 = accl[0][0]
                n_written = write_ego4d_csv(
                    imu_clip_path, sliced_accl, sliced_gyro,
                    t_offset=video_t0 + start,
                )
                print(f"         {clip_tag}: video + IMU ({n_written:,} samples)")
            os.remove(noaudio_path)
        print()

    print(f"{'=' * 60}\nAll done!\nOutput files are in: {output_dir}\n{'=' * 60}")


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
    parser.add_argument("--input", "-i",
                        default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "input"))
    parser.add_argument("--output", "-o",
                        default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "output"))
    parser.add_argument("--clip", "-c", type=float, default=None, metavar="SECONDS")
    parser.add_argument("--no-confirm", action="store_true")
    args = parser.parse_args()

    _check_dependencies()

    if not os.path.isdir(args.input):
        print(f"ERROR: Input directory not found: {args.input}")
        sys.exit(1)

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
