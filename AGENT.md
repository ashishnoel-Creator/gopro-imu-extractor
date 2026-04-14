# GoPro IMU Extractor — Agent Instructions

> **This file is for AI agents (Claude, GPT, Copilot, etc.).**
> Read this entire file before doing anything. It tells you exactly how this tool works, what to ask the user, and how to run it.

---

## What This Tool Does

This tool extracts **6-DOF IMU data** (3-axis accelerometer + 3-axis gyroscope) from GoPro MP4 video files and outputs them as **Ego4D-standard CSV files** at ~200 Hz. It also **strips audio** from all output videos.

**Output CSV format (Ego4D 6-DOF):**
```
timestamp, accl_x, accl_y, accl_z, gyro_x, gyro_y, gyro_z
```
- Accelerometer: m/s²
- Gyroscope: rad/s
- Sample rate: ~200 Hz
- Timestamps: seconds, starting from 0.0

This format is directly compatible with **Ego4D, Epic-Kitchens, Assembly101, and Project Aria** egocentric vision pipelines.

**Works on all GoPro generations:** Hero 5–10 (matched 200 Hz IMU) and Hero 11/12/13 (200 Hz accel + 1600 Hz gyro). For newer GoPros, the gyroscope is automatically interpolated onto the accelerometer timeline so the output is always a clean, paired ~200 Hz CSV.

---

## Project Structure

```
gopro-imu-extractor/
├── AGENT.md           ← You are here. Read this first.
├── README.md          ← Human-readable documentation
├── extract.py         ← Main extraction script
├── requirements.txt   ← Python dependencies
├── input/             ← PUT GOPRO MP4 FILES HERE (user must populate)
└── output/            ← PROCESSED FILES APPEAR HERE (auto-created)
```

---

## Setup (Run Once)

No Python packages are required — `extract.py` uses only the standard library.

You only need **ffmpeg** (and `ffprobe`) installed and available in PATH:
- macOS: `brew install ffmpeg`
- Ubuntu/Debian: `sudo apt install ffmpeg`
- Windows: Download from https://ffmpeg.org/download.html

---

## How to Use This Tool

### Step 1: Ask the User These Questions

Before running anything, **always ask the user**:

1. **"Have you placed your GoPro MP4 files in the `input/` folder?"**
   - If not, tell them to do so first.

2. **"Do you want the videos clipped into shorter segments? If so, what duration (in seconds)?"**
   - Common choices: 180s (3 min), 300s (5 min), 600s (10 min)
   - If they say no, run without the `--clip` flag.

3. **"The `output/` folder will be completely deleted before processing a new batch. Is that okay?"**
   - This is important. Every new run wipes the output folder clean.
   - If they have files they want to keep, tell them to move those files first.

### Step 2: Run the Script

**No clipping:**
```bash
cd /path/to/gopro-imu-extractor
python extract.py
```

**With clipping (e.g., 3-minute segments):**
```bash
python extract.py --clip 180
```

**Skip confirmation prompt (for automation):**
```bash
python extract.py --no-confirm
```

**Custom input/output directories:**
```bash
python extract.py --input /path/to/videos --output /path/to/results
```

### Step 3: Explain the Output

After processing, the `output/` folder will contain:

**Without clipping:**
```
output/
├── GX021077_noaudio.mp4           ← Video with audio removed
├── GX021077_imu_200hz.csv         ← Full IMU data for this video
├── GX031077_noaudio.mp4
└── GX031077_imu_200hz.csv
```

**With clipping (e.g., --clip 180):**
```
output/
├── GX021077_clip01_0s_180s.mp4           ← First 3-min clip
├── GX021077_clip01_0s_180s_imu_200hz.csv ← Matching IMU data
├── GX021077_clip02_180s_320s.mp4         ← Remaining clip
├── GX021077_clip02_180s_320s_imu_200hz.csv
├── GX031077_clip01_0s_180s.mp4
├── GX031077_clip01_0s_180s_imu_200hz.csv
└── ...
```

Each video file has a matching `_imu_200hz.csv` with the **same name prefix**. The IMU timestamps are rebased to start at 0.0 and align with the video timeline.

---

## Important Notes

- **Output folder is wiped every run.** Always warn the user about this. Previous results will be deleted.
- **Audio is always stripped.** Output videos never contain audio tracks.
- **No re-encoding.** Video streams are copied directly (no quality loss). Only the audio track is removed.
- **Only processes .MP4 files.** Other formats are ignored.
- **GoPro-specific.** This tool reads GoPro's GPMF telemetry format. It will not work with non-GoPro cameras.
- **Large files.** GoPro 4K files are typically 3-4 GB each. Processing is I/O bound, not CPU bound (stream copy).

---

## Troubleshooting

| Problem | Solution |
|---------|----------|
| "No MP4 files found" | Place GoPro .MP4 files in the `input/` folder |
| "ffmpeg not found" | Install ffmpeg (see Setup section above) |
| "No GPMF (gpmd) data stream found" | Video isn't from a GoPro, or telemetry was disabled |
| "No IMU data found" | File may not be from a GoPro, or telemetry was disabled during recording |
| Script crashes on large files | Ensure enough disk space (output ≈ input size) |

---

## CLI Reference

```
usage: extract.py [-h] [--input INPUT] [--output OUTPUT] [--clip SECONDS] [--no-confirm]

Extract 6-DOF IMU data from GoPro videos (Ego4D format).

options:
  -h, --help            Show help message
  --input, -i INPUT     Input directory with GoPro MP4s (default: ./input)
  --output, -o OUTPUT   Output directory (default: ./output)
  --clip, -c SECONDS    Clip videos into segments of this duration
  --no-confirm          Skip deletion confirmation prompt
```
