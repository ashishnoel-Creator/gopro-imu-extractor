# GoPro IMU Extractor

Extract 6-DOF IMU data (accelerometer + gyroscope) from GoPro videos in Ego4D-standard CSV format.

## What It Does

- Extracts **accelerometer** (m/s²) and **gyroscope** (rad/s) data at **~200 Hz**
- Strips **audio** from all output videos
- Optionally **clips** videos into fixed-duration segments with matching IMU files
- Outputs **Ego4D-compatible CSVs** ready for egocentric AI training pipelines

## Quick Start

```bash
# 1. Clone and install
git clone <your-repo-url>
cd gopro-imu-extractor
pip install -r requirements.txt

# 2. Place GoPro MP4 files in the input folder
cp /path/to/your/GoPro/*.MP4 input/

# 3. Run
python extract.py
```

Output appears in the `output/` folder — one `_noaudio.mp4` and one `_imu_200hz.csv` per video.

### With Clipping

```bash
python extract.py --clip 180    # 3-minute segments
```

This produces numbered clips with matching IMU files:
```
output/
├── GX021077_clip01_0s_180s.mp4
├── GX021077_clip01_0s_180s_imu_200hz.csv
├── GX021077_clip02_180s_320s.mp4
├── GX021077_clip02_180s_320s_imu_200hz.csv
└── ...
```

## Output Format

```csv
timestamp,accl_x,accl_y,accl_z,gyro_x,gyro_y,gyro_z
0.000000,1.887290,0.772182,9.808153,-0.030884,-0.020234,0.120341
0.000158,1.858513,0.786571,9.757794,-0.026624,-0.031949,0.130990
```

Compatible with: **Ego4D**, **Epic-Kitchens**, **Assembly101**, **Project Aria**

## Requirements

- Python 3.10+
- ffmpeg (installed and in PATH)
- Dependencies: `pip install -r requirements.txt`

## Important

**The `output/` folder is wiped clean at the start of every run.** Move any files you want to keep before processing a new batch.

## Using with an AI Agent

If you're using this with Claude, ChatGPT, or another AI agent, point it to the `AGENT.md` file in this repo. It contains complete instructions for the agent to follow.

## CLI Options

```
python extract.py [options]

Options:
  --input, -i PATH       Input directory (default: ./input)
  --output, -o PATH      Output directory (default: ./output)
  --clip, -c SECONDS     Clip into segments of this duration
  --no-confirm           Skip output deletion confirmation
```
