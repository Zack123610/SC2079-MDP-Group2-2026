# Image Recognition Setup Guide

This guide covers setting up the Pi-to-PC video streaming for YOLO inference.

## Architecture

```
Raspberry Pi (Camera) ──── ZeroMQ TCP ────► PC (YOLO Inference)
```

---

## Raspberry Pi Setup

### 1. Install Dependencies

```bash
# Using pip (recommended for Pi)
pip install pyzmq opencv-python-headless numpy

# Or using uv
uv pip install -e ".[pi]"
```

> Note: `picamera2` is pre-installed on Raspberry Pi OS Bookworm.

### 2. Find Pi's IP Address

```bash
hostname -I
# Example output: 192.168.1.100
```

### 3. Run the Streamer

```bash
python pi_streamer.py --host 0.0.0.0 --port 5555

# With custom settings (1080p @ 30fps)
python pi_streamer.py --width 1920 --height 1080 --fps 30 --quality 85
```

---

## PC Setup

### 1. Install Dependencies

```bash
# Using pip
pip install pyzmq opencv-python ultralytics numpy

# Or using uv
uv pip install -e ".[pc]"
```

### 2. Run the Receiver

```bash
# Replace <PI_IP> with your Pi's IP address
python pc_receiver.py --host <PI_IP> --port 5555

# Example
python pc_receiver.py --host 192.168.1.100 --port 5555

# With custom YOLO model
python pc_receiver.py --host 192.168.1.100 --model yolo11n.pt --conf 0.5
```

---

## Command Line Options

### pi_streamer.py

| Option | Default | Description |
|--------|---------|-------------|
| `--host` | `0.0.0.0` | Bind address |
| `--port` | `5555` | Port number |
| `--width` | `1280` | Frame width |
| `--height` | `720` | Frame height |
| `--fps` | `60` | Target FPS |
| `--quality` | `80` | JPEG quality (1-100) |

### pc_receiver.py

| Option | Default | Description |
|--------|---------|-------------|
| `--host` | (required) | Pi's IP address |
| `--port` | `5555` | Port number |
| `--model` | `yolo11n.pt` | YOLO model path |
| `--conf` | `0.5` | Confidence threshold |
| `--show` | `True` | Show video window |

---

## Troubleshooting

### Connection Issues

1. Ensure Pi and PC are on the same network
2. Check Pi's firewall: `sudo ufw allow 5555/tcp`
3. Test connectivity: `ping <PI_IP>`

### Performance Tips

- Lower resolution for better FPS: `--width 640 --height 480`
- Lower JPEG quality for less bandwidth: `--quality 60`
- Use wired Ethernet for lowest latency

### Legacy Mode (Run YOLO on Pi)

If you need to run YOLO directly on the Pi (slower):

```bash
python rpi-yolo.py
```