# Robotic Arm Vision Controller

Cloud-based object detection system for robotic arm control with real-time web interface.

## Features

- ☁️ Cloud object detection (choose from 3 providers)
- 🎥 Live camera feed accessible via web browser  
- 🤖 Automatic inverse kinematics & arm positioning
- 🎯 Configurable target objects
- 🔒 Secure configuration with environment variables

## Quick Start

### 1. Install Dependencies

```bash
pip install -r requirements.txt
```

### 2. Configure API Credentials

```bash
# Copy the example environment file
cp .env.example .env

# Edit .env with your favorite editor
nano .env
```

Add your API credentials to `.env` file:

```bash
# Choose your provider
DETECTION_PROVIDER=azure  # or roboflow, google

# Add your API key (depending on provider)
AZURE_KEY=your_azure_key_here
AZURE_ENDPOINT=https://YOUR_RESOURCE.cognitiveservices.azure.com/
```

### 3. Run the Controller

```bash
python arm_vision_controller.py
```

### 4. Access Web Interface

Open your browser to: `http://<raspberry-pi-ip>:5000`

Find your IP with: `hostname -I`

## Detection Providers

| Provider | Objects | Free Tier | Setup |
|----------|---------|-----------|-------|
| **Hugging Face** 🥇 | 80 (COCO) | **UNLIMITED** | No key needed! |
| **Azure** | 10,000+ | 5,000/month | [Get Free Key](https://azure.microsoft.com/free/) |
| **Google Vision** | 10,000+ | 1,000/month | [Setup Guide](https://cloud.google.com/vision/docs/setup) |
| **Roboflow** | 80 (COCO) | 1,000/month | [Get Key](https://roboflow.com) |

### 🎉 Hugging Face - Recommended (FREE & UNLIMITED!)

**Best choice for getting started quickly!**

- ✅ **No API key required** - Works immediately with public models
- ✅ **Unlimited free requests** - No monthly limits!
- ✅ **Many pre-trained models** available
- ✅ **Popular models:**
  - `facebook/detr-resnet-50` - Fast, 80 COCO objects
  - `facebook/detr-resnet-101` - More accurate
  - `XintongHan/rt-detr-coco-1x` - Optimized for real-time

**Setup:** Just set `DETECTION_PROVIDER=huggingface` in `.env` - that's it!

## Configuration

All settings are in `.env` file:

- `DETECTION_PROVIDER` - Which API to use (azure/google/roboflow)
- `TARGET_OBJECTS` - Comma-separated list of objects to react to
- `WORKING_SERVOS` - Comma-separated servo channel numbers
- `CONFIDENCE_THRESHOLD` - Minimum detection confidence (0.0-1.0)
- `DETECT_EVERY_N_FRAMES` - Detection frequency to save API calls

## Files

- `arm_vision_controller.py` - Main controller (replaces therealthing.py)
- `camera_simple.py` - Simple camera viewer without detection
- `camera_cloud_detect.py` - Cloud detection camera viewer
- `.env` - Your configuration (never commit this!)
- `.env.example` - Configuration template

## Security

- Never commit `.env` file to git
- API keys are loaded from environment variables
- `.env` is automatically ignored by git

## Troubleshooting

**No camera feed?**
- Check camera is enabled: `sudo raspi-config`
- Test with: `python camera_simple.py`

**No detections?**
- Verify API key is in `.env`
- Check provider is set correctly
- Look for error messages in terminal

**Servos not moving?**
- Verify `WORKING_SERVOS` list in `.env`
- Check servo connections to PCA9685 board
- Ensure sufficient power supply

## License

MIT
