#!/usr/bin/env python3
"""
Simple fast web-based camera viewer - NO object detection
Starts immediately and shows camera feed
Access from any browser at http://<raspberry-pi-ip>:5000
"""

from flask import Flask, Response, render_template_string
import cv2
from picamera2 import Picamera2
import time

app = Flask(__name__)

# Initialize camera
print("Initializing camera...")
picam2 = Picamera2()
camera_config = picam2.create_preview_configuration(
    main={"size": (640, 480), "format": "RGB888"}
)
picam2.configure(camera_config)
picam2.start()
time.sleep(2)  # Let camera warm up
print("Camera ready!")

def generate_frames():
    """Generator function to continuously capture and encode frames"""
    while True:
        try:
            # Capture frame from camera
            frame = picam2.capture_array()
            
            # Convert RGB to BGR for OpenCV
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            
            # Add timestamp overlay
            timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
            cv2.putText(frame, timestamp, (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # Encode frame as JPEG
            ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
            if not ret:
                continue
                
            frame_bytes = buffer.tobytes()
            
            # Yield frame in multipart format
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
        except Exception as e:
            print(f"Error: {e}")
            break

@app.route('/')
def index():
    """Video streaming home page"""
    html = """
    <!DOCTYPE html>
    <html>
    <head>
        <title>Camera Feed - Simple</title>
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
        <style>
            body {
                margin: 0;
                padding: 20px;
                background-color: #1e1e1e;
                font-family: Arial, sans-serif;
                color: #ffffff;
                text-align: center;
            }
            h1 {
                margin-bottom: 20px;
            }
            .container {
                max-width: 900px;
                margin: 0 auto;
            }
            .video-wrapper {
                position: relative;
                display: inline-block;
                max-width: 100%;
            }
            img {
                width: 100%;
                max-width: 640px;
                border: 3px solid #4CAF50;
                border-radius: 8px;
                box-shadow: 0 4px 12px rgba(76, 175, 80, 0.4);
                display: block;
            }
            .status {
                margin-top: 20px;
                padding: 15px;
                background-color: #2d2d2d;
                border-radius: 8px;
                color: #4CAF50;
            }
            .live-indicator {
                display: inline-block;
                width: 10px;
                height: 10px;
                background-color: #4CAF50;
                border-radius: 50%;
                margin-right: 8px;
                animation: pulse 2s infinite;
            }
            @keyframes pulse {
                0%, 100% { opacity: 1; }
                50% { opacity: 0.3; }
            }
        </style>
    </head>
    <body>
        <div class="container">
            <h1>📹 Live Camera Feed</h1>
            <div class="video-wrapper">
                <img src="{{ url_for('video_feed') }}" alt="Camera Feed" id="videoFeed">
            </div>
            <div class="status">
                <span class="live-indicator"></span>
                <strong>LIVE</strong> - Fast Camera View (No Object Detection)
            </div>
        </div>
        
        <script>
            // Reload image if it fails to load
            document.getElementById('videoFeed').onerror = function() {
                setTimeout(function() {
                    document.getElementById('videoFeed').src = '{{ url_for("video_feed") }}?' + new Date().getTime();
                }, 1000);
            };
        </script>
    </body>
    </html>
    """
    return render_template_string(html)

@app.route('/video_feed')
def video_feed():
    """Video streaming route"""
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    print("\n" + "="*70)
    print("  📹 Simple Camera Viewer (Fast - No Object Detection)")
    print("="*70)
    print("\n🌐 Access at:")
    print("  - http://localhost:5000")
    print("  - http://<raspberry-pi-ip>:5000")
    print("\n💡 Find IP: hostname -I")
    print("⌨️  Stop: Ctrl+C")
    print("="*70 + "\n")
    
    try:
        app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
    except KeyboardInterrupt:
        print("\n\n🛑 Stopping...")
    finally:
        picam2.stop()
        print("📹 Camera stopped!\n")
