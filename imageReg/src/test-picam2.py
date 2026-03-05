from picamera2 import Picamera2
import time

# 1. Initialize the camera
picam2 = Picamera2()

# 2. Configure for still capture (uses default sensor settings)
config = picam2.create_still_configuration()
picam2.configure(config)

# 3. Start the preview (if connected to a screen)
picam2.start()

# 4. Give camera time to adjust (light/auto-focus)
time.sleep(2)

# 5. Capture the image
print("Taking picture...")
picam2.capture_file("test.jpg")
print("Image saved as test.jpg")

# 6. Clean up
picam2.stop()
