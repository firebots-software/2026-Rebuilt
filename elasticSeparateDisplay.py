import cv2
import mss
import numpy as np
# Requires mss and opencv-python packages, install with pip if not already installed (numpy is a dependency of opencv-python):
# pip install mss opencv-python
# Run through terminal:
# python direction/to/directory/elasticSeparateDisplay.py

# Check to make sure that the capture dimensions are correct, edit them below in CAPTURE_DIMENSIONS

# Creates a window that displays a portion of the screen to another display, allows capture window to be resized

CAPTURE_DIMENSIONS = {"top": 0, "left": 0, "width": 1920, "height": 1080}

with mss.mss() as sct:
    while True:
        # Capture screen pixels
        sct_img = sct.grab(CAPTURE_DIMENSIONS)
        
        # Convert to numpy array (RGB)
        img = np.array(sct_img)
        
        # Convert RGB to BGR for OpenCV
        frame = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        
        # Create a window that can be resized
        cv2.namedWindow('Recording', cv2.WINDOW_NORMAL)
        
        # Display the recording (optional)
        cv2.imshow("Recording", frame)
        
        # Stop on 'q' key
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

cv2.destroyAllWindows()
