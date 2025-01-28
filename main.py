import cv2
import serial
import time

# Set up serial communication (replace 'COM7' with your Arduino's port)
arduino = serial.Serial('COM7', 9600)
time.sleep(2)  # Wait for the connection to initialize

# Load the Haar Cascade for face detection
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

# Open the webcam
cap = cv2.VideoCapture(0)

while True:
    # Capture a frame from the webcam
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break

    # Convert to grayscale for face detection
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect faces
    faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

    # Check if a face is detected
    if len(faces) > 0:
        print("Face detected! Sending 160° to Arduino.")
        arduino.write(b'160\n')  # Send 160° to Arduino
    else:
        print("No face detected. Sending 70° to Arduino.")
        arduino.write(b'70\n')  # Send 70° to Arduino

    # Display the video feed with rectangles around detected faces
    for (x, y, w, h) in faces:
        cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)

    cv2.imshow('Face Detection', frame)

    # Break the loop with 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cap.release()
cv2.destroyAllWindows()
arduino.close()
