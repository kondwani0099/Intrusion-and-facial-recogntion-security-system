
# **Intrusion and Facial Recognition Security System**

This project combines intrusion detection and facial recognition to create a robust security system. It uses a servo motor, a buzzer, and Python with OpenCV to detect faces and signal the Arduino for hardware responses. The system ensures heightened security by triggering alerts and actuators based on face detection.

---

## **Features**
1. **Face Detection**:
   - Detects human faces using OpenCV and a Haar Cascade classifier.
   - Distinguishes between a valid face presence and no face in the frame.
   
2. **Servo Motor Control**:
   - Positions the servo to **160°** when a face is detected.
   - Returns to **70°** when no face is detected.

3. **Buzzer Alert**:
   - Activates the buzzer for additional alerts when a face is detected.

4. **Serial Communication**:
   - Python communicates with Arduino via a serial connection to send servo angles based on detection.

5. **Video Feed**:
   - Displays real-time video with face detection annotations.

---

## **Hardware Requirements**
- Arduino Uno or compatible microcontroller.
- Servo Motor.
- Buzzer.
- USB Cable for Arduino.
- Webcam or external camera.
- Connecting wires and a breadboard.

---

## **Software Requirements**
- Python 3.x.
- Arduino IDE.
- Python Libraries:
  - `opencv-python`
  - `pyserial`

---

## **Setup Instructions**

### **1. Hardware Setup**
1. **Servo Motor**:
   - Connect the signal pin to Arduino **pin 9**.
   - Power the servo with **5V** and **GND** from the Arduino.

2. **Buzzer**:
   - Connect the positive lead to any Arduino output pin (e.g., **pin 7**) and the negative lead to **GND**.

3. **Camera**:
   - Connect a webcam to your computer for video input.

4. **Arduino to PC**:
   - Connect the Arduino to your computer via USB.

---

### **2. Arduino Code**
1. Open the Arduino IDE.
2. Copy and paste the following code into the IDE:
   ```cpp
   #include <Servo.h>

   Servo myServo;

   void setup() {
     myServo.attach(9);
     pinMode(7, OUTPUT); // Buzzer pin
     Serial.begin(9600);
   }

   void loop() {
     if (Serial.available() > 0) {
       String input = Serial.readStringUntil('\n');
       int angle = input.toInt();

       if (angle >= 0 && angle <= 180) {
         myServo.write(angle);
         if (angle == 160) {
           digitalWrite(7, HIGH); // Turn buzzer on
         } else {
           digitalWrite(7, LOW); // Turn buzzer off
         }
       }
     }
   }
   ```
3. Upload the code to your Arduino board.

---

### **3. Python Code**
1. Install the required Python libraries:
   ```bash
   pip install opencv-python pyserial
   ```
2. Save the following Python code as `app.py`:
   ```python
   import cv2
   import serial
   import time

   arduino = serial.Serial('COM7', 9600)  # Replace 'COM7' with your Arduino's port
   time.sleep(2)

   face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

   cap = cv2.VideoCapture(0)

   while True:
       ret, frame = cap.read()
       if not ret:
           break

       gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
       faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

       if len(faces) > 0:
           print("Face detected! Sending 160° to Arduino.")
           arduino.write(b'160\n')
       else:
           print("No face detected. Sending 70° to Arduino.")
           arduino.write(b'70\n')

       for (x, y, w, h) in faces:
           cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)

       cv2.imshow('Face Detection', frame)

       if cv2.waitKey(1) & 0xFF == ord('q'):
           break

   cap.release()
   cv2.destroyAllWindows()
   arduino.close()
   ```

---

### **4. Running the System**
1. Connect the hardware components.
2. Run the Python script:
   ```bash
   python app.py
   ```
3. Observe the video feed and hardware responses:
   - Servo moves to **160°** when a face is detected.
   - Servo moves back to **70°** when no face is detected.
   - Buzzer turns on/off based on face presence.

---

## **Applications**
- Home security systems.
- Office surveillance.
- Intrusion detection in restricted areas.

---

## **Future Enhancements**
- Add facial recognition to identify specific individuals.
- Integrate cloud notifications (e.g., email or SMS alerts).
- Use a database to log intrusions and face detections.
- Implement motion detection alongside facial recognition.

---

## **Credits**
- **Developer**: Kondwani Nyirenda 
- **Technologies Used**: Python, OpenCV, Arduino, Serial Communication

Feel free to reach out for any queries or contributions!

