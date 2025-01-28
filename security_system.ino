#define BUZZER_PIN 7  // Connect the buzzer to digital pin 8

void setup() {
  pinMode(BUZZER_PIN, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  if (Serial.available()) {
    char data = Serial.read(); // Read the data sent from Python
    if (data == '1') {
      digitalWrite(BUZZER_PIN, HIGH);  // Turn on the buzzer
    } else {
      digitalWrite(BUZZER_PIN, LOW);   // Turn off the buzzer
    }
  }
}
