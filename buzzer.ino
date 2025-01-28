int buzzerPin = 9; // Pin connected to the buzzer

void setup() {
  pinMode(buzzerPin, OUTPUT); // Set the buzzer pin as an output
}

void loop() {
  tone(buzzerPin, 1000); // Play a 1000 Hz tone
  delay(500);            // Wait for 0.5 seconds
  noTone(buzzerPin);     // Stop the tone
  delay(500);            // Wait for 0.5 seconds
}

