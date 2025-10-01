// Teensy 4.1 PWM example
// PWM pin: 9 (can be changed to another PWM-capable pin)

const int pwmPin = 9;       // PWM output pin
const int pwmFreq = 10000;  // 10 kHz

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for Serial monitor
  }

  // Set PWM frequency on chosen pin
  analogWriteFrequency(pwmPin, pwmFreq);

  // Set initial duty cycle to 0
  analogWrite(pwmPin, 0);

  Serial.println("Send a number from 1 to 255 to set duty cycle %");
}

void loop() {
  if (Serial.available() > 0) {
    int value = Serial.parseInt();  // Read integer from Serial

    if (value >= 1 && value <= 255) {
      // Write PWM duty cycle
      analogWrite(pwmPin, duty);

      Serial.print("Duty cycle set to ");
      Serial.print(value);
      Serial.println("%");
    }
    // flush the input so stray characters don’t linger
    while (Serial.available()) Serial.read();  
  }
}
