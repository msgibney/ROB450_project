/*
  Adjust PWM duty cycle with 'w' and 's' over Serial.
  - PWM frequency: 1000 Hz
  - Duty cycle: 0–255
  - Output pin: 9
*/
const int PWM_pinMap[] = { 1, 22, 23,  4,  5,  6,  7, 28, 36};
const int IO_pinMap[] =  {10,  2,  3, 34,  0, 26, 27,  8,  9};
const int numPins = 18;
const int numMags = 9;

int duty = 128;  // Start at 50% duty (128/255)

static inline void set_all_defaults(){
  for (int i = 0; i < numMags; i++)
  {
    analogWriteFrequency(PWM_pinMap[i], 1000);
    analogWrite(PWM_pinMap[i], duty);

    pinMode(IO_pinMap[i], OUTPUT);
    digitalWrite(IO_pinMap[i], LOW);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Type 'w' to increase, 's' to decrease duty cycle.");

  set_all_defaults();
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();

    if (cmd == 'w') {
      if (duty < 255) duty++;
    } else if (cmd == 's') {
      if (duty > 0) duty--;
    } else {
      return; // ignore other input
    }

    for (int i = 0; i < numMags; i++)
    {
      //analogWriteFrequency(PWM_pinMap[i], 1000);
      analogWrite(PWM_pinMap[i], duty);
    }
    Serial.print("Duty cycle: ");
    Serial.println(duty);
  }
}
