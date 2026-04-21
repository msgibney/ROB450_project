/*
  Adjust PWM duty cycle with 'w' and 's' over Serial.
  - PWM frequency: 1000 Hz
  - Duty cycle: 0–255
  - Output pin: 9
*/

const int magNum = 9;

const int PWM_pinMap[] = { 1, 22, 23,  4,  5,  6,  7, 28, 36};
const int IO_pinMap[] =  {10,  2,  3, 31,  0, 26, 27,  8,  9};
const int numPins = 18;
const int numMags = 9;

const int initDuty = 0; // Start at 50% duty = (128/255)
int duty = initDuty;

static inline void set_all_defaults(){
  for (int i = 0; i < numMags; i++)
  {
    duty = initDuty;
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
    } else if (cmd == '0') {
      duty = 0;
    } else if (cmd == '1') {
      duty = (duty > 155) ? 255 : duty + 100;
    } else if (cmd == '2') {
      duty = (duty > 245) ? 255 : duty + 10;
    } else if (cmd == '3') {
      duty = (duty > 250) ? 255 : duty + 5;
    } else {
      return; // ignore other input
    }

    /*
    for (int i = 0; i < numMags; i++)
    {
      //analogWriteFrequency(PWM_pinMap[i], 1000); this line is dead code
      analogWrite(PWM_pinMap[i], duty);
    }
    */

    analogWrite(PWM_pinMap[magNum-1], duty);


    Serial.print("Duty cycle: ");
    Serial.println(duty);
  }
}
