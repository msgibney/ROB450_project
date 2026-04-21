#include <Arduino.h>

// Magnet pins to teensy pins
const int PWM_pinMap[] = { 1, 22, 23,  4,  5,  6,  7, 28, 36};
const int IO_pinMap[] =  {10,  2,  3, 31,  0, 26, 27,  8,  9};
const int numPins = 18;
const int numMags = 9;

//magnet position (on end effector) to actual designated magnet number
const int config_1_map[] = {6};
const int config_4_map[] = {3, 4, 5, 6};
const int config_5_map[] = {1, 3, 5, 7, 9}; //This one is not used
const int config_9_map[] = {1, 2, 3, 4, 5, 6, 7, 8, 9};

//Defautl params
const int pwmFreq = 1000;
const int initDuty = 0; //(int) (255 * 0.00);

//polarity = LOW  = 0 --> Magnet Attracts Microrobots
//polarity = HIGH = 1 --> Magnet Repels Microrobots
const int initIO = LOW;

const int duty_tuning[] = {
  0,0,8,55,86,103,108,124,134,141,147,151,153,155,157,159,161,164,165,166,170,171,173,175,177,179,181,184,186,188,191,193,195,197,200,202,204,206,209,212,214,217,219,222,224,226,230,233,236,238,241,253,
  0,0,5,55,80,98,103,120,130,140,145,150,152,153,156,158,160,162,164,166,168,170,172,174,176,178,180,182,184,186,189,191,194,196,199,201,203,205,207,211,212,215,217,220,223,225,228,231,234,237,240,252,
  0,0,10,55,81,101,106,122,132,140,146,151,152,153,156,159,161,162,165,166,169,170,172,175,176,178,180,182,184,187,190,191,194,196,199,201,203,205,207,210,212,215,217,220,222,224,227,230,233,236,239,252,
  0,0,10,59,81,100,103,120,132,140,146,150,152,153,156,158,160,162,164,166,168,170,172,174,176,177,179,181,183,185,189,191,193,196,197,200,202,204,206,209,211,214,216,218,220,223,226,230,233,237,239,251,
  0,0,41,72,93,107,117,128,137,144,149,152,154,155,158,160,162,164,166,168,170,172,174,176,178,180,182,184,186,188,191,193,196,198,200,202,204,206,209,212,214,217,219,222,224,227,230,233,236,239,242,254,
  0,0,0,53,77,97,103,120,130,137,145,150,152,154,156,158,160,162,164,166,168,170,172,174,176,178,180,182,183,185,188,190,193,195,197,199,201,203,205,209,211,214,216,219,221,225,227,230,233,237,239,251,
  0,0,5,54,77,92,103,120,132,138,146,150,153,154,156,158,160,162,164,166,168,170,172,174,176,178,180,182,184,187,190,192,194,196,199,201,203,205,208,211,213,216,218,221,223,226,229,232,235,238,241,252,
  0,0,1,51,80,98,103,119,131,139,146,150,153,154,157,158,160,162,164,166,168,170,171,173,175,177,179,181,183,185,188,190,193,196,198,200,202,204,207,210,212,214,216,219,221,225,227,230,233,237,240,252,
  0,0,17,55,80,100,107,122,133,141,147,151,153,154,157,159,161,163,165,167,169,171,173,175,177,179,181,184,186,189,192,194,196,199,201,203,205,207,210,213,215,218,220,223,226,229,232,235,238,240,243,255
};

void setup()
{
  Serial.begin(115200);
  
  // Configure all pins as output
  set_all_defaults();

  Serial.println("Teensy ready to receive serial control messages.");
}

void loop() {
  if (Serial.available()) {
    uint8_t config; uint8_t virtual_mag; uint8_t pol; uint8_t duty;
    bool status = read_config_msg(config, virtual_mag, pol, duty);

    printf("config=%d, virtual_mag=%d, pol=%d, duty=%d\n", config, virtual_mag, pol, duty);

    if (!status) {
      Serial.println("Invalid message format, skipping message.");
      return;
    }

    if(config == 0){
      set_all_defaults();
      return;
    }

    //polarity = LOW  = 0 --> Magnet Attracts Microrobots
    //polarity = HIGH = 1 --> Magnet Repels Microrobots
    bool polarity = (bool) pol;

    //PWM is reversed when polarity is swapped to HIGH
    duty = (polarity) ? (255 - duty) : duty;

    status = tune_duty(duty, config, virtual_mag);
    if (!status) {
      Serial.println("Invalid duty cycle (must be multiple of 5 between 0-255), skipping message.");
      return;
    }
    
    uint8_t real_mag;
    status = virtual_to_real_mag(config, virtual_mag, real_mag);
    if (!status) {
      Serial.println("Invalid configuration and/or magnet specified, skipping message.");
      return;
    }

    int PWM_pin = PWM_pinMap[real_mag];
    int IO_pin = IO_pinMap[real_mag];

    digitalWrite(IO_pin, polarity);
    analogWrite(PWM_pin, duty);
    printf("IO_pin=%d, PWM_pin=%d, polarity=%d, duty=%d\n", IO_pin, PWM_pin, polarity, duty);
  }
}

static inline bool read_config_msg(uint8_t& config, uint8_t& mag, uint8_t& pol, uint8_t& duty)
{
  bool status = false;

  config = Serial.read();
  mag = Serial.read();
  pol = Serial.read();
  duty = Serial.read();
  uint8_t stop = Serial.read();

  if (stop == 0xFF) {status = true;}
  return status;
}

static inline bool tune_duty(uint8_t& duty_actual, uint8_t config, uint8_t virtual_mag)
{
  bool status = true;
  //round to nearest multiple of 5
  uint8_t duty = ((duty_actual + 2) / 5) * 5;
  
  if(0 > duty || duty > 255 || (duty % 5) != 0)
  {
    status = false;
    return status;
  }

  duty /= 5;
 
  //virtual_mag=5 is the default if there are no specific tuning parameters
  //i'm writing it in this silly way just in case we want to expand tuning to other configs.
  if      (config == 9) {}
  else if (config == 4) {virtual_mag = 5;}
  else if (config == 5) {virtual_mag = 5;}
  else if (config == 1) {virtual_mag = 5;}
  else                  {virtual_mag = 5;};

  // 255/5 = 51
  duty_actual = duty_tuning[(virtual_mag-1)*51 + duty];
  
  return status;
}

static inline bool virtual_to_real_mag(const uint8_t config, uint8_t& virtual_mag, uint8_t& real_mag){
  bool status = false;
  
  virtual_mag -= 1;
  if(config == 1){
    if(0 != virtual_mag){status = false; return status;}
    real_mag = config_1_map[virtual_mag] - 1;
    status = true;
  }
  else if(config == 4){
    if(0 > virtual_mag || virtual_mag > 3){status = false; return status;}
    real_mag = config_4_map[virtual_mag] - 1;
    status = true;
  }
  else if(config == 5){
    if(0 > virtual_mag || virtual_mag > 4){status = false; return status;}
    real_mag = config_5_map[virtual_mag] - 1;
    status = true;
  }
  else if(config == 9){
    if(0 > virtual_mag || virtual_mag > 8){status = false; return status;}
    real_mag = config_9_map[virtual_mag] - 1;
    status = true;
  }
  else{
    status = false; return status;
  }

  return status;
}

static inline void set_all_defaults(){
  for (int i = 0; i < numMags; i++)
  {
    analogWriteFrequency(PWM_pinMap[i], pwmFreq);
    analogWrite(PWM_pinMap[i], ((initIO) ? (255 - initDuty) : initDuty));

    pinMode(IO_pinMap[i], OUTPUT);
    digitalWrite(IO_pinMap[i], initIO);
  }
}