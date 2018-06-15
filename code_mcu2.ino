 #include <Q2HX711.h>
#include <Wire.h>

//Interrupt flag
volatile byte ISRflag = 0;

// I2C
uint8_t SLAVE_ADDRESS = 0x32;
uint16_t encoderPosition;

//HX711 variables
const uint8_t HX711_DATAPIN = 5;
const uint8_t HX711_CLKPIN = 4;
Q2HX711 strainGaugeADC(HX711_DATAPIN, HX711_CLKPIN);

//Input, output and filter parameters
double rawStrainInput;
double filteredInput;
double lastFilteredOutput;
double alpha = 0.10;
double rawSetpoint;

//PI controller variables
double Kp = 0.0;
double pTerm;
double Ki = 12.5;
double iTerm;
double errorSignal;
double strainSetpoint = 0.0;
double controllerOutput;
float integratorOutput = 127.5;
const float TIMESTEP = 0.01;
const float MAX_OUTPUT = 127.5;
const float MIN_OUTPUT = -127.5;
const float HIGHER_ERROR_OFFSET = 2.0;
const float LOWER_ERROR_OFFSET = -2.0;

void setup() {
  delay(2000);
  Wire.begin();
  findSetpoint();
  initializeTimerInterrupt();
}

void loop() {
  if (ISRflag == 1) {
    ISRflag = 0;
    readInput();
    filterInput();
    PIcontrol();
    sendOutput();
    requestEncoderInput();
  }
}
ISR(TIMER3_COMPA_vect) {
  ISRflag = 1;
}

void PIcontrol() {
  errorSignal = strainSetpoint - filteredInput; // Calculate the error
  pTerm = Kp * errorSignal;                   // Calculate the P-term
  iTerm += Ki * errorSignal * TIMESTEP;       // Accumulate the I-term
  if (iTerm > MAX_OUTPUT) {                   // Check for I-term overflow
    iTerm = MAX_OUTPUT;
  }
  else if (iTerm < MIN_OUTPUT) {
    iTerm = MIN_OUTPUT;
  }
  controllerOutput = pTerm + iTerm;           // Calculate total PI controller output
  if (controllerOutput > MAX_OUTPUT) {        // Check for output overflow
    controllerOutput = MAX_OUTPUT;
  }
  else if (controllerOutput < MIN_OUTPUT) {
    controllerOutput = MIN_OUTPUT;
  }
}

void readInput() {
  rawStrainInput = strainGaugeADC.read() / 100.0;
  rawStrainInput -= rawSetpoint;
  if (rawStrainInput < HIGHER_ERROR_OFFSET && rawStrainInput > LOWER_ERROR_OFFSET) {
    rawStrainInput = strainSetpoint;
  }
}

void sendOutput() {
  uint8_t speedCommand = controllerOutput + 127.5 + 0.5; //controllerOutput + 127.5 + 0.5;
  Wire.beginTransmission(SLAVE_ADDRESS);
  Wire.write(speedCommand);
  Wire.endTransmission();
}

void requestEncoderInput() {
  uint8_t n = Wire.requestFrom(SLAVE_ADDRESS, 2);
  uint8_t encoderBuffer[2];
  for( int i = 0; i<n; i++)
  {
   encoderBuffer[i] = Wire.read();
  }
  encoderPosition &= 0x00;
  encoderPosition |= encoderBuffer[1];
  encoderPosition |= (encoderBuffer[0] << 8);
}

void findSetpoint() {
  uint8_t numberOfReadings;
  double total = 0;
  for (numberOfReadings = 0; numberOfReadings < 200; numberOfReadings++) {
    total += (strainGaugeADC.read() / 100.0);
    delay(10);
  }
  rawSetpoint = total / 200.0;
}

void initializeTimerInterrupt() {
  TCCR3A = 0; // Clear register TCCR3A
  TCCR3B = 0; // Clear register TCCR3B
  TCCR3B |= (1 << WGM32); // set the "clear timer on compare match" (CTC) mode
  TCCR3B |= (1 << CS32); // Set prescaler to 256
  TIMSK3 |= (1 << OCIE3A); // Enable interrupt on match
  TCNT3 = 0; // Reset
  OCR3A = 625; // Count value
}


void filterInput() {
    filteredInput = alpha * rawStrainInput + (1 - alpha) * lastFilteredOutput;
    lastFilteredOutput = filteredInput;
}

