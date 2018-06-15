#include <Wire.h>
#include <math.h>

// I2C variables
const uint8_t SLAVE_ADDRESS = 0x32; // 50
uint8_t encoderBytes[2];

// IO 
const uint8_t en1 = 8;
const uint8_t en2 = 4;
const uint8_t en3 = 6;

const uint8_t in1 = 5;
const uint8_t in2 = 9;
const uint8_t in3 = 10;

const uint8_t encoderPinA = 7;
const uint8_t encoderPinB = 11;

// Commutation variables
uint16_t indexA;
uint16_t indexB;
uint16_t indexC;
const uint8_t phaseShift = 120;
const uint8_t commutationShift = 90;
const int pwmArray[] = {128, 132, 136, 140, 143, 147, 151, 155, 159, 162, 166, 170, 174, 178, 181, 185, 189, 192, 196, 200, 203, 207, 211, 214, 218, 221, 225, 228, 232, 235, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 248, 249, 250, 250, 251, 252, 252, 253, 253, 253, 254, 254, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 254, 254, 254, 253, 253, 253, 252, 252, 251, 250, 250, 249, 248, 248, 247, 246, 245, 244, 243, 242, 241, 240, 239, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 248, 249, 250, 250, 251, 252, 252, 253, 253, 253, 254, 254, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 254, 254, 254, 253, 253, 253, 252, 252, 251, 250, 250, 249, 248, 248, 247, 246, 245, 244, 243, 242, 241, 240, 239, 238, 235, 232, 228, 225, 221, 218, 214, 211, 207, 203, 200, 196, 192, 189, 185, 181, 178, 174, 170, 166, 162, 159, 155, 151, 147, 143, 140, 136, 132, 128, 124, 120, 116, 113, 109, 105, 101, 97, 94, 90, 86, 82, 78, 75, 71, 67, 64, 60, 56, 53, 49, 45, 42, 38, 35, 31, 28, 24, 21, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 8, 7, 6, 6, 5, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 5, 6, 6, 7, 8, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 8, 7, 6, 6, 5, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 5, 6, 6, 7, 8, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 21, 24, 28, 31, 35, 38, 42, 45, 49, 53, 56, 60, 64, 67, 71, 75, 78, 82, 86, 90, 94, 97, 101, 105, 109, 113, 116, 120, 124};

// BLDC speed and direction
float torqueControl = 0.00;
uint8_t bldcDirection = 0; // 1 = CW, 0 = CCW

// Encoder variables
volatile int16_t encoderCounter = 0;
const float elDegPerPulse = 2.4609375;
double elDegrees;


void setup() {
  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(receiveSpeedCommand);
  Wire.onRequest(sendEncoderValue);
  pinMode(en1, OUTPUT);
  pinMode(en2, OUTPUT);
  pinMode(en3, OUTPUT);

  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);

  pinMode(A2, INPUT);

  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);

  TCCR1B = TCCR1B & 0b11111000 | 0x01;
  TCCR3B = TCCR3B & 0b11111000 | 0x01;

  bldcInitializePosition();
  attachInterrupt(digitalPinToInterrupt(encoderPinA), newPulseA, RISING);
}

void loop() {
  setBldcTorque();
  bldcCommutation();
}

void newPulseA() {
  if (digitalRead(encoderPinB) == LOW)
  {
    encoderCounter--;
  }
  else
  {
    encoderCounter++;
  }
}


void bldcInitializePosition() {
  indexA = 0;
  indexB = phaseShift;
  indexC = indexB + phaseShift;
  digitalWrite(en1, HIGH);
  digitalWrite(en2, HIGH);
  digitalWrite(en3, HIGH);
  analogWrite(in1, pwmArray[indexA]);
  analogWrite(in2, pwmArray[indexB]);
  analogWrite(in3, pwmArray[indexC]);
  delay(2000);
  encoderCounter = 0;
}

void bldcCommutation() {
  if (encoderCounter > 1024) {
    encoderCounter = encoderCounter % 1024;
  }
  else if (encoderCounter < 0) {
    encoderCounter = 1024 + encoderCounter;
  }
  elDegrees = fmod(encoderCounter * elDegPerPulse, 360.0);
  if (bldcDirection == true) {
    indexA = elDegrees + commutationShift + 0.5;
  }
  else {
    indexA = elDegrees + commutationShift + 180.5;
  }
  if (indexA > 360) {
    indexA = indexA - 360;
  }
  else if (indexA < 0) {
    indexA = 360 - indexA;
  }
  indexB = indexA + phaseShift;
  if (indexB > 360) {
    indexB = indexB - 360;
  }
  else if (indexB < 0) {
    indexB = 360 - indexB;
  }
  indexC = indexB + phaseShift;
  if (indexC > 360) {
    indexC = indexC - 360;
  }
  else if (indexC < 0) {
    indexC = 360 - indexC;
  }
}

void setBldcTorque() {
  analogWrite(in1, torqueControl * pwmArray[indexA]);
  analogWrite(in2, torqueControl * pwmArray[indexB]);
  analogWrite(in3, torqueControl * pwmArray[indexC]);
}

void receiveSpeedCommand() {
  uint8_t tempBuffer = Wire.read();
  torqueControl = map(tempBuffer, 0, 255, -100, 100);
  if (torqueControl < 0.0) {
    bldcDirection = false;
    torqueControl *= -1.0;
  }
  else {
    bldcDirection = true;
  }
  torqueControl /= 100.0;
}

void sendEncoderValue() {
  uint16_t tempEncoderValue = encoderCounter;
  encoderBytes[0] = tempEncoderValue >> 8;
  tempEncoderValue &= 0xFF;
  encoderBytes[1] = tempEncoderValue;
  Wire.write(encoderBytes, 2);
}

