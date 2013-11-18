/*
  ServoSequence
  Reads an analog input, and plays different servo sequences depending on the analog value
 
  This example code is in the public domain.
 */
 
#include <VarSpeedServo.h> 

VarSpeedServo myservo1;

const int servoPin1 = 9;  // the digital pin used for the servo

servoSequencePoint_t slowHalf[] = {{0,20},{45,70},{90,20}};
servoSequencePoint_t twitchy[] = {{180,255},{20,40},{90,127},{45,60}};

const int analogPin = A0;

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  myservo1.attach(servoPin1);
  myservo1.sequenceInit(0,slowHalf,3); // initialize sequence 0 to slowHalf sequence, 3 positions
  myservo1.sequenceInit(1,twitchy,4); // initialize sequence 1 to twitchy sequence, 4 positions
}

// the loop routine runs over and over again forever:
void loop() {
  // read the input on analog pin 0:
  int sensorValue = analogRead(analogPin);
  if (sensorValue > 200) {
    myservo1.sequencePlay(0, true); // play sequence 0, don't loop (start at default position 0)
  } else {
    myservo1.sequencePlay(1, true, 2); // play sequence 1, loop, start at third position
  }
  delay(2);        // delay in between reads for analog in stability
}

