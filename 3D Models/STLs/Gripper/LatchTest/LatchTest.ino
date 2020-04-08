#include <Servo.h>


// 150 max for open so fingers are both always constrained on both sides of the gear
// 0 for close but if fingers will be farther apart than 0, close incrementally so servo doesn't overheat or skip teeth
//      example: fingers open and grabbing large object so smallest fingers can close is larger than the servo's zero position.
//               Close incrementally by subtracting 10 every 100ms until commanding servo to 0 position. This stops servo from skipping teeth.

Servo latchServo;

int latchPWM = 0;

void setup() {
  Serial.begin(9600);
  latchServo.attach(9);
}

void loop() {
  if(Serial.available() > 0)
  {
    String message = Serial.readStringUntil('\n');
    Serial.println(message);
    if(message == "close") { closeRoutine(); }
    else if(message == "open") { latchServo.write(150); }
    else { latchServo.write(message.toInt()); Serial.println("written");}
  }
}

void closeRoutine()
{
  latchServo.write(0);
  delay(1000);
  latchServo.write(15);
  delay(1000);
  Serial.println("connected to tool");
}
