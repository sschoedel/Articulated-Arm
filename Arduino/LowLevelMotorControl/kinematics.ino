void kinematics(float x, float y, float z, float a, float b, float g)
{
  // Only calculates position for wrist using motors 1 2 3
  float z1 = z - h1;

  // Y axis
  float theta1Degrees = atan(y/x) * 180/M_PI;
  float x1 = sqrt(sq(x) + sq(y));

  // X axis
  float off2Degrees = atan(z1/x1) * 180/M_PI;
  float z2 = sqrt(sq(x1) + sq(z1)); // pass z2 to the next stage to figure out final motor angles

  // Z axis
  double theta3Degrees = acos((sq(len1) + sq(len2) - sq(z2))/(2*len1*len2)) * 180/M_PI;
  double off1Degrees = (180 - theta3Degrees)/2;
  double theta2Degrees = off1Degrees + off2Degrees;
  Serial.print("theta1: ");Serial.println(theta1Degrees);
  Serial.print("theta2: ");Serial.println(theta2Degrees);
  Serial.print("theta3: ");Serial.println(theta3Degrees);
  goalPositions[0] = theta1Degrees * stepsPerRev[0]/360;
  goalPositions[1] = theta2Degrees * stepsPerRev[1]/360;
  goalPositions[2] = theta3Degrees * stepsPerRev[2]/360;
  steppers.moveTo(goalPositions);
}
