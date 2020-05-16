void kinematics(float x, float y, float z, float a, float b, float g) // x,y,z in mm a,b,g in degrees
{
  float d = len3 * cos(b*M_PI/180); // hand distance projected onto global plane

  // work back from given point to find spherical joint location
  float zSph = z + len3 * sin(b*M_PI/180);
  float xSph = x - d * cos(a*M_PI/180);
  float ySph = y - d * sin(a*M_PI/180);
  
  float z1 = zSph - h1;

  // Y axis
  float theta1Degrees = atan(ySph/xSph) * 180/M_PI;
  float x1 = sqrt(sq(xSph) + sq(ySph));

  // X axis
  float off2Degrees = atan(z1/x1) * 180/M_PI;
  float z2 = sqrt(sq(x1) + sq(z1)); // pass z2 to the next stage to figure out final motor angles

  // Z axis
  double theta3Degrees = acos((sq(len1) + sq(len2) - sq(z2))/(2*len1*len2)) * 180/M_PI;
  double off1Degrees = (180 - theta3Degrees)/2;
  double theta2Degrees = off1Degrees + off2Degrees;
  
  // find elbow and shoulder location
  float zProjected = len1 * cos(b*M_PI/180);
  
  float zElbow = h1 + len1 * sin(theta2Degrees*M_PI/180);
  float xElbow = len1 * cos(theta1Degrees*M_PI/180);
  float yElbow = len1 * sin(theta1Degrees*M_PI/180);

  float zShoulder = h1;
  float xShoulder = 0;  // TODO: Change x and y of shoulder to reflect offset in shoulder motor mounting location
  float yShoulder = 0;

  // calculate unit direction vectors v1, v2, v3 for bicep, forearm, and hand
  float v1[3] = {(xElbow - xShoulder)/len1, (yElbow - yShoulder)/len1, (zElbow - zShoulder)/len1};
  float v2[3] = {(xElbow - xSph)/len2, (yElbow - ySph)/len2, (zElbow - zSph)/len2};
  float v3[3] = {(xElbow - x)/len3, (yElbow - y)/len3, (zElbow - z)/len3};

  // calculate vectors perpendicular to v1 v2 and v2 v3
  float p[3] = {v1[1]*v2[2] - v1[2]*v2[1], v1[0]*v2[2] - v1[2]*v2[0], v1[0]*v2[1] - v1[1]*v2[0]}; // perpendicular to the plane including bicep and forearm
  float q[3] = {v3[1]*v2[2] - v3[2]*v2[1], v3[0]*v2[2] - v3[2]*v2[0], v3[0]*v2[1] - v3[1]*v2[0]}; // perpendicular to the plane including forearm and hand

  // find angle between p and q (theta 4)
  float pDotq = p[0]*q[0] + p[1]*q[1] + p[2]*q[2];
  float pMagnitude = sqrt(sq(p[0]) + sq(p[1]) + sq(p[2]));
  float qMagnitude = sqrt(sq(q[0]) + sq(q[1]) + sq(q[2]));

  float theta4Degrees = acos(pDotq/(pMagnitude*qMagnitude))*180/M_PI;// - 180;

  if (isnan(theta4Degrees))
  {
    theta4Degrees = 90;
  }

  // find theta 5
  float elbowGripDist = sqrt(sq(x-xElbow) + sq(y-yElbow) + sq(z-zElbow));
  float elbowSphDist = sqrt(sq(xSph-xElbow) + sq(ySph-yElbow) + sq(zSph-zElbow));
  float sphGripDist = sqrt(sq(xSph-x) + sq(ySph-y) + sq(zSph-z));

  float theta5Degrees = acos((sq(elbowSphDist) + sq(sphGripDist) - sq(elbowGripDist))/(2*elbowSphDist*sphGripDist))*180/M_PI;


  float theta6Degrees = g;

  if(debugging)
  {
    // print goal thetas
    Serial.print("theta1: "); Serial.println(theta1Degrees);
    Serial.print("theta2: "); Serial.println(theta2Degrees);
    Serial.print("theta3: "); Serial.println(theta3Degrees);
    Serial.print("theta4: "); Serial.println(theta4Degrees);
    Serial.print("theta5: "); Serial.println(theta5Degrees);
    Serial.print("theta6: "); Serial.println(theta6Degrees);

    Serial.print("hand projection onto XY plane: ");Serial.println(d);
  }
  
  goalPositions[0] = theta1Degrees * stepsPerRev[0]/360;
  goalPositions[1] = theta2Degrees * stepsPerRev[1]/360;
  goalPositions[2] = theta3Degrees * stepsPerRev[2]/360;
  goalPositions[3] = theta4Degrees * stepsPerRev[3]/360;
  // account for extra theta5 angle from theta4
  // rotating axis 4 3450 rotates axis 5 2570, or every one step for axis 4 is 0.745 steps for axis 5
  goalPositions[4] = theta5Degrees * stepsPerRev[4]/360 - long(goalPositions[3] * 0.745);
  goalPositions[5] = theta6Degrees * stepsPerRev[5]/360;
  
  steppers.moveTo(goalPositions);
}
