
// CONTROLS:
// W/S : pitch
// A/D : roll
// Q/E : yaw
// n : reset to neutral
// t : test mode (input X,Y,Z displacement and yaw, pitch, roll values afterwards)

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Geometry.h>
#include <math.h>
#include <string.h>

// ----- Initialize Variables ------

const int SERVOMIN = 150; // 'minimum' pulse length count (out of 4096)
const int SERVOMAX = 480; // 'maximum' pulse length count (out of 4096)
const int SERVOMID = floor((SERVOMAX + SERVOMIN) / 2); // 'mid' pulse length count (out of 4096)
const int SERVOCHG = 10; // 'change' pulse length count

int servoMin [6] = {140, 140, 0, 0, 0, 0};
int servoMax [6] = {540, 510, 0, 0, 0, 570};
int servoZero [6];

String inputString;           // serial input string


float angleX;                 // Current Set angles
float angleY;
float angleZ;

Point dVector;                // Current Set displacement



Point servo [6];              // servo locations
Point platform [6];           //platform joint locations


Point height;                 // height of platform plane above servo axis plane

const float ROD_LENGTH = 165;
const float HORN_LENGTH = 13.5;
const float PLATFORM_HEIGHT = 154.4;
const float FIRST_SERVO_X = -30;
const float FIRST_SERVO_Y = -85;
const float FIRST_PLATFORM_X = -20;
const float FIRST_PLATFORM_Y = -75;
const float MAX_ANGLE = 5 * PI / 180;

float hornDirection [6];  // servo arm direction at horizontal
/*hornDirection[0] = 180;
  hornDirection[1] = 0;
  hornDirection[2] = 300;
  hornDirection[3] = 120;
  hornDirection[4] = 60;
  hornDirection[5] = 240;*/


float servoAngleOutput [6];
float testOutput [6];

boolean isValidAngle;





Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();



// ----- Setup -----
void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600); // opens serial port, sets data rate to 9600 bps
  Serial.setTimeout(10); // change default (1000ms) to have faster response time
  Serial.println("Running example: Inverse kinematics calulator");
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates


  // Start Define geometry

  // set all servo and platform z coord to zero
  for (int i = 0; i < 6; i++) {
    servo[i].Z() = 0;
    platform[i].Z() = 0;
  }

  // Define all servo locations through first servo
  servo[0].X() = FIRST_SERVO_X;
  servo[0].Y() = FIRST_SERVO_Y;

  // mirror second servo location across yz plane
  servo[1].X() = -servo[0].X();
  servo[1].Y() = servo[0].Y();

  // Define all platform joint locations through first platform joint
  platform[0].X() = FIRST_PLATFORM_X;
  platform[0].Y() = FIRST_PLATFORM_Y;

  // mirror second platform joint across yz plane
  platform[1].X() = -platform[0].X();
  platform[1].Y() = platform[0].Y();


  // rotational 120 degree pattern around z axis for servos and platform joints
  for (int i = 1; i <= 2; i++) {
    Rotation RPattern;
    RPattern.RotateZ(i * PI * 2 / 3);

    servo[2 * i] = RPattern * servo[0];
    servo[2 * i + 1] = RPattern * servo[1];

    platform[2 * i] = RPattern * platform[0];
    platform[2 * i + 1] = RPattern * platform[1];
  }

  float nominalRodLength = sqrt(ROD_LENGTH * ROD_LENGTH + HORN_LENGTH * HORN_LENGTH);

  Point horizontalDist = servo[0] - platform[0];
  float nominalHeight = sqrt(nominalRodLength * nominalRodLength - horizontalDist.Magnitude() * horizontalDist.Magnitude());

  nominalHeight = sqrt(ROD_LENGTH * ROD_LENGTH - HORN_LENGTH * HORN_LENGTH - pow(servo[0].X() - platform[0].X(), 2) - pow(servo[0].Y() - platform[0].Y(), 2));

  height.X() = 0.0;
  height.Y() = 0.0;
  height.Z() = nominalHeight;


  hornDirection[0] = 0;
  hornDirection[1] = 180;
  hornDirection[2] = 120;
  hornDirection[3] = 300;
  hornDirection[4] = 240;
  hornDirection[5] = 60;


  // set current state to zero
  dVector.X() = 0;
  dVector.Y() = 0;
  dVector.Z() = 0;

  angleX = 0;
  angleY = 0;
  angleZ = 0;

  for (int i = 0; i < 6; i++) {
    servoZero[i] = SERVOMAX + SERVOMIN / 2;

  }

  for (int i = 0; i < 6; i++) {
    Serial.print("\nServo ");
    Serial.print(i);
    Serial.print(": X:");
    Serial.print(servo[i].X());
    Serial.print(" Y:");
    Serial.print(servo[i].Y());
    Serial.print(" Z:");
    Serial.print(servo[i].Z());
  }

  for (int i = 0; i < 6; i++) {
    Serial.print("\nPlatform ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(": X:");
    Serial.print(platform[i].X());
    Serial.print(" Y:");
    Serial.print(platform[i].Y());
    Serial.print(" Z:");
    Serial.print(platform[i].Z());
  }

}


// ----- Loop -----
void loop() {
  // put your main code here, to run repeatedly:

  if (Serial.available() > 0) {

    inputString = Serial.readString();
    Serial.print("\nReceieved: ");
    Serial.print(inputString);



    switch (inputString[0]) {

      // test case
      case 't':
        float inputFloat [6];
        for (int i = 0; i < 6; i++) {

          Serial.print("\nEnter input ");
          Serial.print(i);
          Serial.print(": ");
          while (!Serial.available()) {} // wait for input
          inputFloat[i] = Serial.parseFloat();
          Serial.print(inputFloat[i]);
          Serial.print("\n");
        }

        inputFloat[3] = inputFloat[3] / 180 * PI;
        inputFloat[4] = inputFloat[4] / 180 * PI;
        inputFloat[5] = inputFloat[5] / 180 * PI;

        int currentErrorState;
        currentErrorState = toServoAngles(testOutput, inputFloat[0], inputFloat[1], inputFloat[2], inputFloat[3], inputFloat[4], inputFloat[5]);

        Serial.print("TEST ANGLES\n");
        printServoState(testOutput, currentErrorState);


        break;
      case 'w':           // plus pitch
        angleX = angleX + (1 * PI / 180);
        break;
      case 's':           // minus pitch
        angleX = angleX - (1 * PI / 180);
        break;
      case 'a':           // plus roll
        angleY = angleY + (1 * PI / 180);
        break;
      case 'd':           // minus roll
        angleY = angleY - (1 * PI / 180);
        break;
      case 'e':           // plus yaw
        angleZ = angleZ + (1 * PI / 180);
        break;
      case 'q':           // minus yaw
        angleZ = angleZ - (1 * PI / 180);
        break;
      case 'n':           // reset to neutral
        dVector.X() = 0;
        dVector.Y() = 0;
        dVector.Z() = 0;
        angleX = 0;
        angleY = 0;
        angleZ = 0;
        break;
    }

    int currentErrorState;
    currentErrorState = toServoAngles(servoAngleOutput, dVector.X(), dVector.Y(), dVector.Z(), angleZ, angleX, angleY);

    printServoState(servoAngleOutput, currentErrorState);

  }

}

void angleToPulseWidth() {

}

void printServoState();

void printServoState(float servoAngleInput[], int errorState) {

  Serial.print("\nTest Servo Angles: ");
  for (int i = 0; i < 6; i++) {

    Serial.print(servoAngleInput[i] / PI * 180);
    Serial.print(" ");
  }

  if (errorState != 0) {
    Serial.print ("\nServo angle out of range!\n");
  } else {
    Serial.print ("\nServo angles OK\n");
  }
}

int toServoAngles(float servoAngle[], float dx, float dy, float dz, float yaw, float pitch, float roll) {

  int errorState = 0;           // error state of the function, returns number of servos out of bounds

  Point displacement;           // displacement vector in mm
  displacement.X() = dx;
  displacement.Y() = dy;
  displacement.Z() = dz;

  Rotation R;                   // Platform rotation definition using euler angles
  R.RotateZ(yaw);
  R.RotateX(pitch);
  R.RotateY(roll);

  float virtualRodLength;  // Virtual rod length used to calculate servo angle

  float rodL = ROD_LENGTH;
  float hornL = HORN_LENGTH;

  // for every rod
  for (int i = 0; i < 6; i++) {

    Point platformRotated;
    Point virtualRod;

    // rotate platform joint vector
    platformRotated = R * platform[i];

    // calculate virtual rod vector
    virtualRod = height + displacement + platformRotated - servo[i];

    // get virtual rod length
    virtualRodLength = virtualRod.Magnitude();

    //Serial.print("\nVirtual rod length:");
    //Serial.print(virtualRodLength[i]);

    // some angle compensation shit
    float virtualRodAngle;

    Point hornVector;
    hornVector.X() = cos(hornDirection[i] * PI / 180);
    hornVector.Y() = sin(hornDirection[i] * PI / 180);
    hornVector.Z() = 0;

    virtualRodAngle = acos((hornVector.DotProduct(virtualRod)) / (hornVector.Magnitude() * virtualRod.Magnitude()));

    float e = 2 * hornL * virtualRod.Z();

    float f = 2 * hornL * (cos(hornDirection[i] * PI / 180) * virtualRod.X() + sin(hornDirection[i] * PI / 180) * virtualRod.Y());

    float g = virtualRodLength * virtualRodLength - (rodL * rodL - hornL * hornL);

    float alpha = asin(g / sqrt(e * e + f * f)) - atan2(f, e);

    Serial.print("\nNew Alpha");
    Serial.print(alpha * 180 / PI);

    if (alpha != alpha) {
      errorState += 1;
    }


    /*// determine validity and calculate servo angle
      float cosServoAngle =  ((virtualRodLength * virtualRodLength) + (hornL * hornL) - (rodL * rodL)) / (2 * virtualRodLength * hornL);

      if (virtualRodLength >= rodL + hornL) {           //acos has a range of -1 to 1, check if out of bounds
      servoAngle[i] = virtualRodAngle;
      errorState = errorState + 1;

      } else if (virtualRodLength <= rodL - hornL) {
      servoAngle[i] = virtualRodAngle - PI;
      errorState = errorState + 1;

      } else {
      servoAngle[i] = virtualRodAngle - acos(cosServoAngle);
      }*/

    servoAngle[i] = alpha;


    Serial.print("\nRodLength:");
    Serial.print(virtualRodLength);
    Serial.print("\nVirtualRodAngle:");
    Serial.print(virtualRodAngle * 180 / PI);


  }

  return errorState;

}
