
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

String inputString;

float angleX;
float angleY;
float angleZ;

Point dVector;

Point servo [6];
Point platform [6];

Point height;                 // height of platform plane above servo axis plane

const float ROD_LENGTH = 165;
const float HORN_LENGTH = 17.5;
const float PLATFORM_HEIGHT = 155;
const float FIRST_SERVO_X = -65;
const float FIRST_SERVO_Y = -60;
const float FIRST_PLATFORM_X = -20;
const float FIRST_PLATFORM_Y = -80;
const float MAX_ANGLE = 5 * PI / 180;

float output [6];
float testOutput [6];



Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();



// ----- Setup -----
void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600); // opens serial port, sets data rate to 9600 bps
  Serial.setTimeout(10); // change default (1000ms) to have faster response time
  Serial.println("Running example: Servo motor actuation using messaging");
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

  height.X() = 0.0;
  height.Y() = 0.0;
  height.Z() = PLATFORM_HEIGHT;

  dVector.X() = 0;
  dVector.Y() = 0;
  dVector.Z() = 0;

  angleX=0;
  angleY=0;
  angleZ=0;

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
    Serial.print("Receieved: ");
    Serial.print(inputString);



    switch (inputString[0]) {
      case 't':
        String inputs [6];
        float inputFloat [6];
        for (int i = 0; i < 6; i++) {

          Serial.print("\nEnter input ");
          Serial.print(i);
          Serial.print(": ");
          while (!Serial.available()) {}
          inputs[i] = Serial.readString();
          inputFloat[i] = inputs[i].toFloat();
          Serial.print(inputFloat[i]);
          inputFloat[i] = inputFloat[i]/180*PI;
        }

        toServoAngles(testOutput, inputFloat[0], inputFloat[1], inputFloat[2], inputFloat[3], inputFloat[4], inputFloat[5]);
        
        Serial.print("\nTest Servo Angles: ");
        for (int i = 0; i < 6; i++) {

          Serial.print(testOutput[i]/PI*180);
          Serial.print(" ");
        }

        break;
      case 'w':
        angleX = angleX + (1 * PI / 180);
        break;
      case 's':
        angleX = angleX - (1 * PI / 180);
        break;
      case 'a':
        angleY = angleY + (1 * PI / 180);
        break;
      case 'd':
        angleY = angleY - (1 * PI / 180);
        break;
      case 'e':
        dVector.Z() = dVector.Z() + 1;
        break;
      case 'q':
        dVector.Z() = dVector.Z() - 1;
        break;
    }


    toServoAngles(output, dVector.X(), dVector.Y(), dVector.Z(), angleZ, angleX, angleY);

    Serial.print("\nServo Angles: ");
    for (int i = 0; i < 6; i++) {

      Serial.print(output[i]/PI*180);
      Serial.print(" ");
    }
  }

}

void angleToPulseWidth() {

}

void toServoAngles(float servoAngle[], float dx, float dy, float dz, float yaw, float pitch, float roll) {

  Point platformRotated [6];
  Point virtualRod [6];

  Point displacement;           // displacement vector in mm
  displacement.X() = dx;
  displacement.Y() = dy;
  displacement.Z() = dz;

  Rotation R;                   // Platform rotation definition using euler angles
  R.RotateZ(yaw);
  R.RotateX(pitch);
  R.RotateY(roll);

  float virtualRodLength [6];  // Virtual rod length used to calculate servo angle

  float rodL = ROD_LENGTH;
  float hornL = HORN_LENGTH;

  // for every rod
  for (int i = 0; i < 6; i++) {

    // rotate platform joint vector
    platformRotated[i] = R * platform[i];

    // calculate virtual rod vector
    virtualRod[i] = height + displacement + platformRotated[i] - servo[i];

    // get virtual rod length
    virtualRodLength[i] = virtualRod[i].Magnitude();

    Serial.print("\nVirtual rod length:");
    Serial.print(virtualRodLength[i]);


    // calculate servo angle
    if (virtualRodLength[i] >= rodL + hornL) {
      servoAngle[i] = PI/2;
    } else if (virtualRodLength[i] <= rodL - hornL) {
      servoAngle[i] = -PI/2;
    } else {
      servoAngle[i] = PI/2 - acos( ((virtualRodLength[i] * virtualRodLength[i]) + (hornL * hornL) - (rodL * rodL)) / (2 * virtualRodLength[i] * hornL) );
    }
    
    Serial.print("\nServoAngle:");
    Serial.print(servoAngle[i]);



  }

}
