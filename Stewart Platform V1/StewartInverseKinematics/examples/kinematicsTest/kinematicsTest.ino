/*
 * kinematicsTest
 * 
 * This sketch demonstrates the use of the StewarInverseKinematics library by
 * allowing the user to input a requested orientation over the serial monitor.
 * The resultant servo angles are then calculated and printed.
 * 
 * Discriptions of the methods and their parameters in the libary are
 * included where the method is used.
 * 
 * Created by Eureka Seven, Oct 2019
 * 
 */


#include <StewartInverseKinematics.h>


StewartInverseKinematics kinematics = StewartInverseKinematics(); // Initalize a StewartInverseKinematics object

float testOutput [6];

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); // opens serial port, sets data rate to 9600 bps
  Serial.setTimeout(10); // change default (1000ms) to have faster response time
  Serial.println("Running example: Inverse kinematics calulator");

  kinematics.setGeometry(-30, -85, -45, -62.5, 165, 13.5, false); // define the geometry of the stewart platform

  /*
   * setGeometry
     Parameters:
     X location of bottom left servo (when viewed from top) (in mm) (float)
     Y location of bottom left servo (when viewed from top) (in mm) (float)
     X location of bottom left platform joint (when viewed from top) (in mm) (float)
     Y location of bottom left platform joint (when viewed from top) (in mm) (float)
     rod length (float)
     servo horn length (float)
     whether or not the bottom two servo horns face outward in neutral position (boolean)
  */

  kinematics.printGeometry();


}

void loop() {

  float inputFloat[6];
  String labels [6] =
  { "x displacement",
    "y displacement",
    "z displacement",
    "yaw",
    "pitch",
    "roll"
  };

  Serial.print("\n ---------- \n");

  for (int i = 0; i < 6; i++) {

    Serial.print("Enter ");
    Serial.print(labels[i]);
    Serial.print(": ");
    while (!Serial.available()) {} // wait for input
    inputFloat[i] = Serial.parseFloat();
    Serial.println(inputFloat[i]);
  }

  inputFloat[3] = inputFloat[3] / 180 * PI;
  inputFloat[4] = inputFloat[4] / 180 * PI;
  inputFloat[5] = inputFloat[5] / 180 * PI;

  int currentErrorState;

  // calculate the servo angles given a requested orientation
  currentErrorState = kinematics.getServoAngles(testOutput, inputFloat[0], inputFloat[1], inputFloat[2], inputFloat[3], inputFloat[4], inputFloat[5]);

  /*
    getServoAngles
    Parameters:
    output array of servo angles (float [6])
    x displacement (float)
    y displacement (float)
    z displacement (float)
    yaw (rotation about z) (radians) (float)
    pitch (rotation about x) (radians) (float)
    roll (rotation about y) (radians) (float)
  */

  printServoState(testOutput, currentErrorState);

}


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

