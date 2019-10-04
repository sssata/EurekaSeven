/*
	StewartInverseKinematics - Library for calculating the 
	inverse kinematics of a servo-driven stewart platform.
	Created by Eureka Seven, October 2019
*/

#ifndef StewartInverseKinematics_h
#define StewartInverseKinematics_h

#include "Geometry.h"
#include <math.h>

// Inverse kinematics class
class StewartInverseKinematics {
  public:
    StewartInverseKinematics();
    void setGeometry(float servoX, float servoY, float platformX, float platformY, float newRodLength, float newHornLength, bool isHornOut);
    int getServoAngles(float outputAngles [], float dx, float dy, float dz, float yaw, float pitch, float roll);
    void printGeometry();
	bool isInitalized;
  private:
  	void populateServoPlatformVectors();
    Point heightVector;
    Point displacementVector;
    Point servoVector [6];
    Point platformVector [6];
    float hornDirection [6];
    float rodLength;
    float hornLength;
};


#endif
