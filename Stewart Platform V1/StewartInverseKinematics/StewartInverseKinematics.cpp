/*
	StewartInverseKinematics - Library for calculating the 
	inverse kinematics of a servo-driven stewart platform.
	Created by Eureka Seven, October 2019
*/

#include "Arduino.h"
#include "Geometry.h"
#include "StewartInverseKinematics.h"

StewartInverseKinematics :: StewartInverseKinematics(){
	isInitalized = false;
}


void StewartInverseKinematics :: setGeometry(
float servoX, 
float servoY, 
float platformX, 
float platformY, 
float newRodLength, 
float newHornLength,
bool isHornOut) {
	
	
	// Define all platform joint locations through first platform joint and servo
	servoVector[0].X() = servoX;
	servoVector[0].Y() = servoY;
	
	platformVector[0].X() = platformX;
	platformVector[0].Y() = platformY;
	
	// populate the rest of the servo and platform locations
	populateServoPlatformVectors();
	
	// define servo horn layout
	if(isHornOut){
		hornDirection[0] = 180;
		hornDirection[1] = 0;
		hornDirection[2] = 300;
		hornDirection[3] = 120;
		hornDirection[4] = 60;
		hornDirection[5] = 240;
	} else {
		hornDirection[0] = 0;
		hornDirection[1] = 180;
		hornDirection[2] = 120;
		hornDirection[3] = 300;
		hornDirection[4] = 240;
		hornDirection[5] = 60;
	}
	
	rodLength = newRodLength;
	hornLength = newHornLength;
	
	float nominalHeight = sqrt((rodLength * rodLength) - (hornLength * hornLength) - pow(servoVector[0].X() - platformVector[0].X(), 2) - pow(servoVector[0].Y() - platformVector[0].Y(), 2));
	
	heightVector.X() = 0.0;
	heightVector.Y() = 0.0;
	heightVector.Z() = nominalHeight;
	
	isInitalized = true;
	
	
}

int StewartInverseKinematics :: getServoAngles(float outputAngles [], float dx, float dy, float dz, float yaw, float pitch, float roll){
	int errorState = 0;
	
	Point displacementVector;           // given displacement vector
	displacementVector.X() = dx;
	displacementVector.Y() = dy;
	displacementVector.Z() = dz;
	
	Rotation R;
	R.RotateZ(yaw);
	R.RotateX(pitch);
	R.RotateY(roll);
	
	for (int i = 0; i < 6; i++) {
		
		// Rotate platform vector
		Point platformVectorRotated = R * platformVector[i];
		
		// Calulate virtual rod vector
		Point virtualRodVector = heightVector + displacementVector + platformVectorRotated - servoVector[i];
		
		// Find virtual rod length
		float virtualRodLength = virtualRodVector.Magnitude();
		
		// Calculate servo angle above horizontal
		float e = 2 * hornLength * virtualRodVector.Z();
		
		float f = 2 * hornLength * (cos(hornDirection[i] * PI / 180) * virtualRodVector.X() + sin(hornDirection[i] * PI / 180) * virtualRodVector.Y());
		
		float g = virtualRodLength * virtualRodLength - (rodLength * rodLength - hornLength * hornLength);
		
		float alpha = asin(g / sqrt(e * e + f * f)) - atan2(f, e);
		
		// check if alpha is nan
		if (alpha != alpha){
			errorState += 1 << i;
		}
		
		outputAngles[i] = alpha;
		
		
	}
	
}

void StewartInverseKinematics :: populateServoPlatformVectors() {
	
	// set all servo and platform z coord to zero
	for (int i = 0; i < 6; i++) {
    	servoVector[i].Z() = 0;
    	platformVector[i].Z() = 0;
	}
	
	// mirror second servo location across yz plane
	servoVector[1].X() = -servoVector[0].X();
	servoVector[1].Y() = servoVector[0].Y();
	
	// mirror second platform joint across yz plane
	platformVector[1].X() = -platformVector[0].X();
	platformVector[1].Y() = platformVector[0].Y();
	
	// rotational 120 degree pattern around z axis for servos and platform joints
	for (int i = 1; i <= 2; i++) {
		Rotation RPattern;
		RPattern.RotateZ(i * PI * 2 / 3);

    	servoVector[2 * i] = RPattern * servoVector[0];
    	servoVector[2 * i + 1] = RPattern * servoVector[1];

		platformVector[2 * i] = RPattern * platformVector[0];
		platformVector[2 * i + 1] = RPattern * platformVector[1];
	}
}

void StewartInverseKinematics :: printGeometry() {
	if (isInitalized){
		for (int i = 0; i < 6; i++) {
    		Serial.print("\nServo ");
    		Serial.print(i);
    		Serial.print(": X:");
    		Serial.print(servoVector[i].X());
    		Serial.print(" Y:");
    		Serial.print(servoVector[i].Y());
    		Serial.print(" Z:");
    		Serial.print(servoVector[i].Z());
		}

  		for (int i = 0; i < 6; i++) {
			Serial.print("\nPlatform ");
			Serial.print(i);
			Serial.print(": ");
			Serial.print(": X:");
			Serial.print(platformVector[i].X());
			Serial.print(" Y:");
			Serial.print(platformVector[i].Y());
			Serial.print(" Z:");
			Serial.print(platformVector[i].Z());
		}
	
		for (int i = 0; i < 6; i++) {
			Serial.print("\nHorn Direction ");
			Serial.print(i);
			Serial.print(": ");
			Serial.print(hornDirection[i]);
		}
	
		Serial.print("\nrod length: ");
		Serial.println(rodLength);
		Serial.print("horn length: ");
		Serial.println(hornLength);
		Serial.print("nominal height: ");
		Serial.println(heightVector.Z());
	} else {
		Serial.println("Stewart platform geometry not set.");
	}
	
	
}




