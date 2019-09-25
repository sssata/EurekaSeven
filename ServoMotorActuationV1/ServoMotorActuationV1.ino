/* =================================================================================================== 
 *  This code has been provided as an example to help you get started on your project. The objective 
 *  is to provide user input to the Arduino board and have the servo motors actuate. Several lines of 
 *  code are accredited to the Adafruit PWM Servo Driver Library example code. To use the Adafruit 
 *  PWM Servo Shield, be sure to add the Adafruit library to your Arduino IDE. 
 *  (Adafruit example: File menu > Examples > Adafruit PWM Servo Drivers Library > servo)
 *  
 *  Add Adafruit Library: In the Arduino IDE application select: Sketch menu > Include Libraries > 
 *  Manage Libraries. In the Library Manager window, search and install the "Adafruit PWM Servo 
 *  Driver Library".
 *  
 *  NOTE: Depending on your servo motor, the pulse width min/max may be different. Adjust to match 
 *  your servo motor.
 =================================================================================================== */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Geometry.h>
#include <math.h>

const int SERVOMIN = 150; // 'minimum' pulse length count (out of 4096)
const int SERVOMAX = 480; // 'maximum' pulse length count (out of 4096)
const int SERVOMID = floor((SERVOMAX+SERVOMIN)/2); // 'mid' pulse length count (out of 4096)
const int SERVOCHG = 10; // 'change' pulse length count

int servoMin [6] = {140,140,0,0,0,0};
int servoMax [6] = {540,510,0,0,0,570};

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

String valInput; // Serial input var.
int i=0; // loop index var.
int val[6] = {SERVOMID, SERVOMID, SERVOMID, SERVOMID, SERVOMID, SERVOMID}; // PWM var

void setup() {
  Serial.begin(9600); // opens serial port, sets data rate to 9600 bps
  Serial.setTimeout(10); // change default (1000ms) to have faster response time
  Serial.println("Running example: Servo motor actuation using messaging");
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
}

void toServoAngles(float servoAngleOutput[], float x, float y, float z, float yaw, float pitch, float roll){
  
  // Initialize variables
  
  Point servo [6];
  Point platform [6];

  // Start Define geometry

  // set all servo and platform z coord to zero
  for (int i=0; i<6; i++){
    servo[i].Z() = 0;
    platform[i].Z() = 0;
  }

  // Define all servos through first servo
  servo[0].X() = -30;
  servo[0].Y() = -60;
  
  // mirror second servo across yz plane
  servo[1].X() = - servo[0].X();
  servo[1].Y() = servo[0].Y();

  
  // Define all platform joints through first servo
  platform[0].X() = -30; 
  platform[0].Y() = -60;

  // mirror second platform across yz plane
  platform[1].X() = - platform[0].X();
  platform[1].Y() = platform[0].Y();

  // rotational 120 degree pattern around z axis
  for (int i=1; i<=2; i++){
    Rotation RPattern;
    RPattern.RotateZ(i*PI*2/3); // 120 degree rotation
    
    servo[2*i] = RPattern * servo[0];
    servo[2*i+1] = RPattern * servo[1];

    platform[2*i] = RPattern * platform[0];
    platform[2*i+1] = RPattern * platform[1];
  }

  Point height;                 // height of platform plane above servo axis plane
  height.X() = 0.0;
  height.Y() = 0.0;
  height.Z() = 150.0;

  const float rodLength = 150;
  const float hornLength = 15;

  // End Define geometry

  // everything above here can be defined once in the beginning (stewart platform geometry definition)
  // everything below here must be in function
 
  Point platformRotated [6];
  Point virtualRod [6];
  
  Point displacement;           // displacement vector in mm
  displacement.X() = x;
  displacement.Y() = y;
  displacement.Z() = z;
  
  Rotation R;                   // Platform rotation definition using euler angles
  R.RotateZ(yaw);
  R.RotateX(pitch);
  R.RotateY(roll);

  float virtualRodLength [6];  // Virtual rod length used to calculate servo angle
  float servoAngle [6];        // Servo angle above horizontal
  

  // for every rod
  for (int i=0; i<6; i++){

    // rotate platform joint vector
    platformRotated[i] = R * platform[i];

    // calculate virtual rod vector
    virtualRod[i] = height + displacement + platformRotated[i] - servo[i];

    // get virtual rod length
    virtualRodLength[i] = virtualRod[i].Magnitude();

    // calculate servo angle
    servoAngle[i] = PI/2 - acos((virtualRodLength[i]*virtualRodLength[i] + hornLength*hornLength - rodLength*rodLength) / (2 * virtualRodLength[i] * hornLength));
    
  }

  servoAngleOutput = servoAngle;
  
}

float nvidiaACos(float x) { // 3x faster than math.h acos()
  float negate = float(x < 0);
  x = abs(x);
  float ret = -0.0187293;
  ret = ret * x;
  ret = ret + 0.0742610;
  ret = ret * x;
  ret = ret - 0.2121144;
  ret = ret * x;
  ret = ret + 1.5707288;
  ret = ret * sqrt(1.0 - x);
  ret = ret - 2 * negate * ret;
  return negate * 3.14159265358979 + ret;
}

void loop() {
  if (Serial.available() > 0) {

    valInput = Serial.readString();
    Serial.print("I received: ");
    Serial.print(valInput);

    switch (valInput[0]) {
      // Input of "1" to "6" -> increase respective (1..6) values
      // Input of [q,w,e,r,t,y] -> decrease respective (1..6) values
      
      case '1': 
        val[0]= min( val[0]+SERVOCHG, SERVOMAX );
        break;
      case 'q': 
        val[0]= max( val[0]-SERVOCHG, SERVOMIN );
        break;
      
      case '2': 
        val[1]= min( val[1]+SERVOCHG, SERVOMAX );
        break;
      case 'w': 
        val[1]= max( val[1]-SERVOCHG, SERVOMIN );
        break;
      
      case '3': 
        val[2]= min( val[2]+SERVOCHG, SERVOMAX );
        break;
      case 'e': 
        val[2]= max( val[2]-SERVOCHG, SERVOMIN );
        break;

      case '4':
        val[3]= min( val[3]+SERVOCHG, SERVOMAX );
        break;
      case 'r':
        val[3]= max( val[3]-SERVOCHG, SERVOMIN );
        break;

      case '5':
        val[4]= min( val[4]+SERVOCHG, SERVOMAX );
        break;
      case 't':
        val[4]= max( val[4]-SERVOCHG, SERVOMIN );
        break;

      case '6':
        val[5]= min( val[5]+SERVOCHG, SERVOMAX );
        break;
      case 'y':
        val[5]= max( val[5]-SERVOCHG, SERVOMIN );
        break;

      case '<':
        for (i=0; i<6; i++) {
          val[i] = SERVOMIN;
        }
        break;
      case '>':
        for (i=0; i<6; i++) {
          val[i] = SERVOMAX;
        }
        break;
      case 'm':
        for (i=0; i<6; i++) {
          val[i] = SERVOMID;
        }
        break;
        
        case '-':
        for (i=0; i<6; i++) {
          val[i] = max( val[i]-5, SERVOMIN );
        }
        break;
                
        case '+':
        for (i=0; i<6; i++) {
          val[i] = min( val[i]+5, SERVOMAX );
        }
        break;
        
      default: Serial.print(" No action taken");
    } // end switch statement
    
    Serial.print(" Servo values = [");
    for (i=0; i<6; i++) {
      Serial.print(val[i]);
      Serial.print(" ");
    }
    Serial.println("]");

    // Update servo commands:
    for (i=0; i<6; i++) {
      pwm.setPWM(i+1, 0, val[i]); // added +1 to match PWM port numbering (pins 1..6 used)
    }
  }
}
