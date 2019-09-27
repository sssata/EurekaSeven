#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

int requestedPosition;
int currentPosition;
int rate = 6;
int setDelay = 33;

String input;
int inputValue;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); // opens serial port, sets data rate to 9600 bps
  Serial.setTimeout(10); // change default (1000ms) to have faster response time
  Serial.println("Running example: Sweep tes-1t");
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  currentPosition = 300;
  pwm.setPWM(10, 0, currentPosition);

}

void loop() {
  // put your main code here, to run repeatedly:

  while (!Serial.available()) {}
  input = Serial.readString();
  inputValue = input.substring(1).toInt();

  switch (input[0]) {
    case 'p':
      requestedPosition = inputValue;

      requestedPosition = map(requestedPosition, 0, 90, 310, 150);
      requestedPosition = constrain(requestedPosition, 420,150);
    
      Serial.print("Moving servo from: ");
      Serial.print(currentPosition);
      Serial.print(" to ");
      Serial.println(requestedPosition);

      while (requestedPosition != currentPosition) {

        int difference = requestedPosition - currentPosition;

        if (difference > 0) {
          currentPosition += min(rate, abs(difference));
        } else {
          currentPosition -= min(rate, abs(difference));
        }

        pwm.setPWM(10, 0, currentPosition);

        delay(setDelay);
      }
      break;
    case 'r':
      Serial.print("Changing rate from: ");
      Serial.print(rate);
      Serial.print(" to ");
      Serial.println(inputValue);
      rate = inputValue;
      break;
    case 'd':
      Serial.print("Changing delay from: ");
      Serial.print(setDelay);
      Serial.print(" to ");
      Serial.println(inputValue);
      setDelay = inputValue;
      break;
  }



}
