#include <Servo.h>

#define PIN_SERVOY 10
#define PIN_SERVOX 9

Servo servoX;
Servo servoY;

int valorServoX;
int valorServoY;

String lecturaSerial;

void setup() {
  Serial.begin(9600);

  servoX.attach(PIN_SERVOX);
  servoY.attach(PIN_SERVOY);
  servoX.write(75);
  servoY.write(95);
}

void loop() {
  if (Serial.available()) {
    lecturaSerial = Serial.readString();
    servoPosition(lecturaSerial);
    servoX.write(valorServoX);
    servoY.write(valorServoY);
  }
}

void servoPosition(String serial) {
  int positionComa;
  String X, Y;

  positionComa = serial.indexOf(',');
  X = serial.substring(0, positionComa);
  Y = serial.substring(positionComa + 1);

  valorServoX = X.toInt();
  valorServoY = Y.toInt();
  valorServoY = map(valorServoY, 0, 180, 180, 0);
}
