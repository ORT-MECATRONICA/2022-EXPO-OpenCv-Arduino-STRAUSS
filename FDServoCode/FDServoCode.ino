#include <Servo.h>
#define PIN_SERVO 10
Servo servo;

int valorServo;
String lecturaSerial;

void setup() {
  Serial.begin(9600);
  servo.attach(PIN_SERVO);
}

void loop() {
  if (Serial.available()) {
    //lecturaSerial = Serial.readString();
    //valorServo = lecturaSerial.toInt();
    valorServo = Serial.parseInt();
    servo.write(valorServo);
  }
}
