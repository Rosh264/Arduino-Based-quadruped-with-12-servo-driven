#include <Servo.h>

Servo servo[4][3];
const int pin[4][3] = {{3,4,2},{6,7,5},{9,8,10},{12,11,13}};

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 3; j++) {
      servo[i][j].attach(pin[i][j]);
      servo[i][j].write(90);
      delay(50);
    }
  Serial.println(F("SERVO_TEST_READY"));
}

void sweep(int leg, int jnt) {
  Serial.print(F("L"));
  Serial.print(leg);
  Serial.print(F("J"));
  Serial.print(jnt);
  Serial.print(F(" P"));
  Serial.print(pin[leg][jnt]);
  Serial.print(F(": "));
  for (int p = 0; p <= 180; p += 5) { servo[leg][jnt].write(p); delay(15); }
  for (int p = 180; p >= 0; p -= 5) { servo[leg][jnt].write(p); delay(15); }
  servo[leg][jnt].write(90);
  Serial.println(F("OK"));
  delay(100);
}

void loop() {
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 3; j++)
      sweep(i, j);
  Serial.println(F("CYCLE_DONE"));
  delay(5000);
}



