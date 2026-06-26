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
  Serial.println(F("ALL_SERVOS_AT_90"));
}

void loop() {
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 3; j++)
      servo[i][j].write(90);
  delay(500);
}
