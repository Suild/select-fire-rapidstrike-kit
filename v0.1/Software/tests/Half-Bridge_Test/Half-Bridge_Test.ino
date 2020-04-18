#include <Button.h>

#define TRIGGER_PIN 2
#define HIGH_SIDE_PIN 3
#define LOW_SIDE_PIN 5

#define SHOOT_THROUGH_DELAY 25

Button trgBtn (TRIGGER_PIN, true, true, 25);

void setup() {
  // put your setup code here, to run once:
  pinMode(HIGH_SIDE_PIN, OUTPUT);  
  pinMode(LOW_SIDE_PIN, OUTPUT);

  controlMotors(0);

}

void loop() {
  // put your main code here, to run repeatedly:
  trgBtn.read();
  if (trgBtn.isPressed()) {
    controlMotors(2);
  } else {
    controlMotors(1);
  }
}

// targetState;
// 0 - all off
// 1 - brake
// 2 - full on
void controlMotors(int targetState) {
  if (targetState == 0) {
    digitalWrite(HIGH_SIDE_PIN, LOW);
    digitalWrite(LOW_SIDE_PIN, LOW);
  } else if (targetState == 1) {
    delay(SHOOT_THROUGH_DELAY);
    digitalWrite(LOW_SIDE_PIN, LOW);
    delay(SHOOT_THROUGH_DELAY);
    digitalWrite(HIGH_SIDE_PIN, HIGH);
  } else if (targetState == 2) {
    delay(SHOOT_THROUGH_DELAY);
    digitalWrite(HIGH_SIDE_PIN, LOW);
    delay(SHOOT_THROUGH_DELAY);
    digitalWrite(LOW_SIDE_PIN, HIGH);
  }
}


