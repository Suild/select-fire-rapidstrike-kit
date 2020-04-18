#include <SoftwareSerial.h>
#include <Button.h>

Button btn (2, true, true, 25);

void setup() {

  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  btn.read();
  
  if (btn.isPressed()) {
    Serial.println("pressed");
  } else {
    Serial.println("not pressed");
  }
}
