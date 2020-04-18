#include <SoftwareSerial.h>
#include <Button.h>

#define BTN_PIN 2
#define OUT_PIN 3

Button btn (BTN_PIN, true, true, 25);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  pinMode(OUT_PIN, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  btn.read();
  
  if (btn.isPressed()) {
    Serial.println("pressed");
    digitalWrite(OUT_PIN, HIGH);
  } else {
    Serial.println("not pressed");
    digitalWrite(OUT_PIN, LOW);
  }
}
