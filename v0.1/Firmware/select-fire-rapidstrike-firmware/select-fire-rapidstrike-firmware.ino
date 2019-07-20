/* ------------------------------------------------------------------------

Firmware for first prototype of Select-Fire Rapidstrike Kit
By Monty Choy/Suild
Sun, 06/23/19

This sketch features a half-bridge with shoot-through protection to drive
the pusher motor, a rotary switch to toggle between different fire modes,
and a pot to adjust rate of fire.

This version currently utilizes open-source libraries because I don't 
intend on selling the kit with this version of the firmware. In the future,
I will make my own libraries that will be in production firmware.

This version also supports solenoid control, but everything must be 
configured in software.

Future software features:
- Jumper to manually switch between pusher and solenoid control
- On-board pot to adjust pot duty cycle
- OCP implemented in pusher control feedback loop
- Indicator LEDs for overcurrent, overvoltage, and undervoltage

I put methods that may be usefull to people Frankensteining this code 
closer to the top right under void loop()

------------------------------------------------------------------------ */

#include <Button.h>

//
// Macros and vars for trigger and cycle control switch stuff
#define TRIGGER_PIN 2
#define CYC_CTRL_PIN 4

Button trgSw (TRIGGER_PIN, true, true, 25);
Button cycCtrlSw (CYC_CTRL_PIN, true, true, 25);

//
// Macros and variables for motor drive stuff
#define HIGH_SIDE_PIN 3
#define LOW_SIDE_PIN 5

#define PUSHER_DRIVE_STATE_OFF 0
#define PUSHER_DRIVE_STATE_BRAKE 1
#define PUSHER_DRIVE_STATE_ON 2

#define SHOOT_THROUGH_DELAY 5    // Be careful when decreasing this! A value too small can pop your MOSFETs

//
// Macros and vars for rotary switch stuff
#define ROT_SW_SAFETY_PIN 6
#define ROT_SW_SEMI_AUTO_PIN 7
#define ROT_SW_BURST_FIRE_PIN 8
#define ROT_SW_FULL_AUTO_PIN 9

#define NUM_OF_ROT_SW_PINS 4
// Array of all rotary switch pins so it's easier to iterate through all of them
const uint8_t ROT_SW_PINS[] = {ROT_SW_SAFETY_PIN, ROT_SW_SEMI_AUTO_PIN, ROT_SW_BURST_FIRE_PIN, ROT_SW_FULL_AUTO_PIN};

//
// Macros for fire values
#define SAFETY 0
#define SEMI_AUTO 1
#define BURST_FIRE 2
#define FULL_AUTO 3

#define BURST_FIRE_LENGTH 3

//
// Macros and vars for variable fire rate control values
#define POT_PIN 0

//
// Singleton for coordinating all firing actions
struct FiringSingleton {
  // Switch states
  bool wasTriggerPulled = false;
  bool isTriggerPressed = false;
  bool isCycCtrlSwPressed = false;

  uint8_t currentFireMode = SAFETY;

  uint8_t dartsFired = 0;
  uint8_t dartsToFire = 0;

  uint8_t targetState = PUSHER_DRIVE_STATE_OFF;

  uint8_t pusherPow = 0;

  // For handling shoot-through
  uint8_t lastTargetState = PUSHER_DRIVE_STATE_OFF;
  uint32_t halfBridgeTransitionBeginTime = 0;
  uint32_t halfBridgeDriveBeginTime = 0;
  bool hasTargetStateTransitioned = false;
} firingSingleton;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  pinMode(HIGH_SIDE_PIN, OUTPUT);  
  pinMode(LOW_SIDE_PIN, OUTPUT);

  setRotSwPins();

}

void loop() {
  // put your main code here, to run repeatedly:
  handleTriggerPull();
  handleCycleControl();
  handleFiring();
  controlMotors();
}

// Print fire modes over serial 
void printCurrentFireMode() {
  if (firingSingleton.currentFireMode == SAFETY) {
      Serial.println("Safety");
    } else if (firingSingleton.currentFireMode == SEMI_AUTO) {
      Serial.println("Single Shot");
    } else if (firingSingleton.currentFireMode == BURST_FIRE) {
      Serial.println("Burst Fire");
    } else if (firingSingleton.currentFireMode == FULL_AUTO) {
      Serial.println("Full Auto");
    }
}

// Returns position of rotary switch in terms of which pin is connected to COM (pin is low)
uint8_t getRotSwPos(uint8_t rotSwPins[], uint8_t numOfRotSwPins) {
  for (int rotSwPin = 0; rotSwPin < NUM_OF_ROT_SW_PINS; rotSwPin++) {
    if (digitalRead(rotSwPins[rotSwPin]) == LOW) {
      return rotSwPins[rotSwPin];
    }
  }

}

// Directly handles driving the pusher, including shoot-through protection 
void controlMotors() {
  if (firingSingleton.targetState == PUSHER_DRIVE_STATE_OFF) {
    // Wait for pusher retract fully before killing power
    digitalWrite(HIGH_SIDE_PIN, LOW);
    digitalWrite(LOW_SIDE_PIN, LOW);
  // Targetstate requires MOSFETs to change state that's at risk of shoot-through
  } else {
    initHalfBridgeTransitionTiming();
    transitionHalfBridge();
    driveMOSFETs();
  }

}

// Keep track of time that transition of target state requested
// init times to keep track of half-bridge transition timing to prevent shoot through
void initHalfBridgeTransitionTiming() {
  if (firingSingleton.targetState != firingSingleton.lastTargetState) {
      firingSingleton.halfBridgeTransitionBeginTime = millis() + SHOOT_THROUGH_DELAY;
      firingSingleton.halfBridgeDriveBeginTime = millis() + (SHOOT_THROUGH_DELAY * 2);

      firingSingleton.lastTargetState = firingSingleton.targetState;

      // Reset flags for target state and MOSFET transitions to allow MOSFETs to transition
      firingSingleton.hasTargetStateTransitioned = false;
    }
}

// Transition MOSFETs to prevent shoot through
void transitionHalfBridge() {
  if (!firingSingleton.hasTargetStateTransitioned && millis() > firingSingleton.halfBridgeTransitionBeginTime) {
    if (firingSingleton.targetState == PUSHER_DRIVE_STATE_BRAKE) digitalWrite(LOW_SIDE_PIN, LOW);
    if (firingSingleton.targetState == PUSHER_DRIVE_STATE_ON) digitalWrite(HIGH_SIDE_PIN, LOW);

   firingSingleton.hasTargetStateTransitioned = true;
  } 
}

// Drive MOSFETs after transition occurs to prevent shoot-through
void driveMOSFETs() {
  if (millis() > firingSingleton.halfBridgeDriveBeginTime) {  
      firingSingleton.pusherPow = map(analogRead(POT_PIN), 0, 1024, 0, 255);

      if (firingSingleton.targetState == PUSHER_DRIVE_STATE_BRAKE && firingSingleton.isCycCtrlSwPressed) digitalWrite(HIGH_SIDE_PIN, HIGH);
      // if (firingSingleton.targetState == PUSHER_DRIVE_STATE_ON) analogWrite(LOW_SIDE_PIN, firingSingleton.pusherPow);
      if (firingSingleton.targetState == PUSHER_DRIVE_STATE_ON) digitalWrite(LOW_SIDE_PIN, HIGH);

    } 
}

// Sets pins for rotary switch
void setRotSwPins() {
  // Set all rotary switch pins to inputs with pullups
  for (int i = 0; i < sizeof(ROT_SW_PINS)/sizeof(ROT_SW_PINS[0]); i++) {
    pinMode(ROT_SW_PINS[i], INPUT_PULLUP);
  }
}


// Handles firing initiation when the trigger is pulled
void handleTriggerPull() {
  trgSw.read();
  firingSingleton.isTriggerPressed = trgSw.isPressed();
  firingSingleton.wasTriggerPulled = trgSw.wasPressed();

  if (firingSingleton.wasTriggerPulled) {
    const uint8_t ROT_SW_POS = getRotSwPos(ROT_SW_PINS, NUM_OF_ROT_SW_PINS);

    // Set fire mode based on position of rotary switch
    // The firemode is set here so the rotary switch is sampled less and if the rotary switch 
    // changes position while firing, it won't break the system
    setCurrentFiremode(ROT_SW_POS);
    printCurrentFireMode();

  } 

  // basic on/brake on pusher for testing
  // if (trgSw.isPressed()) {
  //   firingSingleton.targetState = PUSHER_DRIVE_STATE_ON;
  // } else {
  //   firingSingleton.targetState = PUSHER_DRIVE_STATE_BRAKE;
  // }
}

void setCurrentFiremode(uint8_t ROT_SW_POS) {
  if (ROT_SW_POS == ROT_SW_SAFETY_PIN) {
    firingSingleton.currentFireMode = SAFETY;
  } else if (ROT_SW_POS == ROT_SW_SEMI_AUTO_PIN) {
    firingSingleton.currentFireMode = SEMI_AUTO;
  } else if (ROT_SW_POS == ROT_SW_BURST_FIRE_PIN) {
    firingSingleton.currentFireMode = BURST_FIRE;
  } else if (ROT_SW_POS == ROT_SW_FULL_AUTO_PIN) {
    firingSingleton.currentFireMode = FULL_AUTO;
  }
}

void handleCycleControl() {
  cycCtrlSw.read();
  firingSingleton.isCycCtrlSwPressed = cycCtrlSw.isPressed();

  if (cycCtrlSw.wasPressed()) {
    firingSingleton.dartsFired++;
    Serial.println(firingSingleton.dartsFired);
  }
}

void handleFiring() {
  if (firingSingleton.wasTriggerPulled) {
    // Safety, brake motors
    handleSafety();
    setDartsToFire(); 
    firingSingleton.targetState = PUSHER_DRIVE_STATE_ON;
  }

  // If motors are running
  if (firingSingleton.targetState == PUSHER_DRIVE_STATE_ON) {
    if (firingSingleton.currentFireMode == SEMI_AUTO || firingSingleton.currentFireMode == BURST_FIRE) {
      handleSemiAutoAndBurst();
    } else if (firingSingleton.currentFireMode == FULL_AUTO) {
      handleFullAuto();
    } else {  // Motors are running but not in full auto or semi auto mode, so turn off motors
      firingSingleton.targetState = PUSHER_DRIVE_STATE_BRAKE;
    }
  }
  
}

void handleSafety() {
  if (firingSingleton.currentFireMode == SAFETY) {
      firingSingleton.targetState = PUSHER_DRIVE_STATE_BRAKE;
      return;
    }
}

void handleSemiAutoAndBurst() {
   // If no more darts to fire, brake pusher
  if (firingSingleton.dartsFired >= firingSingleton.dartsToFire) {
    firingSingleton.targetState = PUSHER_DRIVE_STATE_BRAKE;
    firingSingleton.dartsFired = 0;
  }
}

void handleFullAuto() {
   // Trigger let go and pusher returned, so brake
  if (!firingSingleton.isTriggerPressed && firingSingleton.isCycCtrlSwPressed) {
    firingSingleton.targetState = PUSHER_DRIVE_STATE_BRAKE;
  }
}

void setDartsToFire() {
  if (firingSingleton.currentFireMode == SEMI_AUTO) {
    firingSingleton.dartsToFire = 1;
  } else if (firingSingleton.currentFireMode == BURST_FIRE) {
    firingSingleton.dartsToFire = BURST_FIRE_LENGTH;
  }

  firingSingleton.dartsFired = 0;
}

