#include <Arduino.h>
#include <timer.h>
#include <JC_Button.h>
#include <CircularBuffer.h>

//
// Macros and vars for trigger and cycle control switch stuff
#define TRIGGER_PIN 2
#define CYC_CTRL_PIN 4

//
// Macros for FET pins
#define HIGH_SIDE_PIN 3			
#define LOW_SIDE_PIN 5

// Macros to improve readability of FET transition functions
#define ON true
#define OFF false

// Macros for pusher drive state
#define PUSHER_DRIVE_STATE_OFF 0
#define PUSHER_DRIVE_STATE_BRAKE 1
#define PUSHER_DRIVE_STATE_ON 2

// Macros for current sensing
#define CURRENT_SENSE_PIN 1
#define SENSE_RESISTANCE 0.01
#define ARDUINO_SUPPLY_VOLTAGE 5.0
#define NUM_OF_SAMPLES_FOR_DIFFERENTIATION 3

//
// Macros and vars for variable fire rate control values
#define POT_PIN 0

//
// Macros for fire values
#define SAFETY 0
#define SEMI_AUTO 1
#define BURST_FIRE 2
#define FULL_AUTO 3

#define BURST_FIRE_LENGTH 3

//
// Macros to keep track of fire mechanisms
// Improve readability of pusher mechanism logic
#define PUSHER_MOTOR_FIRING_MECHANISM 0
#define SOLENOID_FIRING_MECHANISM 1

// Pin that is used to select pusher mechanism. Jumper off for motor, jumper on 
// for solenoid. Uses internal pullup so HIGH = motor, LOW = solenoid
#define PUSHER_MECHANISM_SELECT_PIN 11

//
// Macros and vars for rotary switch stuff
#define ROT_SW_SAFETY_PIN 6
#define ROT_SW_SEMI_AUTO_PIN 7
#define ROT_SW_BURST_FIRE_PIN 8
#define ROT_SW_FULL_AUTO_PIN 9

#define NUM_OF_ROT_SW_PINS 4
// Array of all rotary switch pins so it's easier to iterate through all of them
const uint8_t ROT_SW_PINS[] = {ROT_SW_SAFETY_PIN, ROT_SW_SEMI_AUTO_PIN, ROT_SW_BURST_FIRE_PIN, ROT_SW_FULL_AUTO_PIN};


// Function prototypes incase you're not using Arduino IDE 
// I'd highly recommend using Platform.io instead of Arduino IDE
void initSerial(void);
void initFETPins(void);

void controlMotors();
bool handleComplementaryFETTransition();
void turnOffAllFETs();
void setFETsForBraking();
void setFETsForOn();
void transitionHighSideFET(bool);
void transitionLowSideFET(bool);

void handleCycleControl();

void handleFiring();

uint8_t getRotSwPos(uint8_t rotSwPins[], uint8_t);
void setCurrentFiremode(uint8_t);
void printCurrentFireMode();
void setPusherMechanism();
void setRateOfFire();
void setDartsToFire();

void handlePusherMotorFiring();
void handlePusherMotorSemiAutoAndBurst();
void handlePusherMotorFullAuto();

void handleSolenoidFiring();
void updateSolenoidTiming();
bool handleSolenoidTuronOn();
void handleSolenoidSemiAutoAndBurst();
void handleSolenoidFullAuto();

float analogReadingToVoltage();
float voltageToCurrent(float);
void updateCurrentSenseValues(float);

Button trgSw (TRIGGER_PIN, 50, true, true);
Button cycCtrlSw (CYC_CTRL_PIN, 50, true, true);

// variables for managing all of firing state
struct firingState {
	uint8_t currentPusherState = PUSHER_DRIVE_STATE_OFF;
	// should only be mutated in controlMotors(), but can be accessed anywhere
	uint8_t targetPusherState = PUSHER_DRIVE_STATE_OFF;	

  // Switch states
  bool wasTriggerPulled = false;
  bool isTriggerPressed = false;
  bool isCycCtrlSwPressed = false;

	// Keep track of firing mechanism, solenoid or pusher. This is updated on 
	// trigger pull by reading the jumper
	bool pusherMechanism = PUSHER_MOTOR_FIRING_MECHANISM;

	uint8_t currentFireMode = FULL_AUTO;

	// Keep track if pusher is firing. I can't just check if the pusher is being 
	// powered because sometimes, the pusher is firing but off (for example, a
	// solenoid returning to its retracted position)
	bool isFiring = false;

	// Keeps track of how many darts to fire in semi- and burst-fire modes
	uint8_t dartsToFire = 0;

	// Keep track of how many darts have been fired
	uint8_t dartsFired = 0;

	// Rate of fire from 0 to 100 (100 fastest)
	uint8_t rateOfFire = 100;

	// Duty cyle for solenoid firing. Different fire rates will be a factor of 
	// this. Units are percent
	const uint8_t SOLENOID_DUTY_CYCLE = 70;

	// Time that solenoid is powered when firing, in ms
	uint16_t solenoidOnTime = 35;

	// Time that solenoid is off when firing, in ms
	uint16_t solenoidOffTime = 15;

	// Timer to keep track of when to turn on/off the pusher
	Timer<> firingTimer = timer_create_default();


} firingState;

// variables for managing all of MOSFET state
// these values should only be mutated in a FET function
// I should probably abstract this into its own class
struct fetState {
	// Timer for managing shoot through delay. 
	// From this lib: https://github.com/contrem/arduino-timer
	// Quick description: Non-blocking library for delaying function calls
	Timer<> shootThroughProtectionTimer = timer_create_default();

	// Shoot through delay, in ms. 
	// This value is read-only, so no need to store it in dynamic memory
	const uint8_t SHOOT_THROUGH_DELAY = 1;

	// Flag to indicate that a complementary FET transition has begun
	// This is mainly so multiple shoot-through timers doesn't start 
	bool hasComplementaryTransitionBegun = false;		

	bool currentHighSideFETState = OFF;
	bool targetHighSideFETState = OFF;

	bool currentLowSideFETState = OFF;
	bool targetLowSideFETState = OFF;

} fetState;

// Struct to keep track of sense resitors values for differentiation
struct currentSenseState {
	const uint8_t SAMPLE_DELAY = 15;	// Current sample delay, in ms

	// Current limit for OCP
	// If this limit is hit for longer than MAX_OCP_SAMPLES samples, then load 
	// will be turned off
	// This is only for flywheel operation
	const uint8_t OCP_CURRENT_LIMIT = 5;

	// If more than OCP_CURRENT_LIMIT is observed for MAX_OCP_SAMPLES samples, 
	// power to the load will be cut
	const uint8_t MAX_OCP_SAMPLES = 25;

	// Flag if OCP Threshold counter hit limit
	// flag to see if too much current. If this flag is true, load will turn off
	// in controlMotors()
	bool isTooMuchCurrent = false;

	// Counter to keep track how many times current is above OCP_CURRENT_LIMIT
	// Gets reset to 0 if current is less than OCP_CURRENT_LIMIT
	uint8_t ocpThresholdHitCounter = 0;

	// const uint8_t NUM_OF_SAMPLES_FOR_DIFFERENTIATION = 5;

	// Paralleling FIFO buffer for sample time (x) and current (y)
	CircularBuffer<float, NUM_OF_SAMPLES_FOR_DIFFERENTIATION> sampleTimeBuffer;	
	CircularBuffer<float, NUM_OF_SAMPLES_FOR_DIFFERENTIATION> currentValues;

	Timer<> currentSenseTimer = timer_create_default();

	// -1 instantaneousChangeInCurrent means that this value isn't ready yet
	// instantaneousChangeInCurrent might not be ready if there aren't enough
	// samples
	float instantaneousChangeInCurrent = -1;

	float instantaneousCurrentDraw = -1;
} currentSenseState;

void setup() {
	initSerial();
	initFETPins();
	initRotSwPins();

	pinMode(PUSHER_MECHANISM_SELECT_PIN, INPUT_PULLUP);

	trgSw.begin();
	cycCtrlSw.begin();

	// Start timer that executes monitorCurrent() every 
	// currentSenseState.SAMPLE_DELAY ms
	currentSenseState.currentSenseTimer
		.every(currentSenseState.SAMPLE_DELAY, monitorCurrent);
}

void loop() {
	fetState.shootThroughProtectionTimer.tick();
	currentSenseState.currentSenseTimer.tick();
	firingState.firingTimer.tick();


	handleTriggerPull();
	handleCycleControl();
	handleFiring();
 	controlMotors();

  
}

void initSerial() {
	Serial.begin(9600);
	while(!Serial){}
	Serial.println("Serial communication established");
}

void initFETPins() {
	pinMode(HIGH_SIDE_PIN, OUTPUT);
	pinMode(LOW_SIDE_PIN, OUTPUT);

	firingState.targetPusherState = PUSHER_DRIVE_STATE_OFF;

	controlMotors();
}

void initRotSwPins() {
  // Set all rotary switch pins to inputs with pullups
  for (int i = 0; i < sizeof(ROT_SW_PINS)/sizeof(ROT_SW_PINS[0]); i++) {
    pinMode(ROT_SW_PINS[i], INPUT_PULLUP);
  }
}

// Directly handles driving the pusher, including shoot-through protection 
// Controls motors based on firingState currentPusherState and targetPusherState
void controlMotors() {
	// If too much current, cut off power to load and don't run anything else
	if (currentSenseState.isTooMuchCurrent) {
		Serial.println("OCP");

		turnOffAllFETs();
		
		firingState.targetPusherState = PUSHER_DRIVE_STATE_OFF;
    firingState.isFiring = false;

		return;
	}

	if (firingState.targetPusherState == PUSHER_DRIVE_STATE_OFF) {
    turnOffAllFETs();

  // Targetstate requires MOSFETs to change state that's at risk of shoot-
  // through
  // When transition FETs where the FETs are complementary, here's the sequence: 
	// BRAKE -> OFF -> [shoot-through delay] -> ON 
	// (or)
	// ON -> OFF -> [shoot-through delay] -> BRAKE
  					// Nake sure pusher state changed
  } else if (firingState.currentPusherState != firingState.targetPusherState
  	&& !fetState.hasComplementaryTransitionBegun
  	// Make sure target pusher state results in a complementary FET state 
  	&& (firingState.targetPusherState == PUSHER_DRIVE_STATE_ON
  	|| firingState.targetPusherState == PUSHER_DRIVE_STATE_BRAKE)) {

   	turnOffAllFETs();

   	// Set flag to indicate that transition of FETs to a complementary state has
   	// begun
   	fetState.hasComplementaryTransitionBegun = true;

   	// Transition FETs to match their target state afte SHOOT_THROUGH_DELAY
   	fetState.shootThroughProtectionTimer
   		.in(fetState.SHOOT_THROUGH_DELAY, handleComplementaryFETTransition);
  }
}

// Actually drives FETs in complementary switching
bool handleComplementaryFETTransition() {
	if (firingState.targetPusherState == PUSHER_DRIVE_STATE_ON) {
		setFETsForOn();	
	} else if (firingState.targetPusherState == PUSHER_DRIVE_STATE_BRAKE) {
		setFETsForBraking();	
	}

	// Reset flag, complementary transition complete
 	fetState.hasComplementaryTransitionBegun = false;

 	// Target pusher state has been achieved
	firingState.currentPusherState = firingState.targetPusherState;

	return true;		//required by timer lib
}

void turnOffAllFETs() {
	transitionHighSideFET(OFF);
	transitionLowSideFET(OFF);	

	firingState.currentPusherState = PUSHER_DRIVE_STATE_OFF;
}

void setFETsForBraking() {
	transitionHighSideFET(ON);
	transitionLowSideFET(OFF);	
}

void setFETsForOn() {
	transitionHighSideFET(OFF);
	transitionLowSideFET(ON);
}

// Abstracts away digitalWrites, helps make code more readable
void transitionHighSideFET(bool toTurnOn) {
	// transitioning high side FET can be confusing since the digitalWrite() must
	// be inverted
	digitalWrite(HIGH_SIDE_PIN, !toTurnOn);
}

// Abstracts away digitalWrites, helps make code more readable
void transitionLowSideFET(bool toTurnOn) {
	digitalWrite(LOW_SIDE_PIN, toTurnOn);
}

void handleTriggerPull() {
  trgSw.read();

  firingState.isTriggerPressed = trgSw.isPressed();
  firingState.wasTriggerPulled = trgSw.wasPressed();

}

void handleCycleControl() {
  cycCtrlSw.read();
  firingState.isCycCtrlSwPressed = cycCtrlSw.isPressed();

  // Only count shots through cycle control if it's in pusher motor motde
  if (cycCtrlSw.wasPressed() 
  	&& firingState.pusherMechanism == PUSHER_MOTOR_FIRING_MECHANISM) {

  	Serial.println("Pressed");

    firingState.dartsFired++;
    // Serial.println(firingState.dartsFired);
  }
}

void handleFiring() {
	if (firingState.wasTriggerPulled) {
    const uint8_t ROT_SW_POS = getRotSwPos(ROT_SW_PINS, NUM_OF_ROT_SW_PINS);
		setCurrentFiremode(ROT_SW_POS);
		// printCurrentFireMode();

    setPusherMechanism();
    setRateOfFire();
    setDartsToFire();

    firingState.isFiring = true;

  }

  // If pusher in firing sequence
  if (firingState.isFiring) {
  	if (firingState.pusherMechanism == PUSHER_MOTOR_FIRING_MECHANISM) {
			handlePusherMotorFiring();
    } else if (firingState.pusherMechanism == SOLENOID_FIRING_MECHANISM) {
    	handleSolenoidFiring();
    }
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

void setCurrentFiremode(uint8_t ROT_SW_POS) {
  if (ROT_SW_POS == ROT_SW_SAFETY_PIN) {
    firingState.currentFireMode = SAFETY;
  } else if (ROT_SW_POS == ROT_SW_SEMI_AUTO_PIN) {
    firingState.currentFireMode = SEMI_AUTO;
  } else if (ROT_SW_POS == ROT_SW_BURST_FIRE_PIN) {
    firingState.currentFireMode = BURST_FIRE;
  } else if (ROT_SW_POS == ROT_SW_FULL_AUTO_PIN) {
    firingState.currentFireMode = FULL_AUTO;
  }
}

// Print fire modes over serial 
void printCurrentFireMode() {
  if (firingState.currentFireMode == SAFETY) {
      Serial.println("Safety");
    } else if (firingState.currentFireMode == SEMI_AUTO) {
      Serial.println("Single Shot");
    } else if (firingState.currentFireMode == BURST_FIRE) {
      Serial.println("Burst Fire");
    } else if (firingState.currentFireMode == FULL_AUTO) {
      Serial.println("Full Auto");
    }
}

// Sets pusher mechanism based on reading jumper pin (D11)
// If high (no jumper cap on), pusher mech is pusher
// If low (jumper cap on), pusher mech is solenoid
void setPusherMechanism() {
	if (digitalRead(PUSHER_MECHANISM_SELECT_PIN)) {
		firingState.pusherMechanism = PUSHER_MOTOR_FIRING_MECHANISM;
	} else {
		firingState.pusherMechanism = SOLENOID_FIRING_MECHANISM;
	}
}

void setRateOfFire() {
	firingState.rateOfFire = map(analogRead(POT_PIN), 0, 1024, 1, 100);
}

void setDartsToFire() {
  if (firingState.currentFireMode == SEMI_AUTO) {
    firingState.dartsToFire = 1;
  } else if (firingState.currentFireMode == BURST_FIRE) {
    firingState.dartsToFire = BURST_FIRE_LENGTH;
  }

  firingState.dartsFired = 0;
}

void handlePusherMotorFiring() {
	if (firingState.currentFireMode == SEMI_AUTO 
		|| firingState.currentFireMode == BURST_FIRE) {
    handlePusherMotorSemiAutoAndBurst();
  } else if (firingState.currentFireMode == FULL_AUTO) {
    handlePusherMotorFullAuto();

  // Firemode is safety	
  } else { 
    firingState.targetPusherState = PUSHER_DRIVE_STATE_BRAKE;
    firingState.isFiring = false;
  }
}

void handlePusherMotorSemiAutoAndBurst() {
  // If no more darts to fire, stop solenoid
  if (firingState.dartsFired >= firingState.dartsToFire) {
    firingState.targetPusherState = PUSHER_DRIVE_STATE_BRAKE;
    firingState.dartsFired = 0;
    firingState.isFiring = false;

    Serial.println("stopped");

  // Still more darts to fire, so keep firing pusher
  } else {
  	firingState.targetPusherState = PUSHER_DRIVE_STATE_ON;

    Serial.println("going");

  }
}

void handlePusherMotorFullAuto() {
	// Trigger is let go and pusher is retracted, so turn off pusher
  if (!firingState.isTriggerPressed && firingState.isCycCtrlSwPressed) {
    firingState.targetPusherState = PUSHER_DRIVE_STATE_BRAKE;
    firingState.isFiring = false;

    Serial.println("stopped");


  // Trigger still pressed, so keep firing pusher
  } else {
  	firingState.targetPusherState = PUSHER_DRIVE_STATE_ON;

  	    Serial.println("going");

  }
}

void handleSolenoidFiring() {
	// Right when trigger was pulled, initiate firing sequence
	if (firingState.wasTriggerPulled) {
		// Set solenoid on and off times based on rate of fire
		updateSolenoidTiming();

		// Turn solenoid on, set it to turn off later. Solenoid shouldn't be 
		// turned on in safety
		if (firingState.currentFireMode != SAFETY) {
			firingState.targetPusherState = PUSHER_DRIVE_STATE_ON;
			firingState.firingTimer
				.in(firingState.solenoidOnTime, turnSolenoidOff);

		// Firemode is safety
		} else {
			firingState.targetPusherState = PUSHER_DRIVE_STATE_BRAKE;
			firingState.isFiring = false;
		}
	}

}

// Sets solenoid timing based on rate of fire and duty cycle
void updateSolenoidTiming() {
	firingState.solenoidOnTime = (100.0 / firingState.rateOfFire) 
		* (firingState.SOLENOID_DUTY_CYCLE / 2.0);

	firingState.solenoidOffTime = firingState.solenoidOnTime 
		/ (firingState.SOLENOID_DUTY_CYCLE/(100.0/2.0));

	// Serial.print("On time "); Serial.println(firingState.solenoidOnTime);
	// Serial.print("Off time "); Serial.println(firingState.solenoidOffTime);
}

// Turns solenoid off and continues firing loop. Don't call this if you just 
// want to turn off the solenoid
bool turnSolenoidOff() {
	// Serial.println("Off");
	firingState.targetPusherState = PUSHER_DRIVE_STATE_OFF;


	firingState.firingTimer
		.in(firingState.solenoidOffTime, handleSolenoidTuronOn);


	return true;
}

// Turns solenoid on and continues firing loop. Don't call this if you just 
// want to turn on the solenoid
bool turnSolenoidOn() {
	// Serial.println("On");
	firingState.targetPusherState = PUSHER_DRIVE_STATE_ON;

	firingState.firingTimer
 		.in(firingState.solenoidOnTime, turnSolenoidOff);


	return true;
}

// Called after solenoid turns off and about to turn back on. Ideally, this is 
// executed when plunger in retraced position. Determines whether to continue 
// firing the solenoid or to turn the solenoid off
bool handleSolenoidTuronOn() {
	firingState.dartsFired++;

	printCurrentFireMode();

	// Update timing in case rate of fire changed
	updateSolenoidTiming();

	if (firingState.currentFireMode == SEMI_AUTO 
		|| firingState.currentFireMode == BURST_FIRE) {
    handleSolenoidSemiAutoAndBurst();
  } else if (firingState.currentFireMode == FULL_AUTO) {
    handleSolenoidFullAuto();
  } else {
		firingState.targetPusherState = PUSHER_DRIVE_STATE_BRAKE;
	}

	return true;
}

void handleSolenoidSemiAutoAndBurst() {
  // If no more darts to fire, stop solenoid
  if (firingState.dartsFired >= firingState.dartsToFire) {
    firingState.targetPusherState = PUSHER_DRIVE_STATE_BRAKE;
    firingState.dartsFired = 0;
    firingState.isFiring = false;

  // Still more darts to fire, so keep firing pusher and continue firing loop
  } else {
  	turnSolenoidOn();
  }
}

void handleSolenoidFullAuto() {
	// Trigger is let go, so turn off solenoid
  if (!firingState.isTriggerPressed) {
    firingState.targetPusherState = PUSHER_DRIVE_STATE_BRAKE;
    firingState.isFiring = false;

  // Trigger still pressed, so keep firing pusher and continue firing loop
  } else {
  	turnSolenoidOn();
  }
}

float analogReadingToVoltage() {
	return (analogRead(CURRENT_SENSE_PIN) * ARDUINO_SUPPLY_VOLTAGE)/1024.0;
}

float voltageToCurrent(float voltage) {
	return voltage/SENSE_RESISTANCE;	// Basic Ohm's law
}

bool monitorCurrent() {
 	currentSenseState.instantaneousCurrentDraw = voltageToCurrent(analogReadingToVoltage());

	updateCurrentSenseValues(currentSenseState.instantaneousCurrentDraw);

	currentSenseState.instantaneousChangeInCurrent = differentiateCurrentSenseValues();

	// Increment ocpThresholdHitCounter if necessary
	if (currentSenseState.instantaneousCurrentDraw < currentSenseState.OCP_CURRENT_LIMIT) {
		resetOCPCounters();
	} else {
		currentSenseState.ocpThresholdHitCounter++;
	}

	// Set tooMuchCurrent flag if too much current
	if (currentSenseState.ocpThresholdHitCounter 
		> currentSenseState.MAX_OCP_SAMPLES) {
		currentSenseState.isTooMuchCurrent = true;
	}


	// Serial.print("OCP counters: "); Serial.println(currentSenseState.ocpThresholdHitCounter);
	// Serial.print("I: "); Serial.println(currentSenseState.instantaneousCurrentDraw);
  	

	// printCurrentSenseValues(currentSenseState.instantaneousCurrentDraw);


	// Must return tru because this function is executed through a timer function
	return true;
}

void updateCurrentSenseValues(float current) {
	// Update time and current arrays with new sample values
	currentSenseState.sampleTimeBuffer.push(millis()/1000.0);
	currentSenseState.currentValues.push(current);
}

float differentiateCurrentSenseValues() {
	// Only differentiate if current arr full
	if (currentSenseState.currentValues.isFull()
		&& currentSenseState.sampleTimeBuffer.isFull()) {

		float totaldI = 0;
		float totaldt = 0;

		// Calculations start at second value because the previous value is needed
		// to find slope. The first value doesn't have a previous value, so the 
		// calculations won't work
		for (int i = 1; i < currentSenseState.currentValues.size(); i++) {
			totaldI += currentSenseState.currentValues[i] 
				- currentSenseState.currentValues[i - 1];

			totaldt += currentSenseState.sampleTimeBuffer[i] 
				- currentSenseState.sampleTimeBuffer[i - 1];

		}

		// printCurrentSenseDifferentiationBuffer();

		return totaldI/totaldt;


	}

	return -1;
}

void printCurrentSenseDifferentiationBuffer() {
	if (currentSenseState.currentValues.isFull()) {
		for (int i = 0; i < currentSenseState.currentValues.size(); i++) {
			Serial.print(i); Serial.print(": "); 
			Serial.print(currentSenseState.sampleTimeBuffer[i]);
			Serial.print(", ");
			Serial.print(currentSenseState.currentValues[i]);
			Serial.println("A ");
		}
	} else {
		Serial.print("Not full yet! Here's the current length: ");
		Serial.println(currentSenseState.currentValues.size());
	}
}

void printCurrentSenseValues(float current) {
	Serial.print("I [A]: "); Serial.println(current);

	// Print instantaneous change in current if the value is valid
	if (currentSenseState.instantaneousChangeInCurrent > -1) {
		Serial.print("dI/dt [A/s]: "); 
		Serial.println(currentSenseState.instantaneousChangeInCurrent);
	}

	// Print threshold counter
	// if (currentSenseState.OCP_CURRENT_LIMIT != 0) {
	// 	Serial.print("ocpThresholdHitCounter: "); 
	// 	Serial.println(currentSenseState.ocpThresholdHitCounter);
	// }

	Serial.println("\n");
}

void resetOCPCounters() {
	currentSenseState.ocpThresholdHitCounter = 0;
	currentSenseState.isTooMuchCurrent = false;
}
