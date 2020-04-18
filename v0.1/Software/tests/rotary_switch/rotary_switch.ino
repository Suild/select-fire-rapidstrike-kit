#define ROT_SW_SAFETY_PIN 6
#define ROT_SW_SEMI_AUTO_PIN 7
#define ROT_SW_BURST_FIRE_PIN 8
#define ROT_SW_FULL_AUTO_PIN 9

#define NUM_OF_ROT_SW_POS 4
// Array of all rotary switch pins so it's easier to iterate through all of them
const uint8_t ROT_SW_PINS[] = {ROT_SW_SAFETY_PIN, ROT_SW_SEMI_AUTO_PIN, ROT_SW_BURST_FIRE_PIN, ROT_SW_FULL_AUTO_PIN};

void setup() {
	Serial.begin(9600);

	// Set all rotary switch pins to inputs with pullupd
	for (int i = 0; i < sizeof(ROT_SW_PINS)/sizeof(ROT_SW_PINS[0]); i++) {
		pinMode(ROT_SW_PINS[i], INPUT_PULLUP);
	}

}

void loop() {
	// Check position of rotary swtich and print out the corresponding fire mode
	const uint8_t NUM_OF_ROT_SW_PINS = sizeof(ROT_SW_PINS)/sizeof(ROT_SW_PINS[0]);
	const uint8_t ROT_SW_POS = getRotSwPos(ROT_SW_PINS, NUM_OF_ROT_SW_PINS);

	if (ROT_SW_POS == ROT_SW_SAFETY_PIN) {
		Serial.println("Safety");
	} else if (ROT_SW_POS == ROT_SW_SEMI_AUTO_PIN) {
		Serial.println("single shot");
	} else if (ROT_SW_POS == ROT_SW_BURST_FIRE_PIN) {
		Serial.println("Burst fire");
	} else if (ROT_SW_POS == ROT_SW_FULL_AUTO_PIN) {
		Serial.println("Full auto");
	}

	Serial.println(ROT_SW_POS);

}

// Returns position of rotary switch in terms of which pin is connected to COM (pin is low)
uint8_t getRotSwPos(uint8_t rotSwPins[], uint8_t NUM_OF_ROT_SW_PINS) {
	for (int rotSwPin = 0; rotSwPin < NUM_OF_ROT_SW_PINS; rotSwPin++) {
		if (digitalRead(rotSwPins[rotSwPin]) == LOW) {
			return rotSwPins[rotSwPin];
		}
	}

}