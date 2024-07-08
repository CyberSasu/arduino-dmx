#include <Arduino.h>
#include <AccelStepper.h>
#include <Bounce2.h>
#include <esp_dmx.h>

// Define stepper motor pins
#define STEP_PIN 2
#define DIR_PIN 3
#define EN_PIN 21

// Define AccelStepper
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// Define LED pin
#define LED_PIN 22

// Define DIP switch pins
const int dipPins[8] = {18, 19, 23, 33, 25, 26, 27, 14};
Bounce dipSwitches[8];

// Define limit switch pin
#define LIMIT_SWITCH_PIN 19

// DMX settings
#define DMX_CHANNEL_DIRECTION 1
#define DMX_CHANNEL_SPEED 2
#define DMX_PORT 1
#define DMX_RX_PIN 16
#define DMX_TX_PIN 17
#define DMX_RE_DE_PIN 4

// Variables to keep track of previous DMX direction value
int previousDmxDirectionValue = 0;

// DMX data buffer
byte dmxData[DMX_PACKET_SIZE];

// DMX connection status
bool dmxIsConnected = false;
unsigned long lastUpdate = millis();

void setup() {
  // Initialize Serial Monitor for debugging
  Serial.begin(115200);

  // Initialize stepper motor driver
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW); // Enable the driver

  // Initialize DMX
  dmx_config_t config = DMX_CONFIG_DEFAULT;
  dmx_personality_t personalities[] = {
    {1, "Default Personality"}
  };
  int personality_count = 1;
  dmx_driver_install(DMX_PORT, &config, personalities, personality_count);
  dmx_set_pin(DMX_PORT, DMX_TX_PIN, DMX_RX_PIN, DMX_RE_DE_PIN);

  // Initialize stepper motor
  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(500);

  // Initialize LEDs
  pinMode(LED_PIN, OUTPUT);

  // Initialize DIP switches
  for (int i = 0; i < 8; i++) {
    pinMode(dipPins[i], INPUT_PULLUP);
    dipSwitches[i].attach(dipPins[i]);
    dipSwitches[i].interval(5);
  }

  // Initialize limit switch
  pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);

  // Perform homing sequence
  homeStepperMotor();
}

void loop() {
  // Update DIP switch states
  for (int i = 0; i < 8; i++) {
    dipSwitches[i].update();
  }

  // DMX packet to store information
  dmx_packet_t packet;

  // Receive DMX data
  if (dmx_receive(DMX_PORT, &packet, DMX_TIMEOUT_TICK)) {
    unsigned long now = millis();

    // Check for DMX errors
    if (!packet.err) {
      if (!dmxIsConnected) {
        Serial.println("DMX is connected!");
        dmxIsConnected = true;
      }

      dmx_read(DMX_PORT, dmxData, packet.size);

      if (now - lastUpdate > 1000) {
        Serial.printf("Start code is 0x%02X and slot 1 is 0x%02X\n", dmxData[0], dmxData[1]);
        lastUpdate = now;
      }

      // Read DMX values for direction and speed
      int currentDmxDirectionValue = dmxData[DMX_CHANNEL_DIRECTION];
      int dmxSpeedValue = dmxData[DMX_CHANNEL_SPEED];

      // Determine direction based on DMX value change
      if (currentDmxDirectionValue > previousDmxDirectionValue) {
        digitalWrite(DIR_PIN, HIGH); // Forward
      } else if (currentDmxDirectionValue < previousDmxDirectionValue) {
        digitalWrite(DIR_PIN, LOW); // Reverse
      }

      // Update previous DMX direction value
      previousDmxDirectionValue = currentDmxDirectionValue;

      // Determine speed
      int speed = map(dmxSpeedValue, 0, 255, 0, 1000);
      stepper.setSpeed(speed);

      // Control stepper motor based on DMX value
      stepper.runSpeed();

      // Control LED based on DMX speed value
      analogWrite(LED_PIN, dmxSpeedValue);
    } else {
      Serial.println("A DMX error occurred.");
    }
  } else if (dmxIsConnected) {
    Serial.println("DMX was disconnected.");
    dmx_driver_delete(DMX_PORT);

    // Stop the program
    while (true) yield();
  }

  // Print DIP switch states
  for (int i = 0; i < 8; i++) {
    Serial.print("DIP ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(dipSwitches[i].read());
  }

  delay(10);
}

void homeStepperMotor() {
  // Move the motor until the limit switch is triggered
  stepper.setSpeed(-200); // Set speed in the direction towards the limit switch

  while (digitalRead(LIMIT_SWITCH_PIN) == HIGH) {
    stepper.runSpeed();
  }

  // Stop the motor
  stepper.setCurrentPosition(0); // Set the current position as home (0)
  stepper.setSpeed(0); // Stop the motor
}
