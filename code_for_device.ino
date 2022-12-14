/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// NAME:     Holly Hammons, Charles Nissen, Trevor Olsen, Ty Smith, and Abi Thompson

// GROUP:    Backup Assist for Power Wheelchairs

// TITLE:    Ultrasonic Sensor Testing Code (Adafruit)

// FUNCTION: This code measures the distance from an ultrasonic sensor and dispays a green, yellow, or red LED, depending on the distance measured.
//           A sound will appear at a specific frequency, based on the distance measured.

// HARDWARE: Ultrasonic Sensor    ->      Arduino or Redboard
//           VCC                          PIN 5V
//           GND                          PIN GND
//           ECHO                         PIN 10
//           TRIG                         PIN 11

//           CIRCUIT              ->      Arduino or Redboard
//           ALL GND LEDS                 250ohm RESISISTORs TO GND
//           GREEN LED (+)                PIN 2
//           YELLOW LED (+)               PIN 3
//           RED LED (+)                  PIN 4
//           POTENTIOMETER(LEFT PIN)      GND PIN
//           POTENTIOMETER(MIDDLE PIN)    125ohm RESISTOR TO SPEAKER (+)
//           POTENTIOMETER(RIGHT PIN)     PIN 9
//           SPEAKER (-)                  GND PIN

// SOFTWARE: None

// EXECUTE: To use the code, make all connections to an arduino or redboard as stated above. 
//          If desired, the serial monitor can be used to display distances.

// RESOURCE: This code is based off of two resources listed below:
//           https://www.makerguides.com/hc-sr04-arduino-tutorial/

// PURPOSE: This code is used for testing the HC-SR04 and Y401 Adafruit ultrasonic sensors. If chosen, this code can be used as final code
//          for the design.
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// define constants
#define TRIGGER_PIN 11
#define ECHO_PIN 10

// don't connect speaker pin to pin 13, it will chirp at start up due to the LED flashing
#define SPEAKER_PIN 9

//#define RED_LED 7
//#define YELLOW_LED 6
//#define GREEN_LED 5

#define RED_LED 4
#define YELLOW_LED 3
#define GREEN_LED 2

// distance ranges
int upperLimit = 60;
int lowerLimit = 40;

// speaker frequencies
int yellowFrequency = 730;
int redFrequency = 750;

// main loop timing
long previousMillis = 0;
float interval = 500;

long previousMillisTrigger = 0;
long previousMillisTrigger2 = 0;

// for the sensor calculations
long duration;
float distance;

float current_dist;


void setup() {
  // Define inputs and outputs:
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(RED_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);

  pinMode(SPEAKER_PIN, OUTPUT);

  digitalWrite(SPEAKER_PIN, LOW);

  // Begin Serial communication at a baudrate of 9600 for debugging:
  Serial.begin(9600);

  // small delay at startup to allow everything to calm down
  delay(50);
}

void loop() {
  // get current time
  unsigned long currentMillis = millis();

  // run main loop every "interval" in ms
  if (currentMillis - previousMillis > interval) {
    // set previous time to current time
    previousMillis = currentMillis;

    // get reading from sensor and average it
    current_dist = distanceInAdafruit();

    // GREEN LED
    if (current_dist > upperLimit) {
      digitalWrite(GREEN_LED, HIGH);
      digitalWrite(YELLOW_LED, LOW);
      digitalWrite(RED_LED, LOW);
      // turn off all tones
      noTone(SPEAKER_PIN);
    }
    // YELLOW LED
    if (current_dist >= lowerLimit && current_dist <= upperLimit) {
      digitalWrite(GREEN_LED, LOW);
      digitalWrite(YELLOW_LED, HIGH);
      digitalWrite(RED_LED, LOW);
      // middle frequency tone
      tone(SPEAKER_PIN, yellowFrequency);
    }
    // RED LED
    if (current_dist < lowerLimit) {
      digitalWrite(GREEN_LED, LOW);
      digitalWrite(YELLOW_LED, LOW);
      digitalWrite(RED_LED, HIGH);
      // high frequency tone
      tone(SPEAKER_PIN, redFrequency);
    }
  }
}

float distanceInAdafruit() {
  // Clear the TRIGGER_PIN by setting it LOW:
  digitalWrite(TRIGGER_PIN, LOW);

  // get current time
  unsigned long currentMillisTrigger = millis();

  // wait 5us before setting trigger pin high
  if (currentMillisTrigger - previousMillisTrigger > 0.005) {
    // set previous time to current time
    previousMillisTrigger = currentMillisTrigger;

    // Trigger the sensor
    digitalWrite(TRIGGER_PIN, HIGH);

    // get current time
    unsigned long currentMillisTrigger2 = millis();

    // wait 10us before making trigger pin low
    if (currentMillisTrigger2 - previousMillisTrigger2 > 0.01) {
      // set previous time to current time
      previousMillisTrigger2 = currentMillisTrigger2;

      digitalWrite(TRIGGER_PIN, LOW);
    }
  }
  // Read the ECHO_PIN, pulseIn() returns the duration (length of the pulse) in microseconds:
  duration = pulseIn(ECHO_PIN, HIGH);

  // Calculate the distance: distance = length of pulse (us) * speed of sound (us) * 1/2 (sound waves traveling to and from object)
  distance = duration * 0.034 / 2.0;

  Serial.print("distance is: ");
    Serial.print(distance);
    Serial.println(" ft");

  return distance;
}
