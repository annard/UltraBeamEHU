/*
 Defines pin numbers for motor controller
 */
const int DIRECTION_PIN = 5;
const int STEPPER_PIN = 4;
const int ENABLE_PIN = 6;

/*
Interrupts for direction and enabling the engine.
*/
const int CHANGE_DIRECTION_PIN = 2;
const int CHANGE_ENABLE_PIN = 3;

/*
State for interrupt management
*/
const unsigned long DEBOUNCE_DELAY = 50; // Delay for debouncing in milliseconds
const boolean DIR_OUT = true;
const boolean DIR_IN = !DIR_OUT;
volatile boolean dirState = DIR_OUT;
volatile boolean enableState = false;

/*
Measuring Current Using ACS712
*/
const int ANALOG_IN = A0;
const int MILLI_VOLT_PER_AMP = 185;  // use 185 for 5A Module, 100 for 20A Module and 66 for 30A Module
const int ACS_OFFSET = 1640;

// Debugging
const int LED_PIN = 13;
volatile int changeCount = 0;

volatile unsigned long lastDebounceTime1 = 0; // Last time the output pin was toggled
volatile unsigned long lastDebounceTime2 = 0; // Last time the output pin was toggled

void setup() {
  interrupts();
  digitalWrite(ENABLE_PIN, LOW);

  Serial.begin(9600);
  pinMode (LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Sets the two pins as Outputs
  pinMode(STEPPER_PIN, OUTPUT);
  pinMode(DIRECTION_PIN, OUTPUT);

  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);

  digitalWrite(CHANGE_DIRECTION_PIN, HIGH);
  attachInterrupt(digitalPinToInterrupt(CHANGE_DIRECTION_PIN), setDirection, FALLING);

  digitalWrite(CHANGE_ENABLE_PIN, HIGH);
  attachInterrupt(digitalPinToInterrupt(CHANGE_ENABLE_PIN), setEnabled, FALLING);

  printStatus();
}

void loop() {
  if (enableState) {
    readAmpSensor(ANALOG_IN);
    if (dirState == DIR_IN) {
      Serial.println("Antenna going in");
      antennaIn();
    }
    else {
      Serial.println("Antenna going out");
      antennaOut();
    }
    readAmpSensor(ANALOG_IN);
  }
  else {
    digitalWrite(ENABLE_PIN, LOW);
  }
  delay(1000);  // One second delay
}

void setEnabled() {
    unsigned long currentTime = millis();
    if (currentTime - lastDebounceTime1 > DEBOUNCE_DELAY) {
        lastDebounceTime1 = currentTime; // Update the last debounce time
        digitalWrite(LED_PIN, !enableState);
        enableState = !enableState;
    }
}

void setDirection() {
      unsigned long currentTime = millis();
    if (currentTime - lastDebounceTime2 > DEBOUNCE_DELAY) {
        lastDebounceTime2 = currentTime; // Update the last debounce time

        dirState = !dirState;
    }
}

void antennaOut() {
  digitalWrite(ENABLE_PIN, HIGH);
  digitalWrite(DIRECTION_PIN, HIGH);  // Enables the motor to move in a particular direction
  // Makes 200 pulses for making one full cycle rotation
  for (int x = 0; x < 1600; x++) {
    digitalWrite(STEPPER_PIN, LOW);
    delayMicroseconds(500);
    digitalWrite(STEPPER_PIN, HIGH);
    delayMicroseconds(500);
  }
  digitalWrite(ENABLE_PIN, LOW);
}

void antennaIn() {
  digitalWrite(ENABLE_PIN, HIGH);
  digitalWrite(DIRECTION_PIN, LOW);  //Changes the rotations direction
  // Makes 200 pulses for making one full cycle rotation
  for (int x = 0; x < 1600; x++) {
    digitalWrite(STEPPER_PIN, HIGH);
    delayMicroseconds(500);
    digitalWrite(STEPPER_PIN, LOW);
    delayMicroseconds(500);
  }
  digitalWrite(ENABLE_PIN, LOW);
}

double readAmpSensor(int sensorPin) {
  int rawValue = analogRead(sensorPin);
  double voltage = (rawValue / 1023.0) * 5000;  // Gets you mV
  double amps = ((voltage - ACS_OFFSET) / MILLI_VOLT_PER_AMP);

  Serial.print("Raw Value = ");  // shows pre-scaled value
  Serial.print(rawValue);
  Serial.print("t mV = ");    // shows the voltage measured
  Serial.print(voltage, 3);   // the ‘3’ after voltage allows you to display 3 digits after decimal point
  Serial.print("t Amps = ");  // shows the voltage measured
  Serial.println(amps, 3);    // the ‘3’ after voltage allows you to display 3 digits after decimal point
  return amps;
}

void printStatus() {
  Serial.print("Enabled: ");
  Serial.println((enableState) ? "On" : "Off");
  Serial.print("Direction: ");
  Serial.println((dirState == DIR_IN) ? "In" : "Out");
}