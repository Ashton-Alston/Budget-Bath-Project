#define CUSTOM_SETTINGS
#define INCLUDE_TERMINAL_MODULE

#include <Dabble.h>

// Pins for SoftwareSerial, managed by Dabble
int rxPin = 5;  // RX Pin
int txPin = 4;  // TX Pin

const int pinSolenoidHot = 12;
const int pinSolenoidCold = 7;

// Flow sensor pins
int flowSensorHotPin = 3;
int flowSensorColdPin = 2;

// Pulse volume constants in liters
const float pulseVolumeHot = 0.0028;
const float pulseVolumeCold = 0.00295;

volatile int pulseCountHot = 0;
volatile int pulseCountCold = 0;

unsigned long totalVolumeHot = 0;  // in milliliters
unsigned long totalVolumeCold = 0; // in milliliters

unsigned long requiredVolumeHot = 0;   // Required volume in milliliters
unsigned long requiredVolumeCold = 0;  // Required volume in milliliters

unsigned long totalHotWaterAvailable = 6000; // Total hot water available in milliliters (9 liters)

bool solenoidsOpened = false;

void setup() {
  Serial.begin(9600);
  Dabble.begin(9600, rxPin, txPin);

  pinMode(pinSolenoidHot, OUTPUT);
  pinMode(pinSolenoidCold, OUTPUT);
  pinMode(flowSensorHotPin, INPUT);
  pinMode(flowSensorColdPin, INPUT);

  digitalWrite(pinSolenoidHot, LOW); 
  digitalWrite(pinSolenoidCold, LOW); 

  attachInterrupt(digitalPinToInterrupt(flowSensorHotPin), countPulseHot, RISING);
  attachInterrupt(digitalPinToInterrupt(flowSensorColdPin), countPulseCold, RISING);

  Serial.println("System initialized. Solenoids closed.");
  Terminal.println("Enter Desired Bath Temperature in Fahrenheit:");
}

void loop() {
  Dabble.processInput();

  if (Terminal.available()) {
    String inputString = Terminal.readString();
    inputString.trim();
    float desiredTempF = inputString.toFloat();
    float desiredTempC = (desiredTempF - 32) * 5.0 / 9.0;

    if (desiredTempF > 0 && !solenoidsOpened) {
      double hotTempC, coldTempC;
      readWaterTemperatures(hotTempC, coldTempC);
      calculateWaterVolumes(desiredTempC, hotTempC, coldTempC);
      digitalWrite(pinSolenoidHot, HIGH);
      digitalWrite(pinSolenoidCold, HIGH);
      solenoidsOpened = true;
    }
  }

  updateWaterVolumes();

  delay(1000);  // Delay for 1 second before the next loop iteration
}

void countPulseHot() {
  pulseCountHot++;
}

void countPulseCold() {
  pulseCountCold++;
}

void readWaterTemperatures(double& hotTempC, double& coldTempC) {
  int hotTempVal = analogRead(A4);
  hotTempC = (0.2078 * hotTempVal) + 6.284;
  
  int coldTempVal = analogRead(A5);
  coldTempC = (0.2028 * coldTempVal) + 6.2588;
}

void calculateWaterVolumes(float desiredTempC, double hotTempC, double coldTempC) {
  float totalVolume = 6;  // Total desired volume in liters
  float adjustmentFactor = 1.05;  // Adjust by 5% to compensate for heat loss or sensor inaccuracies
  requiredVolumeHot = ((totalVolume * (desiredTempC - coldTempC) / (hotTempC - coldTempC)) * adjustmentFactor) * 1000;  // Convert to milliliters
  requiredVolumeCold = (totalVolume - (requiredVolumeHot / 1000)) * 1000;  // Convert to milliliters

  Serial.print("Required hot water volume with adjustment: ");
  Serial.print(requiredVolumeHot);
  Serial.println(" mL");
  Serial.print("Required cold water volume with adjustment: ");
  Serial.print(requiredVolumeCold);
  Serial.println(" mL");
}

void updateWaterVolumes() {
  unsigned long volumeThisIntervalHot = pulseCountHot * pulseVolumeHot * 1000;
  unsigned long volumeThisIntervalCold = pulseCountCold * pulseVolumeCold * 1000;

  if (digitalRead(pinSolenoidHot) == HIGH) {  // Check if hot solenoid is still open
    totalVolumeHot += volumeThisIntervalHot;
    Serial.print("Hot water interval volume: ");
    Serial.print(volumeThisIntervalHot);
    Serial.println(" mL");
    Serial.print("Total hot water volume: ");
    Serial.print(totalVolumeHot);
    Serial.println(" mL");

    if (totalVolumeHot >= requiredVolumeHot) {
      digitalWrite(pinSolenoidHot, LOW);
      Serial.println("Hot water solenoid closed.");
      totalHotWaterAvailable -= requiredVolumeHot;  // Subtract only once when closing
      Serial.print("Remaining hot water available: ");
      Serial.print(totalHotWaterAvailable / 1000.0);  // Display in liters
      Serial.println(" liters");
    }
  }

  if (digitalRead(pinSolenoidCold) == HIGH) {  // Check if cold solenoid is still open
    totalVolumeCold += volumeThisIntervalCold;
    Serial.print("Cold water interval volume: ");
    Serial.print(volumeThisIntervalCold);
    Serial.println(" mL");
    Serial.print("Total cold water volume: ");
    Serial.print(totalVolumeCold);
    Serial.println(" mL");

    if (totalVolumeCold >= requiredVolumeCold) {
      digitalWrite(pinSolenoidCold, LOW);
      Serial.println("Cold water solenoid closed.");
    }
  }

  pulseCountHot = 0;  // Reset the hot pulse count after reporting
  pulseCountCold = 0;  // Reset the cold pulse count after reporting
}

