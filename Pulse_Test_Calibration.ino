// Pin setup for the water flow sensor
int flowSensorPin = 2;  // Digital pin connected to the water flow sensor
int solenoid = 7;

volatile int pulseCount = 0;  // Variable to store the pulse count

void setup() {
  pinMode(flowSensorPin, INPUT);  // Set the sensor pin as input
  pinMode(solenoid,OUTPUT);
  attachInterrupt(digitalPinToInterrupt(flowSensorPin), countPulse, RISING);  // Attach interrupt

  digitalWrite(solenoid,HIGH);
  Serial.begin(9600);  // Start serial communication at 9600 bps
  Serial.println("Starting pulse count... Pour 1 liter of water through the sensor.");
}

void loop() {
  // Print the current pulse count to the Serial Monitor
  Serial.print("Current Pulse Count: ");
  Serial.println(pulseCount);
  delay(1000);  // Update about every second
}
void countPulse() {
  pulseCount++;  // Increment the pulse count with each pulse detected
  
}

