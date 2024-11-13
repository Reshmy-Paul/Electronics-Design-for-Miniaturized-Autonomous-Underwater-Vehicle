/*
  ReadAnalogVoltage

  Reads an analog input on pin 0, converts it to voltage, and prints the result to the Serial Monitor.
  Graphical representation is available using Serial Plotter (Tools > Serial Plotter menu).
  Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.

  This example code is in the public domain.

  https://www.arduino.cc/en/Tutorial/BuiltInExamples/ReadAnalogVoltage
*/

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
}

// the loop routine runs over and over again forever:
void loop() {
  analogReadResolution(12);
  // read the input on analog pin 0:
  float sensorValue = analogRead(A0);
  // Convert the analog reading (which goes from 0 - 4095) to a voltage (0 - 3.3V):
  float voltage = sensorValue * (3.3 / 4095.0);
  // print out the value you read:
  Serial.println(voltage);
  delay(100);
}