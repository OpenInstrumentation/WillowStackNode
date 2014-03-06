#include <Adafruit_CC3000.h>
#include <ccspi.h>
#include <SPI.h>

// CC3000 configuration.
#define ADAFRUIT_CC3000_IRQ    3
#define ADAFRUIT_CC3000_VBAT   5
#define ADAFRUIT_CC3000_CS     10

// Wifi network configuration.
#define WLAN_SSID "WillowNet"
#define WLAN_PASS ""
// Security can be WLAN_SEC_UNSEC, WLAN_SEC_WEP, WLAN_SEC_WPA or WLAN_SEC_WPA2
#define WLAN_SECURITY   WLAN_SEC_UNSEC
#include <math.h>
#define SERVER_IP              192, 168, 1, 100    // Logging server IP address.  Note that this
                                               // should be separated with commas and NOT periods!
#define SERVER_PORT            8000                // Logging server listening port.

// Internal state used by the sketch.
Adafruit_CC3000 cc3000 = Adafruit_CC3000(ADAFRUIT_CC3000_CS, ADAFRUIT_CC3000_IRQ, ADAFRUIT_CC3000_VBAT);
uint32_t ip;

const int NodeID=1;
const double phidgConst=0.9765625;  // Phidgets conversion constant
const int Q1pin=A0;  // Anemometer at top of stack (ModernDevice Wind Sensor)
const double Q1Area=0.00116;  // Area of venturi tube in m^2
const int P0pin=A1;  // Absolute pressure sensor (Phidgets 1115)
const int dPpin=A2;  // Differential pressure sensor (Phidgets 1126)
const int T6pin=A3;  // T half of T/RH sensor at position 6 (Phidgets 1125)
const int RH6pin=A4;  // RH half of T/RH sensor at position 6 (Phidgets 1125)
const int T5pin=A5;  // T half of T/RH sensor at position 5 (Phidgets 1125)
const int RH5pin=A6;  // RH half of T/RH sensor at position 5 (Phidgets 1125)
const int T4pin=A7;  // T half of T/RH sensor at position 4 (Phidgets 1125)
const int RH4pin=A8;  // RH half of T/RH sensor at position 4 (Phidgets 1125)
const int T3pin=A9;  // T half of T/RH sensor at position 3 (Phidgets 1125)
const int RH3pin=A10;  // RH half of T/RH sensor at position 3 (Phidgets 1125)
const int T2pin=A11;  // T half of T/RH sensor at position 2 (Phidgets 1125)
const int RH2pin=A12;  // RH half of T/RH sensor at position 2 (Phidgets 1125)
const int T1pin=A13;  // T half of T/RH sensor at position 1 (Phidgets 1125)
const int RH1pin=A14;  // RH half of T/RH sensor at position 1 (Phidgets 1125)
const int Q0pin=A15;  // Anemometer at bottom of stack (ModernDevice Wind Sensor)
const double Q0Area=0.0016;  // Area of venturi tube in m^2
double Q1;  // Volumetric flow at top of stack in m^3/sec
double P0;  // Absolute pressure at top of stack in kPa
double dP;  // Differential pressure between top and bottom of stack in kPa
double T6;  // Temperature at position 6 (top) in °C
double RH6;  // Relative humidity at position 6 in %RH
double T5;  // Temperature at position 5 in °C
double RH5;  // Relative humidity at position 5 in %RH
double T4;  // Temperature at position 4 in °C
double RH4;  // Relative humidity at position 4 in %RH
double T3;  // Temperature at position 3 in °C
double RH3;  // Relative humidity at position 3 in %RH
double T2;  // Temperature at position 2 in °C
double RH2;  // Relative humidity at position 2 in %RH
double T1;  // Temperature at position 1 (bottom) in °C
double RH1;  // Relative humidity at position 1 in %RH
double Q0;  // Volumetric flow at bottom of stack in m^3/sec

void setup() {
  Serial.begin(9600);
  
  // Initialize the CC3000.
  Serial.println(F("\nInitializing CC3000..."));
  cc3000.begin();
  cc3000.connectToAP(WLAN_SSID, WLAN_PASS, WLAN_SECURITY);
  while (!cc3000.checkDHCP()){
    delay(100);
  }
  // Store the IP of the server.
  ip = cc3000.IP2U32(SERVER_IP);
  Serial.println(F("Setup complete."));
}

void loop() {
  String serOut="";                // String to hold output data
  Q1=analogRead(Q1pin);  // Read in flow speed at top of stack
  Q1=map(Q1, 0, 1023, 0, 27*Q1Area);  // Convert linear to volumetric flow
  P0=(analogRead(P0pin)*phidgConst)/4+10;  // Read in absolute pressure
  dP=(analogRead(dPpin)*phidgConst)/18-27.7777;  // Read in differential pressure
  T6=(analogRead(T6pin)*phidgConst)*0.22222-61.11;  // Read in temperature at position 6
  RH6=(analogRead(RH6pin)*phidgConst)*0.1906-40.2;  // Read in relative humidity at position 6
  T5=(analogRead(T5pin)*phidgConst)*0.22222-61.11;  // Read in temperature at position 5
  RH5=(analogRead(RH5pin)*phidgConst)*0.1906-40.2;  // Read in relative humidity at position 5
  T4=(analogRead(T4pin)*phidgConst)*0.22222-61.11;  // Read in temperature at position 4
  RH4=(analogRead(RH4pin)*phidgConst)*0.1906-40.2;  // Read in relative humidity at position 4
  T3=(analogRead(T3pin)*phidgConst)*0.22222-61.11;  // Read in temperature at position 3
  RH3=(analogRead(RH3pin)*phidgConst)*0.1906-40.2;  // Read in relative humidity at position 3
  T2=(analogRead(T2pin)*phidgConst)*0.22222-61.11;  // Read in temperature at position 2
  RH2=(analogRead(RH2pin)*phidgConst)*0.1906-40.2;  // Read in relative humidity at position 2
  T1=(analogRead(T1pin)*phidgConst)*0.22222-61.11;  // Read in temperature at position 1
  RH1=(analogRead(RH1pin)*phidgConst)*0.1906-40.2;  // Read in relative humidity at position 1
  Q0=analogRead(Q0pin);  // Read in flow speed at bottom of stack
  Q0=map(Q0, 0, 1023, 0, 27*Q0Area);  // Convert linear to volumetric flow
  char Q1char[5], P0char[5], dPchar[5], T6char[5], RH6char[5], T5char[5], RH5char[5], T4char[5], RH4char[5], T3char[5], RH3char[5], T2char[5], RH2char[5], T1char[5], RH1char[5], Q0char[5];  // Initialize character buffers for variables
  Adafruit_CC3000_Client server = cc3000.connectTCP(ip, SERVER_PORT);  
  serOut+="Node ";  // Send output:
  serOut+=NodeID;  // Node ID #
  serOut+=",";  // Comma for good measure
  serOut+=millis();  // Send the time
  serOut+=",";  // A comma as a separator
  serOut+=dtostrf(Q1, 2, 3, Q1char);  // Top flow rate in m^3/sec
  serOut+=",";  // Another comma as a separator
  serOut+=dtostrf(P0, 2, 3, P0char);  // Absolute pressure at top of stack in kPa
  serOut+=",";  // Another comma
  serOut+=dtostrf(dP, 2, 3, dPchar);  // Differential pressure in kPa between the top and bottom of the stack
  serOut+=",";  // Another comma
  serOut+=dtostrf(T6, 2, 3, T6char);  // Temperature at the top of the stack in °C
  serOut+=",";  // Another comma
  serOut+=dtostrf(RH6, 2, 3, RH6char);  // Relative humidity at the top of the stack in %RH
  serOut+=",";  // Another comma
  serOut+=dtostrf(T5, 2, 3, T5char);  // Temperature at position 5, 0.5 meters down in the stack
  serOut+=",";  // Another comma
  serOut+=dtostrf(RH5, 2, 3, RH5char);  // Relative humidity at position 5
  serOut+=",";  // Yet another comma
  serOut+=dtostrf(T4, 2, 3, T4char);  // Temperature at position 4, 1 meter down in the stack
  serOut+=",";  // Were you expecting something different right about now?
  serOut+=dtostrf(RH4, 2, 3, RH4char);  //Relative humidity at position 4
  serOut+=",";  // You should have seen this coming
  serOut+=dtostrf(T3, 2, 3, T3char);  // Temperature at position 3, 1.5 meters down in the stack
  serOut+=",";  // The next comma
  serOut+=dtostrf(RH3, 2, 3, RH3char);  // Relative humidity at position 3
  serOut+=",";  // Really?
  serOut+=dtostrf(T2, 2, 3, T2char);  // Temperature at position 2, 2 meters down in the stack
  serOut+=",";  // Going into a comma coma yet?
  serOut+=dtostrf(RH2, 2, 3, RH2char);  // Relative humidity at position 2
  serOut+=",";  // [No comment]
  serOut+=dtostrf(T1, 2, 3, T1char);  // Temperature at position 1 at the bottom of the stack
  serOut+=",";  // [No comma, either]
  serOut+=dtostrf(RH1, 2, 3, RH1char);  // Relative humidity at the bottom of the stack
  serOut+=",";  // Enough with the comma jokes
  serOut+=dtostrf(Q0, 2, 3, Q0char);  // Volumetric flow rate at the bottom of the stack
  Serial.println(serOut);  // Send data to the serial port
  server.println(serOut);  // Send data to the server
  while(millis()%1000==0){  // While it's been less than a second since the last transmission,
    delay(1);               // wait until it's been a second.
  }
}
