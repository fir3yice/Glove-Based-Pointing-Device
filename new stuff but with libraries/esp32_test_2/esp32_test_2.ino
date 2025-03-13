#include "BluetoothSerial.h"

//#define USE_NAME  // Comment this to use MAC address instead of a slaveName

//and other pins

// Check if Bluetooth is available
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

// Check Serial Port Profile
#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Port Profile for Bluetooth is not available or not enabled. It is only available for the ESP32 chip.
#endif
BluetoothSerial SerialBT;

#ifdef USE_NAME
String slaveName = "BT_SLAVE";  // Change this to reflect the real name of your slave BT device
#else
String MACadd = "00:23:00:00:31:A5";                        // This only for printing
uint8_t address[6] = {0x00, 0x23, 0x00, 0x00, 0x31, 0xA5};  // Change this to reflect real MAC address of your slave BT device
#endif

String myName = "ESP32-BT-Master";

const int irPins[] = {27, 26, 25, 33};
const int numSensors = sizeof(irPins)/sizeof(irPins[0]);

void setup() {
  bool connected;
  Serial.begin(115200);
  const char *pin = "1234";
  SerialBT.setPin(pin, 4);
  SerialBT.begin(myName, true);
  Serial.printf("The device \"%s\" started in master mode, make sure slave BT device is on!\n", myName.c_str());
  
#ifdef USE_NAME
  //connected = SerialBT.connect(slaveName);
  connected = SerialBT.connect(address, 1, ESP_SPP_SEC_ENCRYPT|ESP_SPP_SEC_AUTHENTICATE, ESP_SPP_ROLE_MASTER);
  Serial.printf("Connecting to slave BT device named \"%s\"\n", slaveName.c_str());
#else
  //connected = SerialBT.connect(address, 1, ESP_SPP_SEC_ENCRYPT|ESP_SPP_SEC_AUTHENTICATE, ESP_SPP_ROLE_MASTER);
  connected = SerialBT.connect(address);
  Serial.print("Connecting to slave BT device with MAC ");
  Serial.println(MACadd);
#endif

  if (connected) {
    Serial.println("Connected Successfully!");
  } else {
    //while (!SerialBT.connected(10000)) {
      Serial.println("Failed to connect. Make sure remote device is available and in range, then restart app.");
    //}
  }
//  if (SerialBT.disconnect())
//  SerialBT.connect();
//  if (connected) {}

//  for(int i=0; i<numSensors;i++){
//    pinMode(irPins[i], INPUT);
//    Serial.print(irPins[i]);
//    Serial.println(" started");
//  }
}

boolean hasIR = false;
int currIR = -1;
float signalIR = 0.0;
const float threshold = 3403;
int mypin = 32;
void loop() {

  for (int i = 0; i < numSensors; i++) {
    mypin = irPins[i];
    signalIR = analogRead(mypin);
    if(signalIR < threshold){
    //check for beams
      hasIR=true;
      currIR = mypin;
      Serial.println(currIR);
      SerialBT.println((int)currIR); // Send as integer explicitly
      delay(1000);
      break;
    }
  }

   //beam broken
  if (currIR != -1 && analogRead(currIR) >= threshold && hasIR) {
    hasIR = false;
    currIR = -1;
    SerialBT.println(currIR);
  }

  if(!hasIR){
    delay(50);
  }
  
  delay(50);
}
