#include <ESP32_SerialRMT.h>

SerialRMT serialRMT[16];

void setup() {
  Serial.begin(115200);
  while(!Serial);

  Serial.println("This SerialRMT demo tries to open as many ports as possible.");
  Serial.println("It will crash when opening too many ports, but that is normal.");
  Serial.println("...");
  Serial.flush();
  delay(1000);

  for(int i=0; i<sizeof(serialRMT); i++) {
    Serial.printf("Trying to start SerialRMT number %d ... ", i+1);
    Serial.flush();
    delay(1000);    
    serialRMT[i].begin(115200,4,5); //rx,tx
    Serial.printf("OK\n");
  }
}

void loop() {}
