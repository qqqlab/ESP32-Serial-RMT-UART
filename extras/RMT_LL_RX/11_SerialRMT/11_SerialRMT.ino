#include "ESP32_SerialRMT.h"

SerialRMT rmt;



void setup() {
  Serial.begin(115200);
  rmt.begin(115200, 17, -1); //baud, rxPin, txPin
  Serial1.begin(115200,SERIAL_8N1,-1,16);
}

void loop() {
 Serial1.print("ABCDEFGHIJKLMNOPQRSTUVWXYZ");
 delay(100);
 uint8_t rbuf[100];
 int len = rmt.read(rbuf,sizeof(rbuf));
 Serial.printf("rx len=%d ",len);
 //for(int i=0;i<len;i++) Serial.printf("%02X'%c' ",rbuf[i],rbuf[i]);
 Serial.write(rbuf,len);
 Serial.println();
 
}


/*
void setup() {
  Serial.begin(115200);
  rmt.begin(115200, 17, 16); //baud, rxPin, txPin  
}

void loop() {
 rmt.print("ABCDEFGHIJKLMNOPQRSTUVWXYZ");
 delay(100);
 uint8_t rbuf[100];
 int len = rmt.read(rbuf,sizeof(rbuf));
 Serial.printf("rx len=%d ",len);
 //for(int i=0;i<len;i++) Serial.printf("%02X'%c' ",rbuf[i],rbuf[i]);
 Serial.write(rbuf,len);
 Serial.println();
 
}
*/