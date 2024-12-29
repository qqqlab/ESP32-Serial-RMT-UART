#include <ESP32_SerialRMT.h>

int baud = 115200;

SerialRMT RMT;

void setup() {
  Serial.begin(115200);
  while(!Serial);

  Serial.println("Testing SerialRMT UART transmitter/receiver");
  Serial.println("Connect pin 16 (SerialRMT TX) to pin 17 (Serial1 RX");
  Serial.println("Connect pin 15 (SerialRMT RX) to pin 7 (Serial1 TX");  
  Serial.println("...");
  delay(1000);

  Serial1.begin(baud, SERIAL_8N1, 17, 7); //baud, mode, rxPin, txPin
  RMT.begin(baud, 15, 16); //baud, rxPin, txPin

  //ESP32 and ESP32-S2 only have RMT transmitters, the above code will crash. Use following lines instead (and connect pin 16 to pin 17)
  //Serial1.begin(baud, SERIAL_8N1, 17, -1); //baud, mode, rxPin, txPin
  //RMT.begin(baud, -1, 16); //baud, rxPin, txPin  
}

uint8_t buf[100];
int cnt = 0;
int len;

void loop() {
  cnt++;

  //send from RMT to Serial1
  //RMT.printf("%d:ABCDEFGHIJKLMNOPQRSTUVWXYZ.", cnt);
  //RMT.print("ABCDEFGHIJKLMNOPQRSTUVWXYZ.");
  char s[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ.";
  RMT.write(s);
  delay(10); //give some time for message to transmit
  len = Serial1.read(buf, sizeof(buf));
  Serial.printf("RMT(tx) >> Serial1 len=%d data=", len);
  Serial.write(buf,len);
  Serial.println();

  //send from Serial1 to RMT
  Serial1.printf("%d:ABCDEFGHIJKLMNOPQRSTUVWXYZ.", cnt);
  delay(10); //give some time for message to transmit
  len = RMT.read(buf, sizeof(buf));
  Serial.printf("Serial1 >> RMT(rx) len=%d data=", len);
  Serial.write(buf,len);
  Serial.println();

  delay(1000);
}
