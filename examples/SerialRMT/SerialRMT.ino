#include <ESP32_SerialRMT.h>

#define BAUD 1000000

#define RMT_TX_PIN 15
#define RMT_RX_PIN 17
#define SER_TX_PIN 16
#define SER_RX_PIN 7

SerialRMT SerialRMT0;

void setup() {
  Serial.begin(115200);
  while(!Serial);

  Serial.printf("\n");
  Serial.printf("-------------------------------------------------------------\n");
  Serial.printf("SerialRMT Demo\n");
  Serial.printf("-------------------------------------------------------------\n");
  Serial.printf("Chip              : " CONFIG_IDF_TARGET "\n");
  Serial.printf("SerialRMT TX Ports: %d\n", SerialRMT::txCount());
  Serial.printf("SerialRMT RX Ports: %d\n", SerialRMT::rxCount());
  Serial.printf("Baud              : %d\n", BAUD);
  Serial.printf("-------------------------------------------------------------\n");
  Serial.printf("Connect GPIO %2d to GPIO %2d    SerialRMT0 TX --> Serial1 RX\n", RMT_TX_PIN, SER_RX_PIN );

  if(SerialRMT::rxCount() > 0) {
    //If chip supports RMT_RX then setup SerialRMT0(RX and TX) and Serial1(RX and TX)    
    Serial.printf("    and GPIO %2d to GPIO %2d    SerialRMT0 RX <-- Serial1 TX\n", RMT_RX_PIN, SER_TX_PIN);
    Serial1.begin(BAUD, SERIAL_8N1, SER_RX_PIN, SER_TX_PIN);
    SerialRMT0.begin(BAUD, RMT_RX_PIN, RMT_TX_PIN); //baud, rxPin, txPin
  }else{
    //If chip does not support RMT_RX then setup SerialRMT0(TX only) and Serial1(RX only)
    Serial1.begin(BAUD, SERIAL_8N1, SER_RX_PIN, -1);
    SerialRMT0.begin(BAUD, -1, RMT_TX_PIN); //baud, rxPin, txPin
  }
  Serial.printf("-------------------------------------------------------------\n");
}

int cnt = 0;

void loop() {
  uint8_t buf[100];
  int len;
  cnt++;

  //RMT_TX --> Serial1_RX
  SerialRMT0.printf("RMT_TX-%d-abcdefghijklmnopqrstuvwxyz+1234567890+",cnt);
  delay(100); //delay to get message transmitted (can remove this but then reception will not be in sync of course)
  len = Serial1.read(buf, sizeof(buf));
  Serial.printf(" 0x%02X ",buf[0]);
  Serial.printf("RMT_TX(%2d) -> Ser1_RX(%2d): len=%d ", RMT_TX_PIN, SER_RX_PIN, len);
  Serial.write(buf,len);
  Serial.println();

  if(SerialRMT::rxCount() > 0) {
    //Serial1_TX --> RMT_RX
    Serial1.printf("RMT_RX-%d-ABCDEFGHIJKLMNOPQRSTUVWXYZ+1234567890+",cnt);
    delay(100); //delay to get message transmitted (can remove this but then reception will not be in sync of course)
    len = SerialRMT0.read(buf, sizeof(buf));
    Serial.printf(" 0x%02X ",buf[0]);    
    Serial.printf("RMT_RX(%2d) <- Ser1_TX(%2d): len=%d ", RMT_RX_PIN, SER_TX_PIN, len);
    Serial.write(buf,len);
    Serial.println();
  }
}
