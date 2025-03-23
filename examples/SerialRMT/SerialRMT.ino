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

  //clear Serial1 receive buffer
  while(Serial1.available()) Serial1.read();
}

int cnt = 0;  
int txerr = 0;
int rxerr = 0;

void loop() {
  char txbuf[100];
  char rxbuf[100];
  int txlen;
  int rxlen;

  cnt++;

  //RMT_TX --> Serial1_RX
  txlen = sprintf(txbuf, "RMT_TX-%d-abcdefghijklmnopqrstuvwxyz+1234567890+", cnt);
  SerialRMT0.write(txbuf, txlen);
  delay(100); //delay to get message transmitted (can remove this but then reception will not be in sync of course)
  rxlen = Serial1.read(rxbuf, sizeof(rxbuf)-1);
  rxbuf[rxlen] = 0;
  if(strcmp(txbuf, rxbuf) != 0) txerr++;
  Serial.printf("RMT_TX(%2d) -> Ser1_RX(%2d): txerr=%d txlen=%d ", RMT_TX_PIN, SER_RX_PIN, txerr, txlen);
  Serial.write(rxbuf,rxlen);
  Serial.println();

  if(SerialRMT::rxCount() > 0) {
    //Serial1_TX --> RMT_RX
    txlen = sprintf(txbuf, "RMT_RX-%d-ABCDEFGHIJKLMNOPQRSTUVWXYZ+1234567890+", cnt);
    Serial1.write(txbuf, txlen);
    delay(100); //delay to get message transmitted (can remove this but then reception will not be in sync of course)
    rxlen = SerialRMT0.read((uint8_t*)rxbuf, sizeof(rxbuf)-1);
    rxbuf[rxlen] = 0;
    if(strcmp(txbuf, rxbuf) != 0) rxerr++;
    Serial.printf("RMT_RX(%2d) <- Ser1_TX(%2d): rxerr=%d rxlen=%d ", RMT_RX_PIN, SER_TX_PIN, rxerr, rxlen);
    Serial.write(rxbuf, rxlen);
    Serial.println();
  }

}
