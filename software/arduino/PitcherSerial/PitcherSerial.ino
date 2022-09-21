// rf69 demo tx rx.pde
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messageing client
// with the RH_RF69 class. RH_RF69 class does not provide for addressing or
// reliability, so you should only use RH_RF69  if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example rf69_server.
// Demonstrates the use of AES encryption, setting the frequency and modem 
// configuration

#include <SPI.h>
#include <RH_RF69.h>

/************ Radio Setup ***************/

// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 915.0
#define RFM69_CS      8
#define RFM69_INT     7
#define RFM69_RST     4
#define LED           LED_BUILTIN

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

int16_t packetnum = 0;  // packet counter, we increment per xmission
const int buttonPin = 12;
const int txLEDPin = 0;
const int rxLEDPin = 1;
int buttonState = 0;

void setup() 
{
  Serial.begin(115200);
  while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to computer
   
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);
  // power led
  pinMode(txLEDPin, OUTPUT);
  pinMode(rxLEDPin, OUTPUT);

  digitalWrite(txLEDPin, LOW);
  digitalWrite(rxLEDPin, LOW);

  Serial.println("Feather RFM69 TX Test!");
  Serial.println();

  // manual reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);
  
  if (!rf69.init()) {
    Serial.println("RFM69 radio init failed");
    while (1);
  }
  Serial.println("RFM69 radio init OK!");
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
  // No encryption
  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("setFrequency failed");
  }

  // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
  // ishighpowermodule flag set like this:
  rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

  // The encryption key has to be the same as the one in the server
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  rf69.setEncryptionKey(key);
  
  pinMode(LED, OUTPUT);

  Serial.print("RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");
}

int incomingByte = 0;

void loop() {
  //delay(1000);  // Wait 1 second between transmits, could also 'sleep' here!

  uint8_t data[1];
  //Serial.print("Sending "); Serial.println(radiopacket);

  if (Serial.available() > 0) {
    // read the incoming bytes:
    while (Serial.available() > 0) {
      incomingByte = Serial.read();
    }
    // say what you got:
    //Serial.print("I received: ");
    //Serial.println(incomingByte, DEC);
    Serial.print("Switching color patterns . . . ");
    //always send a high to switch colors
    data[0] = 1;
    rf69.send((uint8_t *)data, 1);
    rf69.waitPacketSent();
    Serial.print("Switch command sent! \n");
  }
  /*// Send button state!
  buttonState = digitalRead(buttonPin);
  if (buttonState == HIGH) {
    data[0] = 1;
    digitalWrite(txLEDPin, HIGH);
    digitalWrite(LED, HIGH);
  } else {
    digitalWrite(LED, LOW);
    data[0] = 0;
  }
  rf69.send((uint8_t *)data, 1);
  rf69.waitPacketSent();*/
}
