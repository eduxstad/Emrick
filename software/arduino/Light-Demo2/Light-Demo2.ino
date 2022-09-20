// rf69 demo tx rx.pde
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messageing client
// with the RH_RF69 class. RH_RF69 class does not provide for addressing or
// reliability, so you should only use RH_RF69  if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example rf69_server.
// Demonstrates the use of AES encryption, setting the frequency and modem 
// configuration
#include <FastLED.h>

FASTLED_USING_NAMESPACE

#include <SPI.h>
#include <RH_RF69.h>

/************ Radio Setup ***************/

// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 915.0
#define RFM69_CS      8
#define RFM69_INT     7
#define RFM69_RST     4
#define LED           LED_BUILTIN

#define DATA_PIN    18
//#define CLK_PIN   4
#define LED_TYPE    WS2813
#define COLOR_ORDER GRB
#define NUM_LEDS    45
CRGB leds[NUM_LEDS];

#define BRIGHTNESS          96
#define FRAMES_PER_SECOND  120

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

int16_t packetnum = 0;  // packet counter, we increment per xmission
const int txLEDPin = 0;
const int rxLEDPin = 1;
int buttonState = 0;

CRGBPalette16 gPal;
bool gReverseDirection = false;

void setup() 
{
  delay(3000); // 3 second delay for recovery
  Serial.begin(115200);

  FastLED.addLeds<LED_TYPE,DATA_PIN,COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);

  FastLED.setBrightness(BRIGHTNESS);

  pinMode(LED, OUTPUT);     
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

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

#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))

// List of patterns to cycle through.  Each is defined as a separate function below.
typedef void (*SimplePatternList[])();
SimplePatternList gPatterns = {white, red, green, rainbow, Glitter, confetti, juggle, Rain, Fire2012};
uint8_t gCurrentPatternNumber = 0; // Index number of which pattern is current
uint8_t gHue = 0; // rotating "base color" used by many of the patterns

void loop() {
  gPatterns[gCurrentPatternNumber]();

  // send the 'leds' array out to the actual LED strip
  FastLED.show();  
  // Now wait for a reply
  uint8_t reply[1];
  uint8_t len = 1;

  if (rf69.waitAvailableTimeout(25))  { 
    // Should be a reply message for us now   
    if (rf69.recv(reply, &len)) {
      Serial.print("Received packet with data: ");
      Serial.print(reply[0]);
      Serial.println();
      if (reply[0]) {
        digitalWrite(LED, HIGH);
        digitalWrite(rxLEDPin, HIGH);
        nextPattern();
        if(rxLEDPin == HIGH)
        {
          digitalWrite(rxLEDPin, LOW);
        }
      } else {
        digitalWrite(LED, LOW);
        digitalWrite(rxLEDPin, LOW);
      }
    }
  }
  EVERY_N_MILLISECONDS( 20 ) { gHue++; }
}

void nextPattern()
{
  // add one to the current pattern number, and wrap around at the end
  gCurrentPatternNumber = (gCurrentPatternNumber + 1) % ARRAY_SIZE( gPatterns);
}

void rainbow() 
{
  // FastLED's built-in rainbow generator
  fill_rainbow( leds, NUM_LEDS, gHue, 7);
}

void Glitter() 
{
  // built-in FastLED rainbow, plus some random sparkly glitter
  //rainbow();
  fadeToBlackBy( leds, NUM_LEDS, 20);
  addGlitter(160);
}

void addGlitter( fract8 chanceOfGlitter) 
{
  if( random8() < chanceOfGlitter) {
    leds[ random16(NUM_LEDS) ] += CRGB::White;
  }
}

void confetti() 
{
  // random colored speckles that blink in and fade smoothly
  fadeToBlackBy( leds, NUM_LEDS, 10);
  int pos = random16(NUM_LEDS);
  leds[pos] += CHSV( gHue + random8(64), 200, 255);
}

void juggle() {
  // eight colored dots, weaving in and out of sync with each other
  fadeToBlackBy( leds, NUM_LEDS, 20);
  uint8_t dothue = 0;
  for( int i = 0; i < 8; i++) {
    leds[beatsin16( i+7, 0, NUM_LEDS-1 )] |= CHSV(dothue, 200, 255);
    dothue += 32;
  }
}

void red() {
  // eight colored dots, weaving in and out of sync with each other
  fill_solid( leds, NUM_LEDS, CRGB::Red);
}

void green() {
  // eight colored dots, weaving in and out of sync with each other
  fill_solid( leds, NUM_LEDS, CRGB::Green);
}

void white() {
  // eight colored dots, weaving in and out of sync with each other
  fill_solid( leds, NUM_LEDS, CRGB::White);
}

#define COOLING  55
#define SPARKING 120


void Fire2012()
{
// Array of temperature readings at each simulation cell
  static uint8_t heat[NUM_LEDS];

  // Step 1.  Cool down every cell a little
    for( int i = 0; i < NUM_LEDS; i++) {
      heat[i] = qsub8( heat[i],  random8(0, ((COOLING * 10) / NUM_LEDS) + 2));
    }
  
    // Step 2.  Heat from each cell drifts 'up' and diffuses a little
    for( int k= NUM_LEDS - 1; k >= 2; k--) {
      heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2] ) / 3;
    }
    
    // Step 3.  Randomly ignite new 'sparks' of heat near the bottom
    if( random8() < SPARKING ) {
      int y = random8(7);
      heat[y] = qadd8( heat[y], random8(160,255) );
    }

    // Step 4.  Map from heat cells to LED colors
    for( int j = 0; j < NUM_LEDS; j++) {
      CRGB color = HeatColor( heat[j]);
      int pixelnumber;
      if( gReverseDirection ) {
        pixelnumber = (NUM_LEDS-1) - j;
      } else {
        pixelnumber = j;
      }
      leds[pixelnumber] = color;
    }
}

void Rain() 
{  

  CRGB ColorBackground = CRGB(0x00,0x00,0x00);
  CRGB ColorMeteor = CRGB(0x00,0x00,0xff);
  byte meteorSize = 3;
  byte meteorTrailDecay = 64;
  boolean meteorRandomDecay = true;
  
  // set background color
  fill_solid( leds, NUM_LEDS, CRGB(0x00,0x00,0x10));

  for(int i = 0; i < NUM_LEDS+NUM_LEDS; i++) 
  {
    // fade color to background color for all LEDs
    for(int j=0; j < NUM_LEDS; j++) {
      if( (!meteorRandomDecay) || (random(10) > 5) ) {
        leds[j] = fadeTowardColor(leds[j], ColorBackground, meteorTrailDecay ); 
      }
    }

    // draw meteor
    for(int j = 0; j < meteorSize; j++) {
      if( ( i-j < NUM_LEDS) && (i-j >= 0) ) {
        leds[i-j]= ColorMeteor;
      }
    }
   
    FastLED.show();
    FastLED.delay(30);
  }
}

// Functions from Kriegsman example
CRGB fadeTowardColor( CRGB& cur, const CRGB& target, uint8_t amount)
{
  nblendU8TowardU8( cur.red,   target.red,   amount);
  nblendU8TowardU8( cur.green, target.green, amount);
  nblendU8TowardU8( cur.blue,  target.blue,  amount);
  return cur;
}

void nblendU8TowardU8( uint8_t& cur, const uint8_t target, uint8_t amount)
{
  if( cur == target) return;
  
  if( cur < target ) {
    uint8_t delta = target - cur;
    delta = scale8_video( delta, amount);
    cur += delta;
  } else {
    uint8_t delta = cur - target;
    delta = scale8_video( delta, amount);
    cur -= delta;
  }
}
