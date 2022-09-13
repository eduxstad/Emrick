#include <FastLED.h>

#define DATA_PIN    2
//#define CLK_PIN   4
#define LED_TYPE    WS2813
#define COLOR_ORDER GRB
#define NUM_LEDS    45

CRGB leds[NUM_LEDS];

void setup(){
  FastLED.addLeds<WS2811, DATA_PIN, GRB>(leds, NUM_LEDS);

}
//void loop(){
//
//  // Let's take 256 steps to get from blue to red
//  // (the most possible with an LED with 8 bit RGB values)
//
//  for( int colorStep=0; colorStep<256; colorStep++ ) {
//
//      int r = colorStep;  // Redness starts at zero and goes up to full
//      for(int i1 = 0; i1 < 256; i1++)
//      {
//        int b = i1;  // Blue starts at full and goes down to zero
//        for(int i2 = 0; i2 < 256; i2++)
//        {
//          int g = i2;              // No green needed to go from blue to red
//    
//          // Now loop though each of the LEDs and set each one to the current color
//    
//          for(int x = 0; x < NUM_LEDS; x++){
//              leds[x] = CRGB(r,g,b);
//          }
//    
//    
//    
//          // Display the colors we just set on the actual LEDs
//          FastLED.show();
//    
//          //delay(10); 
//        }
//      }
//  }
//
//}

void loop(){

//start from red
  for( int colorStep=0; colorStep <= 255; colorStep++ ) {

  int r = 255;
  int g = 0;
  int b = colorStep;

  // Now loop though each of the LEDs and set each one to the current color
  for(int x = 0; x < NUM_LEDS; x++){
      leds[x] = CRGB(r,g,b);
  }

  // Display the colors we just set on the actual LEDs
      delay(10); 
      FastLED.show();
      }

    //into blue
      for( int colorStep=255; colorStep >= 0; colorStep-- ) {

      int r = colorStep;
      int g = 0;
      int b = 255;

      // Now loop though each of the LEDs and set each one to the current color
      for(int x = 0; x < NUM_LEDS; x++){
          leds[x] = CRGB(r,g,b);
      }

      // Display the colors we just set on the actual LEDs
  delay(100); 
  FastLED.show();
  }

//start from blue
  for( int colorStep=0; colorStep <= 255; colorStep++ ) {

      int r = 0;
      int g = colorStep;
      int b = 255; 

      // Now loop though each of the LEDs and set each one to the current color
      for(int x = 0; x < NUM_LEDS; x++){
          leds[x] = CRGB(r,g,b);
      }

      // Display the colors we just set on the actual LEDs
  delay(100); 
  FastLED.show();
  }

//into green
  for( int colorStep=255; colorStep >= 0; colorStep-- ) {

      int r = 0;
      int g = 255;
      int b = colorStep; 

      // Now loop though each of the LEDs and set each one to the current color
      for(int x = 0; x < NUM_LEDS; x++){
          leds[x] = CRGB(r,g,b);
      }

      // Display the colors we just set on the actual LEDs
  delay(100); 
  FastLED.show();
  }

//start from green
  for( int colorStep=0; colorStep <= 255; colorStep++ ) {

      int r = colorStep;
      int g = 255;
      int b = 0;

      // Now loop though each of the LEDs and set each one to the current color
      for(int x = 0; x < NUM_LEDS; x++){
          leds[x] = CRGB(r,g,b);
      }

      // Display the colors we just set on the actual LEDs
  delay(100);
  FastLED.show();
  }

//into yellow
  for( int colorStep=255; colorStep >= 0; colorStep-- ) {

      int r = 255;
      int g = colorStep;
      int b = 0;

      // Now loop though each of the LEDs and set each one to the current color
      for(int x = 0; x < NUM_LEDS; x++){
          leds[x] = CRGB(r,g,b);
      }

      // Display the colors we just set on the actual LEDs
  delay(100); 
  FastLED.show();
  }


} //end main loop
