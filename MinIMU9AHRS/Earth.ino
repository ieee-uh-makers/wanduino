// call with Water(40,120,20);

void Earth(int Cooling, int Sparking, int SpeedDelay) {
  static byte heat[NUM_LEDS];
  int cooldown;
  
  // Step 1.  Cool down every cell a little
  for( int i = 0; i < NUM_LEDS; i++) {
    cooldown = random(20, ((Cooling * 10) / NUM_LEDS) + 2);
    
    if(cooldown>heat[i]) {
      heat[i]=0;
    } else {
      heat[i]=heat[i]-cooldown;
    }
  }
  
  // Step 2.  Heat from each cell drifts 'up' and diffuses a little
  for( int k= NUM_LEDS - 1; k >= 2; k--) {
    heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2]) / 3;
  }
    
  // Step 3.  Randomly ignite new 'sparks' near the bottom
  if( random(255) < Sparking ) {
    int y = random(7);
   heat[y] = heat[y] + random(160,255);
   // heat[y] = random(160,255);
  }

  // Step 4.  Convert heat to LED colors
  for( int j = 0; j < NUM_LEDS; j++) {
    setPixelEarthColor(j, heat[j] );
  }

  showStrip();
  delay(SpeedDelay);
}

void setPixelEarthColor (int Pixel, byte temperature) {
  // Scale 'heat' down from 0-255 to 0-191
  byte t192 = round((temperature/255.0)*191);
 
  // calculate ramp up from
  byte heatramp = t192 & 0x3F; // 0..63
  heatramp <<= 2; // scale up to 0..252
 
  // figure out which third of the spectrum we're in:
  if( t192 > 0x80) {                     // hottest
    setPixel(Pixel, heatramp/(NUM_LEDS*2), 255, heatramp/NUM_LEDS);
    //setPixel(Pixel, 255, heatramp/NUM_LEDS, heatramp/(NUM_LEDS*2));//heatramp, heatramp);
  } else if( t192 > 0x40 ) {             // middle
    setPixel(Pixel, 0, 255, heatramp*2/NUM_LEDS);
    //setPixel(Pixel, 255, heatramp*2/NUM_LEDS, 0);
  } else {                               // coolest
    setPixel(Pixel, 0, heatramp + 90, heatramp/NUM_LEDS);
  }
}
