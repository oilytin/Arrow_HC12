#include <FastLED.h>
#include <SoftwareSerial.h>
#include <AceButton.h>
#include <EEPROM.h>
using namespace ace_button;

/*

   ______         __         _      ___         
  / __/ ___ ____ / / __ __  | | /| / (____ ___ _
 / _// / _ `(_-</ _ / // /  | |/ |/ / / _ / _ `/
/_/ /_/\_,_/___/_//_\_, /   |__/|__/_/_//_\_, / 
                   /___/                 /___/  
 

  PWM hardware mod

  0 for standard
  D2	HC12 SET
  D3	LED DATA LEFT WING
  D4	LED CLOCK LEFT WING
  D5	LED DATA RIGHT WING
  D6	LED CLOCK RIGHT WING
  D7	SOFTSERIAL RX > HC12 TX
  D8	SOFTSERIAL TX > HC12 RX
  D9	BUTTON

  1 for PWM functions on pin2 (HC12 set moves to pin12)
  D2	PWM input
  D3	LED DATA LEFT WING
  D4	LED CLOCK LEFT WING
  D5	LED DATA RIGHT WING
  D6	LED CLOCK RIGHT WING
  D7	SOFTSERIAL RX > HC12 TX
  D8	SOFTSERIAL TX > HC12 RX
  D9	BUTTON
  D12 HC12 SET

  2 for HC12 on hardware serial (enables panic mode, requires hardware Mod 1)
  D2	PWM input
  D3	LED DATA LEFT WING
  D4	LED CLOCK LEFT WING
  D5	LED DATA RIGHT WING
  D6	LED CLOCK RIGHT WING
  RX0 HC12 TX
  TX1 HC12 RX
  D9	BUTTON
  D12 HC12 SET
*/
#define hwMod 2

#if hwMod > 0
// pwm input definitions
#define INPUT_PIN_0  2        // pwm input pin
#define INPUT_PIN_0_STEPS 7   // num positions to devide the signal into 
// e.g 2 for 2 position switch
// 3 for 3 position switch
// 'n' for slider
// ranhe is 0 - (INPUT_PIN_0_STEPS -1)

volatile int pwm_value = 0;
volatile int prev_time = 0;
uint8_t pwmByte = 0;
bool pwmRangeLatch = 0;
bool stickMove = 0;

// Serial defines

#define HC12_MODE_PIN   12
#else
#define HC12_MODE_PIN   2
#endif

#define HC12_RX_PIN     7
#define HC12_TX_PIN     8

// Button defines
#define BUTTON          9
//fast led defines
#define FPS             120     //LED display frames per second
#define navWidth        6       //width of nav beacons on wingtips (panic mode XD)
#define NUM_LEDS        84      //total leds
#define WING_LEDS       21      //per strip (2 wings, 2 strips per wing)
#define COOLING         60      //fire settings
#define SPARKING        90      //fire settings
#define blinkTime       500     //blink time in ms
#define strobeTime      1500    //strobe time in ms

// ID of the settings block
#define CONFIG_VERSION  "ls4"
// Tell it where to store your config data in EEPROM
#define CONFIG_START    32
// settings structure
struct StoreStruct {
  // This is for mere detection if they are your settings
  char version[4];
  // The variables of your settings
  // settings[0] is the wing id 1-8 (Red)
  // settings[1] is the mode, 1 = wireless control, 8 = TX control, all others are standalone patterns (Yellow)
  // settings[2] is the brightness 1-8 (Blue)
  // settings[3] is the PWM divisions/calibriation (Purple)
  uint8_t settings[4];
  // PWM value limits - these are re-calculated each time we use the arduino so the users needw to do a check cycle end to end so
  // it calculates the right limits.
  uint16_t pwmLowVal;
  uint16_t pwmHighVal;
  //PWM divisions lookup table based on settings[3]
  uint8_t pwmLut[9][3];
}
storage = {
  CONFIG_VERSION,
  // The default values
  {1, 1, 8, 3},
  1500,
  1500,
  {
    {0, 0, 0},
    {0, 0, 0},
    {0, 0, 0},
    {0, 0, 0},
    {0, 0, 0},
    {0, 0, 0},
    {0, 0, 0},
    {0, 0, 0},
    {0, 0, 0},
  }
};

//serial
#if hwMod <2
SoftwareSerial HC12(HC12_RX_PIN, HC12_TX_PIN);//rx,tx
#endif

// variables to hold the received serial data
const byte numBytes = 10; // 5 data bytes
byte hc12rec[numBytes];
byte tempBuffer[numBytes];
boolean allReceived = false;

//Previous data for momentary modes (blink & Strobe)
byte hc12recPrev[numBytes];

// timers for various stuff
long lastMillis = 0;
boolean buttonLatch = 0;
boolean blinkLatch = 0;
boolean strobeLatch = 0;

//fastled
CRGB leds[NUM_LEDS];
CRGB rgb (0, 0, 0);
CHSV hsv (0, 0, 0);
bool gReverseDirection = false;
uint8_t hue = 0;
uint8_t gHue = 0;

// strobe
bool isOn = false;
uint32_t lastStrobeMs = 0; // The previous light change time

// menu stuff
AceButton button(BUTTON);
uint8_t Menu = 0;

void setup() {
  delay(3000); // sanity delay

#if hwMod > 0
  pinMode(INPUT_PIN_0, INPUT_PULLUP);
  attachInterrupt(0, rising, RISING);
# endif

  //load config from EEPROM
  loadConfig();

  //serial

  pinMode(HC12_MODE_PIN, OUTPUT);                  // Output High for Transparent / Low for Command
  digitalWrite(HC12_MODE_PIN, HIGH);               // Enter Transparent mode
  delay(80);
#if hwMod >1
  Serial.begin(2400);
#else
  Serial.begin(9600);
  HC12.begin(2400);
#endif
  //fastled
  /*
    PIXEL REFERENCE

         0        83
         /        \
        / /41  42\ \
       / /        \ \
    20/ /          \ \63
       / 21       62\

  */
  FastLED.addLeds<SK9822, 3, 4, BGR>(leds, 0, NUM_LEDS / 2); // left leds
  FastLED.addLeds<SK9822, 5, 6, BGR>(leds, NUM_LEDS / 2, NUM_LEDS / 2); // right leds
  FastLED.setBrightness( storage.settings[2] * 32 - 1 ); //set max brightness to value stored in eeprom
  FastLED.setMaxPowerInVoltsAndMilliamps(5, 2000);
  set_max_power_indicator_LED(13);

  // AceButton
  pinMode(BUTTON, INPUT_PULLUP);
  ButtonConfig* buttonConfig = button.getButtonConfig();
  buttonConfig->setEventHandler(handleEvent);
  buttonConfig->setFeature(ButtonConfig::kFeatureLongPress);
  buttonConfig->setFeature(ButtonConfig::kFeatureSuppressAfterLongPress);
}

void loop() {
  button.check();
  recvBytesWithStartEndMarkers();
  EVERY_N_MILLIS(1000 / FPS) {
#if hwMod > 0
    // map pwm value to 8 bit value
    pwmByte = map(pwm_value, storage.pwmLowVal, storage.pwmHighVal, 3, 252);
    uint8_t select = 0;
    select = pwmSelect(pwmByte);
#endif
    gHue++; // slowly cycle the "base color" through the rainbow
    if (buttonLatch == 1) {
      menuDisplay();
    }
    else {
      switch (storage.settings[1]) {
        case 1:
#if hwMod >1
          if (select < 1)navLight();    //panic mode :)
          else {
#endif
            switch (hc12rec[1]) {
              case 0:           //solid colour
                fillSolid(hc12rec[2], hc12rec[3], hc12rec[4]);
                break;
              case 1:           //Fire
                Fire2012L(); // run simulation frame
                Fire2012R();
                break;
              case 2:           //Pride
                pride();
                break;
              case 3:         //blink
                blinko(hc12rec[2], hc12rec[3], hc12rec[4]);
                break;
              case 4:
                rainbowWithGlitter_2(hc12rec[2], hc12rec[3]);
                break;
              case 5:
                sinelon_2(hc12rec[2], hc12rec[3]);
                break;
              case 6:
                bpm_2(hc12rec[2], hc12rec[3]);
                break;
              case 7:
                confetti_2(hc12rec[2], hc12rec[3]);
                break;
              case 8:
                juggle_2(hc12rec[2], hc12rec[3]);
                break;
              case 9:
                strobe(hc12rec[2], hc12rec[3], hc12rec[4]);
                break;
              default:
                fillSolid(255, 255, 255);
                break;
            }
#if hwMod>1
          }
#endif
          break;
        case 2:
          EVERY_N_SECONDS(2)rainbowColour(33, 255);
          fillSolid(rgb.r, rgb.g, rgb.b);
          break;
        case 3:
          EVERY_N_SECONDS(2)randomColour(255);
          fillSolid(rgb.r, rgb.g, rgb.b);
          break;
        case 4:
          rainbowWithGlitter_2(4, 100);
          break;
        case 5:
          confetti_2(75, 29);
          break;
        case 6:
          sinelon_2(37, 4);
          break;
        case 7:
          bpm_2(120, 4);
          break;
#if hwMod > 0
        case 8: // rc control option - MC

          switch (select)
          {
            case 0:
              navLight();
              break;
            case 1:
              sinelon_2(37, 4);
              break;
            case 2:
              Fire2012L();
              Fire2012R();
              break;
            case 3:
              pride();
              break;
            case 4:
              confetti_2(75, 29);
              break;
            case 5:
              bpm_2(120, 4);
              break;
            case 6:
              EVERY_N_SECONDS(2)rainbowColour(33, 255);
              fillSolid(rgb.r, rgb.g, rgb.b);
              break;
            case 7:
              EVERY_N_SECONDS(2)randomColour(255);
              fillSolid(rgb.r, rgb.g, rgb.b);
              break;

            default:
              navLight();
              break;
          }
          break;
#else
        case 8:
          Fire2012L(); // run simulation frame
          Fire2012R();
          break;
#endif
      }
    }
    FastLED.show(); // display this frame
    // FastLED.delay(1000 / FPS);
  }
}
//Load settings from eeprom
void loadConfig() {
  // To make sure there are settings, and they are YOURS!
  // If nothing is found it will use the default settings.
  if (EEPROM.read(CONFIG_START + 0) == CONFIG_VERSION[0] &&
      EEPROM.read(CONFIG_START + 1) == CONFIG_VERSION[1] &&
      EEPROM.read(CONFIG_START + 2) == CONFIG_VERSION[2])
    for (unsigned int t = 0; t < sizeof(storage); t++)
      *((char*)&storage + t) = EEPROM.read(CONFIG_START + t);
}
//Save settings to eeprom
void saveConfig() {
  for (unsigned int t = 0; t < sizeof(storage); t++)
    EEPROM.update(CONFIG_START + t, *((char*)&storage + t));
}
// strobes for predefined time then returns to the previous pattern (HC12 only)
void strobe(uint8_t strobeMs, uint8_t HUE, uint8_t SAT) {

  if (strobeLatch == 0) {
    lastMillis = millis();
    strobeLatch = 1;
  }
  uint32_t ms = millis();
  if (ms - lastStrobeMs >= strobeMs) {
    fill_solid(leds, NUM_LEDS, isOn ? CHSV( 0, 0, 0) : CHSV( HUE, SAT, 255));
    isOn = !isOn;
    lastStrobeMs = ms;
  }
  if (strobeLatch == 1 && millis() - lastMillis >  strobeTime) {
    strobeLatch = 0;
    if (hc12rec[1] == hc12recPrev[1]) {
      byte fault[] = {0, 0, 200, 0, 0};
      memcpy(hc12rec, fault, 5);
    }
    else memcpy(hc12rec, hc12recPrev, 5);
  }
}
// show requested colour for predefined time then return to previous pattern (HC12 only)
void blinko(byte valueR, byte valueG, byte valueB)
{
  if (blinkLatch == 0) {
    blinkLatch = 1;
    lastMillis = millis();
    fill_solid( &(leds[0]), NUM_LEDS, CRGB( valueR, valueG, valueB) );
  }
  else {
    if (millis() - lastMillis > blinkTime) {
      blinkLatch = 0;
      if (hc12rec[1] == hc12recPrev[1]) {
        byte fault[] = {0, 0, 200, 0, 0};
        memcpy(hc12rec, fault, 5);
      }
      else memcpy(hc12rec, hc12recPrev, 5);
    }
  }
}

// fill wing with requested colour
void fillSolid(byte valueR, byte valueG, byte valueB)
{
  fill_solid( &(leds[0]), NUM_LEDS, CRGB( valueR, valueG, valueB) );
}
//fire simulation for left wing
void Fire2012L()
{
  // Array of temperature readings at each simulation cell
  static byte heat[WING_LEDS];
  // Step 1.  Cool down every cell a little
  for ( int i = 0; i < WING_LEDS; i++) {
    heat[i] = qsub8( heat[i],  random8(0, ((COOLING * 10) / WING_LEDS) + 2));
  }
  // Step 2.  Heat from each cell drifts 'up' and diffuses a little
  for ( int k = WING_LEDS - 1; k >= 2; k--) {
    heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2] ) / 3;
  }
  // Step 3.  Randomly ignite new 'sparks' of heat near the bottom
  if ( random8() < SPARKING ) {
    int y = random8(7);
    heat[y] = qadd8( heat[y], random8(160, 255) );
  }
  // Step 4.  Map from heat cells to LED colors
  for ( int j = 0; j < WING_LEDS; j++) {
    CRGB color = HeatColor( heat[j]);
    int pixelnumber;
    if ( gReverseDirection ) {
      pixelnumber = (WING_LEDS - 1) - j;
    } else {
      pixelnumber = j;
    }
    leds[pixelnumber] = color;
    leds[WING_LEDS * 2 - 1 - pixelnumber] = color;
  }
}
//fire simulation for right wing
void Fire2012R()
{
  // Array of temperature readings at each simulation cell
  static byte heat[WING_LEDS];
  // Step 1.  Cool down every cell a little
  for ( int i = 0; i < WING_LEDS; i++) {
    heat[i] = qsub8( heat[i],  random8(0, ((COOLING * 10) / WING_LEDS) + 2));
  }
  // Step 2.  Heat from each cell drifts 'up' and diffuses a little
  for ( int k = WING_LEDS - 1; k >= 2; k--) {
    heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2] ) / 3;
  }
  // Step 3.  Randomly ignite new 'sparks' of heat near the bottom
  if ( random8() < SPARKING ) {
    int y = random8(7);
    heat[y] = qadd8( heat[y], random8(160, 255) );
  }
  // Step 4.  Map from heat cells to LED colors
  for ( int j = 0; j < WING_LEDS; j++) {
    CRGB color = HeatColor( heat[j]);
    int pixelnumber;
    if ( gReverseDirection ) {
      pixelnumber = (WING_LEDS - 1) - j;
    } else {
      pixelnumber = j;
    }
    leds[WING_LEDS * 2 + pixelnumber] = color;
    leds[WING_LEDS * 4 - 1 - pixelnumber] = color;
  }
}
// rainbow pattern with glitter added
void rainbowWithGlitter_2( uint8_t stripeDensity, uint8_t chanceOfGlitter)
{
  // built-in FastLED rainbow, plus some random sparkly glitter
  fill_rainbow( leds, NUM_LEDS, gHue, stripeDensity);
  addGlitter(chanceOfGlitter);
}
// random sparkles
void addGlitter( fract8 chanceOfGlitter)
{
  if ( random8() < chanceOfGlitter) {
    leds[ random16(NUM_LEDS) ] += CRGB::White;
  }
}
// random colored speckles that blink in and fade smoothly
void confetti_2( uint8_t colorVariation, uint8_t fadeAmount)
{
  fadeToBlackBy( leds, NUM_LEDS, fadeAmount);
  int pos = random16(NUM_LEDS);
  leds[pos] += CHSV( gHue + random8(colorVariation), 200, 255);
}
// a colored dot sweeping back and forth, with fading trails
void sinelon_2( uint8_t bpmSpeed, uint8_t fadeAmount)
{
  fadeToBlackBy( leds, NUM_LEDS, fadeAmount);
  int pos = beatsin16(bpmSpeed, 0, NUM_LEDS);
  leds[pos] += CHSV( gHue, 255, 192);
}
// colored stripes pulsing at a defined Beats-Per-Minute (BPM)
void bpm_2( uint8_t bpmSpeed, uint8_t stripeWidth)
{
  uint8_t BeatsPerMinute = bpmSpeed;
  CRGBPalette16 palette = PartyColors_p;
  uint8_t beat = beatsin8( BeatsPerMinute, 64, 255);
  for ( int i = 0; i < NUM_LEDS; i++) {
    leds[i] = ColorFromPalette(palette, gHue + (i * stripeWidth), beat);
  }
}
// colored dots, weaving in and out of sync with each other
void juggle_2( uint8_t numDots, uint8_t baseBpmSpeed) {
  fadeToBlackBy( leds, NUM_LEDS, 100);
  byte dothue = 0;
  for ( int i = 0; i < numDots; i++) {
    leds[beatsin16(i + baseBpmSpeed, 0, NUM_LEDS)] |= CHSV(dothue, 255, 224);
    dothue += (256 / numDots);
  }
}
// This function draws rainbows with an ever-changing,
// widely-varying set of parameters.
void pride() {
  static uint16_t sPseudotime = 0;
  static uint16_t sLastMillis = 0;
  static uint16_t sHue16 = 0;
  uint8_t sat8 = beatsin88( 87, 220, 250);
  uint8_t brightdepth = beatsin88( 341, 96, 224);
  uint16_t brightnessthetainc16 = beatsin88( 203, (25 * 256), (40 * 256));
  uint8_t msmultiplier = beatsin88(147, 23, 60);
  uint16_t hue16 = sHue16;//gHue * 256;
  uint16_t hueinc16 = beatsin88(113, 1, 3000);
  uint16_t ms = millis();
  uint16_t deltams = ms - sLastMillis ;
  sLastMillis  = ms;
  sPseudotime += deltams * msmultiplier;
  sHue16 += deltams * beatsin88( 400, 5, 9);
  uint16_t brightnesstheta16 = sPseudotime;
  for ( uint16_t i = 0 ; i < NUM_LEDS; i++) {
    hue16 += hueinc16;
    uint8_t hue8 = hue16 / 256;
    brightnesstheta16  += brightnessthetainc16;
    uint16_t b16 = sin16( brightnesstheta16  ) + 32768;
    uint16_t bri16 = (uint32_t)((uint32_t)b16 * (uint32_t)b16) / 65536;
    uint8_t bri8 = (uint32_t)(((uint32_t)bri16) * brightdepth) / 65536;
    bri8 += (255 - brightdepth);
    CRGB newcolor = CHSV( hue8, sat8, bri8);
    uint16_t pixelnumber = i;
    pixelnumber = (NUM_LEDS - 1) - pixelnumber;
    nblend( leds[pixelnumber], newcolor, 64);
  }
}
//generate succsesive rainbow RGB values
void rainbowColour(byte hueStep, byte v) {
  hue = hue + hueStep;
  hsv = CHSV(hue, 255, v);
  hsv2rgb_rainbow( hsv, rgb);  //convert HSV to RGB
}
// Generate random RGB values
void randomColour(byte v) {
  hue = random8();
  hsv = CHSV(hue, 255, v);
  hsv2rgb_rainbow( hsv, rgb);  //convert HSV to RGB
}
// White centre with green and red wingtips (panic mode)
void navLight() {
  fill_solid( &(leds[0]), NUM_LEDS, CRGB( 255, 255, 255) );
  fill_solid( &(leds[NUM_LEDS / 4 - navWidth]), navWidth * 2, CRGB( 255, 0, 0) );
  fill_solid( &(leds[NUM_LEDS / 4 * 3 - navWidth]), navWidth * 2, CRGB( 0, 255, 0) );
}
//Receive data from HC12
void recvBytesWithStartEndMarkers() {
  // Receives data into tempBuffer[]
  static boolean inProgress = false;
  static byte bytesRecvd = 0;
  byte startMarker = 0x82;
  byte endMarker = 0x83;
#if hwMod > 1
  if (Serial.available() > 0 && allReceived == false) {
    byte x = Serial.read();
#else    
  if (HC12.available() > 0 && allReceived == false) {
    byte x = HC12.read();
#endif
    if (x == startMarker) {           //detect start marker
      bytesRecvd = 0;
      inProgress = true;
    }

    if (inProgress) {                 //receive data
      tempBuffer[bytesRecvd] = x;
      bytesRecvd ++;
    }

    if (bytesRecvd >= numBytes) {     //check we dont get too much
      inProgress = false;
    }

    if (x == endMarker) {             //detect end marker
      inProgress = false;
      allReceived = true;

      checkReceivedBytes();
    }
  }
}
//check and parse HC12 data received
void checkReceivedBytes() {
  if (allReceived) {

    uint8_t checksum = 0;
    //Calculate checksum
    for (uint8_t i = 1; i < sizeof(tempBuffer) - 3; i++) {
      checksum ^= tempBuffer[i];
    }
    //Only use if checksum is correct
    if (checksum == tempBuffer[7] + tempBuffer[8] && checksum != 0) {
      //only use if id is the same or 0 (all) or odd/even
      if (tempBuffer[1] == storage.settings[0] ||
          tempBuffer[1] == 0 || tempBuffer[1] == (storage.settings[0] % 2 + 10)) {
        memcpy(hc12recPrev, hc12rec, 5);
        for (uint8_t i = 1; i < sizeof(tempBuffer) - 4; i++) {
          hc12rec[i - 1] = tempBuffer[i];
        }
      }
    }
    allReceived = false;
  }
}
//Display menu and store values
void menuDisplay()
{
  fill_solid( &(leds[0]), NUM_LEDS, CRGB( 0, 0, 0) );
  for (int x = 1; x <= storage.settings[Menu]; x++) {
    leds[x * 2] = CHSV(Menu * (256 / sizeof(storage.settings)), 255, 255);
    FastLED.setBrightness( storage.settings[2] * 32 - 1 );
  }
#if hwMod > 0
  if ( Menu == 3)pwmRange();          //detect pwm changes if in menu 3
#endif
  if (millis() - lastMillis > 3000) { //return from menu after inactivity
#if hwMod > 0
    calLut();                         //calculate look up table for pwm divisions (storage.settings[3])
    pwmRangeLatch = 0;
    stickMove = 0;
#endif
    buttonLatch = 0;
    saveConfig();
  }
}
// detect short and long button presses
// short button presses increment setting
// long button presses inctrement menu
void handleEvent(AceButton* button, uint8_t eventType, uint8_t buttonState) {
  switch (eventType) {
    case AceButton::kEventReleased:
      if (millis() - lastMillis < 1500) {
        storage.settings[Menu]++;
        if (storage.settings[Menu] > 8)storage.settings[Menu] = 1;
      }
      buttonLatch = 1;
      lastMillis = millis();
      break;
    case AceButton::kEventLongPressed:
      buttonLatch = 1;
      Menu++;
      if (Menu > sizeof(storage.settings) - 1)Menu = 0;
      lastMillis = millis();
      break;
  }
}

#if hwMod > 0
// interrup - detect rising edge
void rising() {
  attachInterrupt(0, falling, FALLING);
  prev_time = micros();
}
// interrupt - detect falling edge
void falling() {
  attachInterrupt(0, rising, RISING);
  pwm_value = micros() - prev_time;
  //Serial.println(pwm_value);
}
// determine selection based on pwm
uint8_t pwmSelect (uint8_t pwmIn) {
  static uint8_t current = 0;
  if (pwmIn < storage.pwmLut[current][1] || pwmIn > storage.pwmLut[current][2]) {
    uint8_t i = 0;
    for (i = 0; i < storage.settings[3]; i++) {
      if ( pwmIn >= storage.pwmLut [i] [0] && pwmIn <= storage.pwmLut [i + 1][0])break;

    }
    current = i;
  }
  return current;
}
// detect if PWM input changes, if so store min max range.
void pwmRange() {
  static int pwmStart = 0;
  if (!pwmRangeLatch) {
    pwmStart = pwm_value;
    pwmRangeLatch = 1;
  }
  if (stickMove == 0 && (pwm_value - pwmStart > 100 || pwmStart - pwm_value > 100)) {
    stickMove = 1;
    storage.pwmHighVal = 1500;
    storage.pwmLowVal = 1500;
  }
  if (stickMove) {
    if (pwm_value < storage.pwmLowVal) storage.pwmLowVal = pwm_value;
    if (pwm_value > storage.pwmHighVal) storage.pwmHighVal = pwm_value;
  }
}
// calculate look up table for pwm division selection with hysterisis (12.5%)
void calLut() {
  uint8_t y = 0;
  uint8_t margin = 255 / storage.settings[3] / 8; // 12.5%
  // divisions
  for (y = 0; y <= storage.settings[3]; y++) {
    storage.pwmLut[y][0] = 255 * y / storage.settings[3];
  }
  // lower bounds
  storage.pwmLut[0][1] = 0;
  for (y = 1; y <= storage.settings[3]; y++) {
    storage.pwmLut[y][1] = storage.pwmLut[y][0] - margin;
  }
  // upper bounds
  for (y = 0; y < storage.settings[3]; y++) {
    storage.pwmLut[y][2] = storage.pwmLut[y][0] + margin;
  }
  y++;
  storage.pwmLut[y][2] = 255;
}
#endif