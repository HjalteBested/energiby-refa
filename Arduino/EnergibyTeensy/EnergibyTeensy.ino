#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>
#include <OSCMessage.h>
#include <OSCBundle.h>
#include <OSCData.h>
#include <Adafruit_NeoPixel.h>
#include <Bounce.h>

#define PRINT_DEBUG

// A variable to know how long the LED has been turned on
elapsedMillis ledOnMillis;

// NeoPixel Led Strips
#define NUM_NEOPIXEL_STRIPS 6
const unsigned char NeoPixelPin[NUM_NEOPIXEL_STRIPS]   = {33, 34, 35, 36, 37, 38};
const unsigned char NeoPixelCount[NUM_NEOPIXEL_STRIPS] = {64, 78, 14, 28, 29, 29};
Adafruit_NeoPixel strip1(NeoPixelCount[0], NeoPixelPin[0], NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip2(NeoPixelCount[1], NeoPixelPin[1], NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip3(NeoPixelCount[2], NeoPixelPin[2], NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip4(NeoPixelCount[3], NeoPixelPin[3], NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip5(NeoPixelCount[4], NeoPixelPin[4], NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip6(NeoPixelCount[5], NeoPixelPin[5], NEO_GRB + NEO_KHZ800);

Adafruit_NeoPixel* strips[NUM_NEOPIXEL_STRIPS] = {&strip1, &strip2, &strip3, &strip4, &strip5, &strip6};

elapsedMillis pixelUpdateMillis;
unsigned long pixelUpdateInterval = 37;

// Vind Variables
auto& vind_strip = strip3;
float vind = 0.0f;
float vind_max = 35.0f;
float vind_numPixels = 14;
int vind_pixelOffset = 0;

// Sol Variables
auto& sol_strip = strip4;
float sol = 0.0f;
float sol_max = 5.0f;
float sol_numPixels = 14;
int sol_pixelOffset = 0;

// Bio Variables
auto& bio_strip = strip4;
float bio_numPixels = 14;
int bio_pixelOffset = 14;
float bio = 0;
float bio_max = 60.0f;

// Ilt
auto& ilt_strip = strip2;
float ilt_numPixels = 26;
int ilt_pixelOffset = 26;

// Ovn
int amountInOven = 0;
int amountInOven_ok_min = 8;
int amountInOven_ok_max = 18;
int amountInOven_max = 26;
int amountInStorage = 64;

// Production
float production = 1.0;
float productionMin = 1.0f;
float productionPercent = 1.0f;


const uint8_t elActiveLedPIN = 26;
const uint8_t heatActiveLedPIN = 27;
bool elActive = true;
bool heatActive = true;

float timeOfDay = 0.0f;

// A Lowpass Filter
struct OnePole {
  OnePole(float aAlpha = 0.1, float aValue = 0.0f):alpha(aAlpha),value(aValue),in(aValue){}

  float alpha;
  float value;
  float in;

  float process(float aIn){
    in = aIn;
    value = in*alpha + value*(1.0-alpha);
    return value;
  }
};

OnePole elFilter  (0.02 ,1.0);
OnePole heatFilter(0.01 ,1.0);

float elAmount = 1.0;
float heatAmount = 1.0;


elapsedMillis startButtonElapsed;

// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:
byte mac[] = {
  0xDA, 0xA9, 0x1E, 0x2F, 0xAE, 0xE7
};

// buffers for receiving and sending data
char str[UDP_TX_PACKET_MAX_SIZE];  // buffer to hold incoming packet,

// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Udp;

// ----------------------------------------- //
// ------------ Open Sound Control --------- //
// ----------------------------------------- //
const unsigned int localPort = 7134;         // local port to listen for OSC packets (actually not used for sending)

const IPAddress PiIp(192,168,1,33);        // remote IP of your computer
const unsigned int PiPort   = 7133;         // remote port to receive OSC

const IPAddress BroadCastIp(255,255,255,255);     // remote IP of your computer
const unsigned int BroadCastPort = 7255;          // remote port to receive OSC

OSCMessage oscMsg("/airSpeed");            // Outgoing OSC Message


OSCErrorCode error;

// ------------------ Sensor --------------- //
// ----------------------------------------- //
unsigned long start = millis();
unsigned long current_millis;

// Digital Input Variables
#define NUM_BUTTONS 4
const byte buttonPINS[NUM_BUTTONS]   = {15, 16, 17, 18};
// --- Bounce the Button's --->
const int debounceTime = 20;
Bounce bounceButtons[] = {
  Bounce(buttonPINS[0], debounceTime),
  Bounce(buttonPINS[1], debounceTime),
  Bounce(buttonPINS[2], debounceTime),
  Bounce(buttonPINS[3], debounceTime),
};

enum buttonEnum {
  FillOven, StartGame, HeatActive, ElActive
};

elapsedMillis buttonReadMillis;
unsigned long buttonReadInterval = 23;

// Analog Input Variables
elapsedMillis analogReadMillis;
unsigned long analogReadInterval = 31;
const uint8_t airSpeedPIN = 14;
int lastAirSpeed = 0;

// Output for lights in city

struct CityLight {
  CityLight(uint8_t a_pin, uint8_t a_pwm, bool a_on, float a_onTime, float a_offTime, float a_onTime2 = -1.0f, float a_offTime2 = 1.0f)
  :pin(a_pin), pwm(a_pwm), on(a_on), onTime(a_onTime), offTime(a_offTime), onTime2(a_onTime2), offTime2(a_offTime2), initOn(a_on){}

  void update(float timeOfDay){
    if     (on && lastTimeOfDay < offTime && timeOfDay >= offTime) on = false;
    else if(!on && lastTimeOfDay < onTime && timeOfDay >= onTime)  on = true;
    else if(on && onTime2 >= 0  && lastTimeOfDay < offTime2 && timeOfDay >= offTime2) on = false;
    else if(!on && onTime2 >= 0 && lastTimeOfDay < onTime2 && timeOfDay >= onTime2)   on = true;

    lastTimeOfDay = timeOfDay;
  }

  void init(){
    on = initOn;
  }

  uint8_t pin;
  uint8_t pwm;
  bool on;
  float onTime;
  float offTime;
  float onTime2;
  float offTime2;

  bool initOn;
  float lastTimeOfDay = 0.0f;
};

#define NUM_cityLights 15
CityLight cityLights[NUM_cityLights] = {
  CityLight( 0, 255, false, 9.00, 19.00),    // Bygning hvor varmen går ind
  CityLight( 1, 255, false, 8.00, 23.95),    // Hvid etage bolig midt højre
  CityLight( 2, 255, true , 0.00, 48.00),    // Industri bagerst til hjøre
  CityLight( 3, 255, false, 6.70, 22.7),    // Etagebolig (stor) bag blå hus
  CityLight( 4, 255, true , 0.00, 48.0),    // Kraftværk (el)
  CityLight( 5, 255, true , 7.40, 1.00),    // Etagebolig (lille) bag blå hus
  CityLight( 7, 255, false, 7.0 , 23.00),    // Lille røde hus
  CityLight( 8, 255, false, 8.55, 17.19),    // Brun erhverv
  CityLight( 9, 255, true , 0.0 , 48.0  ),    // Fjernvarme
  CityLight(10, 255, false, 7.70, 16.66),    // Rådhus
  CityLight(11, 255, false, 7.75, 15.90),   // Lille Skole
  CityLight(12, 255, false, 6.90, 23.30),   // Vinkel Byhus / Erhverv
  CityLight(24, 255, false, 6.00, 22.0),    // Sort / Hvid Villa
  CityLight(25, 255, false, 6.50, 22.8),  // Stor etage bolig bag rådhus
  CityLight(28, 255, true , 7.80,  2.0),    // Moderne hvid firkant bolig
};


// Windmill
const uint8_t windmill_PIN   = 29;
const uint8_t windmill_speed = 30;


// Analog Input Variables
elapsedMillis cityLightMillis;
unsigned long cityLightInterval = 29;

// --- Initialize ----------------------------------------------------------->
void setup() {
    Serial.begin(115200);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, 1);    // turn *on* led

    delay(1000);

    
    pinMode(windmill_PIN, OUTPUT);
    analogWrite(windmill_PIN, 0);

    // Setup Buttons
    Serial.print("Setting up IO channels");
    for(int i=0; i<NUM_BUTTONS; i++){
      pinMode(buttonPINS[i], INPUT_PULLUP);
    }
    
    pinMode(elActiveLedPIN  , OUTPUT);
    pinMode(heatActiveLedPIN, OUTPUT);
    digitalWrite(elActiveLedPIN  , HIGH);
    digitalWrite(heatActiveLedPIN, HIGH);

    for(int i=0; i<NUM_cityLights; i++){
      pinMode(cityLights[i].pin, OUTPUT);
      analogWrite(cityLights[i].pin, 0);
    }

    for(int i=0; i<NUM_cityLights; i++){
      analogWrite(cityLights[i].pin, cityLights[i].pwm);
      delay(100);
    }

    Serial.println("....done");

    // NeoPixel Strips
    Serial.print("Init NeoPixel Strips");
    for(int i=0; i<NUM_NEOPIXEL_STRIPS; i++){
      strips[i]->begin();
      strips[i]->show();
      strips[i]->setBrightness(50);
    }
    Serial.println("....done");

    Serial.print("Test NeoPixel Strips - ColorWipe");
    colorWipe(Adafruit_NeoPixel::Color(255,   0,   0)     , 2); // Red
    colorWipe(Adafruit_NeoPixel::Color(  0, 255,   0)     , 2); // Green
    colorWipe(Adafruit_NeoPixel::Color(  0,   0, 255)     , 2); // Blue
    colorWipe(Adafruit_NeoPixel::Color(  0,   0,   0)     , 2); // Blue
    Serial.println("....done");



    // Init Ethernet
    Serial.print("Starting Ethernet");
    Ethernet.begin(mac);
    
    // Check for Ethernet hardware present
    if(Ethernet.linkStatus() == LinkOFF) {
      Serial.println("...Ethernet cable is not connected.");
    }
    else {
        Serial.println("....done");
    }

    // start UDP
    Serial.print("Starting UDP: ");
    Udp.begin(localPort);
    Serial.print("Local port: ");
    Serial.print(localPort);
    Serial.println("....done");

    digitalWrite(LED_BUILTIN, 0);    // turn *off* led

    strip5.setBrightness(255);
    strip6.setBrightness(255);

    reset();

}

void reset(){
  elActive   = true;
  heatActive = true;
  for(int i=0; i<NUM_cityLights; i++) cityLights[i].init();
}

// Fill strip pixels one after another with a color. Strip is NOT cleared
// first; anything there will be covered pixel by pixel. Pass in color
// (as a single 'packed' 32-bit value, which you can get by calling
// strip.Color(red, green, blue) as shown in the loop() function above),
// and a delay time (in milliseconds) between pixels.
void colorWipe(uint8_t strip, uint32_t color, int wait) {
  for(int i=0; i<strips[strip]->numPixels(); i++) { // For each pixel in strip...
    strips[strip]->setPixelColor(i, color);         //  Set pixel's color (in RAM)
    strips[strip]->show();                          //  Update strip to match
    delay(wait);                                    //  Pause for a moment
  }
}

void colorWipe(uint32_t color, int wait) {
  for(int i=0; i<NUM_NEOPIXEL_STRIPS; i++){
    colorWipe(i, color, wait);
  }
}

void sendOsc(OSCMessage& msg, const IPAddress& ip, const unsigned int port){
  Udp.beginPacket(ip, port);
  msg.send(Udp);
  Udp.endPacket();
}

bool loopOsc(){
  bool activity = false;
  // Parse Incomming OSC
  OSCMessage msg;
  int packetSize = Udp.parsePacket();

  if (packetSize) {
    #ifdef PRINT_DEBUG
    Serial.print("Received packet of size ");
    Serial.println(packetSize);
    Serial.print("From ");
    IPAddress remote = Udp.remoteIP();
    for (int i=0; i < 4; i++) {
      Serial.print(remote[i], DEC);
      if (i < 3) {
        Serial.print(".");
      }
    }
    Serial.print(", port ");
    Serial.println(Udp.remotePort());
    #endif
    
    while(packetSize--){
      msg.fill(Udp.read());
    }
    if(!msg.hasError()){
      msg.route   ("/Led" , oscLed);
      msg.dispatch("/ElData" , oscElData);
      // msg.getAddress(str);
      // Serial.println(str);
      activity = true;
    }
    else {
      error = msg.getError();
      Serial.print("error: ");
      Serial.println(error);
    }
  }
  return activity;
}

void sendCmd(const char* cmd){
    Serial.println(cmd);
    oscMsg.setAddress("/cmd");
    oscMsg.empty();
    oscMsg.add(cmd);
    sendOsc(oscMsg, PiIp, PiPort);
}

bool loopButtons(){
    bool activity = false;
    
    if(buttonReadMillis > buttonReadInterval){

      // Update Buttons
      for(int i=0; i<NUM_BUTTONS; i++){
          bounceButtons[i].update();
      };

      if(bounceButtons[StartGame].fallingEdge()){
          startButtonElapsed = 0;
          activity = true;
      }
      else if(bounceButtons[StartGame].risingEdge()){
          if(startButtonElapsed < 2000){
            sendAirSpeed();
            sendCmd("StartButton");
          }
          else {
            sendCmd("Reset");
            reset();
          }
          activity = true;
      }
      if(bounceButtons[FillOven].fallingEdge()){
          sendCmd("FillButton");
          activity = true;
      }
      if(bounceButtons[ElActive].fallingEdge()){
          sendCmd("ElButton");
          elActive = !elActive;
          activity = true;
      }
      if(bounceButtons[HeatActive].fallingEdge()){
          sendCmd("HeatButton");
          heatActive = !heatActive;
          activity = true;
      }
      bool productionOk = productionPercent >= 1.0f;
      elAmount   = elActive && productionOk ? 1.0f : 0.0f;
      heatAmount = heatActive && productionOk ? 1.0f : 0.0f;

      elFilter  .process(elAmount);
      heatFilter.process(heatAmount);

      digitalWriteFast(elActiveLedPIN  , elActive  );
      digitalWriteFast(heatActiveLedPIN, heatActive);


      buttonReadMillis = 0;
    }

    return activity;
}

float oscAirSpeed = 0.0f;

void sendAirSpeed(){
  oscMsg.setAddress("/value");
  oscMsg.empty();
  oscMsg.add(oscAirSpeed);
  sendOsc(oscMsg, PiIp, PiPort);
}

bool loopAnalog(){
    bool activity = false;

    if(analogReadMillis > analogReadInterval){
        int airSpeed = analogRead(airSpeedPIN);
        int diff = abs(lastAirSpeed - airSpeed);
        if(airSpeed < 4)  airSpeed = 0;
        else if(diff < 8) airSpeed = lastAirSpeed;
        if(airSpeed != lastAirSpeed){
            lastAirSpeed = airSpeed;
            int airSpeedMidi = airSpeed/8;
            oscAirSpeed = 1.0 - (airSpeedMidi / 127.);
            sendAirSpeed();
            activity = true;
            Serial.print("Air: ");
            Serial.println(oscAirSpeed);
        }
        analogReadMillis = 0;
    }
    return activity;
}

void ovenPixelLoop(){
  for(int i=0; i<26; i++){
    if(i < amountInOven){
      uint32_t color = Adafruit_NeoPixel::Color(255,0,0);
      if(i > amountInOven_ok_min && i < amountInOven_ok_max) color = Adafruit_NeoPixel::Color(0,255,0);
      strip2.setPixelColor(25-i, color);
    }
    else {
      strip2.setPixelColor(25-i, 0);
    }
  }
  strip2.show();

  for(int i=0; i<strip1.numPixels(); i++){
    if(i < amountInStorage){
      uint32_t color = Adafruit_NeoPixel::Color(255,0,128);
      strip1.setPixelColor(strip1.numPixels()-1-i, color);
    }
    else {
      strip1.setPixelColor(i, 0);
    }
  }
  strip1.show();
}

void pixelLoop(){
  static unsigned long counter = 0;
  if(pixelUpdateMillis > pixelUpdateInterval){
    setBarLed(vind_strip,vind_pixelOffset,vind_numPixels,vind/vind_max,0,0,255,true);
    setBarLed(sol_strip ,sol_pixelOffset ,sol_numPixels ,sol/sol_max  ,255,100,0,false);
    setBarLed(bio_strip ,bio_pixelOffset ,bio_numPixels ,bio/bio_max  ,0,255,0,true);
    setBarLed(strip2, 26, 26, oscAirSpeed, 0  , 100, 255, false);
    setBarLed(strip2, 52, 26, bio/bio_max, 200,  55,   0, false );
    ovenPixelLoop();
    for(int i=0; i<strip5.numPixels(); i++){
      float h = heatFilter.value;
      float speed = 0.1;
      float a = abs(cos(2 * M_PI * i/strip5.numPixels() - counter * speed ));
      a = a*a*a*255;
      uint32_t hotColor  = Adafruit_NeoPixel::Color(a*h,0,a*(1.0-h));
      uint32_t coldColor = Adafruit_NeoPixel::Color(0,0,a);
      strip5.setPixelColor(i, hotColor);
      strip6.setPixelColor(strip6.numPixels()-1-i,coldColor);
    }
    strip5.show();
    strip6.show();
    
    if(vind > 0){
      analogWrite(windmill_PIN, 15+windmill_speed*vind/vind_max);
    }
    else {
      analogWrite(windmill_PIN, 0);
    }


    pixelUpdateMillis = 0;
    counter++;
  }
}

void cityLightsLoop(){
  if(cityLightMillis > cityLightInterval){

    for(int i=0; i<NUM_cityLights; i++){
      auto& x = cityLights[i];
      x.update(timeOfDay);
      if(x.on && i < elFilter.value*NUM_cityLights){
        analogWrite(x.pin, x.pwm * elFilter.value);
      }
      else {
        analogWrite(x.pin, 0);
      }

      cityLights[i].lastTimeOfDay = timeOfDay;
    }

    cityLightMillis = 0;
  }
}

void loop(){
  current_millis = millis();    
  bool oscActivity    = loopOsc();
  bool buttonActivity = loopButtons();
  bool analogActivity = loopAnalog();
  bool activity = oscActivity || buttonActivity || analogActivity;

  if(!oscActivity){
    pixelLoop();
    cityLightsLoop();
  }

  // blink the LED when any activity has happened
  if(activity){
    digitalWriteFast(LED_BUILTIN, HIGH); // LED on
    ledOnMillis = 0;
  }
  if(ledOnMillis > 15){
    digitalWriteFast(LED_BUILTIN, LOW);  // LED off
  }

}

void setBarLed(Adafruit_NeoPixel& strip, int offset, int numPixels, float value, uint8_t r, uint8_t g, uint8_t b, bool show){
  float numOn = numPixels * value;
  int numFullOn = static_cast<int>(numOn);
  float scale = numOn-numFullOn;

  for(int i=0; i<numPixels; i++) { 
    if(i < numFullOn) strip.setPixelColor(offset+numPixels-1-i, Adafruit_NeoPixel::Color(r,g,b));         
    else if (i < ceil(numOn)) strip.setPixelColor(offset+numPixels-1-i, Adafruit_NeoPixel::Color(r*scale,g*scale,b*scale));         
    else strip.setPixelColor(offset+numPixels-1-i,0);   
  }
  if(show) strip.show();                          
}


void oscElData(OSCMessage& msg){
  // Serial.print("ElData Received, Size: ");
  // Serial.print(msg.size());
  // Serial.print(", tags: ");
  // for(int i=0; i<msg.size(); i++){
  //   Serial.print(msg.getType(i));
  // }
  // Serial.println();
 
  if(msg.isFloat(0)) vind = msg.getFloat(0);
  if(msg.isFloat(1)) sol  = msg.getFloat(1);
  if(msg.isFloat(2)) bio  = msg.getFloat(2);
  if(msg.isFloat(3)) amountInOven    = msg.getFloat(3);
  if(msg.isFloat(4)) amountInStorage = msg.getFloat(4);
  if(msg.isFloat(5)) production      = msg.getFloat(5);
  if(msg.isFloat(6)) productionMin   = msg.getFloat(6);
  if(msg.isFloat(7)) timeOfDay       = msg.getFloat(7);

  Serial.print("Vind: ");            Serial.println(vind);
  Serial.print("Sol: ");             Serial.println(sol);
  Serial.print("Bio: ");             Serial.println(bio);
  Serial.print("amountInOven: ");    Serial.println(amountInOven);
  Serial.print("amountInStorage: "); Serial.println(amountInStorage);
  Serial.print("production: ");      Serial.println(production);
  Serial.print("productionMin: ");   Serial.println(productionMin);
  Serial.print("timeOfDay: ");       Serial.println(timeOfDay);
  productionPercent = production / productionMin;
  if(productionPercent > 1.0f) productionPercent = 1.0f;



}
 

void oscLed(OSCMessage& msg, int addr_offset){
  Serial.println("Led Received");
}
 
