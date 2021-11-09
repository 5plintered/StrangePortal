
#include <RFM69.h>     //get it here: https://www.github.com/lowpowerlab/rfm69
#include <RFM69_ATC.h> //get it here: https://www.github.com/lowpowerlab/rfm69
#include <SPIFlash.h>  //get it here: https://www.github.com/lowpowerlab/spiflash
#include <SPI.h>       //included with Arduino IDE install (www.arduino.cc)
#include <Adafruit_NeoPixel.h>
#include <math.h>
//*********************************************************************************************
//************ IMPORTANT SETTINGS - YOU MUST CHANGE/CONFIGURE TO FIT YOUR HARDWARE *************
//*********************************************************************************************
#define NODEID 1
#define NETWORKID 100
#define FREQUENCY RF69_915MHZ
#define ENCRYPTKEY "sampleEncryptKey" //has to be same 16 characters/bytes on all nodes, not more not less!
#define IS_RFM69HW_HCW                //uncomment only for RFM69HW/HCW! Leave out if you have RFM69W/CW!
#define ENABLE_ATC                    //comment out this line to disable AUTO TRANSMISSION CONTROL
#define SERIAL_BAUD 115200
#define LEDPIN 4       // Which pin on the Arduino is connected to the NeoPixels?
#define NUMPIXELS 186  // How many NeoPixels are attached to the Arduino?
#define NUMPARTICLES 7 //Number of lit particles
#define TAILLENGTH 20  //Number of lit particles

#define DELAYVAL 1 // Time (in milliseconds) to pause between pixels

Adafruit_NeoPixel pixels(NUMPIXELS, LEDPIN, NEO_GRB + NEO_KHZ800);

#ifdef ENABLE_ATC
RFM69_ATC radio;
#else
RFM69 radio;
#endif

#ifdef __AVR__
#include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

SPIFlash flash(SS_FLASHMEM, 0xEF30); //EF40 for 16mbit windbond chip
bool spy = false;                    //set to 'true' to sniff all packets on the same network

typedef struct
{
  int nodeId;           //store this nodeId
  unsigned long uptime; //uptime in ms
  float magnitude;      //temperature maybe?
} Payload;
Payload theData;

const int RFade[20] = {0, 0, 1, 4, 7, 12, 18, 26, 35, 46, 58, 73, 89, 107, 126, 148, 172, 197, 225, 255};
const int GFade[20] = {0, 0, 0, 1, 2, 4, 6, 8, 11, 15, 19, 23, 28, 34, 40, 47, 55, 63, 72, 81};
class Particle
{
public:
  int position;

public:
  Particle(int pos = 0)
  {
    position = pos;
  }

  void animate()
  {
    for (int i = 0; i < TAILLENGTH; i++)
    {
      pixels.setPixelColor(((position + i) % NUMPIXELS), pixels.Color(RFade[i], GFade[i], 0));
    }
     position = ( position+ 1) % NUMPIXELS;
  }

};

Particle particles[NUMPARTICLES];
void initRadio()
{
  delay(10);
  radio.initialize(FREQUENCY, NODEID, NETWORKID);
#ifdef IS_RFM69HW_HCW
  radio.setHighPower(); //must include this only for RFM69HW/HCW!
#endif
  radio.encrypt(ENCRYPTKEY);
  radio.spyMode(spy);
  char buff[50];
  sprintf(buff, "\nListening at %d Mhz...", FREQUENCY == RF69_433MHZ ? 433 : FREQUENCY == RF69_868MHZ ? 868
                                                                                                      : 915);
  Serial.println(buff);
  if (flash.initialize())
    Serial.println("SPI Flash Init OK!");
  else
    Serial.println("SPI Flash Init FAIL! (is chip present?)");
}

void initLeds()
{
  delay(200);
  // These lines are specifically to support the Adafruit Trinket 5V 16 MHz.
  // Any other board, you can remove this part (but no harm leaving it):
#if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)
  clock_prescale_set(clock_div_1);
#endif
  // END of Trinket-specific code.

  double increment = round(NUMPIXELS / NUMPARTICLES);
  for (int i = 0; i < NUMPARTICLES; i++)
  {
    particles[i] = Particle(round((increment * i)));
  }

  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
}

int lastSpin = -1;
void spinMode(int delay, int timeNow)
{
  if (timeNow - lastSpin >= delay)
  {
    pixels.clear(); // Set all pixel colors to 'off'

    for (int i = 0; i < NUMPARTICLES; i++)
    { // For each particle...
      Particle *currentParticle = &particles[i];
      currentParticle->animate();
    }
    lastSpin = timeNow;
    pixels.show();
  }
}

int lastPulse = -1;
int fadeIndex = 0;
void pulseMode(int pulseDelay, int timeNow)
{

  if ((timeNow - lastPulse) >= pulseDelay)
  {
    pixels.clear();
    fadeIndex = (fadeIndex + 1) % TAILLENGTH;
    for (int i = 0; i < NUMPIXELS; i++)
    {
      pixels.setPixelColor(i, pixels.Color(RFade[fadeIndex], GFade[fadeIndex], 0));
    }

    lastPulse = timeNow;
    pixels.show();
  }
}

void setup()
{
  Serial.begin(SERIAL_BAUD);
  initRadio();
  initLeds();
}

int charge = 0;
void loopLeds() // Animate One frame
{
  
  unsigned long timeNow = millis();

   if( charge <= 0) { // level 0
      pixels.clear();
    }
    else if( 0 < charge && charge <= 2000 ) { // level 1
      pulseMode(50, timeNow);
    }
    else if( 2000 < charge && charge <= 3000 ) { // level 1
      pulseMode(20, timeNow);
    }
    else if( 3000 < charge && charge <= 4500 ) { // level 1
      pulseMode(5, timeNow);
    }
    else if( 4500 <=  charge) { // level 5
      spinMode(1, timeNow);
    }
    
    if( 5000 < charge) { // level 5
      charge = 5000;
    }
    pixels.show(); // Send the updated pixel colors to the hardware.

  if (charge > 0)
  {
    charge -= 3;
  }
  
}

byte ackCount = 0;
int debugTimer = -1;
void loop()
{
  
  
  if (radio.receiveDone())
  {
    Serial.print('[');
    Serial.print(radio.SENDERID, DEC);
    Serial.print("] ");
    if (spy)
      Serial.print("to [");
    Serial.print(radio.TARGETID, DEC);
    Serial.print("] ");

    if (radio.DATALEN != sizeof(Payload))
      Serial.print("Invalid payload received, not matching Payload struct!");
    else
    {
      theData = *(Payload *)radio.DATA; //assume radio.DATA actually contains our struct and not something else
      Serial.print(" nodeId=");
      Serial.print(theData.nodeId);
      Serial.print(" uptime=");
      Serial.print(theData.uptime);
      Serial.print(" mag=");
      Serial.print(theData.magnitude);

      if (theData.magnitude > 0.1)
      {
        charge += 400;
      }
    }
    
    if (radio.ACKRequested())
    {
      byte theNodeID = radio.SENDERID;
      radio.sendACK();
      Serial.print(" - ACK sent.");
    }
    Serial.println();
    Blink(LED_BUILTIN, 3);
  }

  //spinMode(2, millis());
  loopLeds();
}

void Blink(byte PIN, int DELAY_MS)
{
  pinMode(PIN, OUTPUT);
  digitalWrite(PIN, HIGH);
  delay(DELAY_MS);
  digitalWrite(PIN, LOW);
}
