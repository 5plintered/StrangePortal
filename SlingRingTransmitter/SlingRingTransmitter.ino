

// Basic demo for accelerometer readings from Adafruit LIS3DH

#include <Adafruit_BusIO_Register.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_I2CRegister.h>
#include <Adafruit_SPIDevice.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <SPI.h>
#include <math.h>

#include <RFM69.h>     //get it here: https://www.github.com/lowpowerlab/rfm69
#include <RFM69_ATC.h> //get it here: https://www.github.com/lowpowerlab/rfm69
#include <SPIFlash.h>  //get it here: https://www.github.com/lowpowerlab/spiflash
#include <SPI.h>       //included with Arduino IDE install (www.arduino.cc)

//*********************************************************************************************
//************ IMPORTANT SETTINGS - YOU MUST CHANGE/CONFIGURE TO FIT YOUR HARDWARE *************
//*********************************************************************************************
#define NODEID 99
#define NETWORKID 100
#define GATEWAYID 1
#define FREQUENCY RF69_915MHZ
#define ENCRYPTKEY "sampleEncryptKey" //has to be same 16 characters/bytes on all nodes, not more not less!
#define IS_RFM69HW_HCW                //uncomment only for RFM69HW/HCW! Leave out if you have RFM69W/CW!
#define ENABLE_ATC                    //comment out this line to disable AUTO TRANSMISSION CONTROL
#define SERIAL_BAUD 115200
#ifdef ENABLE_ATC
RFM69_ATC radio;
#else
RFM69 radio;
#endif

int TRANSMITPERIOD = 500;   //transmit a packet to gateway so often (in ms)
int MEASUREMENTPERIOD = 100; //Measure accelerometer so often (in ms)

byte sendSize = 0;
boolean requestACK = false;

typedef struct
{
  int nodeId;           //store this nodeId
  unsigned long uptime; //uptime in ms
  float magnitude;      //temperature maybe?
} Payload;
Payload theData;
//*********************************************************************************************

// Used for software SPI
#define LIS3DH_CLK 8
#define LIS3DH_SDO 5
#define LIS3DH_SDA 6
#define LIS3DH_CS 7

// software SPI
Adafruit_LIS3DH lis = Adafruit_LIS3DH(LIS3DH_CS, LIS3DH_SDA, LIS3DH_SDO, LIS3DH_CLK);

SPIFlash flash(SS_FLASHMEM, 0xEF30); //EF40 for 16mbit windbond chip
float prevMag = -1;
int averageAccelerationPeriod = 2000; //In milliseconds
const int numDataPoints = averageAccelerationPeriod / MEASUREMENTPERIOD;
float magArray[20]; // Need to manually calculate numdatapoints as array size
long lastPeriod = -1;

void initAccelerometer()
{
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("LIS3DH test!");

  if (!lis.begin(0x18))
  { // change this to 0x19 for alternative i2c address
    Serial.println("Couldnt start");
    while (1)
      yield();
  }
  Serial.println("LIS3DH found!");

  lis.setRange(LIS3DH_RANGE_2_G); // 2, 4, 8 or 16 G!

  Serial.print("Range = ");
  Serial.print(2 << lis.getRange());
  Serial.println("G");

  lis.setDataRate(LIS3DH_DATARATE_50_HZ);
  Serial.print("Data rate set to: ");
  switch (lis.getDataRate())
  {
  case LIS3DH_DATARATE_1_HZ:
    Serial.println("1 Hz");
    break;
  case LIS3DH_DATARATE_10_HZ:
    Serial.println("10 Hz");
    break;
  case LIS3DH_DATARATE_25_HZ:
    Serial.println("25 Hz");
    break;
  case LIS3DH_DATARATE_50_HZ:
    Serial.println("50 Hz");
    break;
  case LIS3DH_DATARATE_100_HZ:
    Serial.println("100 Hz");
    break;
  case LIS3DH_DATARATE_200_HZ:
    Serial.println("200 Hz");
    break;
  case LIS3DH_DATARATE_400_HZ:
    Serial.println("400 Hz");
    break;

  case LIS3DH_DATARATE_POWERDOWN:
    Serial.println("Powered Down");
    break;
  case LIS3DH_DATARATE_LOWPOWER_5KHZ:
    Serial.println("5 Khz Low Power");
    break;
  case LIS3DH_DATARATE_LOWPOWER_1K6HZ:
    Serial.println("16 Khz Low Power");
    break;
  }

  for (int i = 0; i < numDataPoints; i++)
  {
    magArray[i] = 0;
  }
}

void initRadio()
{
  radio.initialize(FREQUENCY, NODEID, NETWORKID);
#ifdef IS_RFM69HW_HCW
  radio.setHighPower(); //must include this only for RFM69HW/HCW!
#endif
  radio.encrypt(ENCRYPTKEY);
  //   char buff[50];
  //   sprintf(buff, "\nTransmitting at %d Mhz...", FREQUENCY == RF69_433MHZ ? 433 : FREQUENCY == RF69_868MHZ ? 868 : 915);
  //   Serial.println(buff);

  if (flash.initialize())
    Serial.println("SPI Flash Init OK!");
  else
    Serial.println("SPI Flash Init FAIL! (is chip present?)");
}

float calculateAverageAcceleration()
{
  float sum = 0;
  for (int i = 0; i < numDataPoints; i++)
  {
    sum += magArray[i];
  }
  return sum / numDataPoints;
}

int index = 0;
void SaveDataPoint(float value)
{
  magArray[index++] = value;
  if(index >= numDataPoints) {
    index = 0;
  }
}

void printAccelerometer()
{
  int currPeriod = millis() / MEASUREMENTPERIOD;
  if (currPeriod != lastPeriod)
  {
    /* Or....get a new sensor event, normalized */
    sensors_event_t event;
    lis.getEvent(&event);

    float x = event.acceleration.x;
    float y = event.acceleration.y;
    float z = event.acceleration.z;
    float magnitude = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
    float magDelta = abs(magnitude - prevMag);
    SaveDataPoint(magDelta);
    theData.magnitude = calculateAverageAcceleration();
    /* Display the results (acceleration is measured in m/s^2) */
    Serial.print("X: ");
    Serial.print(x);
    Serial.print(" Y: ");
    Serial.print(y);
    Serial.print(" Z: ");
    Serial.print(z);
    Serial.print(" m/s^2 ");
    Serial.print(" \tmag: ");
    Serial.print(magnitude);
    Serial.print(" delta: ");
    Serial.print(magDelta);
    Serial.print(" avg: ");
    Serial.println(calculateAverageAcceleration());
    Serial.println();

    prevMag = magnitude;
  }
}

void printRadio()
{
  //process any serial input
  if (Serial.available() > 0)
  {
    char input = Serial.read();
    if (input >= 48 && input <= 57) //[0,9]
    {
      TRANSMITPERIOD = 100 * (input - 48);
      if (TRANSMITPERIOD == 0)
        TRANSMITPERIOD = 1000;
      Serial.print("\nChanging delay to ");
      Serial.print(TRANSMITPERIOD);
      Serial.println("ms\n");
    }

    if (input == 'r') //d=dump register values
      radio.readAllRegs();
    //if (input == 'E') //E=enable encryption
    //  radio.encrypt(ENCRYPTKEY);
    //if (input == 'e') //e=disable encryption
    //  radio.encrypt(null);

    if (input == 'd') //d=dump flash area
    {
      Serial.println("Flash content:");
      int counter = 0;

      while (counter <= 256)
      {
        Serial.print(flash.readByte(counter++), HEX);
        Serial.print('.');
      }
      while (flash.busy())
        ;
      Serial.println();
    }
    if (input == 'e')
    {
      Serial.print("Erasing Flash chip ... ");
      flash.chipErase();
      while (flash.busy())
        ;
      Serial.println("DONE");
    }
    if (input == 'i')
    {
      Serial.print("DeviceID: ");
      word jedecid = flash.readDeviceId();
      Serial.println(jedecid, HEX);
    }
  }

  //check for any received packets
  if (radio.receiveDone())
  {
    Serial.print('[');
    Serial.print(radio.SENDERID, DEC);
    Serial.print("] ");
    for (byte i = 0; i < radio.DATALEN; i++)
      Serial.print((char)radio.DATA[i]);
    Serial.print("   [RX_RSSI:");
    Serial.print(radio.readRSSI());
    Serial.print("]");

    if (radio.ACKRequested())
    {
      radio.sendACK();
      Serial.print(" - ACK sent");
      delay(10);
    }
    Blink(LED_BUILTIN, 5);
    Serial.println();
  }

  int currPeriod = millis() / TRANSMITPERIOD;
  if (currPeriod != lastPeriod)
  {
    //fill in the struct with new values
    theData.nodeId = NODEID;
    theData.uptime = millis();
    // theData.temp = 69; //it's hot!

    Serial.print("Sending struct (");
    Serial.print(sizeof(theData));
    Serial.print(" bytes) ... ");
    if (radio.sendWithRetry(GATEWAYID, (const void *)(&theData), sizeof(theData)))
      Serial.print(" ok!");
    else
      Serial.print(" nothing...");
    Serial.println();
    Blink(LED_BUILTIN, 3);
    lastPeriod = currPeriod;
  }
}

void Blink(byte PIN, int DELAY_MS)
{
  pinMode(PIN, OUTPUT);
  digitalWrite(PIN, HIGH);
  delay(DELAY_MS);
  digitalWrite(PIN, LOW);
}

void setup(void)
{
  Serial.begin(SERIAL_BAUD);
  initAccelerometer();
  initRadio();
}

void loop()
{
  printAccelerometer();
  printRadio();
}
