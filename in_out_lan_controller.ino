
/*
 Basic MQTT example
*/

#include <SPI.h>
#include <Wire.h>
#include <Ethernet2.h>
#include <PubSubClient.h>
#include <MsTimer2.h>
#include <OneWire.h>


#define MODULE_ID 1+129

#define IO_MODULE_NAME "modul_io_2"

#define MQTT_CLINT_NAME "arduino_" IO_MODULE_NAME

char TOPIC_ROOM_LIGHT[28] = { "Dom/" IO_MODULE_NAME "/wyjście_" };
#define TOPIC_ROOM_LIGHT_NR_INDEX 24

const char MQTT_SWITCH_SYBSCRIBE_TOPIC[] = { "Dom/" IO_MODULE_NAME "/komenda/#" };
#define MQTT_SWITCH_SYBSCRIBE_TOPIC_NR_INDEX 32

#define LONG_CLICK_MS 5000

// Update these with values suitable for your network.
byte mac[] = { 0x6C, 0x75, 0x1E, 0xD2, 0x00, MODULE_ID };

EthernetClient ethClient;
PubSubClient client(ethClient);
OneWire ds2401(A1);

static const int NUM_IOS = 8;
static int inputPins[NUM_IOS] = {2, 3, 4, 5, 6, 7, 8, 9};
static unsigned short inputCounters[NUM_IOS] = {0, 0, 0, 0, 0, 0, 0, 0};
static bool inputStates[NUM_IOS] = {false, false, false, false, false, false, false, false};
static bool lastInputStates[NUM_IOS] = {false, false, false, false, false, false, false, false};

static byte outputIndexToBit[NUM_IOS] = {1<<4, 1<<5, 1<<6, 1<<7, 1<<0, 1<<1, 1<<2, 1<<3};
static byte outputsState = 0;

#define ETH_SHIELD_RESET_PIN A0

#define PCF8574_ADRESS 0x38
#define MCP27008_ADRESS 0x27

void MCP27008_setup()
{
   Wire.beginTransmission(MCP27008_ADRESS);      // start talking to the device
   Wire.write(0x00);                   // select the IODIR register
   Wire.write(0x00);                   // set register value-all high, sets all pins as outputs on MCP23008
   Wire.endTransmission();            // stop talking to the devicevice
}
uint8_t MCP27008_write(uint8_t value)
{
  uint8_t error;
  Wire.beginTransmission(MCP27008_ADRESS);
  Wire.write(0x09);                   // select the GPIO register
  Wire.write(value);                   // set register value-all high
  error = Wire.endTransmission();
  if(error != 0)
  {
    Serial.print("MCP27008_write error: ");
    Serial.println(error);
  }
  return error;
}

uint8_t PCF8574_read()
{
  uint8_t _data, error;
  Wire.beginTransmission(56);
  Wire.requestFrom(56, 0x01);
  //while (Wire.available() < 1);
#if (ARDUINO <  100)
   _data = Wire.receive();
#else
   _data = Wire.read();
#endif
  error = Wire.endTransmission();
  if(error != 0)
  {
    Serial.print("PCF8574_read error: ");
    Serial.println(error);
  }
  return _data;
}
uint8_t PCF8574_write(uint8_t value)
{
  uint8_t error;
  Wire.beginTransmission(56);
  Wire.write(value);
  error = Wire.endTransmission();
  if(error != 0)
  {
    Serial.print("PCF8574_write error: ");
    Serial.println(error);
  }
  return error;
}

void setOutputState(int index, bool state)
{
  byte currentState = outputsState;
  TOPIC_ROOM_LIGHT[TOPIC_ROOM_LIGHT_NR_INDEX] = '0' + (index+1);
  TOPIC_ROOM_LIGHT[TOPIC_ROOM_LIGHT_NR_INDEX+1] = '\0';
  if(state)
  {
    Serial.print(TOPIC_ROOM_LIGHT);
    Serial.println(": ON");
    client.publish(TOPIC_ROOM_LIGHT, "ON");
    outputsState |= outputIndexToBit[index];
  }
  else
  {
    Serial.print(TOPIC_ROOM_LIGHT);
    Serial.println(": OFF");
    client.publish(TOPIC_ROOM_LIGHT, "OFF");
    outputsState &= ~(outputIndexToBit[index]);
  }
  if(outputsState != currentState)
  {
    Serial.print("Setting outputs: ");
    Serial.println(outputsState, BIN);
    PCF8574_write(outputsState);
  }
}
void toggleOutputState(int index)
{
  setOutputState(index, (outputsState & outputIndexToBit[index]) ? false : true);
}
void callback(char* topic, byte* payload, unsigned int length)
{
  int relayPin = 0;
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("[");
  int i=0;
  for (i=0;i<length;i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println("]");

  relayPin = *(topic+MQTT_SWITCH_SYBSCRIBE_TOPIC_NR_INDEX) - '1';

  if(relayPin < 0 || relayPin >= NUM_IOS)
  {
    Serial.println("Topic doesn't match");
    return;
  }
  if(length == 2 && memcmp((char*)payload, "ON", 2) == 0)
  {
    setOutputState(relayPin, true );
  }
  else if(length == 3 && memcmp((char*)payload, "OFF", 3) == 0)
  {
    setOutputState(relayPin, false );
  }
  else
  {
    Serial.println("Payload doesn't match");
  }
}

void readInputs()
{
  for(int i = 0; i < NUM_IOS; ++i)
  {
    inputCounters[i] <<= 1;
    if(digitalRead(inputPins[i]) == LOW)
    {
      inputCounters[i] |= 1;
    }
    if(inputCounters[i] == 0xFFFF)
    {
      inputStates[i] = true;
    }
    else if(inputCounters[i] == 0)
    {
      inputStates[i] = false;
    }
  }
}

void checkAndPublish()
{
  uint8_t outState = PCF8574_read();
  uint8_t changeState = outState;
  
  for(int i = 0; i < NUM_IOS; ++i)
  {
    if(inputStates[i] != lastInputStates[i])
    {
      lastInputStates[i] = inputStates[i];
      if(inputStates[i])
      {
        toggleOutputState(i);
      }
    }
  }
}

void PrintTwoDigitHex (byte b, boolean newline)
{
  Serial.print(b/16, HEX);
  Serial.print(b%16, HEX);
  if (newline) Serial.println();
}
void getMacAddress(byte* target)
{
  byte i;           // This is for the for loops
  byte data[8];     // container for the data from device
  byte crc_calc;    //calculated CRC
  byte crc_byte;    //actual CRC as sent by ds24012401
  //1-Wire bus reset, needed to start operation on the bus,
  //returns a 1/TRUE if presence pulse detected
  if (ds2401.reset() == TRUE)
  {
    Serial.println("---------- Device present ----------");
    ds2401.write(0x33);  //Send Read data command
    data[0] = ds2401.read();
    Serial.print("Family code: 0x");
    PrintTwoDigitHex (data[0], 1);
    Serial.print("Hex ROM data: ");
    for (i = 1; i <= 6; i++)
    {
      data[i] = ds2401.read(); //store each byte in different position in array
      PrintTwoDigitHex (data[i], 0);
      Serial.print(" ");
    }
    Serial.println();
    crc_byte = ds2401.read(); //read CRC, this is the last byte
    crc_calc = OneWire::crc8(data, 7); //calculate CRC of the data
    Serial.print("Calculated CRC: 0x");
    PrintTwoDigitHex (crc_calc, 1);
    Serial.print("Actual CRC: 0x");
    PrintTwoDigitHex (crc_byte, 1);
  }
  else //Nothing is connected in the bus
  {
    Serial.println("xxxxx Mac address device not connected xxxxx");
  }
}

void setup()
{
  Serial.begin(115200);
  /*
  for(int i = 0; i < NUM_IOS; ++i)
  {
    pinMode(inputPins[i], INPUT_PULLUP);      // sets the switch sensor digital pin as input
  }
  pinMode(ETH_SHIELD_RESET_PIN, OUTPUT);


  getMacAddress(mac);

  client.setServer("192.168.1.3", 1883);
  client.setCallback(callback);

  digitalWrite(A0, HIGH);
  Serial.println("Connecting Ethernet");
  
  Ethernet.begin(mac);
  // Allow the hardware to sort itself out
  delay(100);
  Serial.println(Ethernet.localIP());
  
  MsTimer2::set(2, readInputs);
  MsTimer2::start();
*/
  Wire.begin();
  //PCF8574_write(outputsState);
  

  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
// The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
// a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
            if (address<16) 
      Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");
  
      nDevices++;
    }
    else if (error==4)
    {
       Serial.print("Unknow error at address 0x");
      if (address<16)
        Serial.print("0");
        Serial.println(address,HEX);
    }
  }
  Serial.println("Scanning finished");

  MCP27008_setup();
}

unsigned long lastMillis = 0;
unsigned long lastConnectMillis = 0xFFFFFFFF;
unsigned long maintainLastMillis = 0xFFFFFFFF;
long sinceLastMaintain;
long sinceLastConnect;
int res;

uint8_t outputs= 0;

void loop()
{
  /*
  sinceLastMaintain = millis() - maintainLastMillis;
  sinceLastMaintain = sinceLastMaintain > 0 ? sinceLastMaintain : ((~((unsigned long)0) - maintainLastMillis) + millis());
  if(sinceLastMaintain >= 5000)
  {
    maintainLastMillis = millis();
    res = Ethernet.maintain();
    if(res == 2 || res == 4)
    {
      Serial.println("Connecting Ethernet");
      Serial.println(Ethernet.localIP()); 
    }
    return;
  }
  if (!client.connected())
  {
    sinceLastConnect = millis() - lastConnectMillis;
    sinceLastConnect = sinceLastConnect > 0 ? sinceLastConnect : ((~((unsigned long)0) - lastConnectMillis) + millis());

    if(sinceLastConnect >= 10000)
    {
      lastConnectMillis = millis();
      
      Serial.print("Attempting MQTT connection...");
      if (client.connect(MQTT_CLINT_NAME, "openhabian", "swiatek123")) {
        Serial.println(" connected");
        
        client.subscribe(MQTT_SWITCH_SYBSCRIBE_TOPIC);
      }
      else
      {
        Serial.print("failed, rc=");
        Serial.print(client.state());
        Serial.println(" try again in 5 seconds");
      }
    }
    //return;
  }
  
  client.loop();

  long sinceLastCheck = millis() - lastMillis;
  sinceLastCheck = sinceLastCheck > 0 ? sinceLastCheck : ((~((unsigned long)0) - lastMillis) + millis());
  if(sinceLastCheck >= 500)
  {
    lastMillis = millis();
    checkAndPublish();
  }
  */

  /*uint8_t currVal = PCF8574_read();

  Serial.print("Read: ");
  Serial.println(currVal);
  */
  delay(1000);
  MCP27008_write(outputs);
  
  Serial.print("Write: ");
  Serial.println(outputs);

  //outputs += 1;
  outputs <<= 1;
  if(outputs == 0)
    outputs = 1;
}

