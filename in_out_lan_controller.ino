
/*
 Basic MQTT example
*/

#include <SPI.h>
//#include <Wire.h>
#include <Ethernet2.h>
#include <PubSubClient.h>
#include <MsTimer2.h>
#include <OneWire.h>
#include <EEPROM.h>

#define SOFT_VER "1.0.1"

#define SDA_PORT PORTC
#define SDA_PIN 4
#define SCL_PORT PORTC
#define SCL_PIN 5

#define I2C_TIMEOUT 100
#define I2C_FASTMODE 1

#include <SoftWire.h>

SoftWire Wire = SoftWire();

#define EEPROM_VERSION_OFFSET  0
#define EEPROM_SETTINGS_OFFSET 1
#define EEPROM_VERSION         0x55

#define ETH_SHIELD_RESET_PIN   A0

static struct StoredSettings{
  uint8_t  ver;
  uint8_t  mqtt_ip[4];
  uint16_t mqtt_port;
  char     mqtt_username[32];
  char     mqtt_password[16];
  // 0 - input changes output, output is published and subscribed
  // 1 - input and output independent, input is published, output is published and subscribed
  byte     mode;
} boardSettings;

#define set_bit(var, bit_nr) ((var) |= 1 << (bit_nr))
#define clear_bit(var, bit_nr) ((var) &= ~(1 << (bit_nr)))
#define get_bit(var, bit_nr) (((var) & (1 << (bit_nr))) ? true : false)
/*
const char hex_to_char[] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
inline void toHexStr (byte b, char* target)
{
  target[0] = hex_to_char[b>>4];
  target[1] = hex_to_char[b&0x0F];
}*/
inline void toHexStr (byte b, char* target)
{
  target[0] = (b>>4) > 9 ? (b>>4)-10 + 'A' : (b>>4) + '0';
  target[1] = (b&0x0F) > 9 ? (b&0x0F)-10 + 'A' : (b&0x0F) + '0';
}

static const uint8_t TOPIC_ID_START_INDEX = 6;

static const uint8_t TOPIC_CMD_CHANNEL_INDEX = 27;
char outputCommandTopic[]     = { "RELIO/\0\0\0\0\0\0\0\0/command/out/+\0" };

static const uint8_t TOPIC_IN_STATE_CHANNEL_INDEX = 24;
char inputStateTopic[]  = { "RELIO/\0\0\0\0\0\0\0\0/state/in/ \0"  };

static const uint8_t TOPIC_OUT_STATE_CHANNEL_INDEX = 25;
char outputStateTopic[] = { "RELIO/\0\0\0\0\0\0\0\0/state/out/ \0"  };

// Update these with values suitable for your network.
byte mac[] = { 0x6C, 0x75, 0x00, 0x00, 0x00, 0x00, 0x00 };

EthernetServer ethServer(80);
EthernetClient remoteClient;
PubSubClient   mqttClient(remoteClient);
OneWire        ds2401(A1);

static const uint8_t NUM_IOS = 8;
static const uint8_t INPUT_HIGH_STATE = 0xFF;
static const uint8_t INPUT_LOW_STATE = 0x00;
static const uint8_t INPUT_PINS_START = 2;
static const uint8_t MCP27008_ADRESS = 0x27;

static  uint8_t inputCounters[NUM_IOS] = {0, 0, 0, 0, 0, 0, 0, 0};
static  byte    inputsState = 0;
static  byte    inputsStateToPublish = 0;
static  byte    lastinputsState = 0;
static  byte    outputsState = 0;
static  byte    outputsStateToPublish = 0;

void MCP27008_setup()
{
  uint8_t error;
  Wire.beginTransmission(MCP27008_ADRESS);      // start talking to the device
  Wire.write(0x00);                   // select the IODIR register
  Wire.write(0x00);                   // set register value-all low, sets all pins as outputs on MCP23008
  error = Wire.endTransmission();             // stop talking to the devicevice
  if(error != 0)
  {
    Serial.print(F("MCP setup er: "));
    Serial.println(error);
  }
}
uint8_t MCP27008_write(uint8_t value)
{
  uint8_t error;
  Wire.beginTransmission(MCP27008_ADRESS);
  Wire.write(0x09);                   // select the GPIO register
  Wire.write(value);                  // set register value
  error = Wire.endTransmission();
  if(error != 0)
  {
    Serial.print(F("MCP write er: "));
    Serial.println(error);
  }
  return error;
}

void setOutputState(int index, bool state)
{
  byte currentState = outputsState;
  if(state)
  {
    set_bit(outputsState, index);
  }
  else
  {
    clear_bit(outputsState, index);
  }
  if(outputsState != currentState)
  {
    set_bit(outputsStateToPublish, index);
    //Serial.print("O: ");
    //Serial.println(outputsState, BIN);
    MCP27008_write(outputsState);
  }
}
void toggleOutputState(int index)
{
  setOutputState(index, !get_bit(outputsState, index));
}
void callback(char* topic, byte* payload, unsigned int length)
{
  int8_t relayPin = 0;
  Serial.print("Msg: ");
  Serial.println(topic);
//  Serial.print("[");
//  int i=0;
//  for (i=0;i<length;i++) {
//    Serial.print((char)payload[i]);
//  }
//  Serial.println("]");

  relayPin = topic[TOPIC_CMD_CHANNEL_INDEX] - '0';

  if(relayPin < 1 || relayPin > NUM_IOS)
  {
    Serial.println("Wrg pin");
    return;
  }
  if(length == 2 && memcmp(payload, "ON", 2) == 0)
  {
    setOutputState(relayPin-1, true );
  }
  else if(length == 3 && memcmp(payload, "OFF", 3) == 0)
  {
    setOutputState(relayPin-1, false );
  }
  else if(length == 6 && memcmp(payload, "TOGGLE", 6) == 0)
  {
    toggleOutputState(relayPin-1);
  }
  else
  {
    Serial.println(F("Wrg payload"));
  }
}

void readInputs()
{
  for(uint8_t i = 0; i < NUM_IOS; ++i)
  {
    inputCounters[i] <<= 1;
    if(digitalRead(INPUT_PINS_START + i) == LOW)
    {
      inputCounters[i] += 1;
    }
    else
    {
      inputCounters[i] += 0;
    }
    if(inputCounters[i] == INPUT_HIGH_STATE)
    {
      set_bit(inputsState, i);
    }
    else if(inputCounters[i] == INPUT_LOW_STATE)
    {
      clear_bit(inputsState, i);
    }
    if(get_bit(inputsState, i) != get_bit(lastinputsState, i))
    {
      if(!get_bit(boardSettings.mode, i))
      {
        if(get_bit(inputsState, i))
        {
          toggleOutputState(i);
        }
      }
      else
      {  
        set_bit(inputsStateToPublish, i);
      }
    }
  }
  lastinputsState = inputsState;
}

void checkOutputsAndPublish(PubSubClient& client)
{
  noInterrupts();
  byte currentOutputsStateToPublish = outputsStateToPublish;
  byte currentOutputsState = outputsState;
  outputsStateToPublish = 0;
  interrupts();
  
  if(currentOutputsStateToPublish == 0)
    return;
  
  for(uint8_t i = 0; i < NUM_IOS; ++i)
  {
    if(get_bit(currentOutputsStateToPublish, i))
    {
      outputStateTopic[TOPIC_OUT_STATE_CHANNEL_INDEX] = i + '1';
      Serial.print(outputStateTopic);
      if(get_bit(currentOutputsState, i))
      {
        Serial.println(": ON");
        client.publish((const char*)outputStateTopic, "ON", true);
      }
      else
      {
        Serial.println(": OFF");
        client.publish((const char*)outputStateTopic, "OFF", true);
      }
    }
  }
}
void checkInputsAndPublish(PubSubClient& client)
{
  noInterrupts();
  byte currentInputsStateToPublish = inputsStateToPublish;
  byte currentInputsState = inputsState;
  inputsStateToPublish = 0;
  interrupts();
  
  if(currentInputsStateToPublish == 0)
    return;
  
  for(uint8_t i = 0; i < NUM_IOS; ++i)
  {
    if(get_bit(boardSettings.mode, i) && get_bit(currentInputsStateToPublish, i))
    {
      inputStateTopic[TOPIC_IN_STATE_CHANNEL_INDEX] = i + '1';
      Serial.print(inputStateTopic);
      if(get_bit(currentInputsState, i))
      {
        Serial.println(": ON");
        client.publish((const char*)inputStateTopic, "ON");
      }
      else
      {
        Serial.println(": OFF");
        client.publish((const char*)inputStateTopic, "OFF");
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
  int8_t i;           // This is for the for loops
  //byte crc_calc;    //calculated CRC
  //byte crc_byte;    //actual CRC as sent by ds24012401
  //1-Wire bus reset, needed to start operation on the bus,
  //returns a 1/TRUE if presence pulse detected
  if (ds2401.reset() == TRUE)
  {
    ds2401.write(0x33);  //Send Read data command
    //Serial.print("FC: 0x");
    //PrintTwoDigitHex (ds2401.read(), 1);
    //Serial.print("HD: ");
    ds2401.read();
    for (i = 3; i >= 0; i--)
    {
      target[i+2] = ds2401.read();
      toHexStr(target[i+2], outputCommandTopic + (TOPIC_ID_START_INDEX + i*2));
      toHexStr(target[i+2], inputStateTopic + (TOPIC_ID_START_INDEX + i*2));
      toHexStr(target[i+2], outputStateTopic + (TOPIC_ID_START_INDEX + i*2));
    }
    Serial.print("MAC: ");
    for (i = 0; i < 6; i++)
    {
      PrintTwoDigitHex (target[i], 0);
      if(i< 5)Serial.print(":");
    }
    Serial.println();
    Serial.println(outputCommandTopic);
    Serial.println(inputStateTopic);
    Serial.println(outputStateTopic);
  }
  else //Nothing is connected in the bus
  {
    Serial.println(F("No MAC"));
  }
}

void setup()
{
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
  Serial.print(F("RELIO ver: "));
  Serial.println(SOFT_VER);
 
  for(uint8_t i = 0; i < NUM_IOS; ++i)
  {
    pinMode(INPUT_PINS_START + i, INPUT_PULLUP);      // sets the switch sensor digital pin as input
  }
  
  pinMode(ETH_SHIELD_RESET_PIN, OUTPUT);
  
  getMacAddress(mac);

  if(EEPROM.read(EEPROM_VERSION_OFFSET) != EEPROM_VERSION)
  {
    Serial.println(F("Clearing!"));
    memset(&boardSettings, 0, sizeof(boardSettings));
    EEPROM.write(EEPROM_VERSION_OFFSET, EEPROM_VERSION);
    EEPROM.put(EEPROM_SETTINGS_OFFSET, boardSettings);
  }
  
  EEPROM.get(EEPROM_SETTINGS_OFFSET, boardSettings);
  
  for(uint8_t i = 0; i < 4; ++i)
  {
    Serial.print(boardSettings.mqtt_ip[i]);
    if(i < 3)Serial.print(".");
  }
  Serial.print(":");
  Serial.println(boardSettings.mqtt_port);
  
  Serial.print(boardSettings.mqtt_username);
  Serial.print(":");
  Serial.println(boardSettings.mqtt_password);
  
  Wire.begin();
  
  MCP27008_setup();
  MCP27008_write(outputsState);
  
  MsTimer2::set(2, readInputs);
  MsTimer2::start();

/*
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
      Serial.print("I2C at 0x");
      if (address<16) 
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("!");
  
      nDevices++;
    }
    else if (error==4)
    {
      Serial.print("Err at 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }
  }
*/
  // Enable eth module.
  digitalWrite(A0, HIGH);

  mqttClient.setCallback(callback);
  Serial.print("Eth: ");
  //Ethernet.begin(mac, IPAddress(192, 168, 1, 6), IPAddress(255, 255, 255, 0), IPAddress(192, 168, 1, 254));
  while(!Ethernet.begin(mac))
  {
    delay(1000);
  }
  ethServer.begin();

  Serial.println(Ethernet.localIP()); 
  //Serial.println(Ethernet.subnetMask());
  //Serial.println(Ethernet.gatewayIP());
  //Serial.println(Ethernet.dnsServerIP());
}

void handleMqttClient()
{
  static unsigned long lastMillis = 0;  
  static unsigned long lastConnectMillis = 0xFFFFFFFF;
  static long sinceLastConnect;
  
  if (!mqttClient.connected())
  {
    sinceLastConnect = millis() - lastConnectMillis;
    sinceLastConnect = sinceLastConnect > 0 ? sinceLastConnect : ((~((unsigned long)0) - lastConnectMillis) + millis());

    if(sinceLastConnect >= 10000)
    {
      mqttClient.setServer(boardSettings.mqtt_ip, boardSettings.mqtt_port);
      Serial.print(F("MQTT con..."));
      if (mqttClient.connect((const char*)(mac+2), boardSettings.mqtt_username, boardSettings.mqtt_password)) {
        Serial.print(F(" Con, Sub:"));
        Serial.println(outputCommandTopic);
        mqttClient.subscribe(outputCommandTopic);
      }
      else
      {
        Serial.print(F("Err, rc="));
        Serial.println(mqttClient.state());
      }
      lastConnectMillis = millis();
    }
  }
  else
  {
    checkOutputsAndPublish(mqttClient);
    checkInputsAndPublish(mqttClient);
    mqttClient.loop();
  }
}
void handleHttpServer()
{
  static const int PROGMEM srvBufferSize = 100;
  static char srvBuffer[101];
  static byte srvBufferPos = 0;
  static EthernetClient remoteClient;
  
  // listen for incoming clients
  remoteClient = ethServer.available();
  if (!remoteClient)
    return;
    
  while (remoteClient.connected()) {
    if (remoteClient.available()) {
      char c = remoteClient.read();

      if (c != '\n' && c != '\r') {
        //read char by char HTTP request
        if (srvBufferPos < srvBufferSize) {

          //store characters to string 
          srvBuffer[srvBufferPos] = c;
          ++srvBufferPos;
        } 
        else
        {
          srvBufferPos = 0;
        }
        continue;
      }
    
      srvBuffer[srvBufferPos] = '\0';
      ///////////////
      //Serial.println(srvBuffer);

      //now output HTML data header
      // Parse get request
      char* ipStr, *portStr, *pch, *userStr, *pwdStr, *modeStr;
      byte counter = 0;
      /////////////////////
      ipStr = strstr(srvBuffer, "ip=");
      portStr = strstr(srvBuffer, "port=");
      userStr = strstr(srvBuffer, "user=");
      pwdStr = strstr(srvBuffer, "pwd=");
      modeStr = strstr(srvBuffer, "mode=");
      if(ipStr != NULL)
      {
        ipStr += 3;
        Serial.print("ip: ");
        pch = strtok (ipStr, ".");
        while (ipStr != NULL && counter < 4)
        {
          Serial.print(pch);
          if(counter<3)Serial.print(".");
          else Serial.println("");
          boardSettings.mqtt_ip[counter] = atoi(pch);
          // go to next token
          pch = strtok (NULL, ".&");
          ++counter;
        }
      }
      if(portStr != NULL)
      {
        portStr += 5;
        Serial.print("port: ");
        pch = strtok (portStr, "&");
        Serial.println(pch);
        boardSettings.mqtt_port = atoi(pch);
      }
      if(userStr != NULL)
      {
        userStr += 5;
        Serial.print("user: ");
        pch = strtok (userStr, "&");
        Serial.println(pch);
        strcpy(boardSettings.mqtt_username, pch);
      }
      if(pwdStr != NULL)
      {
        pwdStr += 4;
        Serial.print("pwd: ");
        pch = strtok (pwdStr, "&");
        Serial.println(pch);
        strcpy(boardSettings.mqtt_password, pch);
      }
      if(modeStr != NULL)
      {
        modeStr += 5;
        Serial.print("mode: ");
        pch = strtok (modeStr, "&");
        Serial.println(pch);
        for(uint8_t i = 0; i < NUM_IOS; ++i)
        {
          if(pch[i] == '1')
            set_bit(boardSettings.mode, 7-i);
          else if(pch[i] == '0')
            clear_bit(boardSettings.mode, 7-i);
        }
      }
      if(ipStr || portStr || userStr || pwdStr || modeStr)
      {
        EEPROM.put(EEPROM_SETTINGS_OFFSET, boardSettings);
        mqttClient.disconnect();
      }
      // Respond with current configuration
      remoteClient.println(F("HTTP/1.1 200 OK"));
      remoteClient.println(F("Content-Type: text/html"));
      remoteClient.println("");
      remoteClient.println(F("<HTML>"));
      remoteClient.println(F("<HEAD>"));
      remoteClient.println(F("<TITLE>RELIO controller setup</TITLE>"));
      remoteClient.println(F("</HEAD>"));
      remoteClient.println(F("<BODY>"));

      remoteClient.println(F("<PRE>"));
      remoteClient.print(F("<H1>Settings:</H1>"));
      remoteClient.print("MQTT ip: \t");
      for(uint8_t i = 0; i < 4; ++i)
      {
        remoteClient.print(boardSettings.mqtt_ip[i]);
        if(i < 3)remoteClient.print(".");
        else remoteClient.println("");
      }
      remoteClient.print(F("MQTT port: \t"));
      remoteClient.println(boardSettings.mqtt_port);
      remoteClient.print(F("MQTT user: \t"));
      remoteClient.println((const char*)boardSettings.mqtt_username);
      remoteClient.print(F("MQTT pwd: \t"));
      remoteClient.println((const char*)boardSettings.mqtt_password);
      remoteClient.print(F("Outputs mode: \t"));
      for(int8_t i = 7; i >= 0; --i)
      {
        remoteClient.print(get_bit(boardSettings.mode, i) ? '1' : '0');
      }
      remoteClient.println(F("<BR>"));
      
      remoteClient.print(F("<H1>State:</H1>"));
      remoteClient.print(F("Soft version: \t\t"));
      remoteClient.println(SOFT_VER);
      remoteClient.print(F("Subscription: \t\t"));
      remoteClient.println(outputCommandTopic);
      remoteClient.print(F("MQTT connection state: \t"));
      remoteClient.println(mqttClient.state());
      
      remoteClient.println(F("</PRE>"));

      remoteClient.println(F("</FORM>"));
      remoteClient.println(F("<BR>"));
      remoteClient.println(F("</BODY>"));
      remoteClient.println(F("</HTML>"));
      
      delay(1);
      //stopping client
      remoteClient.stop();
      //clearing string for next read
      srvBufferPos = 0;
    }
  }
}
  
static unsigned long maintainLastMillis = 0xFFFFFFFF;
static long sinceLastMaintain;
static byte maintainRes;
void loop()
{
  sinceLastMaintain = millis() - maintainLastMillis;
  sinceLastMaintain = sinceLastMaintain > 0 ? sinceLastMaintain : ((~((unsigned long)0) - maintainLastMillis) + millis());
  if(sinceLastMaintain >= 5000)
  {
    maintainLastMillis = millis();
    maintainRes = Ethernet.maintain();
    if(maintainRes == 2 || maintainRes == 4)
    {
      Serial.print("Eth: ");
      Serial.println(Ethernet.localIP()); 
    }
    else
    {
      return;
    }
  }
  handleHttpServer();
  handleMqttClient();
}

