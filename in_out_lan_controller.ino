
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

#define SDA_PORT PORTC
#define SDA_PIN 4
#define SCL_PORT PORTC
#define SCL_PIN 5

#define I2C_TIMEOUT 100
#define I2C_FASTMODE 1

#include <SoftWire.h>

SoftWire Wire = SoftWire();

#define EEPROM_VER 0x55

static struct StoredSettings{
  uint8_t  ver;
  uint8_t  mqtt_ip[4];
  uint16_t mqtt_port;
  char     mqtt_username[32];
  char     mqtt_password[16];
  byte     mode; // 0 - input changes output, output is published and subscribed, 1 - input and output independent, input is published, output is subscribed
} mqttSettings;

#define set_bit(var, bit_nr) ((var) |= 1 << (bit_nr))
#define clear_bit(var, bit_nr) ((var) &= ~(1 << (bit_nr)))
#define get_bit(var, bit_nr) (((var) & (1 << (bit_nr))) ? true : false)

const char hex_to_char[] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
inline void toHexStr (byte b, char* target)
{
  target[0] = hex_to_char[b>>4];
  target[1] = hex_to_char[b&0x0F];
}

#define TOPIC_ID_START_INDEX 7
#define TOPIC_CHANNEL_START_INDEX 24

char TOPIC_CMD_TO_SUBSCRIBE[] = { "Dom/io/\0\0\0\0\0\0\0\0/kome/wy/+\0" };
char TOPIC_IN_TO_PUBLISH[]    = { "Dom/io/\0\0\0\0\0\0\0\0/stan/we/ \0"  };
char TOPIC_OUT_TO_PUBLISH[]   = { "Dom/io/\0\0\0\0\0\0\0\0/stan/wy/ \0"  };

// Update these with values suitable for your network.
byte mac[] = { 0x6C, 0x75, 0x00, 0x00, 0x00, 0x00, 0x00 };

EthernetServer ethServer(80);
EthernetClient remoteClient;
PubSubClient   mqttClient(remoteClient);
OneWire        ds2401(A1);

static const int     NUM_IOS = 8;
static const uint8_t INPUT_HIGH_STATE = 0xFF;
static const uint8_t INPUT_LOW_STATE = 0x00;
static const uint8_t inputPins[NUM_IOS] = {2, 3, 4, 5, 6, 7, 8, 9};
static  uint8_t inputCounters[NUM_IOS] = {0, 0, 0, 0, 0, 0, 0, 0};
static  byte    inputStates = 0;
static  byte    lastInputStates = 0;
static  byte    outputsState = 0;
static  byte    lastPublishedOutputsState = 0;

#define ETH_SHIELD_RESET_PIN A0

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
  int relayPin = 0;
  Serial.print("Msg: ");
  Serial.println(topic);
//  Serial.print("[");
//  int i=0;
//  for (i=0;i<length;i++) {
//    Serial.print((char)payload[i]);
//  }
//  Serial.println("]");

  relayPin = topic[TOPIC_CHANNEL_START_INDEX] - '0';

  if(relayPin < 1 || relayPin > NUM_IOS)
  {
    Serial.println("Wrg pin");
    return;
  }
  if(length == 2 && memcmp((char*)payload, "ON", 2) == 0)
  {
    setOutputState(relayPin-1, true );
  }
  else if(length == 3 && memcmp((char*)payload, "OFF", 3) == 0)
  {
    setOutputState(relayPin-1, false );
  }
  else
  {
    Serial.println("Wrg payload");
  }
}

void readInputs()
{
  for(int i = 0; i < NUM_IOS; ++i)
  {
    inputCounters[i] <<= 1;
    if(digitalRead(inputPins[i]) == LOW)
    {
      inputCounters[i] += 1;
    }
    else
    {
      inputCounters[i] += 0;
    }
    if(inputCounters[i] == INPUT_HIGH_STATE)
    {
      set_bit(inputStates, i);
    }
    else if(inputCounters[i] == INPUT_LOW_STATE)
    {
      clear_bit(inputStates, i);
    }
    if(get_bit(inputStates, i) != get_bit(lastInputStates, i))
    {
      if(get_bit(inputStates, i))
      {
        toggleOutputState(i);
      }
    }
  }
  lastInputStates = inputStates;
}

void checkOutputsAndPublish(PubSubClient& client)
{
  byte currentOutputsState = outputsState;
  if(lastPublishedOutputsState == currentOutputsState)
    return;
  
  for(int i = 0; i < NUM_IOS; ++i)
  {
    if(get_bit(lastPublishedOutputsState, i) != get_bit(currentOutputsState, i))
    {
      TOPIC_OUT_TO_PUBLISH[TOPIC_CHANNEL_START_INDEX] = i + '1';
      if(get_bit(outputsState, i))
      {
        Serial.print(TOPIC_OUT_TO_PUBLISH);
        Serial.println(": ON");
        client.publish((const char*)TOPIC_OUT_TO_PUBLISH, "ON");
      }
      else
      {
        Serial.print(TOPIC_OUT_TO_PUBLISH);
        Serial.println(": OFF");
        client.publish((const char*)TOPIC_OUT_TO_PUBLISH, "OFF");
      }
    }
  }
  lastPublishedOutputsState = currentOutputsState;
}
void checkInputsAndPublish(PubSubClient& client)
{
  
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
  byte crc_calc;    //calculated CRC
  byte crc_byte;    //actual CRC as sent by ds24012401
  //1-Wire bus reset, needed to start operation on the bus,
  //returns a 1/TRUE if presence pulse detected
  if (ds2401.reset() == TRUE)
  {
    ds2401.write(0x33);  //Send Read data command
    //Serial.print("FC: 0x");
    //PrintTwoDigitHex (ds2401.read(), 1);
    //Serial.print("HD: ");
    ds2401.read();
    Serial.print("MAC: ");
    PrintTwoDigitHex (target[0], 0);
    Serial.print(":");
    PrintTwoDigitHex (target[1], 0);
    Serial.print(":");
    for (i = 0; i < 4; i++)
    {
      target[i+2] = ds2401.read();
      toHexStr(target[i+2], TOPIC_CMD_TO_SUBSCRIBE + (TOPIC_ID_START_INDEX + i*2));
      toHexStr(target[i+2], TOPIC_IN_TO_PUBLISH + (TOPIC_ID_START_INDEX + i*2));
      toHexStr(target[i+2], TOPIC_OUT_TO_PUBLISH + (TOPIC_ID_START_INDEX + i*2));
      
      PrintTwoDigitHex (target[i+2], 0);
      if(i<3)Serial.print(":");
    }
    Serial.println();
    //Serial.println(TOPIC_TO_SUBSCRIBE);
    //Serial.println(TOPIC_IN_TO_PUBLISH);
    //Serial.println(TOPIC_OUT_TO_PUBLISH);
  }
  else //Nothing is connected in the bus
  {
    Serial.println( "No MAC");
  }
}

void setup()
{
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
 
  for(int i = 0; i < NUM_IOS; ++i)
  {
    pinMode(inputPins[i], INPUT_PULLUP);      // sets the switch sensor digital pin as input
  }
  
  pinMode(ETH_SHIELD_RESET_PIN, OUTPUT);
  
  getMacAddress(mac);
  

  if(EEPROM.read(0) != EEPROM_VER)
  {
    memset(&mqttSettings, 0, sizeof(mqttSettings));
//    mqttSettings.mqtt_ip[0] = 192;
//    mqttSettings.mqtt_ip[1] = 168;
//    mqttSettings.mqtt_ip[2] = 1;
//    mqttSettings.mqtt_ip[3] = 3;
//    mqttSettings.mqtt_port = 1883;
    EEPROM.write(0, EEPROM_VER);
    EEPROM.put(1, mqttSettings);
  }
  
  EEPROM.get(1, mqttSettings);
  
  for(int i = 0; i < 4; ++i)
  {
    Serial.print(mqttSettings.mqtt_ip[i]);
    if(i < 3)Serial.print(".");
  }
  Serial.print(":");
  Serial.println(mqttSettings.mqtt_port);
  
  Serial.print(mqttSettings.mqtt_username);
  Serial.print(":");
  Serial.println(mqttSettings.mqtt_password);
  
  Wire.begin();
  
  MCP27008_setup();
  MCP27008_write(outputsState);
  
  MsTimer2::set(2, readInputs);
  MsTimer2::start();

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
      mqttClient.setServer(mqttSettings.mqtt_ip, mqttSettings.mqtt_port);
      Serial.print("MQTT con...");
      if (mqttClient.connect((const char*)(mac+2), mqttSettings.mqtt_username, mqttSettings.mqtt_password)) {
        Serial.print(" conn: Sub:");
        Serial.println(TOPIC_CMD_TO_SUBSCRIBE);
        mqttClient.subscribe(TOPIC_CMD_TO_SUBSCRIBE);
      }
      else
      {
        Serial.print("Err, rc=");
        Serial.println(mqttClient.state());
      }
      lastConnectMillis = millis();
    }
  }
  else
  {
    mqttClient.loop();
  }
  checkOutputsAndPublish(mqttClient);
}
void handleHttpServer()
{
  static const int PROGMEM srvBufferSize = 100;
  static char srvBuffer[101];
  static int  srvBufferPos = 0;
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
      char* addressStr, *portStr, *pch, *userStr, *pwdStr, *modeStr;
      byte counter = 0;
      /////////////////////
      addressStr = strstr(srvBuffer, "ip=");
      portStr = strstr(srvBuffer, "port=");
      userStr = strstr(srvBuffer, "user=");
      pwdStr = strstr(srvBuffer, "pwd=");
      modeStr = strstr(srvBuffer, "mode=");
      if(addressStr != NULL)
      {
        addressStr += 3;
        Serial.print("IP:");
        pch = strtok (addressStr, ".");
        while (addressStr != NULL && counter < 4)
        {
          Serial.print(pch);
          if(counter<3)Serial.print(".");
          else Serial.println("");
          mqttSettings.mqtt_ip[counter] = atoi(pch);
          // go to next token
          pch = strtok (NULL, ".&");
          ++counter;
        }
      }
      if(portStr != NULL)
      {
        portStr += 5;
        Serial.print("PORT:");
        pch = strtok (portStr, "&");
        Serial.println(pch);
        mqttSettings.mqtt_port = atoi(pch);
      }
      if(userStr != NULL)
      {
        userStr += 5;
        Serial.print("USER:");
        pch = strtok (userStr, "&");
        Serial.println(pch);
        strcpy(mqttSettings.mqtt_username, pch);
      }
      if(pwdStr != NULL)
      {
        pwdStr += 4;
        Serial.print("PWD:");
        pch = strtok (pwdStr, "&");
        Serial.println(pch);
        strcpy(mqttSettings.mqtt_password, pch);
      }
      if(modeStr != NULL)
      {
        modeStr += 5;
        Serial.print("MODE:");
        pch = strtok (modeStr, "&");
        Serial.println(pch);
        for(int i = 0; i < NUM_IOS; ++i)
        {
          if(pch[i] == '1')
            set_bit(mqttSettings.mode, i);
          else if(pch[i] == '0')
            clear_bit(mqttSettings.mode, i);
        }
      }
      if(addressStr || portStr || userStr || pwdStr || modeStr)
      {
        EEPROM.put(0, mqttSettings);
      }
      // Respond with current configuration
      remoteClient.println("HTTP/1.1 200 OK");
      remoteClient.println("Content-Type: text/html");
      remoteClient.println();

      remoteClient.println("<HTML>");
      remoteClient.println("<HEAD>");
      remoteClient.println("<TITLE>In/Out controller setup</TITLE>");
      remoteClient.println("</HEAD>");
      remoteClient.println("<BODY>");

      remoteClient.println("<H1>MQTT settings</H1>");

      remoteClient.print("IP: ");
      for(int i = 0; i < 4; ++i)
      {
        remoteClient.print(mqttSettings.mqtt_ip[i]);
        if(i < 3)remoteClient.print(".");
      }
      remoteClient.println("<BR>");
      remoteClient.print("port: ");
      remoteClient.print(mqttSettings.mqtt_port);
      remoteClient.println("<BR>");
      remoteClient.print("user: ");
      remoteClient.print((const char*)mqttSettings.mqtt_username);
      remoteClient.println("<BR>");
      remoteClient.print("pwd: ");
      remoteClient.print((const char*)mqttSettings.mqtt_password);
      remoteClient.println("<BR>");
      remoteClient.print("mode: ");
      remoteClient.print(mqttSettings.mode, BIN);
      remoteClient.println("<BR>");

      remoteClient.println("</FORM>");
      remoteClient.println("<BR>");
      remoteClient.println("</BODY>");
      remoteClient.println("</HTML>");
      
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
void loop()
{
  static int res;
  sinceLastMaintain = millis() - maintainLastMillis;
  sinceLastMaintain = sinceLastMaintain > 0 ? sinceLastMaintain : ((~((unsigned long)0) - maintainLastMillis) + millis());
  if(sinceLastMaintain >= 5000)
  {
    maintainLastMillis = millis();
    res = Ethernet.maintain();
    if(res == 2 || res == 4)
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

