
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

#define SOFT_VER "1.1.0"

#define SDA_PORT PORTC
#define SDA_PIN 4
#define SCL_PORT PORTC
#define SCL_PIN 5

#define I2C_TIMEOUT 100
#define I2C_FASTMODE 1

#include <utils.hpp>
#include <http_server.hpp>

SoftWire       sWire = SoftWire();
EthernetServer ethServer(80);
EthernetClient remoteClient;
PubSubClient   mqttClient(remoteClient);
OneWire        ds2401(A1);

static const uint8_t EEPROM_VERSION_OFFSET  = 0;
static const uint8_t EEPROM_SETTINGS_OFFSET = 1;
static const uint8_t EEPROM_VERSION         = 0x55;

static const uint8_t ETH_SHIELD_RESET_PIN   = A0;

static const uint8_t NUM_IOS = 8;
static const uint8_t INPUT_HIGH_STATE = 0xFF;
static const uint8_t INPUT_LOW_STATE = 0x00;
static const uint8_t INPUT_PINS_START = 2;
static const uint8_t MCP27008_ADRESS = 0x27;

static struct StoredSettings{
  MqttSettings mqtt;
  // 0 - input changes output, output is published and subscribed
  // 1 - input and output independent, input is published, output is published and subscribed
  byte         mode[NUM_IOS];
} boardSettings;

static const uint8_t TOPIC_ID_START_INDEX = 6;

static const uint8_t TOPIC_CMD_CHANNEL_INDEX = 27;
char outputCommandTopic[]     = { "RELIO/\0\0\0\0\0\0\0\0/command/out/+\0" };

static const uint8_t TOPIC_IN_STATE_CHANNEL_INDEX = 24;
char inputStateTopic[]  = { "RELIO/\0\0\0\0\0\0\0\0/state/in/ \0"  };

static const uint8_t TOPIC_OUT_STATE_CHANNEL_INDEX = 25;
char outputStateTopic[] = { "RELIO/\0\0\0\0\0\0\0\0/state/out/ \0"  };

// Update these with values suitable for your network.
byte mac[] = { 0x6C, 0x75, 0x00, 0x00, 0x00, 0x00, 0x00 };

static  uint8_t inputCounters[NUM_IOS] = {0, 0, 0, 0, 0, 0, 0, 0};
static  byte    inputsState = 0;
static  byte    inputsStateToPublish = 0;
static  byte    lastinputsState = 0;
static  byte    outputsState = 0;
static  byte    outputsStateToPublish = 0;


bool httpReqHandler(char* data, uint16_t size)
{
  char *pch, *modeStr;
  modeStr = strstr(data, "mode=");
  if(modeStr != NULL)
  {
    modeStr += 5;
    Serial.print("mode: ");
    pch = strtok (modeStr, "&,");
    Serial.println(pch);
    for(uint8_t i = 0; i < NUM_IOS; ++i)
    {
      if(pch[i] == '1')
        boardSettings.mode[i]= 1;
      else if(pch[i] == '0')
        boardSettings.mode[i] = 0;
    }
    return true;
  }
  return false;
}
void httpRespBuilder(EthernetClient& client)
{
  client.print(F("Outputs mode: \t"));
  for(uint8_t i = 0; i < NUM_IOS; ++i)
  {
    client.print(boardSettings.mode[i]);
    if(i+1 < NUM_IOS)client.print(F(", "));
  }
  client.println(F("<BR>"));
  
  client.print(F("<H1>State:</H1>"));
  client.print(F("Soft version: \t\t"));
  client.println(SOFT_VER);
  client.print(F("Subscription: \t\t"));
  client.println(outputCommandTopic);
  client.print(F("MQTT connection state: \t"));
  client.println(mqttClient.state());
  client.print(F("Uptime: \t\t"));
  client.println(millis()); 
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
    MCP27008_write(sWire, MCP27008_ADRESS, outputsState);
  }
}
void toggleOutputState(int index)
{
  setOutputState(index, !get_bit(outputsState, index));
}
void callback(char* topic, byte* payload, unsigned int length)
{
  int8_t relayPin = 0;
  Serial.print(F("Rcv: "));
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
    Serial.println(F("Wrg pin"));
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

void readInputsInterruptHandler()
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
      if(!boardSettings.mode[i])
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
void publishMsg(PubSubClient& client, const char* topic, const char* payload)
{
  Serial.print(F("Pub: "));
  Serial.print(topic);
  Serial.print(F(": "));
  Serial.println(payload);

  client.publish((const char*)topic, payload, true);
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
      if(get_bit(currentOutputsState, i))
      {
        publishMsg(client, (const char*)outputStateTopic, "ON");
      }
      else
      {
        publishMsg(client, (const char*)outputStateTopic, "OFF");
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
    if(boardSettings.mode[i] && get_bit(currentInputsStateToPublish, i))
    {
      inputStateTopic[TOPIC_IN_STATE_CHANNEL_INDEX] = i + '1';
      Serial.print(inputStateTopic);
      if(get_bit(currentInputsState, i))
      {
        Serial.println(F(": ON"));
        client.publish((const char*)inputStateTopic, "ON");
      }
      else
      {
        Serial.println(F(": OFF"));
        client.publish((const char*)inputStateTopic, "OFF");
      }
    }
  }
}
/*
void PrintTwoDigitHex (byte b, boolean newline)
{
  Serial.print(b/16, HEX);
  Serial.print(b%16, HEX);
  if (newline) Serial.println();
}*/

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

  if(EEPROM.read(EEPROM_VERSION_OFFSET) != EEPROM_VERSION)
  {
    Serial.println(F("Clearing EEPROM!"));
    memset(&boardSettings, 0, sizeof(boardSettings));
    EEPROM.write(EEPROM_VERSION_OFFSET, EEPROM_VERSION);
    EEPROM.put(EEPROM_SETTINGS_OFFSET, boardSettings);
  }
  
  EEPROM.get(EEPROM_SETTINGS_OFFSET, boardSettings);

  getMacAddress(ds2401, mac);
  
  for (uint8_t i = 0; i < 4; i++)
  {
    byteToHexStr(mac[i+2], outputCommandTopic + (TOPIC_ID_START_INDEX + i*2));
    byteToHexStr(mac[i+2], inputStateTopic + (TOPIC_ID_START_INDEX + i*2));
    byteToHexStr(mac[i+2], outputStateTopic + (TOPIC_ID_START_INDEX + i*2));
  }
  Serial.println(outputCommandTopic);
  Serial.println(inputStateTopic);
  Serial.println(outputStateTopic);
  
  Serial.print(F("Mode: "));
  for(uint8_t i = 0; i < NUM_IOS; ++i)
  {
    Serial.print(boardSettings.mode[i]);
    if(i+1 < NUM_IOS)Serial.print(F(", "));
    else Serial.println(F(""));
  }
  
  for(uint8_t i = 0; i < 4; ++i)
  {
    Serial.print(boardSettings.mqtt.mqtt_ip[i]);
    if(i < 3)Serial.print(F("."));
  }
  Serial.print(F(":"));
  Serial.println(boardSettings.mqtt.mqtt_port);
  
  Serial.print(boardSettings.mqtt.mqtt_username);
  Serial.print(F(":"));
  Serial.println(boardSettings.mqtt.mqtt_password);
  
  sWire.begin();
  
  MCP27008_setup(sWire, MCP27008_ADRESS);
  MCP27008_write(sWire, MCP27008_ADRESS, outputsState);
  
  MsTimer2::set(2, readInputsInterruptHandler);
  MsTimer2::start();

  // Enable eth module.
  digitalWrite(A0, HIGH);

  mqttClient.setCallback(callback);
  Serial.print(F("Ethernet: "));
  //Ethernet.begin(mac, IPAddress(192, 168, 1, 6), IPAddress(255, 255, 255, 0), IPAddress(192, 168, 1, 254));
  while(!Ethernet.begin(mac))
  {
    delay(5000);
  }
  ethServer.begin();

  Serial.println(Ethernet.localIP()); 
  //Serial.println(Ethernet.subnetMask());
  //Serial.println(Ethernet.gatewayIP());
  //Serial.println(Ethernet.dnsServerIP());
}

static unsigned long lastConnectMillis = 0;
void handleMqttClient()
{
  if (!mqttClient.connected())
  {
    if(calcTimestampDiff(lastConnectMillis, millis()) >= 10000)
    {
      lastConnectMillis = millis();
      
      mqttClient.setServer(boardSettings.mqtt.mqtt_ip, boardSettings.mqtt.mqtt_port);
      Serial.print(F("MQTT connecting..."));
      if (mqttClient.connect((const char*)mac, boardSettings.mqtt.mqtt_username, boardSettings.mqtt.mqtt_password))
      {
        Serial.print(F(" Connected, sub= "));
        Serial.println(outputCommandTopic);
        mqttClient.subscribe(outputCommandTopic);
      }
      else
      {
        Serial.print(F(" Disconnected, err="));
        Serial.println(mqttClient.state());
      }
    }
  }
  else
  {
    checkOutputsAndPublish(mqttClient);
    checkInputsAndPublish(mqttClient);
    mqttClient.loop();
  }
}

static unsigned long maintainLastMillis = 0;
static byte maintainRes, connectionFlag;
void loop()
{
  if(calcTimestampDiff(maintainLastMillis, millis()) >= 10000)
  {
    maintainLastMillis = millis();
    
    maintainRes = Ethernet.maintain();
    if(maintainRes == 2 || maintainRes == 4)
    {
      connectionFlag = 1;
      Serial.print(F("Ethernet: "));
      Serial.println(Ethernet.localIP()); 
    }
    else
    {
      if(connectionFlag)
      {
        Serial.println(F("Ethernet: disconnected"));
      }
      connectionFlag = 0;
      return;
    }
  }
  
  handleMqttClient();
  
  HttpResult httpRes = httpHandle(ethServer, boardSettings.mqtt, httpReqHandler, httpRespBuilder);
  if(httpRes != HTTP_NO_ACTION)
  {
    EEPROM.put(EEPROM_SETTINGS_OFFSET, boardSettings);
    Serial.println(F("EEPROM updated")); 
    if(httpRes & HTTP_MQTT_CHANGE)
    {
      mqttClient.disconnect();
      Serial.println(F("MQTT disconnected"));
    }
  }
}

