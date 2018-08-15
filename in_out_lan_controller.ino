
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

static struct StoredSettings{
  uint8_t  mqtt_ip[4];
  uint16_t mqtt_port;
  char     mqtt_username[32];
  char     mqtt_password[16];
  byte     mode; // 0 - input changes output, output is published and subscribed, 1 - input and output independent, input is published, output is subscribed
} mqttSettings;

#define set_bit(var, bit_nr) ((var) |= 1 << (bit_nr))
#define clear_bit(var, bit_nr) ((var) &= ~(1 << (bit_nr)))
#define get_bit(var, bit_nr) (((var) & (1 << (bit_nr))) ? true : false)

const char hex_to_char[]= {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};

#define MODULE_ID 1+130

#define IO_MODULE_NAME "io_2"

char TOPIC_TO_SUBSCRIBE[] = { "Dom/io_        \0" };
#define TOPIC_TO_SUBSCRIBE_ID_INDEX 7

char TOPIC_OUTPUT_STATE[28] = { "Dom/" IO_MODULE_NAME "/out_ \0" };
#define TOPIC_OUTPUT_STATE 13

// Update these with values suitable for your network.
byte mac[] = { 0x6C, 0x75, 0x1E, 0xD2, 0x00, MODULE_ID };

EthernetServer ethServer(80);
EthernetClient remoteClient;
PubSubClient   mqttClient(remoteClient);
OneWire        ds2401(A1);

static const int     NUM_IOS = 8;
static const uint8_t INPUT_HIGH_STATE = 0xFF;
static const uint8_t inputPins[NUM_IOS] = {2, 3, 4, 5, 6, 7, 8, 9};
static  uint8_t inputCounters[NUM_IOS] = {0, 0, 0, 0, 0, 0, 0, 0};
static  byte    inputStates = 0;
static  byte    lastInputStates = 0;
static  byte    outputsState = 0;
static  byte    outputChangedFlags = 0;

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
    //Serial.print(TOPIC_ROOM_LIGHT);
    //Serial.println(": ON");
    //client.publish(TOPIC_ROOM_LIGHT, "ON");
    outputsState |= outputIndexToBit[index];
  }
  else
  {
    //Serial.print(TOPIC_ROOM_LIGHT);
    //Serial.println(": OFF");
    //client.publish(TOPIC_ROOM_LIGHT, "OFF");
    outputsState &= ~(outputIndexToBit[index]);
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
  setOutputState(index, (outputsState & outputIndexToBit[index]) ? false : true);
}
void callback(char* topic, byte* payload, unsigned int length)
{
  int relayPin = 0;
  Serial.print("Msg. arrived: ");
  Serial.print(topic);
//  Serial.print("[");
//  int i=0;
//  for (i=0;i<length;i++) {
//    Serial.print((char)payload[i]);
//  }
//  Serial.println("]");

  relayPin = 0;//*(topic+MQTT_SWITCH_SYBSCRIBE_TOPIC_NR_INDEX) - '1';

  if(relayPin < 0 || relayPin >= NUM_IOS)
  {
    Serial.println("Wrg topic");
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
      inputStates[i] = true;
    }
    else if(inputCounters[i] == 0)
    {
      inputStates[i] = false;
    }
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

void checkAndPublish()
{
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
/*bool getWorkMode()
{
  int numLow = 0;
  for(int i = 0; i < NUM_IOS; ++i)
  {
    numLow += digitalRead(inputPins[i]) == LOW? 1 : 0;
  }
  return numLow > 2;
}*/

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
    Serial.println("MAC:");
    ds2401.write(0x33);  //Send Read data command
    data[0] = ds2401.read();
    Serial.print("FC: 0x");
    PrintTwoDigitHex (data[0], 1);
    Serial.print("HD: ");
    for (i = 1; i <= 6; i++)
    {
      data[i] = ds2401.read(); //store each byte in different position in array
      if(i < 5)
      {
        TOPIC_TO_SUBSCRIBE[(i-1)*2+TOPIC_TO_SUBSCRIBE_ID_INDEX] = hex_to_char[(data[i]>>4) & 0x0F];
        TOPIC_TO_SUBSCRIBE[(i-1)*2+1+TOPIC_TO_SUBSCRIBE_ID_INDEX] = hex_to_char[data[i] & 0x0F];
      }
      PrintTwoDigitHex (data[i], 0);
      Serial.print(" ");
    }
    Serial.println();
    /*crc_byte = ds2401.read(); //read CRC, this is the last byte
    crc_calc = OneWire::crc8(data, 7); //calculate CRC of the data
    Serial.print("CRC: 0x");
    PrintTwoDigitHex (crc_calc, 1);
    Serial.print("CRC: 0x");
    PrintTwoDigitHex (crc_byte, 1);
    */
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
  
  //getMacAddress(mac);
  
  EEPROM.get(0, mqttSettings);
  mqttSettings.mqtt_ip[0] = 192;
  mqttSettings.mqtt_ip[1] = 168;
  mqttSettings.mqtt_ip[2] = 1;
  mqttSettings.mqtt_ip[3] = 3;
  mqttSettings.mqtt_port = 1883;
  
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

  // Enable eth module.
  digitalWrite(A0, HIGH);

  mqttClient.setCallback(callback);
  Serial.println("Client:");
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
      Serial.println("  !");
  
      nDevices++;
    }
    else if (error==4)
    {
       Serial.print("Error at 0x");
      if (address<16)
        Serial.print("0");
        Serial.println(address,HEX);
    }
  }*/
}


uint8_t outputs= 0;

char srvBuffer[101];
int srvBufferPos = 0;
PROGMEM const int srvBufferSize = 100;

void handleHttpServer()
{
  /*static EthernetClient remoteClient;
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
      remoteClient.print("Port: ");
      remoteClient.print(mqttSettings.mqtt_port);
      remoteClient.println("<BR>");
      remoteClient.print("User: ");
      remoteClient.print((const char*)mqttSettings.mqtt_username);
      remoteClient.println("<BR>");
      remoteClient.print("Password: ");
      remoteClient.print((const char*)mqttSettings.mqtt_password);
      remoteClient.println("<BR>");

      remoteClient.println("</FORM>");
      remoteClient.println("<BR>");
      remoteClient.println("</BODY>");
      remoteClient.println("</HTML>");
      
      delay(1);
      //stopping client
      remoteClient.stop();
      char* addressStr, *portStr, *pch, *userStr, *pwdStr;
      byte counter = 0;
      /////////////////////
      addressStr = strstr(srvBuffer, "ip=");
      portStr = strstr(srvBuffer, "port=");
      userStr = strstr(srvBuffer, "user=");
      pwdStr = strstr(srvBuffer, "pwd=");
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
      if(addressStr != NULL || portStr != NULL || pch != NULL || userStr != NULL || pwdStr)
      {
        EEPROM.put(0, mqttSettings);
      }
      //clearing string for next read
      srvBufferPos = 0;
    }
  }*/
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
      if (mqttClient.connect(TOPIC_TO_SUBSCRIBE+4, mqttSettings.mqtt_username, mqttSettings.mqtt_password)) {
        Serial.println(" connected");
        
        mqttClient.subscribe(TOPIC_TO_SUBSCRIBE);
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

  /*long sinceLastCheck = millis() - lastMillis;
  sinceLastCheck = sinceLastCheck > 0 ? sinceLastCheck : ((~((unsigned long)0) - lastMillis) + millis());
  if(sinceLastCheck >= 100)
  {
    lastMillis = millis();
    checkAndPublish();
  }*/
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

