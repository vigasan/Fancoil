/**************************************************************************************************************************************************
* File name     : FanCoil.c
* Compiler      : 
* Autor         : VIGASAN   
* Created       : 13/04/2023
* Modified      : 
* Last modified :
*
*
* Description   : 
*
* Other info    : 
**************************************************************************************************************************************************/


/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------Include Files----------------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
#include <OneWire.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <PubSubClient.h> // You need to change the value of costant MQTT_MAX_PACKET_SIZE to 1024 in the file PubSubClient.h 
                          // because the MQTT payload could exceed standard value of 256


/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------  Constants  ----------------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
#define UNPRESSED       0
#define DEBOUNCE        1
#define PRESSED         2

#define OFF             0
#define ON              1

#define RELAY_OFF       0
#define RELAY_ON        1

#define MODE_OFF        0
#define MODE_COOL       1
#define MODE_HEAT       2
#define MODE_FAN_ONLY   3

#define FAN_OFF         0
#define FAN_LOW         1
#define FAN_MEDIUM      2
#define FAN_HIGH        3

#define DAC_LEVEL_FAN_OFF       0
#define DAC_LEVEL_FAN_LOW       50
#define DAC_LEVEL_FAN_MEDIUM    100
#define DAC_LEVEL_FAN_HIGH      250

#define DELTA_HISTERESYS_TEMP           0.1



/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------I/O Definitions--------------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
const int IN_DIN1 = 14;
const int IN_DIN2 = 13;
const int RELAY1 = 33;
const int RELAY2 = 27;
const int LED_R = 32;
const int LED_G = 15;
const int LED_B = 12;
const int LED_1 = 21;
const int LED_2 = 19;
const int LED_3 = 18;
const int DAC_1 = 26;
const int DAC_2 = 25;
const int DS18S20_PIN = 23;

/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------MQTT DISCOVERY PARAMETERS----------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
const char*     g_ssid = "your Wifi Name";                      // Wifi Name
const char*     g_password = "Wifi Password";                   // Wifi Password
const char*     g_mqtt_server = "192.168.1.25";                 // MQTT Server IP, same of Home Assistant
const char*     g_mqttUser = "mqttUser";                        // MQTT Server User Name
const char*     g_mqttPsw = "password";                         // MQTT Server password
int             g_mqttPort = 1883;                              // MQTT Server Port
const char*     g_mqtt_DeviceName = "FanCoilSalone";            // MQTT Device Name

// Variable used for MQTT Discovery
const char*		g_deviceModel = "FCBoard";                        // Hardware Model
const char*		g_swVersion = "1.0";                              // Firmware Version
const char*		g_manufacturer = "Vigasan";                       // Manufacturer Name
String			  g_deviceName = "FCSalone";                        // Device Name

String        g_ClimateArea = "sal";
String        g_mqtt_Topic_Area = "FC/" + g_ClimateArea;
String      	g_TopicProbeTemperature = "FC/temp";		          // Topic for entity temperature

/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------Global variables-------------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
WiFiClient          g_WiFiClient;
PubSubClient        g_mqttPubSub(g_WiFiClient);
OneWire             g_tempSensor(DS18S20_PIN);

unsigned long       g_TimeInputs = 0;
unsigned long       g_TimeLed = 0;
unsigned long       g_TimeTemperature = 0;
int                 g_Mode = MODE_OFF;
int                 g_Fan = FAN_OFF;
int                 g_mqttCounterConn = 0;
String              g_UniqueId;
bool                g_InitSystem = true;
int                 g_canPublish = 0;

byte                g_RelayStatus1 = RELAY_OFF;
byte                g_RelayStatus2 = RELAY_OFF;
byte                g_Input1 = 0;
byte                g_Input2 = 0;
byte				        g_AnOut1 = 0;
byte				        g_AnOut2 = 0;
float               g_TargetTemperature = 15.0;

byte                g_st_input1 = UNPRESSED;
byte                g_st_input2 = UNPRESSED;

int                 g_HeatingStatus = OFF;
int                 g_CoolingStatus = OFF;
int                 g_FanOnlyStatus = OFF;
float               g_ProbeTemperature = 15.0;

/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------ SETUP ----------------------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
void setup() 
{
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // I/O Configuration
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    pinMode(IN_DIN1, INPUT);
    pinMode(IN_DIN2, INPUT);
    pinMode(RELAY1, OUTPUT);
    pinMode(RELAY2, OUTPUT);
    pinMode(LED_R, OUTPUT);
    pinMode(LED_G, OUTPUT);
    pinMode(LED_B, OUTPUT);
    pinMode(LED_1, OUTPUT);
    pinMode(LED_2, OUTPUT);
    pinMode(LED_3, OUTPUT);
    
    dacWrite(DAC_1, g_AnOut1);
    dacWrite(DAC_2, g_AnOut2);

    Serial.begin(115200);
    delay(500); 
	
    Serial.println("");
    Serial.println("----------------------------------------------");
    Serial.print("MODEL: ");
    Serial.println(g_deviceModel);
    Serial.print("DEVICE: ");
    Serial.println(g_deviceName);
    Serial.print("SW Rev: ");
    Serial.println(g_swVersion);
    Serial.println("----------------------------------------------");
  
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Wifi Connection
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    setup_wifi();

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // MQTT Configuration
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    g_mqttPubSub.setServer(g_mqtt_server, g_mqttPort);
    g_mqttPubSub.setCallback(MqttReceiverCallback);
}

/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------ LOOP -----------------------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
void loop() 
{
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // MQTT Connection
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if(WiFi.status() == WL_CONNECTED)
    {
        if(!g_mqttPubSub.connected())
            MqttReconnect();
        else
            g_mqttPubSub.loop();
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // MQTT Discovery Init
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if(g_InitSystem)
    {
        delay(1000);
        g_InitSystem = false;
		    Serial.println("INIT SYSTEM...");
        MqttHomeAssistantDiscovery();    
       
    }
	
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Area Temperature Management
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if(g_Mode == MODE_HEAT)
    {
        if(g_HeatingStatus == OFF)
        {
            if(g_ProbeTemperature < (g_TargetTemperature - DELTA_HISTERESYS_TEMP))
            {
                digitalWrite(RELAY1, RELAY_ON);
                g_HeatingStatus = ON;
            }
        } else if(g_HeatingStatus == ON)
        {
            if(g_ProbeTemperature > (g_TargetTemperature + DELTA_HISTERESYS_TEMP))
            {
                digitalWrite(RELAY1, RELAY_OFF);
                g_HeatingStatus = OFF;
            }
        }
    } else if(g_Mode == MODE_COOL)
    {
        if(g_CoolingStatus == OFF)
        {
            if(g_ProbeTemperature > (g_TargetTemperature + DELTA_HISTERESYS_TEMP))
            {
                digitalWrite(RELAY1, RELAY_ON);
                g_CoolingStatus = ON;
            }
        } else if(g_CoolingStatus == ON)
        {
            if(g_ProbeTemperature < (g_TargetTemperature - DELTA_HISTERESYS_TEMP))
            {
                digitalWrite(RELAY1, RELAY_OFF);
                g_CoolingStatus = OFF;
            }
        }
    } else if(g_Mode == MODE_FAN_ONLY)
    {
        digitalWrite(RELAY1, RELAY_OFF);
        g_CoolingStatus = OFF;
        g_HeatingStatus = OFF;
        g_FanOnlyStatus = ON;
    } else if(g_Mode == MODE_OFF)
    {
        digitalWrite(RELAY1, RELAY_OFF);
        g_CoolingStatus = OFF;
        g_HeatingStatus = OFF;
        g_FanOnlyStatus = OFF;
    }
    
   

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Inputs Buttons Monitor
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if(millis() - g_TimeInputs > 100)
    {
        g_TimeInputs = millis();

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // BUTTON 2: SET MODE
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        switch(g_st_input1)
        {
            case UNPRESSED:
            {
                if(digitalRead(IN_DIN1) == 0)            
                {
                    g_st_input1 = DEBOUNCE;
                }
            } break;
    
            case DEBOUNCE:
            {
                if(digitalRead(IN_DIN1) == 0)           
                {
                    g_st_input1 = PRESSED;

                    g_HeatingStatus = OFF;
                    g_CoolingStatus = OFF;
                    g_FanOnlyStatus = OFF;
                    digitalWrite(RELAY1, RELAY_OFF);

                    switch(g_Mode)
                    {
                        case MODE_OFF:
                        {
                            g_Mode = MODE_COOL;
                        } break;

                        case MODE_COOL:
                        {
                            g_Mode = MODE_HEAT;
                        } break;

                        case MODE_HEAT:
                        {
                            g_Mode = MODE_FAN_ONLY;
                        } break;

                        case MODE_FAN_ONLY:
                        {
                            g_Mode = MODE_OFF;
                            g_Fan = FAN_OFF;
                            MqttPublishFanState();
                        } break;
                    }
                    
                    MqttPublishModeState();
                } else                                  
                {
                    g_st_input1 = UNPRESSED;
                                         
                }
            } break;
    
            case PRESSED:
            {
                if(digitalRead(IN_DIN1) == 1)
                {
                    g_st_input1 = DEBOUNCE;
                }
            } break;
        }
		
		    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // BUTTON 2: SET FAN SPEED
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        switch(g_st_input2)
        {
            case UNPRESSED:
            {
                if(digitalRead(IN_DIN2) == 0)            
                {
                    g_st_input2 = DEBOUNCE;
                }
            } break;
    
            case DEBOUNCE:
            {
                if(digitalRead(IN_DIN2) == 0)           
                {
                    g_st_input2 = PRESSED;

                    switch(g_Fan)
                    {
                        case FAN_OFF:
                        {
                            g_Fan = FAN_LOW;
                        } break;

                        case FAN_LOW:
                        {
                            g_Fan = FAN_MEDIUM;
                        } break;

                        case FAN_MEDIUM:
                        {
                            g_Fan = FAN_HIGH;
                        } break;

                        case FAN_HIGH:
                        {
                            g_Fan = FAN_OFF;
                        } break;
                    }
                    digitalWrite(LED_1, 0);
                    digitalWrite(LED_2, 0);
                    digitalWrite(LED_3, 0);
                    MqttPublishFanState();
                } else                                  
                {
                    g_st_input2 = UNPRESSED;
                      
                }
            } break;
    
            case PRESSED:
            {
                if(digitalRead(IN_DIN2) == 1)
                {
                    g_st_input2 = DEBOUNCE;
                }
            } break;
        }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Reading Probe Temperature
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if(millis() - g_TimeTemperature > 10000)
    {
        g_TimeTemperature = millis();
        g_ProbeTemperature = GetTemperature();
        MqttPublishTemperature();
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // LEDs Status and FAN Management
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if(millis() - g_TimeLed > 500)
    {
        g_TimeLed = millis();
        
        if(g_Mode == MODE_OFF)
        {
            digitalWrite(LED_R, 0);
            digitalWrite(LED_G, 0);
            digitalWrite(LED_B, 0);
        } else if(g_Mode == MODE_COOL)
        {
            digitalWrite(LED_R, 0);
            digitalWrite(LED_G, 0);
            digitalWrite(LED_B, !digitalRead(LED_B));
        } else if(g_Mode == MODE_HEAT)
        {
            digitalWrite(LED_R, !digitalRead(LED_R));
            digitalWrite(LED_G, 0);
            digitalWrite(LED_B, 0);
        }  else if(g_Mode == MODE_FAN_ONLY)
        {
            digitalWrite(LED_R, 0);
            digitalWrite(LED_G, !digitalRead(LED_G));
            digitalWrite(LED_B, 0);
        }

        switch(g_Fan)
        {
            case FAN_OFF:
            {
                dacWrite(DAC_1, DAC_LEVEL_FAN_OFF);
                digitalWrite(LED_1, 0);
                digitalWrite(LED_2, 0);
                digitalWrite(LED_3, 0);
            } break;
    
            case FAN_LOW:
            {
                if((g_HeatingStatus + g_CoolingStatus + g_FanOnlyStatus) > 0)
                {
                    digitalWrite(LED_1, 1);
                    dacWrite(DAC_1, DAC_LEVEL_FAN_LOW);
                } else
                {
                    digitalWrite(LED_1, !digitalRead(LED_1));
                    dacWrite(DAC_1, DAC_LEVEL_FAN_OFF);
                }
                digitalWrite(LED_2, 0);
                digitalWrite(LED_3, 0);
            } break;
    
            case FAN_MEDIUM:
            {
                if((g_HeatingStatus + g_CoolingStatus + g_FanOnlyStatus) > 0)
                {
                    digitalWrite(LED_1, 1);
                    digitalWrite(LED_2, 1);
                    dacWrite(DAC_1, DAC_LEVEL_FAN_MEDIUM);
                } else
                {
                    digitalWrite(LED_1, !digitalRead(LED_1));
                    digitalWrite(LED_2, !digitalRead(LED_2));
                    dacWrite(DAC_1, DAC_LEVEL_FAN_OFF);
                }
                digitalWrite(LED_3, 0);
            } break;
    
            case FAN_HIGH:
            {
                if((g_HeatingStatus + g_CoolingStatus + g_FanOnlyStatus) > 0)
                {
                    digitalWrite(LED_1, 1);
                    digitalWrite(LED_2, 1);
                    digitalWrite(LED_3, 1);
                    dacWrite(DAC_1, DAC_LEVEL_FAN_HIGH);
                } else
                {
                    digitalWrite(LED_1, !digitalRead(LED_1));
                    digitalWrite(LED_2, !digitalRead(LED_2));
                    digitalWrite(LED_3, !digitalRead(LED_3));
                    dacWrite(DAC_1, DAC_LEVEL_FAN_OFF);
                }
            } break;
        }
    }
}


/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------ Public Functions -----------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
void setup_wifi() 
{
    int counter = 0;
    byte mac[6];
    delay(10);


    WiFi.begin(g_ssid, g_password);

    WiFi.macAddress(mac);
    g_UniqueId =  String(mac[0],HEX) + String(mac[1],HEX) + String(mac[2],HEX) + String(mac[3],HEX) + String(mac[4],HEX) + String(mac[5],HEX); 
    
    while(WiFi.status() != WL_CONNECTED && counter++ < 5) 
    {
        delay(500);
    }

}

void MqttReconnect() 
{
    // Loop until we're MqttReconnected
    while (!g_mqttPubSub.connected()  && (g_mqttCounterConn++ < 4))
    {
        Serial.print("Attempting MQTT connection...");
        if (g_mqttPubSub.connect(g_mqtt_DeviceName, g_mqttUser, g_mqttPsw)) 
        {
            Serial.println("connected");
            //ESP32 Subscribe following topics
            // Home assistant status
            g_mqttPubSub.subscribe("homeassistant/status");

            g_mqttPubSub.subscribe((g_mqtt_Topic_Area + "/set_mode").c_str());
            g_mqttPubSub.subscribe((g_mqtt_Topic_Area + "/set_fan").c_str());
            g_mqttPubSub.subscribe((g_mqtt_Topic_Area + "/set_temp").c_str());
            delay(500);
        } else 
        {
            Serial.print("failed, rc=");
            Serial.print(g_mqttPubSub.state());
            Serial.println(" try again in 3 seconds");
            delay(3000);
        }
    }  
    g_mqttCounterConn = 0;
}

void MqttHomeAssistantDiscovery()
{
    String discoveryTopic;
    String payload;
    String strPayload;
    int uniqueId_increment = 0;
    if(g_mqttPubSub.connected())
    {
        StaticJsonDocument<800> payload;
        JsonArray modes;
        JsonArray fan_modes;
        JsonObject device;
        JsonArray identifiers;

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Climate Area
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////
        discoveryTopic = "homeassistant/climate/" + g_ClimateArea + "/config";

        uniqueId_increment++;
        payload["name"] = "clima." + g_ClimateArea;                         // Climate Entity Name          
        payload["uniq_id"] = g_UniqueId + "_" + String(uniqueId_increment); // Entity univoque Id
        payload["mode_cmd_t"] = g_mqtt_Topic_Area + "/set_mode";            // Set Mode Topic
        payload["mode_stat_t"] = g_mqtt_Topic_Area + "/state_mode";         // State mode Topic
        payload["fan_mode_cmd_t"] = g_mqtt_Topic_Area + "/set_fan";         // Set Fan Speed Topic
        payload["fan_mode_stat_t"] = g_mqtt_Topic_Area + "/state_fan";      // State Fan Speed Topic
        payload["temp_cmd_t"] = g_mqtt_Topic_Area + "/set_temp";            // Set Target Temperature Topic
        payload["temp_stat_t"] = g_mqtt_Topic_Area + "/state_temp";         // State Target Temperature Topic
        payload["curr_temp_t"] = g_TopicProbeTemperature;                   // Room Temperature Topic
        payload["min_temp"] = "13";
        payload["max_temp"] = "28";
        payload["temp_step"] = "0.1";
        modes = payload.createNestedArray("modes");
        modes.add("off");
        modes.add("cool");
        modes.add("heat");
        modes.add("fan_only");
        fan_modes = payload.createNestedArray("fan_modes");
        fan_modes.add("off");
        fan_modes.add("low");
        fan_modes.add("medium");
        fan_modes.add("high");
        device = payload.createNestedObject("device");
        device["name"] = g_deviceName;
        device["mdl"] = g_deviceModel;
        device["sw"] = g_swVersion;
        device["mf"] = g_manufacturer;
        identifiers = device.createNestedArray("ids");
        identifiers.add(g_UniqueId);
        
        serializeJsonPretty(payload, Serial);
        Serial.println(" ");
        serializeJson(payload, strPayload);
        g_mqttPubSub.publish(discoveryTopic.c_str(), strPayload.c_str());
        delay(500);
		
		    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Probe Temperature
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////
        discoveryTopic = "homeassistant/sensor/" + g_deviceName + "_temp/config"; // Discovery Topic for analog out 2
        payload.clear();
        modes.clear();
        fan_modes.clear();
        device.clear();
        identifiers.clear();
        strPayload.clear();
        
        uniqueId_increment++;
        payload["name"] = g_deviceName + ".temp";      
        payload["uniq_id"] = g_UniqueId + "_" + String(uniqueId_increment);
        payload["stat_t"] = g_TopicProbeTemperature;
        payload["dev_cla"] = "temperature";
        payload["unit_of_meas"] = "°C";
        device = payload.createNestedObject("device");
        device["name"] = g_deviceName;
        device["mdl"] = g_deviceModel;
        device["sw"] = g_swVersion;
        device["mf"] = g_manufacturer;
        identifiers = device.createNestedArray("ids");
        identifiers.add(g_UniqueId);

        serializeJsonPretty(payload, Serial);
        Serial.println(" ");
        serializeJson(payload, strPayload);
        g_mqttPubSub.publish(discoveryTopic.c_str(), strPayload.c_str());
        delay(500);
    }
}

void MqttReceiverCallback(char* topic, byte* inFrame, unsigned int length) 
{
    byte state = 0;
    String payload;
    String topicMsg;
    StaticJsonDocument<256> doc;
    
    for (int i = 0; i < length; i++) 
    {
        payload += (char)inFrame[i];
    }
    
    if(String(topic) == String("homeassistant/status")) 
    {
        if(payload == "online")
        {
            MqttHomeAssistantDiscovery();
        }
    }

    if(String(topic) == (g_mqtt_Topic_Area  + "/set_mode")) 
    {
        g_HeatingStatus = OFF;
        g_CoolingStatus = OFF;
        g_FanOnlyStatus = OFF;
        digitalWrite(RELAY1, RELAY_OFF);
        
        if(payload == "off")
        {
            g_Mode = MODE_OFF;
            g_Fan = FAN_OFF;
            MqttPublishFanState();
        } else if(payload == "cool")
            g_Mode = MODE_COOL;
        else if(payload == "heat")
            g_Mode = MODE_HEAT;
        else if(payload == "fan_only")
            g_Mode = MODE_FAN_ONLY;

        MqttPublishModeState();
    }

    if(String(topic) == (g_mqtt_Topic_Area  + "/set_fan")) 
    {
        if(payload == "off")
            g_Fan = FAN_OFF;
        else if(payload == "low")
            g_Fan = FAN_LOW;
        else if(payload == "medium")
            g_Fan = FAN_MEDIUM;
        else if(payload == "high")
            g_Fan = FAN_HIGH;

        digitalWrite(LED_1, 0);
        digitalWrite(LED_2, 0);
        digitalWrite(LED_3, 0);
        MqttPublishFanState();
    }

    if(String(topic) == (g_mqtt_Topic_Area  + "/set_temp")) 
    {
        g_TargetTemperature = payload.toFloat();
        topicMsg = g_mqtt_Topic_Area + "/state_temp";
        g_mqttPubSub.publish(topicMsg.c_str(), String(g_TargetTemperature).c_str());
    }
}

void MqttPublishModeState()
{
    String topicMsg;
    String payload;
    if(g_mqttPubSub.connected())
    {
        if(g_Mode == MODE_OFF)
            payload = "off";
        else if(g_Mode == MODE_COOL)
            payload = "cool";
        else if(g_Mode == MODE_HEAT)
            payload = "heat";
        else if(g_Mode == MODE_FAN_ONLY)
            payload = "fan_only";
        
        topicMsg = g_mqtt_Topic_Area + "/state_mode";
        g_mqttPubSub.publish(topicMsg.c_str(), payload.c_str());
    }
}

void MqttPublishFanState()
{
    String topicMsg;
    String payload;
    if(g_mqttPubSub.connected())
    {
        if(g_Fan == FAN_OFF)
            payload = "off";
        else if(g_Fan == FAN_LOW)
            payload = "low";
        else if(g_Fan == FAN_MEDIUM)
            payload = "medium";
        else if(g_Fan == FAN_HIGH)
            payload = "high";
        
        topicMsg = g_mqtt_Topic_Area + "/state_fan";
        g_mqttPubSub.publish(topicMsg.c_str(), payload.c_str());
    }
}

void MqttPublishTemperature() // Sens MQTT status for Probe Temperature
{
    String topicMsg;
    String payload;
    if(g_mqttPubSub.connected())
    {
        payload = g_ProbeTemperature;
        topicMsg = g_TopicProbeTemperature;
        g_mqttPubSub.publish(topicMsg.c_str(), payload.c_str());
    }
}

float GetTemperature()  // Get Temperature from DS18S20 probe
{
    byte data[12];
    byte addr[8];
  
    if ( !g_tempSensor.search(addr)) {
        //no more sensors on chain, reset search
        g_tempSensor.reset_search();
        return -1000;
    }
  
    if ( OneWire::crc8( addr, 7) != addr[7]) {
        return -1000;
    }
  
    if ( addr[0] != 0x10 && addr[0] != 0x28) {
        return -1000;
    }
  
    g_tempSensor.reset();
    g_tempSensor.select(addr);
    g_tempSensor.write(0x44,1); // start conversion, with parasite power on at the end
  
    byte present = g_tempSensor.reset();
    g_tempSensor.select(addr);
    g_tempSensor.write(0xBE); // Read Scratchpad
  
    for (int i = 0; i < 9; i++) { // we need 9 bytes
      data[i] = g_tempSensor.read();
    }
  
    g_tempSensor.reset_search();
  
    byte MSB = data[1];
    byte LSB = data[0];
  
    float tempRead = ((MSB << 8) | LSB); //using two's compliment
    float TemperatureSum = tempRead / 16;
  
    return TemperatureSum;
}
