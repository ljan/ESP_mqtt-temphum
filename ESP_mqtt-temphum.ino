#include <FS.h>                   //this needs to be first, or it all crashes and burns...

#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino

// needed for WifiManager
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager
#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson

// needed for Functionality
#include <PubSubClient.h>         // MQTT Client
#include "config.h"               // config
#include "debug.h"                // debugging

#if SENSOR_TYPE == DHT22
  #include <Adafruit_Sensor.h>
  #include <DHT.h>
  #include <DHT_U.h>
  DHT mySensor(DHT_PIN, SENSOR_TYPE);
#elif SENSOR_TYPE == HTU21
  #include <Wire.h>
  #include <SparkFunHTU21D.h>
  HTU21D mySensor;
#elif SENSOR_TYPE == BME280_I2C || SENSOR_TYPE == BME280_SPI
  #include <Wire.h>
  #include <SPI.h>
  #include <Adafruit_Sensor.h>
  #include <Adafruit_BME280.h>
  #if SENSOR_TYPE == BME280_I2C
    Adafruit_BME280 mySensor; // I2C
  #elif SENSOR_TYPE == BME280_SPI
    Adafruit_BME280 mySensor(CS_PIN, MOSI_PIN, MISO_PIN, SCK_PIN); // software SPI;
  #endif
#else
  
#endif


const int attemptDelay = 100;   // Delay in ms between measurement attempts
const int attemptMax = 5000;    // Max ms for attempts

// Data to go into FS
// define your default values here, if there are different values in config.json, they are overwritten.
// length should be max size + 1 
char mqtt_server[40] = MQTT_SERVER;
char mqtt_port[6] = MQTT_PORT;
char mqtt_user[40] = MQTT_USER;
char mqtt_password[40] = MQTT_PASSWORD;
char temp_topic[40] = TEMP_TOPIC;
char hum_topic[40] = HUM_TOPIC;
char pres_topic[40] = PRES_TOPIC;
char bat_topic[40] = BAT_TOPIC;
char tel_topic[40] = TEL_TOPIC;
// default custom static IP
char static_hostname[40] = DEFAULT_HOSTNAME;
char static_ip[16] = DEFAULT_IP;
char static_gw[16] = DEFAULT_GW;
char static_sn[16] = DEFAULT_SN;


bool shouldSaveConfig = false;    // flag for saving data

// callback notifying us of the need to save config
void saveConfigCallback () {
  dbprintln("Should save config");
  shouldSaveConfig = true;
}

WiFiClient espClient;
PubSubClient mqttClient(espClient);

ADC_MODE(ADC_VCC);  // Read internal VCC rather than voltage on ADC pin (A0 must be floating)

// *******SETUP*******
/* Bootup, Power and Initialize Sensor, Setup Wifi and MQTT */
void setup() {
  dbserialbegin(74880);
  dbprintln("");
  dbprintln("");
  dbprintln("BOOT");
  dbprint("VCC: ");
  dbprintln(ESP.getVcc()*VCC_ADJ/1024.00f);

  // Sensor Powerup
  pinMode(SENSOR_PWR, OUTPUT);
  digitalWrite(SENSOR_PWR, HIGH);
  
  // Sensor Setup
#if SENSOR_TYPE == HTU21 || SENSOR_TYPE == BME280_I2C
  Wire.begin(SDA_PIN, SCL_PIN); // custom i2c ports (SDA, SCL)
#endif
  mySensor.begin();
#if SENSOR_TYPE == DHT22
  mySensor.readTemperature();  // first reading to initialize DHT
  mySensor.readHumidity();
#endif
  
  //clean FS, for testing
  //SPIFFS.format();

  //read configuration from FS json
  dbprintln("mounting FS...");

  if (SPIFFS.begin()) {
    dbprintln("mounted file system");
    if (SPIFFS.exists("/config.json")) {
      //file exists, reading and loading
      dbprintln("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        dbprintln("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        json.printTo(Serial);
        if (json.success()) {
          dbprintln("\nparsed json");

          strcpy(mqtt_server, json["mqtt_server"]);
          strcpy(mqtt_port, json["mqtt_port"]);

          if(json["ip"]) {
            dbprintln("setting custom ip from config");
            //static_ip = json["ip"];
            strcpy(static_ip, json["ip"]);
            strcpy(static_gw, json["gateway"]);
            strcpy(static_sn, json["subnet"]);
            //strcat(static_ip, json["ip"]);
            //static_gw = json["gateway"];
            //static_sn = json["subnet"];
            dbprintln(static_ip);
/*            dbprintln("converting ip");
            IPAddress ip = ipFromCharArray(static_ip);
            dbprintln(ip);*/
          } else {
            dbprintln("no custom ip in config");
          }
        } else {
          dbprintln("failed to load json config");
        }
      }
    }
  } else {
    dbprintln("failed to mount FS");
  }
  //end read
  
  // The extra parameters to be configured (can be either global or just in the setup)
  // After connecting, parameter.getValue() will get you the configured value
  // id/name placeholder/prompt default length
  WiFiManagerParameter custom_mqtt_server("server", "mqtt server", mqtt_server, 40);
  WiFiManagerParameter custom_mqtt_port("port", "mqtt port", mqtt_port, 5);

  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;

  //set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  //set static ip
  IPAddress _ip,_gw,_sn;
  _ip.fromString(static_ip);
  _gw.fromString(static_gw);
  _sn.fromString(static_sn);

  wifiManager.setSTAStaticIPConfig(_ip, _gw, _sn);
  
  //add all your parameters here
  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_port);

  //reset settings - for testing
  //wifiManager.resetSettings();

  //set minimu quality of signal so it ignores AP's under that quality
  //defaults to 8%
  wifiManager.setMinimumSignalQuality();
  
  //sets timeout until configuration portal gets turned off
  //useful to make it all retry or go to sleep
  //in seconds
  wifiManager.setTimeout(180);

  //fetches ssid and pass and tries to connect
  //if it does not connect it starts an access point with the specified name
  //here  "AutoConnectAP"
  //and goes into a blocking loop awaiting configuration
  if (!wifiManager.autoConnect()) {
    dbprintln("failed to connect and hit timeout");
    delay(3000);
    //reset and try again, or maybe put it to deep sleep
    gotodeepsleep(SLEEP_TIME_S);
  }
  //if you get here you have connected to the WiFi
  dbprintln("connected...yeey :)");
  
  //read updated parameters
  strcpy(mqtt_server, custom_mqtt_server.getValue());
  strcpy(mqtt_port, custom_mqtt_port.getValue());

  //save the custom parameters to FS
  if (shouldSaveConfig) {
    dbprintln("saving config");
    DynamicJsonBuffer jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
    json["mqtt_server"] = mqtt_server;
    json["mqtt_port"] = mqtt_port;

    json["ip"] = WiFi.localIP().toString();
    json["gateway"] = WiFi.gatewayIP().toString();
    json["subnet"] = WiFi.subnetMask().toString();

    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {
      dbprintln("failed to open config file for writing");
    }

    json.prettyPrintTo(Serial);
    json.printTo(configFile);
    configFile.close();
    //end save
  }
  dbprintln();
  
  // setup mqtt
  mqttClient.setServer(mqtt_server, atoi(mqtt_port)); // parseInt to the port
  reconnect_mqtt();
  mqttClient.loop(); // This allows the client to maintain the connection and check for any incoming messages.
  yield();
}
// ******* end setup *******

// *******LOOP*******
/* Try to read Sensor for at leas 5 Seconds, send Results via MQTT, then go to Deep Sleep*/
void loop() {
  float humi=0.0;
  float temp=0.0;
  float pres=0.0;
  bool  readok=false;
  int   startreading=millis();
  int   lastreading=millis()+100;
  
  dbprint("Reading Sensor: ");
  do { // read sensore while read is not ok or 5s have not elapsed
    temp=mySensor.readTemperature();
    humi=mySensor.readHumidity();
    #if SENSOR_TYPE == BME280_I2C || SENSOR_TYPE == BME280_SPI
      pres = mySensor.readPressure() / 100.0F;  // hPa
    #endif
    if(isnan(humi) || isnan(temp)) {
      readok=false;
      if(lastreading <= millis()) {
        dbprint(". ");
        lastreading=millis()+100;
      }
      delay(attemptDelay);
    } else {
      readok=true;
    }
  } while(!readok && (millis()-startreading) <= attemptMax);
  lastreading=millis();

  //debugging output
  dbprintln("DONE");
  dbprint(temp); dbprint(" °C ");
  dbprint(humi); dbprint(" % ");
  #if SENSOR_TYPE == BME280_I2C || SENSOR_TYPE == BME280_SPI
    dbprint(pres); dbprint(" hPa ");
  #endif
  dbprintln();

  // publish
  if (readok) {
    mqttClient.publish(temp_topic, String(temp).c_str(), false);
    mqttClient.publish(hum_topic,  String(humi).c_str(), false);
  #if SENSOR_TYPE == BME280_I2C || SENSOR_TYPE == BME280_SPI
    mqttClient.publish(pres_topic,  String(pres).c_str(), false);
  #endif
    mqttClient.publish(tel_topic, String(lastreading).c_str(), false);
  }
  else {
    mqttClient.publish(tel_topic, "Sensor Error", false);
  }
  mqttClient.publish(bat_topic,  String(ESP.getVcc()*VCC_ADJ/1024.00).c_str(), false);
  
  gotodeepsleep(SLEEP_TIME_S);
}
// ******* end main *******

void setup_wifi() {
  /* Explicitly set the ESP8266 to be a WiFi-client, otherwise, it by default,
  would try to act as both a client and an access-point and could cause
  network-issues with your other WiFi-devices on your WiFi-network. */
  WiFi.mode(WIFI_STA);
  WiFi.persistent(false); // do not store settings in EEPROM
  WiFi.hostname(static_hostname + String("-") + String(ESP.getChipId(), HEX));  
  WiFi.begin(DEFAULT_SSID, DEFAULT_PASSWORD);
  
  dbprint("Connecting to ");
  dbprint(DEFAULT_SSID);

  /* This function returns following codes to describe what is going on with Wi-Fi connection: 
  0 : WL_IDLE_STATUS when Wi-Fi is in process of changing between statuses
  1 : WL_NO_SSID_AVAIL in case configured SSID cannot be reached
  3 : WL_CONNECTED after successful connection is established
  4 : WL_CONNECT_FAILED if password is incorrect
  6 : WL_DISCONNECTED if module is not configured in station mode
  Serial.printf( "Connection status: %d\n", WiFi.status() ); */
  while(WiFi.status() != WL_CONNECTED) {
    delay(200); // pauses the sketch for a given number of milliseconds and allows WiFi and TCP/IP tasks to run
    dbprint(".");
  }
  dbprint("\nWiFi connected, IP address: ");
  dbprintln(WiFi.localIP());
}

void reconnect_mqtt() {
  /* Reconnect to MQTT Server */
  int fails = 0;
  
  while (!mqttClient.connected()) {
    dbprint("Attempting MQTT connection...");
    // Attempt to connect
    // If you do not want to use a username and password, change next line to
    // if (mqttClient.connect("ESP8266Client"))
    if (mqttClient.connect(static_hostname, mqtt_user, mqtt_password)) {
      dbprintln("connected");
    }
    else {
      dbprint("failed, rc=");
      dbprint(mqttClient.state());
      dbprintln(" try again in 5 seconds");
      // Wait 1 second before retrying
      delay(1000);
      if(fails > 3) {
        gotodeepsleep(SLEEP_TIME_S*4);
      }
      fails++;
    }
  }
}

void gotodeepsleep(int sleeptime) {
/*ESP.deepSleep(microseconds, mode) will put the chip into deep sleep.
  mode is one of WAKE_RF_DEFAULT, WAKE_RFCAL, WAKE_NO_RFCAL, WAKE_RF_DISABLED.
  (GPIO16 needs to be tied to RST to wake from deepSleep.)*/
  dbprintln("Power off Sensor -  going to deep sleep");
  digitalWrite(SENSOR_PWR, LOW);
  
  mqttClient.loop();
  yield();
  delay(100);
  #ifdef DEBUG
    ESP.deepSleep(sleeptime*1e6, WAKE_RF_DEFAULT);
  #else
    ESP.deepSleep(sleeptime*1e6, WAKE_RF_DEFAULT);
  #endif
}

// EOF
