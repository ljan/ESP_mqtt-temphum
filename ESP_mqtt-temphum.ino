#include <FS.h>                   //this needs to be first, or it all crashes and burns...

#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino

// needed for WifiManager
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager
#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson

// needed for Functionality
#include <PubSubClient.h>         // MQTT Client
#include "DHT.h"                  // Temperature Sensor
#include "config.h"               // Config
#include "debug.h"                // debugging

// Config
const int AttemptDelay = 10;      // Delay in ms between measurement attempts

// Data to go into FS
// define your default values here, if there are different values in config.json, they are overwritten.
// length should be max size + 1 
char mqtt_server[40] = MQTT_SERVER;
char mqtt_port[6] = MQTT_PORT;
// default custom static IP
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

DHT dht(DHT_PIN, DHT_TYPE);

ADC_MODE(ADC_VCC);  // Read internal VCC rather than voltage on ADC pin (A0 must be floating)

// *******SETUP*******
/* Bootup, Power and Initialize DHT, Setup Wifi and MQTT */
void setup() {
  dbserialbegin(74880);
  dbprintln("");
  dbprintln("");
  dbprintln("BOOT");
  dbprint("VCC: ");
  dbprintln(ESP.getVcc()*VCC_ADJ/1024.00f);

  pinMode(DHT_PWR, OUTPUT);
  digitalWrite(DHT_PWR, HIGH);
  
  // DHT Setup
  dht.begin();
  dht.readTemperature();  // first reading to initialize DHT
  dht.readHumidity();
  
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
  bool  sensor=true;
  int   startreading=millis();
  int   lastreading=millis()+100;
  
  dbprint("Reading Sensor");
  do {
    sensor=true;
    // Reading temperature or humidity takes about 250 milliseconds!
    // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
    temp=dht.readTemperature();
    humi=dht.readHumidity();
    if(isnan(humi) || isnan(temp)) {
      sensor=false;
      if(lastreading <= millis()) {
        dbprint(".");
        lastreading=millis()+100;
      }
      delay(AttemptDelay);
    }
  } while(!sensor && (millis()-startreading)<=5000);
  lastreading=millis();
  dbprintln("DONE");
  
  mqttClient.publish(TEMP_TOPIC, String(temp).c_str(), false);
  mqttClient.publish(HUM_TOPIC,  String(humi).c_str(), false);
  mqttClient.publish(BAT_TOPIC,  String(ESP.getVcc()*VCC_ADJ/1024.00).c_str(), false);
  if (sensor) {
    mqttClient.publish(TEL_TOPIC, String(lastreading).c_str(), false);
  }
  else {
    mqttClient.publish(TEL_TOPIC, "Sensor Error", false);
  }

  mqttClient.loop();
  yield();
  delay(100);
  
  dbprintln("Power off Sensor -  going to deep sleep");
  digitalWrite(DHT_PWR, LOW);
  
  gotodeepsleep(SLEEP_TIME_S);
}
// ******* end main *******

void setup_wifi() {
  /* Explicitly set the ESP8266 to be a WiFi-client, otherwise, it by default,
  would try to act as both a client and an access-point and could cause
  network-issues with your other WiFi-devices on your WiFi-network. */
  WiFi.mode(WIFI_STA);
  WiFi.persistent(false); // do not store settings in EEPROM
  WiFi.hostname(DEFAULT_HOSTNAME + String("-") + String(ESP.getChipId(), HEX));  
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
  dbprint("WiFi connected, IP address: ");
  dbprintln(WiFi.localIP());
}

void reconnect_mqtt() {
  int fails = 0;
  
  while (!mqttClient.connected()) {
    dbprint("Attempting MQTT connection...");
    // Attempt to connect
    // If you do not want to use a username and password, change next line to
    // if (mqttClient.connect("ESP8266Client"))
    if (mqttClient.connect(DEFAULT_HOSTNAME, MQTT_USER, MQTT_PASSWORD)) {
      dbprintln("connected");
    }
    else {
      dbprint("failed, rc=");
      dbprint(mqttClient.state());
      dbprintln(" try again in 5 seconds");
      // Wait 1 second before retrying
      delay(1000);
      if(fails > 3) {
        gotodeepsleep(SLEEP_TIME_S);
      }
      fails++;
    }
  }
}

void gotodeepsleep(int sleeptime) {
/*ESP.deepSleep(microseconds, mode) will put the chip into deep sleep.
  mode is one of WAKE_RF_DEFAULT, WAKE_RFCAL, WAKE_NO_RFCAL, WAKE_RF_DISABLED.
  (GPIO16 needs to be tied to RST to wake from deepSleep.)*/
  #ifdef DEBUG
    ESP.deepSleep(sleeptime*1e6, WAKE_RF_DEFAULT);
    yield();
    delay(100);
//    dbprintln("Debugging - only doing delay and reset");
//    delay(sleeptime*1e3);
//    ESP.reset();
  #else
    ESP.deepSleep(sleeptime*1e6, WAKE_RF_DEFAULT);
    yield();
    delay(100);
  #endif
}

// EOF
