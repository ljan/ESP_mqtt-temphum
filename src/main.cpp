#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#include "config.h"
#include "debug.h"

#if SENSOR_TYPE == 'DHT22'
  #include <Adafruit_Sensor.h>
  #include <DHT.h>
  #include <DHT_U.h>
  DHT mySensor(DHT_PIN, SENSOR_TYPE);
#elif SENSOR_TYPE == 'HTU21'
  #include <Wire.h>
  #include <SparkFunHTU21D.h>
  HTU21D mySensor;
#elif SENSOR_TYPE == 'BME280_I2C' || SENSOR_TYPE == 'BME280_SPI'
  #include <Wire.h>
  #include <SPI.h>
  #include <Adafruit_Sensor.h>
  #include <Adafruit_BME280.h>
  #if SENSOR_TYPE == 'BME280_I2C'
    Adafruit_BME280 mySensor;
  #elif SENSOR_TYPE == 'BME280_SPI'
    Adafruit_BME280 mySensor(CS_PIN, MOSI_PIN, MISO_PIN, SCK_PIN);
  #endif
#else
  
#endif

#ifdef WIFI_IP && WIFI_GW && WIFI_SN && WIFI_DNS
  IPAddress wifi_ip(WIFI_IP);
  IPAddress wifi_gw(WIFI_GW);
  IPAddress wifi_sn(WIFI_SN);
  IPAddress wifi_dns(WIFI_DNS);
#endif

const uint attemptDelay = 100;   // Delay in ms between measurement attempts
const uint attemptMax = 5000;    // Max ms for attempts (Sensor, WiFi, MQTT)

WiFiClient espClient;
PubSubClient mqttClient(espClient);

ADC_MODE(ADC_VCC);  // Read internal vcc rather than voltage on ADC pin (A0 must be floating)


void gotodeepsleep(int sleeptime) {
/* turn off WiFi, send MQTT and go to deep sleep */
  
  mqttClient.loop();
  yield();
  delay(100);

  WiFi.mode( WIFI_OFF );
  WiFi.forceSleepBegin();
  
  #ifdef DEBUG
    dbprint("Total Uptime was: "); dbprint(millis()); dbprintln(" ms");
    ESP.deepSleep(sleeptime*1e6, WAKE_RF_DEFAULT);
  #else
    ESP.deepSleep(sleeptime*1e6, WAKE_RF_DEFAULT);
    // GPIO16 needs to be tied to RST to wake from deep sleep
  #endif
  
  yield(); // this will not be reached
  delay(100);
}

void setup_wifi() {
/* Connect to WiFi */
  int startwifi = millis();

  WiFi.forceSleepWake(); // wake up to use WiFi again
  yield();
  //wifi_fpm_do_wakeup();
  //wifi_fpm_close();
  
  WiFi.mode(WIFI_STA); // explicitly set the ESP8266 to be a WiFi-client
  WiFi.persistent(false); // do not store settings in EEPROM
  WiFi.hostname(WIFI_HOSTNAME + String("-") + String(ESP.getChipId(), HEX));
  #ifdef WIFI_IP && WIFI_GW && WIFI_SN && WIFI_DNS
    WiFi.config(wifi_ip, wifi_gw, wifi_sn, wifi_dns);
  #endif

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  dbprint("Connecting to "); dbprint(WIFI_SSID);
  while(WiFi.status() != WL_CONNECTED) {
    delay(100); // pauses the sketch and allows WiFi and TCP/IP tasks to run
    dbprint(".");
    if(millis() >= startwifi + attemptMax && WiFi.status() != WL_CONNECTED) {
      dbprintln("\nNo WiFi connection after 5 s, goint to deep sleep");
      gotodeepsleep(SLEEP_TIME_S*4);
    }
  }
  dbprint("\nWiFi connected, IP address: "); dbprintln(WiFi.localIP());
}

void reconnect_mqtt() {
/* Connect to MQTT Server */
  uint startmqtt = millis();

  dbprint("Attempting MQTT connection: ");
  while (!mqttClient.connected()) {
    if (mqttClient.connect(WIFI_HOSTNAME, MQTT_USER, MQTT_PASSWORD)) {
      dbprintln("connected");
    } else {
      dbprint("failed, rc="); dbprint(mqttClient.state());
      if(millis() >= startmqtt + attemptMax) {
        dbprintln("\nNo MQTT connection after 5 s, going to deep sleep");
        gotodeepsleep(SLEEP_TIME_S*4);
      }
      delay(100);
      dbprintln("try again");
    }
  }
}

// ******* SETUP *******
void setup() {
/* Boot-up, Power and Initialize Sensor, Setup Wifi and MQTT */
  dbserialbegin(74880);
  dbprintln(""); dbprintln("");
  dbprintln("BOOT");
  dbprint("VCC: "); dbprintln(ESP.getVcc()*VCC_ADJ/1024.0f);

  WiFi.mode(WIFI_OFF); // Start with WiFi turned off
  WiFi.forceSleepBegin();

  // Sensor Setup
  pinMode(SENSOR_PWR, OUTPUT);
  digitalWrite(SENSOR_PWR, HIGH);
  
  #if SENSOR_TYPE == 'HTU21' || SENSOR_TYPE == 'BME280_I2C'
    Wire.begin(SDA_PIN, SCL_PIN); // custom i2c ports (SDA, SCL)
  #endif
  mySensor.begin();
  #if SENSOR_TYPE == 'DHT22'
    mySensor.readTemperature();  // first reading to initialize DHT
    mySensor.readHumidity();
  #endif
  
  // start wifi
  setup_wifi();
  
  // setup mqtt
  mqttClient.setServer(MQTT_SERVER, atoi(MQTT_PORT));
  reconnect_mqtt();
  mqttClient.loop(); // This allows the client to maintain the connection and check for any incoming messages.
  yield();
}
// ******* end setup *******

// ******* LOOP *******
void loop() {
/* Read Sensor, publish Results via MQTT, then go to Deep Sleep*/
  float temp=0.0f;
  float humi=0.0f;
  float pres=0.0f;
  bool  readok=false;
  int   sensortime=millis();
  char  message[256];
  StaticJsonDocument<200> json;
  
  // read sensor
  dbprint("Reading Sensor: ");
  do { // read sensore while read is not ok or 5s have not elapsed
    temp=mySensor.readTemperature();
    humi=mySensor.readHumidity();
    #if SENSOR_TYPE == 'BME280_I2C' || SENSOR_TYPE == 'BME280_SPI'
      pres = mySensor.readPressure() / 100.0f;  // hPa
    #endif
    if( //read error
      (isnan(temp) || isnan(humi)) || // DHT21
      ((temp==998 || humi==998) || (temp==-46.85 && humi==-6)) || // HTU21
      (temp==0 && humi==0 && pres==0)// BME280
      ) {
      readok=false;
      dbprint(".");
      delay(attemptDelay);
    } else {
      readok=true;
      digitalWrite(SENSOR_PWR, LOW);
      dbprintln("DONE - Power off Sensor"); // debugging output
      dbprint(temp); dbprint(" °C ");
      dbprint(humi); dbprint(" % ");
      #if SENSOR_TYPE == 'BME280_I2C' || SENSOR_TYPE == 'BME280_SPI'
        dbprint(pres); dbprint(" hPa ");
      #endif
      dbprintln();
    }
  } while(!readok && (millis()) <= sensortime + attemptMax);
  sensortime=millis();

  // build mqtt json message
  json["device"] = WIFI_HOSTNAME + String("-") + String(ESP.getChipId(), HEX);
  json["sensor"] = SENSOR_TYPE;
  if (readok) {
    json["temperature"] = temp;
    json["humidity"] = humi;
    #if SENSOR_TYPE == 'BME280_I2C' || SENSOR_TYPE == 'BME280_SPI'
    json["pressure"] = pres;
    #endif
    json["status"] = sensortime;
  } else {
    dbprintln("Sensor Error");
    json["status"] = "Sensor Error";
  }
  json["battery"] = ESP.getVcc()*VCC_ADJ/1024.0f;

  // publish via MQTT
  size_t n = serializeJson(json, message); // serialize json into message
  dbprintln(message);
  mqttClient.publish(TOPIC, message, n); // publish json message
  
  // go to deep sleep
  gotodeepsleep(SLEEP_TIME_S);
}
// ******* end loop *******

// EOF
