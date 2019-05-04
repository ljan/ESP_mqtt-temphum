#include <ESP8266WiFi.h>
#include <PubSubClient.h>

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
    Adafruit_BME280 mySensor; // I2C
  #elif SENSOR_TYPE == 'BME280_SPI'
    Adafruit_BME280 mySensor(CS_PIN, MOSI_PIN, MISO_PIN, SCK_PIN); // software SPI;
  #endif
#else
  
#endif

#ifdef WIFI_IP && WIFI_GW && WIFI_SN
  IPAddress wifi_ip(WIFI_IP);
  IPAddress wifi_gw(WIFI_GW);
  IPAddress wifi_sn(WIFI_SN);
#endif

const int attemptDelay = 100;   // Delay in ms between measurement attempts
const int attemptMax = 5000;    // Max ms for attempts

WiFiClient espClient;
PubSubClient mqttClient(espClient);

ADC_MODE(ADC_VCC);  // Read internal vcc rather than voltage on ADC pin (A0 must be floating)


// *******SETUP*******
/* Bootup, Power and Initialize Sensor, Setup Wifi and MQTT */
void setup() {
  dbserialbegin(74880);
  dbprintln("");
  dbprintln("");
  dbprintln("BOOT");
  dbprint("VCC: ");
  dbprintln(ESP.getVcc()*VCC_ADJ/1024.00f);
  
  pinMode(SENSOR_PWR, OUTPUT);
  digitalWrite(SENSOR_PWR, HIGH);
  
  // Sensor Setup
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

// *******LOOP*******
/* Try to read Sensor for at leas 5 Seconds, send Results via MQTT, then go to Deep Sleep*/
void loop() {
  float humi=0.0f;
  float temp=0.0f;
  float pres=0.0f;
  bool  readok=false;
  int   startreading=millis();
  int   lastreading=millis()+100;
  
  dbprint("Reading Sensor: ");
  do { // read sensore while read is not ok or 5s have not elapsed
    temp=mySensor.readTemperature();
    humi=mySensor.readHumidity();
    #if SENSOR_TYPE == 'BME280_I2C' || SENSOR_TYPE == 'BME280_SPI'
      pres = mySensor.readPressure() / 100.0f;  // hPa
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
  #if SENSOR_TYPE == 'BME280_I2C' || SENSOR_TYPE == 'BME280_SPI'
    dbprint(pres); dbprint(" hPa ");
  #endif
  dbprintln();

  // publish
  if (readok) {
    mqttClient.publish(TEMP_TOPIC, String(temp).c_str(), false);
    mqttClient.publish(HUM_TOPIC,  String(humi).c_str(), false);
  #if SENSOR_TYPE == 'BME280_I2C' || SENSOR_TYPE == 'BME280_SPI'
    mqttClient.publish(PRES_TOPIC,  String(pres).c_str(), false);
  #endif
    mqttClient.publish(TEL_TOPIC, String(lastreading).c_str(), false);
  }
  else {
    mqttClient.publish(TEL_TOPIC, "Sensor Error", false);
  }
  mqttClient.publish(BAT_TOPIC,  String(ESP.getVcc()*VCC_ADJ/1024.00).c_str(), false);
  
  gotodeepsleep(SLEEP_TIME_S);
}
// ******* end main *******

void setup_wifi() {
  //WiFi.setAutoConnect(false); // not working by its own
  //WiFi.disconnect(); // prevent connecting to wifi based on previous configuration
  WiFi.mode(WIFI_STA); // explicitly set the ESP8266 to be a WiFi-client
  WiFi.persistent(false); // do not store settings in EEPROM
  WiFi.hostname(WIFI_HOSTNAME + String("-") + String(ESP.getChipId(), HEX));
  #ifdef WIFI_IP && WIFI_GW && WIFI_SN
    WiFi.config(wifi_ip, wifi_gw, wifi_sn);
  #endif
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  dbprint("Connecting to ");
  dbprint(WIFI_SSID);

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
    if (mqttClient.connect(WIFI_HOSTNAME, MQTT_USER, MQTT_PASSWORD)) {
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
  
  yield();
  delay(100);
}

// EOF
