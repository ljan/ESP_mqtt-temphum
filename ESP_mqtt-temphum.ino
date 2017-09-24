#include <ESP8266WiFi.h>

#include <PubSubClient.h>

#include "DHT.h"

#include "config.h"
#include "debug.h"

const int AttemptDelay = 10;        // Delay in ms between measurement attempts

WiFiClient espClient;
PubSubClient mqttClient(espClient);

DHT dht(DHT_PIN, DHT_TYPE);

ADC_MODE(ADC_VCC);  // Read internal vcc rather than voltage on ADC pin (A0 must be floating)

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
  
  // start wifi
  setup_wifi();
  
  // setup mqtt
  mqttClient.setServer(MQTT_SERVER, 1883);
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
        gotodeepsleep(sleepTimeS);
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
