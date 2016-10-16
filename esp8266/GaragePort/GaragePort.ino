/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2016 Pierre Ståhl
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <DHT.h>
#include <WiFiClient.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>

/******************************************************************************/

#define PROGMEM_STR(name, value) const char name[] PROGMEM = value

/******************************************************************************/

// Config: change values below

static const char* WIFI_SSID = "ssid";
static const char* WIFI_PASSWORD = "password";

static const int GARAGE_PORT_PIN = D6;
static const int GARAGE_PORT_RELAY_PIN = D8;

#define TEMPERATURE_REPORT_INTERVAL 60 // Seconds

// During the grace period (x seconds after connection established to MQTT
// broker), all open/close requests are ignored as a security measure.
#define MQTT_STARTUP_GRACE_PERIOD 10 // Seconds

#define MQTT_SERVERPORT 1883
PROGMEM_STR(MQTT_SERVER, "ip");
PROGMEM_STR(MQTT_USERNAME, "user");
PROGMEM_STR(MQTT_PASSWORD, "pass");

/******************************************************************************/

static WiFiClient client;
static Adafruit_MQTT_Client mqtt(&client,
  MQTT_SERVER, MQTT_SERVERPORT, MQTT_USERNAME, MQTT_PASSWORD);

PROGMEM_STR(GARAGE_FEED, "home-assistant/garage/port/state");
Adafruit_MQTT_Publish garage = Adafruit_MQTT_Publish(&mqtt, GARAGE_FEED);

PROGMEM_STR(TEMPERATURE_FEED, "home-assistant/garage/temperature");
Adafruit_MQTT_Publish temperature = Adafruit_MQTT_Publish(&mqtt, TEMPERATURE_FEED);

PROGMEM_STR(HUMIDITY_FEED, "home-assistant/garage/humidity");
Adafruit_MQTT_Publish humidity = Adafruit_MQTT_Publish(&mqtt, HUMIDITY_FEED);

// Subscribes to changes to trigger opening of garage port
PROGMEM_STR(GARAGE_OPEN, "home-assistant/garage/port/set");
Adafruit_MQTT_Subscribe garage_open = Adafruit_MQTT_Subscribe(&mqtt, GARAGE_OPEN);

/******************************************************************************/

// You can change these if you want, but they should be fine as-is
#define NUM_LOG_ENTRIES 35
#define LOG_STR_LEN 50

struct _log_entry {
  unsigned long time;
  char str[LOG_STR_LEN];
};

static struct _log_entry log_entries[NUM_LOG_ENTRIES] = {{0}};
static int log_latest_index = 0;

static void _do_log(const String& str) {
  _log_entry* entry = &log_entries[log_latest_index];
  entry->time = millis();
  strncpy(entry->str, str.c_str(), LOG_STR_LEN - 1);
  entry->str[LOG_STR_LEN-1] = '\0';

  Serial.println(str);
  log_latest_index = (log_latest_index + 1) % NUM_LOG_ENTRIES;
}

#define LOG1(str) do { _do_log(str); } while(0)
#define LOG2(a, b) do { String tmp = a; tmp += b; _do_log(tmp); } while(0)

/******************************************************************************/

static ESP8266WebServer server(80);
static DHT dht(D0, DHT11);

static volatile unsigned long last_button_change = 0;
static volatile bool last_button_value = false;
static bool garage_port_state = false;

// Hack to trigger sending of temperature and humidity at start up
static long last_publish_dht11 = (TEMPERATURE_REPORT_INTERVAL * 1000) + 1;

static long start_of_grace_period = 0;

/******************************************************************************/

void setup(void){
  Serial.begin(115200);

  LOG1("Booting up...");

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  connect_to_wifi();

  server.on("/", handle_root);
  server.on("/open", handle_open);
  server.on("/log", handle_log);
  server.onNotFound(handle_not_found);
  server.begin();

  dht.begin();
  pinMode(GARAGE_PORT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(GARAGE_PORT_PIN), interrupt_handler, CHANGE);

  read_garage_port_state();
  garage_port_state = !last_button_value;

  // Default state is LOW to ensure no button press during boot/restart
  pinMode(GARAGE_PORT_RELAY_PIN, OUTPUT);
  digitalWrite(GARAGE_PORT_RELAY_PIN, LOW);

  mqtt.subscribe(&garage_open);

  LOG1("Up and running!");
}

// Called when the garage port button is pressed/released
static void interrupt_handler() {
  read_garage_port_state();
}

/******************************************************************************/

static bool is_connected_to_wifi() {
  return (WiFi.status() == WL_CONNECTED);
}

static void connect_to_wifi() {
  LOG1("Connecting to wifi...");

  while (!is_connected_to_wifi()) {
    delay(500);
    Serial.print(".");
  }

  LOG2("Connected to ", WIFI_SSID);
  LOG2("IP address: ", WiFi.localIP().toString());
}

static void read_garage_port_state() {
  last_button_change = millis();
  last_button_value = digitalRead(GARAGE_PORT_PIN);
}

void loop(void){
  if (!is_connected_to_wifi()) {
    connect_to_wifi();
  }

  if (mqtt_connect()) {
    process_loop();
  } else {
    LOG1("Connect to MQTT broker failed");
  }
}

static void process_loop() {
  // De-bounce garage port button
  if ((millis() - last_button_change) > 100 && garage_port_state == last_button_value) {
    garage_port_state = is_garage_port_open();

    LOG2("Garage port state: ", garage_port_state ? "OPEN" : "CLOSED");

    publish_garage(garage_port_state);
  }

  // If specified time has elapsed, report new temperature and humidity
  if ((millis() - last_publish_dht11) > (TEMPERATURE_REPORT_INTERVAL * 1000)) {
    // @todo: handle invalid temperature readings
    publish_dht11(dht.readTemperature(), dht.readHumidity());
    last_publish_dht11 = millis();
  }

  // Receive incoming messages over MQTT
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(200))) {
    if (subscription == &garage_open) {
      handle_garage_port_open_mqtt();
    }
  }

  server.handleClient();
}

static void handle_garage_port_open_mqtt() {
  const char* value = (const char *)garage_open.lastread;

  if ((millis() - start_of_grace_period) <= MQTT_STARTUP_GRACE_PERIOD * 1000) {
    LOG2("MQTT: Grace period, ignoring: ", (char *)garage_open.lastread);
    return;
  }

  if (strcmp(value, "OPEN") == 0) {
    if (!is_garage_port_open()) {
      LOG1("MQTT: Opening garage port");
      open_garage_door();
    } else {
      LOG1("MQTT: Ignoring request to open garage port since it is already open");
    }
  } else if (strcmp(value, "CLOSE") == 0) {
    if (is_garage_port_open()) {
      LOG1("MQTT: Closing garage port");
      open_garage_door();
    } else {
      LOG1("MQTT: Ignoring request to close garage port since it is already closed");
    }
  } else {
    LOG2("MQTT: Unhandled request: ", (char *)garage_open.lastread);
  }
}

static void publish_garage(const bool is_open) {
  LOG2("New garage state: ", is_open ? "OPEN" : "CLOSED");
  garage.publish(is_open ? "OPEN" : "CLOSED");
}

static bool is_garage_port_open() {
  return (digitalRead(GARAGE_PORT_PIN) == LOW);
}

static bool open_garage_door() {
  LOG1("Pressed button and toggled state");
  digitalWrite(GARAGE_PORT_RELAY_PIN, HIGH);
  delay(300);
  digitalWrite(GARAGE_PORT_RELAY_PIN, LOW);
  return true;
}

static void publish_dht11(const float _temperature, const float _humidity) {
  String output = "New temperature: ";
  output += _temperature;
  output += ", humidity: ";
  output += _humidity;
  Serial.println(output); // Do not log, will just flood the log

  temperature.publish(_temperature);
  humidity.publish(_humidity);
}

static bool mqtt_connect() {
  if (mqtt.connected()) {
    return true;
  }

  LOG1("Connecting to MQTT... ");

  int ret = 0;
  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected

    LOG1(mqtt.connectErrorString(ret));
    LOG1("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    delay(5000);  // wait 5 seconds

    if (--retries == 0) {
      return false;
    }
  }

  LOG1("MQTT Connected!");
  on_mqtt_connect(); // Ugly to have a callback here, but what the heck

  return true;
}

void on_mqtt_connect() {
  start_of_grace_period = millis();
  publish_garage(is_garage_port_open());
}

/******************************************************************************/

static void handle_root() {
  String output = "";
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();

  // Temperature
  output += "Temperature: ";
  if (!isnan(temperature)) {
    output += temperature;
  } else {
    output += "err";
  }

  // Humidity
  output += ", humidity: ";
  if (!isnan(humidity)) {
    output += humidity;
  } else {
    output += "err";
  }

  // Garage port state
  output += ", garage port: ";
  if (is_garage_port_open()) {
    output += "open";
  } else {
    output += "closed";
  }

  // Send response to client
  server.send(200, "text/plain", output);
}

/******************************************************************************/

static void handle_open() {
  LOG2("Request to open garage door from ", server.client().remoteIP().toString());

  if (open_garage_door()) {
    server.send(200, "text/plain", "Garage door is open");
  } else {
    server.send(301, "text/plain", "Failed to open garage door");
  }
}

/******************************************************************************/

static void handle_log() {
  String output = "Log entry list (now: ";
  output += millis();
  output += "):\n";

  for (int i = 0; i < NUM_LOG_ENTRIES; ++i) {
    const int index = (log_latest_index + i) % NUM_LOG_ENTRIES;
    if (log_entries[index].time == 0) {
      continue;
    }
    output += "[";
    output += log_entries[index].time;
    output += "] ";
    output += log_entries[index].str;
    output += "\n";
  }

  server.send(200, "text/plain", output);
}

/******************************************************************************/

static void handle_not_found(){
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET)?"GET":"POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  for (uint8_t i=0; i<server.args(); i++){
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }
  server.send(404, "text/plain", message);
}

/******************************************************************************/

