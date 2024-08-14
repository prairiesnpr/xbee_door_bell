#include <SoftwareSerial.h>
#include <arduino-timer.h>
#include <DHT.h>
#include <DHT_U.h>
#include <avr/wdt.h>
#include <xbee_zha.h>
#include "zha/device_details.h"

#define TEMP_BUS 5

// Define SoftSerial TX/RX pins
#define ssRX 10
#define ssTX 11

#define DOOR_ENDPOINT 1
#define TEMP_ENDPOINT 2
#define DOOR_PIN 5
#define DHTPIN 6
#define DHTTYPE DHT22

#define START_LOOPS 100

uint8_t start_fails = 0;
uint8_t init_status_sent = 0;

DHT_Unified dht(DHTPIN, DHTTYPE);

void (*resetFunc)(void) = 0;

auto timer = timer_create_default(); // create a timer with default settings

unsigned long loop_time = millis();
unsigned long last_msg_time = loop_time - 1000;

bool lastButtonState = 0;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 6;

SoftwareSerial nss(ssRX, ssTX);

void setup()
{
  // Should reset xbee here, but didn't build that in

  pinMode(DOOR_PIN, INPUT_PULLUP);

  Serial.begin(9600);
  Serial.println(F("Startup"));
  dht.begin();
  nss.begin(9600);
  zha.Start(nss, zhaClstrCmd, zhaWriteAttr, NUM_ENDPOINTS, ENDPOINTS);

  // Set up callbacks
  zha.registerCallbacks(atCmdResp, zbTxStatusResp, otherResp, zdoReceive);

  Serial.println(F("CB Conf"));

  timer.every(30000, update_sensors);
  wdt_enable(WDTO_8S);
}

void update_temp()
{
  if (zha.dev_status == READY)
  {
    Endpoint end_point = zha.GetEndpoint(TEMP_ENDPOINT);
    sensors_event_t event;
    dht.temperature().getEvent(&event);
    if (isnan(event.temperature))
    {
      Serial.println(F("Error reading temperature!"));
    }
    else
    {
      Serial.print(F("Temperature: "));
      Serial.print(event.temperature);
      Serial.println(F("°C"));
      Cluster t_cluster = end_point.GetCluster(TEMP_CLUSTER_ID);
      attribute *t_attr = t_cluster.GetAttr(CURRENT_STATE);
      int16_t cor_t = (int16_t)(event.temperature * 100.0);
      t_attr->SetValue(cor_t);
      Serial.print((int16_t)t_attr->GetIntValue());
      Serial.println(F("°C"));
      zha.sendAttributeRpt(t_cluster.id, t_attr, end_point.id, 1);
    }

    dht.humidity().getEvent(&event);
    if (isnan(event.relative_humidity))
    {
      Serial.println(F("Error reading humidity!"));
    }
    else
    {
      Serial.print(F("Humidity: "));
      Serial.print(event.relative_humidity);
      Serial.println(F("%"));
      Cluster h_cluster = end_point.GetCluster(HUMIDITY_CLUSTER_ID);
      attribute *h_attr = h_cluster.GetAttr(CURRENT_STATE);
      uint16_t cor_h = (uint16_t)(event.relative_humidity * 100.0);
      h_attr->SetValue(cor_h);
      zha.sendAttributeRpt(h_cluster.id, h_attr, end_point.id, 1);
    }
  }
}

bool update_sensors(void *)
{
  update_temp();
  return true;
}

void update_bell_state(bool force = 0x00)
{
  uint8_t val = digitalRead(DOOR_PIN) ^ 1;
  Endpoint end_point = zha.GetEndpoint(DOOR_ENDPOINT);
  Cluster cluster = end_point.GetCluster(BINARY_INPUT_CLUSTER_ID);
  attribute *attr = cluster.GetAttr(BINARY_PV_ATTR);

  if (val != lastButtonState)
  {
    lastDebounceTime = millis();
  }

  if (((millis() - lastDebounceTime) > debounceDelay) || force)
  {
    if (val != attr->GetIntValue() || force)
    {
      Serial.print(F("EP"));
      Serial.print(end_point.id);
      Serial.print(F(": "));
      Serial.print(attr->GetIntValue());
      Serial.print(F(" Now "));
      attr->SetValue(val);
      Serial.println(attr->GetIntValue());
      zha.sendAttributeRpt(cluster.id, attr, end_point.id, 1);
    }
  }

  lastButtonState = val;
}

void loop()
{
  zha.loop();

  if (zha.dev_status == READY)
  {
    update_bell_state();
  }
  else if ((loop_time - last_msg_time) > 1000)
  {
    Serial.print(F("Not Started "));
    Serial.print(start_fails);
    Serial.print(F(" of "));
    Serial.println(START_LOOPS);

    last_msg_time = millis();
    if (start_fails > 15)
    {
      // Sometimes we don't get a response from dev ann, try a transmit and see if we are good
      update_bell_state(0x01);
    }
    if (start_fails > START_LOOPS)
    {
      resetFunc();
    }
    start_fails++;
  }

  timer.tick();
  wdt_reset();
  loop_time = millis();
}

void zhaClstrCmd(ZBExplicitRxResponse &erx)
{
  Serial.println(F("Clstr Cmd"));
  // No cluster commands supported
}

void zhaWriteAttr(ZBExplicitRxResponse &erx)
{

  Serial.println(F("Write Cmd"));
  // No write commands supported
}