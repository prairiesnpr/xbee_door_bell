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
  pinMode(DOOR_PIN, INPUT_PULLUP);

  Serial.begin(9600);
  Serial.println(F("Startup"));
  dht.begin();
  nss.begin(9600);
  zha.Start(nss, zdoReceive, NUM_ENDPOINTS, ENDPOINTS);

  // Set up callbacks
  zha.registerCallbacks(atCmdResp, zbTxStatusResp, otherResp);

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

void loop()
{
  zha.loop();

  if (zha.dev_status == READY)
  {
    uint8_t val = digitalRead(DOOR_PIN) ^ 1;
    Endpoint end_point = zha.GetEndpoint(DOOR_ENDPOINT);
    Cluster cluster = end_point.GetCluster(BINARY_INPUT_CLUSTER_ID);
    attribute *attr = cluster.GetAttr(BINARY_PV_ATTR);

    if (val != lastButtonState)
    {
      lastDebounceTime = millis();
    }

    if ((millis() - lastDebounceTime) > debounceDelay)
    {
      if (val != attr->GetIntValue())
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
  else if ((loop_time - last_msg_time) > 1000)
  {
    Serial.print(F("Not Started "));
    Serial.print(start_fails);
    Serial.print(F(" of "));
    Serial.println(START_LOOPS);

    last_msg_time = millis();
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

void zdoReceive(ZBExplicitRxResponse &erx, uintptr_t)
{
  // Create a reply packet containing the same data
  // This directly reuses the rx data array, which is ok since the tx
  // packet is sent before any new response is received

  if (erx.getRemoteAddress16() == 0)
  {
    zha.cmd_seq_id = erx.getFrameData()[erx.getDataOffset() + 1];
    Serial.print(F("Cmd Seq: "));
    Serial.println(zha.cmd_seq_id);

    uint8_t ep = erx.getDstEndpoint();
    uint16_t clId = erx.getClusterId();
    uint8_t cmd_id = erx.getFrameData()[erx.getDataOffset() + 2];
    uint8_t frame_type = erx.getFrameData()[erx.getDataOffset()] & 0x03;

    if (frame_type)
    {
      Serial.println(F("Clstr Cmd"));
      if (ep < TEMP_ENDPOINT)
      {
        Serial.println(F("Door Ep"));
      }
      else if (ep == TEMP_ENDPOINT)
      {
        Serial.println(F("Temp Ep"));
      }
      else
      {
        Serial.println(F("Inv Ep"));
      }
    }
    else
    {
      Serial.println(F("Glbl Cmd"));

      Endpoint end_point = zha.GetEndpoint(ep);
      Cluster cluster = end_point.GetCluster(clId);
      if (cmd_id == 0x00)
      {
        // Read attributes
        Serial.println(F("Read Attr"));
        uint8_t len_data = erx.getDataLength() - 3;
        uint16_t attr_rqst[len_data / 2];
        for (uint8_t i = erx.getDataOffset() + 3; i < (len_data + erx.getDataOffset() + 3); i += 2)
        {
          attr_rqst[i / 2] = (erx.getFrameData()[i + 1] << 8) |
                             (erx.getFrameData()[i] & 0xff);
          attribute *attr = end_point.GetCluster(erx.getClusterId()).GetAttr(attr_rqst[i / 2]);
          Serial.print(F("Clstr Rd Att: "));
          Serial.println(attr_rqst[i / 2]);
          zha.sendAttributeRsp(erx.getClusterId(), attr, ep, 0x01, 0x01, zha.cmd_seq_id);
          zha.cmd_seq_id++;
        }
      }
      else
      {
        Serial.println(F("Not Read Attr"));
      }
    }
    uint8_t frame_direction = (erx.getFrameData()[erx.getDataOffset()] >> 3) & 1;
    if (frame_direction)
    {
      Serial.println(F("Srv to Client"));
    }
    else
    {
      Serial.println(F("Client to Srv"));
    }
    Serial.print(F("ZDO: EP: "));
    Serial.print(ep);
    Serial.print(F(", Clstr: "));
    Serial.print(clId, HEX);
    Serial.print(F(" Cmd Id: "));
    Serial.print(cmd_id, HEX);
    Serial.print(F(" FrmCtl: "));
    Serial.println(erx.getFrameData()[erx.getDataOffset()], BIN);

    if (erx.getClusterId() == ACTIVE_EP_RQST)
    {
      // Have to match sequence number in response
      cmd_result = NULL;
      zha.last_seq_id = erx.getFrameData()[erx.getDataOffset()];
      zha.sendActiveEpResp(zha.last_seq_id);
    }
    if (erx.getClusterId() == SIMPLE_DESC_RQST)
    {
      Serial.print("Simple Desc Rqst, Ep: ");
      // Have to match sequence number in response
      // Payload is EndPoint
      // Can this just be regular ep?
      uint8_t ep_msg = erx.getFrameData()[erx.getDataOffset() + 3];
      Serial.println(ep_msg, HEX);
      zha.sendSimpleDescRpt(ep_msg, erx.getFrameData()[erx.getDataOffset()]);
    }
  }
}