#include <Arduino.h>
#include <MAVLink.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "handlers.h" // szuka w include/

const char *ssid = "SLUMSY";
const char *password = "ujnakolana";

WiFiUDP udp;

MissionPoint mission_points[MISSION_COUNT];

const unsigned int localPort = 12346; // port nasłuchu
const char *pcIp = "192.168.35.138";  // IP komputera z aplikacją Qt
const unsigned int pcPort = 12345;    // port PC do wysyłki danych

// const IPAddress localIp(192, 168, 35, 100); // k1
// const IPAddress localIp(192, 168, 35, 101); // k2
// const IPAddress localIp(192, 168, 35, 102); // k3
const IPAddress localIp(192, 168, 35, 103); // k4

const IPAddress gateway(192, 168, 35, 1); // << brama
const IPAddress subnet(255, 255, 255, 0); // << maska
// const IPAddress dns(8, 8, 8, 8);            // << opcjonalnie

const int MAVLINK_RX_PIN = D0;
const int MAVLINK_TX_PIN = D1;
const int SW_PIN = D7;
const uint8_t ESP_SYSTEM_ID = 100;
const uint8_t ESP_COMPONENT_ID = 1;
const unsigned long HEARTBEAT_INTERVAL_MS = 1000;
// const unsigned long GLOBAL_POSITION_DELAY = 10000;
const unsigned long GPS_INTERVAL_US = 1000000;
const unsigned long PARAM_INTERVAL_US = 1000000;
const unsigned long telemetryInterval = 1000;
const unsigned long missionOkInterval = 5000;

HardwareSerial mavlink_serial(1);
uint8_t fc_system_id = 0;
uint8_t fc_component_id = 0;
unsigned long last_heartbeat = 0;
uint16_t mission_count = 0;
uint32_t mission_opaque_id = 0;
unsigned long lastSendTime = 0;
unsigned long lastmissionOkInterval = 0;
String mission_ok = "Nie wgrywam";

int32_t wp_x;
int32_t wp_y;
float wp_z;
float wp_delay;

bool is_mavlink_connected = false;
bool is_new_mission = false;
bool is_mission_cleared = false;
bool is_mission_uploaded = false;
bool is_mounted = false;

const int NOT_INSERTED = HIGH;
const int INSERTED = LOW;

// ----------

// jakby obcinało msg, odkomentuj w setupie 'mavlinkSerial.setRxBufferSize' (i zwiększ jeśli trzeba)

// to se możesz wysyłać jak chcesz (aktualizowane co 0.5s)
float curr_x, curr_y, curr_z_m; // [mm]
uint8_t fix_type;               // https://mavlink.io/en/messages/common.html#GPS_FIX_TYPE
uint8_t satellites;
String can_arm;
String arm_errors;

float MISS_ALT = 55;
int POINT_DELAY = 5;
// to proszę wywołać jak odbierzesz punkty z serwera
void set_wp(int32_t x, int32_t y, float z, float delay)
{
  wp_x = x;
  wp_y = y;
  wp_z = z;
  wp_delay = POINT_DELAY; // czas wiszenia
  is_new_mission = true;
}

void setup()
{
  pinMode(SW_PIN, INPUT_PULLUP);
  Serial.begin(9600);
  WiFi.setHostname("Kurwik1");
  WiFi.config(localIp, gateway, subnet); // << Ustaw statyczne IP
  WiFi.begin(ssid, password);

  Serial.print("Laczenie z WiFi");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\
    ! IP: " + WiFi.localIP().toString());

  udp.begin(localPort);
  // mavlinkSerial.setRxBufferSize(2048);
  mavlink_serial.begin(115200, SERIAL_8N1, MAVLINK_RX_PIN, MAVLINK_TX_PIN);

  while (!is_mavlink_connected)
  {
    Serial.print(".");

    mavlink_message_t msg;
    mavlink_status_t status;

    while (mavlink_serial.available())
    {
      uint8_t c = mavlink_serial.read();
      if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
      {
        if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT)
        {
          fc_system_id = msg.sysid;
          fc_component_id = msg.compid;
          is_mavlink_connected = true;
          break;
        }
      }
    }

    if (millis() - last_heartbeat >= HEARTBEAT_INTERVAL_MS)
    {
      send_heartbeat();
      last_heartbeat = millis();
    }
    delay(10);
  }
  send_command_long(MAV_CMD_SET_MESSAGE_INTERVAL, MAVLINK_MSG_ID_SYS_STATUS, 1000000); // co 1s
  send_request_position_with_interval(GPS_INTERVAL_US);                                // ustawia co ile przesyłać dane z gpsa
}

void loop()
{
  mavlink_message_t msg;
  mavlink_status_t status;

  int packetSize = udp.parsePacket();
  if (packetSize)
  {
    char buffer[255];
    int len = udp.read(buffer, 255);
    if (len > 0)
    {
      buffer[len] = '\0';
      Serial.println("Odebrano wspolrzedne: " + String(buffer));

      // Parsowanie współrzędnych

      if (buffer[0] == 'Z')
      {
        if (sscanf(buffer, "Z%d,%f", &POINT_DELAY, &MISS_ALT) == 2)
        {

          Serial.print("POINT_DELAY = ");
          Serial.println(POINT_DELAY);
          Serial.print("MISS_ALT = ");
          Serial.println(MISS_ALT);
        }
        else
        {
          Serial.println("Błąd parsowania współrzędnych!");
        }
      }
      else
      {
        float x = 0.0, y = 0.0;
        if (sscanf(buffer, "%f,%f", &x, &y) == 2)
        {
          x *= 1e7;
          y *= 1e7;
          Serial.print("X = ");
          Serial.println(x);
          Serial.print("Y = ");
          Serial.println(y);

          Serial.println(curr_z_m);

          Serial.print("curr_z_m + MISS_ALT = ");
          Serial.println(curr_z_m + MISS_ALT);
          set_wp(x, y, curr_z_m + MISS_ALT, POINT_DELAY); // lub dowolne inne parametry misji
        }
        else
        {
          Serial.println("Błąd parsowania współrzędnych!");
        }
      }
    }
  }
  if (millis() - lastmissionOkInterval >= missionOkInterval)
  {

    lastmissionOkInterval = millis();
    mission_ok = "Nie wgrywam";
    arm_errors = "0";
  }

  // Wysyłanie telemetrii co X sekund
  if (millis() - lastSendTime >= telemetryInterval)
  {

    lastSendTime = millis();

    String msg = mission_ok + "\n" + can_arm + "\nARMING CHECKS: \n" + arm_errors + "\n" +
                 "GPS: lat: " + String(curr_x / 1e7, 4) + " lon: " + String(curr_y / 1e7, 4) +
                 " alt: " + String(curr_z_m, 2) + " sats: " + String(satellites, DEC) + "\n";
    udp.beginPacket(pcIp, pcPort);
    udp.print(msg);
    udp.endPacket();
  }

  if (millis() - last_heartbeat >= HEARTBEAT_INTERVAL_MS) // esp mówi fc że utrzmuje połącznie wysyłając heartbeat
  {
    send_heartbeat();
    last_heartbeat = millis();
  }

  if (!is_mounted && digitalRead(SW_PIN) == INSERTED) // montujemy kurwika na drona matkę
  {
    is_mounted = true;
  }

  if (is_mounted && digitalRead(SW_PIN) == NOT_INSERTED) // spuszczamy kurwika z drona matki
  {
    is_mounted = false;
    send_set_mode_auto();
    send_arm();
  }

  if (is_new_mission)
  {
    is_new_mission = false;
    send_mission_clear_all();
    is_mission_cleared = false;
  }

  while (mavlink_serial.available())
  {
    uint8_t c = mavlink_serial.read();
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
    {
      //  Serial.print("Odebrano MAVLink ID: ");
      //  Serial.println(msg.msgid);

      switch (msg.msgid)
      {
      case MAVLINK_MSG_ID_PARAM_VALUE:
        handle_param_value(&msg);
        break;
      case MAVLINK_MSG_ID_SYS_STATUS:
        handle_sys_status(&msg);
        break;
      case MAVLINK_MSG_ID_STATUSTEXT: // wszystkie nfo errory statusy itp
        handle_statustext(&msg);
        break;
      case MAVLINK_MSG_ID_MISSION_ACK: // potiwersdza że fc przyjęło misję
        handle_mission_ack(&msg);
        break;
      case MAVLINK_MSG_ID_MISSION_REQUEST:
        handle_mission_request(&msg);
        break;
      case MAVLINK_MSG_ID_MISSION_REQUEST_INT:
        handle_mission_request_int(&msg);
        break;
      case MAVLINK_MSG_ID_GPS_RAW_INT:
        handle_gps_raw_int(&msg);
        break;
      }
    }
  }
}