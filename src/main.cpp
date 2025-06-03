#include <Arduino.h>
#include <MAVLink.h>

struct MissionPoint {
  uint16_t seq;
  MAV_FRAME frame;
  MAV_CMD command;
  float param1;
  float param2;
  float param3;
  float param4;
  int32_t x;
  int32_t y;
  float z;

  MissionPoint(uint16_t _seq = 0, MAV_FRAME _frame = MAV_FRAME_GLOBAL, MAV_CMD _command = MAV_CMD_NAV_WAYPOINT, float _param1 = 0, float _param2 = 0, float _param3 = 0, float _param4 = 0, int32_t _x = 0, int32_t _y = 0, float _z = 0)
    : seq(_seq), frame(_frame), command(_command), param1(_param1), param2(_param2), param3(_param3), param4(_param4), x(_x), y(_y), z(_z) {}
};

const int MAVLINK_RX_PIN = D0;
const int MAVLINK_TX_PIN = D1;
const int SW_PIN = D7;
const uint8_t ESP_SYSTEM_ID = 100;
const uint8_t ESP_COMPONENT_ID = 1;
const unsigned long HEARTBEAT_INTERVAL_MS = 1000;
const unsigned long GLOBAL_POSITION_DELAY = 10000;
const unsigned long GPS_INTERVAL_US = 500000;
const float WAYPOINT_RADIUS = 0.1;
const float WAYPOINT_PASS_RADIUS = 10; // nw co to jest
const int MISSION_COUNT = 4; // przy zmianie tego zmień też prepare_new_mission, liczba punktów to n + 1, punkt zerowy ma być jakikolwiek

HardwareSerial mavlink_serial(1);
uint8_t fc_system_id = 0;
uint8_t fc_component_id = 0;
unsigned long last_heartbeat = 0;
uint16_t mission_count = 0;
uint32_t mission_opaque_id = 0;
MissionPoint mission_points[MISSION_COUNT] = {};

int32_t wp_x;
int32_t wp_y;
float wp_z;
float wp_delay;

bool is_mavlink_connected = false;
bool is_new_mission = false;
bool is_mission_cleared = false;
bool is_mission_uploaded = false;
bool is_mounted = false;

// ----------

// jakby obcinało msg, odkomentuj w setupie 'mavlinkSerial.setRxBufferSize' (i zwiększ jeśli trzeba)

// to se możesz wysyłać jak chcesz (aktualizowane co 0.5s)
int32_t curr_x;
int32_t curr_y;
int32_t curr_z; // [mm]
uint8_t fix_type; // https://mavlink.io/en/messages/common.html#GPS_FIX_TYPE
uint8_t satellites;

// to proszę wywołać jak odbierzesz punkty z serwera
void set_wp(int32_t x, int32_t y, float z, float delay) {
  wp_x = x;
  wp_y = y;
  wp_z = z;
  wp_delay = delay; // czas wiszenia
  is_new_mission = true;
}

void handle_mission_upload_success() {
  // tu się baw jak chcesz wysłać potwierdzenie
}

// param_id : char[16], chcesz se coś jeszcze wyciągnąć to jest funkcja o tej samej nazwie niżej
void handle_param_value(char param_id[], float param_value) {
}

// https://mavlink.io/en/messages/common.html#STATUSTEXT
// uwaga bo jeśli jakiś status ma >50 znaków to będzie podzielony na chunki
void handle_status(uint8_t severity, char text[], uint16_t msg_id, uint16_t seq) {
}

// ----------

void send_message(mavlink_message_t* msg) {
  uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buffer, msg);
  mavlink_serial.write(buffer, len);
}

void send_command_long(MAV_CMD command, float param1, float param2) {
  mavlink_message_t msg;
  mavlink_msg_command_long_pack(ESP_SYSTEM_ID, ESP_COMPONENT_ID, &msg, fc_system_id, fc_component_id, command, 0, param1, param2, 0, 0, 0, 0, 0);
  send_message(&msg);
}

void send_heartbeat() {
  mavlink_message_t msg;
  mavlink_msg_heartbeat_pack(ESP_SYSTEM_ID, ESP_COMPONENT_ID, &msg, MAV_TYPE_GCS, MAV_AUTOPILOT_GENERIC, 0, 0, MAV_STATE_ACTIVE);
  send_message(&msg);
}

void send_mission_clear_all() {
  mavlink_message_t msg;
  mavlink_msg_mission_clear_all_pack(ESP_SYSTEM_ID, ESP_COMPONENT_ID, &msg, fc_system_id, fc_component_id, MAV_MISSION_TYPE_MISSION);
  send_message(&msg);
}

void send_mission_count(uint16_t count) {
  mavlink_message_t msg;
  mavlink_msg_mission_count_pack(ESP_SYSTEM_ID, ESP_COMPONENT_ID, &msg, fc_system_id, fc_component_id, count, MAV_MISSION_TYPE_MISSION, 0);
  send_message(&msg);
}

void send_mission_item_int(MissionPoint& wp) {
  printf("send: %d\n", wp.seq);
  mavlink_message_t msg;
  mavlink_msg_mission_item_int_pack(ESP_SYSTEM_ID, ESP_COMPONENT_ID, &msg, fc_system_id, fc_component_id, wp.seq, wp.frame, wp.command, 0, 1, wp.param1, wp.param2, wp.param3, wp.param4, wp.x, wp.y, wp.z, MAV_MISSION_TYPE_MISSION);
  send_message(&msg);
}

// deprecated...
void send_mission_item(MissionPoint& wp) {
  mavlink_message_t msg;
  mavlink_msg_mission_item_pack(ESP_SYSTEM_ID, ESP_COMPONENT_ID, &msg, fc_system_id, fc_component_id, wp.seq, wp.frame, wp.command, 0, 1, wp.param1, wp.param2, wp.param3, wp.param4, wp.x, wp.y, wp.z, MAV_MISSION_TYPE_MISSION);
  send_message(&msg);
}

void send_set_mode_auto() {
  send_command_long(MAV_CMD_DO_SET_MODE, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 3);
}

void send_arm() {
  send_command_long(MAV_CMD_COMPONENT_ARM_DISARM, 1, 0);
}

void send_request_position_with_interval(float interval) {
  send_command_long(MAV_CMD_SET_MESSAGE_INTERVAL, MAVLINK_MSG_ID_GLOBAL_POSITION_INT, interval);
}

void prepare_new_mission(int32_t x, int32_t y, float z, float delay) {
  mission_points[0] = MissionPoint(0, MAV_FRAME_GLOBAL, MAV_CMD_NAV_WAYPOINT, 0, WAYPOINT_RADIUS, WAYPOINT_PASS_RADIUS, 0, wp_x, wp_y, wp_z);
  mission_points[1] = MissionPoint(1, MAV_FRAME_GLOBAL, MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, z);
  mission_points[2] = MissionPoint(2, MAV_FRAME_GLOBAL, MAV_CMD_NAV_WAYPOINT, delay, WAYPOINT_RADIUS, WAYPOINT_PASS_RADIUS, 0, x, y, z);
  mission_points[3] = MissionPoint(3, MAV_FRAME_GLOBAL, MAV_CMD_NAV_LAND, 0, 0, 0, 0, x, y, z); // możliwe że tu 0 zamiast z
}

void handle_param_value(mavlink_message_t* msg) {
  mavlink_param_value_t msg_val; // https://mavlink.io/en/messages/common.html#PARAM_VALUE
  mavlink_msg_param_value_decode(msg, &msg_val);
  handle_param_value(msg_val.param_id, msg_val.param_value);
}

void handle_statustext(mavlink_message_t* msg) {
  mavlink_statustext_t msg_status;
  mavlink_msg_statustext_decode(msg, &msg_status);
  Serial.printf("%d: %s\n", msg_status.severity, msg_status.text);
  handle_status(msg_status.severity, msg_status.text, msg_status.id, msg_status.chunk_seq);
}

void handle_mission_ack(mavlink_message_t* msg) {
  mavlink_mission_ack_t msg_ack;
  mavlink_msg_mission_ack_decode(msg, &msg_ack);
  if (!is_mission_cleared) {
    is_mission_cleared = true;
    send_mission_count(MISSION_COUNT);
    prepare_new_mission(wp_x, wp_y, wp_z, wp_delay);
    Serial.println("upload start");
    is_mission_uploaded = false;
  }
  else if (is_mission_uploaded) {
    is_mission_uploaded = false;
    // punkty wgrane - wyślij potwierdzenie do serwera
    handle_mission_upload_success();
    Serial.println("Punkty wgrane");
  }   
}

void handle_mission_request_int(mavlink_message_t* msg) {
  mavlink_mission_request_int_t msg_req;
  mavlink_msg_mission_request_int_decode(msg, &msg_req);
  uint16_t seq = msg_req.seq;
  if (seq < MISSION_COUNT) {
    send_mission_item_int(mission_points[seq]);
  }
  if (seq + 1 == MISSION_COUNT) {
    is_mission_uploaded = true;
  }
}

// deprecated...
void handle_mission_request(mavlink_message_t* msg) {
  mavlink_mission_request_t msg_req;
  mavlink_msg_mission_request_decode(msg, &msg_req);
  uint16_t seq = msg_req.seq;
  Serial.printf("req: %d\n", seq);
  if (seq < MISSION_COUNT) {
    send_mission_item_int(mission_points[seq]);
  }
  if (seq + 1 == MISSION_COUNT) {
    is_mission_uploaded = true;
  }
}

void handle_gps_raw_int(mavlink_message_t* msg) {
  mavlink_gps_raw_int_t msg_gps;
  mavlink_msg_gps_raw_int_decode(msg, &msg_gps);
  curr_x = msg_gps.lat;
  curr_y = msg_gps.lon;
  curr_z = msg_gps.alt;
  fix_type = msg_gps.fix_type;
  satellites = msg_gps.satellites_visible;
}

void setup() {
  pinMode(SW_PIN, INPUT_PULLUP);
  Serial.begin(9600);

  // mavlinkSerial.setRxBufferSize(2048);
  mavlink_serial.begin(115200, SERIAL_8N1, MAVLINK_RX_PIN, MAVLINK_TX_PIN);

  while (!is_mavlink_connected) {
    mavlink_message_t msg;
    mavlink_status_t status;

    while (mavlink_serial.available()) {
      uint8_t c = mavlink_serial.read();
      if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
        if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
          fc_system_id = msg.sysid;
          fc_component_id = msg.compid;
          is_mavlink_connected = true;
          break;
        }
      }
    }
    if (millis() - last_heartbeat >= HEARTBEAT_INTERVAL_MS) {
      send_heartbeat();
      last_heartbeat = millis();
    }
    delay(10);
  }
  send_request_position_with_interval(GPS_INTERVAL_US);
}

void loop() {
  mavlink_message_t msg;
  mavlink_status_t status;

  if (millis() - last_heartbeat >= HEARTBEAT_INTERVAL_MS) {
    send_heartbeat();
    last_heartbeat = millis();
  }

  if (!is_mounted && digitalRead(SW_PIN) == LOW) {
    is_mounted = true;
  }

  if (is_mounted && digitalRead(SW_PIN) == HIGH) {
    is_mounted = false;
    // set_wp(1, 1, 100, 50);
    send_set_mode_auto();
    send_arm();
  }

  if (is_new_mission) {
    is_new_mission = false;
    send_mission_clear_all();
    is_mission_cleared = false;
  }

  while (mavlink_serial.available()) {
    uint8_t c = mavlink_serial.read();
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      Serial.print("Odebrano MAVLink ID: ");
      Serial.println(msg.msgid);

      switch (msg.msgid) {
        case MAVLINK_MSG_ID_PARAM_VALUE:
          handle_param_value(&msg);
          break;
        case MAVLINK_MSG_ID_STATUSTEXT:
          handle_statustext(&msg);
          break;
        case MAVLINK_MSG_ID_MISSION_ACK:
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