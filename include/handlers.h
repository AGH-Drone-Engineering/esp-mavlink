#ifndef HANDLERS_H
#define HANDLERS_H

#include <Arduino.h>
#include <MAVLink.h>
// Stałe związane z misją
constexpr int MISSION_COUNT = 4;
constexpr float WAYPOINT_RADIUS = 0.1f;
constexpr float WAYPOINT_PASS_RADIUS = 10.0f;

// Struktura punktu misji
struct MissionPoint
{
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

    MissionPoint(uint16_t _seq = 0, MAV_FRAME _frame = MAV_FRAME_GLOBAL, MAV_CMD _command = MAV_CMD_NAV_WAYPOINT,
                 float _param1 = 0, float _param2 = 0, float _param3 = 0, float _param4 = 0,
                 int32_t _x = 0, int32_t _y = 0, float _z = 0)
        : seq(_seq), frame(_frame), command(_command),
          param1(_param1), param2(_param2), param3(_param3), param4(_param4),
          x(_x), y(_y), z(_z) {}
};

// Zmienne globalne (zadeklarowane w main.cpp)
extern HardwareSerial mavlink_serial;

extern bool is_mission_cleared;
extern bool is_mission_uploaded;
extern uint8_t fc_system_id, fc_component_id;
extern const uint8_t ESP_SYSTEM_ID, ESP_COMPONENT_ID;
extern float curr_x, curr_y, curr_z_m;
extern uint8_t fix_type, satellites;
extern int32_t wp_x, wp_y;
extern float wp_z, wp_delay;
extern String can_arm;
extern String arm_errors;
// Tablica punktów misji
extern MissionPoint mission_points[MISSION_COUNT];

// Deklaracje funkcji handlerów i komunikacji
void handle_mission_upload_success();
void handle_param_value(char param_id[], float param_value);
void handle_status(uint8_t severity, char text[], uint16_t msg_id, uint16_t seq);

void send_message(mavlink_message_t *msg);
void send_command_long(MAV_CMD command, float param1, float param2);
void send_heartbeat();
void send_mission_clear_all();
void send_mission_count(uint16_t count);
void send_mission_item_int(MissionPoint &wp);
void send_mission_item(MissionPoint &wp); // deprecated
void send_set_mode_auto();
void send_arm();
void send_request_position_with_interval(float interval);

void prepare_new_mission(int32_t x, int32_t y, float z, float delay);

// Handlery wiadomości MAVLink
void handle_param_value(mavlink_message_t *msg);
void handle_statustext(mavlink_message_t *msg);
void handle_mission_ack(mavlink_message_t *msg);
void handle_mission_request_int(mavlink_message_t *msg);
void handle_mission_request(mavlink_message_t *msg); // deprecated
void handle_gps_raw_int(mavlink_message_t *msg);
void handle_sys_status(mavlink_message_t *msg);

#endif // HANDLERS_H
