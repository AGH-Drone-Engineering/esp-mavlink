#include "handlers.h"
#include <Arduino.h>
#include <MAVLink.h> // albo <common/mavlink.h>

void handle_mission_upload_success()
{
    // tu się baw jak chcesz wysłać potwierdzenie
}

// param_id : char[16], chcesz se coś jeszcze wyciągnąć to jest funkcja o tej samej nazwie niżej
void handle_param_value(char param_id[], float param_value)
{
    Serial.printf("PARAM [%s] = %.2f\n", param_id, param_value);
}

// https://mavlink.io/en/messages/common.html#STATUSTEXT
// uwaga bo jeśli jakiś status ma >50 znaków to będzie podzielony na chunki
void handle_status(uint8_t severity, char text[], uint16_t msg_id, uint16_t seq)
{

    // Serial.print("🔔 STATUSTEXT odebrany:\n");
    // Serial.print(" - Poziom (severity): ");
    // Serial.println(severity);
    if (strstr(text, "ARMING_CHECK") != nullptr || strstr(text, "PreArm") != nullptr)
    {
        arm_errors = String(text);
    }
    // Serial.print(" - Tekst: ");
    // Serial.println(text);

    // Serial.print(" - ID wiadomości: ");
    // Serial.println(msg_id);

    // Serial.print(" - Chunk sequence: ");
    // Serial.println(seq);
}

// ----------

void send_message(mavlink_message_t *msg)
{
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, msg);
    mavlink_serial.write(buffer, len);
}

void send_command_long(MAV_CMD command, float param1, float param2)
{
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(ESP_SYSTEM_ID, ESP_COMPONENT_ID, &msg, fc_system_id, fc_component_id, command, 0, param1, param2, 0, 0, 0, 0, 0);
    send_message(&msg);
}

void send_heartbeat()
{
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(ESP_SYSTEM_ID, ESP_COMPONENT_ID, &msg, MAV_TYPE_GCS, MAV_AUTOPILOT_GENERIC, 0, 0, MAV_STATE_ACTIVE);
    send_message(&msg);
}

void send_mission_clear_all()
{
    mavlink_message_t msg;
    mavlink_msg_mission_clear_all_pack(ESP_SYSTEM_ID, ESP_COMPONENT_ID, &msg, fc_system_id, fc_component_id, MAV_MISSION_TYPE_MISSION);
    send_message(&msg);
}

void send_mission_count(uint16_t count)
{
    mavlink_message_t msg;
    mavlink_msg_mission_count_pack(ESP_SYSTEM_ID, ESP_COMPONENT_ID, &msg, fc_system_id, fc_component_id, count, MAV_MISSION_TYPE_MISSION, 0);
    send_message(&msg);
}

void send_mission_item_int(MissionPoint &wp)
{
    printf("send: %d\n", wp.seq);
    mavlink_message_t msg;
    mavlink_msg_mission_item_int_pack(ESP_SYSTEM_ID, ESP_COMPONENT_ID, &msg, fc_system_id, fc_component_id, wp.seq, wp.frame, wp.command, 0, 1, wp.param1, wp.param2, wp.param3, wp.param4, wp.x, wp.y, wp.z, MAV_MISSION_TYPE_MISSION);
    send_message(&msg);
}

// deprecated...
void send_mission_item(MissionPoint &wp)
{
    mavlink_message_t msg;
    mavlink_msg_mission_item_pack(ESP_SYSTEM_ID, ESP_COMPONENT_ID, &msg, fc_system_id, fc_component_id, wp.seq, wp.frame, wp.command, 0, 1, wp.param1, wp.param2, wp.param3, wp.param4, wp.x, wp.y, wp.z, MAV_MISSION_TYPE_MISSION);
    send_message(&msg);
}

void send_set_mode_auto()
{
    send_command_long(MAV_CMD_DO_SET_MODE, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 3);
}

void send_arm()
{
    send_command_long(MAV_CMD_COMPONENT_ARM_DISARM, 1, 0);
}

void send_request_position_with_interval(float interval)
{
    send_command_long(MAV_CMD_SET_MESSAGE_INTERVAL, MAVLINK_MSG_ID_GPS_RAW_INT, interval);
}

void handle_sys_status(mavlink_message_t *msg)
{
    mavlink_sys_status_t sys2;
    mavlink_msg_sys_status_decode(msg, &sys2);

    if (sys2.errors_count1 > 0 || sys2.errors_count2 > 0 || sys2.errors_count3 > 0 || sys2.errors_count4 > 0)
    {

        can_arm = "System ma błędy – arming zablokowany";
    }
    else
    {
        can_arm = "System zdrowy – arming możliwy";
    }
}

void prepare_new_mission(int32_t x, int32_t y, float z, float delay)
{

    mission_points[0] = MissionPoint(0, MAV_FRAME_GLOBAL, MAV_CMD_NAV_WAYPOINT, 0, WAYPOINT_RADIUS, WAYPOINT_PASS_RADIUS, 0, wp_x, wp_y, wp_z);
    mission_points[1] = MissionPoint(1, MAV_FRAME_GLOBAL, MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, z);
    mission_points[2] = MissionPoint(2, MAV_FRAME_GLOBAL, MAV_CMD_NAV_WAYPOINT, delay, WAYPOINT_RADIUS, WAYPOINT_PASS_RADIUS, 0, x, y, z);
    mission_points[3] = MissionPoint(3, MAV_FRAME_GLOBAL, MAV_CMD_NAV_LAND, 0, 0, 0, 0, x, y, z); // możliwe że tu 0 zamiast z
}

void handle_param_value(mavlink_message_t *msg)
{
    mavlink_param_value_t msg_val; // https://mavlink.io/en/messages/common.html#PARAM_VALUE
    mavlink_msg_param_value_decode(msg, &msg_val);
    handle_param_value(msg_val.param_id, msg_val.param_value);
}

void handle_statustext(mavlink_message_t *msg)
{

    mavlink_statustext_t msg_status;
    mavlink_msg_statustext_decode(msg, &msg_status);
    arm_errors = "0";
    handle_status(msg_status.severity, msg_status.text, msg_status.id, msg_status.chunk_seq);
}

void handle_mission_ack(mavlink_message_t *msg)
{
    mavlink_mission_ack_t msg_ack;
    mavlink_msg_mission_ack_decode(msg, &msg_ack);
    if (!is_mission_cleared)
    {
        is_mission_cleared = true;
        send_mission_count(MISSION_COUNT);
        prepare_new_mission(wp_x, wp_y, wp_z, wp_delay);
        Serial.println("upload start");
        is_mission_uploaded = false;
    }
    else if (is_mission_uploaded)
    {
        is_mission_uploaded = false;
        // punkty wgrane - wyślij potwierdzenie do serwera
        handle_mission_upload_success();
        mission_ok = "Punkty wgrane";
    }
}

void handle_mission_request_int(mavlink_message_t *msg)
{
    mavlink_mission_request_int_t msg_req;
    mavlink_msg_mission_request_int_decode(msg, &msg_req);
    uint16_t seq = msg_req.seq;
    if (seq < MISSION_COUNT)
    {
        send_mission_item_int(mission_points[seq]);
    }
    if (seq + 1 == MISSION_COUNT)
    {
        is_mission_uploaded = true;
    }
}

// deprecated...
void handle_mission_request(mavlink_message_t *msg)
{
    mavlink_mission_request_t msg_req;
    mavlink_msg_mission_request_decode(msg, &msg_req);
    uint16_t seq = msg_req.seq;
    Serial.printf("req: %d\n", seq);
    if (seq < MISSION_COUNT)
    {
        send_mission_item_int(mission_points[seq]);
    }
    if (seq + 1 == MISSION_COUNT)
    {
        is_mission_uploaded = true;
    }
}

void handle_gps_raw_int(mavlink_message_t *msg)
{
    mavlink_gps_raw_int_t msg_gps;
    mavlink_msg_gps_raw_int_decode(msg, &msg_gps);
    curr_x = msg_gps.lat ;
    curr_y = msg_gps.lon ;
    curr_z_m = msg_gps.alt / 1000;
    fix_type = msg_gps.fix_type;
    satellites = msg_gps.satellites_visible;
    // Serial.printf("Z gpsa: lat:%d lon:%d alt:%d sats:%d\n", curr_x, curr_y, curr_z, satellites);
}
