#pragma once
// MESSAGE ACTUATOR_MOTORS PACKING

#define MAVLINK_MSG_ID_ACTUATOR_MOTORS 255


typedef struct __mavlink_actuator_motors_t {
 uint32_t time_boot_ms; /*< [ms] Timestamp (time since system boot).*/
 float thrust_1_raw; /*<  Thrust 1*/
 float thrust_2_raw; /*<  Thrust 2*/
 float thrust_3_raw; /*<  Thrust 3*/
 float thrust_4_raw; /*<  Thrust 4*/
 float thrust_5_raw; /*<  Thrust 5*/
 float thrust_6_raw; /*<  Thrust 6*/
 float thrust_7_raw; /*<  Thrust 7*/
 float thrust_8_raw; /*<  Thrust 8*/
 uint8_t num; /*<  Number of motors*/
} mavlink_actuator_motors_t;

#define MAVLINK_MSG_ID_ACTUATOR_MOTORS_LEN 37
#define MAVLINK_MSG_ID_ACTUATOR_MOTORS_MIN_LEN 37
#define MAVLINK_MSG_ID_255_LEN 37
#define MAVLINK_MSG_ID_255_MIN_LEN 37

#define MAVLINK_MSG_ID_ACTUATOR_MOTORS_CRC 20
#define MAVLINK_MSG_ID_255_CRC 20



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ACTUATOR_MOTORS { \
    255, \
    "ACTUATOR_MOTORS", \
    10, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_actuator_motors_t, time_boot_ms) }, \
         { "thrust_1_raw", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_actuator_motors_t, thrust_1_raw) }, \
         { "thrust_2_raw", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_actuator_motors_t, thrust_2_raw) }, \
         { "thrust_3_raw", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_actuator_motors_t, thrust_3_raw) }, \
         { "thrust_4_raw", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_actuator_motors_t, thrust_4_raw) }, \
         { "thrust_5_raw", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_actuator_motors_t, thrust_5_raw) }, \
         { "thrust_6_raw", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_actuator_motors_t, thrust_6_raw) }, \
         { "thrust_7_raw", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_actuator_motors_t, thrust_7_raw) }, \
         { "thrust_8_raw", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_actuator_motors_t, thrust_8_raw) }, \
         { "num", NULL, MAVLINK_TYPE_UINT8_T, 0, 36, offsetof(mavlink_actuator_motors_t, num) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ACTUATOR_MOTORS { \
    "ACTUATOR_MOTORS", \
    10, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_actuator_motors_t, time_boot_ms) }, \
         { "thrust_1_raw", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_actuator_motors_t, thrust_1_raw) }, \
         { "thrust_2_raw", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_actuator_motors_t, thrust_2_raw) }, \
         { "thrust_3_raw", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_actuator_motors_t, thrust_3_raw) }, \
         { "thrust_4_raw", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_actuator_motors_t, thrust_4_raw) }, \
         { "thrust_5_raw", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_actuator_motors_t, thrust_5_raw) }, \
         { "thrust_6_raw", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_actuator_motors_t, thrust_6_raw) }, \
         { "thrust_7_raw", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_actuator_motors_t, thrust_7_raw) }, \
         { "thrust_8_raw", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_actuator_motors_t, thrust_8_raw) }, \
         { "num", NULL, MAVLINK_TYPE_UINT8_T, 0, 36, offsetof(mavlink_actuator_motors_t, num) }, \
         } \
}
#endif

/**
 * @brief Pack a actuator_motors message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param thrust_1_raw  Thrust 1
 * @param thrust_2_raw  Thrust 2
 * @param thrust_3_raw  Thrust 3
 * @param thrust_4_raw  Thrust 4
 * @param thrust_5_raw  Thrust 5
 * @param thrust_6_raw  Thrust 6
 * @param thrust_7_raw  Thrust 7
 * @param thrust_8_raw  Thrust 8
 * @param num  Number of motors
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_actuator_motors_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t time_boot_ms, float thrust_1_raw, float thrust_2_raw, float thrust_3_raw, float thrust_4_raw, float thrust_5_raw, float thrust_6_raw, float thrust_7_raw, float thrust_8_raw, uint8_t num)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ACTUATOR_MOTORS_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, thrust_1_raw);
    _mav_put_float(buf, 8, thrust_2_raw);
    _mav_put_float(buf, 12, thrust_3_raw);
    _mav_put_float(buf, 16, thrust_4_raw);
    _mav_put_float(buf, 20, thrust_5_raw);
    _mav_put_float(buf, 24, thrust_6_raw);
    _mav_put_float(buf, 28, thrust_7_raw);
    _mav_put_float(buf, 32, thrust_8_raw);
    _mav_put_uint8_t(buf, 36, num);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ACTUATOR_MOTORS_LEN);
#else
    mavlink_actuator_motors_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.thrust_1_raw = thrust_1_raw;
    packet.thrust_2_raw = thrust_2_raw;
    packet.thrust_3_raw = thrust_3_raw;
    packet.thrust_4_raw = thrust_4_raw;
    packet.thrust_5_raw = thrust_5_raw;
    packet.thrust_6_raw = thrust_6_raw;
    packet.thrust_7_raw = thrust_7_raw;
    packet.thrust_8_raw = thrust_8_raw;
    packet.num = num;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ACTUATOR_MOTORS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ACTUATOR_MOTORS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ACTUATOR_MOTORS_MIN_LEN, MAVLINK_MSG_ID_ACTUATOR_MOTORS_LEN, MAVLINK_MSG_ID_ACTUATOR_MOTORS_CRC);
}

/**
 * @brief Pack a actuator_motors message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param thrust_1_raw  Thrust 1
 * @param thrust_2_raw  Thrust 2
 * @param thrust_3_raw  Thrust 3
 * @param thrust_4_raw  Thrust 4
 * @param thrust_5_raw  Thrust 5
 * @param thrust_6_raw  Thrust 6
 * @param thrust_7_raw  Thrust 7
 * @param thrust_8_raw  Thrust 8
 * @param num  Number of motors
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_actuator_motors_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t time_boot_ms,float thrust_1_raw,float thrust_2_raw,float thrust_3_raw,float thrust_4_raw,float thrust_5_raw,float thrust_6_raw,float thrust_7_raw,float thrust_8_raw,uint8_t num)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ACTUATOR_MOTORS_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, thrust_1_raw);
    _mav_put_float(buf, 8, thrust_2_raw);
    _mav_put_float(buf, 12, thrust_3_raw);
    _mav_put_float(buf, 16, thrust_4_raw);
    _mav_put_float(buf, 20, thrust_5_raw);
    _mav_put_float(buf, 24, thrust_6_raw);
    _mav_put_float(buf, 28, thrust_7_raw);
    _mav_put_float(buf, 32, thrust_8_raw);
    _mav_put_uint8_t(buf, 36, num);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ACTUATOR_MOTORS_LEN);
#else
    mavlink_actuator_motors_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.thrust_1_raw = thrust_1_raw;
    packet.thrust_2_raw = thrust_2_raw;
    packet.thrust_3_raw = thrust_3_raw;
    packet.thrust_4_raw = thrust_4_raw;
    packet.thrust_5_raw = thrust_5_raw;
    packet.thrust_6_raw = thrust_6_raw;
    packet.thrust_7_raw = thrust_7_raw;
    packet.thrust_8_raw = thrust_8_raw;
    packet.num = num;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ACTUATOR_MOTORS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ACTUATOR_MOTORS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ACTUATOR_MOTORS_MIN_LEN, MAVLINK_MSG_ID_ACTUATOR_MOTORS_LEN, MAVLINK_MSG_ID_ACTUATOR_MOTORS_CRC);
}

/**
 * @brief Encode a actuator_motors struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param actuator_motors C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_actuator_motors_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_actuator_motors_t* actuator_motors)
{
    return mavlink_msg_actuator_motors_pack(system_id, component_id, msg, actuator_motors->time_boot_ms, actuator_motors->thrust_1_raw, actuator_motors->thrust_2_raw, actuator_motors->thrust_3_raw, actuator_motors->thrust_4_raw, actuator_motors->thrust_5_raw, actuator_motors->thrust_6_raw, actuator_motors->thrust_7_raw, actuator_motors->thrust_8_raw, actuator_motors->num);
}

/**
 * @brief Encode a actuator_motors struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param actuator_motors C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_actuator_motors_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_actuator_motors_t* actuator_motors)
{
    return mavlink_msg_actuator_motors_pack_chan(system_id, component_id, chan, msg, actuator_motors->time_boot_ms, actuator_motors->thrust_1_raw, actuator_motors->thrust_2_raw, actuator_motors->thrust_3_raw, actuator_motors->thrust_4_raw, actuator_motors->thrust_5_raw, actuator_motors->thrust_6_raw, actuator_motors->thrust_7_raw, actuator_motors->thrust_8_raw, actuator_motors->num);
}

/**
 * @brief Send a actuator_motors message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param thrust_1_raw  Thrust 1
 * @param thrust_2_raw  Thrust 2
 * @param thrust_3_raw  Thrust 3
 * @param thrust_4_raw  Thrust 4
 * @param thrust_5_raw  Thrust 5
 * @param thrust_6_raw  Thrust 6
 * @param thrust_7_raw  Thrust 7
 * @param thrust_8_raw  Thrust 8
 * @param num  Number of motors
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_actuator_motors_send(mavlink_channel_t chan, uint32_t time_boot_ms, float thrust_1_raw, float thrust_2_raw, float thrust_3_raw, float thrust_4_raw, float thrust_5_raw, float thrust_6_raw, float thrust_7_raw, float thrust_8_raw, uint8_t num)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ACTUATOR_MOTORS_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, thrust_1_raw);
    _mav_put_float(buf, 8, thrust_2_raw);
    _mav_put_float(buf, 12, thrust_3_raw);
    _mav_put_float(buf, 16, thrust_4_raw);
    _mav_put_float(buf, 20, thrust_5_raw);
    _mav_put_float(buf, 24, thrust_6_raw);
    _mav_put_float(buf, 28, thrust_7_raw);
    _mav_put_float(buf, 32, thrust_8_raw);
    _mav_put_uint8_t(buf, 36, num);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACTUATOR_MOTORS, buf, MAVLINK_MSG_ID_ACTUATOR_MOTORS_MIN_LEN, MAVLINK_MSG_ID_ACTUATOR_MOTORS_LEN, MAVLINK_MSG_ID_ACTUATOR_MOTORS_CRC);
#else
    mavlink_actuator_motors_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.thrust_1_raw = thrust_1_raw;
    packet.thrust_2_raw = thrust_2_raw;
    packet.thrust_3_raw = thrust_3_raw;
    packet.thrust_4_raw = thrust_4_raw;
    packet.thrust_5_raw = thrust_5_raw;
    packet.thrust_6_raw = thrust_6_raw;
    packet.thrust_7_raw = thrust_7_raw;
    packet.thrust_8_raw = thrust_8_raw;
    packet.num = num;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACTUATOR_MOTORS, (const char *)&packet, MAVLINK_MSG_ID_ACTUATOR_MOTORS_MIN_LEN, MAVLINK_MSG_ID_ACTUATOR_MOTORS_LEN, MAVLINK_MSG_ID_ACTUATOR_MOTORS_CRC);
#endif
}

/**
 * @brief Send a actuator_motors message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_actuator_motors_send_struct(mavlink_channel_t chan, const mavlink_actuator_motors_t* actuator_motors)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_actuator_motors_send(chan, actuator_motors->time_boot_ms, actuator_motors->thrust_1_raw, actuator_motors->thrust_2_raw, actuator_motors->thrust_3_raw, actuator_motors->thrust_4_raw, actuator_motors->thrust_5_raw, actuator_motors->thrust_6_raw, actuator_motors->thrust_7_raw, actuator_motors->thrust_8_raw, actuator_motors->num);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACTUATOR_MOTORS, (const char *)actuator_motors, MAVLINK_MSG_ID_ACTUATOR_MOTORS_MIN_LEN, MAVLINK_MSG_ID_ACTUATOR_MOTORS_LEN, MAVLINK_MSG_ID_ACTUATOR_MOTORS_CRC);
#endif
}

#if MAVLINK_MSG_ID_ACTUATOR_MOTORS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_actuator_motors_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, float thrust_1_raw, float thrust_2_raw, float thrust_3_raw, float thrust_4_raw, float thrust_5_raw, float thrust_6_raw, float thrust_7_raw, float thrust_8_raw, uint8_t num)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, thrust_1_raw);
    _mav_put_float(buf, 8, thrust_2_raw);
    _mav_put_float(buf, 12, thrust_3_raw);
    _mav_put_float(buf, 16, thrust_4_raw);
    _mav_put_float(buf, 20, thrust_5_raw);
    _mav_put_float(buf, 24, thrust_6_raw);
    _mav_put_float(buf, 28, thrust_7_raw);
    _mav_put_float(buf, 32, thrust_8_raw);
    _mav_put_uint8_t(buf, 36, num);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACTUATOR_MOTORS, buf, MAVLINK_MSG_ID_ACTUATOR_MOTORS_MIN_LEN, MAVLINK_MSG_ID_ACTUATOR_MOTORS_LEN, MAVLINK_MSG_ID_ACTUATOR_MOTORS_CRC);
#else
    mavlink_actuator_motors_t *packet = (mavlink_actuator_motors_t *)msgbuf;
    packet->time_boot_ms = time_boot_ms;
    packet->thrust_1_raw = thrust_1_raw;
    packet->thrust_2_raw = thrust_2_raw;
    packet->thrust_3_raw = thrust_3_raw;
    packet->thrust_4_raw = thrust_4_raw;
    packet->thrust_5_raw = thrust_5_raw;
    packet->thrust_6_raw = thrust_6_raw;
    packet->thrust_7_raw = thrust_7_raw;
    packet->thrust_8_raw = thrust_8_raw;
    packet->num = num;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACTUATOR_MOTORS, (const char *)packet, MAVLINK_MSG_ID_ACTUATOR_MOTORS_MIN_LEN, MAVLINK_MSG_ID_ACTUATOR_MOTORS_LEN, MAVLINK_MSG_ID_ACTUATOR_MOTORS_CRC);
#endif
}
#endif

#endif

// MESSAGE ACTUATOR_MOTORS UNPACKING


/**
 * @brief Get field time_boot_ms from actuator_motors message
 *
 * @return [ms] Timestamp (time since system boot).
 */
static inline uint32_t mavlink_msg_actuator_motors_get_time_boot_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field thrust_1_raw from actuator_motors message
 *
 * @return  Thrust 1
 */
static inline float mavlink_msg_actuator_motors_get_thrust_1_raw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field thrust_2_raw from actuator_motors message
 *
 * @return  Thrust 2
 */
static inline float mavlink_msg_actuator_motors_get_thrust_2_raw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field thrust_3_raw from actuator_motors message
 *
 * @return  Thrust 3
 */
static inline float mavlink_msg_actuator_motors_get_thrust_3_raw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field thrust_4_raw from actuator_motors message
 *
 * @return  Thrust 4
 */
static inline float mavlink_msg_actuator_motors_get_thrust_4_raw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field thrust_5_raw from actuator_motors message
 *
 * @return  Thrust 5
 */
static inline float mavlink_msg_actuator_motors_get_thrust_5_raw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field thrust_6_raw from actuator_motors message
 *
 * @return  Thrust 6
 */
static inline float mavlink_msg_actuator_motors_get_thrust_6_raw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field thrust_7_raw from actuator_motors message
 *
 * @return  Thrust 7
 */
static inline float mavlink_msg_actuator_motors_get_thrust_7_raw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field thrust_8_raw from actuator_motors message
 *
 * @return  Thrust 8
 */
static inline float mavlink_msg_actuator_motors_get_thrust_8_raw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field num from actuator_motors message
 *
 * @return  Number of motors
 */
static inline uint8_t mavlink_msg_actuator_motors_get_num(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  36);
}

/**
 * @brief Decode a actuator_motors message into a struct
 *
 * @param msg The message to decode
 * @param actuator_motors C-struct to decode the message contents into
 */
static inline void mavlink_msg_actuator_motors_decode(const mavlink_message_t* msg, mavlink_actuator_motors_t* actuator_motors)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    actuator_motors->time_boot_ms = mavlink_msg_actuator_motors_get_time_boot_ms(msg);
    actuator_motors->thrust_1_raw = mavlink_msg_actuator_motors_get_thrust_1_raw(msg);
    actuator_motors->thrust_2_raw = mavlink_msg_actuator_motors_get_thrust_2_raw(msg);
    actuator_motors->thrust_3_raw = mavlink_msg_actuator_motors_get_thrust_3_raw(msg);
    actuator_motors->thrust_4_raw = mavlink_msg_actuator_motors_get_thrust_4_raw(msg);
    actuator_motors->thrust_5_raw = mavlink_msg_actuator_motors_get_thrust_5_raw(msg);
    actuator_motors->thrust_6_raw = mavlink_msg_actuator_motors_get_thrust_6_raw(msg);
    actuator_motors->thrust_7_raw = mavlink_msg_actuator_motors_get_thrust_7_raw(msg);
    actuator_motors->thrust_8_raw = mavlink_msg_actuator_motors_get_thrust_8_raw(msg);
    actuator_motors->num = mavlink_msg_actuator_motors_get_num(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ACTUATOR_MOTORS_LEN? msg->len : MAVLINK_MSG_ID_ACTUATOR_MOTORS_LEN;
        memset(actuator_motors, 0, MAVLINK_MSG_ID_ACTUATOR_MOTORS_LEN);
    memcpy(actuator_motors, _MAV_PAYLOAD(msg), len);
#endif
}
