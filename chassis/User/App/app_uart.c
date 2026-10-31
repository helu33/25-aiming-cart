#include "app_uart.h"

#include "app_motor.h"
#include "app_odometry.h"
#include "board_config.h"
#include "bsp_encoder.h"
#include "bsp_uart.h"
#include "stm32f1xx_hal.h"
#include <stddef.h>
#include <string.h>

#define PROTOCOL_SOF1                (0xAAU)
#define PROTOCOL_SOF2                (0x55U)
#define PROTOCOL_VERSION             (0x10U)
#define PROTOCOL_FLAG_ACK_REQUIRED   (0x01U)
#define PROTOCOL_FLAG_RESPONSE       (0x02U)
#define PROTOCOL_MAX_PAYLOAD         (64U)
#define PROTOCOL_BODY_MAX            (6U + PROTOCOL_MAX_PAYLOAD + 2U)

#define MSG_PING                     (0x01U)
#define MSG_HEARTBEAT                (0x02U)
#define MSG_SET_MODE                 (0x10U)
#define MSG_CMD_VELOCITY             (0x11U)
#define MSG_ACK                      (0x80U)
#define MSG_CHASSIS_STATE            (0x82U)
#define MSG_IMU_STATE                (0x83U)

#define MODE_STOPPED                 (0U)
#define MODE_VELOCITY                (2U)

#define ACK_OK                       (0U)
#define ACK_UNSUPPORTED_MSG          (1U)
#define ACK_INVALID_LENGTH           (2U)
#define ACK_INVALID_VALUE            (3U)

#define TEXT_COMMAND_SIZE            (32U)

typedef struct {
    uint8_t state;
    uint8_t body[PROTOCOL_BODY_MAX];
    uint16_t body_length;
    uint16_t expected_length;
} frame_parser_t;

static frame_parser_t parser;
static uint8_t control_mode = MODE_STOPPED;
static uint8_t velocity_command_seen;
static uint32_t last_velocity_command_ms;
static uint16_t fault_flags;
static uint8_t tx_sequence;

static uint16_t crc16_ccitt_false(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;
    uint16_t i;
    uint8_t bit;
    for (i = 0U; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8U;
        for (bit = 0U; bit < 8U; bit++) {
            crc = ((crc & 0x8000U) != 0U) ? (uint16_t)((crc << 1U) ^ 0x1021U) :
                                            (uint16_t)(crc << 1U);
        }
    }
    return crc;
}

static int16_t read_i16_le(const uint8_t *data)
{
    return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8U));
}

static void write_u16_le(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8U);
}

static void write_i16_le(uint8_t *data, int16_t value)
{
    write_u16_le(data, (uint16_t)value);
}

static void write_u32_le(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8U);
    data[2] = (uint8_t)(value >> 16U);
    data[3] = (uint8_t)(value >> 24U);
}

static void write_i32_le(uint8_t *data, int32_t value)
{
    write_u32_le(data, (uint32_t)value);
}

static int16_t float_to_i16(float value)
{
    if (value >= 32767.0f) return INT16_MAX;
    if (value <= -32768.0f) return INT16_MIN;
    return (int16_t)((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));
}

static void frame_send(uint8_t message_id, uint8_t flags, const uint8_t *payload, uint16_t payload_length)
{
    uint8_t frame[10U + PROTOCOL_MAX_PAYLOAD];
    uint16_t crc;
    if (payload_length > PROTOCOL_MAX_PAYLOAD) return;
    frame[0] = PROTOCOL_SOF1;
    frame[1] = PROTOCOL_SOF2;
    frame[2] = PROTOCOL_VERSION;
    frame[3] = flags;
    frame[4] = tx_sequence++;
    frame[5] = message_id;
    write_u16_le(&frame[6], payload_length);
    if ((payload != NULL) && (payload_length != 0U)) memcpy(&frame[8], payload, payload_length);
    crc = crc16_ccitt_false(&frame[2], (uint16_t)(6U + payload_length));
    write_u16_le(&frame[8U + payload_length], crc);
    (void)uart_bsp_write(frame, (uint16_t)(10U + payload_length));
}

static void ack_send(uint8_t request_sequence, uint8_t request_message_id, uint8_t status)
{
    uint8_t payload[3];
    payload[0] = request_sequence;
    payload[1] = request_message_id;
    payload[2] = status;
    frame_send(MSG_ACK, PROTOCOL_FLAG_RESPONSE, payload, (uint16_t)sizeof(payload));
}

static uint8_t command_execute(const uint8_t *body, uint16_t payload_length)
{
    const uint8_t sequence = body[2];
    const uint8_t message_id = body[3];
    const uint8_t *payload = &body[6];
    uint8_t status = ACK_OK;

    if (message_id == MSG_PING) {
        if (payload_length != 0U) status = ACK_INVALID_LENGTH;
    } else if (message_id == MSG_HEARTBEAT) {
        if (payload_length != 4U) status = ACK_INVALID_LENGTH;
    } else if (message_id == MSG_SET_MODE) {
        if (payload_length != 1U) {
            status = ACK_INVALID_LENGTH;
        } else if (payload[0] == MODE_STOPPED) {
            control_mode = MODE_STOPPED;
            velocity_command_seen = 0U;
            app_motor_stop();
        } else if (payload[0] == MODE_VELOCITY) {
            control_mode = MODE_VELOCITY;
            velocity_command_seen = 0U;
            app_motor_stop();
        } else {
            status = ACK_INVALID_VALUE;
        }
    } else if (message_id == MSG_CMD_VELOCITY) {
        if (payload_length != 6U) {
            status = ACK_INVALID_LENGTH;
        } else if (control_mode != MODE_VELOCITY) {
            /* A STOP command is latched; only SET_MODE(VELOCITY) may re-arm motion. */
            status = ACK_INVALID_VALUE;
        } else {
            const float linear_x_m_s = (float)read_i16_le(&payload[0]) / 1000.0f;
            const float angular_z_rad_s = (float)read_i16_le(&payload[2]) / 1000.0f;
            app_motor_set_chassis_velocity(linear_x_m_s, angular_z_rad_s);
            velocity_command_seen = 1U;
            last_velocity_command_ms = HAL_GetTick();
            fault_flags &= (uint16_t)~CAR_FAULT_CMD_TIMEOUT;
        }
    } else {
        status = ACK_UNSUPPORTED_MSG;
    }

    if (((body[1] & PROTOCOL_FLAG_ACK_REQUIRED) != 0U) ||
        (message_id == MSG_PING) || (status != ACK_OK)) {
        ack_send(sequence, message_id, status);
    }
    return status;
}

static void binary_byte_process(uint8_t byte)
{
    if (parser.state == 0U) {
        if (byte == PROTOCOL_SOF1) parser.state = 1U;
        return;
    }
    if (parser.state == 1U) {
        if (byte == PROTOCOL_SOF2) {
            parser.state = 2U;
            parser.body_length = 0U;
            parser.expected_length = 0U;
        } else if (byte != PROTOCOL_SOF1) {
            parser.state = 0U;
        }
        return;
    }

    if (parser.body_length >= PROTOCOL_BODY_MAX) {
        parser.state = 0U;
        return;
    }
    parser.body[parser.body_length++] = byte;
    if (parser.body_length == 6U) {
        const uint16_t payload_length = (uint16_t)parser.body[4] |
                                        ((uint16_t)parser.body[5] << 8U);
        if ((parser.body[0] != PROTOCOL_VERSION) ||
            ((parser.body[1] & (uint8_t)~(PROTOCOL_FLAG_ACK_REQUIRED | PROTOCOL_FLAG_RESPONSE)) != 0U) ||
            (payload_length > PROTOCOL_MAX_PAYLOAD)) {
            parser.state = 0U;
            return;
        }
        parser.expected_length = (uint16_t)(6U + payload_length + 2U);
    }
    if ((parser.expected_length != 0U) && (parser.body_length == parser.expected_length)) {
        const uint16_t payload_length = (uint16_t)(parser.expected_length - 8U);
        const uint16_t calculated_crc = crc16_ccitt_false(parser.body, (uint16_t)(6U + payload_length));
        const uint16_t received_crc = (uint16_t)parser.body[6U + payload_length] |
                                      ((uint16_t)parser.body[7U + payload_length] << 8U);
        if (calculated_crc == received_crc) {
            (void)command_execute(parser.body, payload_length);
        } else {
            fault_flags |= CAR_FAULT_RX_CRC;
        }
        parser.state = 0U;
    }
}

static void text_byte_process(uint8_t byte)
{
    static char command[TEXT_COMMAND_SIZE];
    static uint8_t length;
    if ((byte == '\r') || (byte == '\n')) {
        if (length == 0U) return;
        command[length] = '\0';
        if (strcmp(command, "STOP") == 0) {
            control_mode = MODE_STOPPED;
            velocity_command_seen = 0U;
            app_motor_stop();
            (void)uart_bsp_write_str("OK STOP\r\n");
        } else if (strcmp(command, "RUN") == 0) {
            control_mode = MODE_VELOCITY;
            velocity_command_seen = 0U;
            app_motor_stop();
            (void)uart_bsp_write_str("OK RUN\r\n");
        } else {
            (void)uart_bsp_write_str("ERR UNKNOWN CMD\r\n");
        }
        length = 0U;
    } else if ((byte >= 0x20U) && (byte <= 0x7EU)) {
        if (length < (TEXT_COMMAND_SIZE - 1U)) command[length++] = (char)byte;
        else length = 0U;
    }
}

void car_uart_command_poll(void)
{
    uint8_t byte;
    while (uart_bsp_read_byte(&byte) != 0U) {
        if ((parser.state != 0U) || (byte == PROTOCOL_SOF1)) binary_byte_process(byte);
        else text_byte_process(byte);
    }
    if ((velocity_command_seen != 0U) &&
        ((uint32_t)(HAL_GetTick() - last_velocity_command_ms) > BOARD_COMMAND_TIMEOUT_MS)) {
        velocity_command_seen = 0U;
        fault_flags |= CAR_FAULT_CMD_TIMEOUT;
        app_motor_stop();
    }
    if (uart_bsp_rx_overflow_count_get() != 0U) fault_flags |= CAR_FAULT_RX_OVERFLOW;
}

uint8_t car_uart_halted_get(void)
{
    return ((control_mode != MODE_VELOCITY) || (velocity_command_seen == 0U)) ? 1U : 0U;
}

void car_uart_fault_set(uint16_t fault_mask, uint8_t active)
{
    if (active != 0U) fault_flags |= fault_mask;
    else fault_flags &= (uint16_t)~fault_mask;
}

uint16_t car_uart_fault_get(void) { return fault_flags; }

void car_uart_publish_telemetry(const imu_sample_t *imu, uint8_t imu_valid)
{
    uint8_t payload[32];
    int32_t x_mm;
    int32_t y_mm;
    int32_t yaw_mrad;
    const odometry_context_t *odom = app_odometry_state_get();

    app_odometry_pose_fixed_get(&x_mm, &y_mm, &yaw_mrad);
    write_u32_le(&payload[0], HAL_GetTick());
    write_i32_le(&payload[4], encoder_bsp_pulse_count_get(ENCODER_SIDE_LEFT));
    write_i32_le(&payload[8], encoder_bsp_pulse_count_get(ENCODER_SIDE_RIGHT));
    write_i16_le(&payload[12], float_to_i16(app_motor_speed_meas_filtered_m_s_get(ENCODER_SIDE_LEFT) * 1000.0f));
    write_i16_le(&payload[14], float_to_i16(app_motor_speed_meas_filtered_m_s_get(ENCODER_SIDE_RIGHT) * 1000.0f));
    write_i32_le(&payload[16], x_mm);
    write_i32_le(&payload[20], y_mm);
    write_i32_le(&payload[24], yaw_mrad);
    write_i16_le(&payload[28], float_to_i16(odom->angular_velocity_rad_s * 1000.0f));
    write_u16_le(&payload[30], fault_flags);
    frame_send(MSG_CHASSIS_STATE, 0U, payload, (uint16_t)sizeof(payload));

    if ((imu_valid != 0U) && (imu != NULL)) {
        uint8_t imu_payload[18];
        uint8_t axis;
        write_u32_le(&imu_payload[0], HAL_GetTick());
        for (axis = 0U; axis < 3U; axis++) {
            write_i16_le(&imu_payload[4U + 2U * axis],
                         float_to_i16((imu->accel_m_s2[axis] / 9.80665f) * 1000.0f));
            write_i16_le(&imu_payload[10U + 2U * axis],
                         float_to_i16(imu->gyro_rad_s[axis] * 1000.0f));
        }
        write_i16_le(&imu_payload[16], float_to_i16(imu->temperature_c * 100.0f));
        frame_send(MSG_IMU_STATE, 0U, imu_payload, (uint16_t)sizeof(imu_payload));
    }
}
