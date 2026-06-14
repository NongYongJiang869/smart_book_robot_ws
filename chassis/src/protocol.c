/**
 * @file protocol.c
 * @brief STM32 ↔ RDK X5 二进制帧协议 — 帧构建 + 发送
 *
 * CRC-16-CCITT (poly=0x1021, init=0x0000)
 * 覆盖范围: 帧类型 + 帧序号 + 数据负载
 *
 * 用法: 调用 protocol_send_xxx() 函数, 帧自动通过 USART2 发出
 */

#include "protocol.h"
#include "bsp_usart.h"
#include "stm32f10x.h"
#include <string.h>

/* ── 发送缓冲 (最大帧: 头2+L1+type1+seq1+payload255+crc2 = 262B) ── */
#define TX_BUF_SIZE 264
static uint8_t tx_buf[TX_BUF_SIZE];
static uint8_t tx_seq;

/* ── 接收缓冲 & 状态机 ── */
#define RX_BUF_SIZE 264
static uint8_t rx_buf[RX_BUF_SIZE];
static int rx_pos;           /* 已接收的帧体字节数 */
static int rx_total;         /* 需要接收的总字节数 (帧体=1+L) */
static int rx_state;         /* 0=等0x5A, 1=等0xA5, 2=读长度, 3=收帧体 */

/* ================================================================
 * CRC16-CCITT
 * ================================================================ */

static uint16_t crc16_ccitt(const uint8_t *data, int len)
{
    uint16_t crc = 0x0000;
    int i, j;

    for (i = 0; i < len; i++)
    {
        crc ^= ((uint16_t)data[i] << 8);
        for (j = 0; j < 8; j++)
        {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }
    return crc;
}

/* ================================================================
 * USART2 字节发送
 * ================================================================ */

static void uart_send_byte(uint8_t byte)
{
    USART_SendData(USART2, byte);
    while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET) {}
}

static void uart_send_buf(const uint8_t *buf, int len)
{
    int i;
    for (i = 0; i < len; i++)
    {
        uart_send_byte(buf[i]);
    }
}

/* ================================================================
 * 通用帧构建
 *
 * buf 布局 (调用方已填充 payload):
 *   [0:1]   0x5A 0xA5  帧头
 *   [2]     length      = 4 + payload_len
 *   [3]     type       帧类型
 *   [4]     seq        帧序号
 *   [5:5+N] payload   数据负载
 *   [5+N:7+N] crc16   LE
 * ================================================================ */

static void protocol_send_frame(uint8_t type, const uint8_t *payload, int payload_len)
{
    int len_field = 4 + payload_len;  /* type(1)+seq(1)+payload(N)+crc(2) */
    int total = 2 + 1 + len_field;    /* 头(2) + 长度字节(1) + 内容 */

    if (total > TX_BUF_SIZE) return;

    /* 帧头 */
    tx_buf[0] = 0x5A;
    tx_buf[1] = 0xA5;
    tx_buf[2] = (uint8_t)len_field;
    tx_buf[3] = type;
    tx_buf[4] = tx_seq;

    /* 负载 */
    if (payload_len > 0)
        memcpy(&tx_buf[5], payload, payload_len);

    /* CRC16 覆盖 type + seq + payload */
    uint16_t crc = crc16_ccitt(&tx_buf[3], 2 + payload_len);

    int crc_pos = 5 + payload_len;
    tx_buf[crc_pos]     = (uint8_t)(crc & 0xFF);      /* LE 低字节在前 */
    tx_buf[crc_pos + 1] = (uint8_t)((crc >> 8) & 0xFF);

    /* 发送 */
    uart_send_buf(tx_buf, total);

    /* 序号递增 */
    tx_seq++;
}

/* ================================================================
 * 初始化
 * ================================================================ */

void protocol_init(void)
{
    tx_seq = 0;
    rx_state = 0;
    rx_pos = 0;
}

/* ================================================================
 * ODOM_DATA 帧 (0x01, 24B 负载)
 * ================================================================ */

void protocol_send_odom_data(int32_t left_enc, int32_t right_enc,
                             float left_speed, float right_speed,
                             int16_t gyro_z, int16_t accel_x, int16_t accel_y,
                             uint16_t timestamp_ms)
{
    uint8_t payload[24];

    /* 全部 Little-Endian (Cortex-M3 原生) */
    memcpy(&payload[0],  &left_enc,  4);
    memcpy(&payload[4],  &right_enc, 4);
    memcpy(&payload[8],  &left_speed,  4);
    memcpy(&payload[12], &right_speed, 4);
    memcpy(&payload[16], &gyro_z,    2);
    memcpy(&payload[18], &accel_x,   2);
    memcpy(&payload[20], &accel_y,   2);
    memcpy(&payload[22], &timestamp_ms, 2);

    protocol_send_frame(PROTO_TYPE_ODOM_DATA, payload, 24);
}

/* ================================================================
 * STATUS 帧 (0x02, 6B 负载)
 * ================================================================ */

void protocol_send_status(uint8_t motor_state, uint8_t sensor_state,
                          int16_t mcu_temp, uint16_t error_code)
{
    uint8_t payload[6];

    payload[0] = motor_state;
    payload[1] = sensor_state;
    memcpy(&payload[2], &mcu_temp, 2);
    memcpy(&payload[4], &error_code, 2);

    protocol_send_frame(PROTO_TYPE_STATUS, payload, 6);
}

/* ================================================================
 * ERROR 帧 (0x04, 2B 负载)
 * ================================================================ */

void protocol_send_error(uint16_t error_code)
{
    uint8_t payload[2];
    memcpy(payload, &error_code, 2);
    protocol_send_frame(PROTO_TYPE_ERROR, payload, 2);
}

/* ================================================================
 * HEARTBEAT 帧 (0x06, 无负载)
 * ================================================================ */

void protocol_send_heartbeat(void)
{
    protocol_send_frame(PROTO_TYPE_HEARTBEAT, NULL, 0);
}

/* ================================================================
 * 帧接收 — 状态机从环形缓冲区逐字节解析
 *
 * 帧体 = 长度字节 + type + seq + payload + crc = 1 + L 字节
 *
 * 状态:
 *   0 = 等待 0x5A
 *   1 = 收到 0x5A, 等待 0xA5
 *   2 = 收到帧头, 读取长度字节 L, 计算帧体大小 = 1+L
 *   3 = 接收帧体, 满 1+L 后校验 CRC
 * ================================================================ */

int protocol_try_parse(uint8_t *out_type, const uint8_t **out_payload, int *out_len)
{
    uint8_t byte;

    while (uart_rx_get(&byte))
    {
        switch (rx_state)
        {
        case 0:  /* 等待 0x5A */
            if (byte == 0x5A)
            {
                rx_buf[0] = byte;
                rx_state = 1;
            }
            break;

        case 1:  /* 等待 0xA5 */
            if (byte == 0xA5)
            {
                rx_buf[1] = byte;
                rx_state = 2;
            }
            else if (byte != 0x5A)
            {
                rx_state = 0;  /* 不是帧头, 回退 */
            }
            /* byte==0x5A: 保持状态1, 可能是 0x5A 0x5A 0xA5 */
            break;

        case 2:  /* 读长度字段 */
            rx_buf[2] = byte;
            rx_total = 1 + byte;  /* 帧体 = 长度字节(1) + L */
            if (rx_total > RX_BUF_SIZE - 3)
            {
                rx_state = 0;  /* 长度异常, 丢弃 */
                break;
            }
            rx_pos = 3;  /* 下一字节写入位置 */
            rx_state = 3;
            break;

        case 3:  /* 接收帧体 */
            rx_buf[rx_pos++] = byte;
            if (rx_pos >= 3 + rx_total)
            {
                /* 帧体完整, 校验 CRC */
                rx_state = 0;

                int L = rx_buf[2];
                uint8_t ftype = rx_buf[3];
                int plen = L - 4;  /* 减去 type+seq+crc */
                if (plen < 0) break;  /* 长度不够 */

                /* CRC 覆盖: type(1) + seq(1) + payload(plen) */
                uint16_t crc_calc = crc16_ccitt(&rx_buf[3], 2 + plen);
                uint16_t crc_recv = rx_buf[3 + 2 + plen]
                                  | ((uint16_t)rx_buf[3 + 2 + plen + 1] << 8);

                if (crc_calc != crc_recv) break;  /* CRC 错误, 丢弃 */

                *out_type = ftype;
                *out_payload = &rx_buf[5];  /* buf[0:1]=帧头, buf[2]=长度, buf[3]=type, buf[4]=seq, buf[5:]=负载 */
                *out_len = plen;
                return 1;
            }
            break;
        }
    }

    return 0;  /* 暂无完整帧 */
}

/* ================================================================
 * VEL_CMD 负载解析
 * ================================================================ */

int protocol_parse_vel_cmd(const uint8_t *payload, int len, VelCmd *cmd)
{
    if (len < 8) return 0;

    /* Little-Endian: float linear_x, float angular_z */
    memcpy(&cmd->linear_x,  &payload[0], 4);
    memcpy(&cmd->angular_z, &payload[4], 4);

    return 1;
}
