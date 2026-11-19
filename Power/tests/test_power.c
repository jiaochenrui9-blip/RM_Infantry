#include "Power.h"
#include "i2c_bus.h"
#include "robot.h"
#include "can.h"
#include "can_bus.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

I2C_HandleTypeDef hi2c1;
static uint32_t registers[64];
static uint32_t now_tick;
static unsigned calls;
static unsigned writes;
static int fail_read = -1;
static int fail_write = -1;
static int corrupt_read = -1;
static uint8_t read_order[64];
static unsigned read_count;

CAN_HandleTypeDef hcan = {CAN1};
static CAN_RxHeaderTypeDef rx_header;
static uint8_t rx_data[8], tx_data[8];
static unsigned rx_pending, tx_count;
static HAL_StatusTypeDef start_result = HAL_OK, send_result = HAL_OK;

HAL_StatusTypeDef CAN_Bus_ConfigStdIdMask(CAN_HandleTypeDef *can,
    uint32_t bank, uint32_t slave_bank, uint16_t id, uint16_t mask)
{
    assert(can == &hcan && bank == 0U && slave_bank == 14U);
    assert(id == 0x200U && mask == 0x7F0U);
    return HAL_OK;
}

HAL_StatusTypeDef CAN_Bus_StartRxFifo0(CAN_HandleTypeDef *can)
{
    assert(can == &hcan);
    return start_result;
}

HAL_StatusTypeDef CAN_Bus_SendStdData(CAN_HandleTypeDef *can, uint16_t id,
    const uint8_t *data, uint8_t length)
{
    assert(can == &hcan && id == 0x200U && length == 8U);
    for (unsigned i = 2; i < 8; ++i) assert(data[i] == 0U);
    if (send_result != HAL_OK) return send_result;
    memcpy(tx_data, data, 8);
    ++tx_count;
    return HAL_OK;
}

uint32_t HAL_CAN_GetRxFifoFillLevel(CAN_HandleTypeDef *can, uint32_t fifo)
{
    assert(can == &hcan && fifo == CAN_RX_FIFO0);
    return rx_pending;
}

HAL_StatusTypeDef CAN_Bus_ReadFifo0(CAN_HandleTypeDef *can,
    CAN_RxHeaderTypeDef *header, uint8_t data[8])
{
    assert(can == &hcan && rx_pending == 1U);
    *header = rx_header;
    memcpy(data, rx_data, 8);
    rx_pending = 0;
    return HAL_OK;
}

static void ReceiveSpeed(uint16_t id, int16_t rpm)
{
    rx_header = (CAN_RxHeaderTypeDef){.StdId=id, .DLC=8U};
    memset(rx_data, 0, sizeof(rx_data));
    rx_data[2] = (uint8_t)((uint16_t)rpm >> 8);
    rx_data[3] = (uint8_t)rpm;
    rx_pending = 1U;
    HAL_CAN_RxFifo0MsgPendingCallback(&hcan);
}

static int16_t SentCurrent(void)
{
    return (int16_t)(((uint16_t)tx_data[0] << 8) | tx_data[1]);
}

uint32_t HAL_GetTick(void) { return now_tick; }

static unsigned RegisterLength(uint16_t reg)
{
    return reg == 4U || reg == 5U || reg == 7U || reg == 8U ? 3U : 2U;
}

HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *handle, uint16_t address,
    uint16_t reg, uint16_t reg_size, uint8_t *data, uint16_t length, uint32_t timeout)
{
    assert(handle == &hi2c1 && address == 0x80U);
    assert(reg_size == I2C_MEMADD_SIZE_8BIT && reg < 64U);
    assert(length == RegisterLength(reg) && timeout == 5U);
    assert(read_count < sizeof(read_order));
    ++calls;
    read_order[read_count++] = (uint8_t)reg;
    if ((int)reg == fail_read) {
        data[0] = 0xFFU;
        return HAL_TIMEOUT;
    }
    uint32_t value = registers[reg];
    if ((int)reg == corrupt_read) {
        value ^= 1U;
    }
    for (unsigned i = 0; i < length; ++i) {
        data[i] = (uint8_t)(value >> (8U * (length - i - 1U)));
    }
    if (reg == 0x0BU) {
        registers[reg] &= ~2U;
    }
    return HAL_OK;
}

HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *handle, uint16_t address,
    uint16_t reg, uint16_t reg_size, uint8_t *data, uint16_t length, uint32_t timeout)
{
    assert(handle == &hi2c1 && address == 0x80U);
    assert(reg_size == I2C_MEMADD_SIZE_8BIT && reg < 64U);
    assert(length == 2U && timeout == 5U);
    ++calls;
    ++writes;
    if ((int)reg == fail_write) {
        return HAL_ERROR;
    }
    registers[reg] = ((uint16_t)data[0] << 8) | data[1];
    return HAL_OK;
}

static void ResetModel(void)
{
    memset(registers, 0, sizeof(registers));
    registers[0x3E] = 0x5449U;
    registers[0x3F] = 0x2281U;
    calls = writes = read_count = 0U;
    fail_read = fail_write = corrupt_read = -1;
}

static uint32_t EncodeSigned20(int32_t value)
{
    return ((uint32_t)value & 0xFFFFFU) << 4;
}

static void SetSample(int32_t current_counts)
{
    registers[0x0B] = 3U;
    registers[4] = EncodeSigned20(current_counts);
    registers[5] = 122880U << 4; /* 24 V。 */
    registers[7] = EncodeSigned20(current_counts);
    registers[8] = 240000U;     /* 240 W，功率不应右移四位。 */
    read_count = 0U;
}

static void CheckNear(float actual, float expected, float tolerance)
{
    assert(fabsf(actual - expected) <= tolerance);
}

static void CheckMeasurementsEqual(const PowerFeedback_t *a, const PowerFeedback_t *b)
{
    assert(a->voltage_v == b->voltage_v && a->current_a == b->current_a);
    assert(a->shunt_voltage_v == b->shunt_voltage_v && a->power_w == b->power_w);
    assert(a->sample_tick == b->sample_tick);
}

int main(void)
{
    PowerFeedback_t feedback, previous;
    uint16_t value;
    uint8_t byte = 0;

    ResetModel();
    assert(I2C_Bus_WriteRegU16(&hi2c1, 0x40U, 0U, 0x1234U) == HAL_OK);
    assert(registers[0] == 0x1234U);
    assert(I2C_Bus_ReadRegU16(&hi2c1, 0x40U, 0U, &value) == HAL_OK);
    assert(value == 0x1234U);
    unsigned before = calls;
    assert(I2C_Bus_ReadReg(NULL, 0x40U, 0U, &byte, 1U) == HAL_ERROR);
    assert(I2C_Bus_ReadReg(&hi2c1, 0x80U, 0U, &byte, 1U) == HAL_ERROR);
    assert(I2C_Bus_ReadReg(&hi2c1, 0x40U, 0U, NULL, 1U) == HAL_ERROR);
    assert(I2C_Bus_WriteReg(&hi2c1, 0x40U, 0U, &byte, 0U) == HAL_ERROR);
    assert(I2C_Bus_ReadRegU16(&hi2c1, 0x40U, 0U, NULL) == HAL_ERROR);
    assert(calls == before);
    fail_read = 0;
    value = 0xABCDU;
    assert(I2C_Bus_ReadRegU16(&hi2c1, 0x40U, 0U, &value) == HAL_TIMEOUT);
    assert(value == 0xABCDU);
    puts("PASS: BSP addressing, byte order, invalid arguments, failure propagation");

    ResetModel();
    assert(Power_Init(&hi2c1) == HAL_OK);
    assert(registers[0] == 0U && registers[1] == 0xB6D9U && registers[2] == 4096U);
    assert(writes == 4U);
    Power_GetFeedback(&feedback);
    assert(feedback.manufacturer_id == 0x5449U && feedback.device_id == 0x2281U);
    assert(feedback.status == HAL_BUSY);
    puts("PASS: ID validation, 1 milliohm calibration, initialization readback");

    SetSample(32000);
    assert(Power_Update(10U) == HAL_OK);
    Power_GetFeedback(&feedback);
    CheckNear(feedback.voltage_v, 24.0f, 0.0001f);
    CheckNear(feedback.shunt_voltage_v, 0.01f, 0.000001f);
    CheckNear(feedback.current_a, 10.0f, 0.0001f);
    CheckNear(feedback.power_w, 240.0f, 0.001f);
    assert(feedback.sample_tick == 10U && feedback.status == HAL_OK);
    const uint8_t expected_order[] = {0x0B, 0x04, 0x05, 0x07, 0x08};
    assert(read_count == sizeof(expected_order));
    assert(memcmp(read_order, expected_order, sizeof(expected_order)) == 0);
    previous = feedback;
    assert(Power_Update(11U) == HAL_BUSY);
    Power_GetFeedback(&feedback);
    CheckMeasurementsEqual(&previous, &feedback);
    puts("PASS: separate register reads, physical units, conversion-ready handling");

    const int32_t signed_cases[] = {0, 1, -1, -32000, 524287, -524288};
    for (unsigned i = 0; i < sizeof(signed_cases) / sizeof(signed_cases[0]); ++i) {
        SetSample(signed_cases[i]);
        assert(Power_Update(20U + i) == HAL_OK);
        Power_GetFeedback(&feedback);
        CheckNear(feedback.current_a, (float)signed_cases[i] * 0.0003125f, 0.0001f);
        CheckNear(feedback.shunt_voltage_v, (float)signed_cases[i] * 0.0000003125f, 0.000001f);
    }
    puts("PASS: zero, positive/negative current, signed 20-bit boundaries");

    previous = feedback;
    for (unsigned i = 0; i < sizeof(expected_order); ++i) {
        SetSample(32000);
        fail_read = expected_order[i];
        assert(Power_Update(50U) == HAL_TIMEOUT);
        Power_GetFeedback(&feedback);
        CheckMeasurementsEqual(&previous, &feedback);
        assert(feedback.status == HAL_TIMEOUT);
    }
    fail_read = -1;
    SetSample(32000);
    assert(Power_Update(60U) == HAL_OK);
    puts("PASS: each read failure preserves the last sample, next cycle recovers");

    ResetModel();
    registers[0x3E] = 0U;
    assert(Power_Init(&hi2c1) == HAL_ERROR && writes == 0U);
    before = calls;
    assert(Power_Update(70U) == HAL_ERROR && calls == before);
    ResetModel();
    registers[0x3F] = 0x2291U;
    assert(Power_Init(&hi2c1) == HAL_ERROR && writes == 0U);
    ResetModel();
    registers[0x3F] = 0x228FU;
    assert(Power_Init(&hi2c1) == HAL_OK);
    ResetModel();
    fail_write = 2;
    assert(Power_Init(&hi2c1) == HAL_ERROR);
    ResetModel();
    corrupt_read = 2;
    assert(Power_Init(&hi2c1) == HAL_ERROR);
    assert(Power_Init(NULL) == HAL_ERROR);
    Power_GetFeedback(NULL);
    puts("PASS: wrong ID, revision mask, write/readback errors, null handling");

    ResetModel();
    now_tick = UINT32_MAX - 5U;
    Robot_Init();
    before = calls;
    now_tick = 3U;
    Robot_Loop();
    assert(calls == before);
    SetSample(32000);
    now_tick = 4U;
    Robot_Loop();
    Power_GetFeedback(&feedback);
    assert(feedback.status == HAL_OK && feedback.sample_tick == 4U);
    before = calls;
    Robot_Loop();
    assert(calls == before);
    puts("PASS: 10 ms scheduling and tick wraparound");
    /* 无反馈不驱动；只接收 ID 1，速度低于/高于目标时输出方向相反。 */
    before = calls;
    Robot_ControlTick();
    assert(SentCurrent() == 0 && calls == before);
    ReceiveSpeed(0x202U, 0);
    Robot_ControlTick();
    assert(SentCurrent() == 0);
    ReceiveSpeed(0x201U, 0);
    Robot_ControlTick();
    assert(SentCurrent() == 3000);
    ReceiveSpeed(0x201U, 1000);
    Robot_ControlTick();
    assert(abs(SentCurrent()) <= 5);
    ReceiveSpeed(0x201U, 2000);
    Robot_ControlTick();
    assert(SentCurrent() < 0 && SentCurrent() >= -3000);
    now_tick += 101U;
    Robot_ControlTick();
    assert(SentCurrent() == 0);
    ReceiveSpeed(0x201U, 0);
    Robot_ControlTick();
    assert(SentCurrent() == 3000);
    assert(calls == before);
    puts("PASS: ID1 routing, 1000 RPM loop, current limit, offline stop/recovery");

    /* 忙邮箱不等待，下一拍使用最新反馈；启动失败时不发送控制帧。 */
    send_result = HAL_BUSY;
    before = tx_count;
    Robot_ControlTick();
    assert(tx_count == before);
    ReceiveSpeed(0x201U, 2000);
    send_result = HAL_OK;
    Robot_ControlTick();
    assert(tx_count == before + 1 && SentCurrent() < 0);
    ResetModel();
    start_result = HAL_ERROR;
    Robot_Init();
    before = tx_count;
    Robot_ControlTick();
    assert(tx_count == before);
    puts("PASS: busy mailbox retry and CAN initialization failure");
    puts("All 9 test groups passed.");
    return 0;
}
