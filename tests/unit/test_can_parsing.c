#include "unity.h"
#include "can.h"
#include "mocks.h"

TEST_GROUP(CAN_Parsing);

void setUp(void)
{
    /* Reset global state */
    mock_tick_reset();
    memset(&g_in, 0, sizeof(g_in));
}

void tearDown(void)
{
    /* Cleanup */
}

/**
 * UT-001: Parse CAN ID 0x20 (ACK de precarga)
 */
TEST(CAN_Parsing, parse_precarga_ack_received)
{
    /* Arrange */
    can_msg_t frame = mock_can_precarga_ack(0x01);
    app_inputs_t st = {0};
    
    /* Act */
    CanRx_ParseAndUpdate(&frame, &st);
    
    /* Assert */
    TEST_ASSERT_EQUAL_INT(0x01, st.ok_precarga);
}

TEST(CAN_Parsing, parse_precarga_ack_not_received)
{
    /* Arrange */
    can_msg_t frame = mock_can_precarga_ack(0x00);
    app_inputs_t st = {0};
    
    /* Act */
    CanRx_ParseAndUpdate(&frame, &st);
    
    /* Assert */
    TEST_ASSERT_EQUAL_INT(0x00, st.ok_precarga);
}

/**
 * UT-002: Parse CAN ID 0x100 (Tensión DC del bus)
 * Formato: 400V = 0x0190 en little-endian
 */
TEST(CAN_Parsing, parse_dc_bus_voltage_400v)
{
    /* Arrange */
    can_msg_t frame = mock_can_dc_bus_voltage(0x0190);
    app_inputs_t st = {0};
    
    /* Act */
    CanRx_ParseAndUpdate(&frame, &st);
    
    /* Assert */
    TEST_ASSERT_EQUAL_INT(0x0190, st.inv_dc_bus_voltage);
}

TEST(CAN_Parsing, parse_dc_bus_voltage_500v)
{
    /* Arrange */
    can_msg_t frame = mock_can_dc_bus_voltage(0x01F4);  /* 500 */
    app_inputs_t st = {0};
    
    /* Act */
    CanRx_ParseAndUpdate(&frame, &st);
    
    /* Assert */
    TEST_ASSERT_EQUAL_INT(0x01F4, st.inv_dc_bus_voltage);
}

/**
 * UT-003: Parse CAN ID 0x101 (S1 aceleración)
 */
TEST(CAN_Parsing, parse_throttle_s1)
{
    /* Arrange */
    can_msg_t frame = mock_can_throttle_s1(2100);
    app_inputs_t st = {0};
    
    /* Act */
    CanRx_ParseAndUpdate(&frame, &st);
    
    /* Assert */
    TEST_ASSERT_EQUAL_INT(2100, st.s1_aceleracion);
}

/**
 * UT-004: Parse CAN ID 0x102 (S2 aceleración)
 */
TEST(CAN_Parsing, parse_throttle_s2)
{
    /* Arrange */
    can_msg_t frame = mock_can_throttle_s2(1960);
    app_inputs_t st = {0};
    
    /* Act */
    CanRx_ParseAndUpdate(&frame, &st);
    
    /* Assert */
    TEST_ASSERT_EQUAL_INT(1960, st.s2_aceleracion);
}

/**
 * UT-005: Parse CAN ID 0x103 (Freno)
 */
TEST(CAN_Parsing, parse_brake_sensor)
{
    /* Arrange */
    can_msg_t frame = mock_can_brake(2000);  /* No activado */
    app_inputs_t st = {0};
    
    /* Act */
    CanRx_ParseAndUpdate(&frame, &st);
    
    /* Assert */
    TEST_ASSERT_EQUAL_INT(2000, st.s_freno);
}

TEST(CAN_Parsing, parse_brake_sensor_engaged)
{
    /* Arrange */
    can_msg_t frame = mock_can_brake(4500);  /* Activado */
    app_inputs_t st = {0};
    
    /* Act */
    CanRx_ParseAndUpdate(&frame, &st);
    
    /* Assert */
    TEST_ASSERT_EQUAL_INT(4500, st.s_freno);
}

/**
 * UT-006: Parse CAN ID 0x12C (Tensión mínima celda)
 */
TEST(CAN_Parsing, parse_cell_min_voltage)
{
    /* Arrange */
    can_msg_t frame = mock_can_cell_min_voltage(150);  /* 1.5V */
    app_inputs_t st = {0};
    
    /* Act */
    CanRx_ParseAndUpdate(&frame, &st);
    
    /* Assert */
    TEST_ASSERT_EQUAL_INT(150, st.v_celda_min);
}

/**
 * UT-007: Parse CAN ID desconocido (debe ser no-op)
 */
TEST(CAN_Parsing, unknown_id_no_crash)
{
    /* Arrange */
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    can_msg_t frame = mock_can_frame(0xDEAD, data);
    app_inputs_t st = {
        .inv_dc_bus_voltage = 400
    };
    uint16_t old_vdc = st.inv_dc_bus_voltage;
    
    /* Act */
    CanRx_ParseAndUpdate(&frame, &st);
    
    /* Assert: sin cambios */
    TEST_ASSERT_EQUAL_INT(old_vdc, st.inv_dc_bus_voltage);
}

/**
 * UT-008: CAN Pack/Unpack roundtrip
 */
TEST(CAN_Packing, roundtrip_pack_unpack)
{
    /* Arrange */
    can_msg_t original = {
        .bus = CAN_BUS_INV,
        .id = 0x123,
        .dlc = 8,
        .ide = 0,
        .data = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08}
    };
    can_qitem16_t packed;
    
    /* Act: Pack */
    CAN_Pack16(&original, &packed);
    
    /* Act: Unpack */
    can_msg_t unpacked;
    CAN_Unpack16(&packed, &unpacked);
    
    /* Assert */
    TEST_ASSERT_EQUAL_INT(original.id, unpacked.id);
    TEST_ASSERT_EQUAL_INT(original.dlc, unpacked.dlc);
    TEST_ASSERT_EQUAL_INT(original.bus, unpacked.bus);
    TEST_ASSERT_EQUAL_INT(original.ide, unpacked.ide);
    TEST_ASSERT_EQUAL_MEMORY(original.data, unpacked.data, 8);
}

/**
 * UT-009: CAN Pack con datos parciales (dlc < 8)
 */
TEST(CAN_Packing, roundtrip_dlc_4)
{
    /* Arrange */
    can_msg_t original = {
        .bus = CAN_BUS_INV,
        .id = 0x200,
        .dlc = 4,
        .ide = 0,
        .data = {0xAA, 0xBB, 0xCC, 0xDD, 0x00, 0x00, 0x00, 0x00}
    };
    can_qitem16_t packed;
    
    /* Act: Pack */
    CAN_Pack16(&original, &packed);
    
    /* Act: Unpack */
    can_msg_t unpacked;
    CAN_Unpack16(&packed, &unpacked);
    
    /* Assert */
    TEST_ASSERT_EQUAL_INT(4, unpacked.dlc);
    TEST_ASSERT_EQUAL_INT(0xAA, unpacked.data[0]);
    TEST_ASSERT_EQUAL_INT(0xBB, unpacked.data[1]);
}

/**
 * UT-010: Multiple sequential parses (orden FIFO)
 */
TEST(CAN_Parsing, multiple_frames_sequential)
{
    /* Arrange */
    app_inputs_t st = {0};
    
    /* Act & Assert: procesar 3 frames en orden */
    can_msg_t f1 = mock_can_precarga_ack(0x01);
    CanRx_ParseAndUpdate(&f1, &st);
    TEST_ASSERT_EQUAL_INT(0x01, st.ok_precarga);
    
    can_msg_t f2 = mock_can_dc_bus_voltage(0x0190);
    CanRx_ParseAndUpdate(&f2, &st);
    TEST_ASSERT_EQUAL_INT(0x0190, st.inv_dc_bus_voltage);
    
    can_msg_t f3 = mock_can_throttle_s1(2100);
    CanRx_ParseAndUpdate(&f3, &st);
    TEST_ASSERT_EQUAL_INT(2100, st.s1_aceleracion);
    
    /* Todos los campos se mantienen */
    TEST_ASSERT_EQUAL_INT(0x01, st.ok_precarga);
    TEST_ASSERT_EQUAL_INT(0x0190, st.inv_dc_bus_voltage);
}

TEST_GROUP_RUNNER(CAN_Parsing)
{
    RUN_TEST_CASE(CAN_Parsing, parse_precarga_ack_received);
    RUN_TEST_CASE(CAN_Parsing, parse_precarga_ack_not_received);
    RUN_TEST_CASE(CAN_Parsing, parse_dc_bus_voltage_400v);
    RUN_TEST_CASE(CAN_Parsing, parse_dc_bus_voltage_500v);
    RUN_TEST_CASE(CAN_Parsing, parse_throttle_s1);
    RUN_TEST_CASE(CAN_Parsing, parse_throttle_s2);
    RUN_TEST_CASE(CAN_Parsing, parse_brake_sensor);
    RUN_TEST_CASE(CAN_Parsing, parse_brake_sensor_engaged);
    RUN_TEST_CASE(CAN_Parsing, parse_cell_min_voltage);
    RUN_TEST_CASE(CAN_Parsing, unknown_id_no_crash);
    RUN_TEST_CASE(CAN_Packing, roundtrip_pack_unpack);
    RUN_TEST_CASE(CAN_Packing, roundtrip_dlc_4);
    RUN_TEST_CASE(CAN_Parsing, multiple_frames_sequential);
}

TEST_GROUP(CAN_Packing);

void setUp_packing(void)
{
}

void tearDown_packing(void)
{
}

TEST_GROUP_RUNNER(CAN_Packing)
{
    RUN_TEST_CASE(CAN_Packing, roundtrip_pack_unpack);
    RUN_TEST_CASE(CAN_Packing, roundtrip_dlc_4);
}
