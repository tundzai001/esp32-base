#include "Top_Lvl_Config.h"
#include <unity.h>
#include <Arduino.h>
#include "Prog_Config.h"
#include "hardware/Sim_handler.h"

TinyGsmClient testClient(modem);

void setUp(void)
{
}

void tearDown(void)
{
}

void test_startSIM()
{
    Serial.println("Testing startSIM...");
    auto result = startSIM();
    TEST_ASSERT_TRUE(result);
}

void test_TCP_connection()
{
    Serial.println("Testing TCP connection...");
    // Require a reachable TCP service at this test-network address and caster port.
    auto result = testClient.connect("192.168.102.111", NTRIP_CASTER_PORT);
    TEST_ASSERT_TRUE(result);
}

void setup()
{
    UNITY_BEGIN();
    Serial.begin(115200);
    SerialAT.begin(115200, SERIAL_8N1, RX_TO_MODEM_TX, TX_TO_MODEM_RX);
    delay(2000);

    // Configure the board LED as an output before running the modem tests.
    pinMode(LED_PIN, OUTPUT);
    delay(2000);

    // Run the test cases
    RUN_TEST(test_startSIM);
    RUN_TEST(test_TCP_connection);
}

void loop()
{
    // Nothing to do here, as the tests are run in `setup()`
}
