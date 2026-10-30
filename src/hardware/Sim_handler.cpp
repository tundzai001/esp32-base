#include "Prog_Config.h"
#include "hardware/Sim_handler.h"

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm        modem(debugger);
#else
TinyGsm        modem(SerialAT);
#endif

bool startSIM() {
    
    int retrys = 1;
 
    digitalWrite(LED_PIN, HIGH);
    pinMode(MODEM_DC_PIN, OUTPUT);
    digitalWrite(MODEM_DC_PIN, HIGH);
    delay(1000);
    digitalWrite(MODEM_DC_PIN, LOW);
    SerialMon.println("[GSM] Doi modem SIM/GSM khoi dong (khoang 8 giay)...");
    delay(8000);
    digitalWrite(LED_PIN, LOW);

    SerialMon.println("[GSM] Khoi tao modem... So lan thu: " + String(retrys));
    
    #ifndef NATIVE_BUILD
    modem.init();
    #endif

    String modemInfo = modem.getModemInfo();
    while (modemInfo.length() == 0) {
        if (retrys > 10) {
            SerialMon.println("[GSM] Lay thong tin modem that bai sau nhieu lan thu ! Khoi dong modem that bai !");
            return false;
        }
        SerialMon.println("[GSM] Lay thong tin modem that bai ! So lan thu: " 
            + String(retrys));
        delay(1000);
        #ifndef NATIVE_BUILD
        modem.restart();
        #endif
        modemInfo = modem.getModemInfo();
        retrys++;
    }
    SerialMon.print("[GSM] Khoi dong modem thanh cong.Thong tin modem: ");
    SerialMon.println(modemInfo);
    digitalWrite(LED_PIN, LOW);

    delay(3000);
    return true;
}
bool connectGSM(uint8_t maxAttempts) {
    uint8_t retries = 0;
    while (retries < maxAttempts) {
        retries++;
        String simOperator = modem.getProvider();
        SerialMon.println("[GSM] Nha mang GSM: " + simOperator);
        String apn = "";
        if (simOperator.indexOf("VIETTEL") >= 0) {
            apn = "v-internet";
        } else if (simOperator.indexOf("MOBIFONE") >= 0) {
            apn = "m-wap";
        } else if (simOperator.indexOf("VINAPHONE") >= 0) {
            apn = "m3-world";
        } else {
            SerialMon.println("[GSM] Nha mang khong xac dinh, su dung APN mac dinh: " + String(APN));
            apn = String(APN);
        }
        prefs.begin("myPrefs", true);
        String gprsUser = prefs.getString("GPRS_USER", String(GPRS_USER));
        String gprsPass = prefs.getString("GPRS_PASS", String(GPRS_PASS));
        prefs.end();

        SerialMon.println("[GSM] Dang ket noi mang GSM...");
        if (modem.isNetworkConnected()) {
            SerialMon.println("[GSM] Mang GSM da ket noi.");
            if (modem.gprsConnect(apn.c_str(), gprsUser.c_str(), gprsPass.c_str()) && modem.isGprsConnected()) {
                SerialMon.println("[GSM] GPRS da ket noi.");
                return true;
            } else {
                SerialMon.println("[GSM] GPRS chua ket noi.");
            }
        }
        SerialMon.println("[GSM] Chua ket noi duoc mang GSM ngay lap tuc. Dang doi...");
        if (modem.waitForNetwork(30000)) {
            delay(1000);
            SerialMon.println("[GSM] Mang GSM da ket noi.");
            if (modem.gprsConnect(apn.c_str(), gprsUser.c_str(), gprsPass.c_str()) && modem.isGprsConnected()) {
                SerialMon.println("[GSM] GPRS da ket noi.");
                return true;
            } else {
                SerialMon.println("[GSM] GPRS chua ket noi.");
            }
        }
        SerialMon.println("[GSM] Ket noi khong thanh cong, lan thu " + String(retries) + "...");
        delay(5000);
    }
    SerialMon.println("[GSM] Ket noi mang GSM that bai sau " + String(retries) + " lan thu. Ket thuc ket noi !");
    return false;
}
