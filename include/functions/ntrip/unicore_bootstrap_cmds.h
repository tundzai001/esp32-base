#ifndef UNICORE_BOOTSTRAP_CMDS_H
#define UNICORE_BOOTSTRAP_CMDS_H

inline constexpr const char nmeaCmdUnlog[] = "UNLOG\r\n";
inline constexpr const char nmeaCmdSetBase[] = "MODE BASE TIME 120 2.5\r\n";
inline constexpr const char rtcm1084SetOutputCom2[] = "RTCM1084 COM2 1\r\n";
inline constexpr const char rtcm1074SetOutputCom2[] = "RTCM1074 COM2 1\r\n";
inline constexpr const char rtcm1006SetOutputCom2[] = "RTCM1006 COM2 1\r\n";
inline constexpr const char rtcm1124SetOutputCom2[] = "RTCM1124 COM2 1\r\n";
inline constexpr const char rtcm1087SetOutputCom2[] = "RTCM1087 COM2 1\r\n";
inline constexpr const char rtcm1077SetOutputCom2[] = "RTCM1077 COM2 1\r\n";
inline constexpr const char rtcm1127SetOutputCom2[] = "RTCM1127 COM2 1\r\n";
inline constexpr const char rtcmSetBaudCom2[] = "CONFIG COM2 115200\r\n";
inline constexpr const char cmdSaveConfig[] = "saveconfig\r\n";

#endif
