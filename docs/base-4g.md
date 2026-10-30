# Base 4G di động và CORS

ESP32 chạy firmware C++, nhận lệnh và gửi trạng thái theo giao thức của
`cors/geodetic/agent_universal.py`. Python agent không chạy trên ESP32.
Mặc định thiết bị dùng 4G; có thể chuyển sang Wi-Fi bằng lệnh cấu hình sẵn có.

## Chức năng

| Chọn trên CORS | Lệnh xuống ESP32 | Hoạt động |
| --- | --- | --- |
| Tọa độ chính xác (`FIXED_LLA`) | `EXECUTE_RAW_COMMANDS` + `original_config` | Gửi nguyên lệnh tọa độ do backend tạo, lưu cấu hình và loại chip. |
| Survey-in (`SURVEY_IN`) | `EXECUTE_RAW_COMMANDS` + `original_config` | Gửi lệnh khảo sát do backend tạo. Unicore dùng thời gian khảo sát; u-blox có thêm ngưỡng chính xác, đúng theo command builder của CORS. |
| Auto base (`AUTO_CORS`) | `TRIGGER_AUTO_BASE` | Chuyển rover, nhận cải chính CORS, đợi RTK_FIXED với GGA mới, lấy mẫu, đặt lại base rồi phát RTCM tới caster. |
| Khóa/mở trạm | `LOCK_DEVICE` / `UNLOCK_DEVICE` | Giữ kênh quản lý; ngừng output RTCM theo phạm vi khóa. |

Base di động ở đây là bộ base có thể mang đến điểm đặt mới. Khi phát cải chính,
anten vẫn phải đứng yên. Sau khi chuyển vị trí, chọn lại một trong ba chế độ;
không tiếp tục dùng tọa độ của điểm đặt cũ.

Loại GNSS được chọn từ CORS (`sensor_type`: `Ublox` hoặc `Unicorecomm`), lưu
trong NVS và dùng cho auto base. Cùng một firmware chứa cả hai bộ lệnh.
Boot chờ 2 giây rồi dò receiver bằng UBX-MON-VER hoặc VERSIONA trên các baud hỗ trợ;
`UNKNOWN` có thể xuất hiện trước khi nhận diện hoặc khi chưa có phản hồi. Nhánh u-blox yêu cầu receiver hỗ trợ
UBX-CFG-TMODE3, như ZED-F9P. Nhánh Unicore dùng bộ lệnh UM980/UM982 trong CORS.

Survey-In Unicore xuất theo cấu hình UI: `RTCM only` gửi `unlogall` để tắt
output cũ, rồi bật bộ RTCM MSM4/MSM7 đã chọn và lưu cấu hình. Firmware
không tự hỏi GGA/GSA/GSV/GST định kỳ.
Trước khi bắt đầu Survey-In, luồng Unicore đặt `config undulation 0.0` để
receiver báo trực tiếp độ cao ellipsoid WGS 84. UI vẫn tính h bằng độ cao
chip báo cộng undulation của từng BESTNAV, nên đọc được cả cấu hình cũ.
Firmware tự đọc `MODE` và `BESTNAVA` mỗi 30 giây, kể cả sau reboot, để cập nhật UI.
`BESTNAV: FIXEDPOS` xác nhận UM đang giữ tọa độ Base; đây không phải phần trăm
hay số giây lấy mẫu hợp lệ bên trong UM. Kết quả Survey-In gần nhất lưu NVS,
vẫn có thể đối chiếu sau khi chuyển Fixed LLA hoặc khởi động lại ESP32.
Thông tin vệ tinh trên dashboard dùng decoder RTCM; backend chỉ ưu tiên NMEA
khi thực sự có GSV mới, không ẩn dữ liệu RTCM theo cấu hình NMEA đã lưu cũ.

Truy vấn đọc UM được xử lý trong `Unicore_Readback.cpp`. Firmware gửi CRLF trước
`MODE`/`BESTNAVA` để kết thúc dòng đầu vào còn dang dở; thao tác này không đổi
mode hoặc chạy lại Survey-In. Log `[GNSS][READBACK]` ghi lệnh gửi, ACK và số byte
nhận để phân biệt timeout với lỗi phản hồi/checksum.
Các byte RTCM/NMEA đi qua trong lúc truy vấn vẫn được đưa qua bộ đọc UART và
giữ frame RTCM để luồng NTRIP gửi tiếp. NTRIP ghi nguyên dòng phản hồi lỗi caster
vào Serial và `ntrip_status.serverN.error`; tốc độ nhận từ chip và tốc độ gửi
lên caster là hai số riêng.

Khi BESTNAV báo `SOL_COMPUTED,FIXEDPOS`, UI hiển thị tọa độ Base đang giữ và cho
sao chép hoặc điền vào Fixed LLA để người dùng kiểm tra/hiệu chỉnh trước khi áp
dụng. Vĩ độ, kinh độ lấy từ chuỗi BESTNAV gốc để giữ số lẻ. Độ cao h ellipsoid
được tính từ độ cao báo trong BESTNAV cộng undulation trong **cùng bản tin**;
undulation bằng 0 thì giữ nguyên độ cao. Giá trị này phụ thuộc cấu hình undulation
của chip, không phải geoid địa phương đã hiệu chỉnh. Khi áp dụng Fixed LLA với
`altitude_reference=ELLIPSOID`, backend hiện có đặt `config undulation 0.0` trước
`mode base`. Sao chép/điền form không gửi lệnh cấu hình xuống chip.

`Backend_Agent.cpp` giữ khởi tạo, kết nối và vòng lặp điều phối. Xử lý lệnh nằm
trong `Backend_Agent_Commands.cpp`, payload trạng thái trong `Backend_Agent_Status.cpp`,
workflow Auto Base/đối chiếu trong `Backend_Agent_Workflows.cpp`, phép chiếu tọa độ
trong `Backend_Agent_Projection.cpp`. Các phần này dùng chung lớp `BackendAgent`.

## Kết nối phần cứng

- ESP32 TDM240x: RX GNSS GPIO18, TX GNSS GPIO19; UART GNSS 115200 baud.
- Unicore: mặc định `GNSS_UNICORE_PORT = 0`; output sau cấu hình BASE dùng ngay
  cổng receiver đang nhận lệnh từ ESP32, không giả định dây nối COM2. Giá trị 1–3
  buộc output sang COM1–3 nếu phần cứng cần cổng cụ thể. Nhánh bootstrap lần đầu/reset
  vẫn cấu hình COM2; phải đối chiếu dây nối trước khi reset mặc định.
- u-blox: nối UART1, đặt UART1 115200 baud trước khi dùng. Firmware chuyển
  cấu hình output USB/UART2 do backend tạo sang UART1 và tính lại checksum UBX.
- Modem: RX ESP32 GPIO16, TX ESP32 GPIO17; APN theo SIM/nhà mạng.

Firmware chỉ chạy bootstrap theo chip nhận diện trong lần đầu/reset mặc định;
không ghi lại cấu hình UM980 vô điều kiện ở mọi lần boot. Với Unicore, các lệnh log COM3 do backend
tạo được chuyển sang cổng nối ESP32. Tọa độ và thời gian khảo sát giữ nguyên.

Nếu GGA đã báo Base cố định mà UART vẫn không có RTCM, sau khi nạp firmware mới
có thể thử `ATG GNSS BASE 1006 CURRENT ON` qua Serial Monitor. Lệnh này bật RTCM1006
trên cổng nhận lệnh, không đổi tọa độ hoặc chạy lại khảo sát. Khi thấy frame RTCM
tăng, dùng `ATG GNSS BASE ALL CURRENT ON` để bật toàn bộ danh sách RTCM hỗ trợ.

## Đưa thiết bị lên CORS

1. Build và nạp firmware, mở Serial Monitor 115200 baud:

   ```sh
   pio run -e tdm240x_dev_general
   pio run -e tdm240x_dev_general -t upload
   pio device monitor -b 115200
   ```

2. Cấu hình APN và broker MQTT **cùng broker backend CORS đang dùng**. Thay
   các giá trị ví dụ bằng thông tin triển khai thực tế:

   ```text
   ATG ESP SET 4G APN <apn>
   ATG MQTT SET SERVER <mqtt-host>
   ATG MQTT SET PORT 1883
   ATG MQTT SET USER <mqtt-user>
   ATG MQTT SET PASS <mqtt-password>
   ATG ESP SET BACKEND HOST <backend-host>
   ATG ESP SET BACKEND PORT 8000
   ATG ESP SET CONNECTION 4G
   ```

   Lệnh cuối khởi động lại để áp dụng. Kênh WebSocket hiện dùng `ws`, không hỗ
   trợ `wss`; MQTT vẫn là kênh quản lý khi không dùng được WebSocket. Host/port
   phải trỏ tới endpoint thiết bị thực tế của CORS, không phải URL trang web.

3. Lấy serial `esp_...` từ log `[BACKEND] Device Serial`. Trên CORS, tìm thiết
   bị này, nhận/cấp thiết bị và triển khai license theo luồng quản lý hiện có.
   Khi chưa có license, thiết bị vẫn báo trạng thái nhưng không phát RTCM.

4. Chọn đúng chip và một trong ba chế độ. Với auto base, backend phải có tài
   khoản/mountpoint cải chính trong `AUTO_BASE_CORS_*`. Với fixed, độ cao là
   độ cao ellipsoid, không tự thay bằng độ cao thủy chuẩn.

5. Cấu hình dịch vụ caster/mountpoint đích từ CORS. Thiết bị hỗ trợ hai máy
   chủ NTRIP. Theo dõi trạng thái GNSS, tọa độ, RTK FIX, vệ tinh, tốc độ RTCM,
   kết nối từng máy chủ và `last_command_result` trên hệ thống hiện có.

Không cần tạo dashboard hoặc API riêng cho ESP32. Các topic sử dụng:

```text
pi/devices/<serial>/status
pi/devices/<serial>/raw_data
pi/devices/<serial>/command
pi/devices/<serial>/control_ack
```

WebSocket dùng `/ws/pi/<serial>?token=<license>` và các gói `status_update`,
`nmea_update` tương ứng backend hiện có. Firmware nhận cả token `dt_...` do
backend cấp và license số cũ. `DEPLOY_LICENSE` chỉ báo thành công sau khi token
hợp lệ và lưu NVS thành công; token sai không ghi đè token đang dùng.
Không ghi license vào log.

## Khóa trạm

- `LOCK_DEVICE`, payload `{}`: khóa toàn thiết bị, chặn lệnh ngoài khóa/mở và
  yêu cầu dừng workflow đang chạy.
- `LOCK_DEVICE`, payload `{"locked_servers":[2]}`: chỉ chặn máy chủ 2; máy
  chủ 1 và điều khiển GNSS tiếp tục hoạt động.
- `[1,2]`: khóa cả hai output, vẫn cho quản lý GNSS như Python agent.
- `[]` hoặc `UNLOCK_DEVICE`: bỏ khóa. Việc mở khóa không tự bật một máy chủ
  đã bị tắt trong cấu hình dịch vụ.
- Lưu khóa vào NVS, nạp lại sau reboot. Khóa output luôn ưu tiên hơn fail-open.
- `system_info.device_lock` báo `scoped`, `all`, `locked_servers`, để CORS nhận
  biết firmware hỗ trợ khóa theo từng máy chủ.

Các payload trên mô tả nội dung lệnh; gửi qua cơ chế command dispatcher của
CORS để giữ đúng envelope, command ID và quyền người vận hành.

## Kiểm tra

Chạy kiểm tra host, không cần gắn ESP32:

```sh
c++ -std=c++17 -fsanitize=address,undefined -g -Itest/host -Iinclude \
  test/host/base_checks.cpp src/functions/Unicore_Log.cpp src/functions/gnss_cmd_builder.cpp src/functions/ublox_commands.cpp src/functions/unicore_commands.cpp \
  -o /tmp/esp32-base-checks
/tmp/esp32-base-checks
python3 -B test/host/check_cors.py /path/to/cors /tmp/esp32-base-checks
```

Kiểm tra này chạy bộ tạo lệnh C++ thật, kiểm tra khóa, checksum, routing UART
  và đưa lệnh sinh từ `cors_dashboard/backend/app/command_builder.py` thật qua
bộ routing firmware cho fixed/survey của cả hai loại chip.

Kiểm tra thực địa còn cần: nhận đúng GNSS, survey hoàn tất, auto base đạt FIX,
rover nhận được cải chính, độ đúng tọa độ và khóa/mở từng máy chủ sau reboot.
Auto base hướng tới 60 mẫu trong 60 giây, chỉ lấy RTK_FIXED và cần ít nhất 15
mẫu hợp lệ. Không mặc định chấp nhận RTK_FLOAT hoặc luôn có đủ 60 mẫu.
Với Unicore qua backend và auto base, lệnh cấu hình chỉ thành công khi nhận phản hồi đúng tên lệnh,
`$command,<command>,response: OK`, trong 8 giây. Checksum được kiểm tra nếu có.
Phản hồi lỗi (kể cả `NO MATCHING FUNC`), ghi UART thiếu hoặc timeout sẽ dừng
batch và báo lỗi về backend. Task đọc GNSS nhường UART trong lúc chờ ACK.
Reset chỉ gửi một lần; sau ACK đợi thêm 2 giây trước lệnh tiếp theo.
ACK xác nhận receiver nhận lệnh; chưa thay thế readback cấu hình sau reboot.
Các lệnh ATG qua terminal và cấu hình khởi động cũ vẫn dùng cơ chế gửi UART.

Với u-blox, bước chuyển Rover trong auto base kiểm tra ACK và đọc lại
CFG-TMODE3; các batch cấu hình base còn lại vẫn chỉ xác nhận gửi UART.
ESP32 chuyển tiếp RTCM nguyên byte, không giải
mã datum RTCM 1021/1023/1025 như Python agent. Phần chuyển VN2000 hiện có của
ESP32 dùng tham số tích hợp/tỉnh; cần kiểm chứng hệ tọa độ thực tế trước khi
dùng kết quả auto base cho đo đạc.

## Tách giao thức GNSS

- `gnss_cmd_builder.h/.cpp`: kiểu dữ liệu chung và chọn bộ tạo lệnh theo chip.
- `ublox_commands.h/.cpp`: tạo/routing gói UBX nhị phân cho u-blox.
- `Ubx_Reply.h`: đọc khung UBX, kiểm tra checksum; auto base dùng cho ACK/NAK và readback.
- `unicore_commands.h/.cpp`: tạo lệnh ASCII cho UM980/UM982.
- Khởi động UM980 bật thêm ephemeris `1019/1020/1042/1044/1045` trên COM2,
  chu kỳ 1 giây, trước `saveconfig`. Cấu hình fixed/survey tiếp tục chọn
  ephemeris theo hệ vệ tinh được bật.
- `Unicore_Reply.h`: đọc phản hồi ASCII của UM, khớp tên lệnh và kiểm tra `OK` chính xác.

Luồng Unicore từ chối dữ liệu nhị phân và nhiều dòng lệnh trong cùng một gói,
để mỗi lệnh gửi đi có một phản hồi được kiểm tra riêng. Các truy vấn trạng thái
`mode`/`log` vẫn đi theo luồng truy vấn, không yêu cầu ACK cấu hình.
