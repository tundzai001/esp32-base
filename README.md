# ESP32 GNSS Gateway (UM980 / UM982 / u-blox)

Chế độ **base 4G di động** kết nối CORS: nhập tọa độ chính xác (`FIXED_LLA`), survey-in (`SURVEY_IN`), auto base (`AUTO_CORS`) và khóa trạm.

* 📖 **Tài liệu hướng dẫn**: Xem [hướng dẫn kết nối và kiểm tra](docs/base-4g.md).
* 📝 **Nhật ký thay đổi**: Xem [chi tiết các bản sửa đổi firmware ESP32](CHANGELOG.md).

Loại GNSS được cấu hình từ CORS; cùng một firmware hỗ trợ cả module Unicore (UM980, UM982) và u-blox (ZED-F9P).

## Tính năng chính
- **Chế độ Base 4G di động & Rover**: Hỗ trợ 3 chế độ thiết lập Base: nhập tọa độ cố định (`FIXED_LLA`), khảo sát tự do (`SURVEY_IN`), và tự động đo tọa độ chuẩn qua CORS tham chiếu (`AUTO_CORS`).
- **Chuyển đổi mượt mà & Chống xung đột**: Tự động ngắt task Auto Base ngầm và nhả UART khi người dùng chuyển sang nạp tọa độ chính xác (`EXECUTE_RAW_COMMANDS`).
- **Giao tiếp GNSS đa nền tảng**: Nhận diện và tự động định tuyến lệnh log COM (Unicore) hoặc chuyển sang UART1 kèm tính lại checksum Fletcher (u-blox).
- **Giám sát tài nguyên phần cứng (Telemetry)**: Báo cáo dung lượng RAM Heap (Bytes, KB, MB, %) và bộ nhớ Flash Disk lên CORS Dashboard thời gian thực.
- **NTRIP Client & Gửi GGA định kỳ**: Tự động gửi câu GGA tức thì khi kết nối và nhắc lại định kỳ 5 giây/lần lên Caster để duy trì kết nối VRS/Auto Base.
- **Khóa trạm linh hoạt (Scoped Station Lock)**: Hỗ trợ khóa riêng lẻ từng server xuất RTCM (server 1 hoặc server 2) hoặc khóa toàn bộ thiết bị, lưu trạng thái an toàn trong Flash NVS.
- **MQTT & WebSocket Song song**: Hỗ trợ giao tiếp 2 chiều với CORS Dashboard qua cả MQTT và WebSocket. Cổng WebSocket cấu hình linh hoạt qua NVS.

## Yêu cầu môi trường và thiết bị
- Môi trường phát triển: PlatformIO IDE hoặc bất kỳ IDE nào hỗ trợ PlatformIO.
- Bảng mạch được hỗ trợ: esp32dev (TDM240x series) và các board ESP32 tương thích với cấu hình hiện có trong `platformio.ini`.
- Module GNSS: Unicore UM980 hoặc UM982 (kết nối qua UART).
- Các thư viện được sử dụng:
    - `Arduino` cho lập trình cơ bản trên ESP32.
    - `PubSubClient` cho MQTT.
    - `WiFi` cho kết nối mạng WiFi.
    - `TinyGSM` cho kết nối mạng 4G (nếu sử dụng modem 4G).
    - `ArduinoJson` để xử lý JSON.

## Tổ chức mã nguồn:
### Tổ chức nhánh:
- `main`: Nhánh chính chứa mã nguồn ổn định, đã được kiểm tra kỹ lưỡng. Chỉ được nhận merge từ nhánh `develop` dưới dạng squash commit để giữ lịch sử sạch sẽ. Không được phép merge trực tiếp vào `main` từ các nhánh tính năng hoặc test.
- `develop`: Nhánh phát triển, cũng là nhánh mặc định trên GitHub, nơi các tính năng mới được thêm vào và thử nghiệm. Sau khi hoàn thiện và kiểm tra, các thay đổi sẽ được merge vào nhánh `main`. Squash commit từ `main` phải được merge ngược lại vào `develop` ngay sau khi merge để đồng bộ lịch sử.
- `feature/<tên-tính-năng>`: Các nhánh tính năng riêng biệt để phát triển các tính năng mới hoặc sửa lỗi cụ thể. Sau khi hoàn thành, sẽ được merge vào `develop`.
- `feature-test/<tên-tính-năng>`: Các nhánh riêng để viết và chạy unit test cho các tính năng tương ứng với các nhánh `feature/<tên-tính-năng>`. Sau khi hoàn thành, sẽ được merge vào `feature/<tên-tính-năng>`. Khi nhánh `feature/<tên-tính-năng>` bị xoá, nhánh `feature-test/<tên-tính-năng>` sẽ được xoá.
- `documentation`: Các nhánh riêng để viết tài liệu hướng dẫn sử dụng, cấu hình, v.v. Sau khi hoàn thành phần tài liệu, sẽ được merge vào `develop`. Nhánh này chỉ được phép merge vào `develop` và cũng chỉ được nhận merge từ `develop` để đồng bộ.

### Tổ chức thư mục:
```
├── .pio/           # Thư mục do PlatformIO tạo ra chứa file biên dịch,
|               thư viện đã cài, v.v. Không chỉnh sửa trực tiếp.
|               thư mục này không xuất hiện trong repo vì đã được thêm vào .gitignore
|
├── boards/
|   └── heltec_wifi_lora_32_v4.json  # Cấu hình PlatformIO để biên dịch
|                                     mã nguồn cho Heltec V4
|                                     (đây là file tuỳ chỉnh do PlatformIO chưa
|                                     duyệt cấu hình Heltec V4 chính thức)
|
├──lib/                             # Thư viện riêng của dự án (hiện chưa cần thiết).
|                                  Sẽ được biên dịch thành thư viện liên kết tĩnh
|
├──include/                         # Header files chứa cấu hình, định nghĩa hằng số,
|   |                                 khai báo biến, đối tượng và prototype của hàm.
|   |                                 Ngoại trừ main không có header và một số header
|   |                                 được khai báo trong đoạn markdown này,
|   |                                 các file header còn lại được đặt tên và tổ chức hoàn toàn
|   |                                 giống với tên và vị trí file .cpp tương ứng trong src/
|   |
|   ├── Top_Lvl_Config.h            # Cấu hình cấp cao trước khi biên dịch
|   └── Prog_Config.h               # Cấu hình hoạt động của chương trình
|
├──src/                             # Mã nguồn chính của chương trình.
|    ├── main.cpp                       # Điểm vào chính của chương trình, chứa hàm setup() và loop()
|    ├── helper.cpp                     # Các hàm phụ trợ để xử lý dữ liệu, kết nối, v.v.
|    ├── functions/                      # Thư mục con chứa các hàm được tổ chức theo chức năng (MQTT, NTRIP, NMEA parsing)
|    |  ├── MQTT_Manager.cpp                     # Hàm xử lý kết nối và gửi dữ liệu qua MQTT
|    |  ├── NTRIP_Handler_IP.cpp                 # Hàm xử lý kết nối và nhận dữ liệu từ NTRIP Caster qua IP
|    |  └── NMEA_Parser.cpp                      # Hàm xử lý phân tích chuỗi NMEA
|    └──hardware/                       # Thư mục con chứa các hàm liên quan đến phần cứng (wifi, 4g)
|       ├── WiFi_handler.cpp                     # Hàm xử lý kết nối WiFi
|       └── Sim_handler.cpp                      # Hàm xử lý kết nối 4G
|
├──test/                            # Thư mục dành cho việc viết unit test
|                                  và sử dụng PlatformIO Test Runner.
|
├──variants/                        # Cấu hình biến thể phần cứng (ví dụ: Heltec V4, TDM2402)
|   |                                được định nghĩa dưới dạng macro trong file header (.h)
|   |
|   └── heltec_V4
|           |
|           └── pins_arduino.h     # Định nghĩa chân GPIO cho board Heltec V4
|
└──platformio.ini                    # File cấu hình chính của PlatformIO, xác định board,
```

## Cấu hình chương trình và hệ thống:
### Cấu hình cấp cao trước khi biên dịch, được lưu trong `include/Top_Lvl_Config.h`:
Các cấu hình sau có thể được sửa trong file `Top_Lvl_Config.h` hoặc đưa vào dưới dạng tham số biên dịch trong `platformio.ini` (`-D<MACRO>[=<VALUE>]`).
- `WIFI_LORA_32_V4`: Định nghĩa loại cấu hình phần cứng (ví dụ Heltec V4).
- `RTCM_COMMUNICATION_PROTOCOL`: Chồng giao thức truyền dữ liệu cải chính NTRIP, hiện chỉ hỗ trợ qua TCP/IP stack.

Phương thức kết nối mạng được chọn lúc khởi động từ Preferences `CONNECTION_TYPE`: `WIFI` hoặc `4G` (mặc định `4G`). Lệnh MQTT cấu hình `CONNECTION` sẽ khởi động lại thiết bị để áp dụng giá trị mới.

### Cấu hình hoạt động MẶC ĐỊNH:
Các cấu hình sau được khai báo dưới dạng hằng số inline trong `include/Prog_Config.h` và có thể được sửa trực tiếp trong file này:

- Thiết lập chân UART kết nối với module GNSS (`RX_GNSS`, `TX_GNSS`)
- Thiết lập chân kết nối với module 4G (nếu sử dụng) (`TX_TO_MODEM_RX`, `RX_TO_MODEM_TX`, `MODEM_DC_PIN`, `MODEM_DTR_PIN`)
- Thông tin mạng WiFi (nếu sử dụng) (`SSID`, `Password`)
- Thông tin mạng 4G (`APN`, `User`, `Pass`) nếu sử dụng modem 4G
- Cấu hình kết nối MQTT (`Server`, `Port`, `User`, `Pass`, `Topics`)
- Cấu hình tài khoản NTRIP (`NTRIP_MODE`, `NTRIP_CASTER_IP`, `NTRIP_CASTER_PORT`, `NTRIP_MOUNTPOINT`, `NTRIP_AUTH`(base64))
- Cấu hình kiểm tra sức khoẻ định kỳ (`HEALTH_INTERVAL`)

Ở trên là các cấu hình mặc định, nếu không thể đọc cấu hình từ bộ nhớ flash được triển khai bằng Preferences (NVS), chương trình sẽ sử dụng các giá trị mặc định này. Các cấu hình này có thể được thay đổi bằng cách gửi lệnh qua MQTT (xem phần dưới).

## Quá trình hoạt động

Tùy theo điều kiện mạng, ESP32 sẽ mất khoảng 15 đến 20 giây để khởi động, kết nối mạng 4G, đăng nhập MQTT broker và NTRIP caster. Khi kết nối và đăng nhập thành công, ESP32 sẽ lấy dữ liệu RTCM từ module GNSS và gửi lên NTRIP caster, đồng thời gửi một bản sao của bản tin này lên một topic MQTT.

Để thông báo trạng thái sức khỏe thiết bị, mỗi 30 giây ESP32 sẽ gửi một bản tin health check lên topic MQTT đã cấu hình. Bản tin này có định dạng JSON như sau:

```json
{
    "uptime_s": 123456,
    "free_heap_bytes": 123456,
    "connected_via": "GSM",
    "rssi_dbm": "-77",
    "mqtt_ok": true,
    "ntrip_ok": true,
    "gnss_data_ok": true,
    "rtcm_types": {
        "1005": 12,
        "1074": 8,
        "1077": 10,
        "1084": 0,
        "1087": 0,
        "1094": 0,
        "1097": 0,
        "1124": 0,
        "1127": 0,
        "1230": 0
    }
}
```

## Định dạng bản tin Health Check:

- `uptime_s`: thời gian hoạt động của ESP32, tính bằng giây. Sẽ reset về 0 khi ESP32 khởi động lại.
- `free_heap_bytes`: dung lượng RAM còn trống, tính bằng byte.
- `connected_via`: phương thức kết nối mạng hiện tại, có thể là `GSM` hoặc `WIFI`.
- `rssi_dbm`: cường độ tín hiệu mạng hiện tại, tính bằng dBm.
- `mqtt_ok`: trạng thái kết nối MQTT, `true` nếu đang kết nối, `false` nếu không.
- `ntrip_ok`: trạng thái kết nối NTRIP, `true` nếu đang kết nối, `false` nếu không.
- `gnss_data_ok`: trạng thái dữ liệu GNSS, `true` nếu đang nhận dữ liệu RTCM từ module GNSS, `false` nếu không.
- `rtcm_types`: object chứa số lần xuất hiện của từng loại bản tin RTCM trong khoảng 30 giây gần nhất, tương ứng với một chu kỳ health check. Firmware hiện theo dõi các loại `1005`, `1074`, `1077`, `1084`, `1087`, `1094`, `1097`, `1124`, `1127` và `1230`. Giá trị bằng `0` nếu loại bản tin không xuất hiện trong chu kỳ hiện tại.

## Gửi lệnh qua MQTT hoặc Serial Monitor:

Thiết bị nhận nội dung (payload) từ Serial (nếu kết nối serial với máy tính hoặc điện thoại) hoặc qua topic lệnh MQTT đã cấu hình. Mỗi lệnh phải bắt đầu bằng `ATG`; các thành phần được ngăn cách bằng khoảng trắng. Từ khóa phân biệt chữ hoa/chữ thường. Giá trị không được chứa khoảng trắng (ví dụ mật khẩu có khoảng trắng hiện chưa được hỗ trợ).

### Cấu trúc chung

```text
ATG <NHÓM_LỆNH> <THAO_TÁC> [THAM_SỐ...]
```

Trong đó:

- `ATG`: tiền tố bắt buộc để firmware nhận diện lệnh.
- `NHÓM_LỆNH`: thành phần cần tác động: `GNSS`, `ESP`, `MQTT`, `NTRIP` hoặc `CONFIG`.
- `THAO_TÁC` và `THAM_SỐ`: phụ thuộc vào từng nhóm lệnh bên dưới.

Các lệnh `SET` lưu giá trị vào bộ nhớ Preferences (NVS) của ESP32. Khi thay đổi thông số kết nối đang hoạt động, nên khởi động lại thiết bị để các kết nối được tạo lại với cấu hình mới.

### GNSS

Các lệnh này cấu hình module GNSS ở chế độ base và được gửi qua UART đến module.

| Cú pháp | Giải thích |
| --- | --- |
| `ATG GNSS ROVER` | Chuyển module GNSS sang chế độ rover và bật output GGA/GST cần thiết. |
| `ATG GNSS BASE SURVEY_IN <thời_gian> <độ_chính_xác>` | Bật khảo sát vị trí base và chỉ xuất RTCM. `<thời_gian>` tính bằng giây; `<độ_chính_xác>` tính bằng mét. Ví dụ: `ATG GNSS BASE SURVEY_IN 300 1.0`. |
| `ATG GNSS BASE SURVEY_IN <thời_gian> <độ_chính_xác> W_DIAG` | Bật khảo sát base và xuất cả RTCM lẫn các bản tin diagnostic NMEA. Ví dụ: `ATG GNSS BASE SURVEY_IN 300 1.0 W_DIAG`. |
| `ATG GNSS BASE FIXED <vĩ_độ> <kinh_độ> <độ_cao> <độ_chính_xác>` | Cấu hình vị trí base cố định theo LLA. Vĩ độ và kinh độ ở đơn vị độ thập phân, độ cao và độ chính xác ở mét. Ví dụ: `ATG GNSS BASE FIXED 10.7769 106.7009 12.5 0.5`. |
| `ATG GNSS BASE <RTCM_MSG_TYPE> <PORT> ON` | Bật loại bản tin RTCM trên cổng GNSS chỉ định. |
| `ATG GNSS BASE <RTCM_MSG_TYPE> <PORT> OFF` | Tắt loại bản tin RTCM trên cổng GNSS chỉ định. Với Unicore dùng `CURRENT` (cổng nhận lệnh), `COM1`, `COM2` hoặc `COM3`; với UBlox dùng `I2C`, `UART1`, `UART2`, `USB` hoặc `SPI`. Ví dụ: `ATG GNSS BASE 1074 UART2 OFF`. |
| `ATG GNSS BASE ALL <PORT> <ON/OFF>` | Bật hoặc tắt toàn bộ các bản tin RTCM được firmware hỗ trợ trên cổng chỉ định. |
| `ATG GNSS BASE 1006 CURRENT ON` | Với Unicore, bật RTCM1006 mỗi giây trên cổng đang nối ESP32; không chạy lại Survey-In hay đổi tọa độ. |
| `ATG GNSS SET CHIP <UBLOX/UNICORE>` | Chọn loại chip GNSS, lưu vào Preferences và khởi động lại ESP32 để áp dụng. |

### ESP

| Cú pháp | Giải thích |
| --- | --- |
| `ATG ESP RESTART` | Khởi động lại ESP32 ngay lập tức. |
| `ATG ESP SET CONNECTION 4G` | Chuyển phương thức kết nối mạng sang 4G và khởi động lại ESP32 để áp dụng. |
| `ATG ESP SET CONNECTION WIFI` | Chuyển phương thức kết nối mạng sang Wi-Fi và khởi động lại ESP32 để áp dụng. |
| `ATG ESP SET GNSS TX <GPIO>` | Lưu chân GPIO truyền UART từ ESP32 đến GNSS. Ví dụ: `ATG ESP SET GNSS TX 17`. |
| `ATG ESP SET GNSS RX <GPIO>` | Lưu chân GPIO nhận UART từ GNSS về ESP32. Ví dụ: `ATG ESP SET GNSS RX 16`. |
| `ATG ESP SET WIFI SSID <ssid>` | Lưu tên mạng Wi-Fi cần kết nối. |
| `ATG ESP SET WIFI PASS <mật_khẩu>` | Lưu mật khẩu mạng Wi-Fi. |
| `ATG ESP SET 4G APN <apn>` | Lưu APN của nhà mạng 4G. Ví dụ: `ATG ESP SET 4G APN v-internet`. |
| `ATG ESP SET 4G USER <tên_người_dùng>` | Lưu tên người dùng APN 4G. Ví dụ: `ATG ESP SET 4G USER user`. |
| `ATG ESP SET 4G PASS <mật_khẩu>` | Lưu mật khẩu APN 4G. Ví dụ: `ATG ESP SET 4G PASS password`. |

### MQTT

| Cú pháp | Giải thích |
| --- | --- |
| `ATG MQTT SET SERVER <địa_chỉ>` | Lưu tên miền hoặc địa chỉ IP của MQTT broker. Ví dụ: `ATG MQTT SET SERVER broker.example.com`. |
| `ATG MQTT SET PORT <cổng>` | Lưu cổng MQTT (số nguyên 0–65535). Ví dụ: `ATG MQTT SET PORT 1883`. |
| `ATG MQTT SET USER <tên_người_dùng>` | Lưu tên người dùng đăng nhập MQTT. |
| `ATG MQTT SET PASS <mật_khẩu>` | Lưu mật khẩu đăng nhập MQTT. |
| `ATG MQTT SET PUBTPCHEALTH <topic>` | Lưu topic publish dữ liệu health check. Ví dụ: `ATG MQTT SET PUBTPCHEALTH tdm2402/node-01/health`. |
| `ATG MQTT SET SUBTPCCMD <topic>` | Lưu topic subscribe để nhận lệnh MQTT. Ví dụ: `ATG MQTT SET SUBTPCCMD tdm2402/node-01/cmd`. |

### NTRIP

| Cú pháp | Giải thích |
| --- | --- |
| `ATG NTRIP SET CSTRADDR <địa_chỉ>` | Lưu tên miền hoặc địa chỉ IP của NTRIP caster. |
| `ATG NTRIP SET CSTRPORT <cổng>` | Lưu cổng của NTRIP caster (số nguyên 0–65535), thường là `2101`. |
| `ATG NTRIP SET MNTPNT <mountpoint>` | Lưu mountpoint cần kết nối. Ví dụ: `ATG NTRIP SET MNTPNT VRS_RTCM32`. |
| `ATG NTRIP SET CSTRAUTH <chuỗi_xác_thực>` | Lưu chuỗi xác thực NTRIP theo định dạng base64. Giá trị này thường là base64 của `username:password`. |

### Cấu hình hệ thống

| Cú pháp | Giải thích |
| --- | --- |
| `ATG CONFIG RESET` | Đánh dấu khôi phục cấu hình mặc định, sau đó khởi động lại ESP32. Lần khởi động kế tiếp sẽ xóa toàn bộ cấu hình đã lưu trong Preferences và nạp lại giá trị mặc định. |

> Lệnh JSON từ CORS phản hồi qua `last_command_result` trong gói status MQTT/WebSocket. Lệnh `ATG` cũ vẫn ghi kết quả lên Serial Monitor; lệnh GNSS báo lỗi khi thiếu hoặc thừa tham số.
