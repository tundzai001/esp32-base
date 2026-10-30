# AITOGY-GNSS-BASE-ESP-UM980

**Hướng dẫn kỹ thuật nội bộ | Firmware V2.3.5 | Cập nhật 25/10/2026**

## 1. Phạm vi và nguyên tắc vận hành

Bộ AITOGY-BASE-ESP nhận dữ liệu từ receiver GNSS, chuyển tiếp RTCM lên caster bằng 4G hoặc Wi-Fi và nhận điều khiển từ hệ thống CORS. Anten base phải đứng yên khi phát cải chính. Sau khi di chuyển anten, cấu hình lại tọa độ base trước khi cho rover sử dụng.

Tài liệu mô tả firmware **V2.3.5**, cập nhật **25/10/2026**. Kết quả biên dịch và kiểm tra phần mềm không thay thế nghiệm thu ngoài thực địa. Không cam kết thời gian khởi động cố định hay độ chính xác đo khi chưa kiểm tra receiver, anten, mạng cải chính và hệ tọa độ.

| Chế độ | Khi sử dụng | Điều kiện hoàn thành |
| --- | --- | --- |
| FIXED_LLA | Đã có tọa độ điểm đặt anten đáng tin cậy. | Gửi cấu hình fixed, kiểm tra tọa độ receiver và luồng RTCM. |
| SURVEY_IN | Cần receiver tự khảo sát vị trí đứng yên. | Receiver kết thúc khảo sát; gửi lệnh thành công không có nghĩa khảo sát đã xong. |
| AUTO_CORS | Có tài khoản rover nhận cải chính từ CORS. | Chuyển rover, nhận cải chính, lấy mẫu RTK_FIXED, đặt lại base và kiểm tra phát RTCM. |

Độ cao phải dùng đúng quy ước của tọa độ cung cấp. Với cấu hình fixed theo LLA, sử dụng độ cao ellipsoid. Không thay trực tiếp bằng độ cao thủy chuẩn hoặc bỏ qua chiều cao anten/điểm quy chiếu.

## 2. Kết nối và chuẩn bị

| Đường tín hiệu TDM240x | Kết nối |
| --- | --- |
| GPIO18 / RX GNSS của ESP32 | Nhận từ TX của receiver GNSS. |
| GPIO19 / TX GNSS của ESP32 | Gửi vào RX của receiver GNSS. |
| GND | Chung mass giữa ESP32, receiver và bộ nguồn theo thiết kế bộ thiết bị. |
| GPIO16 / RX modem | Nhận từ TX modem. |
| GPIO17 / TX modem | Gửi vào RX modem. |
| Serial console | 115200 baud; gửi một dòng lệnh và kết thúc bằng LF. |
| UART GNSS | Mặc định 115200 baud, 8N1. |

Đối chiếu nhãn cổng và sơ đồ của đúng board trước khi đấu dây. Không suy ra chân cắm hay mức nguồn từ hình ảnh của board khác. Gắn anten GNSS đúng cổng, đặt nơi thoáng trời, cố định cơ khí; kiểm tra SIM/APN và anten 4G trước khi chạy auto base.

Bản Unicore dùng UM980 hoặc UM982 với bộ lệnh ASCII tương thích. Đường UART firmware nối COM2 receiver; không mặc định coi cổng USB/Type-C trên mọi board là COM2. Cần xác nhận sơ đồ đúng module. Muốn đổi cổng receiver phải sửa hằng GNSS_UNICORE_PORT trong cấu hình nguồn và kiểm tra cả nhánh bootstrap; đây không phải tùy chọn -D đang được hỗ trợ.

## 3. Cấu hình mặc định và thông tin cần bàn giao

| Thông số | Giá trị/ý nghĩa trong firmware |
| --- | --- |
| Kết nối mạng | 4G; có thể chuyển Wi-Fi bằng ATG. |
| APN | v-internet; thay theo SIM thực tế. |
| MQTT broker mặc định | 45.117.179.134:1883; phải khớp hệ thống CORS đang triển khai. |
| Backend WebSocket | base.aitogy.com:8000, giao thức ws; cần endpoint thiết bị thực sự truy cập được. |
| Caster dự phòng trong cấu hình mặc định | aitogy.com.vn:2101, mountpoint /test; không phải thông tin cấp phát cho mọi khách hàng. |
| Kênh quản lý chính | Topic theo serial esp_... do thiết bị tạo; quản trị viên cấp quyền trên CORS. |
| Trạng thái backend | Chu kỳ mặc định 5 giây; kết nối mạng có thể làm chậm việc gửi. |
| Health kiểu cũ | Chu kỳ mặc định 30 giây. |

Đơn vị triển khai bàn giao: serial, tên trạm, loại GNSS, caster/mountpoint phát, tài khoản cải chính đầu vào nếu dùng AUTO_CORS, APN hoặc Wi-Fi và người quản lý trên CORS. Mật khẩu/token do quản trị viên cấp riêng, không lấy từ ví dụ trong tài liệu.

## 4. Đưa thiết bị lên CORS

1. Cấp nguồn ổn định, mở console 115200 baud và ghi lại serial `esp_...`, phiên bản firmware, loại chip, baud nhận diện được.
2. Cấu hình mạng, MQTT và backend theo triển khai. Các chuỗi trong dấu `<...>` là giá trị cần thay, không nhập nguyên dấu ngoặc.
3. Lưu thông tin trước, gửi lệnh chọn mạng sau cùng vì lệnh đó khởi động lại thiết bị.
4. Trên CORS, quản trị viên nhận/cấp thiết bị, đặt tên, triển khai license và chọn đúng loại chip.
5. Triển khai dịch vụ caster: server 1, server 2 nếu cần, mountpoint, tài khoản phát và chế độ luôn phát/theo yêu cầu.
6. Chọn FIXED_LLA, SURVEY_IN hoặc AUTO_CORS. Theo dõi kết quả lệnh và tiến độ, sau đó kiểm tra luồng RTCM bằng rover độc lập.

Ví dụ cấu hình 4G:

```text
ATG ESP SET 4G APN <apn>
ATG MQTT SET SERVER <mqtt_host>
ATG MQTT SET PORT 1883
ATG MQTT SET USER <mqtt_user>
ATG MQTT SET PASS <mqtt_password>
ATG ESP SET BACKEND HOST <backend_host>
ATG ESP SET BACKEND PORT 8000
ATG ESP SET CONNECTION 4G
```

Ví dụ đổi sang Wi-Fi:

```text
ATG ESP SET WIFI SSID <ssid>
ATG ESP SET WIFI PASS <wifi_password>
ATG ESP SET CONNECTION WIFI
```

Console ATG tách tham số theo dấu cách, không hỗ trợ chuỗi có khoảng trắng hoặc giá trị rỗng. Nếu SSID/mật khẩu cần dạng này, liên hệ kỹ thuật; không dùng dấu nháy để giả lập chuỗi.

Thiết bị chưa được cấp quyền hoặc chưa có license vẫn có thể báo trạng thái nhưng không đủ điều kiện phát RTCM. Trạng thái `online` của kênh quản lý cũng chưa chứng minh rover nhận được cải chính.

## 5. Chọn chế độ base và xác nhận kết quả

### 5.1 Tọa độ cố định

Nhập vĩ độ/kinh độ dạng độ thập phân và độ cao ellipsoid tính bằng mét. Kiểm tra đúng điểm quy chiếu anten và hệ tọa độ. Sau khi cấu hình, đối chiếu tọa độ đọc từ receiver, sự hiện diện của bản tin tọa độ base và tọa độ rover tại điểm kiểm tra.

### 5.2 Survey-in

Đặt anten đứng yên, nhập thời gian khảo sát theo giây. Nhánh Unicore hiện sinh `mode base time <seconds>`; tham số accuracy của giao diện ATG vẫn bắt buộc về cú pháp nhưng không được đưa vào lệnh survey/fixed ASCII này.

Survey-in không tự tạo ra tọa độ tuyệt đối đạt độ chính xác khảo sát chỉ vì chọn ngưỡng nhỏ. Khi cần tọa độ gắn với hệ quy chiếu cụ thể, dùng điểm đã biết hoặc AUTO_CORS và thực hiện kiểm tra độc lập.

### 5.3 Auto base từ CORS

Firmware chuyển receiver sang rover, nhận RTCM từ mountpoint cải chính và chỉ chấp nhận GGA mới với trạng thái **RTK_FIXED**. RTK_FLOAT không được dùng làm mẫu auto base.

Luồng mặc định cần 5 lần kiểm tra FIX liên tiếp theo nhịp khoảng 1 giây, sau đó lấy tối đa 60 mẫu trong cửa sổ 60 giây. Cần ít nhất 15 mẫu hợp lệ để tiếp tục; không mặc định coi kết quả luôn có đủ 60 mẫu. Thiết bị đặt lại tọa độ base rồi khôi phục dịch vụ phát đã cấu hình.

Thời gian chờ FIX mặc định 3600 giây; quản trị viên có thể cấu hình theo nghiệp vụ. Khi mất FIX, mất kết nối, thiếu mẫu hoặc có lệnh dừng, xem lý do trong tiến độ workflow. Việc khôi phục kết nối caster sau thất bại không tự chứng minh receiver đã trở về cấu hình base mong muốn.

Phần chuyển VN2000 hiện dùng tham số tích hợp và phép chiếu theo lựa chọn tỉnh/kinh tuyến. Firmware không giải mã các bản tin datum RTCM 1021/1023/1025. Phải đối chiếu mốc kiểm tra thực địa; không coi nhãn VN2000 là chứng nhận độ đúng hệ tọa độ.

## 6. Theo dõi trạng thái và luồng dữ liệu

| Hiển thị/trường | Cách hiểu |
| --- | --- |
| awaiting_license / unprovisioned | Chưa đủ điều kiện cấp quyền/license để phát RTCM. |
| locked | Thiết bị bị khóa toàn bộ; liên hệ quản trị viên. |
| sleep | Đang chờ kích hoạt luồng theo yêu cầu; chưa chắc là mất mạng. |
| online | Kênh quản lý hoạt động; cần kiểm tra thêm từng server NTRIP. |
| auto_setup_* / reference_check* | Workflow đang thực hiện một bước cấu hình hoặc kiểm tra. |
| last_command_result | Kết quả xử lý lệnh; lệnh khởi động workflow thành công chỉ nghĩa đã nhận việc. |
| auto_base_progress | Theo dõi phase, số mẫu và lý do completed/failed/stopped của workflow. |
| server1/2_connected và server1/2_bps | Kết nối và số byte/giây gửi tới từng caster. |
| rtcm_input_bps | Số byte RTCM/giây được đóng khung ở UART; không phải chứng nhận CRC hợp lệ. |
| GGA/fix/satellites | Chất lượng nghiệm GNSS và dữ liệu vị trí đang nhận. |

Ví dụ cấu trúc health kiểu cũ (giá trị minh họa):

```json
{
  "uptime_s": 120,
  "free_heap_bytes": 120000,
  "connected_via": "GSM",
  "rssi_dbm": 20,
  "mqtt_ok": true,
  "ntrip_ok": true,
  "gnss_data_ok": true,
  "rtcm_decoder": false,
  "rtcm_types": {}
}
```

`connected_via` dùng `GSM` hoặc `WiFi`. `gnss_data_ok` phản ánh đã nhận khung RTCM gần đây, trong ngưỡng 5 giây của tác vụ health. `rtcm_types` rỗng vì ESP32 không đếm từng loại bản tin. `rssi_dbm` là tên trường cũ: Wi-Fi dùng RSSI, còn 4G đang giữ giá trị chất lượng tín hiệu thô do modem trả về, chưa tự chuyển sang dBm. Không dùng giá trị đó như số dBm 4G đã hiệu chuẩn.

Hệ thống nhận NMEA GGA/GSA/GSV/GST để theo dõi GNSS. ESP32 không mặc định tạo một bản sao toàn bộ RTCM lên MQTT. Các trường có tên `valid_packets` hiện là bộ đếm đóng khung, không có kiểm tra CRC RTCM. `hacc` có thể lấy từ GST hoặc fallback HDOP; không coi mọi giá trị `hacc` đều là độ chính xác mét đã xác nhận.

## 7. Danh mục lệnh ATG

ATG dùng để cấu hình kết nối và hỗ trợ kỹ thuật. Ưu tiên CORS để cấu hình base và đọc kết quả lệnh theo từng thiết bị. Từ khóa dùng đúng chữ hoa; mỗi dòng kết thúc bằng LF. Lệnh SET thường lưu cấu hình, cần khởi động lại để kết nối đọc lại giá trị mới.

### 7.1 ESP và mạng

| Cú pháp | Tác dụng |
| --- | --- |
| `ATG ESP RESTART` | Khởi động lại ngay. |
| `ATG ESP SET CONNECTION 4G` | Lưu chọn 4G và khởi động lại. |
| `ATG ESP SET CONNECTION WIFI` | Lưu chọn Wi-Fi và khởi động lại. |
| `ATG ESP SET WIFI SSID <ssid>` | Lưu SSID không có khoảng trắng. |
| `ATG ESP SET WIFI PASS <password>` | Lưu mật khẩu không có khoảng trắng. |
| `ATG ESP SET 4G APN <apn>` | Lưu APN. |
| `ATG ESP SET 4G USER <user>` | Lưu tên đăng nhập APN khi nhà mạng yêu cầu. |
| `ATG ESP SET 4G PASS <password>` | Lưu mật khẩu APN khi nhà mạng yêu cầu. |
| `ATG ESP SET BACKEND HOST <host>` | Lưu hostname/IP của endpoint WebSocket, không kèm scheme hoặc đường dẫn. |
| `ATG ESP SET BACKEND PORT <port>` | Lưu cổng WebSocket; chấp nhận 1-65535. |

### 7.2 MQTT và NTRIP phát

| Cú pháp | Tác dụng |
| --- | --- |
| `ATG MQTT SET SERVER <host>` | Lưu broker MQTT. |
| `ATG MQTT SET PORT <port>` | Lưu cổng MQTT; nhập số hợp lệ 1-65535. |
| `ATG MQTT SET USER <user>` | Lưu tên đăng nhập broker. |
| `ATG MQTT SET PASS <password>` | Lưu mật khẩu broker. |
| `ATG MQTT SET PUBTPCHEALTH <topic>` | Lưu topic health kiểu cũ. |
| `ATG MQTT SET SUBTPCCMD <topic>` | Lưu topic subscribe ATG kiểu cũ; thay topic cần kỹ thuật kiểm tra callback thực nhận. |
| `ATG NTRIP SET CSTRADDR <host>` | Lưu caster mặc định/dự phòng. |
| `ATG NTRIP SET CSTRPORT <port>` | Lưu cổng caster; nhập số hợp lệ 1-65535. |
| `ATG NTRIP SET MNTPNT <mountpoint>` | Lưu mountpoint mặc định. |
| `ATG NTRIP SET CSTRAUTH <password>` | Lưu mật khẩu source nguyên văn; không nhập base64 của username:password. |

Dịch vụ đã cấp từ CORS có thể ưu tiên hơn cấu hình NTRIP mặc định lưu bằng ATG. Muốn thay tài khoản/mountpoint đang vận hành, chỉnh dịch vụ trên CORS rồi triển khai lại.

### 7.3 GNSS và đặt lại cấu hình

| Cú pháp | Tác dụng |
| --- | --- |
| `ATG GNSS ROVER` | Chuyển receiver sang rover để chẩn đoán; không tự đăng nhập tài khoản rover CORS. |
| `ATG GNSS BASE SURVEY_IN <seconds> <accuracy_m>` | Gửi batch survey-in theo loại chip; mặc định chỉ xuất RTCM, không xuất NMEA diagnostic. |
| `ATG GNSS BASE SURVEY_IN <seconds> <accuracy_m> W_DIAG` | Survey-in và xuất cả RTCM lẫn các bản tin NMEA diagnostic; `W_DIAG` không phân biệt hoa thường. |
| `ATG GNSS BASE FIXED <lat> <lon> <alt> <accuracy_m>` | Gửi tọa độ LLA cố định; độ cao tính bằng mét ellipsoid. |
| `ATG GNSS BASE <message> <port> ON` | Bật bản tin RTCM trên cổng chỉ định. |
| `ATG GNSS BASE <message> <port> OFF` | Tắt bản tin RTCM trên cổng chỉ định. |
| `ATG GNSS BASE ALL <port> <ON/OFF>` | Bật hoặc tắt toàn bộ các bản tin RTCM được firmware hỗ trợ trên cổng chỉ định. |
| `ATG GNSS SET CHIP UNICORE` | Lưu lựa chọn chip và khởi động lại; boot vẫn thực hiện nhận diện phần cứng. |
| `ATG CONFIG RESET` | Đặt cờ nạp lại mặc định và khởi động lại; có thể làm đổi mạng và cấu hình GNSS. |

Cổng dùng COM1/COM2/COM3, đường nối ESP32 mặc định COM2. Ví dụ chẩn đoán output: `ATG GNSS BASE 1019 COM2 ON`. Ưu tiên thao tác base qua CORS vì đường ATG cũ chưa kiểm tra OK riêng từng lệnh.

Lệnh ATG kiểu cũ chủ yếu trả log Serial, không có kết quả JSON riêng theo command_id như lệnh CORS. Không coi dòng “đã gửi” là receiver đã chấp nhận cấu hình. Reset cấu hình cục bộ không thay thế thao tác gỡ thiết bị/thu hồi quyền trên CORS; quản trị viên phải xử lý trạng thái cấp phát khi cần.

## 8. RTCM, ephemeris và cấu hình receiver

### 8.1 Bản tin phát

| Nhóm | Bản tin |
| --- | --- |
| Tọa độ/nhận dạng base | 1006, 1033. |
| GPS | MSM4 1074 / MSM7 1077; ephemeris 1019. |
| GLONASS | MSM4 1084 / MSM7 1087; ephemeris 1020. |
| Galileo | MSM4 1094 / MSM7 1097; ephemeris 1045. |
| BeiDou | MSM4 1124 / MSM7 1127; ephemeris 1042. |
| QZSS | MSM4 1114 / MSM7 1117; ephemeris 1044. |

Fixed/survey chọn nhóm theo hệ vệ tinh và mức MSM trong cấu hình; mặc định GPS/GLO/GAL/BDS bật, QZSS tắt. Ephemeris của hệ bật không bị lọc bỏ khi chọn MSM4 hoặc MSM7. Bootstrap UM980 bật riêng đủ 1019/1020/1042/1044/1045 trên COM2, chu kỳ yêu cầu 1 giây, trước saveconfig. Bộ cấu hình LTE có cả hai mức MSM và các ephemeris này. Không đồng nhất bootstrap với mọi cấu hình được cấp về sau.

### 8.2 Kiểm tra trực tiếp Unicore

Kết nối terminal với đúng cổng của receiver, dùng baud đang cấu hình và kết thúc lệnh bằng CRLF. Đây là lệnh gửi trực tiếp vào receiver, không thêm tiền tố ATG.

| Lệnh ASCII trực tiếp | Chức năng |
| --- | --- |
| `version` | Truy vấn model/phiên bản receiver. |
| `mode` | Truy vấn chế độ đang đặt. |
| `log` | Truy vấn output log đang cấu hình. |
| `unlogall` | Yêu cầu ngừng các output log trước khi cấu hình lại. |
| `config com2 115200` | Đặt baud cho COM2; sau thay đổi cần nối đúng baud mới. |
| `mode rover` | Chuyển sang rover; còn cần cải chính đầu vào để có RTK. |
| `mode base time 300` | Yêu cầu survey-in tối thiểu 300 giây. |
| `mode base 10.7500000000 106.5000000000 12.5000` | Ví dụ fixed LLA; thay bằng tọa độ thật của anten, không sử dụng nguyên giá trị mẫu. |
| `gpgga com2 1` / `gpgst com2 1` | Yêu cầu output NMEA phục vụ theo dõi vị trí/chất lượng. |
| `rtcm1074 com2 1` | Yêu cầu output GPS MSM4 trên COM2; chọn bộ bản tin phù hợp rover. |
| `saveconfig` | Lưu cấu hình receiver cho lần khởi động sau. |
| `FRESET` | Khôi phục mặc định receiver; làm mất cấu hình đã đặt và có thể thay baud. |

Không gửi toàn bộ bảng như một batch mẫu. Chọn đúng lệnh cho bước cấu hình, chờ phản hồi và kiểm tra trạng thái; không reset receiver đang khai thác chỉ để thử lệnh.

Ví dụ yêu cầu output ephemeris trên COM2:

```text
rtcm1019 com2 1
rtcm1020 com2 1
rtcm1042 com2 1
rtcm1044 com2 1
rtcm1045 com2 1
saveconfig
```

Gửi từng lệnh, đọc phản hồi đúng lệnh trước khi tiếp tục. Ví dụ thành công: `$command,SAVECONFIG,response: OK` với checksum nếu receiver xuất. Lỗi hoặc hết thời gian chờ cần được xử lý, không gửi reset lặp lại. FRESET xóa cấu hình receiver; chỉ dùng trong quy trình khôi phục có chủ đích.

Unicore nhận lệnh ASCII; không gửi gói UBX của u-blox. Backend có thể sinh log COM3 cho hệ dây khác; ESP32 định tuyến các lệnh output RTCM/NMEA sang cổng nối của nó, không đổi tọa độ hay thời gian khảo sát.


## 9. Xử lý sự cố

| Dấu hiệu | Kiểm tra và hướng xử lý |
| --- | --- |
| Không thấy log console | Đúng cổng USB/driver, 115200 baud, nguồn và cáp truyền dữ liệu. |
| UNKNOWN/INVALID, không có GGA | RX/TX đấu chéo, chung GND, đúng cổng receiver, đúng baud; kiểm tra trực tiếp receiver khi cần. |
| MQTT được nhưng CORS chưa quản lý được | Broker/topic theo serial, thiết bị đã nhận/cấp và license; kiểm tra command/result thay vì chỉ nhìn kết nối TCP. |
| WebSocket không kết nối | Đúng host/port endpoint thiết bị; firmware hiện dùng ws, không phải wss. MQTT vẫn là kênh điều khiển khác. |
| Có NMEA nhưng RTCM bằng 0 | Đúng chế độ base, survey đã xong, đúng output UART và bản tin RTCM đã bật. |
| Có RTCM UART nhưng caster không nhận | Đúng source account/mountpoint/version NTRIP; server được bật, không khóa và đủ license/provision. |
| Có hai kết nối cùng tài khoản rover | Xác minh client nào đo, client nào thuộc dịch vụ monitor/decode trên caster; không xóa tài khoản khi chưa xác định. |
| Auto base không FIX | Tài khoản rover, mountpoint đầu vào có dữ liệu, GGA mới, cổng RTCM input, anten/độ che khuất và khoảng cách trạm. |
| Auto base thiếu mẫu | Kiểm tra FIX có duy trì, GGA có mới và đủ mẫu hợp lệ trong cửa sổ lấy mẫu. |
| Cấu hình GNSS báo lỗi | Đọc lỗi Unicore response timeout/rejected, đúng baud/cổng và phản hồi $command; không coi NO MATCHING FUNC là OK. |
| Khởi động lại vẫn dùng cấu hình cũ | Đối chiếu service đang cấp từ CORS với cấu hình ATG; kiểm tra lưu NVS và kết quả triển khai dịch vụ. |
| Tọa độ lệch dù rover FIX | Hệ quy chiếu, độ cao, chiều cao anten, tọa độ base và phép chuyển VN2000; kiểm tra trên mốc độc lập. |

Khi gửi yêu cầu hỗ trợ: cung cấp serial, phiên bản, thời điểm lỗi, phase/lý do workflow, loại receiver, baud/cổng nối, server nào lỗi và log đã che mật khẩu/token. Không gửi lại reset nhiều lần khi chưa xác định lỗi.

## 10. Nghiệm thu và bảo trì

1. Chụp/ghi nhận đúng đấu dây, nguồn, anten, SIM/APN và serial được cấp.
2. Tắt/bật nguồn kiểm tra thiết bị nạp đúng nhận dạng, dịch vụ, loại GNSS và cấu hình base.
3. Với fixed/survey: đối chiếu tọa độ base và bản tin tham chiếu sau khi receiver ổn định.
4. Với auto base: kiểm tra từng bước chuyển rover, nhận cải chính, RTK_FIXED, số mẫu, chuyển base và phục hồi output.
5. Dùng rover độc lập kết nối đúng mountpoint, kiểm tra độ tuổi cải chính, FIX và điểm mốc đã biết.
6. Kiểm tra riêng server 1/2, luôn phát/theo yêu cầu và khóa/mở từng server; khóa không được bị fail-open bỏ qua.
7. Thử mất mạng rồi phục hồi; kiểm tra kết quả thực tế, không chỉ đèn hoặc nhãn online.
8. Sau khi di chuyển anten, thay receiver/firmware hoặc thay hệ tọa độ, thực hiện lại phần nghiệm thu liên quan.

Không có thao tác đo phần cứng tự động nào được thực hiện chỉ bởi việc đọc hoặc xuất bản tài liệu này. Bản hiện tại đã đối chiếu code; nghiệm thu ngoài hiện trường do đơn vị triển khai ghi nhận riêng.

## 11. Kiến trúc và ranh giới giao thức

Firmware ESP32 là chương trình C++, không chạy Python agent. Một binary chứa hai bộ tạo lệnh, lựa chọn theo loại receiver nhận diện được hoặc cấu hình backend. Không còn macro GNSS_MODULE_TYPE để chọn một loại chip khi biên dịch.

| Thành phần | Trách nhiệm |
| --- | --- |
| gnss_cmd_builder | Kiểu Command/CommandList, tùy chọn và dispatch theo chip. |
| unicore_commands | Tạo ASCII mode/output/saveconfig và ephemeris cho Unicore. |
| Unicore_Reply | Tách dòng $command, khớp token tên lệnh, kiểm tra checksum nếu có và kết quả OK. |
| RTCM receiver | Tách biên NMEA/UBX/RTCM qua nhiều lần đọc UART, cập nhật telemetry; không xác thực CRC RTCM. |
| Backend agent | Nhận lệnh MQTT/WebSocket, quyền thiết bị, workflow, lưu trạng thái và kết quả lệnh. |
| NTRIP transport | Hai source stream và một phiên rover tạm dùng lại client server 1. |

Dữ liệu vận hành: receiver phát RTCM qua UART vào ESP32; ESP32 đẩy lên caster. Khi auto base/reference check, chiều ngược là caster cải chính → phiên rover → UART GNSS. NMEA GGA/GSA/GSV/GST đi lên backend để tính trạng thái; không triển khai decoder RTCM trên ESP32.

### 11.1 UART, task và mutex

- TDM240x: GNSS RX18/TX19, modem RX16/TX17; nhánh BOARD_2A53N dùng GNSS RX22/TX23. Không áp dụng pin của board này cho board kia.
- GNSS RX ring 8192 byte; mutex timeout mặc định 1500 ms. Dò boot lần lượt 115200, 38400, 9600, 19200, 57600, 230400, 460800 baud.
- task MQTT còn xử lý WebSocket và JSON; stack cấu hình 12288 byte. NTRIP 6144 byte, health và console 4096 byte mỗi task.
- Bộ đợi ACK cấu hình chiếm rtcmBufferMutex rồi gnssSerialMutex để task đọc không lấy mất phản hồi. tcpStreamMutex bảo vệ thao tác mạng/modem.
- Mux modem: source 1/rover là 0, MQTT là 1, source 2 là 2, WebSocket là 3. Dừng source trước phiên rover tạm.
- GGA gửi upstream lấy từ receiver; giới hạn khoảng 5 giây/lần và không gửi bản đã cũ. Không tạo tọa độ giả để duy trì FIX.

### 11.2 Khởi tạo và nhận diện

Boot nạp identity/quyền/dịch vụ trước khi mở transport, dò UBX-MON-VER hoặc VERSION để chọn chip. GNSS_MDL_TYPE dùng cho ATG, SENSOR_TYPE dùng cho backend: cần đối chiếu cả hai khi chẩn đoán cấu hình lệch. ATG SET CHIP không bảo đảm ép model qua một lần boot vì boot lại dò phần cứng.

Chỉ lần đầu/reset mặc định mới gọi bootstrap theo chip đã dò: UM980 cấu hình survey/output COM2 và ephemeris, u-blox cấu hình output UART1. Không phải mọi lần khởi động đều ghi lại toàn bộ cấu hình GNSS. Sau nhận diện không hợp lệ, không coi thao tác khởi tạo là thành công.

## 12. Giao thức backend và điều kiện nhận lệnh

Topic chính dùng serial thực:

```text
pi/devices/<serial>/status
pi/devices/<serial>/raw_data
pi/devices/<serial>/command
pi/devices/<serial>/control_ack
pi/devices/<serial>/base_config_state
pi/devices/<serial>/service_config_state
```

Có subscribe tương thích `/commands` và `pi/device/<serial>/command`. WebSocket nối đường dẫn `/ws/pi/<serial>?token=<license>` qua ws, gửi status_update/nmea_update. MQTT hiện là TCP không TLS; không suy luận có TLS từ việc website quản trị dùng HTTPS.

Lệnh JSON có command, command_id và payload. original_config của EXECUTE_RAW_COMMANDS nằm ở cấp envelope, không nằm trong payload. Ví dụ dưới đây chỉ mô tả giao thức; gửi qua dispatcher CORS để áp dụng quyền, command ID và kiểm tra tham số.

```json
{
  "command": "LOCK_DEVICE",
  "command_id": "example-lock-server2",
  "payload": {"locked_servers": [2]}
}
```

| Lệnh | Trường chính và hành vi |
| --- | --- |
| PROVISION_DEVICE | payload.name; đặt tên và đánh dấu thiết bị đã cấp phát. |
| DEPLOY_LICENSE | payload.license_key; kiểm tra định dạng token dt_... hoặc license số cũ, lưu thành công mới báo nhận và reboot. |
| DEPLOY_SERVICE_CONFIG | Payload là object dịch vụ; lưu, nạp lại endpoint và thử kết nối. |
| EXECUTE_RAW_COMMANDS | payload.commands_b64; original_config có mode BASE, sensor_type và params nếu cấu hình base. |
| SET_RTCM_STREAM_ACTIVE | payload.active; tùy chọn server_id, mountpoint và address để chọn luồng. Luồng luôn phát bỏ qua yêu cầu tắt theo nhu cầu. |
| TRIGGER_AUTO_BASE | ip, port, user, password, mountpoint, sensor_type; các tùy chọn timeout, province_code, l0_deg, k0, enable_itrf_vn2000_transform. |
| TRIGGER_BASE_REFERENCE_CHECK | Kết nối rover, reference_coords và cấu hình base đã lưu; đo so sánh, sau đó cố gắng khôi phục base cũ. |
| STOP_AUTO_BASE | Yêu cầu hủy workflow; theo dõi phase stopped/failed và kết quả cleanup. |
| APPLY_VN2000_PROVINCE | province_code và cấu hình tọa độ base hợp lệ; áp dụng phép chuyển tỉnh/kinh tuyến. |
| LOCK_DEVICE | Payload rỗng khóa cả thiết bị; locked_servers chọn server 1/2. |
| UNLOCK_DEVICE | Xóa khóa toàn thiết bị và khóa server. |
| CHECK_BASE_STATUS | Gửi poll trạng thái receiver; kết quả success chỉ xác nhận yêu cầu truy vấn đã gửi. |
| REBOOT_DEVICE | Lưu trạng thái và khởi động lại. |
| DELETE_DEVICE | Xóa trạng thái thiết bị theo handler và khởi động lại; phối hợp với quản trị backend trước khi cấp lại. |

Lệnh không biết bị từ chối. Khóa toàn thiết bị chặn lệnh ngoài khóa/mở. Chưa có license chỉ cho phép nhóm khóa/mở, cấp thiết bị và triển khai license. Không được coi việc subscribe topic là đã có quyền thực thi mọi lệnh.

Kết quả có running, success, error hoặc rejected gắn với command_id; workflow chạy bất đồng bộ có tiến độ riêng. Không gộp success của việc xếp hàng workflow với completed của thao tác ngoài hiện trường.

### 12.1 Dịch vụ NTRIP và khóa theo server

Dịch vụ dùng server1_enabled/server2_enabled, serverhost1/2, port1/2, username1/2, password1/2, mountpoint1/2, ntrip_version1/2, server1/2_stream_on_demand và server1/2_stream_active. Kiểu NTRIP v1 dùng SOURCE với mật khẩu nguyên văn; v2 dùng POST và HTTP Basic do firmware tự mã hóa.

```json
{
  "server1_enabled": true,
  "serverhost1": "<caster_host>",
  "port1": 2101,
  "username1": "<source_user>",
  "password1": "<source_password>",
  "mountpoint1": "<station_mountpoint>",
  "ntrip_version1": 1,
  "server1_stream_on_demand": false,
  "server1_stream_active": true,
  "server2_enabled": false
}
```

LOCK_DEVICE với [1,2] khóa cả hai output nhưng vẫn khác khóa toàn thiết bị. [] hoặc UNLOCK_DEVICE bỏ khóa; không tự bật endpoint bị tắt. Khi mất ACK điều khiển khoảng 45 giây, fail-open có thể mở luồng theo chính sách lưu; license, provision và khóa vẫn là điều kiện bắt buộc. ACK mới hợp lệ cần phục hồi ổn định khoảng 5 giây để thoát fail-open.

## 13. ACK, readback và mức bảo đảm

Đường cấu hình từ backend và auto base dùng bộ đọc Unicore riêng. Trước khi gửi, từ chối gói nhị phân hoặc nhiều dòng lệnh. Mỗi lần trao đổi lấy quyền UART RX/TX, bỏ byte tồn cũ, kiểm tra số byte ghi và chờ phản hồi tối đa 8000 ms.

Parser chỉ xét dòng $command,<command>,response:..., so khớp token lệnh đầu tiên không phân biệt hoa/thường; không so khớp toàn bộ tham số echo. Giá trị response sau trim phải đúng OK. NOT OK, NO MATCHING FUNC và lỗi khác bị từ chối. Nếu có *xx thì kiểm tra XOR gồm cả ký tự $; khác quy tắc NMEA thông thường.

Khi lỗi/timeout, dừng batch, không báo success cho cấu hình còn lại. reset/freset chỉ gửi một lần; sau ACK chờ thêm 2000 ms. ACK chưa phải readback toàn bộ cấu hình sau reboot. Truy vấn CHECK_BASE_STATUS gửi mode/log theo đường truy vấn, không áp quy tắc ACK cấu hình cho dữ liệu trả lời truy vấn.

### 13.1 Giới hạn đường ATG cũ

sendGnssCommands của ATG ghép CommandList thành một dãy byte rồi gửi, đọc log tối đa khoảng 500 ms; không kiểm tra ACK riêng từng lệnh. Các marker $DELAY_...$ của batch Unicore không được đường ATG này thực thi thành khoảng chờ. Do đó dùng CORS cho cấu hình base Unicore có kiểm tra phản hồi, và không kết luận receiver đã nhận đủ cấu hình từ log “đã gửi” của ATG.

SUBTPCCMD được lưu và subscribe theo Preferences nhưng callback ATG vẫn so topic với hằng TOPIC_SUB_CMD. Đổi topic có thể làm ATG không được dispatch; triển khai mới nên dùng topic JSON theo serial. MQTT PORT/CSTRPORT chuyển số sang uint16_t mà chưa có kiểm tra miền đầy đủ; công cụ gửi phải kiểm tra số 1-65535.

### 13.2 Auto base và reference check

Auto base chỉ lấy RTK_FIXED với GGA mới; mặc định cần 5 lần kiểm tra FIX liên tiếp theo nhịp khoảng 1 giây. Cửa sổ lấy mẫu 60 giây, mục tiêu 60, tối thiểu 15. Payload samples khác 60 được ghi nhận là overridden, không thay mục tiêu của auto base. Reference check dùng samples riêng, giới hạn 5-120, mặc định 60 và timeout mặc định 300 giây.

Khi reference check dừng/kết thúc, cleanup cố gắng đặt lại tọa độ base đã lưu và phục hồi transport. Auto base thất bại trước bước đặt lại base chỉ khôi phục source transport, chưa luôn khôi phục receiver mode/tọa độ cũ. Kiểm tra lại receiver trước khi đưa mountpoint vào khai thác.

VN2000 dùng tham số tích hợp; không có decoder datum từ RTCM. Tọa độ NMEA gồm độ cao anten theo bản tin receiver; kiểm tra quy đổi ellipsoid/geoid và chiều cao anten trước khi sử dụng kết quả như tọa độ mốc.

## 14. Lưu cấu hình, cập nhật và khôi phục

| Nhóm dữ liệu | Vị trí/đặc điểm |
| --- | --- |
| Wi-Fi, APN, MQTT, legacy NTRIP | Preferences myPrefs; một số thay đổi chỉ nạp lại sau reboot. |
| Serial, tên, chip, provision, license, khóa, BASE_JSON/SERVICE_JSON, kết quả | NVS agent_state; có nhánh tương thích dữ liệu cũ trong myPrefs. |
| GNSS receiver | Cấu hình riêng trong receiver; cần SAVECONFIG/CFG-CFG và kiểm tra sau mất nguồn. |
| CONFIG RESET | Xóa/nạp lại myPrefs và khởi tạo theo needReset; không bảo đảm xóa agent_state riêng hoặc trạng thái trên backend. |

Serial mới có dạng esp_ + phần định danh MAC; ID LP_ cũ được đổi tiền tố và giữ suffix. Chỉ lưu trạng thái agent khi identity đã nạp; token được kiểm tra định dạng tại firmware, backend vẫn chịu trách nhiệm xác thực token thiết bị thực tế.

Bảng phân vùng 4 MB hiện có app0/app1 mỗi 0x140000 byte, agent_state tại 0x3D0000 kích thước 0x20000 và coredump tại 0x3F0000. Có hai slot ứng dụng không có nghĩa firmware đã triển khai lệnh OTA; danh mục lệnh hiện không có cập nhật firmware qua mạng.

Quy trình cập nhật: ghi lại dịch vụ/serial, dùng đúng board và partition, build rồi upload qua USB, kiểm tra log boot và nhận diện chip, triển khai lại dịch vụ nếu cần, thực hiện nghiệm thu. Không dùng erase-flash như bước sửa lỗi mặc định vì có thể mất identity/license/cấu hình.

```sh
pio run -e tdm240x_dev_general
pio run -e tdm240x_dev_general -t upload
pio device monitor -b 115200
pio run -e tdm240x_dev_general -t compiledb
```

Chạy từ root dự án firmware. Trong repo CORS có thể chạy ở root CORS qua cấu hình wrapper hoặc vào geodetic/esp32. compiledb cập nhật IntelliSense sau khi thêm/tách file; compile_commands.json chứa đường dẫn theo workspace, không dùng chung nguyên byte giữa các vị trí checkout.

Các env debug/test bật PROGRAM_DEBUG/DUMP_AT_COMMANDS nên log có thể chứa nội dung ATG/traffic modem. Chia sẻ log đã che dữ liệu truy cập; tài liệu và ví dụ không chứa mật khẩu vận hành thực tế.

## 15. Kiểm tra phần mềm và nghiệm thu nội bộ

```sh
c++ -std=c++17 -fsanitize=address,undefined -g -Itest/host -Iinclude \
  test/host/base_checks.cpp src/functions/Unicore_Log.cpp src/functions/gnss_cmd_builder.cpp \
  src/functions/ublox_commands.cpp src/functions/unicore_commands.cpp \
  -o /tmp/esp32-base-checks
/tmp/esp32-base-checks
python3 -B test/host/check_cors.py /path/to/cors /tmp/esp32-base-checks
python3 -B test/host/check_gnss_stream.py
python3 -B test/host/check_rover_gga.py
```

Đã kiểm tra build env tdm240x_dev_general, bộ tạo lệnh hai protocol/routing với backend, parser khung xen kẽ và GGA mới/cũ. Đây là kiểm tra host/build, không phải thử nghiệm RF hoặc UART trên receiver thật. check_status_flash.py mở cổng Serial để quan sát log sau khi nạp; script không upload firmware. Cần thiết bị thật và Serial Monitor đóng, kiểm tra cấu hình DTR/RTS theo board trước khi chạy.

Biên bản thực địa phải ghi: board/model/firmware receiver, nguồn, cổng/baud, anten và chiều cao, hệ tọa độ, mode, thời gian chờ và số mẫu, account/mountpoint đã che thông tin truy cập, tốc độ/tuổi RTCM, kết quả rover tại mốc và tình trạng sau reboot/mất mạng. Chỉ xác nhận đạt sau khi có số liệu và người nghiệm thu.

## Tài liệu tham chiếu

- Bộ lệnh ASCII và phản hồi mô tả ở đây được đối chiếu trực tiếp với bộ tạo lệnh và parser trong firmware V2.3.5; khi thay receiver/firmware GNSS cần kiểm tra tài liệu của đúng model trước khi áp dụng.
- [Hướng dẫn tích hợp CORS](base-4g.md), command handler, bộ tạo lệnh theo chip và các bài kiểm tra host trong cùng bản nguồn là cơ sở kiểm tra hành vi firmware.
