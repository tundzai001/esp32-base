# Tài liệu AITOGY-BASE-ESP

Firmware **V2.3.5**, đối chiếu mã nguồn ngày **25/10/2026**. Chọn đúng receiver và đối tượng đọc:

| Thiết bị | Khách hàng/vận hành | Kỹ thuật nội bộ |
| --- | --- | --- |
| Unicore UM980/UM982 | [Markdown](Catalogue-Unicore-customer.md) / [PDF](Catalogue-Unicore-customer.pdf) | [Markdown](Catalogue-Unicore-inside.md) / [PDF](Catalogue-Unicore-inside.pdf) |
| u-blox, mục tiêu ZED-F9P tương thích | [Markdown](Catalogue-uBlox-customer.md) / [PDF](Catalogue-uBlox-customer.pdf) | [Markdown](Catalogue-uBlox-inside.md) / [PDF](Catalogue-uBlox-inside.pdf) |

Bản khách hàng gồm đấu dây, cấu hình mạng/CORS, fixed/survey/auto base, theo dõi trạng thái, danh mục ATG, bản tin receiver, xử lý sự cố và nghiệm thu. Bản nội bộ có toàn bộ nội dung vận hành cùng giao thức JSON, quyền/license, ACK/readback, lưu NVS, khóa/fail-open, build và giới hạn triển khai.

[Hướng dẫn tích hợp CORS](base-4g.md) là tài liệu kỹ thuật bổ sung. Mật khẩu/token trong ví dụ đều là placeholder, không phải thông tin cấp cho thiết bị thực tế.

## Điểm cần phân biệt

- Unicore dùng ASCII, u-blox dùng UBX nhị phân; không gửi chéo bộ lệnh.
- TX/RX phải đấu chéo: TDM240x RX18 nhận TX receiver, TX19 gửi vào RX receiver. Unicore mặc định COM2; u-blox dùng UART1.
- Auto base chỉ lấy GGA mới với RTK_FIXED; mục tiêu 60 mẫu trong 60 giây, tối thiểu 15 mẫu hợp lệ.
- RTCM chuyển tiếp nguyên byte; ESP32 không giải mã loại bản tin/CRC/datum RTCM. Bộ đếm khung không phải kiểm định chất lượng cải chính.
- Cấu hình từ backend và ATG cũ có mức kiểm tra ACK khác nhau. Đọc phần giới hạn trước khi kết luận receiver đã nhận lệnh.
- Tài liệu phản ánh bản nguồn; thử nghiệm phần cứng và kiểm chứng tọa độ cần biên bản thực địa riêng.

## Cập nhật và xuất PDF

Sửa Markdown trước, xuất lại cả bốn PDF để không tồn tại nội dung khác nhau giữa hai định dạng:

```sh
python3 -m pip install reportlab
python3 docs/render_catalogues.py
```

Renderer dùng Arial/Menlo trên macOS hoặc DejaVu trong `/usr/share/fonts/truetype/dejavu` trên Linux. Có thể chỉ định thư mục chứa DejaVuSans.ttf, DejaVuSans-Bold.ttf, DejaVuSans-Oblique.ttf và DejaVuSansMono.ttf bằng `--font-dir`. Mặc định PDF ghi đè tên hiện có trong docs; `--output-dir` cho phép xuất sang thư mục con bên trong docs.

Sau khi xuất: mở PDF, kiểm tra chữ tiếng Việt, bảng dài qua trang, ví dụ lệnh, hình u-center và số trang. Đồng bộ Markdown, PDF, hình và renderer giữa `Desktop/tun/esp32/docs` và `cors/geodetic/esp32/docs`. Không đồng bộ thư mục build hoặc compilation database theo đường dẫn máy.
