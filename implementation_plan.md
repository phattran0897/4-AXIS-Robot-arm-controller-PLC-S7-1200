# Thêm Phân Loại Tốt/Xấu vào PLC & Giao Diện

## Bối cảnh

Hiện tại hệ thống YOLO nhận diện vật thể và trả về `class_id` (0 = TỐT, khác = XẤU). Logic phân loại đã có trong code (`_yolo_processing_loop` → `SortResult.GOOD/BAD`), nhưng:

1. **Không ghi trạng thái phân loại lên PLC** – DB5 đã có sẵn 3 biến Bool `PHAN_LOAI_HANG` (78.0), `HANG_TOT` (78.1), `HANG_XAU` (78.2) nhưng code chưa bao giờ đọc/ghi chúng.
2. **Không hiển thị trạng thái tốt/xấu trên giao diện** – GUI chỉ hiện bộ đếm, không có chỉ báo loại hàng hiện tại.
3. **Không hiện nhãn trên camera** – Video overlay chỉ hiện tọa độ, không hiện tên loại hàng.

## Proposed Changes

### PLC Configuration
- **config.yaml**: Thêm 3 offset mới: `phan_loai_hang_byte: 78`, `hang_tot_byte: 78`, `hang_xau_byte: 78` với bit 0/1/2

### Config Loader  
- **config_loader.py**: Thêm 6 fields vào `PLCOffsets`, cập nhật `compute_db_read_size()` → 80 bytes, cập nhật `load_config()`

### PLC Controller
- **plc_controller.py**: `_db_read_size` >= 80, đọc 3 bit phân loại trong `read_status()`, thêm `write_classification()` method

### Sorting Controller
- **sorting_controller.py**: Ghi `PHAN_LOAI_HANG/HANG_TOT/HANG_XAU` lên PLC trước khi sort, reset sau khi xong

### YOLO Detector
- **yolo_detector.py**: Thêm `class_name` vào `DetectionResult`, hiển thị tên class + "TỐT"/"XẤU" trên video overlay

### GUI Page Auto
- **page_auto.py**: Thêm label hiển thị trạng thái phân loại: "✅ HÀNG TỐT" / "❌ HÀNG XẤU" / "⏳ CHỜ PHÂN LOẠI"

## Verification
- Chạy `python -m pytest tests/test_robot_system.py -v`
- Kết nối PLC kiểm tra 3 biến Bool cập nhật đúng trên TIA Portal
