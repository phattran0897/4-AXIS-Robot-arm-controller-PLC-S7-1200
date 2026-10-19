"""Vietnamese operator guidance shared by labels and their controls."""

from html import escape

from PySide6.QtWidgets import QWidget


def set_help(widget: QWidget, title: str, text: str) -> None:
    """Use native hover help, with readable wrapping and a persistent base text."""
    content = (
        '<div style="max-width: 340px; white-space: normal;">'
        f"<b>{escape(title)}</b><br/>{escape(text).replace(chr(10), '<br/>')}</div>"
    )
    widget.setProperty("helpText", content)
    widget.setToolTip(content)
    widget.setToolTipDuration(15000)


def set_help_notice(widget: QWidget, notice: str = "") -> None:
    """Append a temporary blocking reason without losing the usage instructions."""
    content = widget.property("helpText") or ""
    if notice:
        content += (
            '<div style="max-width: 340px; white-space: normal;">'
            f"<br/><b>Chưa thể thao tác:</b><br/>{escape(notice)}</div>"
        )
    if widget.toolTip() != content:
        widget.setToolTip(content)


SECTION_HELP = {
    "Automation overview": "Theo dõi camera, gửi lệnh chạy tự động và xem kết quả phân loại. Giữ chuột trên tên mục hoặc nút để xem hướng dẫn.",
    "Manual workspace": "Nhập góc khớp hoặc tọa độ, tính toán rồi gửi mục tiêu đến PLC. Các nút cơ cấu và JOG gửi lệnh điều khiển riêng.",
    "System configuration": "Chỉnh thông số nhận diện, vùng ảnh và chu kỳ điều khiển. Nhấn Apply & save settings để kiểm tra, lưu và áp dụng thay đổi.",
    "CONTROL PANEL": "Start gửi lệnh chạy tự động đến PLC; Pause gửi lệnh tạm dừng. Capture & classify dùng ảnh camera để phân loại trong chế độ Vision.",
    "JOINT STATUS": "Hiển thị góc mục tiêu, đơn vị độ (°). Vision dùng mục tiêu do PC gửi; QR dùng dữ liệu mục tiêu đọc từ PLC. Đây không phải số đo góc thực tế từ encoder.",
    "CAMERA PREVIEW": "Xem ảnh camera và chọn nguồn ở Source. LIVE là camera đang hoạt động; STANDBY là chưa có camera hoạt động.",
    "KINEMATICS INPUT": "Forward: từ góc J1–J4 tính X, Y, Z. Inverse: từ X, Y, Z tính góc khớp. Nhấn Calculate trước khi Send targets.",
    "ACTUATOR CONTROL": "Điều khiển về Home, đóng/mở kẹp và JOG từng khớp. Kiểm tra vùng chuyển động trước khi gửi lệnh.",
    "VISION & ROI": "Cấu hình độ tin cậy YOLO và quy đổi tọa độ ảnh sang mm cho chế độ Vision. ROI là vùng ảnh được dùng để nhận diện.",
    "ROI BOX": "X, Y là góc trên bên trái; Width, Height là kích thước vùng ảnh, theo pixel của ảnh camera gốc. Nhấn Apply & save settings để áp dụng.",
    "ROBOT LIMITS": "Các độ cao gắp, tiếp cận và thả vật trong chế độ Vision, đơn vị mm theo hệ tọa độ robot.",
    "TIMING": "Cấu hình thời gian chờ giữa các lượt phân loại và chu kỳ đọc trạng thái PLC, đơn vị giây.",
    "SORTING": "Chọn Vision để phân loại qua YOLO hoặc QR để đọc mã và gửi mục tiêu cho PLC. Thay đổi có hiệu lực sau Apply & save settings.",
    "JOG CONTROL": "Nhấn một lần để bật JOG theo hướng đã chọn, nhấn lại để tắt. ON/OFF là trạng thái lệnh. Đổi hướng sẽ tắt hướng đối diện trước; rời trang hoặc mất tiêu điểm cửa sổ sẽ yêu cầu tắt JOG.",
}

SETTINGS_HELP = {
    "yolo.thresh": "Ngưỡng tin cậy YOLO, lớn hơn 0 và không vượt quá 1. Ví dụ 0,50 = 50%. Tăng ngưỡng để loại bớt nhận diện có độ tin cậy thấp; giảm có thể nhận thêm vật nhưng cũng tăng nhận diện sai.",
    "yolo.px2mm": "Hệ số mm trên một pixel của ảnh gốc, phải lớn hơn 0. Đo một khoảng cách thực rồi chia cho số pixel tương ứng. Hiệu chuẩn lại nếu thay vị trí hoặc độ cao camera.",
    "yolo.coordinate_origin": "roi: home_x/home_y được hiệu chuẩn tại tâm vùng ROI. frame: hiệu chuẩn tại tâm toàn bộ ảnh camera. Chọn đúng hệ quy chiếu đã dùng khi hiệu chuẩn.",
    "yolo.roi_x": "Vị trí cạnh trái vùng nhận diện, đơn vị pixel tính từ cạnh trái ảnh camera gốc. Nhập số nguyên không âm.",
    "yolo.roi_y": "Vị trí cạnh trên vùng nhận diện, đơn vị pixel tính từ cạnh trên ảnh camera gốc. Nhập số nguyên không âm.",
    "yolo.roi_width": "Chiều rộng vùng nhận diện, đơn vị pixel của ảnh camera gốc. Nhập số nguyên lớn hơn 0 và giữ vùng ROI nằm trong ảnh.",
    "yolo.roi_height": "Chiều cao vùng nhận diện, đơn vị pixel của ảnh camera gốc. Nhập số nguyên lớn hơn 0 và giữ vùng ROI nằm trong ảnh.",
    "sort_positions.pick_z_down": "Tọa độ Z khi hạ xuống gắp vật, đơn vị mm. Phải thấp hơn Approach height và nằm trong vùng robot có thể với tới.",
    "sort_positions.pick_z_up": "Tọa độ Z khi tiếp cận hoặc nâng vật sau khi gắp, đơn vị mm. Phải cao hơn Pick height; chọn độ cao đủ tránh các vật cản thực tế.",
    "place_good.z_down": "Tọa độ Z khi hạ xuống thả vật vào vị trí GOOD, đơn vị mm. Phải thấp hơn độ cao tiếp cận của vị trí GOOD đã cấu hình.",
    "place_bad.z_down": "Tọa độ Z khi hạ xuống thả vật vào vị trí BAD, đơn vị mm. Phải thấp hơn độ cao tiếp cận của vị trí BAD đã cấu hình.",
    "app.move_cooldown": "Thời gian chờ tối thiểu giữa các lượt phân loại tự động, đơn vị giây, từ 0 trở lên. Giá trị nhỏ không làm robot di chuyển nhanh hơn.",
    "app.plc_poll_interval": "Khoảng thời gian giữa hai lần đọc trạng thái PLC, đơn vị giây, phải lớn hơn 0. Ví dụ 0,10 tương đương khoảng 10 lần/giây. Giá trị nhỏ cập nhật nhanh hơn nhưng tăng số lần giao tiếp.",
    "app.sorting_mode": "vision: YOLO nhận diện và PC điều phối chu trình gắp/thả. qr: đọc mã QR và ghi mục tiêu cho PLC xử lý. Nhấn Apply & save settings để đổi chế độ.",
}

APPLY_HELP = "Kiểm tra toàn bộ giá trị, lưu vào config.yaml và áp dụng cho ứng dụng. Chờ lượt phân loại hoặc chu trình đang chạy kết thúc trước khi lưu. Ô số thập phân chấp nhận dấu phẩy hoặc dấu chấm."
CAMERA_MODE_HELP = "QR READER đọc mã trên toàn bộ ảnh. YOLO DETECTION nhận diện trong vùng ROI đã áp dụng ở Settings. Chế độ được chọn bằng Sorting Mode."
STATUS_HELP = "Tổng hợp kết nối PLC, cờ lỗi và lệnh Stop. OFFLINE nghĩa là không xác nhận được trạng thái. Xem Activity log để kiểm tra lệnh và thông báo gần nhất."
LOG_HELP = "Nhật ký các lệnh PLC và sự kiện ứng dụng gần nhất. Nhấn Show log để mở, Hide log để thu gọn; việc ẩn nhật ký không dừng ghi sự kiện."
