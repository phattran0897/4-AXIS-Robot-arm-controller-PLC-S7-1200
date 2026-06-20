# Deployment Guide — 4-Axis SCARA Robot Control System

## Mục lục
1. [Tổng quan](#tổng-quan)
2. [Yêu cầu hệ thống](#yêu-cầu-hệ-thống)
3. [Cài đặt](#cài-đặt)
4. [Cấu hình](#cấu-hình)
5. [Chạy ứng dụng](#chạy-ứng-dụng)
6. [Docker Deployment](#docker-deployment)
7. [Khắc phục sự cố](#khắc-phục-sự-cố)

---

## Tổng quan

Hệ thống điều khiển Robot SCARA 4 trục sử dụng:
- **AI Vision**: YOLOv11 cho phát hiện khuyết tật real-time
- **PLC**: Siemens S7-1200 giao tiếp qua SNAP7
- **GUI**: CustomTkinter với 2 chế độ Auto/Manual
- **Camera**: USB Camera OpenCV-compatible

---

## Yêu cầu hệ thống

### Phần cứng
| Thành phần | Yêu cầu |
|------------|----------|
| CPU | Intel Core i5+ hoặc tương đương |
| RAM | 8GB minimum (16GB khuyến nghị) |
| GPU | Optional — NVIDIA CUDA-capable cho tăng tốc AI |
| OS | Windows 10/11 hoặc Ubuntu 22.04+ |
| PLC | Siemens S7-1200 (Firmware >= V4.x) |
| Camera | USB Camera với driver hỗ trợ OpenCV |

### Phần mềm
| Package | Phiên bản |
|---------|-----------|
| Python | >= 3.11 |
| pip | Latest |

---

## Cài đặt

### Bước 1: Clone repository

```bash
git clone <repo-url>
cd robot_system
```

### Bước 2: Tạo Virtual Environment

```bash
# Windows
python -m venv .venv
.venv\Scripts\activate

# Linux/macOS
python3 -m venv .venv
source .venv/bin/activate
```

### Bước 3: Cài đặt Dependencies

```bash
pip install -r requirements.txt
```

### Bước 4: Cài đặt YOLO Model

Đặt file model `.pt` đã train vào thư mục `my_model/`:

```bash
mkdir -p my_model
# Copy your trained model as my_model/best.pt
```

---

## Cấu hình

### File cấu hình: `config.yaml`

#### 1. Cấu hình PLC

```yaml
plc:
  ip: "192.168.0.1"      # Đổi IP theo PLC thực tế
  rack: 0
  slot: 1
  db_number: 10
```

> **Lưu ý:** Đảm bảo PLC S7-1200 đã bật PUT/GET communication.

#### 2. Cấu hình YOLO

```yaml
yolo:
  model_path: "my_model/my_model.pt"  # Đường dẫn model
  thresh: 0.70                          # Ngưỡng detection (0.0-1.0)
  px2mm: 0.50                           # Hệ số chuyển đổi pixel sang mm
  home_x: 200.0                         # Vị trí home X (mm)
  home_y: 0.0                           # Vị trí home Y (mm)
```

#### 3. Cấu hình Camera

```yaml
camera:
  default_index: 0        # Index camera (0, 1, 2...)
  display_width: 440      # Chiều rộng hiển thị
  display_height: 310     # Chiều cao hiển thị
  fps: 30                 # Frame rate
```

#### 4. Cấu hình Kinematics

```yaml
kinematics:
  l1: 200.0    # Chiều dài link 1 (mm)
  l2: 150.0    # Chiều dài link 2 (mm)
```

#### 5. Cấu hình Application

```yaml
app:
  title: "4-AXIS SCARA Robot Control | VAA"
  geometry: "1200x750"
  appearance_mode: "dark"    # dark | light | system
  color_theme: "blue"
  plc_poll_interval: 0.10   # Thời gian poll PLC (giây)
  move_cooldown: 1.50        # Cooldown giữa các lệnh (giây)
```

---

## Chạy ứng dụng

### Chế độ thông thường

```bash
python main.py
```

### Chế độ GPU (nếu có NVIDIA GPU)

```bash
# Cài đặt PyTorch với CUDA trước
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu118

# Chạy
python main.py
```

### Kiểm tra Tests

```bash
# Chạy tất cả tests
pytest tests/ -v

# Với coverage report
pytest tests/ --cov=src --cov-report=term-missing -v
```

---

## Docker Deployment

### Build Docker Image

```bash
docker build -t robot-control-system .
```

### Chạy Container

```bash
# Linux (cần X11 forwarding)
docker run --rm -it \
  --device /dev/video0:/dev/video0 \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  robot-control-system
```

### Docker Compose

```yaml
services:
  robot-control:
    build: .
    devices:
      - /dev/video0:/dev/video0
    environment:
      - DISPLAY=${DISPLAY}
    volumes:
      - ./config.yaml:/app/config.yaml
      - ./my_model:/app/my_model
      - /tmp/.X11-unix:/tmp/.X11-unix
    network_mode: host
```

---

## Khắc phục sự cố

### Lỗi kết nối PLC

```
Error: Connection refused to 192.168.0.1
```

**Giải pháp:**
1. Kiểm tra IP PLC trong `config.yaml`
2. Ping thử PLC: `ping 192.168.0.1`
3. Đảm bảo cable mạng kết nối
4. Kiểm tra firewall không block port 102

### Lỗi Camera

```
Error: Unable to open camera
```

**Giải pháp:**
1. Kiểm tra camera có được nhận: `ls /dev/video*` (Linux)
2. Thử index khác trong `config.yaml`
3. Cài driver camera

### Lỗi YOLO Model

```
Error: Model file not found
```

**Giải pháp:**
1. Kiểm tra model file tồn tại: `ls my_model/`
2. Đúng định dạng `.pt`
3. Kiểm tra quyền đọc file

### Lỗi Import

```
ModuleNotFoundError: No module named 'snap7'
```

**Giải pháp:**
```bash
pip install python-snap7
```

### GUI không hiển thị (Docker)

```
_tkinter.TclError: no display name and no $DISPLAY environment variable
```

**Giải pháp:**
1. Linux: Set up X11 forwarding
2. Windows: Sử dụng VcXsrv/Xming
3. Hoặc chạy trực tiếp không qua Docker

---

## Network Topology

```
┌─────────────────┐      Ethernet       ┌─────────────────┐
│   PC (GUI)      │◄──────────────────►│  S7-1200 PLC    │
│                 │    192.168.0.x     │  192.168.0.1    │
│  ┌───────────┐  │                    │                 │
│  │  YOLO     │  │      USB           │  ┌───────────┐  │
│  │  Camera   │──┼────────────────────┼──│  Robot    │  │
│  └───────────┘  │   OpenCV           │  └───────────┘  │
└─────────────────┘                    └─────────────────┘
```

---

## Bảo mật

- **PLC Network**: Sử dụng VLAN riêng cho production
- **Firewall**: Chỉ mở port cần thiết
- **Model File**: Bảo vệ file `.pt` tránh truy cập trái phép
- **Config**: Không commit `config.yaml` với thông tin production

---

## License & Support

Hệ thống được phát triển bởi Vietnam Aviation Academy.
