# Tính toán Động học Kuka/Articulated Robot (Phương pháp D-H)
Quy trình 4 bước thiết lập phương trình động học dựa trên phương pháp Denavit-Hartenberg (D-H) cho cánh tay robot 4 bậc tự do.

---

## Bước 1: Gắn hệ tọa độ lên các khâu

**Giả định (Home Position):** Tất cả các khớp có góc lệch $\theta = 0^\circ$, cánh tay robot duỗi thẳng dọc theo trục X.

| Khâu | Trục $z_i$ (Trục quay) | Trục $x_i$ | Ghi chú |
|------|--------|--------|---------|
| **0 (Đế)** | $z_0$ hướng thẳng đứng lên trên ($\uparrow$) | $x_0$ hướng ra phía trước ($\rightarrow$) | Hệ tọa độ gốc thực tế |
| **1 (Base)** | $z_1$ hướng ra phía trước ($\rightarrow$) | $x_1$ hướng lên trên (theo quy tắc bàn tay phải) | Trục $z_0 \to z_1$ quay quanh $x_1$ 1 góc $\alpha_1 = -90^\circ$ |
| **2 (Shoulder)**| $z_2$ song song với $z_1$ | $x_2$ dọc theo thân khâu 2 | $z_1 \parallel z_2 \Rightarrow \alpha_2 = 0$ |
| **3 (Elbow)** | $z_3$ song song với $z_2$ | $x_3$ dọc theo thân khâu 3 | $z_2 \parallel z_3 \Rightarrow \alpha_3 = 0$ |
| **4 (Wrist)** | $z_4$ song song với $z_3$ | $x_4$ dọc theo thân khâu 4 | $z_3 \parallel z_4 \Rightarrow \alpha_4 = 0$ |

---

## Bước 2: Bảng thông số D-H

Từ các hệ tọa độ đã gắn, ta lập được bảng thông số D-H:

| i (Khâu) | Biến khớp $\theta_i$ | Lệch trục $d_i$ (mm) | Chiều dài khâu $a_i$ (mm) | Góc xoắn $\alpha_i$ (rad) |
|:---:|:---|:---|:---|:---|
| **1** | $\theta_1$ (biến xoay đế) | $d_1 = 300$ | $a_1 = 40$ | $-\pi/2$ |
| **2** | $\theta_2$ (biến xoay vai) | 0 | $a_2 = 190$ | 0 |
| **3** | $\theta_3$ (biến xoay khuỷu)| 0 | $a_3 = 110$ | 0 |
| **4** | $\theta_4$ (biến xoay cổ tay)| 0 | $a_4 = 65$ | 0 |

---

## Bước 3: Xác định các ma trận $A_n$

Công thức biến đổi tổng quát D-H:
$A_n = Rot(z, \theta) \cdot Trans(0, 0, d) \cdot Trans(a, 0, 0) \cdot Rot(x, \alpha)$

Cấu trúc ma trận D-H tổng quát:
$$
A_i = \begin{bmatrix} 
\cos\theta_i & -\sin\theta_i \cos\alpha_i & \sin\theta_i \sin\alpha_i & a_i \cos\theta_i \\
\sin\theta_i & \cos\theta_i \cos\alpha_i & -\cos\theta_i \sin\alpha_i & a_i \sin\theta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1 
\end{bmatrix}
$$

Áp dụng cho từng khâu:

**Ma trận $A_1$:** (với $\cos(-90^\circ)=0, \sin(-90^\circ)=-1$)
$$
A_1 = \begin{bmatrix} 
c_1 & 0 & -s_1 & 40c_1 \\
s_1 & 0 & c_1 & 40s_1 \\
0 & -1 & 0 & 300 \\
0 & 0 & 0 & 1 
\end{bmatrix}
$$

**Ma trận $A_2$:**
$$
A_2 = \begin{bmatrix} 
c_2 & -s_2 & 0 & 190c_2 \\
s_2 & c_2 & 0 & 190s_2 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 
\end{bmatrix}
$$

**Ma trận $A_3$:**
$$
A_3 = \begin{bmatrix} 
c_3 & -s_3 & 0 & 110c_3 \\
s_3 & c_3 & 0 & 110s_3 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 
\end{bmatrix}
$$

**Ma trận $A_4$:**
$$
A_4 = \begin{bmatrix} 
c_4 & -s_4 & 0 & 65c_4 \\
s_4 & c_4 & 0 & 65s_4 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 
\end{bmatrix}
$$
*(Ký hiệu thu gọn: $c_i = \cos\theta_i, s_i = \sin\theta_i$)*

---

## Bước 4: Phương trình Động học & Ma trận $T_{0}^4$

Nhân tuần tự các ma trận $A_1 \cdot A_2 \cdot A_3 \cdot A_4$ để tìm ma trận thế thái tổng quát $T_{0}^4$.

Nhóm 3 khâu phẳng ($A_2 \cdot A_3 \cdot A_4$):
$$
A_{234} = \begin{bmatrix} 
c_{234} & -s_{234} & 0 & 190c_2 + 110c_{23} + 65c_{234} \\
s_{234} & c_{234} & 0 & 190s_2 + 110s_{23} + 65s_{234} \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 
\end{bmatrix}
$$

Sau đó nhân với $A_1$, gọi $r$ là bán kính hình chiếu của tay máy xuống mặt phẳng XY:
$r = 40 + 190\cos\theta_2 + 110\cos(\theta_2+\theta_3) + 65\cos(\theta_2+\theta_3+\theta_4)$

Ta sẽ thu được tọa độ điểm tác động cuối (End-Effector) từ cột thứ 4 của ma trận $T_{0}^4$:

$$
\begin{cases}
P_x = \cos\theta_1 \cdot r \\
P_y = \sin\theta_1 \cdot r \\
P_z = d_1 - 190\sin\theta_2 - 110\sin(\theta_2+\theta_3) - 65\sin(\theta_2+\theta_3+\theta_4)
\end{cases}
$$

**Diễn giải chi tiết phương trình:**
* $P_x = \cos(\theta_1) \cdot [40 + 190\cos(\theta_2) + 110\cos(\theta_2+\theta_3) + 65\cos(\theta_2+\theta_3+\theta_4)]$
* $P_y = \sin(\theta_1) \cdot [40 + 190\cos(\theta_2) + 110\cos(\theta_2+\theta_3) + 65\cos(\theta_2+\theta_3+\theta_4)]$
* $P_z = 300 - 190\sin(\theta_2) - 110\sin(\theta_2+\theta_3) - 65\sin(\theta_2+\theta_3+\theta_4)$
