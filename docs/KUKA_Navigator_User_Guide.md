# KUKA Inspire Navigator — Hướng dẫn sử dụng chi tiết

> **Version:** 1.0 — Phiên bản đầy đủ cho System Integrator
> **Cập nhật:** 2026-02-16
> **Áp dụng cho:** KUKA Inspire Controller v1.x

---

## Mục lục

1. [Giới thiệu tổng quan](#1-giới-thiệu-tổng-quan)
2. [Giao diện Navigator](#2-giao-diện-navigator)
3. [Quản lý chương trình (CRUD)](#3-quản-lý-chương-trình-crud)
4. [Chọn & chạy chương trình (Satzanwahl)](#4-chọn--chạy-chương-trình-satzanwahl)
5. [Quản lý file nâng cao (Cut/Copy/Paste/Rename)](#5-quản-lý-file-nâng-cao-cutcopypasterename)
6. [Archive / Restore / Compare](#6-archive--restore--compare)
7. [Hệ thống FOLD trong chương trình](#7-hệ-thống-fold-trong-chương-trình)
8. [Use case thực tế cho System Integrator](#8-use-case-thực-tế-cho-system-integrator)
9. [Tips, Shortcuts & Troubleshooting](#9-tips-shortcuts--troubleshooting)

---

## 1. Giới thiệu tổng quan

### Navigator là gì?

Navigator là trình quản lý file **kiểu KUKA smartHMI** cho hệ thống KUKA Inspire Controller. Navigator cho phép:

- Duyệt toàn bộ cây thư mục workspace theo cấu trúc `KRC:\`
- Tạo, sửa, xóa, sao chép chương trình robot (`.src` + `.dat`)
- Chọn chương trình để chạy (**Satzanwahl**)
- Backup/Restore toàn bộ workspace
- So sánh workspace hiện tại với bản backup

### Cấu trúc workspace (KRC:\)

Navigator hiển thị workspace dưới dạng cây thư mục ảo `KRC:\`, tương tự KUKA KRC controller thật:

```
KRC:\
├─ R1\                         ← Robot 1
│  ├─ Program\                 ← Chương trình người dùng
│  │  ├─ WeldStation\          ← Thư mục con (nhóm theo trạm)
│  │  │  ├─ WeldSeam01         ← Module (.src + .dat)
│  │  │  └─ WeldSeam02
│  │  ├─ Maintenance\          ← Chương trình bảo trì
│  │  └─ PickPlace01
│  ├─ System\                  ← File hệ thống (CHỈ ĐỌC)
│  │  └─ $config.dat
│  └─ Mada\                    ← Machine Data (CHỈ ĐỌC)
│     ├─ $machine.dat
│     └─ $robcor.dat
├─ Config\                     ← Cấu hình controller
├─ Tools\                      ← Định nghĩa tool
├─ Frames\                     ← Reference frames
├─ Catalog\                    ← Robot catalog (templates)
├─ Station\                    ← Station/scene setup
└─ Log\                        ← Log hệ thống
```

**Lưu ý quan trọng:**
- Thư mục `System\` và `Mada\` là **chỉ đọc** — không thể tạo/xóa/sửa file bên trong
- Thư mục `Log\` và `Catalog\` **không được đưa vào** khi Archive All
- `R1\Program\` là thư mục mặc định khi mở Navigator

### Mở Navigator

**Cách 1:** Nhấn nút **KUKA logo** (góc trên bên trái) → Chọn **"Navigator"** trong Main Menu

**Cách 2:** Navigator là mục đầu tiên trong menu chính — nhấn một lần là mở ngay (không có submenu)

Khi Navigator active, giao diện chia thành **split-screen ngang**:
- Bên trái: Navigator (cây thư mục + danh sách file)
- Bên phải: 3D Viewport (thu nhỏ, vẫn render robot)

---

## 2. Giao diện Navigator

### 2.1 Bố cục tổng thể

```
┌──────────────────────────────────────────────────────────────────────────┐
│ [KUKA] │ Robot: KR10    Program: WeldSeam01  │ T1 │ 100% │ ...        │ ← TopStatusBar
├──────────────────────────────────────────────────────────────────────────┤
│ Message Banner                                                          │
├──────────────────────────────────┬───────────────────────────────────────┤
│ ┌─── Header ─────────────────┐   │                                       │
│ │[Module ▼] KRC:\R1\Prog\ 🔍│   │         3D VIEWPORT                   │
│ ├────────┬───────────────────┤   │                                       │
│ │ Tree   │ File List         │   │    Robot model vẫn render             │
│ │        │                   │   │    Grid + axes visible                │
│ │ 📂R1\  │ ⚙ WeldSeam01     │   │                                       │
│ │  📂Prog│ ⚙ PickPlace01    │   │                                       │
│ │  📁Sys │ 📁 WeldStation   │   │                                       │
│ │  📁Mada│                   │   │                                       │
│ ├────────┴───────────────────┤   │                                       │
│ │ Total: 3 objects           │   │                                       │
│ └────────────────────────────┘   │                                       │
├──────────────────────────────────┴───────────────────────────────────────┤
│ [SK1]   [SK2]   [SK3]   [SK4]   [SK5]   [SK6]   [Edit▲]               │ ← Softkeys
└──────────────────────────────────────────────────────────────────────────┘
```

### 2.2 Thanh Header (trên cùng, 28px)

| Thành phần | Vị trí | Chức năng |
|-----------|--------|-----------|
| **Filter Mode** | Trái, 100px | ComboBox chọn "Module" hoặc "Detail" |
| **Current Path** | Giữa | Hiển thị đường dẫn hiện tại: `KRC:\R1\Program\` |
| **Search Box** | Phải, 140px | Tìm kiếm real-time theo tên file |

### 2.3 Cây thư mục (bên trái, 200px)

- Hiển thị cấu trúc thư mục dạng tree
- 📁 = Thư mục đóng, 📂 = Thư mục mở
- Click vào node → Danh sách file bên phải cập nhật theo thư mục đó
- Có thể kéo splitter để thay đổi kích thước

### 2.4 Danh sách file (bên phải)

**Các cột hiển thị:**

| Cột | Nội dung | Sắp xếp |
|-----|---------|----------|
| **(Icon)** | Biểu tượng loại file (30px) | — |
| **Name** | Tên file/folder (200px) | A-Z / Z-A |
| **Type** | Phần mở rộng (60px) | A-Z / Z-A |
| **Comment** | Ghi chú từ `&COMMENT` (140px) | A-Z / Z-A |
| **Size** | Kích thước (60px) | Nhỏ→Lớn / Lớn→Nhỏ |
| **Changed** | Ngày sửa đổi (100px) | Cũ→Mới / Mới→Cũ |

**Biểu tượng file:**

| Icon | Ý nghĩa |
|------|---------|
| ⚙ | Module (cặp .src + .dat) |
| 📄 | Source file (.src riêng lẻ) |
| 📊 | Data file (.dat riêng lẻ) |
| 📁 | Thư mục |
| 🔒 | File chỉ đọc (trong System/ hoặc Mada/) |
| ▶ | Chương trình đã chọn (Satzanwahl) |
| ▶▶ | Chương trình đang chạy |
| ‖ | Chương trình tạm dừng |
| ✕ | Chương trình lỗi |

**Quy tắc hiển thị:**
- **Thư mục luôn hiển thị trước file** (không phụ thuộc cách sắp xếp)
- Click vào tiêu đề cột → Sắp xếp theo cột đó
- Click lần nữa → Đảo chiều sắp xếp

### 2.5 Thanh trạng thái (dưới cùng, 20px)

Hiển thị thông tin ngắn gọn:
- `Total: 12 objects` — Tổng số file/folder
- `Selected: 1 object    Total: 12` — Khi có file được chọn
- `Created: NewProgram` — Sau khi tạo mới
- `Deleted: OldProgram` — Sau khi xóa

### 2.6 Chế độ hiển thị: Module vs Detail

| Chế độ | Cách hiển thị | Khi nào dùng |
|--------|--------------|-------------|
| **Module** | Gom cặp `.src` + `.dat` thành 1 dòng duy nhất | Dùng hàng ngày — gọn, rõ ràng |
| **Detail** | Hiện mỗi file riêng biệt | Khi cần xem chi tiết từng file |

**Ví dụ: Thư mục chứa WeldSeam01.src + WeldSeam01.dat + README.txt**

Module mode:
```
⚙  WeldSeam01       module    Han moi 1          3.2K    02/16 10:30
📄 README.txt        .txt                         0.5K    02/15 08:00
```

Detail mode:
```
📄 WeldSeam01.src    .src      Han moi 1          1.8K    02/16 10:30
📊 WeldSeam01.dat    .dat                         1.4K    02/16 10:28
📄 README.txt        .txt                         0.5K    02/15 08:00
```

### 2.7 Tìm kiếm (Search)

- Gõ text vào ô **Search** ở góc phải header
- **Real-time**: Kết quả lọc ngay khi gõ (không cần nhấn Enter)
- **Case-insensitive**: "weld" tìm được "WeldSeam01", "WELD_TEST", "weld_partA"
- **Substring match**: "Seam" tìm được "WeldSeam01", "WeldSeam02"
- Xóa text search → Hiện lại toàn bộ file

### 2.8 Menu ngữ cảnh (Right-click)

Click chuột phải vào file/folder trong danh sách → Hiện menu:

```
┌──────────────────────┐
│ Open                 │
│ Select               │
├──────────────────────┤
│ New    ▸  Module     │
│           Folder     │
├──────────────────────┤
│ Cut                  │
│ Copy                 │
│ Paste                │
├──────────────────────┤
│ Rename               │
│ Delete               │
├──────────────────────┤
│ Cancel Select        │
├──────────────────────┤
│ Archive  ▸  All      │
│             Selected │
│ Restore              │
│ Compare with Archive │
├──────────────────────┤
│ Properties           │
└──────────────────────┘
```

### 2.9 Thanh Softkey (dưới cùng màn hình, 72px)

Khi Navigator active, softkey 7 (Edit) mở popup menu phụ:

```
┌──────────────────────────┐
│ ✂  Cut                   │
│ ⌘  Copy                  │
│ ⌙  Paste                 │
│ ─────────────────────── │
│ ✎  Rename                │
│ ⊧  Filter                │
│ ─────────────────────── │
│ 📦 Archive All            │
│ 📥 Archive Selected       │
│ 📤 Restore                │
│ 🔍 Compare                │
└──────────────────────────┘
```

---

## 3. Quản lý chương trình (CRUD)

### 3.1 Tạo chương trình mới (New Module)

**Bước 1:** Click chuột phải → **New** → **Module** (hoặc qua Softkey Edit → ...)

**Bước 2:** Hộp thoại "New Module" xuất hiện:

```
┌──── New Module ──────────────────┐
│                                   │
│  Name:     [________________]     │
│  Comment:  [________________]     │
│                                   │
│              [  OK  ] [Cancel]    │
└───────────────────────────────────┘
```

| Trường | Bắt buộc | Quy tắc |
|--------|---------|---------|
| **Name** | Có | Không chứa ký tự đặc biệt; không cần gõ `.src`; tên phải unique trong folder |
| **Comment** | Không | Mô tả ngắn — sẽ hiện ở cột Comment trong danh sách file |

**Bước 3:** Nhấn **OK** → Hệ thống tự tạo 2 file:

**File `.src` (source code):**
```krl
&ACCESS RV
&REL 1
&COMMENT <comment bạn nhập>

DEF <TenChuongTrinh>()
  ;FOLD INI
    ;FOLD BASISTECH INI
      BAS(#INITMOV, 0)
    ;ENDFOLD
  ;ENDFOLD

  ;FOLD PTP HOME Vel=100% DEFAULT
    PTP HOME Vel=100% DEFAULT
  ;ENDFOLD

END
```

**File `.dat` (data list):**
```krl
DEFDAT <TenChuongTrinh> PUBLIC

  ;FOLD EXTERNAL DECLARATIONS; %{PE}
  ;ENDFOLD

  DECL E6POS HOME={A1 0.0, A2 -90.0, A3 90.0, A4 0.0, A5 0.0, A6 0.0}

ENDDAT
```

### 3.2 Tạo thư mục mới (New Folder)

**Bước 1:** Click chuột phải → **New** → **Folder**

**Kết quả:** Tạo thư mục tên `NewFolder` (nếu đã tồn tại → `NewFolder1`, `NewFolder2`, ...)

Sau đó có thể **Rename** để đặt tên rõ ràng hơn.

### 3.3 Mở chương trình (Open)

**Cách 1:** Double-click vào module trong danh sách file

**Cách 2:** Click chuột phải → **Open**

**Hành vi:**
- Nếu là **folder** → Navigator đi vào thư mục đó (cập nhật cây + danh sách file)
- Nếu là **module/file** → Mở trong **Program Editor** (chuyển sang tab chỉnh sửa code)

### 3.4 Xóa chương trình (Delete)

**Bước 1:** Chọn file/folder → Click chuột phải → **Delete**

**Bước 2:** Hộp thoại xác nhận:

```
┌──── Confirm Delete ──────────────┐
│                                   │
│  Delete "WeldSeam01"?             │
│                                   │
│              [Delete] [Cancel]    │
└───────────────────────────────────┘
```

**Lưu ý quan trọng:**
- Xóa module → **Tự động xóa cả `.src` lẫn `.dat`** (composite delete)
- Xóa folder → Xóa toàn bộ nội dung bên trong
- **Không thể xóa** file trong `System\` hoặc `Mada\` (hiện thông báo read-only)
- Xóa là **vĩnh viễn** — không có thùng rác. Dùng Archive để backup trước khi xóa

### 3.5 Xem thuộc tính (Properties)

Click chuột phải → **Properties** → Hiện hộp thoại chi tiết:

```
┌──── Properties ──────────────────────┐
│                                       │
│  Name:      WeldSeam01                │
│  Type:      Module                    │
│  Size:      3,248 bytes (3.2K)        │
│  Path:      KRC:\R1\Program\          │
│  Created:   2026-02-10 14:30:00       │
│  Modified:  2026-02-16 10:30:45       │
│  Comment:   Han moi dau tien          │
│                                       │
│                            [Close]    │
└───────────────────────────────────────┘
```

---

## 4. Chọn & chạy chương trình (Satzanwahl)

### Satzanwahl là gì?

**Satzanwahl** (tiếng Đức: "Anwahl" = chọn, "Satz" = câu lệnh) là thao tác **chọn chương trình để chạy**. Trên KUKA thật, robot chỉ chạy được chương trình đã được "Select" (Anwahl).

### 4.1 Chọn chương trình (Select)

**Bước 1:** Trong Navigator, chọn module cần chạy

**Bước 2:** Click chuột phải → **Select**

**Kết quả:**
- Module được đánh dấu bằng biểu tượng **▶** (tam giác xanh/đỏ)
- Dòng module chuyển nền **cam nhạt** (#FFF3E0) với **viền trái cam** (#EB6A0A)
- Text chuyển sang **màu đỏ** (#CC0000) và in đậm
- TopStatusBar cập nhật: `Program: WeldSeam01`

```
Trước Select:
  ⚙  WeldSeam01       module    Han moi 1          3.2K

Sau Select:
  ▶  WeldSeam01       module    Han moi 1          3.2K    ← nền cam, text đỏ
```

### 4.2 Trạng thái chương trình

Sau khi Select, chương trình có thể ở các trạng thái:

| Trạng thái | Icon | Ý nghĩa |
|-----------|------|---------|
| **Selected** | ▶ | Đã chọn, sẵn sàng chạy |
| **Running** | ▶▶ | Đang thực thi lệnh motion |
| **Paused** | ‖ | Tạm dừng (nhả Start button) |
| **Error** | ✕ | Lỗi runtime (ví dụ: collision, limit) |

Icon và trạng thái cập nhật **tự động** trong Navigator khi chương trình thay đổi trạng thái.

### 4.3 Hủy chọn (Cancel Select)

Click chuột phải → **Cancel Select**

- Icon trở về ⚙ (module bình thường)
- Nền, text trở về mặc định
- TopStatusBar: `Program: -/-`

### 4.4 Workflow đầy đủ: Từ viết code đến chạy robot

```
1. Navigator → New → Module → Đặt tên "WeldSeam01"
2. Double-click "WeldSeam01" → Program Editor mở
3. Viết code KRL (PTP, LIN, CIRC, I/O...)
4. Nhấn Save → File lưu xuống disk
5. Quay lại Navigator
6. Right-click "WeldSeam01" → Select → Icon chuyển ▶
7. Chuyển sang chế độ T1 (test mode)
8. Nhấn Start → Robot bắt đầu di chuyển → Icon: ▶▶
9. Nhả Start → Robot dừng → Icon: ‖
10. Chạy xong → Icon: ▶ (quay về Selected)
```

---

## 5. Quản lý file nâng cao (Cut/Copy/Paste/Rename)

### 5.1 Copy (Sao chép)

**Bước 1:** Chọn file/folder → Right-click → **Copy** (hoặc Softkey Edit → Copy)

**Bước 2:** Đi đến thư mục đích → Right-click → **Paste**

**Kết quả:**
- File được sao chép vào thư mục đích
- Nếu là module → **cả `.src` lẫn `.dat` đều được copy** (composite copy)
- File gốc **không bị xóa**

### 5.2 Cut (Cắt / Di chuyển)

**Bước 1:** Chọn file/folder → Right-click → **Cut** (hoặc Softkey Edit → Cut)

**Bước 2:** Đi đến thư mục đích → Right-click → **Paste**

**Kết quả:**
- File di chuyển sang thư mục mới
- Nếu là module → **cả `.src` lẫn `.dat` đều được di chuyển**
- File gốc **bị xóa** khỏi vị trí cũ

### 5.3 Rename (Đổi tên)

**Bước 1:** Chọn file/folder → Right-click → **Rename** (hoặc Softkey Edit → Rename)

**Bước 2:** Hộp thoại Rename xuất hiện:

```
┌──── Rename ──────────────────────┐
│                                   │
│  New name:  [WeldSeam01______]    │   ← Pre-filled tên cũ (không có .src)
│                                   │
│              [  OK  ] [Cancel]    │
└───────────────────────────────────┘
```

**Bước 3:** Gõ tên mới → OK

**Kết quả:**
- Module rename → **Đổi tên cả `.src` lẫn `.dat`** (composite rename)
- VD: Rename "WeldSeam01" → "WeldSeam01_v2" → cả `WeldSeam01_v2.src` + `WeldSeam01_v2.dat`

### 5.4 Hạn chế với file Read-Only

Các thao tác **Cut, Delete, Rename** bị **chặn** đối với file trong thư mục `System\` và `Mada\`:
- File hiển thị icon 🔒
- Khi thử thao tác → Status bar hiện thông báo lỗi

Các thao tác vẫn được phép:
- **Copy** → Có thể copy file read-only ra folder khác
- **Properties** → Có thể xem thông tin
- **Archive** → Có thể backup

---

## 6. Archive / Restore / Compare

### 6.1 Archive All (Backup toàn bộ workspace)

**Mục đích:** Tạo bản backup ZIP của toàn bộ workspace

**Bước 1:** Softkey Edit → **Archive All** (hoặc Right-click → Archive → All)

**Bước 2:** Hộp thoại Save File xuất hiện → Chọn vị trí lưu file `.zip`

**Kết quả:**
- Tạo file ZIP chứa toàn bộ workspace
- **Ngoại trừ:** `Log\` và `Catalog\` (không archive)
- File ZIP chứa thêm `_archive_manifest.json`:

```json
{
  "Timestamp": "2026-02-16T10:30:45Z",
  "Version": "1.0",
  "FileCount": 42,
  "Source": "E:\\path\\to\\workspace"
}
```

### 6.2 Archive Selected (Backup file/folder đã chọn)

**Bước 1:** Chọn file hoặc folder cần backup

**Bước 2:** Softkey Edit → **Archive Selected** (hoặc Right-click → Archive → Selected)

**Bước 3:** Chọn vị trí lưu file `.zip`

**Kết quả:**
- Chỉ archive file/folder được chọn
- Nếu chọn module `.src` → **tự động kèm `.dat`** (composite archive)
- Giữ nguyên cấu trúc thư mục trong ZIP

### 6.3 Restore (Khôi phục từ backup)

**Bước 1:** Right-click → **Restore** (hoặc Softkey Edit → Restore)

**Bước 2:** Hộp thoại Open File → Chọn file `.zip` backup

**Bước 3:** Hộp thoại Restore xuất hiện:

```
┌──── Restore from Archive ─────────────────────────────────┐
│                                                             │
│  42 files in archive                                        │
│                                                             │
│  ┌───┬─────────────────────────────────────┬────────┐       │
│  │ ☑ │ File                                │ Size   │       │
│  ├───┼─────────────────────────────────────┼────────┤       │
│  │ ☑ │ KRC:\R1\Program\WeldSeam01.src      │ 1.8K   │       │
│  │ ☑ │ KRC:\R1\Program\WeldSeam01.dat      │ 1.4K   │       │
│  │ ☑ │ KRC:\R1\Program\PickPlace01.src     │ 2.1K   │       │
│  │ ☑ │ KRC:\R1\System\$config.dat          │ 3.5K   │       │
│  │ ...                                              │       │
│  └──────────────────────────────────────────────────┘       │
│                                                             │
│                         [Restore Selected]  [Cancel]        │
└─────────────────────────────────────────────────────────────┘
```

**Bước 4:** Bỏ chọn các file không muốn restore → Nhấn **Restore Selected**

**Xử lý xung đột (Conflict Resolution):**

| Chế độ | Hành vi |
|--------|---------|
| **Overwrite** (mặc định) | Ghi đè file hiện tại |
| **Skip** | Bỏ qua file đã tồn tại |
| **Rename** | Tạo bản mới với hậu tố `_restored1`, `_restored2`... |

### 6.4 Compare with Archive (So sánh workspace với backup)

**Bước 1:** Right-click → **Compare with Archive** (hoặc Softkey Edit → Compare)

**Bước 2:** Chọn file `.zip` backup cần so sánh

**Bước 3:** Hộp thoại Compare hiển thị:

```
┌──── Compare: Archive vs Workspace ────────────────────────────────────┐
│                                                                        │
│  ┌──────────┬────────────────────────────────────┬─────────┬─────────┐ │
│  │ Status   │ File                               │ Archive │ Local   │ │
│  ├──────────┼────────────────────────────────────┼─────────┼─────────┤ │
│  │ Identical│ KRC:\R1\Program\WeldSeam01.src     │ 1.8K    │ 1.8K    │ │
│  │ Different│ KRC:\R1\Program\PickPlace01.src    │ 2.1K    │ 2.4K    │ │
│  │ OnlyArch │ KRC:\R1\Program\OldProgram.src     │ 1.2K    │   -     │ │
│  │ OnlyLocal│ KRC:\R1\Program\NewProgram.src     │   -     │ 1.5K    │ │
│  └──────────┴────────────────────────────────────┴─────────┴─────────┘ │
│                                                                        │
│                                                            [Close]     │
└────────────────────────────────────────────────────────────────────────┘
```

**Các trạng thái so sánh:**

| Status | Màu | Ý nghĩa |
|--------|-----|---------|
| **Identical** | 🟢 Xanh lá | File giống nhau (cùng size + date ± 2 giây) |
| **Different** | 🟠 Cam | File tồn tại ở cả 2 nhưng khác nhau |
| **OnlyInArchive** | 🔵 Xanh dương | File chỉ có trong backup (đã bị xóa ở local) |
| **OnlyLocal** | 🔴 Đỏ | File mới tạo sau khi backup |

---

## 7. Hệ thống FOLD trong chương trình

### FOLD là gì?

FOLD (code folding) cho phép **gom nhóm và ẩn/hiện** các đoạn code trong Program Editor. Tương tự region trong C# hoặc #pragma region trong C++.

### Các loại FOLD được hỗ trợ

| Loại | Mở | Đóng | Ví dụ |
|------|-----|------|-------|
| **KUKA FOLD** | `;FOLD <tên>` | `;ENDFOLD` | `;FOLD INI` ... `;ENDFOLD` |
| **Region** | `;#REGION <tên>` | `;#ENDREGION` | `;#REGION Approach` ... `;#ENDREGION` |
| **ArcStart/ArcEnd** | `ArcStart(...)` | `ArcEnd` | Nhóm lệnh hàn |
| **DEF/END** | `DEF <tên>()` | `END` | Routine/subroutine |

### FOLD lồng nhau (Nested)

```krl
;FOLD INI                          ← FOLD cấp 1
  ;FOLD BASISTECH INI              ← FOLD cấp 2 (lồng trong)
    BAS(#INITMOV, 0)
  ;ENDFOLD                         ← Đóng cấp 2
  BAS(#TOOL, 1)
;ENDFOLD                           ← Đóng cấp 1
```

### Tính năng Seam Length (chiều dài đường hàn)

Với các nhóm ArcStart/ArcEnd, hệ thống tự động tính **chiều dài đường hàn**:

- Chỉ tính lệnh **LIN** (di chuyển tuyến tính)
- **PTP** bị bỏ qua (di chuyển điểm-điểm, không phải đường hàn)
- Tọa độ lấy từ file `.dat` (E6POS declarations)
- Công thức: √((x2-x1)² + (y2-y1)² + (z2-z1)²)

**Ví dụ:**
```krl
;; Trong .src:
ArcStart(Job_ID:=1)
  LIN P1              ← Điểm bắt đầu
  LIN P2              ← Khoảng cách P1→P2 = 300mm
  LIN P3              ← Khoảng cách P2→P3 = 300mm
ArcEnd
;; Seam Length = 600.0 mm

;; Trong .dat:
DECL E6POS P1={X 500.0, Y 0.0, Z 300.0, ...}
DECL E6POS P2={X 500.0, Y 300.0, Z 300.0, ...}
DECL E6POS P3={X 500.0, Y 600.0, Z 300.0, ...}
```

---

## 8. Use case thực tế cho System Integrator

### Use Case 1: Thiết lập trạm hàn mới

**Tình huống:** Bạn vừa lắp đặt robot KUKA cho trạm hàn MIG/MAG. Cần tạo cấu trúc chương trình cho dây chuyền.

**Quy trình:**

1. **Tổ chức thư mục theo trạm:**
   ```
   Navigator → R1\Program\ → Right-click → New → Folder → "WeldStation1"
   Navigator → R1\Program\ → Right-click → New → Folder → "WeldStation2"
   Navigator → R1\Program\ → Right-click → New → Folder → "Maintenance"
   ```

2. **Tạo chương trình hàn cho mỗi seam:**
   ```
   Vào WeldStation1\ → Right-click → New → Module
   Name: "Seam01_ButtWeld"
   Comment: "Han moi dau tien - day chuyen A"
   ```

3. **Viết code trong Program Editor:**
   ```krl
   DEF Seam01_ButtWeld()
     ;FOLD INI
       BAS(#INITMOV, 0)
       BAS(#TOOL, 1)       ; Sung han MIG
       BAS(#BASE, 2)       ; Ban ga ke
     ;ENDFOLD

     PTP HOME Vel=100% DEFAULT

     ;FOLD LIN P1 Vel=2m/s — Di den diem bat dau
       LIN P1 Vel=2m/s CPDAT1 TOOL[1] BASE[2]
     ;ENDFOLD

     ArcStart(Job_ID:=1)
       LIN P2 Vel=10mm/s    ; Han duong thang
       LIN P3 Vel=10mm/s    ; Tiep tuc han
     ArcEnd

     PTP HOME Vel=100% DEFAULT
   END
   ```

4. **Test chương trình:**
   ```
   Navigator → Right-click "Seam01_ButtWeld" → Select
   Chuyển T1 mode → Nhấn Start → Robot di chuyển từng bước
   ```

5. **Backup trước khi chạy production:**
   ```
   Softkey Edit → Archive All → Lưu "WeldStation1_backup_20260216.zip"
   ```

---

### Use Case 2: Clone chương trình sang trạm khác

**Tình huống:** Trạm hàn 2 giống trạm 1 nhưng offset vị trí 1000mm theo trục Y.

**Quy trình:**

1. **Copy toàn bộ folder:**
   ```
   Navigator → Right-click "WeldStation1" → Copy
   Quay lại R1\Program\ → Right-click → Paste
   ```
   → Tạo ra folder `WeldStation1` copy

2. **Rename:**
   ```
   Right-click "WeldStation1 (copy)" → Rename → "WeldStation2"
   ```

3. **Mở từng module → sửa tọa độ trong `.dat`:**
   - Double-click để mở Program Editor
   - Cập nhật Y offset cho tất cả E6POS points

---

### Use Case 3: Bảo trì & khắc phục sự cố

**Tình huống:** Robot dừng đột ngột khi chạy chương trình "Seam03". Cần kiểm tra và sửa lỗi.

**Quy trình:**

1. **Kiểm tra trạng thái trong Navigator:**
   ```
   Mở Navigator → Nhìn icon:
   ✕ Seam03   ← Chương trình đang ở trạng thái Error
   ```

2. **Cancel Select để dừng hoàn toàn:**
   ```
   Right-click "Seam03" → Cancel Select
   ```

3. **Backup trước khi sửa:**
   ```
   Right-click "Seam03" → Archive → Selected
   → Lưu "Seam03_error_backup.zip"
   ```

4. **Mở và sửa code:**
   ```
   Double-click "Seam03" → Program Editor
   → Kiểm tra lệnh motion gần chỗ lỗi
   → Sửa tọa độ nếu bị collision/limit
   ```

5. **So sánh với bản gốc (nếu cần):**
   ```
   Right-click → Compare with Archive → Chọn file backup trước đó
   → Xem file nào bị Different → Xác nhận thay đổi
   ```

6. **Test lại:**
   ```
   Right-click "Seam03" → Select → Chuyển T1 → Start
   ```

---

### Use Case 4: Quản lý chương trình bảo trì định kỳ

**Tình huống:** Cần tạo bộ chương trình bảo trì mà operator có thể chạy hàng ngày/tuần.

**Quy trình:**

1. **Tạo folder Maintenance:**
   ```
   R1\Program\ → New → Folder → "Maintenance"
   ```

2. **Tạo chương trình Zero Position (kiểm tra vị trí 0):**
   ```
   Vào Maintenance\ → New → Module
   Name: "Zero_Position"
   Comment: "Kiem tra vi tri zero"
   ```

   Code:
   ```krl
   DEF Zero_Position()
     ;FOLD INI
       BAS(#INITMOV, 0)
     ;ENDFOLD

     ; Di chuyen tat ca truc ve 0 do
     PTP {A1 0, A2 0, A3 0, A4 0, A5 0, A6 0} Vel=10% DEFAULT

     HALT      ; Dung de operator kiem tra

     PTP HOME Vel=100% DEFAULT
   END
   ```

3. **Tạo chương trình Thay Mỡ (greasing):**
   ```
   Name: "Thay_Mo"
   Comment: "Chay smooth de bom mo"
   ```

4. **Hướng dẫn operator:**
   - Mở Navigator → Vào `Maintenance\`
   - Select chương trình cần chạy
   - Chuyển T1 → Start
   - Khi thấy HALT → Robot dừng → Kiểm tra → Nhấn Start để tiếp

---

### Use Case 5: Handover dự án cho khách hàng

**Tình huống:** Dự án hoàn thành, cần giao bàn cho khách hàng với đầy đủ chương trình và backup.

**Quy trình:**

1. **Kiểm tra cấu trúc workspace:**
   ```
   Navigator → Duyệt từng folder → Đảm bảo:
   ✓ Tất cả chương trình có comment rõ ràng
   ✓ Folder tổ chức theo trạm/chức năng
   ✓ Không có file rác/test thừa
   ```

2. **Xóa file không cần thiết:**
   ```
   Right-click file test → Delete → Confirm
   ```

3. **Archive toàn bộ workspace:**
   ```
   Softkey Edit → Archive All
   → Lưu: "ProjectName_Final_20260216.zip"
   ```

4. **Tạo bản backup thứ 2 vào USB:**
   ```
   Lặp lại Archive All → Chọn đường dẫn USB
   ```

5. **Verify backup:**
   ```
   Softkey Edit → Compare
   → Chọn file ZIP vừa tạo
   → Tất cả phải Identical (xanh lá)
   ```

---

### Use Case 6: Khôi phục sau sự cố (Disaster Recovery)

**Tình huống:** Controller bị reset hoặc file bị hỏng, cần khôi phục từ backup.

**Quy trình:**

1. **Mở Navigator → Restore:**
   ```
   Right-click → Restore → Chọn file backup .zip
   ```

2. **Kiểm tra danh sách file trong archive:**
   ```
   Hộp thoại Restore hiện:
   - 42 files in archive
   - Tất cả được chọn mặc định (☑)
   ```

3. **Chọn cách xử lý xung đột:**
   - **Overwrite**: Ghi đè tất cả (phổ biến nhất khi recovery)
   - **Skip**: Giữ file local, chỉ restore file thiếu
   - **Rename**: Giữ cả 2 bản (có hậu tố `_restored1`)

4. **Nhấn Restore Selected → Đợi hoàn tất**

5. **Verify:**
   ```
   Compare with Archive → Chọn cùng file ZIP
   → Tất cả phải Identical
   ```

---

### Use Case 7: Tìm kiếm chương trình nhanh

**Tình huống:** Workspace có 50+ chương trình, cần tìm nhanh chương trình hàn cho sản phẩm cụ thể.

**Quy trình:**

1. **Dùng Search box:**
   ```
   Gõ "Wave" → Kết quả lọc ngay:
   ⚙ Weld_PartA       Han suon xe Wave Alpha
   ⚙ Weld_PartB       Han co xe Wave Alpha
   ```

2. **Dùng Module filter để xem gọn:**
   ```
   Filter: Module → Mỗi cặp .src+.dat chỉ hiện 1 dòng
   ```

3. **Dùng Sort theo Comment:**
   ```
   Click cột "Comment" → Sắp xếp A-Z
   → Dễ dàng thấy nhóm chương trình cùng loại
   ```

---

## 9. Tips, Shortcuts & Troubleshooting

### Tips hữu ích

| Tip | Chi tiết |
|-----|---------|
| **Luôn backup trước khi sửa** | Dùng Archive Selected cho file cụ thể, Archive All cho toàn bộ |
| **Đặt tên có ý nghĩa** | `WeldSeam01_ButtWeld` thay vì `Program1` |
| **Dùng Comment** | Mô tả rõ mục đích trong &COMMENT khi tạo module mới |
| **Module mode mặc định** | Dùng Module filter hàng ngày, Detail chỉ khi cần debug |
| **Tổ chức folder theo trạm** | `WeldStation1\`, `WeldStation2\`, `Maintenance\`, `Test\` |
| **Kiểm tra icon** | ▶ = đã Select, ▶▶ = đang chạy, ✕ = lỗi |
| **Composite operations** | Rename/Delete/Copy module → tự động xử lý cả .src lẫn .dat |

### Shortcuts & tương tác nhanh

| Hành động | Cách thực hiện |
|-----------|---------------|
| Mở chương trình | Double-click |
| Vào thư mục | Double-click folder |
| Menu ngữ cảnh | Right-click |
| Sắp xếp | Click tiêu đề cột |
| Đảo chiều sort | Click lại tiêu đề cột |
| Tìm kiếm | Gõ vào ô Search |
| Chuyển filter | ComboBox ở header |
| Edit menu (phụ) | Softkey 7 (Edit) |

### Troubleshooting

#### File không hiển thị trong Navigator
**Nguyên nhân:** Filter đang ở chế độ Module và file không phải `.src`/`.dat`

**Giải pháp:** Chuyển sang **Detail** mode để xem tất cả file

---

#### Không thể xóa/rename file
**Nguyên nhân:** File nằm trong thư mục `System\` hoặc `Mada\` (read-only)

**Giải pháp:** File hệ thống không được phép sửa/xóa. Nếu cần chỉnh sửa, hãy **Copy** ra `Program\` trước

---

#### File mới tạo bên ngoài không hiện
**Nguyên nhân:** FileSystemWatcher có debounce 500ms

**Giải pháp:** Đợi 1-2 giây hoặc click vào thư mục khác rồi quay lại để refresh

---

#### Archive không chứa Log/ và Catalog/
**Đây là hành vi bình thường.** `Log\` và `Catalog\` bị loại trừ khỏi Archive All để giảm kích thước backup. Chúng không phải dữ liệu user cần backup.

---

#### So sánh (Compare) hiện "Different" nhưng file giống nhau
**Nguyên nhân:** Ngày sửa đổi khác nhau > 2 giây

**Giải pháp:** So sánh chỉ dựa trên size + date. Nếu file giống nội dung nhưng date khác (do copy) → hiện Different. Đây là hành vi đúng.

---

#### Chương trình Select nhưng không chạy được
**Kiểm tra:**
1. Đã chuyển đúng mode (T1 cho test, Auto cho production)?
2. Chương trình có lỗi syntax? → Mở Program Editor kiểm tra
3. Robot đã enable drives chưa?
4. Có emergency stop đang active?

---

## Phụ lục: Bảng tham chiếu nhanh

### A. Các loại file trong workspace

| Extension | Loại | Ý nghĩa |
|-----------|------|---------|
| `.src` | Source | Code chương trình KRL |
| `.dat` | Data | Dữ liệu điểm (E6POS), biến |
| `$config.dat` | System | Cấu hình hệ thống (TOOL_DATA, BASE_DATA) |
| `$machine.dat` | Machine | Dữ liệu máy (soft limits, gear ratios) |
| `$robcor.dat` | Correction | Dữ liệu hiệu chỉnh robot |
| `.yaml` | Config | File cấu hình (network, safety, tools) |
| `.json` | Station | Cấu hình station/scene |

### B. Cấu trúc file `.src` (KRL)

```krl
&ACCESS RV                    ← Quyền truy cập (Read/Visible)
&REL 1                        ← Release level
&COMMENT <mô tả>              ← Comment hiển thị trong Navigator

DEF <TenChuongTrinh>()        ← Khai báo routine chính
  ;FOLD INI                   ← Khối khởi tạo (luôn có)
    BAS(#INITMOV, 0)
  ;ENDFOLD

  ; Các lệnh motion ở đây
  PTP HOME Vel=100% DEFAULT   ← Di chuyển kiểu point-to-point
  LIN P1 Vel=2m/s             ← Di chuyển tuyến tính
  CIRC P2, P3                 ← Di chuyển cung tròn

END                           ← Kết thúc routine
```

### C. Cấu trúc file `.dat` (Data List)

```krl
DEFDAT <TenChuongTrinh> PUBLIC    ← Khai báo data list

  DECL E6POS HOME={A1 0.0, A2 -90.0, A3 90.0, A4 0.0, A5 0.0, A6 0.0}
  DECL E6POS P1={X 500.0, Y 200.0, Z 300.0, A 0.0, B 90.0, C 0.0, S 2, T 10}

  DECL REAL weld_speed=10.0       ← Biến số thực
  DECL INT weld_mode=1            ← Biến số nguyên

ENDDAT                            ← Kết thúc data list
```

### D. Lệnh motion phổ biến

| Lệnh | Loại | Mô tả |
|-------|------|-------|
| `PTP` | Point-to-Point | Di chuyển nhanh nhất, đường đi không xác định |
| `LIN` | Linear | Di chuyển tuyến tính (đường thẳng) |
| `CIRC` | Circular | Di chuyển cung tròn qua 2 điểm |
| `PTP HOME` | Home | Về vị trí Home đã định nghĩa |

---

*Tài liệu này dành cho System Integrator sử dụng KUKA Inspire Controller. Mọi thao tác nên được test ở chế độ T1 (manual) trước khi chạy Auto.*
