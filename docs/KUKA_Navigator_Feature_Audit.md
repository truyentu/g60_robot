# KUKA Navigator — Feature Reference & Implementation Audit

**Date**: 2026-02-16
**Purpose**: Tài liệu chi tiết về KUKA Navigator, workflow thực tế, và so sánh với implementation hiện tại trong project G60 Robot Controller.

---

## Mục lục

1. [Tổng quan KUKA Navigator](#1-tổng-quan-kuka-navigator)
2. [Cấu trúc thư mục ảo KRC:\](#2-cấu-trúc-thư-mục-ảo-krc)
3. [Workflow thực tế của System Integrator](#3-workflow-thực-tế-của-system-integrator)
4. [Tất cả Features của KUKA Navigator](#4-tất-cả-features-của-kuka-navigator)
5. [Phân quyền User Group](#5-phân-quyền-user-group)
6. [Audit Implementation hiện tại](#6-audit-implementation-hiện-tại)
7. [Feature Gap Analysis](#7-feature-gap-analysis)
8. [Roadmap đề xuất](#8-roadmap-đề-xuất)
9. [Manual Test Cases](#9-manual-test-cases)

---

## 1. Tổng quan KUKA Navigator

### 1.1 Navigator là gì?

KUKA Navigator là **file manager tích hợp** trên smartHMI (Smart Human-Machine Interface), chạy trên smartPAD (teach pendant). Nó là giao diện chính để:

- Quản lý chương trình robot (.src/.dat)
- Tổ chức thư mục dự án
- Chọn chương trình để chạy (Satzanwahl/Program Selection)
- Truy cập cấu hình hệ thống
- Archive/Restore dự án

### 1.2 Kiến trúc

Navigator hiển thị hệ thống file thông qua **ổ ảo KRC:\**, ánh xạ từ `C:\KRC\Roboter\KRC\` trên ổ cứng controller KRC4. Người dùng **không bao giờ thấy ổ C:** trực tiếp — chỉ thấy cấu trúc ảo đã được lọc và bảo vệ.

### 1.3 Layout UI

```
┌─────────────────────────────────────────────────────┐
│ [Filter: Module ▼]  Path: KRC:\R1\Program\          │ ← Header bar
├──────────────┬──────────────────────────────────────┤
│ Directory    │ Name    │ Type  │ Comment  │ Size    │ ← Column headers
│ Tree         │─────────┼───────┼──────────┼─────────│
│              │ ⚙ Cell  │module │ Main PLC │ 4.2 KB  │
│ ▼ KRC:\      │ ⚙ Pick  │module │ Pick A   │ 2.1 KB  │
│   ▼ R1\      │ 📁 Weld │folder │          │         │
│     ▼Program\│ ⚙ Home  │module │ Home pos │ 0.8 KB  │
│       Weld\  │         │       │          │         │
│       Test\  │         │       │          │         │
│     System\  │         │       │          │         │
│     Mada\    │         │       │          │         │
│   STEU\      │         │       │          │         │
├──────────────┴─────────┴───────┴──────────┴─────────┤
│ 4 Objects                                           │ ← Status bar
└─────────────────────────────────────────────────────┘
```

---

## 2. Cấu trúc thư mục ảo KRC:\

### 2.1 Cây thư mục chuẩn KSS 8.x

```
KRC:\
├── R1/                              Robot 1 — Dữ liệu riêng của robot
│   ├── Program/                     ★ Chương trình người dùng
│   │   ├── Cell.src                 Master program (Automatic External)
│   │   ├── Cell.dat                 Data file cho Cell.src
│   │   ├── MyProgram.src            Source code KRL
│   │   ├── MyProgram.dat            Dữ liệu: tọa độ điểm, biến
│   │   ├── WeldStation/             Thư mục con (do user tạo)
│   │   │   ├── Weld_Part_A.src
│   │   │   ├── Weld_Part_A.dat
│   │   │   └── Weld_Part_B.src
│   │   └── Maintenance/
│   │       ├── Zero_Position.src
│   │       └── Thay_Mo.src
│   │
│   ├── System/                      ★ File hệ thống
│   │   ├── $config.dat              Biến toàn cục: TOOL_DATA[], BASE_DATA[]
│   │   ├── bas.src                  Hàm khởi tạo tiêu chuẩn (BAS #INITMOV)
│   │   ├── bas.dat                  Data cho bas.src
│   │   ├── ir_stopm.src             Interrupt handler
│   │   └── sps.sub                  ★ Submit Interpreter (chạy nền)
│   │
│   ├── Mada/                        ★ Machine Data (thông số cơ khí)
│   │   ├── $machine.dat             Giới hạn trục, tỷ số truyền, motor
│   │   ├── $robcor.dat              Dữ liệu hiệu chỉnh robot
│   │   └── $custom.dat              Tùy chỉnh OEM
│   │
│   └── TP/                          Technology Packages
│       ├── GripperTech/
│       ├── ArcTech/                 Hàn hồ quang
│       └── SpotTech/                Hàn điểm
│
├── STEU/                            Steuerung/Control — Cài đặt controller
│   ├── Mada/                        Dữ liệu máy toàn cục
│   │   ├── $option.dat              Options đã mua
│   │   └── $custom.dat              Custom settings
│   └── Config/                      Cấu hình mạng, I/O
│
└── ARCHIVE:\                        Hiển thị file lưu trữ USB/mạng
```

### 2.2 Ý nghĩa từng thư mục

| Thư mục | Mục đích | Ai truy cập | Read/Write |
|---------|----------|-------------|------------|
| `R1\Program\` | Code người dùng viết | User, Expert | Read/Write |
| `R1\System\` | File hệ thống, biến toàn cục | Expert only | Expert: RW, User: RO |
| `R1\Mada\` | Thông số cơ khí robot | Expert/Admin only | Expert: RW, User: RO |
| `R1\TP\` | Technology Packages | Expert only | Read-only (cài từ USB) |
| `STEU\` | Controller settings | Admin only | Admin: RW |
| `ARCHIVE:\` | Backup trên USB | Expert | Read-only (restore riêng) |

### 2.3 Tư duy cốt lõi: Virtual File System (VFS)

KUKA **không** cho user thấy toàn bộ ổ C:. Họ tạo ổ ảo `KRC:\` với các đặc điểm:

- **Lọc**: Chỉ hiện file/folder liên quan đến robot
- **Bảo vệ**: User không thể xóa file hệ thống
- **Đơn giản hóa**: Ẩn đi complexity của Windows OS bên dưới
- **Nhất quán**: Mọi robot KUKA đều có cùng cấu trúc

---

## 3. Workflow thực tế của System Integrator

### Giai đoạn 1: Cấu hình phần cứng (Hardware Configuration)

**Vị trí**: `KRC:\R1\Mada\` và `KRC:\STEU\Mada\`
**Ai làm**: Expert hoặc Administrator
**Khi nào**: Khi mới nhận robot hoặc thay đổi motor/hộp số

**Workflow**:
1. Mở file `$machine.dat` trong Mada
2. Chỉnh sửa thông số: giới hạn trục, tỷ số truyền tải, encoder resolution
3. **Chốt**: Sau khi cấu hình xong → thư mục này "đóng băng". Operator không bao giờ động vào

**File quan trọng trong $machine.dat**:
```krl
; Giới hạn trục mềm
$SOFTP_END[1] = 185.0    ; A1 max
$SOFTN_END[1] = -185.0   ; A1 min
; Tỷ số truyền
$RATIO[1] = -154          ; Gear ratio joint 1
; Hướng encoder
$DIR_TECH[1] = -1         ; Encoder direction
```

### Giai đoạn 2: Thiết lập môi trường (Environment Setup)

**Vị trí**: `KRC:\R1\System\`
**File quan trọng**: `$config.dat`

**Workflow**:
1. Trước khi viết code → khai báo Tool (dụng cụ) và Base (gốc phôi)
2. Mở `$config.dat` hoặc dùng Variable Overview
3. Khai báo:
   - `TOOL_DATA[1]` = TCP của mỏ hàn
   - `BASE_DATA[1]` = Tọa độ bàn xoay
   - Biến toàn cục: `INT iDemSanpham`
4. Khai báo `sps.sub` cho logic nền (safety monitor, I/O)

**File $config.dat ví dụ**:
```krl
DEFDAT $CONFIG PUBLIC
  ; Tool definitions
  DECL E6POS TOOL_DATA[16]
  TOOL_DATA[1] = {X 0, Y 0, Z 150, A 0, B 90, C 0}

  ; Base definitions
  DECL FRAME BASE_DATA[32]
  BASE_DATA[1] = {X 1000, Y 500, Z 0, A 0, B 0, C 0}

  ; Global variables
  DECL INT iDemSanpham = 0
  DECL BOOL bAnToan = TRUE
ENDDAT
```

### Giai đoạn 3: Lập trình ứng dụng (Programming)

**Vị trí**: `KRC:\R1\Program\`
**Đây là nơi dùng NHIỀU NHẤT**

**Workflow thực tế** — Tổ chức theo Module chức năng:

```
📂 Program
├── 📄 Cell.src             Chương trình chính (PLC gateway)
├── 📂 Model_Wave_Alpha     Code cho dòng xe Wave
│   ├── 📄 Han_Suon.src     Hàn sườn xe
│   ├── 📄 Han_Suon.dat     Dữ liệu điểm hàn sườn
│   ├── 📄 Han_Co.src       Hàn cổ xe
│   └── 📄 Han_Co.dat       Dữ liệu điểm hàn cổ
├── 📂 Model_Vision         Code cho dòng Vision
│   └── ...
└── 📂 Maintenance          Bảo dưỡng
    ├── 📄 Thay_Mo.src      Robot ra vị trí bơm mỡ
    └── 📄 Zero_Position.src Về vị trí 0 kiểm tra trục
```

**Tại sao hay?** Khi cần chạy model mới:
1. Copy thư mục `Model_Wave_Alpha`
2. Đổi tên thành `Model_Airblade`
3. Vào trong sửa lại các điểm (TouchUp)
4. Logic và biến giữ nguyên!

### Giai đoạn 4: Chạy nền (Background Logic)

**Vị trí**: `KRC:\R1\System\sps.sub`

**Workflow**:
- File `sps.sub` chạy vòng lặp vô tận ở chế độ nền
- Giống như PLC nhỏ tích hợp sẵn
- Kiểm tra an toàn, monitor I/O, đếm sản phẩm

```krl
LOOP
  IF $IN[1] == FALSE THEN  ; Cửa mở
    $OUT[5] = FALSE          ; Ngừng bơm keo
  ENDIF
  WAIT SEC 0.012             ; 12ms cycle
ENDLOOP
```

### Giai đoạn 5: Sao lưu & Bàn giao (Backup & Handover)

**Tính năng**: Archive (trong menu File)

**Workflow**:
1. Chọn "Archive All" → KUKA nén toàn bộ `KRC:\` thành file ZIP
2. File ZIP chứa **nguyên cấu trúc thư mục**:
   - `$machine.dat` → đúng Mada/
   - Chương trình → đúng Program/
   - Config → đúng System/
3. Khi restore: "Restore" lại file ZIP → tự bung vào đúng vị trí
4. Không cần copy-paste thủ công

---

## 4. Tất cả Features của KUKA Navigator

### 4.1 Navigation & Display

| # | Feature | Mô tả | User Level |
|---|---------|--------|------------|
| 1 | **Tree View (left panel)** | Cây thư mục expand/collapse, hiện tất cả folder | All |
| 2 | **File List (right panel)** | Danh sách file trong folder đang chọn | All |
| 3 | **Column Headers** | Name, Type, Comment, Size, Changed (sortable) | All |
| 4 | **Module View** | Gom cặp .src+.dat thành 1 entry, icon ⚙ | All |
| 5 | **Detail View** | Hiện từng file riêng (.src, .dat, .sub...) | Expert |
| 6 | **Filter ComboBox** | Chuyển đổi Module ↔ Detail view | All |
| 7 | **Path Display** | Hiện đường dẫn KRC:\ hiện tại | All |
| 8 | **Status Bar** | Số object, thông tin file đang chọn | All |
| 9 | **Folder Icons** | 📁 closed / 📂 open (trong tree) | All |
| 10 | **File Type Icons** | ⚙ module, 📄 .src, 📊 .dat, 🔧 .sub | All |

### 4.2 Program Selection & Execution

| # | Feature | Mô tả | User Level |
|---|---------|--------|------------|
| 11 | **Select (Satzanwahl)** | Chọn chương trình để chạy → program pointer nhảy tới DEF | Operator+ |
| 12 | **Cancel Select** | Bỏ chọn chương trình đang active | Operator+ |
| 13 | **Active Program Indicator** | Icon ▶ (xanh) bên cạnh program đang được select | All |
| 14 | **Program State Icons** | Selected (▶), Running (▶▶), Paused (‖), Error (✕) | All |
| 15 | **Double-click Open** | Mở chương trình trong editor | User+ |
| 16 | **Submit Select** | Chọn file .sub để chạy trên Submit Interpreter | Expert |

### 4.3 File Operations (Context Menu)

| # | Feature | Mô tả | User Level |
|---|---------|--------|------------|
| 17 | **New → Module** | Tạo cặp .src+.dat mới từ template | User+ |
| 18 | **New → Folder** | Tạo thư mục con mới | User+ |
| 19 | **New → Submit** | Tạo file .sub mới | Expert |
| 20 | **Open** | Mở file trong editor nội bộ | User+ |
| 21 | **Rename** | Đổi tên file/folder (cả .src và .dat cùng lúc) | User+ |
| 22 | **Delete** | Xóa file/folder (có confirm dialog) | User+ |
| 23 | **Copy** | Copy file/folder (composite .src+.dat) | User+ |
| 24 | **Cut** | Cắt file/folder để di chuyển | User+ |
| 25 | **Paste** | Dán file đã copy/cut vào vị trí hiện tại | User+ |
| 26 | **Properties** | Dialog hiện: Name, Size, Created, Modified, Author, Comment | All |

### 4.4 FOLD (Code Folding)

| # | Feature | Mô tả | User Level |
|---|---------|--------|------------|
| 27 | **INI Fold** | Ẩn block `BAS (#INITMOV,0)` khởi tạo — Operator chỉ thấy 1 dòng "INI" | All |
| 28 | **Motion FOLD** | Ẩn chi tiết motion parameters — chỉ hiện `PTP P1 Vel=100%` | All |
| 29 | **User FOLD** | User tự tạo fold block: `;FOLD ... ;ENDFOLD` | Expert |
| 30 | **Fold/Unfold toggle** | Click để mở/đóng fold — Expert thấy code bên trong | Expert |

### 4.5 Archive & Restore

| # | Feature | Mô tả | User Level |
|---|---------|--------|------------|
| 31 | **Archive All** | Nén toàn bộ KRC:\ thành ZIP (giữ nguyên cấu trúc) | Expert |
| 32 | **Archive Selected** | Chỉ archive thư mục/file đang chọn | Expert |
| 33 | **Archive to USB** | Lưu archive ra USB stick | Expert |
| 34 | **Archive to Network** | Lưu archive qua mạng (SMB share) | Expert |
| 35 | **Restore** | Bung archive ZIP vào controller, tự đặt đúng vị trí | Expert |
| 36 | **Restore Selected** | Chỉ restore một số file từ archive | Expert |
| 37 | **Compare** | So sánh file hiện tại với file trong archive | Expert |

### 4.6 Search & Filter

| # | Feature | Mô tả | User Level |
|---|---------|--------|------------|
| 38 | **Filter by Type** | Lọc theo: All, .src, .dat, .sub, .fold | Expert |
| 39 | **Sort by Column** | Click header để sort: Name, Type, Size, Date | All |
| 40 | **Search by Name** | Tìm kiếm file theo tên (trong WorkVisual, không có trên smartPAD) | Expert |

### 4.7 Advanced Features

| # | Feature | Mô tả | User Level |
|---|---------|--------|------------|
| 41 | **Drag & Drop** | Kéo file giữa thư mục (chỉ trong WorkVisual, không trên smartPAD) | Expert |
| 42 | **Print Program** | In listing chương trình ra printer/PDF | Expert |
| 43 | **Line Numbering** | Hiện số dòng trong editor | Expert |
| 44 | **Read-Only Flag** | System/Mada files hiện khóa 🔒, không cho sửa | All |
| 45 | **File Timestamp** | Hiện ngày/giờ sửa đổi cuối cùng | All |
| 46 | **File Size** | Hiện kích thước file (bytes/KB) | All |
| 47 | **Comment Extraction** | Đọc `&COMMENT` từ header .src để hiện trong list | All |
| 48 | **Auto-refresh** | Tự cập nhật khi file thay đổi (từ WorkVisual deploy) | All |

### 4.8 WorkVisual Integration (Offline)

| # | Feature | Mô tả | User Level |
|---|---------|--------|------------|
| 49 | **Project Navigator** | Tree view trong WorkVisual IDE trên PC | Programmer |
| 50 | **Deploy to Controller** | Upload project từ PC → controller KRC4 | Expert |
| 51 | **Download from Controller** | Tải project từ controller về PC | Expert |
| 52 | **Diff/Merge** | So sánh và merge code giữa PC và controller | Expert |
| 53 | **Multi-robot Project** | Quản lý nhiều robot trong 1 project (MultiMove) | Expert |

### 4.9 Context Menu đầy đủ (Right-click)

```
┌──────────────────────┐
│ ▶ Select             │  ← Chọn chương trình để chạy
│ ✕ Cancel Select      │  ← Bỏ chọn
├──────────────────────┤
│ 📄 Open              │  ← Mở trong editor
│ 📄 New → Module      │  ← Tạo cặp .src/.dat mới
│ 📁 New → Folder      │  ← Tạo thư mục
├──────────────────────┤
│ ✂ Cut                │
│ 📋 Copy              │
│ 📋 Paste             │
├──────────────────────┤
│ ✏ Rename             │
│ 🗑 Delete            │
├──────────────────────┤
│ 📦 Archive           │  ← Backup
│ 📦 Restore           │  ← Khôi phục
├──────────────────────┤
│ ℹ Properties         │  ← Thông tin file chi tiết
└──────────────────────┘
```

---

## 5. Phân quyền User Group

### 5.1 Bảng phân quyền chi tiết

| Feature | Operator | User | Expert | Admin |
|---------|----------|------|--------|-------|
| Xem chương trình | ✅ | ✅ | ✅ | ✅ |
| Select (Satzanwahl) | ✅ | ✅ | ✅ | ✅ |
| Tạo module mới | ❌ | ✅ (chỉ Program/) | ✅ | ✅ |
| Tạo folder | ❌ | ✅ (chỉ Program/) | ✅ | ✅ |
| Rename/Delete | ❌ | ✅ (chỉ Program/) | ✅ | ✅ |
| Copy/Cut/Paste | ❌ | ✅ | ✅ | ✅ |
| Sửa code | ❌ | ✅ (cơ bản) | ✅ (full) | ✅ |
| Xem System/ | ❌ | Read-only | ✅ RW | ✅ |
| Xem Mada/ | ❌ | ❌ | ✅ | ✅ |
| Xem STEU/ | ❌ | ❌ | ❌ | ✅ |
| Archive/Restore | ❌ | ❌ | ✅ | ✅ |
| Detail view | ❌ | ❌ | ✅ | ✅ |
| FOLD unfold | ❌ | ❌ | ✅ | ✅ |
| Sửa $config.dat | ❌ | ❌ | ✅ | ✅ |
| Sửa $machine.dat | ❌ | ❌ | ❌ | ✅ |

### 5.2 Navigator thay đổi theo User Level

**Operator Mode**:
- Chỉ thấy `R1\Program\`
- Chỉ có thể: Select, Cancel, xem Properties
- Không thấy System/, Mada/, STEU/
- FOLD luôn đóng — chỉ thấy dòng tóm tắt

**User Mode**:
- Thấy Program/ (full RW) + System/ (read-only)
- Tạo/xóa/rename trong Program/
- FOLD đóng mặc định, có thể mở

**Expert Mode**:
- Thấy tất cả: Program/, System/, Mada/
- Full CRUD trên mọi thư mục
- Detail view available
- FOLD mở/đóng tùy ý
- Archive/Restore

---

## 6. Audit Implementation hiện tại (G60 Robot Controller)

### 6.1 File cấu trúc

| Component | File | Lines |
|-----------|------|-------|
| ViewModel | `NavigatorViewModel.cs` | 447 |
| View | `NavigatorView.xaml` | 364 |
| Workspace Service | `WorkspaceService.cs` | 625 |
| File Model | `FileItem.cs` | 78 |
| Directory Model | `DirectoryNode.cs` | 20 |
| **Tổng** | | **1,534** |

### 6.2 Virtual File System

**Đã implement**: G60:\ virtual root mapping

```
Assets/VirtualRoot/
├── R1/
│   ├── Program/          Chương trình người dùng
│   ├── System/           File hệ thống
│   └── Mada/             Robot YAML, meshes
├── Tools/                Tool definitions
├── Catalog/              Robot catalog
├── Station/              Station configuration
├── Config/               App config
├── Frames/               Frame definitions
└── Log/                  Log files
```

**Path conversion**: `ToDisplayPath()` / `FromDisplayPath()` — chuyển đổi real path ↔ `KRC:\` format

### 6.3 Features đã implement

| # | Feature KUKA | Status | Chi tiết |
|---|-------------|--------|----------|
| 1 | Tree View (left panel) | ✅ Complete | TreeView with HierarchicalDataTemplate |
| 2 | File List (right panel) | ✅ Complete | GridView: Icon, Name, Type, Comment, Size, Modified |
| 3 | Column Headers | ✅ Complete | Có sort chưa? Chưa verify |
| 4 | Module View | ✅ Complete | Gom .src+.dat thành 1 entry |
| 5 | Detail View | ✅ Complete | Hiện từng file riêng |
| 6 | Filter ComboBox | ✅ Complete | "Module" / "Detail" |
| 7 | Path Display | ✅ Complete | KRC:\ format |
| 8 | Status Bar | ✅ Complete | Item count |
| 9 | Folder Icons | ✅ Complete | 📁/📂 |
| 10 | File Type Icons | ✅ Complete | ⚙ module, 📄 .src, 📊 .dat |
| 11 | Select (Satzanwahl) | ✅ Complete | SelectCommand → SelectProgramRequested event |
| 12 | Cancel Select | ❌ Not impl | — |
| 13 | Active Program Indicator | ✅ Complete | ▶ icon + orange background |
| 14 | Program State Icons | ⚠ Partial | Chỉ có Selected (▶), thiếu Running/Paused/Error |
| 15 | Double-click Open | ✅ Complete | OnFileDoubleClick → OpenProgramRequested |
| 16 | Submit Select | ❌ Not impl | Không có .sub support |
| 17 | New → Module | ✅ Complete | Dialog: Name + Comment, template creation |
| 18 | New → Folder | ✅ Complete | Auto-generated unique name |
| 19 | New → Submit | ❌ Not impl | — |
| 20 | Open | ✅ Complete | Mở trong ProgramEditor |
| 21 | Rename | ✅ Complete | Dialog, composite .src+.dat rename |
| 22 | Delete | ✅ Complete | Confirm dialog, recursive, composite |
| 23 | Copy | ✅ Complete | Composite .src+.dat copy |
| 24 | Cut | ❌ Not impl | — |
| 25 | Paste | ❌ Not impl | — |
| 26 | Properties | ❌ Not impl | — |
| 27-30 | FOLD system | ❌ Not impl | Không có fold/unfold trong editor |
| 31-37 | Archive/Restore | ❌ Not impl | — |
| 38 | Filter by Type | ❌ Not impl | Chỉ có Module/Detail |
| 39 | Sort by Column | ❌ Not impl | Headers có nhưng không sortable |
| 40 | Search by Name | ❌ Not impl | — |
| 41 | Drag & Drop | ❌ Not impl | — |
| 42 | Print Program | ❌ Not impl | — |
| 43 | Line Numbering | ✅ Complete | AvalonEdit có sẵn |
| 44 | Read-Only Flag | ❌ Not impl | Không phân biệt RO/RW |
| 45 | File Timestamp | ✅ Complete | ModifiedDisplay |
| 46 | File Size | ✅ Complete | SizeDisplay |
| 47 | Comment Extraction | ✅ Complete | ExtractComment() đọc &COMMENT |
| 48 | Auto-refresh | ❌ Not impl | — |
| 49-53 | WorkVisual Integration | N/A | Không áp dụng |

### 6.4 Integration đã implement

| Integration | Status | Chi tiết |
|-------------|--------|----------|
| Navigator → ProgramEditor (Open) | ✅ | OpenProgramRequested event |
| Navigator → ProgramEditor (Select/Satzanwahl) | ✅ | SelectProgramRequested → read-only mode |
| Navigator → MainViewModel | ✅ | Page switching khi open/select |
| Navigator → RobotCatalog | ✅ | ActivateRobotFromCatalog() |
| Navigator → Core (IPC) | ⚠ Partial | Chưa trực tiếp gọi LoadProgram |

### 6.5 Context Menu UI

**Hiện tại**: Commands đã implement (Copy, Rename, Delete, New) nhưng **CHƯA CÓ right-click ContextMenu** trong XAML. Các operations chỉ accessible qua buttons hoặc code.

---

## 7. Feature Gap Analysis

### 7.1 Tổng kết

```
Tổng features KUKA Navigator:        ~48 features (không tính WorkVisual)
Đã implement:                         22 features (46%)
Implement một phần:                    2 features (4%)
Chưa implement:                       24 features (50%)
```

### 7.2 Features quan trọng còn thiếu (Priority Order)

#### Priority 1 — Core UX (nên có ngay)

| # | Feature | Lý do quan trọng | Effort |
|---|---------|------------------|--------|
| 1 | **Right-click Context Menu** | Commands có rồi nhưng user không access được! | Low |
| 2 | **Cut/Paste** | Cơ bản cho file management | Low |
| 3 | **Sort by Column** | Click header để sort — UX cơ bản | Low |
| 4 | **Properties Dialog** | Xem thông tin file chi tiết | Medium |
| 5 | **Cancel Select** | Bỏ chọn program đang active | Low |
| 6 | **Search/Filter by Name** | Tìm file nhanh khi có nhiều chương trình | Medium |

#### Priority 2 — Professional Features

| # | Feature | Lý do quan trọng | Effort |
|---|---------|------------------|--------|
| 7 | **Archive/Restore (ZIP)** | Backup/bàn giao dự án — workflow chuẩn | High |
| 8 | **Read-Only Flag** | Bảo vệ System/Mada khỏi sửa nhầm | Medium |
| 9 | **User Access Levels** | Phân quyền Operator/User/Expert | High |
| 10 | **Program State Icons** | Running/Paused/Error indicators | Medium |
| 11 | **Auto-refresh** | FileSystemWatcher khi file thay đổi | Medium |

#### Priority 3 — Advanced

| # | Feature | Effort |
|---|---------|--------|
| 12 | FOLD system (code folding) | High |
| 13 | Drag & Drop | Medium |
| 14 | Submit Interpreter (.sub) | High |
| 15 | Print Program listing | Low |
| 16 | Multi-select operations | Medium |

### 7.3 So sánh trực quan

```
KUKA Navigator Features Coverage:

Navigation & Display  ████████░░  80%  (8/10)
Program Selection     ██████░░░░  60%  (3/5 — thiếu Cancel, State icons)
File Operations       ████████░░  80%  (7/9 — thiếu Cut/Paste, Properties)
FOLD System           ░░░░░░░░░░   0%  (0/4)
Archive/Restore       ░░░░░░░░░░   0%  (0/7)
Search & Filter       ██░░░░░░░░  20%  (1/3 — chỉ có Module/Detail)
Advanced              ██░░░░░░░░  25%  (2/8)
─────────────────────────────────────
Overall               ████░░░░░░  46%  (22/48)
```

---

## 8. Roadmap đề xuất

### Phase 1: Quick Wins (1-2 ngày)
- [ ] Thêm right-click ContextMenu vào NavigatorView.xaml
- [ ] Implement Cut/Paste (đã có Copy)
- [ ] Thêm Cancel Select command
- [ ] Column header sorting (click to sort)

### Phase 2: Professional UX (3-5 ngày)
- [ ] Search box (filter file list by name)
- [ ] Properties dialog (Name, Size, Date, Author, Comment)
- [ ] Program state icons (Running ▶▶, Paused ‖, Error ✕)
- [ ] Read-only flag cho System/Mada folders
- [ ] Auto-refresh (FileSystemWatcher)

### Phase 3: Enterprise Features (1-2 tuần)
- [ ] Archive All → ZIP (giữ cấu trúc thư mục)
- [ ] Restore from ZIP
- [ ] User access levels (Operator/User/Expert)
- [ ] FOLD system trong editor (;FOLD...;ENDFOLD)
- [ ] Drag & Drop files between folders

---

## Phụ lục A: Cặp file .src và .dat

### A.1 Tại sao KUKA dùng cặp file?

**Tách biệt Logic và Data**:
- `.src` = **Logic**: Lệnh motion, điều kiện, vòng lặp
- `.dat` = **Data**: Tọa độ điểm dạy (teach points), biến cục bộ, hằng số

**Lợi ích**:
1. **TouchUp an toàn**: Khi dạy lại điểm → chỉ thay đổi .dat, logic trong .src giữ nguyên
2. **Copy sang robot khác**: Copy cả cặp → tất cả điểm đi theo
3. **Version control**: Thay đổi logic (.src) vs thay đổi dữ liệu (.dat) dễ track

### A.2 Cấu trúc file .src

```krl
&ACCESS RVP          ; Header: quyền truy cập
&REL 1               ; Header: phiên bản relative
&COMMENT Han Suon    ; Header: comment hiện trong Navigator

DEF Han_Suon()       ; Tên chương trình = tên file
  ;FOLD INI          ; Khối khởi tạo (ẩn với Operator)
    BAS (#INITMOV,0)
    BAS (#TOOL,1)     ; Chọn Tool 1
    BAS (#BASE,1)     ; Chọn Base 1
    BAS (#VEL_PTP,100); Tốc độ PTP 100%
    $VEL.CP = 2       ; Tốc độ LIN 2 m/s
    $APO.CDIS = 10    ; Approximation 10mm
  ;ENDFOLD

  PTP HOME Vel=100% DEFAULT

  ;FOLD LIN P1 Vel=1.5 m/s CPDAT1
    LIN {X 500, Y 200, Z 300, A 0, B 90, C 0} C_DIS
  ;ENDFOLD

  LIN P2 C_DIS
  LIN P3

  PTP HOME Vel=100% DEFAULT
END
```

### A.3 Cấu trúc file .dat

```krl
&ACCESS RV
&REL 1

DEFDAT Han_Suon PUBLIC
  ; Điểm dạy (Teach Points)
  DECL E6POS XP1={X 500.0, Y 200.0, Z 300.0, A 0.0, B 90.0, C 0.0, S 2, T 10}
  DECL E6POS XP2={X 600.0, Y 200.0, Z 300.0, A 0.0, B 90.0, C 0.0, S 2, T 10}
  DECL E6POS XP3={X 600.0, Y 300.0, Z 300.0, A 0.0, B 90.0, C 0.0, S 2, T 10}

  ; Biến cục bộ
  DECL INT iLoop = 0
  DECL REAL rSpeed = 1.5
ENDDAT
```

---

## Phụ lục B: Glossary

| Thuật ngữ | Tiếng Đức | Ý nghĩa |
|-----------|-----------|---------|
| Satzanwahl | Program Selection | Chọn chương trình để chạy |
| Mada | Maschinendaten | Machine Data — thông số cơ khí |
| Steuerung (STEU) | Control | Controller settings |
| Betriebsart | Operating Mode | T1/T2/AUT/AUT EXT |
| Handbetrieb | Manual Operation | Chế độ tay (T1/T2) |
| Automatikbetrieb | Automatic Operation | Chế độ tự động (AUT) |
| Überschleifen | Approximation | Lượn qua điểm (C_DIS, C_VEL) |

---

## 9. Manual Test Cases

### Quy ước

| Ký hiệu | Ý nghĩa |
|----------|---------|
| **Pre** | Precondition — điều kiện tiên quyết trước khi test |
| **Steps** | Các bước thực hiện tuần tự |
| **Expected** | Kết quả mong đợi |
| ✅ | Feature đã implement — test trên code hiện tại |
| 🔲 | Feature chưa implement — test khi hoàn thành |
| ⚠ | Feature implement một phần |

**File tham chiếu chính**:
- ViewModel: `src/ui/RobotController.UI/ViewModels/Pages/NavigatorViewModel.cs`
- View: `src/ui/RobotController.UI/Views/Pages/NavigatorView.xaml`
- Workspace: `src/ui/RobotController.UI/Services/WorkspaceService.cs`
- Archive: `src/ui/RobotController.UI/Services/ArchiveService.cs`
- FileItem: `src/ui/RobotController.UI/Models/FileItem.cs`
- DirectoryNode: `src/ui/RobotController.UI/Models/DirectoryNode.cs`
- Folding: `src/ui/RobotController.UI/Editor/WeldSeamFoldingStrategy.cs`

---

### 9.1 Navigation & Display

#### TC-NAV-001: Tree View hiển thị cấu trúc thư mục ✅

| | Nội dung |
|---|---------|
| **Pre** | Workspace có cấu trúc: `R1/Program/`, `R1/System/`, `R1/Mada/`, `Tools/`, `Catalog/` |
| **Steps** | 1. Mở Navigator page |
| | 2. Quan sát panel trái (Directory Tree) |
| **Expected** | - Cây thư mục hiện đầy đủ các folder con<br>- Mỗi node có icon 📁<br>- Có thể expand/collapse bằng click mũi tên<br>- Folder expand hiện icon 📂 |

#### TC-NAV-002: File List hiển thị đúng nội dung thư mục ✅

| | Nội dung |
|---|---------|
| **Pre** | Thư mục `R1/Program/` có file `Test.src`, `Test.dat`, subfolder `Weld/` |
| **Steps** | 1. Click chọn `R1/Program/` trên tree<br>2. Quan sát panel phải (File List) |
| **Expected** | - GridView hiện các cột: Icon, Name, Type, Comment, Size, Changed<br>- Subfolder `Weld/` hiện trước (nếu có)<br>- File `Test` hiện với icon phù hợp |

#### TC-NAV-003: Module View gom cặp .src+.dat ✅

| | Nội dung |
|---|---------|
| **Pre** | Thư mục có `MyProg.src` và `MyProg.dat` |
| **Steps** | 1. Chọn filter mode "Module" từ ComboBox header<br>2. Navigate đến thư mục chứa file |
| **Expected** | - `MyProg.src` và `MyProg.dat` gom thành **1 entry** tên "MyProg"<br>- Icon hiện ⚙ (module)<br>- Type hiện "Module"<br>- Size hiện tổng cộng cả 2 file |

#### TC-NAV-004: Detail View hiện từng file riêng ✅

| | Nội dung |
|---|---------|
| **Pre** | Thư mục có `MyProg.src` và `MyProg.dat` |
| **Steps** | 1. Chọn filter mode "Detail" từ ComboBox header<br>2. Navigate đến thư mục chứa file |
| **Expected** | - Hiện **2 entry** riêng biệt: `MyProg.src` (icon 📄) và `MyProg.dat` (icon 📊)<br>- Type lần lượt là "Source" và "Data" |

#### TC-NAV-005: Path Display hiện đúng đường dẫn KRC:\ ✅

| | Nội dung |
|---|---------|
| **Pre** | Đang ở thư mục `R1/Program/Weld/` |
| **Steps** | 1. Navigate đến subfolder `Weld` bên trong `Program`<br>2. Quan sát thanh header |
| **Expected** | - Path display hiện `KRC:\R1\Program\Weld\`<br>- Dùng ký tự `\` (Windows style) không phải `/` |

#### TC-NAV-006: Status Bar hiện số object ✅

| | Nội dung |
|---|---------|
| **Pre** | Thư mục có 3 file và 1 subfolder |
| **Steps** | 1. Navigate đến thư mục<br>2. Quan sát thanh status bar dưới cùng |
| **Expected** | - Hiện text dạng "4 Objects" hoặc tương tự<br>- Số đúng với số item đang hiện trong file list |

#### TC-NAV-007: File Type Icons hiển thị đúng ✅

| | Nội dung |
|---|---------|
| **Pre** | Thư mục chứa module (.src+.dat), file .src đơn, file .dat đơn, subfolder |
| **Steps** | 1. Chọn Detail view<br>2. Quan sát cột Icon |
| **Expected** | - Module: ⚙<br>- Source file (.src): 📄<br>- Data file (.dat): 📊<br>- Folder: 📁<br>- Read-only (System/Mada): 🔒 |

#### TC-NAV-008: Comment Extraction từ header .src ✅

| | Nội dung |
|---|---------|
| **Pre** | File `Test.src` có dòng `&COMMENT Han Suon` trong header |
| **Steps** | 1. Navigate đến thư mục chứa file<br>2. Quan sát cột Comment |
| **Expected** | - Cột Comment hiện "Han Suon"<br>- Nếu file không có `&COMMENT` → cột Comment trống |

---

### 9.2 Program Selection & Execution

#### TC-SEL-001: Select (Satzanwahl) chương trình ✅

| | Nội dung |
|---|---------|
| **Pre** | Có file `MyProg.src` trong `R1/Program/` |
| **Steps** | 1. Chọn file `MyProg` trong file list<br>2. Right-click → "Select" (hoặc dùng command) |
| **Expected** | - File `MyProg` hiện icon ▶ và background cam nhạt (#FFF3E0)<br>- `ActiveProgramPath` cập nhật đúng<br>- Event `SelectProgramRequested` được raise |

#### TC-SEL-002: Cancel Select bỏ chọn chương trình ✅

| | Nội dung |
|---|---------|
| **Pre** | Đã Select chương trình `MyProg` (đang hiện icon ▶) |
| **Steps** | 1. Right-click → "Cancel Select" |
| **Expected** | - Icon ▶ biến mất, background trở về bình thường<br>- `ActiveProgramPath` trở về null/empty<br>- Event `CancelSelectRequested` được raise |

#### TC-SEL-003: Program State Icons thay đổi theo trạng thái ✅

| | Nội dung |
|---|---------|
| **Pre** | Chương trình đã được Select |
| **Steps** | 1. Select chương trình → quan sát icon<br>2. Gọi `UpdateProgramState(ProgramState.Running)`<br>3. Gọi `UpdateProgramState(ProgramState.Paused)`<br>4. Gọi `UpdateProgramState(ProgramState.Error)` |
| **Expected** | - Selected: icon ▶<br>- Running: icon ▶▶<br>- Paused: icon ‖<br>- Error: icon ✕ |

#### TC-SEL-004: Double-click mở chương trình trong editor ✅

| | Nội dung |
|---|---------|
| **Pre** | Có file `MyProg.src` trong file list |
| **Steps** | 1. Double-click vào file `MyProg` |
| **Expected** | - Event `OpenProgramRequested` được raise với path đúng<br>- Nếu item là folder → navigate vào folder thay vì mở editor |

#### TC-SEL-005: Double-click folder navigates vào bên trong ✅

| | Nội dung |
|---|---------|
| **Pre** | File list hiện subfolder `Weld/` |
| **Steps** | 1. Double-click vào folder `Weld` |
| **Expected** | - Tree tự expand và select folder `Weld`<br>- File list cập nhật hiện nội dung folder `Weld`<br>- Path display cập nhật thành `KRC:\R1\Program\Weld\` |

---

### 9.3 File Operations

#### TC-FILE-001: Tạo Module mới ✅

| | Nội dung |
|---|---------|
| **Pre** | Đang ở thư mục `R1/Program/` |
| **Steps** | 1. Right-click → "New" (hoặc nhấn nút New)<br>2. Chọn "Module" trong dialog type chooser<br>3. Nhập tên "WeldTest" và comment "Test program"<br>4. Nhấn OK |
| **Expected** | - Tạo ra 2 file: `WeldTest.src` và `WeldTest.dat`<br>- File .src có header `&COMMENT Test program` và template `DEF WeldTest()`<br>- File .dat có `DEFDAT WeldTest PUBLIC` với HOME position mặc định<br>- File list tự refresh hiện module mới |

#### TC-FILE-002: Tạo Folder mới ✅

| | Nội dung |
|---|---------|
| **Pre** | Đang ở thư mục `R1/Program/` |
| **Steps** | 1. Right-click → "New"<br>2. Chọn "Folder" trong dialog type chooser |
| **Expected** | - Tạo subfolder mới với tên tự động (unique)<br>- Tree view cập nhật hiện folder mới<br>- File list hiện folder mới với icon 📁 |

#### TC-FILE-003: Rename file (composite .src+.dat) ✅

| | Nội dung |
|---|---------|
| **Pre** | Có module `OldName` (OldName.src + OldName.dat) |
| **Steps** | 1. Chọn module `OldName`<br>2. Right-click → "Rename"<br>3. Nhập tên mới "NewName"<br>4. Nhấn OK |
| **Expected** | - Cả hai file đổi tên: `NewName.src` và `NewName.dat`<br>- File list refresh hiện tên mới<br>- Nội dung bên trong file (DEF/DEFDAT) KHÔNG đổi (chỉ đổi tên file) |

#### TC-FILE-004: Rename folder ✅

| | Nội dung |
|---|---------|
| **Pre** | Có subfolder `OldFolder` trong `Program/` |
| **Steps** | 1. Chọn folder `OldFolder` trong file list<br>2. Right-click → "Rename"<br>3. Nhập "NewFolder"<br>4. Nhấn OK |
| **Expected** | - Folder đổi tên thành `NewFolder`<br>- Tree view cập nhật<br>- Nội dung bên trong folder giữ nguyên |

#### TC-FILE-005: Delete file với confirmation ✅

| | Nội dung |
|---|---------|
| **Pre** | Có module `ToDelete` (ToDelete.src + ToDelete.dat) |
| **Steps** | 1. Chọn file `ToDelete`<br>2. Right-click → "Delete"<br>3. Confirm dialog hiện "Delete ToDelete?"<br>4. Nhấn "Delete" |
| **Expected** | - Cả `ToDelete.src` và `ToDelete.dat` bị xóa khỏi disk<br>- File list refresh, module biến mất<br>- Nếu nhấn "Cancel" → không xóa gì |

#### TC-FILE-006: Delete folder (recursive) ✅

| | Nội dung |
|---|---------|
| **Pre** | Folder `TestFolder/` chứa 2 file bên trong |
| **Steps** | 1. Chọn folder `TestFolder` trong file list<br>2. Right-click → "Delete"<br>3. Confirm dialog hiện<br>4. Nhấn "Delete" |
| **Expected** | - Xóa folder và tất cả nội dung bên trong (recursive)<br>- Tree view cập nhật, folder biến mất |

#### TC-FILE-007: Copy file ✅

| | Nội dung |
|---|---------|
| **Pre** | Có module `Source` trong `Program/`, có subfolder `Dest/` |
| **Steps** | 1. Chọn module `Source`<br>2. Right-click → "Copy"<br>3. Navigate đến folder `Dest`<br>4. Right-click → "Paste" |
| **Expected** | - `Source.src` và `Source.dat` được copy vào `Dest/`<br>- File gốc vẫn còn ở vị trí cũ<br>- File list tại `Dest/` hiện module `Source` mới |

#### TC-FILE-008: Cut và Paste (di chuyển file) ✅

| | Nội dung |
|---|---------|
| **Pre** | Có module `MoveMe` trong `Program/`, có subfolder `Target/` |
| **Steps** | 1. Chọn module `MoveMe`<br>2. Right-click → "Cut"<br>3. Navigate đến folder `Target`<br>4. Right-click → "Paste" |
| **Expected** | - `MoveMe.src` và `MoveMe.dat` di chuyển sang `Target/`<br>- File gốc KHÔNG CÒN ở vị trí cũ<br>- File list tại `Target/` hiện module |

#### TC-FILE-009: Copy folder ✅

| | Nội dung |
|---|---------|
| **Pre** | Folder `SrcFolder/` chứa 3 file, có folder `DestFolder/` |
| **Steps** | 1. Chọn `SrcFolder` trong file list<br>2. Right-click → "Copy"<br>3. Navigate đến `DestFolder`<br>4. Right-click → "Paste" |
| **Expected** | - `SrcFolder` được copy toàn bộ (recursive) vào `DestFolder/SrcFolder/`<br>- File gốc giữ nguyên |

#### TC-FILE-010: Open file mở trong editor ✅

| | Nội dung |
|---|---------|
| **Pre** | Có file `Test.src` với nội dung KRL |
| **Steps** | 1. Chọn file `Test`<br>2. Right-click → "Open" |
| **Expected** | - Event `OpenProgramRequested` raise<br>- Editor mở hiện nội dung file<br>- Kết quả tương tự double-click |

#### TC-FILE-011: Properties Dialog ✅

| | Nội dung |
|---|---------|
| **Pre** | Có module `Info` (Info.src + Info.dat), file .src có `&COMMENT Weld Program` |
| **Steps** | 1. Chọn file `Info`<br>2. Right-click → "Properties" |
| **Expected** | - Dialog hiện các thông tin (read-only):<br>  - Name: "Info"<br>  - Type: "Module"<br>  - Size: kích thước đúng (ví dụ "1.2 KB")<br>  - Path: đường dẫn KRC:\<br>  - Created: ngày tạo file<br>  - Modified: ngày sửa cuối<br>  - Comment: "Weld Program"<br>- Nhấn Close đóng dialog |

---

### 9.4 Context Menu

#### TC-CTX-001: Right-click hiện Context Menu đầy đủ ✅

| | Nội dung |
|---|---------|
| **Pre** | File list có ít nhất 1 file |
| **Steps** | 1. Right-click vào 1 file trong file list |
| **Expected** | - Context menu hiện với các mục:<br>  Open, Select, New (Module/Folder), Cut, Copy, Paste,<br>  Rename, Delete, Cancel Select, Archive All, Archive Selected,<br>  Restore, Compare with Archive, Properties |

#### TC-CTX-002: Context Menu item disabled khi không hợp lệ ✅

| | Nội dung |
|---|---------|
| **Pre** | Không có file nào đang select, chưa copy/cut gì |
| **Steps** | 1. Right-click khi không chọn file nào<br>2. Quan sát trạng thái các menu item |
| **Expected** | - Paste: disabled (chưa copy/cut)<br>- Cancel Select: disabled (chưa select program)<br>- Open, Rename, Delete: disabled nếu không chọn file |

---

### 9.5 FOLD System (Code Folding)

**Test file**: `Assets/VirtualRoot/R1/Program/FoldTest.src` + `FoldTest.dat`

File test chứa đầy đủ tất cả fold types: `;FOLD`/`;ENDFOLD`, `;#REGION`/`;#ENDREGION`,
`ArcStart`/`ArcEnd`, `DEF`/`END`, nested folds, case-insensitive, bare fold, unclosed fold.

#### TC-FOLD-F01: Basic ;FOLD ... ;ENDFOLD ✅

| | Nội dung |
|---|---------|
| **Pre** | Mở `FoldTest.src` trong ProgramEditor |
| **Steps** | 1. Tìm dòng `;FOLD PTP HOME Vel=100%` (dòng ~21)<br>2. Quan sát fold margin bên trái |
| **Expected** | - Fold marker [-] xuất hiện bên trái dòng `;FOLD`<br>- Click collapse → nội dung ẩn, chỉ hiện 1 dòng summary<br>- Click expand → hiện lại nội dung bên trong<br>- `DefaultClosed = false` (mặc định mở) |

#### TC-FOLD-F02: INI FOLD auto-collapse ✅

| | Nội dung |
|---|---------|
| **Pre** | Mở `FoldTest.src` — dòng 5: `;FOLD INI` |
| **Steps** | 1. Mở file lần đầu<br>2. Quan sát block INI |
| **Expected** | - Block INI **tự động collapse** khi mở file<br>- `DefaultClosed = true` cho label chứa "INI"<br>- Click expand → hiện code `BAS(#INITMOV)`, `BAS(#TOOL)`, etc. |

#### TC-FOLD-F03: BASISTECH FOLD auto-collapse ✅

| | Nội dung |
|---|---------|
| **Pre** | Mở `FoldTest.src` — dòng 6: `;FOLD BASISTECH INI` (nested trong INI) |
| **Steps** | 1. Expand block INI bên ngoài<br>2. Quan sát block BASISTECH bên trong |
| **Expected** | - Block BASISTECH **tự động collapse**<br>- `DefaultClosed = true` cho label chứa "BASISTECH"<br>- Nested đúng: nằm bên trong outer INI fold |

#### TC-FOLD-F04: Motion FOLD expanded ✅

| | Nội dung |
|---|---------|
| **Pre** | Mở `FoldTest.src` — dòng ~21: `;FOLD PTP HOME Vel=100%` |
| **Steps** | 1. Quan sát trạng thái mặc định |
| **Expected** | - Block **mở rộng** (expanded) khi mở file<br>- `DefaultClosed = false` — vì label không phải INI/BASISTECH<br>- Label hiện "PTP HOME Vel=100%" |

#### TC-FOLD-F05: Nested folds — INI > BASISTECH ✅

| | Nội dung |
|---|---------|
| **Pre** | `FoldTest.src` dòng 5-13: outer `;FOLD INI` chứa inner `;FOLD BASISTECH INI` |
| **Steps** | 1. Expand outer INI fold<br>2. Thấy inner BASISTECH fold<br>3. Toggle inner fold<br>4. Collapse outer fold |
| **Expected** | - 2 level fold hoạt động độc lập<br>- Collapse outer → inner ẩn cùng<br>- Expand outer → inner vẫn giữ trạng thái riêng |

#### TC-FOLD-F06: User FOLD với label tùy chỉnh ✅

| | Nội dung |
|---|---------|
| **Pre** | `FoldTest.src` dòng ~39: `;FOLD User Section` |
| **Steps** | 1. Tìm block "User Section"<br>2. Quan sát fold marker và label |
| **Expected** | - Fold hoạt động<br>- Label hiện **"User Section"** (text sau `;FOLD `)<br>- `DefaultClosed = false`<br>- Nội dung: `$OUT[1] = TRUE`, `WAIT SEC 0.5`, `$OUT[1] = FALSE` |

#### TC-FOLD-F07: Case insensitive `;fold` ✅

| | Nội dung |
|---|---------|
| **Pre** | `FoldTest.src` dòng ~44: `;fold lower case fold` |
| **Steps** | 1. Quan sát fold marker cho dòng viết thường |
| **Expected** | - Nhận dạng đúng dù viết `;fold` thay vì `;FOLD`<br>- Fold hoạt động bình thường<br>- Label hiện "lower case fold" |

#### TC-FOLD-F08: Bare `;FOLD` (không label) ✅

| | Nội dung |
|---|---------|
| **Pre** | `FoldTest.src` dòng ~48: `;FOLD` (không có text theo sau) |
| **Steps** | 1. Quan sát fold marker |
| **Expected** | - Fold vẫn hoạt động<br>- Label hiện "" hoặc "FOLD" (fallback)<br>- `DefaultClosed = false` |

#### TC-FOLD-F09: Unclosed `;FOLD` báo lỗi ✅

| | Nội dung |
|---|---------|
| **Pre** | `FoldTest.src` dòng ~53: `;FOLD Unclosed fold for error test` — KHÔNG CÓ `;ENDFOLD` |
| **Steps** | 1. Mở file<br>2. Kiểm tra `firstErrorOffset` trả về từ `UpdateFoldings()` |
| **Expected** | - `firstErrorOffset` ≠ -1 (báo có lỗi)<br>- Offset trỏ đến vị trí dòng `;FOLD Unclosed...`<br>- Editor có thể hiện warning marker tại vị trí lỗi |

#### TC-FOLD-F10: Tất cả fold types hoạt động song song ✅

| | Nội dung |
|---|---------|
| **Pre** | `FoldTest.src` chứa đồng thời: `;FOLD`/`;ENDFOLD`, `;#REGION`/`;#ENDREGION`, `ArcStart`/`ArcEnd`, `DEF`/`END` |
| **Steps** | 1. Mở file<br>2. Đếm tổng số fold markers<br>3. Toggle từng loại fold |
| **Expected** | Tất cả fold types hiện cùng lúc:<br>- **DEF blocks**: 2 (FoldTest + SubRoutine1), label "DEF FoldTest", "DEF SubRoutine1"<br>- **;FOLD blocks**: ~7 (INI, BASISTECH, PTP, User Section, lower case, bare, unclosed)<br>- **;#REGION blocks**: 3 (Approach, Weld Pass 1, Retract)<br>- **ArcStart blocks**: 2 (Job:1 + Job:?)<br>- Không xung đột giữa các loại |

---

#### FOLD Test — ArcStart/ArcEnd & Seam Length

#### TC-FOLD-F11: ArcStart với Job_ID — Seam 1 ✅

| | Nội dung |
|---|---------|
| **Pre** | `FoldTest.src` dòng ~27: `ArcStart(Job_ID:=1)` chứa `LIN P1`, `LIN P2`, `LIN P3` |
| **Steps** | 1. Mở file<br>2. Quan sát fold label cho ArcStart block |
| **Expected** | - Label: **"Weld Seam (Job: 1) - Length: 600.0 mm"**<br>- Tính: P1(500,0,300)→P2(500,300,300) = 300mm + P2→P3(500,600,300) = 300mm = **600.0 mm**<br>- Coordinates lookup từ `FoldTest.dat` |

#### TC-FOLD-F12: ArcStart không có Job_ID — Seam 2 ✅

| | Nội dung |
|---|---------|
| **Pre** | `FoldTest.src` dòng ~33: `ArcStart` (bare, không Job_ID), chứa `LIN P4`, `PTP P5`, `LIN P6` |
| **Steps** | 1. Quan sát fold label |
| **Expected** | - Label: **"Weld Seam (Job: ?) - Length: 200.0 mm"**<br>- Job hiện "?" vì không có `Job_ID:=`<br>- `PTP P5` bị **bỏ qua** (chỉ tính LIN)<br>- Tính: P4(600,0,300)→P6(600,200,300) = **200.0 mm** |

#### TC-FOLD-F13: DEF ... END routine folding ✅

| | Nội dung |
|---|---------|
| **Pre** | `FoldTest.src` có 2 routine: `DEF FoldTest()` và `DEF SubRoutine1()` |
| **Steps** | 1. Quan sát fold markers cho DEF blocks |
| **Expected** | - 2 DEF fold markers<br>- Labels: "DEF FoldTest", "DEF SubRoutine1" (bỏ dấu ngoặc `()`)<br>- `DefaultClosed = false`<br>- Mỗi DEF fold bao trọn từ `DEF` đến `END` |

#### TC-FOLD-F14: ;#REGION ... ;#ENDREGION folding ✅

| | Nội dung |
|---|---------|
| **Pre** | `FoldTest.src` có 3 region: "Approach", "Weld Pass 1", "Retract" |
| **Steps** | 1. Quan sát fold markers cho region blocks |
| **Expected** | - 3 region fold markers<br>- Labels: "Approach", "Weld Pass 1", "Retract"<br>- `DefaultClosed = false` (mặc định mở)<br>- Region "Weld Pass 1" chứa nested ArcStart blocks bên trong |

---

### 9.6 Archive & Restore

#### TC-ARC-001: Archive All tạo ZIP toàn bộ workspace ✅

| | Nội dung |
|---|---------|
| **Pre** | Workspace có nhiều file trong `R1/Program/`, `R1/System/`, `Tools/` |
| **Steps** | 1. Right-click → "Archive All"<br>2. Chọn đường dẫn lưu ZIP (SaveFileDialog) |
| **Expected** | - File ZIP được tạo tại đường dẫn chỉ định<br>- ZIP chứa tất cả file trừ `Log/` và `Catalog/`<br>- ZIP chứa `_archive_manifest.json` với Timestamp, Version, FileCount, Source<br>- Cấu trúc thư mục trong ZIP giữ nguyên relative path |

#### TC-ARC-002: Archive Selected tạo ZIP cho file/folder chọn ✅

| | Nội dung |
|---|---------|
| **Pre** | Chọn folder `R1/Program/Weld/` chứa 4 file |
| **Steps** | 1. Chọn folder `Weld` trong file list<br>2. Right-click → "Archive Selected"<br>3. Chọn đường dẫn lưu ZIP |
| **Expected** | - ZIP chỉ chứa file trong `Weld/` (4 file)<br>- Relative path đúng: `R1/Program/Weld/...`<br>- Manifest có `FileCount = 4`, `Source = <folder path>` |

#### TC-ARC-003: Archive Selected với .src tự kèm .dat ✅

| | Nội dung |
|---|---------|
| **Pre** | Chọn single file `Test.src`, cùng thư mục có `Test.dat` |
| **Steps** | 1. Chọn `Test.src` (chỉ file .src)<br>2. Right-click → "Archive Selected" |
| **Expected** | - ZIP chứa CẢ `Test.src` VÀ `Test.dat` (auto-pair)<br>- FileCount = 2 |

#### TC-ARC-004: List Archive Contents ✅

| | Nội dung |
|---|---------|
| **Pre** | Có file ZIP archive đã tạo từ TC-ARC-001 |
| **Steps** | 1. Right-click → "Restore"<br>2. Chọn file ZIP → dialog Restore mở ra |
| **Expected** | - Dialog hiện danh sách các entry trong ZIP<br>- Mỗi entry hiện: RelativePath, DisplayPath, Size, CompressedSize, LastModified<br>- Checkbox cho từng entry để chọn restore |

#### TC-ARC-005: Restore All với Overwrite ✅

| | Nội dung |
|---|---------|
| **Pre** | Có archive ZIP, workspace có một số file trùng tên |
| **Steps** | 1. Right-click → "Restore"<br>2. Chọn file ZIP<br>3. Chọn tất cả entries<br>4. Nhấn "Restore Selected" (mode: Overwrite) |
| **Expected** | - Tất cả file từ ZIP giải nén vào workspace<br>- File trùng tên bị ghi đè<br>- RestoreResult hiện số Restored, Overwritten, Skipped<br>- `_archive_manifest.json` KHÔNG được restore vào workspace |

#### TC-ARC-006: Restore Selected chỉ restore một số file ✅

| | Nội dung |
|---|---------|
| **Pre** | Archive ZIP có 10 file |
| **Steps** | 1. Mở Restore dialog<br>2. Chỉ check 3 file cần restore<br>3. Nhấn "Restore Selected" |
| **Expected** | - Chỉ 3 file được giải nén<br>- 7 file còn lại không bị ảnh hưởng<br>- Workspace giữ nguyên các file không liên quan |

#### TC-ARC-007: Restore với ConflictResolution Skip ✅

| | Nội dung |
|---|---------|
| **Pre** | Archive chứa `Test.src`, workspace đã có `Test.src` |
| **Steps** | 1. Restore với mode = Skip |
| **Expected** | - File `Test.src` hiện tại KHÔNG bị ghi đè<br>- RestoreResult.Skipped tăng 1 |

#### TC-ARC-008: Restore với ConflictResolution Rename ✅

| | Nội dung |
|---|---------|
| **Pre** | Archive chứa `Test.src`, workspace đã có `Test.src` |
| **Steps** | 1. Restore với mode = Rename |
| **Expected** | - File mới được lưu là `Test_restored1.src` (hoặc `_restored2` nếu đã tồn tại)<br>- File cũ giữ nguyên |

#### TC-ARC-009: Compare with Archive ✅

| | Nội dung |
|---|---------|
| **Pre** | Archive ZIP tạo từ workspace trước đó, sau đó sửa 1 file, xóa 1 file, thêm 1 file mới |
| **Steps** | 1. Right-click → "Compare with Archive"<br>2. Chọn file ZIP |
| **Expected** | - Dialog Compare hiện danh sách so sánh<br>- File không đổi: Status = "Identical"<br>- File đã sửa: Status = "Different" (size hoặc date khác > 2 giây)<br>- File bị xóa local: Status = "OnlyInArchive"<br>- File mới thêm local: Status = "OnlyLocal"<br>- Bỏ qua `Log/` và `Catalog/` |

#### TC-ARC-010: Archive manifest chứa metadata đúng ✅

| | Nội dung |
|---|---------|
| **Pre** | Vừa tạo archive |
| **Steps** | 1. Mở file ZIP bằng tool bên ngoài<br>2. Tìm `_archive_manifest.json`<br>3. Đọc nội dung |
| **Expected** | - JSON có các field: `Timestamp` (ISO 8601), `Version` ("1.0"), `FileCount` (đúng số), `Source` (path gốc)<br>- Timestamp là UTC |

---

### 9.7 Search & Filter

#### TC-SEARCH-001: Search box lọc file theo tên ✅

| | Nội dung |
|---|---------|
| **Pre** | Thư mục có files: `Weld_PartA.src`, `Weld_PartB.src`, `Home.src` |
| **Steps** | 1. Gõ "Weld" vào Search box trong header |
| **Expected** | - File list chỉ hiện `Weld_PartA` và `Weld_PartB`<br>- `Home.src` bị ẩn<br>- Filter realtime khi gõ |

#### TC-SEARCH-002: Xóa search text hiện lại tất cả ✅

| | Nội dung |
|---|---------|
| **Pre** | Search box đang có text "Weld", file list đang lọc |
| **Steps** | 1. Xóa hết text trong search box |
| **Expected** | - File list hiện lại tất cả files/folders<br>- Không mất data |

#### TC-SEARCH-003: Sort by Column click header ✅

| | Nội dung |
|---|---------|
| **Pre** | Thư mục có nhiều file với size và date khác nhau |
| **Steps** | 1. Click header "Name" → sort A-Z<br>2. Click lại "Name" → sort Z-A<br>3. Click header "Size" → sort theo size<br>4. Click header "Changed" → sort theo ngày |
| **Expected** | - Mỗi click toggle ascending/descending<br>- Sort đúng theo kiểu dữ liệu (text, number, date) |

---

### 9.8 Read-Only & Protection

#### TC-RO-001: File trong System/ hiện icon khóa ✅

| | Nội dung |
|---|---------|
| **Pre** | Thư mục `R1/System/` có file `$config.dat` |
| **Steps** | 1. Navigate đến `R1/System/`<br>2. Quan sát icon trong file list |
| **Expected** | - File hiện icon 🔒 thay vì icon bình thường<br>- `IsReadOnly = true` trong FileItem model |

#### TC-RO-002: File trong Mada/ hiện icon khóa ✅

| | Nội dung |
|---|---------|
| **Pre** | Thư mục `R1/Mada/` có file `robot.yaml` |
| **Steps** | 1. Navigate đến `R1/Mada/`<br>2. Quan sát icon |
| **Expected** | - File hiện icon 🔒<br>- `IsProtectedDirectory()` return true cho path trong Mada/ |

---

### 9.9 Auto-Refresh (FileSystemWatcher)

#### TC-REFRESH-001: File mới tự hiện trong list ✅

| | Nội dung |
|---|---------|
| **Pre** | Đang view thư mục `R1/Program/` trong Navigator |
| **Steps** | 1. Dùng File Explorer bên ngoài, copy file `NewProg.src` vào `R1/Program/`<br>2. Đợi ~500ms (debounce) |
| **Expected** | - File list tự cập nhật hiện `NewProg.src` mới<br>- Không cần manual refresh |

#### TC-REFRESH-002: File xóa bên ngoài tự biến mất ✅

| | Nội dung |
|---|---------|
| **Pre** | Đang view thư mục có file `Old.src` |
| **Steps** | 1. Dùng File Explorer xóa `Old.src`<br>2. Đợi ~500ms |
| **Expected** | - File list tự cập nhật, `Old.src` biến mất<br>- Không bị crash hoặc exception |

---

### 9.10 Integration Tests

#### TC-INT-001: Navigator → Editor workflow hoàn chỉnh ✅

| | Nội dung |
|---|---------|
| **Pre** | Có file `Test.src` trong `R1/Program/` |
| **Steps** | 1. Mở Navigator page<br>2. Navigate đến `R1/Program/`<br>3. Double-click `Test`<br>4. Editor mở hiện code<br>5. Sửa code, save<br>6. Quay lại Navigator |
| **Expected** | - Editor mở đúng file<br>- Save ghi đúng nội dung xuống disk<br>- Quay lại Navigator, file Modified date cập nhật |

#### TC-INT-002: Select program → Active indicator persists ✅

| | Nội dung |
|---|---------|
| **Pre** | Có file `Main.src` |
| **Steps** | 1. Select program `Main`<br>2. Navigate ra folder khác<br>3. Navigate quay lại folder chứa `Main` |
| **Expected** | - `Main` vẫn hiện icon ▶ và background cam<br>- Active program indicator persistent qua navigation |

#### TC-INT-003: Archive → Restore → Compare round-trip ✅

| | Nội dung |
|---|---------|
| **Pre** | Workspace có 5+ file |
| **Steps** | 1. Archive All → tạo `backup.zip`<br>2. Sửa 1 file, xóa 1 file, thêm 1 file mới<br>3. Compare with Archive `backup.zip`<br>4. Verify compare results đúng<br>5. Restore All từ `backup.zip` (Overwrite)<br>6. Compare lại |
| **Expected** | - Bước 3: Hiện Different, OnlyInArchive, OnlyLocal đúng<br>- Bước 5: Restore thành công<br>- Bước 6: Tất cả file trừ file mới thêm đều "Identical" |

---

### 9.11 Edge Cases & Error Handling

#### TC-EDGE-001: Tạo module trùng tên ✅

| | Nội dung |
|---|---------|
| **Pre** | Đã có module `Existing` trong thư mục |
| **Steps** | 1. Tạo module mới với tên "Existing" |
| **Expected** | - Hiện thông báo lỗi hoặc từ chối tạo<br>- Không ghi đè file cũ |

#### TC-EDGE-002: Rename với tên rỗng hoặc ký tự đặc biệt ✅

| | Nội dung |
|---|---------|
| **Pre** | Có file cần rename |
| **Steps** | 1. Rename với tên rỗng ""<br>2. Rename với ký tự đặc biệt "Test/\\:*?" |
| **Expected** | - Từ chối tên rỗng<br>- Từ chối hoặc sanitize ký tự không hợp lệ<br>- File gốc không bị ảnh hưởng |

#### TC-EDGE-003: Delete khi không chọn file ✅

| | Nội dung |
|---|---------|
| **Pre** | Không có file nào đang selected |
| **Steps** | 1. Gọi Delete command |
| **Expected** | - Command không thực hiện (CanExecute = false hoặc early return)<br>- Không crash |

#### TC-EDGE-004: Paste khi chưa Copy/Cut ✅

| | Nội dung |
|---|---------|
| **Pre** | Chưa thực hiện Copy hoặc Cut |
| **Steps** | 1. Right-click → "Paste" |
| **Expected** | - Paste disabled hoặc không làm gì<br>- Không crash, không tạo file rỗng |

#### TC-EDGE-005: Navigate đến thư mục rỗng ✅

| | Nội dung |
|---|---------|
| **Pre** | Thư mục `Empty/` không chứa file nào |
| **Steps** | 1. Navigate đến `Empty/` |
| **Expected** | - File list trống, không có entry nào<br>- Status bar hiện "0 Objects"<br>- Path display cập nhật đúng |

#### TC-EDGE-006: Archive workspace rỗng ✅

| | Nội dung |
|---|---------|
| **Pre** | Workspace chỉ có cấu trúc thư mục trống |
| **Steps** | 1. Archive All |
| **Expected** | - ZIP tạo thành công<br>- Manifest có `FileCount = 0`<br>- Không crash |

#### TC-EDGE-007: Restore từ ZIP bị corrupt 🔲

| | Nội dung |
|---|---------|
| **Pre** | File ZIP không hợp lệ (truncated hoặc không phải ZIP) |
| **Steps** | 1. Restore từ file corrupt |
| **Expected** | - Hiện thông báo lỗi rõ ràng<br>- Workspace không bị ảnh hưởng<br>- Không crash |

#### TC-EDGE-008: File .src không có .dat tương ứng ✅

| | Nội dung |
|---|---------|
| **Pre** | Thư mục có `Orphan.src` nhưng KHÔNG CÓ `Orphan.dat` |
| **Steps** | 1. Navigate đến thư mục (Module view) |
| **Expected** | - File vẫn hiện trong list (không bị ẩn)<br>- Module view có thể hiện nó như module hoặc single file<br>- Copy/Rename/Delete không crash khi thiếu .dat |

---

### Tổng kết Test Cases

| Section | Số TC | Đã implement (✅) | Chưa impl (🔲) |
|---------|-------|-------------------|-----------------|
| Navigation & Display | 8 | 8 | 0 |
| Program Selection | 5 | 5 | 0 |
| File Operations | 11 | 11 | 0 |
| Context Menu | 2 | 2 | 0 |
| FOLD System | 14 | 14 | 0 |
| Archive & Restore | 10 | 10 | 0 |
| Search & Filter | 3 | 3 | 0 |
| Read-Only & Protection | 2 | 2 | 0 |
| Auto-Refresh | 2 | 2 | 0 |
| Integration Tests | 3 | 3 | 0 |
| Edge Cases & Error | 8 | 7 | 1 |
| **Tổng** | **68** | **67** | **1** |

---

*Document generated: 2026-02-16*
*Manual test cases added: 2026-02-16*
*Based on: KUKA KSS 8.3/8.6 Programming Manual, System Integrator workflow, Project code audit*
