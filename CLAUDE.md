# Robot Controller Project Rules

## Project Overview

| Item | Value |
|------|-------|
| Project | 6-DOF Robot Controller for MIG/MAG Welding |
| Architecture | C++ Core + C# WPF HMI + Teensy Firmware |
| IPC | ZeroMQ (C++ ↔ C#) |
| Status | Implementation Phase |

---

## Quick Navigation

| Cần tìm gì? | Đọc file nào? |
|-------------|---------------|
| Tổng quan project | docs/00_MASTER_ROADMAP.md |
| Kiến trúc hệ thống | docs/01_ARCHITECTURE_OVERVIEW.md |
| Task hiện tại | docs/plans/IMPL_PX_XX_*.md (được giao) |
| Research papers | docs/RESEARCH_INDEX.md |
| Progress tracking | docs/plans/IMPLEMENTATION_PLAN_TRACKER.md |

---

## Session Workflow (BẮT BUỘC)

### BƯỚC 1: Nhận Task
User sẽ giao task theo format: "Implement theo IMPL_PX_XX"

### BƯỚC 2: Load Context
```
1. Đọc file IMPL plan được giao
   → docs/plans/IMPL_PX_XX_*.md

2. Đọc TẤT CẢ files trong "Required Reading" của plan đó
   → Research papers trong ressearch_doc_md/
   → Core docs nếu có

3. KHÔNG đọc thêm files khác trừ khi thực sự cần
```

### BƯỚC 3: Code theo Plan
```
- Follow TỪNG STEP trong IMPL plan theo thứ tự
- Code PHẢI match với plan (không sáng tạo thêm)
- Validate sau mỗi step như plan yêu cầu
- Commit sau mỗi step hoặc nhóm steps
```

### BƯỚC 4: Khi gặp vấn đề kỹ thuật
```
QUAN TRỌNG: Research papers là source of truth đầu tiên!

IF vấn đề liên quan đến topic trong Required Reading:
   → Đọc lại paper đó kỹ hơn
   → Tìm code examples trong paper

ELSE IF vấn đề mới không có trong Required Reading:
   → Đọc docs/RESEARCH_INDEX.md
   → Tìm paper phù hợp theo keyword
   → Đọc paper đó

ELSE IF không có paper nào phù hợp:
   → Mới được search web / dùng kiến thức có sẵn
   → Ghi note lại để bổ sung vào research sau
```

### BƯỚC 5: Kết thúc session
```
- Report progress: Step nào đã xong, step nào chưa
- Nếu chưa xong plan → ghi note để session sau tiếp tục
- Update IMPLEMENTATION_PLAN_TRACKER.md nếu hoàn thành plan
```

---

## Coding Conventions

### C++ (Core Engine)
```cpp
// Naming
class PascalCaseClass {};
void camelCaseFunction();
int camelCaseVariable;
const int UPPER_CASE_CONSTANT = 42;

// Files
ClassName.hpp / ClassName.cpp

// Namespaces
namespace robot_controller { }
```

### C# (WPF HMI)
```csharp
// Naming - PascalCase everywhere
public class PascalCaseClass { }
public void PascalCaseMethod() { }
public int PascalCaseProperty { get; set; }

// Pattern: MVVM with CommunityToolkit.Mvvm
// ViewModels: [ObservableProperty], [RelayCommand]
```

### General Rules
- KHÔNG thêm features ngoài plan
- KHÔNG refactor code không liên quan
- KHÔNG thêm comments/docs cho code không thay đổi
- Follow IMPL plan như checklist

---

## Folder Structure

```
Robot_controller/
├── CLAUDE.md                 ← (file này)
├── docs/
│   ├── 00_MASTER_ROADMAP.md
│   ├── 01_ARCHITECTURE_OVERVIEW.md
│   ├── RESEARCH_INDEX.md     ← Index để tìm research papers
│   ├── phases/               ← Phase overview docs
│   ├── core_platform/        ← Technical specs
│   ├── modes/                ← Application modes
│   ├── testing/              ← Test plans
│   └── plans/                ← IMPL plans (executable)
├── ressearch_doc_md/         ← Research papers (Markdown)
├── ressearch_doc/            ← Research papers (PDF)
└── src/                      ← Source code (sẽ tạo)
    ├── cpp/                  ← C++ Core
    ├── csharp/               ← C# WPF HMI
    └── firmware/             ← Teensy firmware
```

---

## Research Papers Priority

Khi gặp vấn đề, ưu tiên đọc research papers theo thứ tự:

1. **Papers trong Required Reading của IMPL plan hiện tại**
2. **Papers tìm được qua RESEARCH_INDEX.md**
3. **Web search (chỉ khi không có paper phù hợp)**

KHÔNG được bỏ qua research papers để search web trực tiếp!

---

## Common Pitfalls (Tránh mắc phải)

| Pitfall | Đúng cách |
|---------|-----------|
| Đoán API của thư viện | Đọc research paper trước |
| Code khác với IMPL plan | Follow plan chính xác |
| Thêm features "hay ho" | Chỉ làm đúng yêu cầu |
| Skip Required Reading | Đọc đủ trước khi code |
| Search web ngay khi gặp lỗi | Check research papers trước |

---

## Integration với Superpowers /implement

Khi user sử dụng `/implement IMPL_PX_XX`:

### Trước khi code step đầu tiên:
```
1. Superpowers load IMPL plan
2. TÌM section "Required Reading" trong plan
3. ĐỌC TẤT CẢ files được list trong Required Reading
4. CHỈ SAU KHI đọc xong mới bắt đầu code
```

### Trong quá trình code:
```
1. Tạo file .impl_checkpoint.md (nếu chưa có)
2. Update checkpoint sau MỖI step hoàn thành
3. Checkpoint giúp resume nếu context bị compact
```

### Khi hoàn thành:
```
1. Mark tất cả steps = Done trong checkpoint
2. Rename checkpoint thành .impl_checkpoint_DONE.md
3. Update IMPLEMENTATION_PLAN_TRACKER.md
```

---

## Checkpoint System

### File: `.impl_checkpoint.md` (project root)

Checkpoint file được tạo khi bắt đầu `/implement` và update liên tục.

### Khi nào update:

| Trigger | Action |
|---------|--------|
| Bắt đầu plan mới | Tạo `.impl_checkpoint.md` từ template |
| Hoàn thành 1 step | Mark step = "Done", thêm notes |
| Bắt đầu step mới | Mark step = "In Progress" |
| Đổi file đang edit | Update "Last Working Context" |
| Gặp blocker/issue | Ghi vào "Notes" hoặc "Issues" |
| Hoàn thành plan | Rename → `.impl_checkpoint_IMPL_PX_XX_DONE.md` |

### Template checkpoint:

```markdown
# Implementation Checkpoint

## Current Task
| Field | Value |
|-------|-------|
| Plan | IMPL_PX_XX |
| Started | YYYY-MM-DD HH:MM |
| Last Updated | YYYY-MM-DD HH:MM |

## Progress
| Step | Status | Notes |
|------|--------|-------|
| Step 1: ... | Done/In Progress/Pending | |
| Step 2: ... | Pending | |

## Last Working Context
- File đang edit: `path/to/file`
- Function/Class đang làm: `name`
- Vấn đề đang giải quyết: (nếu có)

## Required Reading Status
- [x] Paper đã đọc
- [ ] Paper chưa đọc
```

---

## Resume sau Context Compact

### Khi context bị compact (tự động):

Claude Code sẽ tạo summary chứa thông tin session. Tuy nhiên summary có thể thiếu chi tiết.

### Workflow resume:

```
1. ĐẦU TIÊN: Đọc .impl_checkpoint.md
   → Xác định đang ở plan nào, step nào

2. Đọc lại IMPL plan (chỉ từ step hiện tại trở đi)
   → Không cần đọc lại steps đã Done

3. Đọc lại file đang edit (từ "Last Working Context")
   → Hiểu code đang viết dở

4. KHÔNG cần đọc lại Required Reading nếu đã đọc
   → Check "Required Reading Status" trong checkpoint

5. Tiếp tục code từ ĐÚNG CHỖ dừng
```

### Nếu user nói "tiếp tục" sau compact:

```
AI response:
1. "Đang đọc checkpoint..."
2. "Thấy đang ở IMPL_P2_02, Step 3: InverseKinematics"
3. "Đọc lại file src/cpp/kinematics/InverseKinematics.cpp..."
4. "Tiếp tục implement method solveIK()..."
```

### Nếu không có checkpoint file:

```
AI response:
1. "Không tìm thấy .impl_checkpoint.md"
2. "Bạn muốn bắt đầu implement plan nào?"
3. Hoặc: Đọc summary từ compact để đoán context
```
