# Design Document: AI Coder Navigation System

| Metadata | Value |
|----------|-------|
| Document | AI Coder Navigation System Design |
| Version | 1.0 |
| Status | Approved |
| Created | 2026-02-01 |
| Author | Claude (Brainstorming Session) |

---

## 1. Problem Statement

### 1.1 Context

Robot Controller project có:
- **55+ documentation files** phân bổ trong nhiều folders
- **15 research papers** chứa domain knowledge quan trọng
- **15 implementation plans** với executable steps

### 1.2 Challenges

| Challenge | Impact |
|-----------|--------|
| Context window giới hạn | Không thể load tất cả docs vào một session |
| Session-based workflow | Mỗi session mới, AI phải "học lại" từ đầu |
| Research papers bị bỏ qua | AI đoán/search web thay vì đọc papers đã có |
| AI bị "rối" | Không biết đọc file nào trước, file nào sau |

### 1.3 Goals

1. AI coder nhanh chóng hiểu project context khi bắt đầu session
2. AI ưu tiên đọc research papers trước khi search web
3. AI follow IMPL plan chính xác, không sáng tạo thêm
4. Giảm thiểu context usage nhưng vẫn đủ thông tin

---

## 2. Solution Architecture

### 2.1 3-Layer Context System

```
┌─────────────────────────────────────────────────────────────┐
│  LAYER 1: CLAUDE.md (Auto-loaded by Claude Code)           │
│  ┌───────────────────────────────────────────────────────┐  │
│  │ - Project overview (ngắn gọn)                         │  │
│  │ - Quick navigation table                              │  │
│  │ - Session workflow (5 bước bắt buộc)                  │  │
│  │ - Coding conventions                                  │  │
│  │ - Common pitfalls                                     │  │
│  └───────────────────────────────────────────────────────┘  │
│  Size: ~80 lines | Auto-load: Yes | Context: ~2KB          │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│  LAYER 2: IMPL Plan + Required Reading (Task-specific)     │
│  ┌───────────────────────────────────────────────────────┐  │
│  │ IMPL_PX_XX.md:                                        │  │
│  │ - Required Reading table (pre-mapped papers)          │  │
│  │ - Prerequisites                                       │  │
│  │ - Step-by-step implementation                         │  │
│  │ - Validation commands                                 │  │
│  └───────────────────────────────────────────────────────┘  │
│  Size: ~500 lines | Load: On task assignment | Context: ~15KB│
│                                                              │
│  Research Papers (từ Required Reading):                      │
│  ┌───────────────────────────────────────────────────────┐  │
│  │ - 2-3 papers liên quan đến task                       │  │
│  │ - Domain knowledge cần thiết                          │  │
│  │ - Code examples, API references                       │  │
│  └───────────────────────────────────────────────────────┘  │
│  Size: ~1500 lines | Load: Per task | Context: ~45KB        │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│  LAYER 3: Research Index (On-demand fallback)               │
│  ┌───────────────────────────────────────────────────────┐  │
│  │ RESEARCH_INDEX.md:                                    │  │
│  │ - Keyword → Paper mapping                             │  │
│  │ - Topic categories                                    │  │
│  │ - Paper summaries                                     │  │
│  └───────────────────────────────────────────────────────┘  │
│  Size: ~100 lines | Load: When needed | Context: ~3KB       │
│                                                              │
│  Fallback Paper:                                             │
│  ┌───────────────────────────────────────────────────────┐  │
│  │ - Paper tìm được qua index                            │  │
│  │ - Đọc khi gặp vấn đề không có trong Required Reading  │  │
│  └───────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
```

### 2.2 Context Usage Summary

| Scenario | Files Loaded | Estimated Context |
|----------|--------------|-------------------|
| Session start | CLAUDE.md | ~2KB |
| Task assigned | + IMPL plan | ~17KB |
| Reading papers | + 2-3 papers | ~62KB |
| Edge case | + Index + 1 paper | ~80KB |

**Conclusion:** ~80KB worst case, còn nhiều context cho code + conversation.

---

## 3. Component Details

### 3.1 Project CLAUDE.md

**Location:** `E:\DEV_CONTEXT_PROJECTs\Robot_controller\CLAUDE.md`

**Purpose:** Entry point cho AI coder, được auto-load bởi Claude Code.

**Contents:**

| Section | Purpose |
|---------|---------|
| Project Overview | Tóm tắt project trong 5 dòng |
| Quick Navigation | Table chỉ dẫn file nào cần đọc |
| Session Workflow | 5 bước bắt buộc khi bắt đầu session |
| Coding Conventions | C++ và C# naming, patterns |
| Common Pitfalls | Những lỗi cần tránh |

**Key Rules Enforced:**
1. Đọc IMPL plan trước khi code
2. Đọc Required Reading trước khi code
3. Research papers > Web search
4. Follow plan exactly, no creativity

### 3.2 Research Index

**Location:** `E:\DEV_CONTEXT_PROJECTs\Robot_controller\docs\RESEARCH_INDEX.md`

**Purpose:** Searchable index cho 15 research papers.

**Structure:**

```markdown
## Index by Topic

| Topic | Keywords | Paper | File |
|-------|----------|-------|------|
| Kinematics | IK, FK, DH | Robotics Library... | path/to/file.md |
```

**Features:**
- Keyword-based search
- Topic categories
- Direct file paths
- Paper summaries

### 3.3 Required Reading Section

**Location:** Đầu mỗi IMPL plan (15 files)

**Format:**

```markdown
## Required Reading (ĐỌC TRƯỚC KHI CODE)

| Priority | Document | Lý do |
|----------|----------|-------|
| P0 | `path/to/paper.md` | Reason to read |
| P1 | `path/to/another.md` | Another reason |
```

**Mapping Overview:**

| IMPL | Required Papers Count |
|------|----------------------|
| P1_01 | 1 paper |
| P1_02 | 1 paper |
| P1_03 | 0 (standard libs) |
| P1_04 | 2 papers |
| P2_01 | 1 paper |
| P2_02 | 1 paper |
| P2_03 | 1 paper |
| P2_04 | 1 paper |
| P2_05 | 1 paper |
| P3_01 | 2 papers |
| P3_02 | 1 paper |
| P3_03 | 2 papers |
| P4_01 | 2 (1 paper + 1 doc) |
| P4_02 | 3 papers |
| P4_03 | 2 papers |

---

## 4. Workflow Specification

### 4.1 Session Start Workflow

```
┌─────────────────────────────────────────────────────────────┐
│                    SESSION START                             │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│  Step 1: User giao task                                     │
│  "Implement theo IMPL_P4_01"                                │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│  Step 2: AI đọc IMPL plan                                   │
│  → docs/plans/IMPL_P4_01_SensorDrivers.md                   │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│  Step 3: AI đọc Required Reading                            │
│  → ressearch_doc_md/Robot Hàn_ Cảm Biến Laser...md          │
│  → docs/core_platform/CORE_02_IPC_Layer.md                  │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│  Step 4: AI bắt đầu code theo steps                         │
│  → Follow IMPL plan step-by-step                            │
│  → Validate after each step                                 │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│  Step 5: Session end                                        │
│  → Report progress                                          │
│  → Note remaining steps                                     │
└─────────────────────────────────────────────────────────────┘
```

### 4.2 Problem-Solving Workflow

```
┌─────────────────────────────────────────────────────────────┐
│                    GẶP VẤN ĐỀ KỸ THUẬT                      │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│  Decision 1: Vấn đề có trong Required Reading?              │
│  ├── YES → Đọc lại paper đó kỹ hơn                         │
│  └── NO → Continue to Decision 2                            │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│  Decision 2: Đọc RESEARCH_INDEX.md                          │
│  ├── Found matching paper → Đọc paper đó                   │
│  └── No matching paper → Continue to Decision 3             │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│  Decision 3: Web search / Internal knowledge                │
│  → Chỉ đến bước này mới được search web                    │
│  → Ghi note để bổ sung vào research sau                    │
└─────────────────────────────────────────────────────────────┘
```

---

## 5. File Structure

### 5.1 New Files Created

```
Robot_controller/
├── CLAUDE.md                           ← NEW (Project rules)
└── docs/
    ├── RESEARCH_INDEX.md               ← NEW (Paper index)
    └── plans/
        └── 2026-02-01-ai-coder-navigation-design.md  ← NEW (This document)
```

### 5.2 Modified Files

```
docs/plans/
├── IMPL_P1_01_ProjectSetup.md          ← Added Required Reading
├── IMPL_P1_02_IpcLayer.md              ← Added Required Reading
├── IMPL_P1_03_ConfigLogging.md         ← Added Required Reading
├── IMPL_P1_04_HmiVisualization.md      ← Added Required Reading
├── IMPL_P2_01_StateMachine.md          ← Added Required Reading
├── IMPL_P2_02_Kinematics.md            ← Added Required Reading
├── IMPL_P2_03_Trajectory.md            ← Added Required Reading
├── IMPL_P2_04_FirmwareComm.md          ← Added Required Reading
├── IMPL_P2_05_MotionHMI.md             ← Added Required Reading
├── IMPL_P3_01_WeldingSequencer.md      ← Added Required Reading
├── IMPL_P3_02_WeavingPatterns.md       ← Added Required Reading
├── IMPL_P3_03_WeldingHMI.md            ← Added Required Reading
├── IMPL_P4_01_SensorDrivers.md         ← Added Required Reading
├── IMPL_P4_02_SeamDetection.md         ← Added Required Reading
└── IMPL_P4_03_VisionHMI.md             ← Added Required Reading
```

---

## 6. Usage Examples

### 6.1 Example: Starting a New Session

```
User: Implement theo IMPL_P2_02

AI Response:
1. Đọc IMPL_P2_02_Kinematics.md...
2. Required Reading: Robotics Library_ Robot Tùy Chỉnh & IK Giải Tích.md
3. Đang đọc research paper...
4. Bắt đầu Step 1: Tạo KinematicsTypes.hpp...
```

### 6.2 Example: Encountering a Problem

```
AI (during coding):
Gặp vấn đề: Không rõ cách handle singularity trong IK...

AI Process:
1. Check Required Reading → Paper có section về singularity
2. Đọc lại section "Singularity Handling" trong paper
3. Apply solution từ paper

(Không cần search web)
```

### 6.3 Example: Problem Not in Required Reading

```
AI (during coding):
Gặp vấn đề: Cần biết về point cloud filtering...

AI Process:
1. Check Required Reading → Không có
2. Đọc RESEARCH_INDEX.md
3. Tìm thấy: "Scan-to-Path" paper có keyword "point cloud"
4. Đọc paper đó
5. Apply solution

(Vẫn không cần search web)
```

---

## 7. Maintenance

### 7.1 When to Update

| Trigger | Action |
|---------|--------|
| Thêm research paper mới | Update RESEARCH_INDEX.md |
| Tạo IMPL plan mới | Thêm Required Reading section |
| Paper content thay đổi | Review affected IMPL plans |
| New coding convention | Update CLAUDE.md |

### 7.2 Review Checklist

- [ ] CLAUDE.md còn phù hợp với project structure?
- [ ] RESEARCH_INDEX.md có đủ tất cả papers?
- [ ] Required Reading mappings còn chính xác?
- [ ] Workflow steps còn hợp lý?

---

## 8. Success Metrics

| Metric | Target | How to Measure |
|--------|--------|----------------|
| AI đọc đúng papers | 100% | Check AI logs |
| Web search usage | < 10% of sessions | Count web searches |
| IMPL plan compliance | 100% | Code review |
| Session onboarding time | < 2 min | Observation |

---

## 9. Appendix

### A. Research Papers Inventory

| # | Paper Name | Primary Topic | File Size |
|---|------------|---------------|-----------|
| 1 | PROJECT BLUEPRINT | Architecture | Large |
| 2 | Tìm kiếm nền tảng | Platform comparison | Medium |
| 3 | Robotics Library IK | Kinematics | Large |
| 4 | Tích hợp Ruckig | Trajectory | Medium |
| 5 | Tối ưu grblHAL | Firmware | Medium |
| 6 | Thiết Kế FSM | State Machine | Medium |
| 7 | Thiết kế HMI KUKA | HMI | Large |
| 8 | Thiết Kế Mô Phỏng | 3D Simulation | Medium |
| 9 | Module Hàn MIG_MAG | Welding Control | Large |
| 10 | Lập Trình Hàn | Welding Programming | Medium |
| 11 | Cảm Biến Laser | Vision Sensors | Large |
| 12 | Scan-to-Path | Point Cloud | Medium |
| 13 | Tái tạo 3D | 3D Reconstruction | Medium |
| 14 | Pick & Place | Mode Switching | Small |
| 15 | OpenCASCADE Tube | CAD Processing | Medium |

### B. CLAUDE.md Hierarchy

```
Global CLAUDE.md (C:\Users\brand\.claude\CLAUDE.md)
│
│   Contains:
│   - CAD/CAM general rules
│   - Robot/Automation rules
│   - Debug rules (RULE 1-11)
│   - Known bug patterns
│
└──► Project CLAUDE.md (Robot_controller\CLAUDE.md)
    │
    │   Contains:
    │   - Project-specific overview
    │   - Navigation table
    │   - Session workflow
    │   - Coding conventions
    │
    └──► (Claude Code merges both automatically)
```

### C. Superpowers Integration

Project sử dụng `superpowers` plugin cho `/implement` skill. Integration được thực hiện qua CLAUDE.md instructions thay vì tạo skill riêng.

**Workflow khi dùng `/implement`:**

```
User: /implement IMPL_P2_02
         │
         ▼
┌─────────────────────────────────────────┐
│  Superpowers loads IMPL plan            │
└─────────────────────────────────────────┘
         │
         ▼
┌─────────────────────────────────────────┐
│  CLAUDE.md instructs AI:                │
│  → Đọc Required Reading trước khi code  │
│  → Tạo checkpoint file                  │
│  → Update checkpoint sau mỗi step       │
└─────────────────────────────────────────┘
         │
         ▼
┌─────────────────────────────────────────┐
│  AI executes plan with checkpointing    │
└─────────────────────────────────────────┘
```

### D. Checkpoint System

**File:** `.impl_checkpoint.md` (project root)

**Purpose:** Track implementation progress, survive context compact

**Template:** `.impl_checkpoint_TEMPLATE.md`

**Lifecycle:**

```
Start implement     → Create .impl_checkpoint.md
Complete step       → Update Progress table
Context compact     → File survives on disk
Resume session      → Read checkpoint, continue
Complete plan       → Rename to _DONE.md
```

**Key sections trong checkpoint:**

| Section | Purpose |
|---------|---------|
| Current Task | Plan ID, timestamps |
| Progress | Step-by-step status tracking |
| Last Working Context | File, function đang edit |
| Issues/Blockers | Vấn đề đang gặp |
| Required Reading Status | Papers đã đọc |
| Session Notes | Notes mỗi session |

### E. Resume Workflow

Khi context bị compact hoặc session mới:

```
1. Đọc .impl_checkpoint.md
2. Xác định step hiện tại (In Progress)
3. Đọc IMPL plan từ step đó
4. Đọc file đang edit (từ Last Working Context)
5. Skip Required Reading nếu đã đọc
6. Tiếp tục code
```

---

## 10. Document History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2026-02-01 | Claude | Initial version |
| 1.1 | 2026-02-01 | Claude | Added Superpowers Integration + Checkpoint System |
