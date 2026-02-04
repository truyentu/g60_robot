# UI Design: Robot Model Catalog (Inspired by KUKA)

## Research Summary

Đã research các industrial robot software để hiểu cách họ quản lý robot models:

### KUKA WorkVisual

**Robot Catalog Selection Interface:**
- **Location:** File → New Project → Robot Catalog Selection
- **Layout Components:**
  - **Catalog Browser** (left panel): Hierarchical tree của robot models
  - **3D Preview Window** (center): Visual representation của selected robot
  - **Properties Panel** (right): Configuration parameters
  - **Project Tree** (top-left): Project structure navigation

**Robot Organization:**
- Grouped by series: KR AGILUS, KR QUANTEC, LBR iiwa, etc.
- Hierarchical categorization by payload and reach
- Visual thumbnails for quick identification

**Key Features:**
- Kinematic model configuration
- Tool and base frame setup
- Multi-robot coordination
- Import/export configurations

### ABB RobotStudio

**Adding Robots Workflow:**
1. Open **Modeling Tab**
2. Click **"ABB Library"** or **"Equipment"** button
3. Browse catalog or search for robot model
4. **Drag-and-drop** or click **"Add"** to insert into station

**Catalog Features:**
- Equipment browser with search function
- IRB series organization
- Visual model previews
- Integration with station workspace

### Common UI/UX Patterns in Industrial Automation

**Navigation Patterns:**
1. **Hierarchical Tree Navigation** - By manufacturer, type, application
2. **Breadcrumb Navigation** - Show location in catalog
3. **Tabbed Interface** - Specs, 3D models, docs
4. **Sidebar Filters** - Payload, reach, brand, application

**Catalog Display Patterns:**
1. **Card/Grid Layout** - Visual thumbnails
2. **List View** - Compact with key specs
3. **Comparison Mode** - Side-by-side models
4. **Quick Preview** - Hover/click for specs

**Visual Elements:**
1. **3D Model Viewer** - Interactive visualization
2. **Thumbnail Previews** - Quick identification
3. **Specification Tables** - Technical data
4. **Visual Indicators** - Icons for type/capabilities

**Search & Filter:**
1. **Advanced Search** - Multiple criteria
2. **Auto-complete** - Model suggestions
3. **Saved Filters** - Common searches
4. **Tag-based Navigation** - Application/feature tags

---

## Proposed Design for Robot Controller UI

### Option A: KUKA-Inspired Full Catalog View

**Location:** Separate page in navigation (index 5)

**Layout:**
```
┌─────────────────────────────────────────────────────────────┐
│ Navigation: Robot Catalog                                   │
├──────────────┬──────────────────────────┬──────────────────┤
│              │                          │                  │
│  CATALOG     │    3D PREVIEW            │   PROPERTIES     │
│  BROWSER     │    (HelixViewport)       │   PANEL          │
│              │                          │                  │
│ [Search]     │                          │ Name: KUKA...    │
│              │                          │ DOF: 6           │
│ > Generic    │     [Robot Model]        │ Payload: 10kg    │
│   - 6-axis   │                          │ Reach: 1420mm    │
│ > KUKA       │                          │                  │
│   - KR6      │                          │ DH Parameters:   │
│   - KR10 ✓   │                          │ [Table...]       │
│ > Universal  │                          │                  │
│   - UR5e     │                          │ Meshes: 7 files  │
│              │                          │                  │
│              │                          │ [Load Package]   │
│              │                          │ [Export Config]  │
└──────────────┴──────────────────────────┴──────────────────┘
```

**Features:**
- Left: Tree view catalog với grouping by manufacturer
- Center: Real-time 3D preview khi select model
- Right: Technical specs và actions
- Search/filter at top of catalog browser

### Option B: Simplified Modal/Dialog (Faster Implementation)

**Location:** Accessible via menu or toolbar button

**Layout:**
```
┌────────────────────────────────────────────────────┐
│  Select Robot Model                         [X]    │
├────────────────────────────────────────────────────┤
│                                                    │
│  [Search: ____________]  [Filter: All ▾]          │
│                                                    │
│  ┌──────────┬──────────┬──────────┐               │
│  │ Generic  │  KUKA    │ Universal│               │
│  │ 6-Axis   │  KR10 ✓  │  UR5e    │               │
│  │          │  KR6     │          │               │
│  │ [icon]   │ [icon]   │ [icon]   │               │
│  │ 1200mm   │ 1420mm   │ 850mm    │               │
│  │ 6kg      │ 10kg     │ 5kg      │               │
│  └──────────┴──────────┴──────────┘               │
│                                                    │
│  Selected: KUKA KR 10 R1420                       │
│  - Payload: 10 kg                                 │
│  - Reach: 1420 mm                                 │
│  - DOF: 6                                         │
│  - Meshes: Available (7 STL files)               │
│                                                    │
│               [Cancel]  [Load Package]            │
└────────────────────────────────────────────────────┘
```

**Features:**
- Card-based layout (similar to ABB)
- Quick visual comparison
- Minimal clicks to load
- Can be modal dialog or embedded panel

### Option C: Integrated Navigation Panel (Minimal Changes)

**Location:** Add to existing left navigation

**Layout:**
```
Navigation Panel:
├── Manual Jog
├── Program
├── I/O
├── Configuration
├── Diagnostics
└── Robot Catalog ← NEW
    └── Opens catalog browser in main content area
```

Main content area shows:
- List of available packages
- Details panel when selected
- Load button
- Same 3D viewport in background

---

## Recommendation: Phased Approach

### Phase 1 (IMMEDIATE - For Debug): Option C
**Why:**
- Minimal UI changes (just add navigation item)
- Reuse existing 3D viewport
- Focus on getting STL loading working
- Fast implementation (~1-2 hours)

**Implementation:**
1. Add "Robot Catalog" to navigation ListBox
2. Create simple catalog view page
3. Wire up to RobotPackageBrowserViewModel
4. Test STL loading with UI

### Phase 2 (FUTURE - Polish): Option A
**Why:**
- Professional KUKA-inspired interface
- Better UX for production use
- Advanced features (search, filter, compare)
- Matches industrial standards

**Implementation:**
1. Design full catalog browser with TreeView
2. Add separate 3D preview viewport
3. Implement search/filter functionality
4. Add export/import configs
5. Professional styling

---

## Immediate Action Plan

**To Resolve Current STL Loading Issue:**

1. ✅ **Add Robot Catalog to Navigation** (Quick Win)
   ```xaml
   <!-- MainWindow.xaml line 137 -->
   <ListBoxItem Content="Robot Catalog"/>
   ```

2. ✅ **Wire Up Page Content**
   ```xaml
   <!-- Add RobotPackageBrowser page -->
   <pages:RobotPackageBrowser DataContext="{Binding RobotCatalogViewModel}">
       <!-- Visibility based on SelectedNavIndex = 5 -->
   </pages:RobotPackageBrowser>
   ```

3. ✅ **Register ViewModel in DI**
   ```csharp
   // App.xaml.cs
   services.AddSingleton<RobotPackageBrowserViewModel>();
   ```

4. ✅ **Update MainViewModel**
   ```csharp
   // Add RobotCatalogViewModel property
   public RobotPackageBrowserViewModel RobotCatalogViewModel { get; }
   ```

5. **Test STL Loading**
   - Navigate to Robot Catalog
   - Click Refresh Packages
   - Select KUKA KR 10 R1420
   - Click Load Package
   - Check `stl_debug.log`

---

## UI Design Guidelines (Inspired by KUKA)

### Visual Style
- **Dark Theme:** #1E1E1E background (matching current viewport)
- **Accent Color:** Orange/Blue (KUKA brand colors)
- **Typography:** Clean, technical font (Segoe UI, Consolas for specs)
- **Icons:** Industrial, minimalist style

### Interaction Patterns
- **Single Click:** Select robot in catalog
- **Double Click:** Load robot immediately
- **Hover:** Show quick preview tooltip
- **Right Click:** Context menu (export, compare, details)

### Information Hierarchy
1. **Primary:** Robot name, manufacturer
2. **Secondary:** Payload, reach, DOF
3. **Tertiary:** DH parameters, mesh info
4. **Actions:** Load, export, compare buttons

### Responsive Layout
- **Minimum Width:** Support 1000px window
- **Resizable Panels:** Splitters between catalog/preview/properties
- **Collapsible Sections:** Allow hiding panels for more space

---

## References

Based on research of:
- KUKA WorkVisual robot catalog interface
- ABB RobotStudio equipment library
- Industrial automation UI/UX patterns
- CAD/PLM software navigation patterns

## Next Steps

1. **Implement Phase 1** (add navigation + basic catalog view)
2. **Debug STL loading** with UI access
3. **Iterate UI** based on user feedback
4. **Plan Phase 2** (full KUKA-inspired catalog) for future release

---

**Document Status:** Draft for discussion
**Created:** 2026-02-05
**Last Updated:** 2026-02-05
