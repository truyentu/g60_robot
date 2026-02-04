# CORE_08: HMI Framework

## Document Info
| Item | Value |
|------|-------|
| **Module** | Human-Machine Interface Framework |
| **Layer** | UI Layer (C# WPF) |
| **Priority** | P1 - Foundation |
| **Dependencies** | CORE_01 (Project Setup), CORE_02 (IPC) |
| **Last Updated** | 2026-02-01 |

---

## 1. Overview

### 1.1. Purpose
Xây dựng framework HMI (Human-Machine Interface) cho Robot Controller, tái hiện trải nghiệm người dùng của thiết bị dạy lệnh công nghiệp **KUKA KRC4 smartPAD** trên nền tảng PC/WPF.

### 1.2. Design Goals
| Goal | Description |
|------|-------------|
| **Industrial UX** | Giao diện tối ưu cho màn hình cảm ứng Full HD trong môi trường nhà xưởng |
| **MVVM Pattern** | Tách biệt hoàn toàn UI, Logic, và Data |
| **Touch-First** | Kích thước nút tối thiểu 48x48px, hỗ trợ găng tay bảo hộ |
| **Real-time** | Cập nhật trạng thái robot tần suất cao (30-60 Hz) |
| **Extensible** | Dễ dàng thêm views mới cho các mode (Welding, Pick&Place) |

### 1.3. Architecture Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                    HMI FRAMEWORK ARCHITECTURE                    │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │                        VIEWS (XAML)                         │ │
│  │  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐       │ │
│  │  │  Main    │ │   Jog    │ │ Program  │ │  3D View │       │ │
│  │  │  Shell   │ │   View   │ │   View   │ │  (Helix) │       │ │
│  │  └────┬─────┘ └────┬─────┘ └────┬─────┘ └────┬─────┘       │ │
│  └───────┼────────────┼────────────┼────────────┼─────────────┘ │
│          │            │            │            │                │
│          └────────────┴────────────┴────────────┘                │
│                              │                                   │
│                        Data Binding                              │
│                              │                                   │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │                     VIEWMODELS (C#)                         │ │
│  │  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐       │ │
│  │  │  Main    │ │   Jog    │ │ Program  │ │ Robot3D  │       │ │
│  │  │ViewModel │ │ViewModel │ │ViewModel │ │ViewModel │       │ │
│  │  └────┬─────┘ └────┬─────┘ └────┬─────┘ └────┬─────┘       │ │
│  └───────┼────────────┼────────────┼────────────┼─────────────┘ │
│          │            │            │            │                │
│          └────────────┴────────────┴────────────┘                │
│                              │                                   │
│                         Services                                 │
│                              │                                   │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │                     SERVICES (C#)                           │ │
│  │  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐       │ │
│  │  │   IPC    │ │  Config  │ │  Robot   │ │Navigation│       │ │
│  │  │  Client  │ │ Service  │ │  Model   │ │ Service  │       │ │
│  │  └──────────┘ └──────────┘ └──────────┘ └──────────┘       │ │
│  └────────────────────────────────────────────────────────────┘ │
│                              │                                   │
│                         ZeroMQ                                   │
│                              │                                   │
│                      ┌───────▼───────┐                          │
│                      │   C++ Core    │                          │
│                      └───────────────┘                          │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

---

## 2. Screen Layout Design

### 2.1. KUKA-Style Layout Zones

Màn hình Full HD (1920x1080) được chia thành các vùng chức năng cố định:

```
┌─────────────────────────────────────────────────────────────────┐
│                        STATUS BAR (60px)                         │
│  [Mode: T1] [Motor: ON] [Override: 50%]      [Time] [Connection]│
├────┬────────────────────────────────────────────────────┬───────┤
│    │                                                    │       │
│ L  │                                                    │   R   │
│ E  │                                                    │   I   │
│ F  │                  MAIN CONTENT                      │   G   │
│ T  │                  (Dynamic Area)                    │   H   │
│    │                                                    │   T   │
│ S  │               - 3D Viewport                        │       │
│ I  │               - Program Editor                     │   S   │
│ D  │               - Jog Panel                          │   O   │
│ E  │               - Config Views                       │   F   │
│ B  │                                                    │   T   │
│ A  │                                                    │   K   │
│ R  │                                                    │   E   │
│    │                                                    │   Y   │
│(60)│                  (Flexible)                        │   S   │
│    │                                                    │ (100) │
│    │                                                    │       │
├────┴────────────────────────────────────────────────────┴───────┤
│                       MESSAGE WINDOW (100px)                     │
│  [!] Warning: Joint 2 approaching limit (±5°)                   │
├─────────────────────────────────────────────────────────────────┤
│                      SMARTKEYS BAR (100px)                       │
│  [Motion] [Logic] [Tool] [TouchUp] [Delete] [Copy] [Paste]     │
└─────────────────────────────────────────────────────────────────┘
```

### 2.2. Layout Specifications

| Zone | Position | Size | Purpose | Style |
|------|----------|------|---------|-------|
| **Status Bar** | Top | H: 60px | Mode, Motor, Override, Clock | Dark gray (#333) |
| **Left Sidebar** | Left | W: 60px | Navigation menu (hamburger) | Gray (#555) |
| **Main Content** | Center | Flexible | Working area | Light/Dark theme |
| **Right Softkeys** | Right | W: 100px | Context-sensitive buttons | Gray, 3D effect |
| **Message Window** | Bottom overlay | H: 100px | Errors, warnings | Transparent/Red/Yellow |
| **SmartKeys Bar** | Bottom | H: 100px | Direct action buttons | Large buttons 80x80px |

### 2.3. XAML Layout Structure

```xml
<!-- MainWindow.xaml -->
<Window x:Class="RobotUI.MainWindow"
        xmlns="http://schemas.microsoft.com/winfx/2006/xaml/presentation"
        xmlns:x="http://schemas.microsoft.com/winfx/2006/xaml"
        xmlns:local="clr-namespace:RobotUI"
        Title="Robot Controller"
        WindowState="Maximized"
        WindowStyle="None">

    <Grid>
        <Grid.RowDefinitions>
            <RowDefinition Height="60"/>      <!-- Status Bar -->
            <RowDefinition Height="*"/>        <!-- Main Content -->
            <RowDefinition Height="Auto"/>     <!-- Message Window -->
            <RowDefinition Height="100"/>      <!-- SmartKeys -->
        </Grid.RowDefinitions>

        <!-- Status Bar -->
        <Border Grid.Row="0" Background="{StaticResource StatusBarBrush}">
            <local:StatusBarView DataContext="{Binding StatusViewModel}"/>
        </Border>

        <!-- Middle Section -->
        <Grid Grid.Row="1">
            <Grid.ColumnDefinitions>
                <ColumnDefinition Width="60"/>   <!-- Left Sidebar -->
                <ColumnDefinition Width="*"/>     <!-- Main Content -->
                <ColumnDefinition Width="100"/>   <!-- Right Softkeys -->
            </Grid.ColumnDefinitions>

            <!-- Left Sidebar -->
            <Border Grid.Column="0" Background="{StaticResource SidebarBrush}">
                <local:NavigationSidebarView/>
            </Border>

            <!-- Main Content (Dynamic) -->
            <ContentControl Grid.Column="1"
                            Content="{Binding CurrentView}"/>

            <!-- Right Softkeys -->
            <Border Grid.Column="2" Background="{StaticResource SoftKeysBrush}">
                <local:SoftKeysView DataContext="{Binding SoftKeysViewModel}"/>
            </Border>
        </Grid>

        <!-- Message Window (Overlay) -->
        <Border Grid.Row="2"
                Background="{Binding MessageBackground}"
                Visibility="{Binding HasMessages, Converter={StaticResource BoolToVis}}">
            <local:MessageWindowView DataContext="{Binding MessageViewModel}"/>
        </Border>

        <!-- SmartKeys Bar -->
        <Border Grid.Row="3" Background="{StaticResource SmartKeysBrush}">
            <local:SmartKeysBarView DataContext="{Binding SmartKeysViewModel}"/>
        </Border>
    </Grid>
</Window>
```

---

## 3. MVVM Architecture

### 3.1. Project Structure

```
📁 RobotUI/
├── 📄 App.xaml
├── 📄 App.xaml.cs                    # DI Configuration
├── 📄 MainWindow.xaml
│
├── 📁 Views/                          # XAML Views
│   ├── 📁 Shell/
│   │   ├── 📄 StatusBarView.xaml
│   │   ├── 📄 NavigationSidebarView.xaml
│   │   ├── 📄 SoftKeysView.xaml
│   │   ├── 📄 SmartKeysBarView.xaml
│   │   └── 📄 MessageWindowView.xaml
│   │
│   ├── 📁 Main/
│   │   ├── 📄 HomeView.xaml
│   │   ├── 📄 JogView.xaml
│   │   ├── 📄 ProgramEditorView.xaml
│   │   └── 📄 ConfigView.xaml
│   │
│   ├── 📁 Visualization/
│   │   ├── 📄 Robot3DView.xaml
│   │   ├── 📄 CoordinateDisplayView.xaml
│   │   └── 📄 TrailRendererView.xaml
│   │
│   └── 📁 Welding/
│       ├── 📄 WeldingPanelView.xaml
│       └── 📄 WeldScopeView.xaml
│
├── 📁 ViewModels/                     # ViewModels
│   ├── 📄 ViewModelBase.cs
│   ├── 📄 MainViewModel.cs
│   ├── 📄 StatusViewModel.cs
│   ├── 📄 JogViewModel.cs
│   ├── 📄 ProgramViewModel.cs
│   ├── 📄 Robot3DViewModel.cs
│   └── 📄 ConfigViewModel.cs
│
├── 📁 Models/                         # Data Models
│   ├── 📄 RobotState.cs
│   ├── 📄 JointPosition.cs
│   ├── 📄 CartesianPose.cs
│   ├── 📄 ProgramCommand.cs
│   └── 📄 AlarmInfo.cs
│
├── 📁 Services/                       # Business Services
│   ├── 📄 IIPCClient.cs
│   ├── 📄 IPCClient.cs
│   ├── 📄 IConfigService.cs
│   ├── 📄 ConfigService.cs
│   ├── 📄 INavigationService.cs
│   ├── 📄 NavigationService.cs
│   ├── 📄 IRobotModelService.cs
│   └── 📄 RobotModelService.cs
│
├── 📁 Controls/                       # Custom Controls
│   ├── 📄 JogButton.cs
│   ├── 📄 InlineFormControl.cs
│   ├── 📄 CoordinateDisplay.cs
│   └── 📄 OverrideSlider.cs
│
├── 📁 Converters/                     # Value Converters
│   ├── 📄 BoolToVisibilityConverter.cs
│   ├── 📄 StateToColorConverter.cs
│   └── 📄 JointAngleFormatter.cs
│
├── 📁 Themes/                         # Resource Dictionaries
│   ├── 📄 Colors.xaml
│   ├── 📄 Brushes.xaml
│   ├── 📄 Styles.xaml
│   ├── 📄 IndustrialDarkTheme.xaml
│   └── 📄 IndustrialLightTheme.xaml
│
└── 📁 Assets/                         # Static Resources
    ├── 📁 Icons/
    ├── 📁 Models3D/
    └── 📁 Fonts/
```

### 3.2. ViewModelBase Implementation

```csharp
// ViewModels/ViewModelBase.cs
using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;

namespace RobotUI.ViewModels;

public abstract partial class ViewModelBase : ObservableObject
{
    [ObservableProperty]
    private bool _isBusy;

    [ObservableProperty]
    private string? _errorMessage;

    protected virtual void OnInitialize() { }
    protected virtual void OnCleanup() { }
}
```

### 3.3. MainViewModel with Navigation

```csharp
// ViewModels/MainViewModel.cs
using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using RobotUI.Services;

namespace RobotUI.ViewModels;

public partial class MainViewModel : ViewModelBase
{
    private readonly INavigationService _navigationService;
    private readonly IIPCClient _ipcClient;

    [ObservableProperty]
    private ViewModelBase? _currentView;

    [ObservableProperty]
    private StatusViewModel _statusViewModel;

    [ObservableProperty]
    private SoftKeysViewModel _softKeysViewModel;

    [ObservableProperty]
    private SmartKeysViewModel _smartKeysViewModel;

    [ObservableProperty]
    private MessageViewModel _messageViewModel;

    public MainViewModel(
        INavigationService navigationService,
        IIPCClient ipcClient,
        StatusViewModel statusViewModel,
        SoftKeysViewModel softKeysViewModel,
        SmartKeysViewModel smartKeysViewModel,
        MessageViewModel messageViewModel)
    {
        _navigationService = navigationService;
        _ipcClient = ipcClient;
        _statusViewModel = statusViewModel;
        _softKeysViewModel = softKeysViewModel;
        _smartKeysViewModel = smartKeysViewModel;
        _messageViewModel = messageViewModel;

        // Subscribe to navigation changes
        _navigationService.CurrentViewChanged += OnCurrentViewChanged;

        // Navigate to home on startup
        _navigationService.NavigateTo<HomeViewModel>();
    }

    private void OnCurrentViewChanged(object? sender, ViewModelBase viewModel)
    {
        CurrentView = viewModel;

        // Update context-sensitive softkeys
        SoftKeysViewModel.UpdateContext(viewModel.GetType());
    }

    [RelayCommand]
    private void NavigateToJog() => _navigationService.NavigateTo<JogViewModel>();

    [RelayCommand]
    private void NavigateToProgram() => _navigationService.NavigateTo<ProgramViewModel>();

    [RelayCommand]
    private void NavigateToConfig() => _navigationService.NavigateTo<ConfigViewModel>();
}
```

### 3.4. Navigation Service

```csharp
// Services/NavigationService.cs
using Microsoft.Extensions.DependencyInjection;

namespace RobotUI.Services;

public interface INavigationService
{
    event EventHandler<ViewModelBase>? CurrentViewChanged;
    void NavigateTo<TViewModel>() where TViewModel : ViewModelBase;
    void NavigateTo(Type viewModelType);
    void GoBack();
    bool CanGoBack { get; }
}

public class NavigationService : INavigationService
{
    private readonly IServiceProvider _serviceProvider;
    private readonly Stack<ViewModelBase> _navigationStack = new();

    public event EventHandler<ViewModelBase>? CurrentViewChanged;

    public bool CanGoBack => _navigationStack.Count > 1;

    public NavigationService(IServiceProvider serviceProvider)
    {
        _serviceProvider = serviceProvider;
    }

    public void NavigateTo<TViewModel>() where TViewModel : ViewModelBase
    {
        NavigateTo(typeof(TViewModel));
    }

    public void NavigateTo(Type viewModelType)
    {
        var viewModel = (ViewModelBase)_serviceProvider.GetRequiredService(viewModelType);
        _navigationStack.Push(viewModel);
        CurrentViewChanged?.Invoke(this, viewModel);
    }

    public void GoBack()
    {
        if (!CanGoBack) return;

        _navigationStack.Pop();
        var previousViewModel = _navigationStack.Peek();
        CurrentViewChanged?.Invoke(this, previousViewModel);
    }
}
```

---

## 4. Industrial Theme System

### 4.1. Color Palette

```xml
<!-- Themes/Colors.xaml -->
<ResourceDictionary xmlns="http://schemas.microsoft.com/winfx/2006/xaml/presentation"
                    xmlns:x="http://schemas.microsoft.com/winfx/2006/xaml">

    <!-- Primary Colors -->
    <Color x:Key="PrimaryDark">#2D2D30</Color>
    <Color x:Key="PrimaryMedium">#3E3E42</Color>
    <Color x:Key="PrimaryLight">#555555</Color>

    <!-- Accent Colors (KUKA Orange) -->
    <Color x:Key="AccentOrange">#FF5E00</Color>
    <Color x:Key="AccentOrangeHover">#FF7A2E</Color>
    <Color x:Key="AccentOrangePressed">#CC4B00</Color>

    <!-- Status Colors (Safety-Compliant) -->
    <Color x:Key="StatusGreen">#28A745</Color>     <!-- Ready, Motor ON -->
    <Color x:Key="StatusRed">#DC3545</Color>       <!-- Fault, E-Stop -->
    <Color x:Key="StatusYellow">#FFC107</Color>    <!-- Warning, T1 Mode -->
    <Color x:Key="StatusGray">#6C757D</Color>      <!-- Disabled -->
    <Color x:Key="StatusBlue">#007BFF</Color>      <!-- Info, Running -->

    <!-- Text Colors -->
    <Color x:Key="TextPrimary">#FFFFFF</Color>
    <Color x:Key="TextSecondary">#B0B0B0</Color>
    <Color x:Key="TextDisabled">#666666</Color>

    <!-- Background Colors -->
    <Color x:Key="BackgroundDark">#1E1E1E</Color>
    <Color x:Key="BackgroundMedium">#2D2D30</Color>
    <Color x:Key="BackgroundLight">#3E3E42</Color>

</ResourceDictionary>
```

### 4.2. Industrial Button Styles

```xml
<!-- Themes/Styles.xaml -->
<ResourceDictionary xmlns="http://schemas.microsoft.com/winfx/2006/xaml/presentation"
                    xmlns:x="http://schemas.microsoft.com/winfx/2006/xaml">

    <!-- Base Industrial Button -->
    <Style x:Key="IndustrialButtonStyle" TargetType="Button">
        <Setter Property="MinWidth" Value="48"/>
        <Setter Property="MinHeight" Value="48"/>
        <Setter Property="FontSize" Value="14"/>
        <Setter Property="FontWeight" Value="SemiBold"/>
        <Setter Property="Foreground" Value="{StaticResource TextPrimaryBrush}"/>
        <Setter Property="Background" Value="{StaticResource PrimaryLightBrush}"/>
        <Setter Property="BorderBrush" Value="{StaticResource PrimaryMediumBrush}"/>
        <Setter Property="BorderThickness" Value="1"/>
        <Setter Property="Padding" Value="12,8"/>
        <Setter Property="Cursor" Value="Hand"/>
        <Setter Property="Template">
            <Setter.Value>
                <ControlTemplate TargetType="Button">
                    <Border x:Name="border"
                            Background="{TemplateBinding Background}"
                            BorderBrush="{TemplateBinding BorderBrush}"
                            BorderThickness="{TemplateBinding BorderThickness}"
                            CornerRadius="4">
                        <ContentPresenter HorizontalAlignment="Center"
                                          VerticalAlignment="Center"/>
                    </Border>
                    <ControlTemplate.Triggers>
                        <Trigger Property="IsMouseOver" Value="True">
                            <Setter TargetName="border"
                                    Property="Background"
                                    Value="{StaticResource AccentOrangeBrush}"/>
                        </Trigger>
                        <Trigger Property="IsPressed" Value="True">
                            <Setter TargetName="border"
                                    Property="Background"
                                    Value="{StaticResource AccentOrangePressedBrush}"/>
                        </Trigger>
                        <Trigger Property="IsEnabled" Value="False">
                            <Setter TargetName="border"
                                    Property="Background"
                                    Value="{StaticResource StatusGrayBrush}"/>
                            <Setter Property="Foreground"
                                    Value="{StaticResource TextDisabledBrush}"/>
                        </Trigger>
                    </ControlTemplate.Triggers>
                </ControlTemplate>
            </Setter.Value>
        </Setter>
    </Style>

    <!-- SmartKey Button (Large, Touch-Friendly) -->
    <Style x:Key="SmartKeyButtonStyle" TargetType="Button"
           BasedOn="{StaticResource IndustrialButtonStyle}">
        <Setter Property="Width" Value="80"/>
        <Setter Property="Height" Value="80"/>
        <Setter Property="FontSize" Value="12"/>
        <Setter Property="Margin" Value="4"/>
    </Style>

    <!-- Jog Button (Hold-to-Run) -->
    <Style x:Key="JogButtonStyle" TargetType="RepeatButton">
        <Setter Property="Width" Value="60"/>
        <Setter Property="Height" Value="60"/>
        <Setter Property="FontSize" Value="18"/>
        <Setter Property="FontWeight" Value="Bold"/>
        <Setter Property="Delay" Value="100"/>
        <Setter Property="Interval" Value="50"/>
        <Setter Property="Background" Value="{StaticResource PrimaryLightBrush}"/>
        <Setter Property="Foreground" Value="{StaticResource TextPrimaryBrush}"/>
        <Setter Property="Template">
            <Setter.Value>
                <ControlTemplate TargetType="RepeatButton">
                    <Border x:Name="border"
                            Background="{TemplateBinding Background}"
                            BorderBrush="{StaticResource AccentOrangeBrush}"
                            BorderThickness="2"
                            CornerRadius="4">
                        <ContentPresenter HorizontalAlignment="Center"
                                          VerticalAlignment="Center"/>
                    </Border>
                    <ControlTemplate.Triggers>
                        <Trigger Property="IsPressed" Value="True">
                            <Setter TargetName="border"
                                    Property="Background"
                                    Value="{StaticResource StatusGreenBrush}"/>
                        </Trigger>
                    </ControlTemplate.Triggers>
                </ControlTemplate>
            </Setter.Value>
        </Setter>
    </Style>

</ResourceDictionary>
```

### 4.3. Typography Standards

```xml
<!-- Themes/Typography.xaml -->
<ResourceDictionary xmlns="http://schemas.microsoft.com/winfx/2006/xaml/presentation"
                    xmlns:x="http://schemas.microsoft.com/winfx/2006/xaml">

    <!-- Font Family -->
    <FontFamily x:Key="PrimaryFont">Segoe UI</FontFamily>
    <FontFamily x:Key="MonospaceFont">Consolas</FontFamily>

    <!-- Text Styles -->
    <Style x:Key="HeaderTextStyle" TargetType="TextBlock">
        <Setter Property="FontFamily" Value="{StaticResource PrimaryFont}"/>
        <Setter Property="FontSize" Value="24"/>
        <Setter Property="FontWeight" Value="Bold"/>
        <Setter Property="Foreground" Value="{StaticResource TextPrimaryBrush}"/>
    </Style>

    <Style x:Key="SubHeaderTextStyle" TargetType="TextBlock">
        <Setter Property="FontFamily" Value="{StaticResource PrimaryFont}"/>
        <Setter Property="FontSize" Value="18"/>
        <Setter Property="FontWeight" Value="SemiBold"/>
        <Setter Property="Foreground" Value="{StaticResource TextPrimaryBrush}"/>
    </Style>

    <Style x:Key="BodyTextStyle" TargetType="TextBlock">
        <Setter Property="FontFamily" Value="{StaticResource PrimaryFont}"/>
        <Setter Property="FontSize" Value="14"/>
        <Setter Property="Foreground" Value="{StaticResource TextSecondaryBrush}"/>
    </Style>

    <!-- Coordinate Display (Monospace, Large) -->
    <Style x:Key="CoordinateTextStyle" TargetType="TextBlock">
        <Setter Property="FontFamily" Value="{StaticResource MonospaceFont}"/>
        <Setter Property="FontSize" Value="20"/>
        <Setter Property="FontWeight" Value="Bold"/>
        <Setter Property="Foreground" Value="{StaticResource AccentOrangeBrush}"/>
    </Style>

    <!-- Joint Angle Display -->
    <Style x:Key="JointAngleTextStyle" TargetType="TextBlock">
        <Setter Property="FontFamily" Value="{StaticResource MonospaceFont}"/>
        <Setter Property="FontSize" Value="16"/>
        <Setter Property="Foreground" Value="{StaticResource TextPrimaryBrush}"/>
        <Setter Property="Width" Value="80"/>
        <Setter Property="TextAlignment" Value="Right"/>
    </Style>

</ResourceDictionary>
```

---

## 5. 3D Visualization (Helix Toolkit)

### 5.1. Technology Selection

| Option | Pros | Cons | Decision |
|--------|------|------|----------|
| **HelixToolkit.Wpf** | Easy to use | Low performance | ❌ |
| **HelixToolkit.Wpf.SharpDX** | High performance (DirectX 11) | More complex | ✅ **Selected** |

### 5.2. Robot 3D View Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    ROBOT 3D SCENEGRAPH                           │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  Viewport3DX                                                     │
│  ├── DefaultLights                                               │
│  │   ├── AmbientLight (Color: #404040)                          │
│  │   └── DirectionalLight (Direction: -1,-1,-1)                 │
│  │                                                               │
│  ├── CoordinateSystemModel3D (World Frame)                      │
│  │                                                               │
│  ├── GroupModel3D (RobotAssembly)                               │
│  │   ├── MeshGeometryModel3D (Base - Fixed)                     │
│  │   └── GroupModel3D (Joint1Container)                         │
│  │       ├── MeshGeometryModel3D (Link1)                        │
│  │       └── GroupModel3D (Joint2Container)                     │
│  │           ├── MeshGeometryModel3D (Link2)                    │
│  │           └── ... (recursive to Joint6)                      │
│  │                                                               │
│  ├── SortingGroupModel3D (GhostRobot) ← Transparency sorting    │
│  │   └── (Same structure as RobotAssembly, Opacity=0.3)         │
│  │                                                               │
│  ├── LineGeometryModel3D (TCPTrail)                             │
│  │                                                               │
│  └── CoordinateSystemModel3D (TCP Frame)                        │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### 5.3. Robot3DViewModel Implementation

```csharp
// ViewModels/Robot3DViewModel.cs
using HelixToolkit.Wpf.SharpDX;
using SharpDX;
using System.Numerics;

namespace RobotUI.ViewModels;

public partial class Robot3DViewModel : ViewModelBase
{
    private readonly IRobotModelService _robotModelService;
    private readonly IIPCClient _ipcClient;

    // Robot link meshes
    private readonly MeshGeometry3D[] _linkMeshes = new MeshGeometry3D[7];

    // Joint transforms (updated from robot state)
    [ObservableProperty]
    private Matrix[] _jointTransforms = new Matrix[6];

    // Ghost robot transforms (for target preview)
    [ObservableProperty]
    private Matrix[] _ghostJointTransforms = new Matrix[6];

    [ObservableProperty]
    private bool _showGhostRobot;

    // TCP Trail
    [ObservableProperty]
    private LineGeometry3D? _tcpTrail;

    private readonly List<Vector3> _trailPoints = new();
    private const int MaxTrailPoints = 10000;

    // Coordinate frames
    [ObservableProperty]
    private Matrix _tcpFrameTransform;

    public Robot3DViewModel(
        IRobotModelService robotModelService,
        IIPCClient ipcClient)
    {
        _robotModelService = robotModelService;
        _ipcClient = ipcClient;

        LoadRobotModel();
        SubscribeToRobotState();
    }

    private void LoadRobotModel()
    {
        // Load STL/OBJ files for each link
        var modelLoader = new StLReader();

        _linkMeshes[0] = modelLoader.Read("Assets/Models3D/base.stl");
        _linkMeshes[1] = modelLoader.Read("Assets/Models3D/link1.stl");
        _linkMeshes[2] = modelLoader.Read("Assets/Models3D/link2.stl");
        _linkMeshes[3] = modelLoader.Read("Assets/Models3D/link3.stl");
        _linkMeshes[4] = modelLoader.Read("Assets/Models3D/link4.stl");
        _linkMeshes[5] = modelLoader.Read("Assets/Models3D/link5.stl");
        _linkMeshes[6] = modelLoader.Read("Assets/Models3D/link6.stl");
    }

    private void SubscribeToRobotState()
    {
        _ipcClient.RobotStateReceived += OnRobotStateReceived;
    }

    private void OnRobotStateReceived(object? sender, RobotState state)
    {
        // Calculate FK transforms from joint angles
        UpdateJointTransforms(state.JointAngles);

        // Update TCP trail
        UpdateTrail(state.TCPPosition);

        // Update TCP frame display
        TcpFrameTransform = CalculateTCPFrame(state.JointAngles);
    }

    private void UpdateJointTransforms(double[] jointAngles)
    {
        // Apply DH parameters to calculate each joint transform
        var dh = _robotModelService.DHParameters;

        for (int i = 0; i < 6; i++)
        {
            JointTransforms[i] = CalculateDHTransform(
                dh[i].A, dh[i].Alpha, dh[i].D, jointAngles[i]);
        }

        OnPropertyChanged(nameof(JointTransforms));
    }

    private Matrix CalculateDHTransform(double a, double alpha, double d, double theta)
    {
        // Standard DH transformation matrix
        float ct = (float)Math.Cos(theta);
        float st = (float)Math.Sin(theta);
        float ca = (float)Math.Cos(alpha);
        float sa = (float)Math.Sin(alpha);
        float fa = (float)a;
        float fd = (float)d;

        return new Matrix(
            ct,      -st * ca,  st * sa,  0,
            st,       ct * ca, -ct * sa,  0,
            0,        sa,       ca,       0,
            fa * ct,  fa * st,  fd,       1
        );
    }

    private void UpdateTrail(Vector3 tcpPosition)
    {
        // Add point to trail with distance threshold
        if (_trailPoints.Count == 0 ||
            Vector3.Distance(_trailPoints[^1], tcpPosition) > 0.5f)
        {
            _trailPoints.Add(tcpPosition);

            // Ring buffer behavior
            if (_trailPoints.Count > MaxTrailPoints)
            {
                _trailPoints.RemoveAt(0);
            }

            // Rebuild trail geometry
            RebuildTrailGeometry();
        }
    }

    private void RebuildTrailGeometry()
    {
        var builder = new LineBuilder();

        for (int i = 1; i < _trailPoints.Count; i++)
        {
            builder.AddLine(_trailPoints[i - 1], _trailPoints[i]);
        }

        TcpTrail = builder.ToLineGeometry3D();
    }

    [RelayCommand]
    private void ClearTrail()
    {
        _trailPoints.Clear();
        TcpTrail = null;
    }

    [RelayCommand]
    private void ToggleGhostRobot()
    {
        ShowGhostRobot = !ShowGhostRobot;
    }
}
```

### 5.4. Robot 3D View XAML

```xml
<!-- Views/Visualization/Robot3DView.xaml -->
<UserControl x:Class="RobotUI.Views.Robot3DView"
             xmlns="http://schemas.microsoft.com/winfx/2006/xaml/presentation"
             xmlns:x="http://schemas.microsoft.com/winfx/2006/xaml"
             xmlns:hx="http://helix-toolkit.org/wpf/SharpDX">

    <hx:Viewport3DX
        Camera="{Binding Camera}"
        BackgroundColor="#1E1E1E"
        ShowCoordinateSystem="True"
        CoordinateSystemLabelForeground="White"
        EnableSwapChainRendering="True"
        FXAALevel="Medium">

        <!-- Lighting -->
        <hx:AmbientLight3D Color="#404040"/>
        <hx:DirectionalLight3D Direction="-1,-1,-1" Color="White"/>

        <!-- World Coordinate Frame -->
        <hx:CoordinateSystemModel3D/>

        <!-- Robot Assembly (Kinematic Chain) -->
        <hx:GroupModel3D x:Name="RobotAssembly">
            <!-- Base (Fixed) -->
            <hx:MeshGeometryModel3D Geometry="{Binding LinkMeshes[0]}"
                                    Material="{StaticResource RobotMaterial}"/>

            <!-- Joint 1 Container -->
            <hx:GroupModel3D Transform="{Binding JointTransforms[0]}">
                <hx:MeshGeometryModel3D Geometry="{Binding LinkMeshes[1]}"
                                        Material="{StaticResource RobotMaterial}"/>

                <!-- Joint 2 Container -->
                <hx:GroupModel3D Transform="{Binding JointTransforms[1]}">
                    <hx:MeshGeometryModel3D Geometry="{Binding LinkMeshes[2]}"
                                            Material="{StaticResource RobotMaterial}"/>

                    <!-- Continue recursive structure... -->
                </hx:GroupModel3D>
            </hx:GroupModel3D>
        </hx:GroupModel3D>

        <!-- Ghost Robot (Transparent, for target preview) -->
        <hx:SortingGroupModel3D Visibility="{Binding ShowGhostRobot,
                                             Converter={StaticResource BoolToVis}}">
            <!-- Same structure but with GhostMaterial (Opacity=0.3) -->
        </hx:SortingGroupModel3D>

        <!-- TCP Trail -->
        <hx:LineGeometryModel3D Geometry="{Binding TcpTrail}"
                                Color="#FF5E00"
                                Thickness="2"/>

        <!-- TCP Coordinate Frame -->
        <hx:CoordinateSystemModel3D Transform="{Binding TcpFrameTransform}"
                                    ArrowLengths="50"/>

    </hx:Viewport3DX>
</UserControl>
```

---

## 6. Jog Control Implementation

### 6.1. Safety Jogging Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    SAFETY JOG ARCHITECTURE                       │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌─────────────┐                                                │
│  │  ENABLE     │◄── User must HOLD this button                  │
│  │  BUTTON     │    (Software Deadman Switch)                   │
│  │  (Left)     │                                                │
│  └──────┬──────┘                                                │
│         │                                                        │
│         ▼                                                        │
│  ┌─────────────┐     ┌─────────────┐     ┌─────────────┐       │
│  │   Enable    │────►│   JOG       │────►│  Heartbeat  │       │
│  │   Active?   │ Yes │  Buttons    │     │   Timer     │       │
│  └─────────────┘     │  Enabled    │     │   (50ms)    │       │
│         │            └──────┬──────┘     └──────┬──────┘       │
│         │ No                │                    │               │
│         ▼                   ▼                    ▼               │
│  ┌─────────────┐     ┌─────────────┐     ┌─────────────┐       │
│  │  JOG        │     │  Send JOG   │     │  Send HB    │       │
│  │  Buttons    │     │  Command    │     │  to Core    │       │
│  │  Disabled   │     │  to Core    │     │  (Keep-Alive)│      │
│  └─────────────┘     └─────────────┘     └─────────────┘       │
│                                                                  │
│  ┌─────────────────────────────────────────────────────────────┐│
│  │  DEFENSE IN DEPTH:                                          ││
│  │  1. Two-Handed Operation (Enable + Jog)                     ││
│  │  2. Heartbeat Watchdog (150ms timeout)                      ││
│  │  3. T1 Mode Speed Limit (max 10% override)                  ││
│  │  4. Soft Limits Check                                       ││
│  └─────────────────────────────────────────────────────────────┘│
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### 6.2. JogViewModel Implementation

```csharp
// ViewModels/JogViewModel.cs
using System.Windows.Input;
using System.Timers;

namespace RobotUI.ViewModels;

public partial class JogViewModel : ViewModelBase
{
    private readonly IIPCClient _ipcClient;
    private readonly Timer _heartbeatTimer;
    private readonly object _jogLock = new();

    [ObservableProperty]
    private bool _enableButtonPressed;

    [ObservableProperty]
    private JogMode _currentJogMode = JogMode.Joint;

    [ObservableProperty]
    private int _overridePercent = 10;  // T1 mode: max 10%

    [ObservableProperty]
    private CoordinateFrame _selectedFrame = CoordinateFrame.World;

    // Active jog state
    private int? _activeJogAxis;
    private JogDirection? _activeJogDirection;

    public JogViewModel(IIPCClient ipcClient)
    {
        _ipcClient = ipcClient;

        // Heartbeat timer (50ms interval)
        _heartbeatTimer = new Timer(50);
        _heartbeatTimer.Elapsed += OnHeartbeatTick;
        _heartbeatTimer.Start();
    }

    private void OnHeartbeatTick(object? sender, ElapsedEventArgs e)
    {
        if (EnableButtonPressed && _activeJogAxis.HasValue)
        {
            // Send continuous jog command with heartbeat
            _ipcClient.SendJogCommand(new JogCommand
            {
                Axis = _activeJogAxis.Value,
                Direction = _activeJogDirection!.Value,
                Mode = CurrentJogMode,
                Frame = SelectedFrame,
                OverridePercent = OverridePercent,
                IsHeartbeat = true
            });
        }
    }

    // Called when jog button is pressed
    public void OnJogButtonDown(int axis, JogDirection direction)
    {
        lock (_jogLock)
        {
            if (!EnableButtonPressed)
            {
                // Enable not held - ignore jog
                return;
            }

            _activeJogAxis = axis;
            _activeJogDirection = direction;

            // Send initial jog start command
            _ipcClient.SendJogCommand(new JogCommand
            {
                Axis = axis,
                Direction = direction,
                Mode = CurrentJogMode,
                Frame = SelectedFrame,
                OverridePercent = OverridePercent,
                IsStart = true
            });
        }
    }

    // Called when jog button is released
    public void OnJogButtonUp(int axis)
    {
        lock (_jogLock)
        {
            if (_activeJogAxis == axis)
            {
                // Send jog stop command
                _ipcClient.SendJogStop(axis);

                _activeJogAxis = null;
                _activeJogDirection = null;
            }
        }
    }

    // Enable button handlers
    public void OnEnableButtonDown()
    {
        EnableButtonPressed = true;
    }

    public void OnEnableButtonUp()
    {
        EnableButtonPressed = false;

        // Immediately stop all jogging
        if (_activeJogAxis.HasValue)
        {
            _ipcClient.SendJogStop(_activeJogAxis.Value);
            _activeJogAxis = null;
            _activeJogDirection = null;
        }
    }

    // Override slider (limited in T1 mode)
    partial void OnOverridePercentChanged(int value)
    {
        // T1 mode safety limit
        if (value > 10)
        {
            OverridePercent = 10;
        }
    }
}

public enum JogMode
{
    Joint,      // J1-J6 individual
    Cartesian,  // X, Y, Z, Rx, Ry, Rz
    Tool        // Along tool axis
}

public enum JogDirection
{
    Positive,
    Negative
}

public enum CoordinateFrame
{
    World,
    Base,
    Tool,
    User
}
```

### 6.3. Jog View XAML

```xml
<!-- Views/Main/JogView.xaml -->
<UserControl x:Class="RobotUI.Views.JogView"
             xmlns="http://schemas.microsoft.com/winfx/2006/xaml/presentation"
             xmlns:x="http://schemas.microsoft.com/winfx/2006/xaml"
             xmlns:local="clr-namespace:RobotUI.Controls">

    <Grid>
        <Grid.ColumnDefinitions>
            <ColumnDefinition Width="*"/>    <!-- Jog Controls -->
            <ColumnDefinition Width="300"/>  <!-- Coordinate Display -->
        </Grid.ColumnDefinitions>

        <!-- Jog Controls Panel -->
        <Grid Grid.Column="0">
            <Grid.RowDefinitions>
                <RowDefinition Height="Auto"/>  <!-- Mode Selection -->
                <RowDefinition Height="*"/>      <!-- Jog Buttons -->
                <RowDefinition Height="Auto"/>  <!-- Override Slider -->
            </Grid.RowDefinitions>

            <!-- Mode Selection -->
            <StackPanel Grid.Row="0" Orientation="Horizontal" Margin="20">
                <RadioButton Content="Joint"
                             IsChecked="{Binding CurrentJogMode,
                                         Converter={StaticResource EnumToBool},
                                         ConverterParameter=Joint}"
                             Style="{StaticResource JogModeRadioStyle}"/>
                <RadioButton Content="Cartesian"
                             IsChecked="{Binding CurrentJogMode,
                                         Converter={StaticResource EnumToBool},
                                         ConverterParameter=Cartesian}"
                             Style="{StaticResource JogModeRadioStyle}"/>
                <RadioButton Content="Tool"
                             IsChecked="{Binding CurrentJogMode,
                                         Converter={StaticResource EnumToBool},
                                         ConverterParameter=Tool}"
                             Style="{StaticResource JogModeRadioStyle}"/>
            </StackPanel>

            <!-- Jog Buttons Grid -->
            <Grid Grid.Row="1" Margin="20">
                <!-- Joint Mode Buttons -->
                <ItemsControl Visibility="{Binding CurrentJogMode,
                                           Converter={StaticResource JointModeVis}}">
                    <ItemsControl.ItemsPanel>
                        <ItemsPanelTemplate>
                            <UniformGrid Rows="6" Columns="3"/>
                        </ItemsPanelTemplate>
                    </ItemsControl.ItemsPanel>

                    <!-- J1 -->
                    <TextBlock Text="J1" Style="{StaticResource JogLabelStyle}"/>
                    <local:JogButton Content="-" Axis="0" Direction="Negative"
                                     IsEnabled="{Binding EnableButtonPressed}"/>
                    <local:JogButton Content="+" Axis="0" Direction="Positive"
                                     IsEnabled="{Binding EnableButtonPressed}"/>

                    <!-- J2 -->
                    <TextBlock Text="J2" Style="{StaticResource JogLabelStyle}"/>
                    <local:JogButton Content="-" Axis="1" Direction="Negative"
                                     IsEnabled="{Binding EnableButtonPressed}"/>
                    <local:JogButton Content="+" Axis="1" Direction="Positive"
                                     IsEnabled="{Binding EnableButtonPressed}"/>

                    <!-- ... J3 to J6 ... -->
                </ItemsControl>
            </Grid>

            <!-- Override Slider -->
            <StackPanel Grid.Row="2" Orientation="Horizontal" Margin="20">
                <TextBlock Text="Override:"
                           Style="{StaticResource BodyTextStyle}"
                           VerticalAlignment="Center"/>
                <Slider Value="{Binding OverridePercent}"
                        Minimum="1" Maximum="100"
                        Width="200" Margin="10,0"/>
                <TextBlock Text="{Binding OverridePercent, StringFormat={}{0}%}"
                           Style="{StaticResource CoordinateTextStyle}"
                           Width="50"/>
            </StackPanel>
        </Grid>

        <!-- Coordinate Display -->
        <Border Grid.Column="1" Background="{StaticResource BackgroundDarkBrush}">
            <local:CoordinateDisplayView DataContext="{Binding CoordinateViewModel}"/>
        </Border>

        <!-- ENABLE BUTTON (Bottom-Left, Two-Handed Operation) -->
        <Button x:Name="EnableButton"
                Content="ENABLE"
                Width="120" Height="80"
                HorizontalAlignment="Left"
                VerticalAlignment="Bottom"
                Margin="20"
                Style="{StaticResource EnableButtonStyle}"
                PreviewMouseDown="EnableButton_MouseDown"
                PreviewMouseUp="EnableButton_MouseUp"
                PreviewTouchDown="EnableButton_TouchDown"
                PreviewTouchUp="EnableButton_TouchUp"/>
    </Grid>
</UserControl>
```

---

## 7. Real-time Data Throttling

### 7.1. Problem Statement
- Robot sends position data at 100Hz (10ms/packet)
- UI can only render at 60Hz max
- Direct binding causes UI thread overload and GC pressure

### 7.2. Solution: Rx.NET Throttling

```csharp
// Services/RobotStateThrottler.cs
using System.Reactive.Linq;
using System.Reactive.Subjects;

namespace RobotUI.Services;

public class RobotStateThrottler : IDisposable
{
    private readonly Subject<RobotState> _stateSubject = new();
    private readonly IDisposable _subscription;

    public IObservable<RobotState> ThrottledState { get; }

    public RobotStateThrottler()
    {
        // Throttle to 30Hz for UI updates
        ThrottledState = _stateSubject
            .Sample(TimeSpan.FromMilliseconds(33))  // ~30 FPS
            .ObserveOn(SynchronizationContext.Current!);
    }

    public void OnRawStateReceived(RobotState state)
    {
        _stateSubject.OnNext(state);
    }

    public void Dispose()
    {
        _subscription?.Dispose();
        _stateSubject.Dispose();
    }
}
```

### 7.3. Usage in ViewModel

```csharp
// In StatusViewModel
public StatusViewModel(RobotStateThrottler throttler)
{
    throttler.ThrottledState.Subscribe(state =>
    {
        // UI-safe update at 30Hz
        MotorOn = state.MotorEnabled;
        CurrentMode = state.OperationMode;
        OverrideValue = state.OverridePercent;
        ConnectionStatus = state.IsConnected;
    });
}
```

---

## 8. Tasks Breakdown

### 8.1. Implementation Tasks

| ID | Task | Description | Priority |
|----|------|-------------|----------|
| T01 | Create MainWindow shell | Layout zones, navigation frame | P0 |
| T02 | Implement NavigationService | View switching, back stack | P0 |
| T03 | Create industrial theme | Colors, brushes, button styles | P0 |
| T04 | StatusBar view | Mode, motor, override display | P0 |
| T05 | SmartKeys bar | Context-sensitive buttons | P1 |
| T06 | JogView implementation | Jog buttons, enable safety | P0 |
| T07 | JogButton control | Hold-to-run behavior | P0 |
| T08 | CoordinateDisplay | Joint/Cartesian readout | P1 |
| T09 | Robot3DView (Helix) | Load model, FK animation | P0 |
| T10 | Ghost robot rendering | Transparent preview | P1 |
| T11 | TCP trail rendering | Weld path visualization | P1 |
| T12 | State throttling | Rx.NET 30Hz sampling | P0 |
| T13 | Message window | Errors, warnings display | P1 |
| T14 | Program editor view | Inline forms | P2 |

### 8.2. Testing Checklist

```
[ ] UI renders correctly at 1920x1080
[ ] Touch targets ≥ 48x48px
[ ] Button contrast ratio ≥ 4.5:1 (WCAG AA)
[ ] Jog buttons respond to hold-to-run
[ ] Enable button required for jogging
[ ] State updates at 30Hz without lag
[ ] 3D viewport maintains 60 FPS
[ ] Ghost robot renders correctly (no z-fighting)
[ ] Trail rendering handles 10000+ points
[ ] Memory stable over 1 hour operation
```

---

## 9. References

### 9.1. Research Documents
- Thiết kế HMI Robot KUKA WPF.md
- Thiết Kế Mô Phỏng Robot WPF Helix.md

### 9.2. External Resources
| Resource | URL |
|----------|-----|
| MVVM Pattern | https://learn.microsoft.com/en-us/archive/msdn-magazine/2009/february/patterns-wpf-apps-with-the-model-view-viewmodel-design-pattern |
| Helix Toolkit | https://helix-toolkit.github.io/ |
| Helix SharpDX | https://github.com/helix-toolkit/helix-toolkit |
| CommunityToolkit.Mvvm | https://learn.microsoft.com/en-us/dotnet/communitytoolkit/mvvm/ |
| Rx.NET | https://github.com/dotnet/reactive |

### 9.3. Related Documents
| Document | Description |
|----------|-------------|
| [CORE_01_Project_Setup.md](./CORE_01_Project_Setup.md) | Project structure |
| [CORE_IPC.md](./CORE_IPC.md) | IPC communication |
| [CORE_Config.md](./CORE_Config.md) | Configuration loading |
| [CORE_StateManager.md](./CORE_StateManager.md) | Robot state machine |

---

*Document version: 1.0 | Last updated: 2026-02-01*
