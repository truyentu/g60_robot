using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Windows;
using System.Windows.Controls;
using RobotController.UI.ViewModels;

namespace RobotController.UI.Views.Layout;

public partial class TopStatusBarView : UserControl
{
    private readonly ObservableCollection<string> _recentItems = new();
    private const int MaxRecentItems = 6;

    // Current active panels for submenu tracking
    private string _activeMainMenu = "";
    private string _activeSubL1 = "";

    // Leaf menu tag → Navigator SelectedNavIndex
    private static readonly Dictionary<string, int> MenuToNavIndex = new()
    {
        // Display submenu
        { "Display.Inputs/outputs.Digital I/O",      2 },
        { "Display.Inputs/outputs.Analog I/O",       2 },
        { "Display.Inputs/outputs.Automatic External", 2 },
        { "Display.Actual position",                 0 },

        // Configuration submenu
        { "Configuration.User group",                3 },
        { "Configuration.Inputs/outputs.I/O drivers",        3 },
        { "Configuration.Inputs/outputs.Automatic External", 3 },

        // Diagnosis submenu
        { "Diagnosis.Logbook.Display",               4 },
        { "Diagnosis.Diagnostic monitor",            4 },

        // Start-up submenu
        { "Start-up.Robot data.Catalog",             5 },
        { "Start-up.Robot data.URDF Import",         6 },
        { "Start-up.Calibrate.Tool",                 7 },
        { "Start-up.Network configuration",          3 },
        { "Start-up.Station Setup",                  8 },

        // Help submenu
        { "Help.Info",                               4 },
    };

    // Main menu tag → Submenu L1 panel name
    private static readonly Dictionary<string, string> MainToSubL1 = new()
    {
        { "Display",       "Sub_Display" },
        { "Configuration", "Sub_Configuration" },
        { "Diagnosis",     "Sub_Diagnosis" },
        { "Start-up",      "Sub_Startup" },
        { "Help",          "Sub_Help" },
    };

    // SubL1 tag → SubL2 panel name
    private static readonly Dictionary<string, string> SubL1ToSubL2 = new()
    {
        { "Display.Inputs/outputs",       "Sub_Display_IO" },
        { "Configuration.Inputs/outputs", "Sub_Configuration_IO" },
        { "Diagnosis.Logbook",            "Sub_Diagnosis_Logbook" },
        { "Start-up.Calibrate",           "Sub_Startup_Calibrate" },
        { "Start-up.Robot data",           "Sub_Startup_RobotData" },
    };

    public TopStatusBarView()
    {
        InitializeComponent();
        Loaded += OnLoaded;
    }

    private void OnLoaded(object sender, RoutedEventArgs e)
    {
        RecentItemsList.ItemsSource = _recentItems;
    }

    private MainViewModel? MainVm => DataContext as MainViewModel;

    // ===== Header buttons =====
    private void MainMenuPopup_Closed(object? sender, System.EventArgs e)
    {
        // Reset all submenu state when popup closes (by any means)
        HideAllSubL1();
        HideAllSubL2();
        SubL1Panel.Visibility = Visibility.Collapsed;
        SubL1Header.Visibility = Visibility.Collapsed;
        SubL2Panel.Visibility = Visibility.Collapsed;
        SubL2Header.Visibility = Visibility.Collapsed;
        _activeMainMenu = "";
        _activeSubL1 = "";
    }

    private void MainMenuClose_Click(object sender, RoutedEventArgs e)
    {
        MainMenuToggle.IsChecked = false;
    }

    private void MainMenuHome_Click(object sender, RoutedEventArgs e)
    {
        // Close all submenus, return to main menu only
        HideAllSubL2();
        HideAllSubL1();
        SubL1Panel.Visibility = Visibility.Collapsed;
        SubL1Header.Visibility = Visibility.Collapsed;
        SubL2Panel.Visibility = Visibility.Collapsed;
        SubL2Header.Visibility = Visibility.Collapsed;
        _activeMainMenu = "";
        _activeSubL1 = "";
    }

    private void MainMenuBack_Click(object sender, RoutedEventArgs e)
    {
        // Close deepest open level
        if (SubL2Panel.Visibility == Visibility.Visible)
        {
            HideAllSubL2();
            SubL2Panel.Visibility = Visibility.Collapsed;
            SubL2Header.Visibility = Visibility.Collapsed;
            _activeSubL1 = "";
        }
        else if (SubL1Panel.Visibility == Visibility.Visible)
        {
            HideAllSubL1();
            SubL1Panel.Visibility = Visibility.Collapsed;
            SubL1Header.Visibility = Visibility.Collapsed;
            HideAllSubL2();
            SubL2Panel.Visibility = Visibility.Collapsed;
            SubL2Header.Visibility = Visibility.Collapsed;
            _activeMainMenu = "";
            _activeSubL1 = "";
        }
    }

    // ===== Main Menu Column (Column 0) =====
    private void MainMenuParent_MouseEnter(object sender, System.Windows.Input.MouseEventArgs e)
    {
        if (sender is not Button btn || !btn.IsEnabled) return;
        var tag = btn.Tag?.ToString();
        if (string.IsNullOrEmpty(tag) || tag == _activeMainMenu) return;

        _activeMainMenu = tag;

        // Hide L2
        HideAllSubL2();
        SubL2Panel.Visibility = Visibility.Collapsed;
        SubL2Header.Visibility = Visibility.Collapsed;
        _activeSubL1 = "";

        // Show L1 for this parent
        HideAllSubL1();
        if (MainToSubL1.TryGetValue(tag, out var panelName))
        {
            var panel = FindName(panelName) as StackPanel;
            if (panel != null)
            {
                panel.Visibility = Visibility.Visible;
                SubL1Panel.Visibility = Visibility.Visible;
                SubL1Header.Visibility = Visibility.Visible;
                SubL1HeaderText.Text = tag;
            }
        }
    }

    private void MainMenuLeaf_MouseEnter(object sender, System.Windows.Input.MouseEventArgs e)
    {
        // Hovering over a leaf in main column → close submenus
        HideAllSubL1();
        HideAllSubL2();
        SubL1Panel.Visibility = Visibility.Collapsed;
        SubL1Header.Visibility = Visibility.Collapsed;
        SubL2Panel.Visibility = Visibility.Collapsed;
        SubL2Header.Visibility = Visibility.Collapsed;
        _activeMainMenu = "";
        _activeSubL1 = "";
    }

    // ===== SubL1 Column (Column 1) =====
    private void SubL1Parent_MouseEnter(object sender, System.Windows.Input.MouseEventArgs e)
    {
        if (sender is not Button btn || !btn.IsEnabled) return;
        var tag = btn.Tag?.ToString();
        if (string.IsNullOrEmpty(tag) || tag == _activeSubL1) return;

        _activeSubL1 = tag;

        // Show L2 for this parent
        HideAllSubL2();
        if (SubL1ToSubL2.TryGetValue(tag, out var panelName))
        {
            var panel = FindName(panelName) as StackPanel;
            if (panel != null)
            {
                panel.Visibility = Visibility.Visible;
                SubL2Panel.Visibility = Visibility.Visible;
                SubL2Header.Visibility = Visibility.Visible;
                // Header = last part of tag (e.g. "Display.Inputs/outputs" → "Inputs/outputs")
                var lastDot = tag.LastIndexOf('.');
                SubL2HeaderText.Text = lastDot >= 0 ? tag[(lastDot + 1)..] : tag;
            }
        }
    }

    private void SubL1Leaf_MouseEnter(object sender, System.Windows.Input.MouseEventArgs e)
    {
        // Hovering over leaf in L1 → close L2
        HideAllSubL2();
        SubL2Panel.Visibility = Visibility.Collapsed;
        SubL2Header.Visibility = Visibility.Collapsed;
        _activeSubL1 = "";
    }

    // ===== Leaf click → Navigate + close =====
    private void LeafMenuItem_Click(object sender, RoutedEventArgs e)
    {
        if (sender is not Button btn || !btn.IsEnabled) return;
        var tag = btn.Tag?.ToString();
        if (string.IsNullOrEmpty(tag)) return;

        // Navigate to the mapped page
        if (MainVm != null && MenuToNavIndex.TryGetValue(tag, out int navIndex))
        {
            MainVm.SelectedNavIndex = navIndex;
        }

        // Track in recent — use human-readable label (last part of dotted tag)
        var lastDot = tag.LastIndexOf('.');
        var label = lastDot >= 0 ? tag[(lastDot + 1)..] : tag;
        AddRecentItem(label, tag);

        // Close popup
        MainMenuToggle.IsChecked = false;
    }

    // ===== Recent items =====
    private void RecentItem_Click(object sender, RoutedEventArgs e)
    {
        if (sender is not Button btn) return;
        var label = btn.Content?.ToString();
        if (string.IsNullOrEmpty(label)) return;

        // Resolve label → full dotted tag → nav index
        if (_recentTagMap.TryGetValue(label, out var fullTag) &&
            MainVm != null && MenuToNavIndex.TryGetValue(fullTag, out int navIndex))
        {
            MainVm.SelectedNavIndex = navIndex;
        }

        MainMenuToggle.IsChecked = false;
    }

    private void AddRecentItem(string label, string fullTag)
    {
        // Remove existing entry with same tag
        for (int i = _recentItems.Count - 1; i >= 0; i--)
        {
            if (_recentItems[i] == label)
            {
                _recentItems.RemoveAt(i);
                break;
            }
        }

        _recentItems.Insert(0, label);
        while (_recentItems.Count > MaxRecentItems)
        {
            _recentItems.RemoveAt(_recentItems.Count - 1);
        }

        // Store tag mapping for recent items click
        _recentTagMap[label] = fullTag;
    }

    private readonly Dictionary<string, string> _recentTagMap = new();

    // ===== Helpers =====
    private void HideAllSubL1()
    {
        Sub_Display.Visibility = Visibility.Collapsed;
        Sub_Configuration.Visibility = Visibility.Collapsed;
        Sub_Diagnosis.Visibility = Visibility.Collapsed;
        Sub_Startup.Visibility = Visibility.Collapsed;
        Sub_Help.Visibility = Visibility.Collapsed;
    }

    private void HideAllSubL2()
    {
        Sub_Display_IO.Visibility = Visibility.Collapsed;
        Sub_Configuration_IO.Visibility = Visibility.Collapsed;
        Sub_Diagnosis_Logbook.Visibility = Visibility.Collapsed;
        Sub_Startup_Calibrate.Visibility = Visibility.Collapsed;
        Sub_Startup_RobotData.Visibility = Visibility.Collapsed;
    }
}
