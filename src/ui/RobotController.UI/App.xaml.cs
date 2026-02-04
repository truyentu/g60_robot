using Microsoft.Extensions.DependencyInjection;
using Microsoft.Extensions.Hosting;
using RobotController.Common.Services;
using RobotController.UI.Services;
using RobotController.UI.ViewModels;
using RobotController.UI.ViewModels.Pages;
using RobotController.UI.Views;
using Serilog;
using System.Windows;

namespace RobotController.UI;

public partial class App : Application
{
    private IHost? _host;

    protected override async void OnStartup(StartupEventArgs e)
    {
        base.OnStartup(e);

        // Setup Serilog with correct path
        var baseDir = AppDomain.CurrentDomain.BaseDirectory;
        var logPath = System.IO.Path.Combine(baseDir, "logs", "ui.log");
        System.IO.Directory.CreateDirectory(System.IO.Path.GetDirectoryName(logPath)!);

        Log.Logger = new LoggerConfiguration()
            .MinimumLevel.Debug()
            .WriteTo.Console()
            .WriteTo.File(logPath,
                rollingInterval: RollingInterval.Day,
                retainedFileCountLimit: 7,
                outputTemplate: "{Timestamp:yyyy-MM-dd HH:mm:ss.fff} [{Level:u3}] {Message:lj}{NewLine}{Exception}")
            .CreateLogger();

        Log.Information("========================================");
        Log.Information("Robot Controller UI starting...");
        Log.Information("========================================");

        // Build host with DI
        _host = Host.CreateDefaultBuilder()
            .UseSerilog()
            .ConfigureServices((context, services) =>
            {
                // Configuration (load first)
                services.AddSingleton<IConfigService, ConfigService>();

                // IPC Client
                services.AddSingleton<IIpcClientService, IpcClientService>();

                // Viewport Service
                services.AddSingleton<IViewportService, ViewportService>();

                // ViewModels
                services.AddSingleton<MainViewModel>();
                services.AddSingleton<RobotPackageBrowserViewModel>();

                // Views
                services.AddSingleton<MainWindow>();
            })
            .Build();

        await _host.StartAsync();

        // Get services
        var configService = _host.Services.GetRequiredService<IConfigService>();
        var mainWindow = _host.Services.GetRequiredService<MainWindow>();

        // Apply window settings from config
        var windowConfig = configService.Config.Window;
        if (windowConfig.Width > 0) mainWindow.Width = windowConfig.Width;
        if (windowConfig.Height > 0) mainWindow.Height = windowConfig.Height;
        if (windowConfig.Left >= 0) mainWindow.Left = windowConfig.Left;
        if (windowConfig.Top >= 0) mainWindow.Top = windowConfig.Top;
        if (windowConfig.Maximized) mainWindow.WindowState = WindowState.Maximized;

        mainWindow.Closing += MainWindow_Closing;
        mainWindow.Show();

        Log.Information("UI initialized with theme: {Theme}", configService.Config.Ui.Theme);
    }

    private void MainWindow_Closing(object? sender, System.ComponentModel.CancelEventArgs e)
    {
        if (sender is MainWindow window && _host != null)
        {
            // Save window state
            var configService = _host.Services.GetService<IConfigService>();
            if (configService != null)
            {
                configService.Config.Window.Width = (int)window.Width;
                configService.Config.Window.Height = (int)window.Height;
                configService.Config.Window.Left = (int)window.Left;
                configService.Config.Window.Top = (int)window.Top;
                configService.Config.Window.Maximized = window.WindowState == WindowState.Maximized;
                configService.Save();
            }
        }
    }

    protected override async void OnExit(ExitEventArgs e)
    {
        Log.Information("Robot Controller UI shutting down");

        if (_host != null)
        {
            // Dispose ViewModel (which disposes IPC client)
            var viewModel = _host.Services.GetService<MainViewModel>();
            viewModel?.Dispose();

            await _host.StopAsync();
            _host.Dispose();
        }

        Log.CloseAndFlush();
        base.OnExit(e);
    }
}
