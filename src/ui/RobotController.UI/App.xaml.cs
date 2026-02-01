using Microsoft.Extensions.DependencyInjection;
using Microsoft.Extensions.Hosting;
using RobotController.Common.Services;
using Serilog;
using System.Windows;

namespace RobotController.UI;

public partial class App : Application
{
    private IHost? _host;

    protected override async void OnStartup(StartupEventArgs e)
    {
        base.OnStartup(e);

        // Setup Serilog
        Log.Logger = new LoggerConfiguration()
            .MinimumLevel.Debug()
            .WriteTo.Console()
            .WriteTo.File("../../logs/ui.log",
                rollingInterval: RollingInterval.Day,
                retainedFileCountLimit: 7)
            .CreateLogger();

        Log.Information("Robot Controller UI starting...");

        // Build host with DI
        _host = Host.CreateDefaultBuilder()
            .UseSerilog()
            .ConfigureServices((context, services) =>
            {
                // Services
                services.AddSingleton<IIpcClientService, IpcClientService>();

                // ViewModels
                services.AddSingleton<ViewModels.MainViewModel>();

                // Views
                services.AddSingleton<Views.MainWindow>();
            })
            .Build();

        await _host.StartAsync();

        // Show main window
        var mainWindow = _host.Services.GetRequiredService<Views.MainWindow>();
        mainWindow.Show();
    }

    protected override async void OnExit(ExitEventArgs e)
    {
        Log.Information("Robot Controller UI shutting down");

        // Dispose IPC client
        if (_host != null)
        {
            var ipcClient = _host.Services.GetService<IIpcClientService>();
            ipcClient?.Dispose();

            await _host.StopAsync();
            _host.Dispose();
        }

        Log.CloseAndFlush();
        base.OnExit(e);
    }
}
