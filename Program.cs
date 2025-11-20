using System;
using Avalonia;

using Avalonia;

namespace MinimalRouter;

class Program
{
    public static void Main(string[] args)
        => BuildAvaloniaApp().StartWithClassicDesktopLifetime(args);

    private static AppBuilder BuildAvaloniaApp()
        => AppBuilder.Configure<App>()
            .UsePlatformDetect()
            .LogToTrace();
}

