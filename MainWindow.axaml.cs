using Avalonia;
using Avalonia.Controls;
using Avalonia.Input;
using Avalonia.Media;
using MinimalRouter.Routing;
using System.Collections.Generic;

namespace MinimalRouter;

public partial class MainWindow : Window
{
    private readonly Router _router = new Router();
    private readonly List<Segment> _tracks = new();
    private Point? _last = null;

    public MainWindow()
    {
        InitializeComponent();

        // 加几个默认障碍物
        _router.AddObstacle(new Obstacle(new Rect(200, 200, 100, 80)));
        _router.AddObstacle(new Obstacle(new Rect(450, 120, 120, 140)));

        CanvasArea.PointerPressed += OnClick;
        CanvasArea.PointerMoved += (_, _) => CanvasArea.InvalidateVisual();
        CanvasArea.Draw += Draw;
    }

    private void OnClick(object? sender, PointerPressedEventArgs e)
    {
        var p = e.GetPosition(CanvasArea);

        if (_last == null)
        {
            _last = p;
        }
        else
        {
            var points = _router.Route(_last.Value, p);

            for (int i = 0; i < points.Count - 1; i++)
                _tracks.Add(new Segment(points[i], points[i + 1]));

            _last = null;
            CanvasArea.InvalidateVisual();
        }
    }

    private void Draw(object? sender, DrawingContext dc)
    {
        var penTrack = new Pen(Brushes.DarkBlue, 2);
        var penObstacle = new Pen(Brushes.Red, 1);

        // 画障碍物
        foreach (var o in _router.Obstacles)
            dc.DrawRectangle(null, penObstacle, o.Bounds);

        // 画线段
        foreach (var seg in _tracks)
            dc.DrawLine(penTrack, seg.A, seg.B);
    }
}
