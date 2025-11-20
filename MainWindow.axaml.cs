using Avalonia;
using Avalonia.Controls;
using Avalonia.Input;
using Avalonia.Media;
using MinimalRouter.Routing;
using System.Collections.Generic;
using Avalonia.Controls.Shapes;
using System.Linq;
using System;

namespace MinimalRouter;

public partial class MainWindow : Window
{
    private readonly Router _router = new Router();
    private Point? _last = null;
    private Ellipse? _startPoint = null;
    private Ellipse? _endPoint = null;
    private List<Line> _dynamicLines = new();
    private List<Line> _staticLines = new();
    private List<Point> _currentRoutePoints = new();

    public MainWindow()
    {
        InitializeComponent();

        // 随机生成障碍物，基于网格
        var rand = new Random();
        double gridSize = 10;
        int numObstacles = rand.Next(8, 13); // 8 到 12 个
        for (int i = 0; i < numObstacles; i++)
        {
            bool valid;
            Rect newRect;
            int attempts = 0;
            do
            {
                int widthUnits = rand.Next(5, 16); // 5-15 units -> 50-150
                int heightUnits = rand.Next(5, 16);
                double width = widthUnits * gridSize;
                double height = heightUnits * gridSize;
                int maxXUnits = (int)((800 - width) / gridSize);
                int maxYUnits = (int)((600 - height) / gridSize);
                int xUnits = rand.Next(0, maxXUnits + 1);
                int yUnits = rand.Next(0, maxYUnits + 1);
                double x = xUnits * gridSize;
                double y = yUnits * gridSize;
                newRect = new Rect(x, y, width, height);
                valid = true;
                foreach (var existing in _router.Obstacles)
                {
                    if (newRect.Intersects(existing.Bounds))
                    {
                        valid = false;
                        break;
                    }
                }
                attempts++;
            } while (!valid && attempts < 100);

            if (valid)
            {
                var obstacle = new Obstacle(newRect);
                _router.AddObstacle(obstacle);

                // 添加 Rectangle 到 Canvas
                var rect = new Rectangle
                {
                    Width = newRect.Width,
                    Height = newRect.Height,
                    Stroke = Brushes.Red,
                    StrokeThickness = 1,
                    Fill = null
                };
                Canvas.SetLeft(rect, newRect.X);
                Canvas.SetTop(rect, newRect.Y);
                CanvasArea.Children.Add(rect);
            }
        }

        // subscribe once to pointer moved to show dynamic routing while user is in routing mode
        CanvasArea.PointerPressed += OnClick;
        CanvasArea.PointerMoved += OnPointerMoved;
    }

    private void OnClick(object? sender, PointerPressedEventArgs e)
    {
        var p = Router.SnapToGrid(e.GetPosition(CanvasArea));
        var props = e.GetCurrentPoint(this).Properties;

        if (props.IsLeftButtonPressed)
        {
            // detect double-click to finish routing
            if (e.ClickCount == 2)
            {
                if (_last != null)
                {
                    // commit current dynamic route as static
                    CommitCurrentRoute();

                    // place end marker
                    _endPoint = new Ellipse
                    {
                        Width = 10,
                        Height = 10,
                        Fill = Brushes.Red
                    };
                    var endPt = _currentRoutePoints.Count > 0 ? _currentRoutePoints.Last() : p;
                    Canvas.SetLeft(_endPoint, endPt.X - 5);
                    Canvas.SetTop(_endPoint, endPt.Y - 5);
                    CanvasArea.Children.Add(_endPoint);

                    // stop interactive routing
                    _last = null;
                }
            }
            else
            {
                // single click
                if (_last == null)
                {
                    // start routing
                    _last = p;
                    _startPoint = new Ellipse
                    {
                        Width = 10,
                        Height = 10,
                        Fill = Brushes.Green
                    };
                    Canvas.SetLeft(_startPoint, p.X - 5);
                    Canvas.SetTop(_startPoint, p.Y - 5);
                    CanvasArea.Children.Add(_startPoint);

                    // ensure any previous state cleared
                    ClearDynamicLines();
                    ClearStaticLines();
                    if (_endPoint != null)
                    {
                        CanvasArea.Children.Remove(_endPoint);
                        _endPoint = null;
                    }
                }
                else
                {
                    // confirm current dynamic segment and continue from its end
                    CommitCurrentRoute();

                    // set new _last to the last point of committed route (if any)
                    if (_currentRoutePoints.Count > 0)
                        _last = _currentRoutePoints.Last();
                }
            }
        }
        else if (props.IsRightButtonPressed)
        {
            if (_last != null)
            {
                // cancel routing: remove start/end markers, dynamic and static lines
                if (_startPoint != null)
                {
                    CanvasArea.Children.Remove(_startPoint);
                    _startPoint = null;
                }
                if (_endPoint != null)
                {
                    CanvasArea.Children.Remove(_endPoint);
                    _endPoint = null;
                }

                ClearDynamicLines();
                ClearStaticLines();
                _last = null;
            }
        }
    }

    private void CommitCurrentRoute()
    {
        if (_currentRoutePoints == null || _currentRoutePoints.Count < 2)
            return;

        // create permanent lines from current route points
        for (int i = 0; i < _currentRoutePoints.Count - 1; i++)
        {
            var line = new Line
            {
                StartPoint = _currentRoutePoints[i],
                EndPoint = _currentRoutePoints[i + 1],
                Stroke = Brushes.Blue,
                StrokeThickness = 2
            };
            _staticLines.Add(line);
            CanvasArea.Children.Add(line);
        }

        // clear dynamic visuals (they will be recreated on next move)
        ClearDynamicLines();
    }

    private void ClearDynamicLines()
    {
        foreach (var line in _dynamicLines)
            CanvasArea.Children.Remove(line);
        _dynamicLines.Clear();
    }

    private void ClearStaticLines()
    {
        foreach (var line in _staticLines)
            CanvasArea.Children.Remove(line);
        _staticLines.Clear();
    }

    private void OnPointerMoved(object? sender, PointerEventArgs e)
    {
        if (_last != null)
        {
            var p = Router.SnapToGrid(e.GetPosition(CanvasArea));
            var points = _router.Route(_last.Value, p) ?? new List<Point>();

            // store current route points for commit on click
            _currentRoutePoints = points;

            // 清除动态线段
            foreach (var line in _dynamicLines)
                CanvasArea.Children.Remove(line);
            _dynamicLines.Clear();

            // 添加新的线段
            for (int i = 0; i < points.Count - 1; i++)
            {
                var line = new Line
                {
                    StartPoint = points[i],
                    EndPoint = points[i + 1],
                    Stroke = Brushes.DarkBlue,
                    StrokeThickness = 2
                };
                _dynamicLines.Add(line);
                CanvasArea.Children.Add(line);
            }
        }
    }
}
