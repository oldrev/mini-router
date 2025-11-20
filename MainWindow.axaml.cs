using Avalonia;
using Avalonia.Controls;
using Avalonia.Input;
using Avalonia.Media;
using MinimalRouter.Routing;
using System.Collections.Generic;
using Avalonia.Controls.Shapes;
using System.Linq;
using System;
using Clipper2Lib;

namespace MinimalRouter;

public partial class MainWindow : Window
{
    private readonly Router _router = new Router();
    private Point? _last = null;
    private Ellipse? _startPoint = null;        // overall route start marker (kept at first click)
    private Ellipse? _segmentStartPoint = null; // current segment start marker (moves to each new segment start)
    private Ellipse? _endPoint = null;
    private List<Line> _dynamicLines = new();
    private List<Control> _staticVisuals = new();
    private List<Point> _currentRoutePoints = new();

    private Rect _boardBoundary;
    private List<Control> _pourVisuals = new();
    private const double Scale = 1000.0;

    public MainWindow()
    {
        InitializeComponent();

        // Define Board Boundary
        _boardBoundary = new Rect(50, 50, 700, 500);
        
        // Draw Boundary
        var boundaryRect = new Rectangle
        {
            Width = _boardBoundary.Width,
            Height = _boardBoundary.Height,
            Stroke = Brushes.Gray,
            StrokeThickness = 2,
            StrokeDashArray = new Avalonia.Collections.AvaloniaList<double> { 4, 2 },
            Fill = null
        };
        Canvas.SetLeft(boundaryRect, _boardBoundary.X);
        Canvas.SetTop(boundaryRect, _boardBoundary.Y);
        CanvasArea.Children.Add(boundaryRect);

        // Setup Sliders
        WidthSlider.PropertyChanged += (s, e) => 
        {
            if (e.Property == Slider.ValueProperty)
            {
                WidthDisplay.Text = WidthSlider.Value.ToString("0");
            }
        };
        
        ClearanceSlider.PropertyChanged += (s, e) =>
        {
            if (e.Property == Slider.ValueProperty)
            {
                ClearanceDisplay.Text = ClearanceSlider.Value.ToString("0");
                _router.Clearance = ClearanceSlider.Value;
            }
        };

        // Generate Obstacles
        var rand = new Random();
        double gridSize = 10;
        int numObstacles = rand.Next(8, 13); // 8 to 12
        for (int i = 0; i < numObstacles; i++)
        {
            bool valid;
            List<Point> vertices;
            Rect bounds;
            int attempts = 0;
            do
            {
                // Generate random polygon
                vertices = GenerateRandomPolygon(rand, gridSize, _boardBoundary);
                
                // Compute bounds for intersection check
                double minX = double.MaxValue, minY = double.MaxValue;
                double maxX = double.MinValue, maxY = double.MinValue;
                foreach (var p in vertices)
                {
                    if (p.X < minX) minX = p.X;
                    if (p.Y < minY) minY = p.Y;
                    if (p.X > maxX) maxX = p.X;
                    if (p.Y > maxY) maxY = p.Y;
                }
                bounds = new Rect(minX, minY, maxX - minX, maxY - minY);

                valid = true;
                foreach (var existing in _router.Obstacles)
                {
                    if (bounds.Intersects(existing.Bounds))
                    {
                        valid = false;
                        break;
                    }
                }
                attempts++;
            } while (!valid && attempts < 100);

            if (valid)
            {
                var obstacle = new Obstacle(vertices);
                _router.AddObstacle(obstacle);

                // Draw Polygon
                var polygon = new Polygon
                {
                    Points = new Avalonia.Collections.AvaloniaList<Point>(vertices),
                    Stroke = Brushes.Red,
                    StrokeThickness = 1,
                    Fill = null
                };
                CanvasArea.Children.Add(polygon);
            }
        }
        
        /*

        // // ��������ϰ����������
        // var rand = new Random();
        // double gridSize = 10;
        // int numObstacles = rand.Next(8, 13); // 8 �� 12 ��
        // for (int i = 0; i < numObstacles; i++)
        // {
        //     bool valid;
        //     Rect newRect;
        //     int attempts = 0;
        //     do
        //     {
        //         int widthUnits = rand.Next(5, 16); // 5-15 units -> 50-150
        //         int heightUnits = rand.Next(5, 16);
        //         double width = widthUnits * gridSize;
        //         double height = heightUnits * gridSize;
                
        //         // Constrain to Board Boundary
        //         double minX = _boardBoundary.X;
        //         double minY = _boardBoundary.Y;
        //         double maxX = _boardBoundary.Right - width;
        //         double maxY = _boardBoundary.Bottom - height;

        //         int xUnits = rand.Next((int)(minX / gridSize), (int)(maxX / gridSize) + 1);
        //         int yUnits = rand.Next((int)(minY / gridSize), (int)(maxY / gridSize) + 1);

        //         double x = xUnits * gridSize;
        //         double y = yUnits * gridSize;
        //         newRect = new Rect(x, y, width, height);
        //         valid = true;
        //         foreach (var existing in _router.Obstacles)
        //         {
        //             if (newRect.Intersects(existing.Bounds))
        //             {
        //                 valid = false;
        //                 break;
        //             }
        //         }
        //         attempts++;
        //     } while (!valid && attempts < 100);

        //     if (valid)
        //     {
        //         var obstacle = new Obstacle(newRect);
        //         _router.AddObstacle(obstacle);

        //         // ���� Rectangle �� Canvas
        //         var rect = new Rectangle
        //         {
        //             Width = newRect.Width,
        //             Height = newRect.Height,
        //             Stroke = Brushes.Red,
        //             StrokeThickness = 1,
        //             Fill = null
        //         };
        //         Canvas.SetLeft(rect, newRect.X);
        //         Canvas.SetTop(rect, newRect.Y);
        //         CanvasArea.Children.Add(rect);
        //     }
        // }

        */
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
                    // commit current dynamic route as static and get its end
                    var lastPoint = CommitCurrentRoute();

                    // place end marker at committed route end if available, otherwise at click
                    _endPoint = new Ellipse
                    {
                        Width = 10,
                        Height = 10,
                        Fill = Brushes.Red
                    };
                    var endPt = lastPoint ?? p;
                    Canvas.SetLeft(_endPoint, endPt.X - 5);
                    Canvas.SetTop(_endPoint, endPt.Y - 5);
                    CanvasArea.Children.Add(_endPoint);

                    // remove segment start marker (we're finished)
                    if (_segmentStartPoint != null)
                    {
                        CanvasArea.Children.Remove(_segmentStartPoint);
                        _segmentStartPoint = null;
                    }

                    // stop interactive routing
                    _last = null;
                }
            }
            else
            {
                // single click
                if (_last == null)
                {
                    // start routing (this is the overall route start)
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

                    // also create a segment start marker (shows where current segment starts)
                    _segmentStartPoint = new Ellipse
                    {
                        Width = 6,
                        Height = 6,
                        Fill = Brushes.Yellow
                    };
                    Canvas.SetLeft(_segmentStartPoint, p.X - 3);
                    Canvas.SetTop(_segmentStartPoint, p.Y - 3);
                    CanvasArea.Children.Add(_segmentStartPoint);

                    // ensure any previous dynamic state cleared but keep committed static lines
                    ClearDynamicLines();
                    // do not clear _staticLines so earlier routes persist
                }
                else
                {
                    // confirm current dynamic segment and continue from its end
                    // If the user clicked without moving the pointer, OnPointerMoved may not have populated _currentRoutePoints.
                    // Ensure we compute the route for the clicked point so the segment can be committed.
                    _currentRoutePoints = _router.Route(_last.Value, p, WidthSlider.Value) ?? new List<Point>();

                    // Draw dynamic (temporary) lines exactly like OnPointerMoved so user sees the tentative segment
                    foreach (var line in _dynamicLines)
                        CanvasArea.Children.Remove(line);
                    _dynamicLines.Clear();

                    for (int i = 0; i < _currentRoutePoints.Count - 1; i++)
                    {
                        var line = new Line
                        {
                            StartPoint = _currentRoutePoints[i],
                            EndPoint = _currentRoutePoints[i + 1],
                            Stroke = Brushes.DarkBlue,
                            StrokeThickness = WidthSlider.Value,
                            Opacity = 0.5,
                            StrokeLineCap = PenLineCap.Round
                        };
                        _dynamicLines.Add(line);
                        CanvasArea.Children.Add(line);
                    }

                    var lastPoint = CommitCurrentRoute();

                    // set new _last to the last point of committed route (if any)
                    if (lastPoint.HasValue)
                    {
                        _last = lastPoint.Value;

                        // move the segment start marker to new start (do NOT move the overall start marker)
                        if (_segmentStartPoint != null)
                        {
                            Canvas.SetLeft(_segmentStartPoint, _last.Value.X - 3);
                            Canvas.SetTop(_segmentStartPoint, _last.Value.Y - 3);
                        }

                        // Immediately compute and draw the next dynamic (temporary) route from new _last to current mouse position
                        var curMouse = Router.SnapToGrid(e.GetPosition(CanvasArea));
                        var pts = _router.Route(_last.Value, curMouse, WidthSlider.Value) ?? new List<Point>();
                        _currentRoutePoints = pts;

                        foreach (var line in _dynamicLines)
                            CanvasArea.Children.Remove(line);
                        _dynamicLines.Clear();

                        for (int i = 0; i < pts.Count - 1; i++)
                        {
                            var line = new Line
                            {
                                StartPoint = pts[i],
                                EndPoint = pts[i + 1],
                                Stroke = Brushes.DarkBlue,
                                StrokeThickness = WidthSlider.Value,
                                Opacity = 0.5,
                                StrokeLineCap = PenLineCap.Round
                            };
                            _dynamicLines.Add(line);
                            CanvasArea.Children.Add(line);
                        }
                    }
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
                if (_segmentStartPoint != null)
                {
                    CanvasArea.Children.Remove(_segmentStartPoint);
                    _segmentStartPoint = null;
                }
                if (_endPoint != null)
                {
                    CanvasArea.Children.Remove(_endPoint);
                    _endPoint = null;
                }

                ClearDynamicLines();
                ClearStaticLines();
                _last = null;
                _currentRoutePoints = new List<Point>();
            }
        }
    }

    // commit and return the end point of the committed route (or null if nothing committed)
    private Point? CommitCurrentRoute()
    {
        if (_currentRoutePoints == null || _currentRoutePoints.Count < 2)
            return null;

        double width = WidthSlider.Value;
        var trace = new Trace(width);

        var geometry = new StreamGeometry();
        using (var ctx = geometry.Open())
        {
            ctx.BeginFigure(_currentRoutePoints[0], false);
            for (int i = 1; i < _currentRoutePoints.Count; i++)
            {
                ctx.LineTo(_currentRoutePoints[i]);
                trace.AddSegment(_currentRoutePoints[i-1], _currentRoutePoints[i]);
            }
            ctx.EndFigure(false);
        }

        _router.AddTrace(trace);

        if (width < 3)
        {
            var path = new Path
            {
                Data = geometry,
                Stroke = new SolidColorBrush(Color.Parse("#800000FF")), // 50% Alpha Blue
                StrokeThickness = width,
                StrokeLineCap = PenLineCap.Round,
                StrokeJoin = PenLineJoin.Round
            };
            CanvasArea.Children.Add(path);
            _staticVisuals.Add(path);
        }
        else
        {
            // Border (50% transparent)
            var borderPath = new Path
            {
                Data = geometry,
                Stroke = new SolidColorBrush(Color.Parse("#800000FF")),
                StrokeThickness = width,
                StrokeLineCap = PenLineCap.Round,
                StrokeJoin = PenLineJoin.Round
            };
            
            // Fill (70% transparent -> 30% Alpha)
            var fillPath = new Path
            {
                Data = geometry,
                Stroke = new SolidColorBrush(Color.Parse("#4D0000FF")), 
                StrokeThickness = width - 2, // 1pt border on each side
                StrokeLineCap = PenLineCap.Round,
                StrokeJoin = PenLineJoin.Round
            };
            
            CanvasArea.Children.Add(borderPath);
            CanvasArea.Children.Add(fillPath);
            _staticVisuals.Add(borderPath);
            _staticVisuals.Add(fillPath);
        }

        var last = _currentRoutePoints.Last();
        ClearDynamicLines();
        _currentRoutePoints = new List<Point>();
        return last;
    }

    private void ClearDynamicLines()
    {
        foreach (var line in _dynamicLines)
            CanvasArea.Children.Remove(line);
        _dynamicLines.Clear();
    }

    private void ClearStaticLines()
    {
        foreach (var visual in _staticVisuals)
            CanvasArea.Children.Remove(visual);
        _staticVisuals.Clear();
        _router.ClearTraces();
    }

    private void OnPointerMoved(object? sender, PointerEventArgs e)
    {
        if (_last != null)
        {
            var p = Router.SnapToGrid(e.GetPosition(CanvasArea));
            var points = _router.Route(_last.Value, p, WidthSlider.Value) ?? new List<Point>();

            // store current route points for commit on click
            _currentRoutePoints = points;

            foreach (var line in _dynamicLines)
                CanvasArea.Children.Remove(line);
            _dynamicLines.Clear();

            for (int i = 0; i < points.Count - 1; i++)
            {
                var line = new Line
                {
                    StartPoint = points[i],
                    EndPoint = points[i + 1],
                    Stroke = Brushes.DarkBlue,
                    StrokeThickness = WidthSlider.Value,
                    Opacity = 0.5,
                    StrokeLineCap = PenLineCap.Round
                };
                _dynamicLines.Add(line);
                CanvasArea.Children.Add(line);
            }
        }
    }

    private void OnPourClick(object? sender, Avalonia.Interactivity.RoutedEventArgs e)
    {
        // Clear previous pour
        foreach (var v in _pourVisuals)
            CanvasArea.Children.Remove(v);
        _pourVisuals.Clear();

        double clearance = ClearanceSlider.Value;

        // 1. Prepare Subject (Board Boundary)
        var subject = new Paths64();
        var boundaryPath = MakePathFromRect(_boardBoundary, 0);
        subject.Add(boundaryPath);

        // 2. Prepare Clip (Obstacles + Traces)
        var clip = new Paths64();

        // Obstacles
        foreach (var obs in _router.Obstacles)
        {
            // Use polygon vertices
            var obsPath = MakePathFromPolygon(obs.Vertices, clearance);
            clip.Add(obsPath);
        }

        // Traces
        foreach (var trace in _router.Traces)
        {
             var path = new Path64();
             if (trace.Segments.Count > 0)
             {
                 path.Add(ToPoint64(trace.Segments[0].A));
                 foreach(var seg in trace.Segments)
                 {
                     path.Add(ToPoint64(seg.B));
                 }
             }
             
             // Inflate
             // Delta = (Width/2 + Clearance)
             double delta = (trace.Width / 2.0 + clearance) * Scale;
             var inflated = Clipper.InflatePaths(new Paths64 { path }, delta, JoinType.Round, EndType.Round);
             clip.AddRange(inflated);
        }

        // 3. Execute Difference using PolyTree
        var clipper = new Clipper64();
        clipper.AddSubject(subject);
        clipper.AddClip(clip);
        var solutionTree = new PolyTree64();
        clipper.Execute(ClipType.Difference, Clipper2Lib.FillRule.NonZero, solutionTree);

        // 4. Render
        RenderSolution(solutionTree);
    }

    private void RenderSolution(PolyPath64 pp)
    {
        // If pp is an outer polygon (not a hole, and not the root), render it
        // The root is technically a "hole" (container)
        
        bool isRoot = pp is PolyTree64;
        bool isHole = pp.IsHole; 
        
        if (!isRoot && !isHole && pp.Polygon != null)
        {
            // It is an island. Create a Path for it.
            var geometry = new PathGeometry();
            geometry.Figures ??= new PathFigures();
            geometry.FillRule = Avalonia.Media.FillRule.NonZero;

            // Add the island itself
            AddPathToGeometry(geometry, pp.Polygon);

            // Add its holes (immediate children)
            for (int i = 0; i < pp.Count; i++)
            {
                var child = pp[i];
                if (child.IsHole && child.Polygon != null)
                {
                    AddPathToGeometry(geometry, child.Polygon);
                }
            }

            var shape = new Path
            {
                Data = geometry,
                Fill = new SolidColorBrush(Color.Parse("#90CCCCC0")), // Semi-transparent Pink
                Stroke = new SolidColorBrush(Color.Parse("#FFCCCCC0")), // DeepPink
                StrokeThickness = 1
            };
            CanvasArea.Children.Insert(0, shape);
            _pourVisuals.Add(shape);
        }

        // Recurse
        for (int i = 0; i < pp.Count; i++)
        {
            RenderSolution(pp[i]);
        }
    }

    private void AddPathToGeometry(PathGeometry geometry, Path64? path)
    {
        if (path == null || path.Count == 0) return;

        var figure = new PathFigure();
        figure.StartPoint = ToPointD(path[0]);
        figure.IsClosed = true;
        figure.IsFilled = true;
        figure.Segments ??= new PathSegments();

        var polyLine = new PolyLineSegment();
        polyLine.Points ??= new Avalonia.Collections.AvaloniaList<Point>();
        
        for (int i = 1; i < path.Count; i++)
        {
            polyLine.Points.Add(ToPointD(path[i]));
        }
        figure.Segments.Add(polyLine);
        
        geometry.Figures.Add(figure);
    }

    private Path64 MakePathFromRect(Rect r, double expansion)
    {
        var p = new Path64();
        double extra = expansion;
        // Rect: x, y, w, h
        // TL, TR, BR, BL
        p.Add(ToPoint64(new Point(r.X - extra, r.Y - extra)));
        p.Add(ToPoint64(new Point(r.Right + extra, r.Y - extra)));
        p.Add(ToPoint64(new Point(r.Right + extra, r.Bottom + extra)));
        p.Add(ToPoint64(new Point(r.X - extra, r.Bottom + extra)));
        return p;
    }

    private Point64 ToPoint64(Point p)
    {
        return new Point64(p.X * Scale, p.Y * Scale);
    }

    private Point ToPointD(Point64 p)
    {
        return new Point(p.X / Scale, p.Y / Scale);
    }

    private List<Point> GenerateRandomPolygon(Random rand, double gridSize, Rect boundary)
    {
        int numVertices = rand.Next(3, 8); // 3 to 7 vertices
        
        // Pick a center point within boundary (roughly)
        double cx = rand.Next((int)(boundary.X / gridSize) + 2, (int)(boundary.Right / gridSize) - 2) * gridSize;
        double cy = rand.Next((int)(boundary.Y / gridSize) + 2, (int)(boundary.Bottom / gridSize) - 2) * gridSize;
        var center = new Point(cx, cy);

        // Generate angles
        var angles = new List<double>();
        for (int i = 0; i < numVertices; i++)
        {
            angles.Add(rand.NextDouble() * 2 * Math.PI);
        }
        angles.Sort();

        var vertices = new List<Point>();
        foreach (var angle in angles)
        {
            double radius = rand.Next(3, 10) * gridSize; // 30 to 100 pixels radius
            double x = cx + Math.Cos(angle) * radius;
            double y = cy + Math.Sin(angle) * radius;
            
            // Snap to grid
            x = Math.Round(x / gridSize) * gridSize;
            y = Math.Round(y / gridSize) * gridSize;
            
            // Clamp to boundary
            x = Math.Max(boundary.X, Math.Min(boundary.Right, x));
            y = Math.Max(boundary.Y, Math.Min(boundary.Bottom, y));

            vertices.Add(new Point(x, y));
        }

        // Ensure at least 3 unique vertices
        vertices = vertices.Distinct().ToList();
        if (vertices.Count < 3)
            return GenerateRandomPolygon(rand, gridSize, boundary); // Retry

        return vertices;
    }

    private Path64 MakePathFromPolygon(List<Point> vertices, double expansion)
    {
        var path = new Path64();
        foreach (var v in vertices)
        {
            path.Add(ToPoint64(v));
        }

        if (expansion > 0)
        {
            var paths = new Paths64();
            paths.Add(path);
            var inflated = Clipper.InflatePaths(paths, expansion * Scale, JoinType.Miter, EndType.Polygon);
            if (inflated.Count > 0)
                return inflated[0];
        }
        
        return path;
    }
}
