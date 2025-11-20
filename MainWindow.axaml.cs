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

    public MainWindow()
    {
        InitializeComponent();

        // 随机生成障碍物，基于网格
        var rand = new Random();
        double gridSize = 10;
        int numObstacles = rand.Next(5, 9); // 5 到 8 个
        for (int i = 0; i < numObstacles; i++)
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
            var obstacle = new Obstacle(new Rect(x, y, width, height));
            _router.AddObstacle(obstacle);

            // 添加 Rectangle 到 Canvas
            var rect = new Rectangle
            {
                Width = width,
                Height = height,
                Stroke = Brushes.Red,
                StrokeThickness = 1,
                Fill = null
            };
            Canvas.SetLeft(rect, x);
            Canvas.SetTop(rect, y);
            CanvasArea.Children.Add(rect);
        }

        CanvasArea.PointerPressed += OnClick;
        CanvasArea.PointerMoved += OnPointerMoved;
    }

    private void OnClick(object? sender, PointerPressedEventArgs e)
    {
        var p = e.GetPosition(CanvasArea);
        var props = e.GetCurrentPoint(this).Properties;

        if (props.IsLeftButtonPressed)
        {
            if (_last == null)
            {
                // 设置起点
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

                // 开始监听鼠标移动
                CanvasArea.PointerMoved += OnPointerMoved;
            }
            else
            {
                // 固定终点
                CanvasArea.PointerMoved -= OnPointerMoved;

                // 添加终点
                _endPoint = new Ellipse
                {
                    Width = 10,
                    Height = 10,
                    Fill = Brushes.Red
                };
                Canvas.SetLeft(_endPoint, p.X - 5);
                Canvas.SetTop(_endPoint, p.Y - 5);
                CanvasArea.Children.Add(_endPoint);

                _last = null;
            }
        }
        else if (props.IsRightButtonPressed)
        {
            if (_last != null)
            {
                // 取消起点
                CanvasArea.PointerMoved -= OnPointerMoved;
                if (_startPoint != null)
                {
                    CanvasArea.Children.Remove(_startPoint);
                    _startPoint = null;
                }
                // 清除线段
                var linesToRemove = CanvasArea.Children.Where(c => c is Line).ToList();
                foreach (var line in linesToRemove)
                    CanvasArea.Children.Remove(line);
                _last = null;
            }
        }
    }

    private void OnPointerMoved(object? sender, PointerEventArgs e)
    {
        if (_last != null)
        {
            var p = e.GetPosition(CanvasArea);
            var points = _router.Route(_last.Value, p);

            // 清除之前的线段
            var linesToRemove = CanvasArea.Children.Where(c => c is Line).ToList();
            foreach (var line in linesToRemove)
                CanvasArea.Children.Remove(line);

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
                CanvasArea.Children.Add(line);
            }
        }
    }
}
