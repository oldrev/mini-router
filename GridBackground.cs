using Avalonia;
using Avalonia.Controls;
using Avalonia.Media;

namespace MinimalRouter;

public class GridBackground : Control
{
    public override void Render(DrawingContext context)
    {
        base.Render(context);
        var pen = new Pen(Brushes.LightGray, 1);
        double gridSize = 10;
        for (double x = 0; x < Bounds.Width; x += gridSize)
        {
            context.DrawLine(pen, new Point(x, 0), new Point(x, Bounds.Height));
        }
        for (double y = 0; y < Bounds.Height; y += gridSize)
        {
            context.DrawLine(pen, new Point(0, y), new Point(Bounds.Width, y));
        }
    }
}