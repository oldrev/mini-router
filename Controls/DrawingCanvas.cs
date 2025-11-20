using System;
using Avalonia.Controls;
using Avalonia.Media;
using Avalonia;

namespace MinimalRouter;

public class DrawingCanvas : Control
{
    public IBrush? Background { get; set; }

    public event Action<object?, DrawingContext>? Draw;

    public override void Render(DrawingContext context)
    {
        base.Render(context);

        if (Background != null)
        {
            var rect = new Rect(0, 0, Bounds.Width, Bounds.Height);
            context.FillRectangle(Background, rect);
        }

        Draw?.Invoke(this, context);
    }
}
