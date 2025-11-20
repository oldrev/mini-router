using System;
using Avalonia;
using Avalonia.Media;

namespace MinimalRouter.Routing;

public struct Segment
{
    public Point A;
    public Point B;

    public Segment(Point a, Point b)
    {
        A = a; 
        B = b;
    }

    public Rect BoundingBox(double inflate)
    {
        return new Rect(
            Math.Min(A.X, B.X) - inflate,
            Math.Min(A.Y, B.Y) - inflate,
            Math.Abs(A.X - B.X) + inflate * 2,
            Math.Abs(A.Y - B.Y) + inflate * 2
        );
    }

    public void Draw(DrawingContext dc, Pen pen)
    {
        dc.DrawLine(pen, A, B);
    }
}
