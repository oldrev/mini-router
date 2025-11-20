using System.Collections.Generic;
using Avalonia;

namespace MinimalRouter.Routing;

public class Trace
{
    public List<Segment> Segments { get; } = new();
    public double Width { get; }

    public Trace(double width)
    {
        Width = width;
    }

    public void AddSegment(Point a, Point b)
    {
        Segments.Add(new Segment(a, b));
    }
}
