using System.Collections.Generic;
using Avalonia;
using System.Linq;

namespace MinimalRouter.Routing;

public class Obstacle
{
    public List<Point> Vertices { get; }
    public Rect Bounds { get; }

    public Obstacle(IEnumerable<Point> vertices)
    {
        Vertices = vertices.ToList();
        
        // Compute bounding box
        double minX = double.MaxValue, minY = double.MaxValue;
        double maxX = double.MinValue, maxY = double.MinValue;

        foreach (var p in Vertices)
        {
            if (p.X < minX) minX = p.X;
            if (p.Y < minY) minY = p.Y;
            if (p.X > maxX) maxX = p.X;
            if (p.Y > maxY) maxY = p.Y;
        }
        Bounds = new Rect(minX, minY, maxX - minX, maxY - minY);
    }
}
