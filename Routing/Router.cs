using System;
using System.Collections.Generic;
using System.Linq;
using Avalonia;

namespace MinimalRouter.Routing;

public class Router
{
    private readonly List<Obstacle> _obstacles = new List<Obstacle>();
    private const double Clearance = 6; // “间距” inflate

    public IReadOnlyList<Obstacle> Obstacles => _obstacles;

    public void AddObstacle(Obstacle o)
    {
        _obstacles.Add(o);
    }

    // 判断线段是否与任意障碍物碰撞
    private bool Collides(Segment seg)
    {
        // Use inflated rectangle around obstacles for clearance
        foreach (var o in _obstacles)
        {
            var infRect = new Rect(
                o.Bounds.X - Clearance,
                o.Bounds.Y - Clearance,
                o.Bounds.Width + Clearance * 2,
                o.Bounds.Height + Clearance * 2
            );

            // Quick reject by bounding box
            var segBox = seg.BoundingBox(0);
            if (!segBox.Intersects(infRect))
                continue;

            if (SegmentIntersectsRect(seg, infRect))
                return true;
        }
        return false;
    }

    // 生成可能的拐点（障碍物四角 + 起终点）
    private List<Point> CollectCandidatePoints(Point start, Point end)
    {
        var pts = new List<Point> { start, end };

        foreach (var o in _obstacles)
        {
            // Use corners of obstacle bounds
            pts.Add(new Point(o.Bounds.X, o.Bounds.Y)); // TopLeft
            pts.Add(new Point(o.Bounds.X + o.Bounds.Width, o.Bounds.Y)); // TopRight
            pts.Add(new Point(o.Bounds.X, o.Bounds.Y + o.Bounds.Height)); // BottomLeft
            pts.Add(new Point(o.Bounds.X + o.Bounds.Width, o.Bounds.Y + o.Bounds.Height)); // BottomRight
        }

        return pts;
    }

    // 判断两个点是否有直接视线（可走直线）
    private bool HasLineOfSight(Point a, Point b)
    {
        var seg = new Segment(a, b);
        return !Collides(seg);
    }

    public List<Point> Route(Point start, Point end)
    {
        var pts = CollectCandidatePoints(start, end);

        // Build adjacency list for visibility graph
        var neighbors = new Dictionary<Point, List<Point>>();
        foreach (var p in pts)
            neighbors[p] = new List<Point>();

        for (int i = 0; i < pts.Count; i++)
        for (int j = i + 1; j < pts.Count; j++)
        {
            var p1 = pts[i];
            var p2 = pts[j];
            if (p1 == p2) continue;
            if (HasLineOfSight(p1, p2))
            {
                neighbors[p1].Add(p2);
                neighbors[p2].Add(p1);
            }
        }

        // A* search on points
        var cameFrom = new Dictionary<Point, Point>();
        var gScore = new Dictionary<Point, double>();
        var fScore = new Dictionary<Point, double>();
        var openSet = new PriorityQueue<Point, double>();

        foreach (var p in pts)
        {
            gScore[p] = double.PositiveInfinity;
            fScore[p] = double.PositiveInfinity;
        }

        gScore[start] = 0;
        fScore[start] = Distance(start, end);
        openSet.Enqueue(start, fScore[start]);

        var closed = new HashSet<Point>();

        while (openSet.Count > 0)
        {
            var current = openSet.Dequeue();
            if (ApproximatelyEqual(current, end))
            {
                // Reconstruct path
                var path = new List<Point> { current };
                while (cameFrom.ContainsKey(path.Last()))
                    path.Add(cameFrom[path.Last()]);
                path.Reverse();
                // Ensure end is the exact provided end point
                if (!ApproximatelyEqual(path.Last(), end))
                    path.Add(end);

                return Simplify(path);
            }

            closed.Add(current);

            foreach (var neighbor in neighbors[current])
            {
                if (closed.Contains(neighbor)) continue;

                var tentativeG = gScore[current] + Distance(current, neighbor);
                if (tentativeG < gScore[neighbor])
                {
                    cameFrom[neighbor] = current;
                    gScore[neighbor] = tentativeG;
                    fScore[neighbor] = tentativeG + Distance(neighbor, end);
                    openSet.Enqueue(neighbor, fScore[neighbor]);
                }
            }
        }

        return new List<Point>();
    }

    // 去掉 colinear 点
    private List<Point> Simplify(List<Point> pts)
    {
        if (pts.Count < 3) return pts;

        var result = new List<Point>();
        result.Add(pts[0]);

        for (int i = 1; i < pts.Count - 1; i++)
        {
            var a = pts[i - 1];
            var b = pts[i];
            var c = pts[i + 1];

            // 共线检查
            var cross = (b.X - a.X) * (c.Y - b.Y) - (b.Y - a.Y) * (c.X - b.X);

            if (Math.Abs(cross) > 0.01)   // 不共线就保留
                result.Add(b);
        }

        result.Add(pts.Last());
        return result;
    }

    private static double Distance(Point a, Point b)
    {
        var dx = a.X - b.X;
        var dy = a.Y - b.Y;
        return Math.Sqrt(dx * dx + dy * dy);
    }

    private static bool ApproximatelyEqual(Point a, Point b, double eps = 1e-6)
    {
        return Math.Abs(a.X - b.X) < eps && Math.Abs(a.Y - b.Y) < eps;
    }

    // Segment-rectangle intersection tests
    private static bool SegmentIntersectsRect(Segment s, Rect r)
    {
        // If either endpoint inside rect -> intersects
        if (r.Contains(s.A) || r.Contains(s.B))
            return true;

        var topLeft = new Point(r.X, r.Y);
        var topRight = new Point(r.X + r.Width, r.Y);
        var bottomLeft = new Point(r.X, r.Y + r.Height);
        var bottomRight = new Point(r.X + r.Width, r.Y + r.Height);

        // rectangle edges
        if (SegmentsIntersect(s.A, s.B, topLeft, topRight)) return true;
        if (SegmentsIntersect(s.A, s.B, topRight, bottomRight)) return true;
        if (SegmentsIntersect(s.A, s.B, bottomRight, bottomLeft)) return true;
        if (SegmentsIntersect(s.A, s.B, bottomLeft, topLeft)) return true;

        return false;
    }

    private static bool SegmentsIntersect(Point p1, Point p2, Point p3, Point p4)
    {
        // Check general segment intersection using orientations
        double d1 = Direction(p3, p4, p1);
        double d2 = Direction(p3, p4, p2);
        double d3 = Direction(p1, p2, p3);
        double d4 = Direction(p1, p2, p4);

        if (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) &&
            ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0)))
            return true;

        if (Math.Abs(d1) < 1e-9 && OnSegment(p3, p4, p1)) return true;
        if (Math.Abs(d2) < 1e-9 && OnSegment(p3, p4, p2)) return true;
        if (Math.Abs(d3) < 1e-9 && OnSegment(p1, p2, p3)) return true;
        if (Math.Abs(d4) < 1e-9 && OnSegment(p1, p2, p4)) return true;

        return false;
    }

    private static double Direction(Point a, Point b, Point c)
    {
        return (c.X - a.X) * (b.Y - a.Y) - (c.Y - a.Y) * (b.X - a.X);
    }

    private static bool OnSegment(Point a, Point b, Point p)
    {
        return Math.Min(a.X, b.X) <= p.X && p.X <= Math.Max(a.X, b.X) &&
               Math.Min(a.Y, b.Y) <= p.Y && p.Y <= Math.Max(a.Y, b.Y);
    }
}
