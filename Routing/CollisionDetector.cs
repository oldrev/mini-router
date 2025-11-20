using System;
using System.Collections.Generic;
using Avalonia;

namespace MinimalRouter.Routing;

public static class CollisionDetector
{
    private const double Epsilon = 1e-9;

    public static bool IsColliding(
        Segment segment,
        double width,
        IEnumerable<Obstacle> obstacles,
        IEnumerable<Trace> traces,
        double clearance,
        HashSet<Trace>? ignoredTraces = null)
    {
        // 1. Check Obstacles
        foreach (var obs in obstacles)
        {
            // Quick bounding box check first
            double inflate = clearance + width / 2.0;
            var expandedRect = obs.Bounds.Inflate(inflate);

            if (!SegmentIntersectsRect(segment, expandedRect))
                continue;

            // Detailed polygon check
            if (SegmentIntersectsPolygon(segment, obs.Vertices, inflate))
                return true;
        }

        // 2. Check Traces
        foreach (var trace in traces)
        {
            if (ignoredTraces != null && ignoredTraces.Contains(trace))
                continue;

            foreach (var s in trace.Segments)
            {
                // Check if connected (share endpoint)
                if (AreConnected(segment, s))
                {
                    continue;
                }

                double requiredDist = clearance + (width + trace.Width) / 2.0;
                double dist = DistanceSegmentSegment(segment.A, segment.B, s.A, s.B);

                if (dist < requiredDist - Epsilon)
                {
                    return true;
                }
            }
        }

        return false;
    }

    public static bool IsPointInCollision(
        Point p,
        double width,
        IEnumerable<Obstacle> obstacles,
        IEnumerable<Trace> traces,
        double clearance,
        HashSet<Trace>? ignoredTraces = null)
    {
        // Check obstacles
        foreach (var obs in obstacles)
        {
            double inflate = clearance + width / 2.0;
            if (!obs.Bounds.Inflate(inflate).Contains(p)) continue;

            if (IsPointInPolygon(p, obs.Vertices, inflate)) return true;
        }

        // Check traces
        foreach (var trace in traces)
        {
            if (ignoredTraces != null && ignoredTraces.Contains(trace))
                continue;

            foreach (var s in trace.Segments)
            {
                double requiredDist = clearance + (width + trace.Width) / 2.0;
                if (DistancePointSegment(p, s.A, s.B) < requiredDist - Epsilon)
                    return true;
            }
        }
        return false;
    }

    // --- Polygon Helpers ---

    private static bool SegmentIntersectsPolygon(Segment s, List<Point> vertices, double inflation)
    {
        // Check if either endpoint is inside the inflated polygon
        if (IsPointInPolygon(s.A, vertices, inflation) || IsPointInPolygon(s.B, vertices, inflation))
            return true;

        // Check intersection with any polygon edge (inflated logic is complex, 
        // so we approximate by checking distance to edges)
        // If the segment is "too close" to any edge, it's a collision.

        for (int i = 0; i < vertices.Count; i++)
        {
            var p1 = vertices[i];
            var p2 = vertices[(i + 1) % vertices.Count];

            double dist = DistanceSegmentSegment(s.A, s.B, p1, p2);
            if (dist < inflation - Epsilon)
                return true;
        }

        return false;
    }

    private static bool IsPointInPolygon(Point p, List<Point> vertices, double inflation)
    {
        // First check if point is close to any edge (within inflation)
        for (int i = 0; i < vertices.Count; i++)
        {
            var p1 = vertices[i];
            var p2 = vertices[(i + 1) % vertices.Count];
            if (DistancePointSegment(p, p1, p2) < inflation - Epsilon)
                return true;
        }

        // Then check if point is strictly inside the polygon (Ray Casting)
        // If it is inside, it's definitely a collision regardless of inflation
        bool inside = false;
        for (int i = 0, j = vertices.Count - 1; i < vertices.Count; j = i++)
        {
            if (((vertices[i].Y > p.Y) != (vertices[j].Y > p.Y)) &&
                (p.X < (vertices[j].X - vertices[i].X) * (p.Y - vertices[i].Y) / (vertices[j].Y - vertices[i].Y) + vertices[i].X))
            {
                inside = !inside;
            }
        }
        return inside;
    }

    private static bool AreConnected(Segment s1, Segment s2)
    {
        return ApproximatelyEqual(s1.A, s2.A) || ApproximatelyEqual(s1.A, s2.B) ||
               ApproximatelyEqual(s1.B, s2.A) || ApproximatelyEqual(s1.B, s2.B);
    }

    private static bool ApproximatelyEqual(Point a, Point b)
    {
        return Math.Abs(a.X - b.X) < Epsilon && Math.Abs(a.Y - b.Y) < Epsilon;
    }

    // --- Geometry Helpers ---

    public static bool SegmentIntersectsRect(Segment s, Rect r)
    {
        // If either endpoint inside rect -> intersects
        if (r.Contains(s.A) || r.Contains(s.B))
            return true;

        var topLeft = new Point(r.X, r.Y);
        var topRight = new Point(r.Right, r.Y);
        var bottomLeft = new Point(r.X, r.Bottom);
        var bottomRight = new Point(r.Right, r.Bottom);

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

        if (Math.Abs(d1) < Epsilon && OnSegment(p3, p4, p1)) return true;
        if (Math.Abs(d2) < Epsilon && OnSegment(p3, p4, p2)) return true;
        if (Math.Abs(d3) < Epsilon && OnSegment(p1, p2, p3)) return true;
        if (Math.Abs(d4) < Epsilon && OnSegment(p1, p2, p4)) return true;

        return false;
    }

    private static double Direction(Point a, Point b, Point c)
    {
        return (c.X - a.X) * (b.Y - a.Y) - (c.Y - a.Y) * (b.X - a.X);
    }

    private static bool OnSegment(Point a, Point b, Point p)
    {
        return Math.Min(a.X, b.X) <= p.X + Epsilon && p.X <= Math.Max(a.X, b.X) + Epsilon &&
               Math.Min(a.Y, b.Y) <= p.Y + Epsilon && p.Y <= Math.Max(a.Y, b.Y) + Epsilon;
    }

    public static double DistancePointSegment(Point p, Point s1, Point s2)
    {
        double dx = s2.X - s1.X;
        double dy = s2.Y - s1.Y;
        if (Math.Abs(dx) < Epsilon && Math.Abs(dy) < Epsilon) return Distance(p, s1);

        double t = ((p.X - s1.X) * dx + (p.Y - s1.Y) * dy) / (dx * dx + dy * dy);
        t = Math.Max(0, Math.Min(1, t));

        double closestX = s1.X + t * dx;
        double closestY = s1.Y + t * dy;

        return Distance(p, new Point(closestX, closestY));
    }

    private static double Distance(Point a, Point b)
    {
        double dx = a.X - b.X;
        double dy = a.Y - b.Y;
        return Math.Sqrt(dx * dx + dy * dy);
    }

    // Robust Distance between two segments
    public static double DistanceSegmentSegment(Point S1_P0, Point S1_P1, Point S2_P0, Point S2_P1)
    {
        // Based on http://geomalgorithms.com/a07-_distance.html
        Vector u = S1_P1 - S1_P0;
        Vector v = S2_P1 - S2_P0;
        Vector w = S1_P0 - S2_P0;
        double a = u * u;         // always >= 0
        double b = u * v;
        double c = v * v;         // always >= 0
        double d = u * w;
        double e = v * w;
        double D = a * c - b * b; // always >= 0
        double sc, sN, sD = D;       // sc = sN / sD, default sD = D >= 0
        double tc, tN, tD = D;       // tc = tN / tD, default tD = D >= 0

        // compute the line parameters of the two closest points
        if (D < Epsilon)
        { // the lines are almost parallel
            sN = 0.0;         // force using point P0 on segment S1
            sD = 1.0;         // to prevent possible division by 0.0 later
            tN = e;
            tD = c;
        }
        else
        {                 // get the closest points on the infinite lines
            sN = (b * e - c * d);
            tN = (a * e - b * d);
            if (sN < 0.0)
            {        // sc < 0 => the s=0 edge is visible
                sN = 0.0;
                tN = e;
                tD = c;
            }
            else if (sN > sD)
            {  // sc > 1  => the s=1 edge is visible
                sN = sD;
                tN = e + b;
                tD = c;
            }
        }

        if (tN < 0.0)
        {            // tc < 0 => the t=0 edge is visible
            tN = 0.0;
            // recompute sc for this edge
            if (-d < 0.0)
                sN = 0.0;
            else if (-d > a)
                sN = sD;
            else
            {
                sN = -d;
                sD = a;
            }
        }
        else if (tN > tD)
        {      // tc > 1  => the t=1 edge is visible
            tN = tD;
            // recompute sc for this edge
            if ((-d + b) < 0.0)
                sN = 0;
            else if ((-d + b) > a)
                sN = sD;
            else
            {
                sN = (-d + b);
                sD = a;
            }
        }

        // finally do the division to get sc and tc
        sc = (Math.Abs(sN) < Epsilon ? 0.0 : sN / sD);
        tc = (Math.Abs(tN) < Epsilon ? 0.0 : tN / tD);

        // get the difference of the two closest points
        Vector dP = w + (sc * u) - (tc * v);  // =  S1(sc) - S2(tc)

        return dP.Length;   // return the closest distance
    }
}
