using System;
using System.Collections.Generic;
using System.Linq;
using Avalonia;
using System.Runtime.InteropServices;

namespace MinimalRouter.Routing;

public class Router
{
    private readonly List<Obstacle> _obstacles = new List<Obstacle>();
    private const double Clearance = 6; // “间距” inflate

    // Bias factor <1 to prefer diagonal edges (45°)
    private const double DiagonalBias = 0.8;
    private const double DiagonalTolerance = 2.0; // pixels tolerance for near-45°

    // Limit search area around start/end to reduce candidate set and CPU usage
    private const double MaxSearchRange = 300.0; // pixels radius around start/end center

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

    // 生成可能的拐点（基于障碍物扩展矩形的网格线交点 + 起终点 + 45° 对角交点）
    private List<Point> CollectCandidatePoints(Point start, Point end)
    {
        var pts = new List<Point> { start, end };

        // compute a bounded search rectangle around start/end to limit candidate generation
        var center = new Point((start.X + end.X) / 2.0, (start.Y + end.Y) / 2.0);
        var searchRect = new Rect(center.X - MaxSearchRange, center.Y - MaxSearchRange, MaxSearchRange * 2, MaxSearchRange * 2);
        // expand a bit for clearance
        double inflateAmount = Clearance + 20;
        searchRect = new Rect(searchRect.X - inflateAmount, searchRect.Y - inflateAmount, searchRect.Width + inflateAmount * 2, searchRect.Height + inflateAmount * 2);

        // collect unique X and Y coordinates from inflated obstacle edges and start/end
        var xs = new HashSet<double> { start.X, end.X };
        var ys = new HashSet<double> { start.Y, end.Y };

        // diagonal constants (x+y and x-y) to create diagonal intersection candidates
        var diagPlus = new HashSet<double> { start.X + start.Y, end.X + end.Y };
        var diagMinus = new HashSet<double> { start.X - start.Y, end.X - end.Y };

        foreach (var o in _obstacles)
        {
            var infRect = new Rect(
                o.Bounds.X - Clearance,
                o.Bounds.Y - Clearance,
                o.Bounds.Width + Clearance * 2,
                o.Bounds.Height + Clearance * 2
            );

            // skip obstacles entirely outside the search rect
            if (!infRect.Intersects(searchRect))
                continue;

            // add corners as candidate points (snapped)
            var corners = new[] {
                new Point(infRect.X, infRect.Y),
                new Point(infRect.X + infRect.Width, infRect.Y),
                new Point(infRect.X, infRect.Y + infRect.Height),
                new Point(infRect.X + infRect.Width, infRect.Y + infRect.Height)
            };

            foreach (var c in corners)
            {
                var sc = SnapToGrid(c);
                if (searchRect.Contains(sc))
                {
                    pts.Add(sc);
                    diagPlus.Add(c.X + c.Y);
                    diagMinus.Add(c.X - c.Y);
                }
            }

            if (searchRect.Contains(new Point(infRect.X, infRect.Y))) xs.Add(infRect.X);
            if (searchRect.Contains(new Point(infRect.X + infRect.Width, infRect.Y + infRect.Height))) xs.Add(infRect.X + infRect.Width);
            if (searchRect.Contains(new Point(infRect.X, infRect.Y))) ys.Add(infRect.Y);
            if (searchRect.Contains(new Point(infRect.X + infRect.Width, infRect.Y + infRect.Height))) ys.Add(infRect.Y + infRect.Height);
        }

        // Form grid intersections from those Xs and Ys (filter by searchRect)
        foreach (var x in xs)
        foreach (var y in ys)
        {
            var p = SnapToGrid(new Point(x, y));
            if (searchRect.Contains(p) && !IsInsideAnyObstacle(p)) pts.Add(p);
        }

        // Add diagonal intersections for x+y = const and x-y = const, but only within searchRect
        foreach (var d in diagPlus)
        {
            foreach (var x in xs)
            {
                var y = d - x;
                var p = SnapToGrid(new Point(x, y));
                if (searchRect.Contains(p) && !IsInsideAnyObstacle(p)) pts.Add(p);
            }
            foreach (var y in ys)
            {
                var x = d - y;
                var p = SnapToGrid(new Point(x, y));
                if (searchRect.Contains(p) && !IsInsideAnyObstacle(p)) pts.Add(p);
            }
        }

        foreach (var d in diagMinus)
        {
            foreach (var x in xs)
            {
                var y = x - d;
                var p = SnapToGrid(new Point(x, y));
                if (searchRect.Contains(p) && !IsInsideAnyObstacle(p)) pts.Add(p);
            }
            foreach (var y in ys)
            {
                var x = d + y;
                var p = SnapToGrid(new Point(x, y));
                if (searchRect.Contains(p) && !IsInsideAnyObstacle(p)) pts.Add(p);
            }
        }

        return pts.Distinct().ToList();
    }

    private bool IsInsideAnyObstacle(Point p)
    {
        foreach (var o in _obstacles)
        {
            var inf = new Rect(
                o.Bounds.X - Clearance,
                o.Bounds.Y - Clearance,
                o.Bounds.Width + Clearance * 2,
                o.Bounds.Height + Clearance * 2
            );
            if (inf.Contains(p)) return true;
        }
        return false;
    }

    // 判断两个点是否有直接视线（可走直线）
    private bool HasLineOfSight(Point a, Point b)
    {
        var seg = new Segment(a, b);
        if (Collides(seg))
            return false;

        double dx = b.X - a.X;
        double dy = b.Y - a.Y;
        const double eps = 1e-6;

        // allow horizontal or vertical
        if (Math.Abs(dx) < eps || Math.Abs(dy) < eps)
            return true;

        // allow near-45-degree diagonals within tolerance
        if (Math.Abs(Math.Abs(dx) - Math.Abs(dy)) <= DiagonalTolerance)
            return true;

        return false;
    }

    // Weighted distance: prefer diagonal moves by applying DiagonalBias
    private static double WeightedDistance(Point a, Point b)
    {
        var dx = Math.Abs(a.X - b.X);
        var dy = Math.Abs(a.Y - b.Y);
        var dist = Math.Sqrt(dx * dx + dy * dy);
        // consider diagonal if nearly equal
        if (Math.Abs(dx - dy) <= DiagonalTolerance)
            return dist * DiagonalBias;
        return dist;
    }

    // Heuristic for A*: Euclidean distance (admissible) multiplied by DiagonalBias to encourage diagonals slightly
    private static double Heuristic(Point a, Point b)
    {
        var d = Distance(a, b);
        // small bias to prefer solutions with diagonals
        return d * DiagonalBias;
    }

    public List<Point> Route(Point start, Point end)
    {
        start = SnapToGrid(start);
        end = SnapToGrid(end);

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
        fScore[start] = Heuristic(start, end);
        openSet.Enqueue(start, fScore[start]);

        var closed = new HashSet<Point>();

        // guard against pathological infinite loops: limit iterations
        int maxSteps = Math.Max(10000, pts.Count * 500);
        int steps = 0;

        while (openSet.Count > 0)
        {
            if (++steps > maxSteps)
            {
                // abort search to avoid UI freeze
                return new List<Point>();
            }

            var current = openSet.Dequeue();
            if (ApproximatelyEqual(current, end))
            {
                // Reconstruct path (cycle-safe)
                var path = new List<Point> { current };
                var visited = new HashSet<Point> { current };
                while (cameFrom.ContainsKey(path.Last()))
                {
                    var next = cameFrom[path.Last()];
                    // detect cycle
                    if (visited.Contains(next))
                    {
                        // cycle detected; abort
                        return new List<Point>();
                    }
                    path.Add(next);
                    visited.Add(next);
                }
                path.Reverse();
                // Ensure end is the exact provided end point
                if (!ApproximatelyEqual(path.Last(), end))
                    path.Add(end);

                var simplified = Simplify(path);
                return ApplyChamfers(simplified);
            }

            closed.Add(current);

            // Use batch distance computation for all neighbors to reduce per-neighbor overhead
            var nbrs = neighbors[current];
            if (nbrs.Count == 0) continue;

            // Compute distances from current to all neighbors and from neighbors to end in batches
            var distCurrToNbr = DistanceBatch(current, CollectionsMarshal.AsSpan(nbrs));
            var distNbrToEnd = DistanceBatch(end, CollectionsMarshal.AsSpan(nbrs));

            for (int ni = 0; ni < nbrs.Count; ni++)
            {
                var neighbor = nbrs[ni];
                if (closed.Contains(neighbor)) continue;

                var tentativeG = gScore[current] + distCurrToNbr[ni];
                if (tentativeG < gScore[neighbor])
                {
                    cameFrom[neighbor] = current;
                    gScore[neighbor] = tentativeG;
                    fScore[neighbor] = tentativeG + distNbrToEnd[ni];
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

    // Replace 90-degree corners with 45-degree chamfers (produces 135° turns)
    private List<Point> ApplyChamfers(List<Point> path)
    {
        if (path.Count < 3) return path;

        var result = new List<Point> { path[0] };
        const double eps = 1e-6;
        const double defaultChamfer = 10.0; // chamfer length in pixels (grid units)

        for (int i = 1; i < path.Count - 1; i++)
        {
            var prev = path[i - 1];
            var cur = path[i];
            var next = path[i + 1];

            // vectors from corner
            var vin = new Point(prev.X - cur.X, prev.Y - cur.Y);
            var vout = new Point(next.X - cur.X, next.Y - cur.Y);

            // Check for right-angle (orthogonal) turn: one incoming axis zero, other outgoing axis zero
            bool vinH = Math.Abs(vin.Y) < eps && Math.Abs(vin.X) > eps;
            bool vinV = Math.Abs(vin.X) < eps && Math.Abs(vin.Y) > eps;
            bool voutH = Math.Abs(vout.Y) < eps && Math.Abs(vout.X) > eps;
            bool voutV = Math.Abs(vout.X) < eps && Math.Abs(vout.Y) > eps;

            bool isRightAngle = (vinH && voutV) || (vinV && voutH);

            if (!isRightAngle)
            {
                // keep original corner
                result.Add(cur);
                continue;
            }

            // determine available lengths along each leg
            double lenIn = Math.Sqrt(vin.X * vin.X + vin.Y * vin.Y);
            double lenOut = Math.Sqrt(vout.X * vout.X + vout.Y * vout.Y);
            double chamfer = Math.Min(defaultChamfer, Math.Min(lenIn, lenOut) / 2.0);
            if (chamfer < 1.0)
            {
                result.Add(cur);
                continue;
            }

            // unit directions from corner
            var dirIn = Normalize(vin); // points from corner toward prev
            var dirOut = Normalize(vout); // points from corner toward next

            var p1 = new Point(cur.X + dirIn.X * chamfer, cur.Y + dirIn.Y * chamfer);
            var p2 = new Point(cur.X + dirOut.X * chamfer, cur.Y + dirOut.Y * chamfer);

            // Snap to grid for clean rendering
            p1 = SnapToGrid(p1);
            p2 = SnapToGrid(p2);

            // Check collisions for the three replacement segments: prev->p1, p1->p2, p2->next
            var s1 = new Segment(prev, p1);
            var s2 = new Segment(p1, p2);
            var s3 = new Segment(p2, next);

            if (Collides(s1) || Collides(s2) || Collides(s3))
            {
                // cannot apply chamfer safely
                result.Add(cur);
                continue;
            }

            // append chamfer points (replace corner)
            result.Add(p1);
            result.Add(p2);
        }

        // add last point
        result.Add(path.Last());

        // Remove possible redundant consecutive duplicates introduced by snapping
        var final = new List<Point>();
        foreach (var p in result)
        {
            if (final.Count == 0 || !ApproximatelyEqual(final.Last(), p))
                final.Add(p);
        }

        return final;
    }

    private static Point Normalize(Point v)
    {
        var len = Math.Sqrt(v.X * v.X + v.Y * v.Y);
        if (len < 1e-9) return new Point(0, 0);
        return new Point(v.X / len, v.Y / len);
    }

    // Vectorized batch distance calculation: computes Euclidean distances from `a` to each point in `pts`.
    // Accepts ReadOnlySpan<Point> to avoid allocations when possible. Falls back to scalar if vectorization not available or list small.
    private static double[] DistanceBatch(Point a, ReadOnlySpan<Point> pts)
    {
        int n = pts.Length;
        var res = new double[n];
        if (n == 0) return res;

        // For small n, scalar is cheaper
        if (n < 8 || !System.Numerics.Vector.IsHardwareAccelerated)
        {
            for (int i = 0; i < n; i++)
            {
                var dx = a.X - pts[i].X;
                var dy = a.Y - pts[i].Y;
                res[i] = Math.Sqrt(dx * dx + dy * dy);
            }
            return res;
        }

        // Prepare dx/dy arrays
        var dxs = new double[n];
        var dys = new double[n];
        for (int i = 0; i < n; i++)
        {
            dxs[i] = a.X - pts[i].X;
            dys[i] = a.Y - pts[i].Y;
        }

        int W = System.Numerics.Vector<double>.Count;
        var sq = new double[n];

        int iIdx = 0;
        // vectorized bulk
        for (; iIdx + W <= n; iIdx += W)
        {
            var vdx = new System.Numerics.Vector<double>(dxs, iIdx);
            var vdy = new System.Numerics.Vector<double>(dys, iIdx);
            var vres = vdx * vdx + vdy * vdy; // per-lane squared distances
            vres.CopyTo(sq, iIdx);
        }

        // tail
        for (int t = iIdx; t < n; t++)
            sq[t] = dxs[t] * dxs[t] + dys[t] * dys[t];

        // sqrt pass
        for (int k = 0; k < n; k++)
            res[k] = Math.Sqrt(sq[k]);

        return res;
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

    public static Point SnapToGrid(Point p)
    {
        return new Point(Math.Round(p.X / 10) * 10, Math.Round(p.Y / 10) * 10);
    }
}
