using System;
using System.Collections.Generic;
using System.Linq;
using Avalonia;
using System.Runtime.InteropServices;
using System.Numerics;

namespace MinimalRouter.Routing;

public class Router
{
    private readonly List<Obstacle> _obstacles = new List<Obstacle>();
    private readonly List<Trace> _traces = new List<Trace>();
    public double Clearance { get; set; } = 10;

    // Bias factor <1 to prefer diagonal edges (45°)
    private const double DiagonalBias = 1.0; // No bias needed if we use turn penalty
    private const double TurnPenalty = 10.0; // Penalty for changing direction
    private const double DiagonalTolerance = 2.0; // pixels tolerance for near-45°

    // Limit search area around start/end to reduce candidate set and CPU usage
    private const double MaxSearchRange = 300.0; // pixels radius around start/end center

    public IReadOnlyList<Obstacle> Obstacles => _obstacles;
    public IReadOnlyList<Trace> Traces => _traces;

    public void AddObstacle(Obstacle o)
    {
        _obstacles.Add(o);
    }

    public void AddTrace(Trace t)
    {
        _traces.Add(t);
    }

    public void ClearTraces()
    {
        _traces.Clear();
    }

    public void ClearObstacles()
    {
        _obstacles.Clear();
    }

    // 判断线段是否与任意障碍物碰撞
    private bool Collides(Segment seg, double currentWidth, HashSet<Trace>? ignoredTraces = null)
    {
        return CollisionDetector.IsColliding(seg, currentWidth, _obstacles, _traces, Clearance, ignoredTraces);
    }

    // Heuristic for A*: Euclidean distance (admissible)
    private static double Heuristic(Point a, Point b)
    {
        return Distance(a, b);
    }

    public List<Point> Route(Point start, Point end, double currentWidth)
    {
        start = SnapToGrid(start);
        end = SnapToGrid(end);

        // Identify traces connected to start
        var connectedTraces = new HashSet<Trace>();
        foreach (var t in _traces)
        {
            foreach (var s in t.Segments)
            {
                // Check if start is one of the endpoints
                if (ApproximatelyEqual(s.A, start) || ApproximatelyEqual(s.B, start))
                {
                    connectedTraces.Add(t);
                    break;
                }
                // Also check if start is ON the segment (T-junction start)
                if (CollisionDetector.DistancePointSegment(start, s.A, s.B) < 1.0) // Tolerance
                {
                    connectedTraces.Add(t);
                    break;
                }
            }
        }

        // If start is inside obstacle, we might be stuck, but let's try to route anyway
        // or return empty if strictly invalid. For now, proceed.

        var openSet = new PriorityQueue<Point, double>();
        var cameFrom = new Dictionary<Point, Point>();
        var gScore = new Dictionary<Point, double>();

        gScore[start] = 0;
        openSet.Enqueue(start, Heuristic(start, end));

        // To prevent exploring too far
        var searchRect = new Rect(
            Math.Min(start.X, end.X) - MaxSearchRange,
            Math.Min(start.Y, end.Y) - MaxSearchRange,
            Math.Abs(start.X - end.X) + MaxSearchRange * 2,
            Math.Abs(start.Y - end.Y) + MaxSearchRange * 2
        );

        int steps = 0;
        int maxSteps = 20000; // Safety break

        while (openSet.Count > 0)
        {
            if (steps++ > maxSteps) break;

            var current = openSet.Dequeue();
            if (ApproximatelyEqual(current, end))
            {
                var path = ReconstructPath(cameFrom, current);
                // Ensure end is exact
                if (!ApproximatelyEqual(path.Last(), end)) path.Add(end);
                return Simplify(path);
            }

            foreach (var neighbor in GetGridNeighbors(current))
            {
                if (!searchRect.Contains(neighbor)) continue;

                // Collision check for the EDGE
                var seg = new Segment(current, neighbor);
                if (Collides(seg, currentWidth, connectedTraces)) continue;

                double dist = Distance(current, neighbor);

                // Calculate turn penalty
                double penalty = 0;
                if (cameFrom.TryGetValue(current, out var prev))
                {
                    var prevDir = new Point(current.X - prev.X, current.Y - prev.Y);
                    var newDir = new Point(neighbor.X - current.X, neighbor.Y - current.Y);

                    // Cross product to check for direction change
                    double cross = prevDir.X * newDir.Y - prevDir.Y * newDir.X;
                    if (Math.Abs(cross) > 1e-9)
                    {
                        penalty = TurnPenalty;
                    }
                }

                double tentativeG = gScore[current] + dist + penalty;

                if (!gScore.ContainsKey(neighbor) || tentativeG < gScore[neighbor])
                {
                    cameFrom[neighbor] = current;
                    gScore[neighbor] = tentativeG;
                    double h = Heuristic(neighbor, end);
                    openSet.Enqueue(neighbor, tentativeG + h);
                }
            }
        }

        return new List<Point>();
    }

    private List<Point> ReconstructPath(Dictionary<Point, Point> cameFrom, Point current)
    {
        var path = new List<Point> { current };
        while (cameFrom.ContainsKey(current))
        {
            current = cameFrom[current];
            path.Add(current);
        }
        path.Reverse();
        return path;
    }

    private IEnumerable<Point> GetGridNeighbors(Point p)
    {
        double step = 10;
        // 8 directions
        yield return new Point(p.X + step, p.Y);
        yield return new Point(p.X - step, p.Y);
        yield return new Point(p.X, p.Y + step);
        yield return new Point(p.X, p.Y - step);

        yield return new Point(p.X + step, p.Y + step);
        yield return new Point(p.X + step, p.Y - step);
        yield return new Point(p.X - step, p.Y + step);
        yield return new Point(p.X - step, p.Y - step);
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
    private List<Point> ApplyChamfers(List<Point> path, double currentWidth)
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

            if (Collides(s1, currentWidth) || Collides(s2, currentWidth) || Collides(s3, currentWidth))
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
    private static void DistanceBatch(Point a, ReadOnlySpan<Point> pts, Span<double> outDist)
    {
        int n = pts.Length;
        if (outDist.Length < n) throw new ArgumentException("outDist length must be >= pts length", nameof(outDist));
        if (n == 0) return;

        // For small n, scalar is cheaper
        if (n < 8 || !System.Numerics.Vector.IsHardwareAccelerated)
        {
            for (int i = 0; i < n; i++)
            {
                var dx = a.X - pts[i].X;
                var dy = a.Y - pts[i].Y;
                outDist[i] = Math.Sqrt(dx * dx + dy * dy);
            }
            return;
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

        // sqrt pass into outDist
        for (int k = 0; k < n; k++)
            outDist[k] = Math.Sqrt(sq[k]);
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

    public static Point SnapToGrid(Point p)
    {
        return new Point(Math.Round(p.X / 10) * 10, Math.Round(p.Y / 10) * 10);
    }
}
