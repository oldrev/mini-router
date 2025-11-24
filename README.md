# MinimalRouter

[中文](README.zh.md)

A minimal demo for exploring key algorithms used in PCB routing and layout tools.

![Screen shot](Assets/screenshot.png)

Project goals: study and demonstrate common techniques for PCB routing/layout, including path search (A*), collision detection, handling trace width/clearance, 90° chamfering of corners, and copper pour (difference/clip operations) around obstacles and traces.

This project is a learning and experimentation demo; it intentionally does not separate world coordinates from drawing coordinates to keep the implementation compact.



## Highlights

- Interactive UI (built with .NET Avalonia): place random polygonal obstacles, drag to create traces, and confirm multi-segment traces.
- A* grid search with 8-direction neighbours (includes diagonals) and optional turn penalty to reduce unnecessary corners.
- Collision detection: robust tests for arbitrary polygon obstacles and traces using bounding boxes + edge-to-edge distance checks.
- Trace width & clearance management: expands boundaries by half the trace width plus a global clearance when checking for collisions.
- Chamfering at 90° corners: attempts to replace right-angle corners with 45° chamfers to reduce sharp points and improve manufacturability.
- Copper pour (difference): uses Clipper2 to subtract obstacles and traces (with expansion) from the board polygon, showing allowable copper regions.
- Vectorized distance operations and other optimizations are used where appropriate to reduce cost for large arrays.

---

## Project structure (key files)

- `Program.cs`: Avalonia application entry point.
- `MainWindow.axaml` / `MainWindow.axaml.cs`: UI and interaction logic. Handles random obstacle generation, mouse events (draw/confirm), and copper pour demo.
- `Controls/DrawingCanvas.cs`: Canvas drawing support.
- `Routing/Router.cs`: Core router that implements A*, neighbor selection, simplification, and chamfer logic.
- `Routing/CollisionDetector.cs`: Collision detection for polygons, rectangles and distance-based checks between segments.
- `Routing/Obstacle.cs`: Obstacle polygon data structure.
- `Routing/Trace.cs`, `Routing/Segment.cs`: Trace/segment models.
- `Models/Point.cs`: A small simplified `Point` structure used in the demo.

---

## Design & algorithm details

1. Path search (Router)
   - A* search on a discretized grid (step = 10 pixels) using Euclidean distance as the heuristic. Each cell has 8 neighbors (4 orthogonal, 4 diagonal).
   - Turn penalty encourages straighter routes by increasing g-score when a path changes direction, which reduces unnecessary turns.
   - The search is bounded by a `MaxSearchRange` radius from the start/end points to avoid global blow-ups.
   - Post-processing: remove colinear points (Simplify) and attempt to replace 90° corners with chamfers (ApplyChamfers). A chamfer is only applied after it is confirmed to be collision-free.

2. Collision detection (CollisionDetector)
   - Each obstacle is first quickly cropped using an inflated bounding rectangle (inflated by clearance + width/2). If bounding rectangles overlap, a more expensive polygon-edge/edge distance test is performed.
   - For static traces, the effective minimum allowed distance is trace half-width + clearance — if the segment-to-segment distance is smaller than this threshold, it is considered a collision. Shared endpoints are allowed to connect.
   - Interior point testing uses ray casting; segment-segment shortest distance uses robust geometric algorithms adapted from GeomAlgorithms.

3. Copper pour
   - Uses the Clipper2 library. The board boundary acts as the Subject polygon. Obstacles and traces (after inflating by half trace width + clearance) are the Clip polygons. Performing a Difference yields the regions that can be filled with copper.

4. Performance-friendly measures
   - Where possible, batch distance computations use `System.Numerics.Vector` for SIMD acceleration.
   - Search space clipping and `maxSteps` help prevent very long or infinite searches.

---

## Quick start (Windows / PowerShell)

Prerequisites: .NET SDK installed (.NET 10) and Internet to restore NuGet packages.

Build and run:

```powershell
dotnet restore
dotnet build -c Debug
dotnet run --project MinimalRouter.csproj
```

The app will open a window. Typical actions:
- Left-click to start routing; move the mouse to preview candidate routes; click to confirm a segment. Double-click to finish the trace.
- Right-click to cancel the current trace.
- Use the `Width` slider to change trace width; use the `Clearance` slider to adjust the global clearance. Press the `Pour` button to perform a copper pour (board boundary minus obstacles and traces).

## Interaction notes

- The first click on the canvas sets the start point (green). Subsequent clicks add points to the route; double-click finishes and commits the trace. Right-click cancels or reverts.
- Confirmed traces are added to a static list used by the router for collision detection. The router allows connecting to trace endpoints.
- Obstacles are randomly generated polygons for demo purposes and to demonstrate collision avoidance and copper pour.

---

## License & contribution

This code is provided as a demo for educational/video purposes.

Public Domain (no restrictions).

Author: Li Wei (email: oldrev@gmail.com)