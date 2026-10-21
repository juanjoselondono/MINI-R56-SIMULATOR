using System;
using System.Collections.Generic;
using System.Linq;
using Godot;
using Simulator.Dynamics;
using VehicleDynamics.Core.Simulation;
using VehicleDynamics.Core.Track;

namespace Simulator.Track;

/// <summary>
/// Driver aid (extension, not physics): the track's reference line drawn on the road as a ribbon, coloured by the
/// reference inputs of the core's quasi-steady reference lap (<see cref="ReferenceLap"/>): green = accelerate,
/// yellow = coast / transition / partial throttle, red = heavy braking. Colours blend over a few metres so the zone
/// boundaries read as brake and throttle points. Rebuilt whenever the track or the vehicle parameters change, so a
/// heavier car or lower μ moves the braking zones. Mesh tracks: the AC racing line (the ideal trajectory) on the
/// collision mesh with its grade; procedural tracks: the centreline at y = 0.
/// </summary>
public partial class RacingLine : MeshInstance3D
{
    public static readonly Color Accelerate = new(0.15f, 0.85f, 0.25f);
    public static readonly Color Coast = new(1.0f, 0.82f, 0.1f);
    public static readonly Color Brake = new(0.95f, 0.12f, 0.1f);

    [Export] public float Width { get; set; } = 0.55f;
    /// <summary>Height of the ribbon above the collision surface [m] (keeps it off the visual road mesh).</summary>
    [Export] public float Lift { get; set; } = 0.06f;

    public ReferencePoint[] Points { get; private set; } = Array.Empty<ReferencePoint>();
    /// <summary>Reference lap time of the profile [s] (quasi-steady estimate, not a validated lap time).</summary>
    public double LapTime { get; private set; }

    public RacingLine()
    {
        CastShadow = ShadowCastingSetting.Off;
        MaterialOverride = new StandardMaterial3D
        {
            ShadingMode = BaseMaterial3D.ShadingModeEnum.Unshaded,
            VertexColorUseAsAlbedo = true,
            Transparency = BaseMaterial3D.TransparencyEnum.Alpha,
            CullMode = BaseMaterial3D.CullModeEnum.Disabled,
            AlbedoColor = new Color(1, 1, 1, 0.8f),
        };
    }

    public void Build(ITrackLine line, VehicleParameters p, MeshTrack? mesh)
    {
        const double spacing = 1.0;
        int n = Math.Max(8, (int)Math.Ceiling(line.Length / spacing));
        double ds = line.Length / n;

        // Ground under the line (mesh tracks): walk along it so each point searches around the previous height.
        var h = new float[n];
        var up = new Vector3[n];
        if (mesh != null)
        {
            var pose0 = line.Pose(0);
            float hint = mesh.ProbeGroundHeight(pose0.X, pose0.Y) ?? 0;
            for (int i = 0; i < n; i++)
            {
                var q = line.Pose(i * ds);
                hint = mesh.GroundHeightNear(q.X, q.Y, hint) ?? hint;
                h[i] = hint;
            }
        }
        for (int i = 0; i < n; i++) up[i] = Vector3.Up;

        Func<double, double>? grade = null;
        if (mesh != null)
            grade = s =>
            {
                int i = (int)Math.Round(line.NormalizeS(s) / ds) % n;
                return Math.Atan2(h[(i + 2) % n] - h[(i - 2 + n) % n], 4 * ds);
            };
        double grip = mesh?.Friction.GetValueOrDefault("TARMAC", 1.0) ?? 1.0;
        Points = ReferenceLap.Compute(line, p, spacing: 2.0, grade: grade, gripFactor: grip);
        LapTime = ReferenceLap.LapTime(Points, line.Length);

        var colours = new Color[n];
        double dsRef = line.Length / Points.Length;
        for (int i = 0; i < n; i++)
        {
            // Blend the zone colours over ±3 m around each point.
            Color sum = new(0, 0, 0, 0);
            int count = 0;
            for (double o = -3; o <= 3; o += 1.5)
            {
                int k = (int)Math.Floor(line.NormalizeS(i * ds + o) / dsRef) % Points.Length;
                sum += ZoneColour(Points[k].Zone);
                count++;
            }
            colours[i] = sum / count;
        }

        var st = new SurfaceTool();
        st.Begin(Mesh.PrimitiveType.Triangles);
        for (int i = 0; i <= n; i++)
        {
            int k = i % n;
            var q = line.Pose(k * ds);
            var c = VehicleModelAdapter.ToGodotPosition(q.X, q.Y, h[k] + Lift);
            // Model-frame left normal (−sin ψ, cos ψ) → Godot (x, −y).
            var left = new Vector3(-(float)Math.Sin(q.Psi), 0, -(float)Math.Cos(q.Psi)) * (Width / 2);
            st.SetColor(colours[k]); st.SetNormal(up[k]); st.AddVertex(c + left);
            st.SetColor(colours[k]); st.SetNormal(up[k]); st.AddVertex(c - left);
        }
        for (int i = 0; i < n; i++)
        {
            int a = 2 * i;
            st.AddIndex(a); st.AddIndex(a + 2); st.AddIndex(a + 1);
            st.AddIndex(a + 1); st.AddIndex(a + 2); st.AddIndex(a + 3);
        }
        Mesh = st.Commit();
    }

    public static Color ZoneColour(DriveZone z) => z switch
    {
        DriveZone.Accelerate => Accelerate,
        DriveZone.Brake => Brake,
        _ => Coast,
    };

    /// <summary>Share of the line length per zone (accelerate, coast, brake).</summary>
    public (double Accelerate, double Coast, double Brake) ZoneShares()
    {
        if (Points.Length == 0) return (0, 0, 0);
        double n = Points.Length;
        return (Points.Count(x => x.Zone == DriveZone.Accelerate) / n, Points.Count(x => x.Zone == DriveZone.Coast) / n,
            Points.Count(x => x.Zone == DriveZone.Brake) / n);
    }
}
