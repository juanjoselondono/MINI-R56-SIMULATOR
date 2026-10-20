namespace VehicleDynamics.Core.Track;

public static class TrackLibrary
{
    /// <summary>
    /// model.m §11 oval: two straights of L_recta (400 m) and two semicircles of R_curva (80 m), CCW, starting at the
    /// beginning of straight 1 at the origin heading +X. Runoff 10 m.
    /// </summary>
    public static TrackGeometry Oval(double lRecta = 400, double rCurva = 80, double runoff = 10)
    {
        var segs = new[]
        {
            TrackSegment.Straight(lRecta, "Recta 1"),
            TrackSegment.Arc(rCurva, Math.PI, "Curva 1"),
            TrackSegment.Straight(lRecta, "Recta 2"),
            TrackSegment.Arc(rCurva, Math.PI, "Curva 2"),
        };
        var t = new TrackGeometry("Oval", new Pose2(0, 0, 0), segs, closed: true, new CrossSection(Runoff: runoff));
        AssertClosed(t);
        return t;
    }

    /// <summary>
    /// Road circuit (gameplay design, not from MATLAB): main straight, R120/R150 sweepers, left-right chicane (R40),
    /// back straight, R28 hairpin, R120 right sweeper, connecting straight and an R80 corner onto the main straight.
    /// Turning angles sum to exactly 2 pi; the back straight and the connecting straight lengths are solved from the
    /// 2x2 linear closure condition, so the end pose equals the start pose up to round-off.
    /// </summary>
    public static TrackGeometry Circuit()
    {
        const double h = Math.PI / 2;
        var template = new List<TrackSegment>
        {
            TrackSegment.Straight(600, "Main straight", 12, 12),
            TrackSegment.Arc(120, h, "T1 sweeper", 15, 15),
            TrackSegment.Straight(200, "Short straight"),
            TrackSegment.Arc(150, h, "T2 sweeper", 15, 15),
            TrackSegment.Straight(150, "Approach"),
            TrackSegment.Arc(40, Math.PI / 6, "Chicane left", 8, 8),
            TrackSegment.Straight(15, "Chicane link", 8, 8),
            TrackSegment.Arc(40, -Math.PI / 6, "Chicane right", 8, 8),
            TrackSegment.Straight(0, "Back straight", 10, 10),     // solved
            TrackSegment.Arc(28, Math.PI, "Hairpin", 6, 6),
            TrackSegment.Arc(120, -h, "T6 sweeper", 15, 15),
            TrackSegment.Straight(0, "Link straight", 10, 10),     // solved
            TrackSegment.Arc(80, h, "Final corner", 10, 10),
        };
        const int ia = 8, ib = 11;
        var start = new Pose2(0, 0, 0);
        var probe = new TrackGeometry("probe", start, template, closed: false);
        double ha = probe.Segments[ia].Start.Psi, hb = probe.Segments[ib].Start.Psi;
        double rx = start.X - probe.EndPose.X, ry = start.Y - probe.EndPose.Y;
        // [cos ha  cos hb] [La]   [rx]
        // [sin ha  sin hb] [Lb] = [ry]
        double a11 = Math.Cos(ha), a12 = Math.Cos(hb), a21 = Math.Sin(ha), a22 = Math.Sin(hb);
        double det = a11 * a22 - a12 * a21;
        if (Math.Abs(det) < 1e-9) throw new InvalidOperationException("Closure straights are parallel.");
        double la = (rx * a22 - a12 * ry) / det;
        double lb = (a11 * ry - a21 * rx) / det;
        if (la < 50 || lb < 50) throw new InvalidOperationException($"Circuit closure needs negative/short straights ({la}, {lb}).");
        template[ia] = template[ia].WithLength(la);
        template[ib] = template[ib].WithLength(lb);
        var t = new TrackGeometry("Circuit", start, template, closed: true, new CrossSection(Runoff: 10));
        AssertClosed(t);
        return t;
    }

    private static void AssertClosed(TrackGeometry t)
    {
        var (p, hd) = t.ClosureError();
        if (p > 1e-6 || hd > 1e-9) throw new InvalidOperationException($"Track '{t.Name}' does not close: {p} m, {hd} rad.");
    }
}
