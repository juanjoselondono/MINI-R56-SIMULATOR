namespace VehicleDynamics.Core.Track;

/// <summary>
/// A closed (or open) reference line for track-relative quantities: lap timing, track position, lateral offset,
/// scripted driving. Implemented by the analytic <see cref="TrackGeometry"/> and by <see cref="PolylineTrack"/>
/// (measured racing lines, e.g. Assetto Corsa's fast_lane.ai). Model frame: X east, Y north, lateral + = left.
/// </summary>
public interface ITrackLine
{
    string Name { get; }
    double Length { get; }
    bool IsClosed { get; }
    double NormalizeS(double s);
    Pose2 Pose(double s);
    double Curvature(double s);
    (double X, double Y) PointAt(double s, double lateral);
    TrackProjection Project(double x, double y);
    /// <summary>Track limits (road + kerbs) at s as distances from the line: left and right, both positive.</summary>
    (double Left, double Right) Limits(double s);
}
