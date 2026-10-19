using System;
using Godot;
using Simulator.Vehicle;

namespace Simulator.UI;

/// <summary>Minimal readout of the physical state. Every number comes from the core's state/outputs.</summary>
public partial class DebugHud : Label
{
    [Export] public NodePath VehiclePath { get; set; } = new("");
    private VehicleController? _vehicle;

    public override void _Ready() => _vehicle = GetNodeOrNull<VehicleController>(VehiclePath);

    public override void _Process(double delta)
    {
        if (_vehicle?.Dynamics == null) return;
        var s = _vehicle.Dynamics.State;
        var o = _vehicle.Dynamics.Outputs;
        var u = _vehicle.LastInputs;
        Text = FormattableString.Invariant($"""
            {s.V * 3.6,6:F1} km/h    gear {(s.EngineBlown ? "X" : s.Gear.ToString())}{(_vehicle.AutoShift ? " (auto)" : "")}    {s.EngineRpm,5:F0} rpm{(o.RevLimiter ? "  LIMITER" : "")}
            ax {o.Ax,6:F2} m/s²   ay {o.Ay,6:F2} m/s² [E1]   r {o.YawRate,6:F3} rad/s [E1]   β {o.Beta,6:F3} rad [E1]
            F_trac {o.FTractiveEff,6:F0} N   F_x,max {o.FxMax,6:F0} N   grip use {o.GripUsage,4:F2}   slip {o.SlipIndicator,4:F2}{(o.Understeer ? "  UNDERSTEER" : "")}
            thr {u.Throttle:F2}  brk {u.Brake:F2}  δ {u.SteerAngle:F3} rad   t {s.Time:F2} s   d {s.Distance:F0} m{(_vehicle.Paused ? "   PAUSED" : "")}
            {(s.EngineBlown ? "ENGINE DESTROYED BY MONEY SHIFT (> 7200 rpm) — press R" : "")}
            W/S throttle/brake · A/D steer · E/Q shift · T auto-shift · R reset · P pause · C camera · H lights
            """);
    }
}
