using VehicleDynamics.Core.Collision;
using VehicleDynamics.Core.Simulation;
using Dyn = VehicleDynamics.Core.Simulation.VehicleDynamics;

namespace VehicleDynamics.Tests;

public class LoopHookTests
{
    [Fact]
    public void BeforeStep_RunsOncePerStep_AndItsSurfaceReachesTheModel()
    {
        var p = new VehicleParameters();
        var loop = new SimulationLoop(new Dyn(p, VehicleState.AtRest()));
        int calls = 0;
        loop.BeforeStep = (_, u) => { calls++; u.Surface = Surface.Grass; return u; };
        var levels = VehicleInputs.FromNormalized(1, 0, 0, p);
        int steps = loop.Advance(0.1, levels);
        Assert.Equal(steps, calls);

        // Grass grip factor scales F_x,max, so the reported limit must be the grass one.
        double expected = TractionLimit.FxMax(p, p.Mu * p.GripFactor(Surface.Grass));
        Assert.Equal(expected, loop.Vehicle.Outputs.FxMax, 9);
    }

    [Fact]
    public void ApplyCollision_WritesPoseAndSpeed_OnlyWhenImpulseApplied()
    {
        var d = new Dyn(new VehicleParameters(), VehicleState.AtRest());
        var s = d.State; s.V = 20; d.State = s;

        d.ApplyCollision(new CollisionResult(false, 1, 2, 0.5, 5, 0, 0));
        Assert.Equal((1.0, 2.0), (d.State.X, d.State.Y));
        Assert.Equal(20, d.State.V);
        Assert.Equal(0, d.State.Psi);

        d.ApplyCollision(new CollisionResult(true, 3, 4, 0.5, 12, 8, -8));
        Assert.Equal((3.0, 4.0, 0.5, 12.0), (d.State.X, d.State.Y, d.State.Psi, d.State.V));
    }

    [Fact]
    public void ApplyCollision_RejectsSpeedGainAndNonFiniteValues()
    {
        var d = new Dyn(new VehicleParameters(), VehicleState.AtRest());
        var s = d.State; s.V = 10; d.State = s;
        Assert.Throws<ArgumentException>(() => d.ApplyCollision(new CollisionResult(true, 0, 0, 0, 11, 1, -1)));
        Assert.Throws<ArgumentException>(() => d.ApplyCollision(new CollisionResult(true, double.NaN, 0, 0, 5, 1, -1)));
        Assert.Equal(10, d.State.V);
    }
}
