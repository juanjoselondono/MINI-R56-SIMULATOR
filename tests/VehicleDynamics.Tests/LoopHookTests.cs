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
    public void ApplyCollision_WritesPoseHeadingAndSpeed()
    {
        var d = new Dyn(new VehicleParameters(), VehicleState.AtRest());
        var s = d.State; s.V = 20; d.State = s;
        d.ApplyCollision(new CollisionResult(true, 3, 4, 0.5, 12, 8, -8));
        Assert.Equal((3.0, 4.0, 0.5, 12.0), (d.State.X, d.State.Y, d.State.Psi, d.State.V));
    }

    [Fact]
    public void SustainedContact_RemovesApproachingNormalVelocityOnly()
    {
        // Wall normal +Y (wall below the car). Car heading 30 deg below the wall tangent, i.e. approaching.
        double a = -Math.PI / 6;
        var c = new CollisionInput(0, 0, a, 0, 20, 0, 1, 0.05);
        var r = SustainedContact.Resolve(c);
        Assert.True(r.SustainedContact);
        Assert.False(r.ImpulseApplied);
        Assert.Equal(20 * Math.Cos(a), r.V, 12);   // tangential part kept
        Assert.Equal(a, r.Psi);                     // no heading change
        Assert.Equal(0.05, r.Y, 12);               // pushed out

        var headOn = SustainedContact.Resolve(new CollisionInput(0, 0, -Math.PI / 2, 0, 5, 0, 1, 0.01));
        Assert.Equal(0, headOn.V, 9);

        var leaving = SustainedContact.Resolve(new CollisionInput(0, 0, Math.PI / 6, 0, 20, 0, 1, 0.01));
        Assert.False(leaving.SustainedContact);
        Assert.Equal(20, leaving.V);
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
