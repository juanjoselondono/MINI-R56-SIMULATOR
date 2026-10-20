using VehicleDynamics.Core.Simulation;
using Xunit.Abstractions;
using Dyn = VehicleDynamics.Core.Simulation.VehicleDynamics;

namespace VehicleDynamics.Tests;

public class InfrastructureTests
{
    private readonly ITestOutputHelper _out;
    public InfrastructureTests(ITestOutputHelper output) => _out = output;

    // ---------- Frame conversion ----------

    [Fact]
    public void Frame_StraightAhead_MapsToGodot()
    {
        var g = FrameConversion.ToGodotPosition(10, 5, 0.3);
        Assert.Equal(new Vec3(10, 0.3, -5), g);
        var back = FrameConversion.FromGodotPosition(g);
        Assert.Equal((10.0, 5.0), back);
        var f = FrameConversion.GodotForward(0);
        Assert.Equal(1, f.X, 15);
        Assert.Equal(0, f.Z, 15);
        var fNorth = FrameConversion.GodotForward(Math.PI / 2);
        Assert.Equal(0, fNorth.X, 15);
        Assert.Equal(-1, fNorth.Z, 15); // model north = Godot -Z
        Assert.Equal(-Math.PI / 2, FrameConversion.ToGodotYaw(0), 15);
        Assert.Equal(1.234, FrameConversion.FromGodotYaw(FrameConversion.ToGodotYaw(1.234)), 15);
    }

    [Theory]
    [InlineData(0.1)]
    [InlineData(-0.1)]
    public void Frame_SteerSignGivesYawSign(double delta)
    {
        var d = new Dyn(new VehicleParameters());
        var s = VehicleState.AtRest();
        s.V = 8;
        d.State = s;
        double ry0 = FrameConversion.ToGodotYaw(d.State.Psi);
        for (int i = 0; i < 100; i++) d.Step(new VehicleInputs { Throttle = 0.2, SteerAngle = delta }, 0.005);
        Assert.Equal(Math.Sign(delta), Math.Sign(d.Outputs.YawRate));
        Assert.Equal(Math.Sign(delta), Math.Sign(d.State.Psi));
        double ry1 = FrameConversion.ToGodotYaw(d.State.Psi);
        Assert.Equal(Math.Sign(delta), Math.Sign(ry1 - ry0)); // positive Godot Rotation.Y = turn left
        var gp = FrameConversion.ToGodotPosition(d.State.X, d.State.Y);
        Assert.True(gp.X > 0);                                 // moved forward (+X east)
        Assert.Equal(-Math.Sign(delta), Math.Sign(gp.Z));      // left turn -> north -> Godot -Z
        var fwd = FrameConversion.GodotForward(d.State.Psi);
        Assert.Equal(Math.Cos(d.State.Psi), fwd.X, 12);
        Assert.Equal(-Math.Sin(d.State.Psi), fwd.Z, 12);
    }

    // ---------- Clock & loop ----------

    [Fact]
    public void Clock_StepCountIndependentOfFramePattern()
    {
        var rng = new Random(42);
        var patterns = new List<double[]>
        {
            Enumerable.Repeat(0.005, 400).ToArray(),
            Enumerable.Repeat(1.0 / 60, 120).ToArray(),
            Enumerable.Repeat(1.0 / 144, 288).ToArray(),
            Enumerable.Repeat(0.002, 1000).ToArray(),
        };
        // random pattern summing to 2 s
        var rnd = new List<double>();
        double rem = 2.0;
        while (rem > 1e-12) { double f = Math.Min(rem, 0.001 + rng.NextDouble() * 0.03); rnd.Add(f); rem -= f; }
        patterns.Add(rnd.ToArray());
        foreach (var pat in patterns)
        {
            var c = new SimulationClock(0.005);
            long steps = 0;
            foreach (var f in pat) steps += c.Advance(f);
            Assert.Equal(400, steps);
            Assert.InRange(c.Alpha, 0, 1);
        }
    }

    [Fact]
    public void Clock_ClampsLongFramesAndPauses()
    {
        var c = new SimulationClock(0.005, 0.25);
        Assert.Equal(50, c.Advance(1.0));
        c.Paused = true;
        Assert.Equal(0, c.Advance(0.1));
        c.Paused = false;
        Assert.Equal(1, c.Advance(0.0075));
        Assert.Equal(0.5, c.Alpha, 6);
    }

    [Fact]
    public void Loop_InputsHeldAcrossSubsteps_ShiftEdgeConsumedOnce()
    {
        var d = new Dyn(new VehicleParameters());
        var loop = new SimulationLoop(d);
        var seen = new List<VehicleInputs>();
        loop.AfterStep = (_, u) => seen.Add(u);

        int n = loop.Advance(0.02, new VehicleInputs { Throttle = 0.7, ShiftUp = true });
        Assert.Equal(4, n);
        Assert.All(seen, u => Assert.Equal(0.7, u.Throttle));
        Assert.Single(seen, u => u.ShiftUp);
        Assert.True(seen[0].ShiftUp);
        Assert.Equal(2, d.State.Gear);

        // An edge arriving on a frame that runs no step stays pending and is consumed by the next step only.
        seen.Clear();
        Assert.Equal(0, loop.Advance(0.001, new VehicleInputs { Throttle = 0.7, ShiftUp = true }));
        Assert.Equal(2, d.State.Gear);
        Assert.Equal(3, loop.Advance(0.016, new VehicleInputs { Throttle = 0.7 }));
        Assert.Equal(3, d.State.Gear);
        Assert.Single(seen, u => u.ShiftUp);
    }

    [Fact]
    public void InputShaper_RampsPerSimulatedSecond()
    {
        var a = new InputShaper();
        for (int i = 0; i < 20; i++) a.Update(1, 0, 1, 0.005);   // 0.1 s
        var b = new InputShaper();
        for (int i = 0; i < 5; i++) b.Update(1, 0, 1, 0.02);     // 0.1 s
        Assert.Equal(a.Throttle, b.Throttle, 12);
        Assert.Equal(0.5, a.Throttle, 12);
        Assert.Equal(0.25, a.Steer, 12);
    }

    // ---------- Recorder ----------

    [Fact]
    public void Recorder_SamplesAtFixedSimInterval()
    {
        var d = new Dyn(new VehicleParameters());
        var rec = new SimulationRecorder(0.005, 0.02);
        Assert.Equal(4, rec.StepsPerSample);
        var u = new VehicleInputs { Throttle = 1, AutoShift = true };
        rec.RecordInitial(d.State, d.Outputs, u);
        for (int i = 0; i < 200; i++)
        {
            d.Step(u, 0.005);
            if (i == 5) rec.AddEvent("test_event");
            rec.OnStep(d.State, d.Outputs, u);
        }
        Assert.Equal(51, rec.Run.Samples.Count);
        for (int k = 0; k < rec.Run.Samples.Count; k++) Assert.Equal(0.02 * k, rec.Run.Samples[k].T, 9);
        Assert.Equal("test_event", rec.Run.Samples[2].Event); // step 6 -> next sample at step 8
    }

    [Fact]
    public void Recorder_CsvHeaderAndColumns()
    {
        var d = new Dyn(new VehicleParameters());
        var rec = new SimulationRecorder(0.005, 0.02, "unit");
        rec.AddMetadata("scenario", "S00");
        rec.AddParameters(d.Parameters);
        rec.RecordInitial(d.State, d.Outputs, default);
        var sw = new StringWriter { NewLine = "\n" };
        SimulationRecorder.WriteCsv(sw, rec.Run);
        var lines = sw.ToString().Split('\n');
        Assert.Equal("# run_id=unit", lines[0]);
        Assert.Contains("# scenario=S00", lines);
        Assert.Contains("# Mass=1420", lines);
        Assert.Contains("# MaxSteerAngle=0.3", lines);
        var header = lines.First(l => !l.StartsWith('#'));
        Assert.Equal("t_s,x_m,y_m,psi_rad,v_mps,v_kmh,vx_mps,vy_mps,yaw_rate_rps,beta_rad,ax_mps2,ay_mps2,gear,engine_rpm," +
                     "throttle,brake,steer_rad,f_tractive_n,f_tractive_eff_n,f_x_max_n,f_drag_n,f_rolling_n,f_grade_n,f_brake_n," +
                     "grip_usage,slip_indicator,track_s_m,track_lateral_m,surface,lap,lap_time_s,collision_severity_mps,event", header);
        var row = lines[Array.IndexOf(lines, header) + 1].Split(',');
        Assert.Equal(33, row.Length);
    }

    [Fact]
    public void RunComparison_IdenticalRunsHaveZeroError()
    {
        var a = new RecordedRun { Name = "a" };
        var b = new RecordedRun { Name = "b" };
        for (int i = 0; i < 10; i++)
        {
            a.Samples.Add(new TelemetrySample { T = i * 0.1, V = i });
            b.Samples.Add(new TelemetrySample { T = i * 0.1, V = i + (i == 5 ? 0.5 : 0) });
        }
        var r = RunComparison.Compare(a, b, s => s.V);
        Assert.Equal(10, r.Count);
        Assert.Equal(0.5, r.MaxAbs, 12);
    }

    // ---------- Config ----------

    [Fact]
    public void Config_RoundTrip()
    {
        var p = new VehicleParameters { Mass = 1500, Mu = 1.05, MaxSteerAngle = 0.25, SlopeValue = 3, MechanicalDamage = false };
        p.SurfaceGripFactor[2] = 0.5;
        var json = ConfigSerializer.ToJson(p);
        var q = ConfigSerializer.FromJson(json);
        Assert.Equal(ConfigSerializer.ToJson(q), json);
        Assert.Equal(1500, q.Mass);
        Assert.Equal(0.5, q.SurfaceGripFactor[2]);
        Assert.False(q.MechanicalDamage);
        Assert.DoesNotContain("WheelRadius", json);

        string path = Path.Combine(Path.GetTempPath(), $"vd_cfg_{Guid.NewGuid():N}.json");
        try
        {
            ConfigSerializer.Save(p, path);
            Assert.Equal(json, ConfigSerializer.ToJson(ConfigSerializer.Load(path)));
        }
        finally { File.Delete(path); }
    }

    [Theory]
    [InlineData("{\"Mass\": -1}")]
    [InlineData("{\"Mass\": 100000}")]
    [InlineData("{\"Mu\": 0}")]
    [InlineData("{\"Dt\": 0.5}")]
    [InlineData("{\"GearRatios\": [1, 2, 3, 4, 5, 6]}")]
    [InlineData("{\"MassFactors\": [0.1]}")]
    [InlineData("{\"CgToRear\": 3.0}")]
    [InlineData("{\"RpmRaw\": [1000, 900, 2000], \"TorqueRaw\": [1, 2, 3]}")]
    [InlineData("{\"NotAParameter\": 1}")]
    [InlineData("{\"Mass\": \"heavy\"}")]
    [InlineData("{\"Mass\": NaN}")]
    [InlineData("not json")]
    [InlineData("{\"MaxSteerAngle\": 2}")]
    [InlineData("{\"CollisionRestitution\": 1.5}")]
    public void Config_RejectsBadValues(string json)
    {
        Assert.Throws<ArgumentException>(() => ConfigSerializer.FromJson(json));
    }

    [Fact]
    public void Config_PartialJsonKeepsDefaults()
    {
        var p = ConfigSerializer.FromJson("{\"Mass\": 1800}");
        Assert.Equal(1800, p.Mass);
        Assert.Equal(0.378, p.Cd);
    }
}
