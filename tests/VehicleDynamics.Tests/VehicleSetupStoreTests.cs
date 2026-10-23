using VehicleDynamics.Core.Simulation;

namespace VehicleDynamics.Tests;

/// <summary>Per-vehicle setup files (VehicleSetupStore): each profile reads and writes only its own.</summary>
public class VehicleSetupStoreTests : IDisposable
{
    private readonly string _dir = Path.Combine(Path.GetTempPath(), "vd_setups_" + Guid.NewGuid().ToString("N"));

    public void Dispose()
    {
        if (Directory.Exists(_dir)) Directory.Delete(_dir, true);
    }

    [Fact]
    public void EachProfile_HasItsOwnSetupFile_AndDefaultsWithoutOne()
    {
        var store = new VehicleSetupStore(_dir);
        Assert.EndsWith("setup_mini.json", store.ActivePath(VehicleProfiles.MiniR56));
        Assert.EndsWith("setup_f1.json", store.ActivePath(VehicleProfiles.F1));
        Assert.Equal(ConfigSerializer.ToJson(VehicleParameters.F1Provisional()),
            ConfigSerializer.ToJson(store.LoadActive(VehicleProfiles.F1, out var none)));
        Assert.Null(none);

        var mini = VehicleParameters.MiniCooperR56();
        mini.Mass = 1234;
        var f1 = VehicleParameters.F1Provisional();
        f1.BrakeBiasFront = 0.6;
        mini.SaveSetup(_dir);
        f1.SaveSetup(_dir);
        Assert.Equal(1234, VehicleParameters.LoadSetup(_dir, VehicleProfiles.MiniR56).Mass);
        Assert.Equal(0.6, VehicleParameters.LoadSetup(_dir, VehicleProfiles.F1).BrakeBiasFront);

        // Changing one never touches the other.
        f1.Mass = 800;
        f1.SaveSetup(_dir);
        Assert.Equal(1234, VehicleParameters.LoadSetup(_dir, VehicleProfiles.MiniR56).Mass);
    }

    [Fact]
    public void ForeignSetups_AreRejected()
    {
        var store = new VehicleSetupStore(_dir);
        Directory.CreateDirectory(_dir);
        // A Mini setup copied over the F1's file: the F1 keeps its defaults and reports why.
        ConfigSerializer.Save(VehicleParameters.MiniCooperR56(), store.ActivePath(VehicleProfiles.F1));
        var p = store.LoadActive(VehicleProfiles.F1, out var err);
        Assert.Equal(VehicleProfiles.F1, p.Profile);
        Assert.Contains("Mini", err);
        string path = store.SaveNamed(VehicleParameters.MiniCooperR56(), "x");
        Assert.Throws<ArgumentException>(() => VehicleSetupStore.LoadFor(VehicleProfiles.F1, path));
        Assert.Equal(new[] { "x" }, store.ListNamed(VehicleProfiles.MiniR56));
        Assert.Empty(store.ListNamed(VehicleProfiles.F1));
    }

    [Fact]
    public void MissingKeys_TakeTheFilesOwnProfileDefaults()
    {
        var f1 = ConfigSerializer.FromJson("{\"Profile\": \"f1_2019\", \"Mass\": 760}");
        Assert.Equal(760, f1.Mass);
        Assert.Equal(VehicleParameters.F1Provisional().CorneringStiffnessRear, f1.CorneringStiffnessRear);
        Assert.Equal(DriveLayout.RearWheelDrive, f1.DriveLayout);
        // Without a profile: model.m's defaults, as before.
        Assert.Equal(new VehicleParameters().CorneringStiffnessRear, ConfigSerializer.FromJson("{\"Mass\": 1800}").CorneringStiffnessRear);
    }

    [Fact]
    public void LegacyFlatConfigs_MoveIntoTheirProfileFolder()
    {
        Directory.CreateDirectory(_dir);
        ConfigSerializer.Save(VehicleParameters.MiniCooperR56(), Path.Combine(_dir, "old_mini.json"));
        ConfigSerializer.Save(VehicleParameters.F1Provisional(), Path.Combine(_dir, "old_f1.json"));
        File.WriteAllText(Path.Combine(_dir, "broken.json"), "{");
        var store = new VehicleSetupStore(_dir);
        store.SaveActive(VehicleParameters.F1Provisional());
        Assert.Equal(2, store.MigrateLegacy());
        Assert.Equal(new[] { "old_mini" }, store.ListNamed(VehicleProfiles.MiniR56));
        Assert.Equal(new[] { "old_f1" }, store.ListNamed(VehicleProfiles.F1));
        Assert.True(File.Exists(Path.Combine(_dir, "broken.json")));
        Assert.True(File.Exists(store.ActivePath(VehicleProfiles.F1)));
    }
}
