using System;
using System.Collections.Generic;
using System.IO;
using Simulator.Vehicle;
using VehicleDynamics.Core.Simulation;

namespace Simulator.Simulation;

/// <summary>A finished recording together with the exact parameter set it was produced with.</summary>
public sealed record RunEntry(RecordedRun Run, VehicleParameters Parameters, string Track, DateTime Started)
{
    public string? CsvPath { get; set; }
}

/// <summary>
/// Telemetry recording on top of the core <see cref="SimulationRecorder"/>: samples are taken by the physics loop at a
/// fixed simulation interval (default 0.02 s), never at render rate. Finished runs are kept for comparison and can be
/// exported as CSV (schema of DESIGN_CONTRACT §6) with the parameter JSON saved next to them.
/// </summary>
public sealed class TelemetryService
{
    public double SampleInterval { get; set; } = 0.02;
    public SimulationRecorder? Active { get; private set; }
    public bool Recording => Active != null;
    public List<RunEntry> Runs { get; } = new();
    /// <summary>Samples of the live run, or of the latest finished run when not recording.</summary>
    public IReadOnlyList<TelemetrySample> CurrentSamples
        => Active != null ? Active.Run.Samples : Runs.Count > 0 ? Runs[^1].Run.Samples : Array.Empty<TelemetrySample>();

    /// <summary>Rolling live buffer for the plots (same fixed sim interval as recordings; kept even when not recording).</summary>
    public int LiveCapacity { get; set; } = 1500;
    public LinkedList<TelemetrySample> Live { get; } = new();
    public TelemetrySample Latest { get; private set; }
    private long _liveStep;

    /// <summary>Called by the simulation after every physics step; keeps every k-th step (k = interval / dt).</summary>
    public void OnStep(in VehicleState s, in VehicleOutputs o, in VehicleInputs u, TrackSample track, double dt)
    {
        Latest = SimulationRecorder.MakeSample(s, o, u, track);
        long every = Math.Max(1, (long)Math.Round(SampleInterval / dt));
        if (_liveStep++ % every != 0) return;
        Live.AddLast(Latest);
        while (Live.Count > LiveCapacity) Live.RemoveFirst();
    }

    public void ClearLive()
    {
        Live.Clear();
        _liveStep = 0;
    }

    private VehicleParameters? _activeParams;
    private string _activeTrack = "";
    private DateTime _activeStart;
    private int _counter;

    public void Start(VehicleController vc, string track, string? name = null)
    {
        if (Recording) Stop(vc);
        _counter++;
        _activeStart = DateTime.Now;
        name ??= $"run{_counter:00}_{track}_{_activeStart:HHmmss}";
        var rec = new SimulationRecorder(vc.Parameters.Dt, SampleInterval, name);
        rec.AddMetadata("track", track);
        rec.AddMetadata("started_local", _activeStart.ToString("yyyy-MM-dd HH:mm:ss"));
        rec.AddMetadata("sample_interval_s", SampleInterval.ToString(System.Globalization.CultureInfo.InvariantCulture));
        rec.AddMetadata("model", "reference/matlab/model.m port + extensions E1-E3 (see docs/vehicle_model.md)");
        rec.AddParameters(vc.Parameters);
        var ctx = vc.TrackContext?.Invoke(vc.Dynamics);
        rec.RecordInitial(vc.Dynamics.State, vc.Dynamics.Outputs, vc.LastInputs, ctx);
        Active = rec;
        _activeParams = vc.Parameters.Clone();
        _activeTrack = track;
        vc.Recorder = rec;
    }

    public RunEntry? Stop(VehicleController vc)
    {
        if (Active == null) return null;
        vc.Recorder = null;
        var entry = new RunEntry(Active.Run, _activeParams!, _activeTrack, _activeStart);
        Runs.Add(entry);
        Active = null;
        return entry;
    }

    public void AddEvent(string evt) => Active?.AddEvent(evt);

    /// <summary>Writes the run CSV and its parameter JSON; returns the CSV path.</summary>
    public static string Export(RunEntry entry, string directory)
    {
        Directory.CreateDirectory(directory);
        string stem = Path.Combine(directory, $"{entry.Started:yyyyMMdd_HHmmss}_{Sanitize(entry.Run.Name)}");
        string csv = stem + ".csv";
        using (var w = new StreamWriter(csv, false, new System.Text.UTF8Encoding(false)) { NewLine = "\n" })
            SimulationRecorder.WriteCsv(w, entry.Run);
        ConfigSerializer.Save(entry.Parameters, stem + ".config.json");
        entry.CsvPath = csv;
        return csv;
    }

    private static string Sanitize(string s)
    {
        foreach (char c in Path.GetInvalidFileNameChars()) s = s.Replace(c, '_');
        return s.Replace(' ', '_');
    }
}
