using System;

namespace Simulator.Audio;

/// <summary>Per-buffer control values, all taken from the physics core's state/outputs (or the driver input).</summary>
public struct SynthControl
{
    public double Rpm;
    public double Throttle;
    public double SpeedMps;
    public bool Coasting;
    public bool RevLimiter;
    public bool EngineOff;
    /// <summary>Core slip indicator; squeal only above 1 (traction limit or lateral grip saturated).</summary>
    public double SlipIndicator;
    /// <summary>Braking at the model's friction-limited maximum force (brake input ≥ 0.95 in the Brake branch).</summary>
    public bool BrakingAtLimit;
}

/// <summary>
/// Procedural engine/tyre/wind synthesis, allocation-free in the sample loop. Mapping (documented in
/// docs/architecture.md): fundamental f0 = rpm / 60 · 2 (inline-4, four-stroke: two firing events per revolution) with
/// 8 harmonics whose upper-harmonic weight grows with throttle (load); intake noise ∝ throttle; overrun burble when
/// coasting; 15 Hz on/off gate while the rev limiter cuts fuel; tyre squeal (band-pass noise ~1.1 kHz) only when the
/// slip indicator exceeds 1; brake scrub when braking at the limit; wind noise ∝ (V/50)²; impact one-shots.
/// </summary>
public sealed class EngineSynth
{
    public readonly int SampleRate;
    private double _phase;
    private double _rpmS, _throttleS, _engineGain, _squealS, _windS, _scrubS, _limiterPhase;
    private double _shiftDip;
    private uint _rng = 0x12345678;
    private Biquad _squealBp, _scrubBp;
    private double _windLp, _noiseLp, _burbleTimer;
    private double _impactAmp, _impactPhase, _explosion;
    private double _explosionLp;

    public double MasterGain { get; set; } = 0.6;

    public EngineSynth(int sampleRate)
    {
        SampleRate = sampleRate;
        _squealBp = Biquad.BandPass(sampleRate, 1100, 3.0);
        _scrubBp = Biquad.BandPass(sampleRate, 550, 1.5);
        _rpmS = 1000;
    }

    public static double FundamentalHz(double rpm) => rpm / 60.0 * 2.0;

    public void TriggerShift() => _shiftDip = 1.0;
    public void TriggerImpact(double severityMps) => _impactAmp = Math.Max(_impactAmp, Math.Clamp(severityMps / 12.0, 0.15, 1.2));
    public void TriggerExplosion() => _explosion = 1.0;

    /// <summary>Fills <paramref name="left"/>/<paramref name="right"/> (same length) with stereo samples in [-1, 1].</summary>
    public void Render(in SynthControl c, Span<float> left, Span<float> right)
    {
        double dt = 1.0 / SampleRate;
        double kFast = 1 - Math.Exp(-dt / 0.02);   // 20 ms smoothing for pitch / levels
        double kSlow = 1 - Math.Exp(-dt / 0.08);
        double targetGain = c.EngineOff ? 0 : 1;

        for (int i = 0; i < left.Length; i++)
        {
            _rpmS += (c.Rpm - _rpmS) * kFast;
            _throttleS += (c.Throttle - _throttleS) * kFast;
            _engineGain += (targetGain - _engineGain) * kFast;
            double squealTarget = c.SlipIndicator > 1 && c.SpeedMps > 3 ? Math.Min(1, (c.SlipIndicator - 1) * 2) : 0;
            _squealS += (squealTarget - _squealS) * kSlow;
            _scrubS += ((c.BrakingAtLimit && c.SpeedMps > 5 ? Math.Min(1, c.SpeedMps / 30) : 0) - _scrubS) * kSlow;
            _windS += (Math.Min(1.5, Math.Pow(c.SpeedMps / 50, 2)) - _windS) * kSlow;

            double f0 = FundamentalHz(_rpmS);
            _phase += f0 * dt;
            if (_phase >= 1) _phase -= Math.Floor(_phase);
            double ph = 2 * Math.PI * _phase;

            // Harmonic stack: brighter under load.
            double load = 0.25 + 0.75 * _throttleS;
            double eng = 0;
            for (int k = 1; k <= 8; k++)
            {
                double amp = k == 1 ? 1.0 : Math.Pow(0.55 + 0.3 * load, k - 1) / k * (k % 2 == 0 ? 1.3 : 0.8);
                eng += amp * Math.Sin(k * ph);
            }
            double noise = NextNoise();
            _noiseLp += (noise - _noiseLp) * 0.15;
            // Intake/exhaust noise pulsed at the firing frequency.
            double pulse = 0.5 + 0.5 * Math.Sin(ph);
            eng += _noiseLp * pulse * (0.4 + 1.2 * _throttleS);
            eng = Math.Tanh(eng * (0.6 + 0.8 * load)) * (0.35 + 0.45 * _throttleS + 0.15 * Math.Min(1, _rpmS / 6500));

            if (c.Coasting)
            {
                _burbleTimer -= dt;
                if (_burbleTimer <= 0 && _rpmS > 2500) { _burbleTimer = 0.02 + 0.08 * Next01(); eng += 0.6 * NextNoise(); }
            }
            if (c.RevLimiter)
            {
                _limiterPhase += 15 * dt;
                if (_limiterPhase % 1 > 0.5) eng *= 0.15;
            }
            if (_shiftDip > 0) { eng *= 1 - 0.8 * _shiftDip; _shiftDip = Math.Max(0, _shiftDip - dt / 0.08); }
            eng *= _engineGain;

            double squeal = _squealBp.Process(noise) * 2.5 * _squealS;
            double scrub = _scrubBp.Process(noise) * 1.2 * _scrubS;
            _windLp += (noise - _windLp) * 0.03;
            double wind = _windLp * 1.2 * _windS;

            double impact = 0;
            if (_impactAmp > 1e-3)
            {
                _impactPhase += 55 * dt;
                impact = _impactAmp * (0.8 * Math.Sin(2 * Math.PI * _impactPhase) + 0.5 * noise);
                _impactAmp *= Math.Exp(-dt / 0.12);
            }
            double boom = 0;
            if (_explosion > 1e-3)
            {
                _explosionLp += (noise - _explosionLp) * (0.02 + 0.2 * _explosion);
                boom = _explosionLp * 3.0 * _explosion;
                _explosion *= Math.Exp(-dt / 0.5);
            }

            double mix = MasterGain * (0.55 * eng + 0.35 * squeal + 0.25 * scrub + 0.2 * wind + 0.6 * impact + 0.8 * boom);
            mix = Math.Clamp(mix, -1, 1);
            left[i] = (float)mix;
            right[i] = (float)mix;
        }
    }

    private double NextNoise() => Next01() * 2 - 1;

    private double Next01()
    {
        _rng ^= _rng << 13; _rng ^= _rng >> 17; _rng ^= _rng << 5;
        return _rng / (double)uint.MaxValue;
    }

    private struct Biquad
    {
        private double _b0, _b1, _b2, _a1, _a2, _x1, _x2, _y1, _y2;

        public static Biquad BandPass(double fs, double f, double q)
        {
            double w = 2 * Math.PI * f / fs, alpha = Math.Sin(w) / (2 * q), a0 = 1 + alpha;
            return new Biquad
            {
                _b0 = alpha / a0, _b1 = 0, _b2 = -alpha / a0,
                _a1 = -2 * Math.Cos(w) / a0, _a2 = (1 - alpha) / a0,
            };
        }

        public double Process(double x)
        {
            double y = _b0 * x + _b1 * _x1 + _b2 * _x2 - _a1 * _y1 - _a2 * _y2;
            _x2 = _x1; _x1 = x; _y2 = _y1; _y1 = y;
            return y;
        }
    }
}
