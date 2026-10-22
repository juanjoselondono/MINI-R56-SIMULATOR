using System;
using System.Linq;
using Godot;

namespace Simulator.Rendering;

public enum GraphicsPreset { Ultra, High, Performance }

/// <summary>
/// Forward+ quality presets for an RTX 3050-class GPU. The look (ACES, SSAO, SSIL, SSR, volumetric fog, glow, DoF,
/// motion blur) is authored in Main.tscn's WorldEnvironment and on the camera; a preset only changes resolution scaling,
/// anti-aliasing and the cost knobs of those effects:
///   Ultra        native resolution + TAA; SSAO/SSIL high (full-res SSAO); SSR 64 steps at full resolution.
///   High         FSR 2.2 Quality (render scale 1/1.5, its own temporal AA instead of TAA); SSAO high, SSIL medium
///                (half-res); SSR 48 steps at half resolution.
///   Performance  FSR 2.2 Balanced (1/1.7); SSAO medium, SSIL off; SSR 32 steps (half-res); lighter volumetric fog and blur.
/// The default was chosen from measured frame times on Monaco (docs/validation_report.md §17). F6 cycles presets,
/// `-- --gfx=ultra|high|performance` picks one at start.
/// </summary>
public static class GraphicsSettings
{
    public static GraphicsPreset Current { get; private set; } = DefaultPreset;
    public const GraphicsPreset DefaultPreset = GraphicsPreset.Ultra;

    public static GraphicsPreset FromCommandLine()
    {
        var arg = OS.GetCmdlineUserArgs().FirstOrDefault(a => a.StartsWith("--gfx="));
        return arg?["--gfx=".Length..].ToLowerInvariant() switch
        {
            "ultra" => GraphicsPreset.Ultra,
            "performance" => GraphicsPreset.Performance,
            "high" => GraphicsPreset.High,
            _ => Current,
        };
    }

    public static GraphicsPreset Next(GraphicsPreset p) => (GraphicsPreset)(((int)p + 1) % 3);

    public static string Describe(GraphicsPreset p) => p switch
    {
        GraphicsPreset.Ultra => "Ultra: native + TAA",
        GraphicsPreset.High => "High: FSR 2.2 Quality",
        _ => "Performance: FSR 2.2 Balanced",
    };

    public static void Apply(GraphicsPreset preset, Viewport viewport, Godot.Environment? env, MotionBlurEffect? blur)
    {
        Current = preset;
        bool fsr = preset != GraphicsPreset.Ultra;
        viewport.Msaa3D = Viewport.Msaa.Disabled;                     // temporal AA replaces MSAA
        viewport.ScreenSpaceAA = Viewport.ScreenSpaceAAEnum.Disabled;
        viewport.Scaling3DMode = fsr ? Viewport.Scaling3DModeEnum.Fsr2 : Viewport.Scaling3DModeEnum.Bilinear;
        viewport.Scaling3DScale = preset switch { GraphicsPreset.High => 1f / 1.5f, GraphicsPreset.Performance => 1f / 1.7f, _ => 1f };
        viewport.UseTaa = !fsr;                                        // FSR 2.2 does its own temporal AA
        viewport.FsrSharpness = 0.2f;

        RenderingServer.EnvironmentSetSsaoQuality(
            preset == GraphicsPreset.Performance ? RenderingServer.EnvironmentSsaoQuality.Medium : RenderingServer.EnvironmentSsaoQuality.High,
            preset != GraphicsPreset.Ultra, 0.5f, 2, 50f, 300f);
        RenderingServer.EnvironmentSetSsilQuality(
            preset == GraphicsPreset.Ultra ? RenderingServer.EnvironmentSsilQuality.High : RenderingServer.EnvironmentSsilQuality.Medium,
            true, 0.5f, 4, 50f, 300f);
        RenderingServer.EnvironmentSetSsrHalfSize(preset != GraphicsPreset.Ultra);
        RenderingServer.EnvironmentSetVolumetricFogVolumeSize(preset == GraphicsPreset.Performance ? 48 : 64,
            preset == GraphicsPreset.Performance ? 48 : 64);

        if (env != null)
        {
            env.SsilEnabled = preset != GraphicsPreset.Performance;
            env.SsrMaxSteps = preset switch { GraphicsPreset.Ultra => 64, GraphicsPreset.High => 48, _ => 32 };
        }
        if (blur != null) blur.Samples = preset == GraphicsPreset.Performance ? 8 : 12;
    }
}
