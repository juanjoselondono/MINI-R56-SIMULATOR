using System;
using System.Linq;
using Godot;

namespace Simulator.Rendering;

public enum GraphicsPreset { Ultra, High, Performance }

/// <summary>
/// Forward+ quality presets for an RTX 3050-class GPU. The look (ACES, SSAO, SSIL, SSR, volumetric fog, glow, DoF,
/// motion blur) is authored in Main.tscn's WorldEnvironment and on the camera; a preset only changes resolution scaling,
/// anti-aliasing and the cost knobs of those effects:
///   Ultra        native resolution + TAA, motion blur; SSAO/SSIL high (full-res SSAO); SSR 64 steps at full resolution.
///   High         FSR 2.2 Quality (render scale 1/1.5, its own temporal AA instead of TAA); SSAO high, SSIL medium
///                (half-res); SSR 48 steps at half resolution.
///   Performance  FSR 2.2 Balanced (1/1.7); SSAO medium, SSIL off; SSR 32 steps (half-res); lighter volumetric fog and blur.
/// All presets: 16× anisotropic filtering, −0.5 texture mip bias; mesh LOD threshold 0.25 / 0.5 / 1.0 px.
/// Default: High (FSR 2.2 Quality), the sharpest and fastest preset (about 2× the road detail of native + TAA, 7.8 ms GPU
/// on Monaco; docs/validation_report.md §18). Motion blur only on Ultra (see <see cref="MotionBlurSupported"/>).
/// F6 cycles presets, `-- --gfx=ultra|high|performance` picks one at start.
/// </summary>
public static class GraphicsSettings
{
    public static GraphicsPreset Current { get; private set; } = DefaultPreset;
    /// <summary>Motion blur on/off (the camera fades its strength with speed when on).</summary>
    public static bool MotionBlurEnabled { get; set; } = true;
    /// <summary>
    /// Motion blur needs native resolution + TAA (Ultra). A compositor effect can only run before Godot's upscaler,
    /// and FSR 2.2 accumulates its jittered input over many frames: blurring that input made the history clamp accept
    /// the smears, which then built up to several times the intended length (measured: 100+ px trails at 38 km/h,
    /// validation_report §18). With FSR 2.2 the blur is therefore off.
    /// </summary>
    public static bool MotionBlurSupported { get; private set; } = true;
    public const GraphicsPreset DefaultPreset = GraphicsPreset.High;

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
        GraphicsPreset.Ultra => "Ultra: native + TAA, motion blur",
        GraphicsPreset.High => "High: FSR 2.2 Quality (no motion blur)",
        _ => "Performance: FSR 2.2 Balanced (no motion blur)",
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
        MotionBlurSupported = !fsr;
        viewport.FsrSharpness = 0.2f;
        // Texture definition: 16× anisotropic filtering on every preset (the road is seen at grazing angles, where
        // anisotropy, not texture resolution, decides sharpness). Mesh LOD: a lower pixel threshold keeps full geometry
        // further out (alpha-tested foliage additionally gets MeshTrack.FoliageLodBias).
        viewport.AnisotropicFilteringLevel = Viewport.AnisotropicFiltering.Anisotropy16X;
        // −0.5 mip bias: measured +8–14 % road sharpness at 10–80 m under FSR 2.2 without visible sparkle (−1.0 sparkles).
        viewport.TextureMipmapBias = -0.5f;
        viewport.MeshLodThreshold = preset switch { GraphicsPreset.Ultra => 0.25f, GraphicsPreset.High => 0.5f, _ => 1f };

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

        // Dev overrides for measurements (docs/validation_report.md §18): --mip-bias=<f>, --fsr-sharpness=<f>,
        // --aniso=<0..4>, --no-motion-blur.
        foreach (var a in OS.GetCmdlineUserArgs())
        {
            static float F(string v) => float.Parse(v, System.Globalization.CultureInfo.InvariantCulture);
            if (a.StartsWith("--mip-bias=")) viewport.TextureMipmapBias = F(a["--mip-bias=".Length..]);
            else if (a.StartsWith("--fsr-sharpness=")) viewport.FsrSharpness = F(a["--fsr-sharpness=".Length..]);
            else if (a.StartsWith("--aniso=")) viewport.AnisotropicFilteringLevel = (Viewport.AnisotropicFiltering)int.Parse(a["--aniso=".Length..]);
            else if (a == "--no-motion-blur") MotionBlurEnabled = false;
        }
    }
}
