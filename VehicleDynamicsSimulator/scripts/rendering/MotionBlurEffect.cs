using System;
using Godot;
using Godot.Collections;

namespace Simulator.Rendering;

/// <summary>
/// Camera motion blur. Godot 4.7 has no built-in motion blur (CameraAttributesPractical only offers DoF and
/// exposure), so this is a compositor effect: after the transparent pass it runs shaders/motion_blur.glsl on the
/// internal-resolution colour buffer using the engine's motion-vector buffer (requested via NeedsMotionVectors, the same
/// vectors TAA/FSR2 use). The blur length follows each pixel's on-screen motion (camera and object velocity); the
/// strength is faded in with the car's speed by <see cref="Strength"/> (set every frame by the camera), so a parked
/// car with a swaying camera stays sharp.
/// </summary>
[GlobalClass]
public partial class MotionBlurEffect : CompositorEffect
{
    /// <summary>Exposure as a fraction of the frame (0.5 = 180° shutter, the film/broadcast convention).</summary>
    [Export] public float Shutter { get; set; } = 0.5f;
    [Export] public float MaxBlurPixels { get; set; } = 28f;
    [Export] public int Samples { get; set; } = 12;
    /// <summary>0..1, written on the main thread each frame (speed fade, cut suppression); read on the render thread.</summary>
    public float Strength { get; set; } = 1f;

    private RenderingDevice? _rd;
    private Rid _shader, _pipeline;
    private bool _warned;
    private const string Context = "motion_blur";
    private static readonly StringName ContextName = Context, TempName = "temp";

    public MotionBlurEffect()
    {
        EffectCallbackType = EffectCallbackTypeEnum.PostTransparent;
        NeedsMotionVectors = true;
        RenderingServer.CallOnRenderThread(Callable.From(InitOnRenderThread));
    }

    private void InitOnRenderThread()
    {
        _rd = RenderingServer.GetRenderingDevice();
        if (_rd == null) return;   // compatibility renderer: no compute
        var file = GD.Load<RDShaderFile>("res://shaders/motion_blur.glsl");
        _shader = _rd.ShaderCreateFromSpirV(file.GetSpirV());
        if (_shader.IsValid) _pipeline = _rd.ComputePipelineCreate(_shader);
    }

    public override void _Notification(int what)
    {
        if (what == NotificationPredelete && _rd != null && _shader.IsValid)
            _rd.FreeRid(_shader);   // frees the dependent pipeline too
    }

    public override void _RenderCallback(int effectCallbackType, RenderData renderData)
    {
        if (_rd == null || !_pipeline.IsValid || Strength <= 0.001f) return;
        if (renderData.GetRenderSceneBuffers() is not RenderSceneBuffersRD buffers) return;
        var size = buffers.GetInternalSize();
        if (size.X <= 0 || size.Y <= 0) return;

        for (uint view = 0; view < buffers.GetViewCount(); view++)
        {
            var color = buffers.GetColorLayer(view);
            var velocity = buffers.GetVelocityLayer(view);
            if (!color.IsValid || !velocity.IsValid) return;
            var fmt = _rd.TextureGetFormat(color);
            if (fmt.Format != RenderingDevice.DataFormat.R16G16B16A16Sfloat)
            {
                if (!_warned) GD.PushWarning($"Motion blur disabled: unexpected colour format {fmt.Format}.");
                _warned = true;
                return;
            }

            if (buffers.HasTexture(ContextName, TempName))
            {
                var tf = _rd.TextureGetFormat(buffers.GetTexture(ContextName, TempName));
                if (tf.Width != size.X || tf.Height != size.Y) buffers.ClearContext(ContextName);
            }
            if (!buffers.HasTexture(ContextName, TempName))
                buffers.CreateTexture(ContextName, TempName, RenderingDevice.DataFormat.R16G16B16A16Sfloat,
                    (uint)(RenderingDevice.TextureUsageBits.StorageBit | RenderingDevice.TextureUsageBits.SamplingBit),
                    RenderingDevice.TextureSamples.Samples1, size, 1, 1, true, false);
            var temp = buffers.GetTexture(ContextName, TempName);

            Dispatch(size, color, velocity, temp, mode: 0);   // snapshot: colour → temp
            Dispatch(size, temp, velocity, color, mode: 1);   // blur: temp → colour
        }
    }

    private void Dispatch(Vector2I size, Rid input, Rid velocity, Rid output, int mode)
    {
        var set = UniformSetCacheRD.GetCache(_shader, 0, new Array<RDUniform> { Image(0, input), Image(1, velocity), Image(2, output) });
        float[] pc = { size.X, size.Y, Shutter * Strength, MaxBlurPixels, Samples, 0.75f, mode, 0f };
        var bytes = new byte[pc.Length * sizeof(float)];
        Buffer.BlockCopy(pc, 0, bytes, 0, bytes.Length);
        long list = _rd!.ComputeListBegin();
        _rd.ComputeListBindComputePipeline(list, _pipeline);
        _rd.ComputeListBindUniformSet(list, set, 0);
        _rd.ComputeListSetPushConstant(list, bytes, (uint)bytes.Length);
        _rd.ComputeListDispatch(list, (uint)((size.X + 7) / 8), (uint)((size.Y + 7) / 8), 1);
        _rd.ComputeListEnd();
    }

    private static RDUniform Image(int binding, Rid texture)
    {
        var u = new RDUniform { UniformType = RenderingDevice.UniformType.Image, Binding = binding };
        u.AddId(texture);
        return u;
    }
}
