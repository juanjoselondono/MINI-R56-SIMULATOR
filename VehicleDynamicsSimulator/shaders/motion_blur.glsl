#[compute]
#version 450

// Per-pixel motion blur from Godot's motion-vector buffer (the one TAA/FSR2 use). Two dispatches: a snapshot of the
// colour buffer into a temporary image (the colour buffer cannot be a copy destination), then the blur reads the
// snapshot and writes the colour buffer.
// velocity = prev_uv − cur_uv in UV units (scene_forward_clustered.glsl), so velocity × resolution is the on-screen
// path of the surface over the last frame in texels. The blur integrates the colour along that path, centred on the
// pixel, over a fraction `shutter` of the frame (0.5 = 180° shutter).
// Samples from surfaces that move much less on screen than the centre pixel (the car under a chase camera) are
// down-weighted, so the sharp car does not smear into the blurred background around it.

layout(local_size_x = 8, local_size_y = 8, local_size_z = 1) in;

layout(rgba16f, set = 0, binding = 0) uniform restrict readonly image2D color_in;
layout(rg16f, set = 0, binding = 1) uniform restrict readonly image2D velocity_in;
layout(rgba16f, set = 0, binding = 2) uniform restrict writeonly image2D color_out;

layout(push_constant, std430) uniform Params {
	vec2 size;       // internal resolution [px]
	float shutter;   // exposure as a fraction of the frame time, already scaled by the speed fade
	float max_px;    // longest blur path [px]
	float samples;   // taps along the path
	float min_px;    // shorter paths are left sharp
	float mode;      // 0 = copy color_in → color_out (snapshot pass), 1 = blur
	float pad1;
} p;

void main() {
	ivec2 px = ivec2(gl_GlobalInvocationID.xy);
	ivec2 sz = ivec2(p.size);
	if (px.x >= sz.x || px.y >= sz.y) {
		return;
	}
	vec4 c0 = imageLoad(color_in, px);
	if (p.mode < 0.5) {
		imageStore(color_out, px, c0);
		return;
	}
	vec2 v = imageLoad(velocity_in, px).xy * p.size * p.shutter;
	float len = length(v);
	if (len < p.min_px) {
		imageStore(color_out, px, c0);
		return;
	}
	if (len > p.max_px) {
		v *= p.max_px / len;
		len = p.max_px;
	}
	int n = int(p.samples);
	vec4 acc = c0;
	float wsum = 1.0;
	for (int i = 0; i < n; i++) {
		float t = (float(i) + 0.5) / float(n) - 0.5;
		ivec2 q = clamp(ivec2(vec2(px) + 0.5 + v * t), ivec2(0), sz - ivec2(1));
		vec2 vq = imageLoad(velocity_in, q).xy * p.size * p.shutter;
		float w = clamp(2.0 * length(vq) / len, 0.0, 1.0);
		acc += imageLoad(color_in, q) * w;
		wsum += w;
	}
	imageStore(color_out, px, acc / wsum);
}
