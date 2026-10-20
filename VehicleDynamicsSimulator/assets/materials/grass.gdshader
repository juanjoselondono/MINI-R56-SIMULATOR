shader_type spatial;
// Grass with world-space colour variation; optional mowing stripes along s (UV.y) on the runoff.
#include "res://assets/materials/noise.gdshaderinc"

uniform vec3 grass_a : source_color = vec3(0.17, 0.33, 0.12);
uniform vec3 grass_b : source_color = vec3(0.26, 0.42, 0.16);
uniform float mow_stripes = 1.0;

varying vec3 world_pos;

void vertex() {
	world_pos = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz;
}

void fragment() {
	float n = fbm(world_pos.xz * 0.15);
	float fine = value_noise(world_pos.xz * 4.0);
	vec3 col = mix(grass_a, grass_b, n);
	float stripe = mod(floor(UV.y / 6.0), 2.0) * mow_stripes;
	col *= 0.9 + 0.12 * stripe + 0.1 * fine;
	ALBEDO = col;
	ROUGHNESS = 0.95;
}
