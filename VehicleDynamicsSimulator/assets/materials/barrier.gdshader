shader_type spatial;
// Concrete wall with a painted band. UV.x = height above ground [m], UV.y = arc length s [m].
#include "res://assets/materials/noise.gdshaderinc"

uniform vec3 concrete : source_color = vec3(0.62, 0.62, 0.6);
uniform vec3 band_a : source_color = vec3(0.85, 0.08, 0.06);
uniform vec3 band_b : source_color = vec3(0.92, 0.92, 0.9);

varying vec3 world_pos;

void vertex() {
	world_pos = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz;
}

void fragment() {
	float n = fbm(world_pos.xz * 1.5 + world_pos.y * 2.0);
	vec3 col = concrete * (0.85 + 0.25 * n);
	float band = step(0.55, UV.x) * step(UV.x, 0.85);
	vec3 band_col = mix(band_a, band_b, mod(floor(UV.y / 4.0), 2.0));
	ALBEDO = mix(col, band_col, band);
	ROUGHNESS = 0.85;
}
