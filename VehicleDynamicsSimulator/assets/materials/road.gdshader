shader_type spatial;
// Asphalt. UV.x = lateral offset [m] (+ left), UV.y = arc length s [m] along the centreline.
#include "res://assets/materials/noise.gdshaderinc"

uniform float road_half_width = 6.0;
uniform float track_length = 1000.0;
uniform float line_width = 0.18;
uniform vec3 asphalt_color : source_color = vec3(0.115, 0.118, 0.125);
uniform vec3 line_color : source_color = vec3(0.92, 0.92, 0.9);

varying vec3 world_pos;

void vertex() {
	world_pos = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz;
}

void fragment() {
	float lat = UV.x;
	float s = UV.y;
	float grain = fbm(world_pos.xz * 3.0);
	float patches = fbm(world_pos.xz * 0.05);
	// Darker rubbered band near the centre of the road (cosmetic only).
	float rubber = smoothstep(4.5, 0.5, abs(lat)) * 0.25;
	vec3 col = asphalt_color * (0.8 + 0.35 * grain + 0.25 * patches) * (1.0 - rubber * 0.35);
	float rough = 0.78 + 0.15 * grain - rubber * 0.12;

	// Painted edge lines just inside the road edge.
	float d_edge = road_half_width - abs(lat);
	float edge = step(0.12, d_edge) * step(d_edge, 0.12 + line_width);
	// Start/finish: chequered band across the road for 0 <= s < 1.6 m.
	float sm = mod(s, track_length);
	float start_band = step(sm, 1.6) * step(0.0, sm);
	float checker = mod(floor(lat / 0.4) + floor(sm / 0.4), 2.0);
	col = mix(col, line_color, edge);
	col = mix(col, mix(vec3(0.03), line_color, checker), start_band);
	rough = mix(rough, 0.55, max(edge, start_band));

	ALBEDO = col;
	ROUGHNESS = rough;
	SPECULAR = 0.35;
}
