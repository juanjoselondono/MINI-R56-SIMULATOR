shader_type spatial;
// Red/white curb band. UV.y = arc length s [m].
uniform float stripe_length = 2.0;
uniform vec3 red : source_color = vec3(0.78, 0.06, 0.05);
uniform vec3 white : source_color = vec3(0.92, 0.92, 0.9);

void fragment() {
	float stripe = mod(floor(UV.y / stripe_length), 2.0);
	ALBEDO = mix(red, white, stripe);
	ROUGHNESS = 0.6;
}
