# Writes the driving controls into project.godot's [input] section (Project Settings > Input Map), so they can be
# remapped in the editor. Gamepad bindings use Godot's standard (SDL / XInput) layout: Logitech F710 in X mode.
# Run: $GODOT --headless --path . --script res://tools/write_input_map.gd
extends SceneTree

func key(k: Key) -> InputEventKey:
	var e := InputEventKey.new()
	e.device = -1
	e.physical_keycode = k
	return e

func button(b: JoyButton) -> InputEventJoypadButton:
	var e := InputEventJoypadButton.new()
	e.device = -1
	e.button_index = b
	return e

func axis(a: JoyAxis, value: float) -> InputEventJoypadMotion:
	var e := InputEventJoypadMotion.new()
	e.device = -1
	e.axis = a
	e.axis_value = value
	return e

func _init() -> void:
	# name: [deadzone, events]. Triggers report 0..1; the stick deadzone absorbs F710 centre drift.
	var actions := {
		"throttle": [0.05, [key(KEY_W), key(KEY_UP), axis(JOY_AXIS_TRIGGER_RIGHT, 1.0)]],
		"brake": [0.05, [key(KEY_S), key(KEY_DOWN), axis(JOY_AXIS_TRIGGER_LEFT, 1.0)]],
		"steer_left": [0.1, [key(KEY_A), key(KEY_LEFT), axis(JOY_AXIS_LEFT_X, -1.0)]],
		"steer_right": [0.1, [key(KEY_D), key(KEY_RIGHT), axis(JOY_AXIS_LEFT_X, 1.0)]],
		"shift_up": [0.5, [key(KEY_E), button(JOY_BUTTON_RIGHT_SHOULDER)]],
		"shift_down": [0.5, [key(KEY_Q), button(JOY_BUTTON_LEFT_SHOULDER)]],
		"reset_vehicle": [0.5, [key(KEY_R), button(JOY_BUTTON_BACK)]],
		"pause_sim": [0.5, [key(KEY_P), button(JOY_BUTTON_START)]],
		"toggle_camera": [0.5, [key(KEY_C), button(JOY_BUTTON_Y)]],
		"toggle_autoshift": [0.5, [key(KEY_T)]],
		"toggle_headlights": [0.5, [key(KEY_H)]],
		"toggle_racing_line": [0.5, [key(KEY_L), button(JOY_BUTTON_A), button(JOY_BUTTON_DPAD_UP)]],
		"toggle_reverse": [0.5, [key(KEY_X), button(JOY_BUTTON_X)]],
		"cycle_graphics": [0.5, [key(KEY_F6)]],
		"recover_to_track": [0.5, [key(KEY_BACKSPACE), button(JOY_BUTTON_B)]],
		"toggle_setup_panel": [0.5, [key(KEY_TAB)]],
		"toggle_telemetry_panel": [0.5, [key(KEY_F2)]],
		"toggle_comparison_panel": [0.5, [key(KEY_F3)]],
		"toggle_recording": [0.5, [key(KEY_F5)]],
		"toggle_help": [0.5, [key(KEY_F1)]],
		"back_to_menu": [0.5, [key(KEY_ESCAPE)]],
	}
	for action_name in actions:
		var entry: Array = actions[action_name]
		ProjectSettings.set_setting("input/" + action_name, {"deadzone": entry[0], "events": entry[1]})
	var err := ProjectSettings.save()
	print("input map written: ", actions.size(), " actions, error=", err)
	quit()
