# InputManager (autoload): reads the Logitech F710 (XInput, Godot's standard layout) and the Microsoft SideWinder
# Precision Racing Wheel USB v1.0 (045e:001a, no SDL mapping: raw events only) at the same time.
#
# Outputs, refreshed every frame, for any script (C#: GetNode("/root/InputManager").Get("throttle")):
#   throttle, brake   0..1
#   steering          -1..1, positive = LEFT (the sign VehicleController uses)
#
# Both devices stay live: the pedals take the larger of wheel and pad, steering the larger magnitude. A device resting
# inside its deadzone therefore never holds the other one back.
#
# To find the wheel's IDs and calibration values, run the game with:  $GODOT --path . -- --joy-probe
# Every raw axis and button is printed with the range seen so far. Then fill in wheel_config below.
extends Node

const NO_DEVICE := -1
const UNMAPPED := -1
const ALL_DEVICES := -1
# A device id no joypad ever has: binds the Input Map's pad events to nothing.
const UNREACHABLE_DEVICE := 9999
const SIDEWINDER_VENDOR_ID := 0x045e
const SIDEWINDER_PRODUCT_ID := 0x001a

# Edit here: an autoload added as a script has no Inspector, so @export variables could not be set from the editor.
#
# Pedals are described by their raw value at rest and fully pressed, which covers every hardware variant:
#   -1..1 pedal:             raw_released -1, raw_pressed  1
#   inverted -1..1 pedal:    raw_released  1, raw_pressed -1
#   0..1 pedal:              raw_released  0, raw_pressed  1
#   both pedals on one axis: same axis id; throttle 0 -> -1, brake 0 -> 1 (or the other way round)
# deadzone and saturation are fractions of the pedal's travel: below deadzone reads 0, above saturation reads 1.
#
# Values below are calibrated and driver-tested on the real wheel (evdev: 3 axes ABS_X/ABS_Y/ABS_RZ, 8 buttons).
var wheel_config := {
	"steering": {"axis": 0, "raw_left": -1.0, "raw_center": 0.088, "raw_right": 1.0, "deadzone": 0.02},
	"throttle": {"axis": 1, "raw_released": 1.0, "raw_pressed": -1.0, "deadzone": 0.03, "saturation": 0.98, "gamma": 1.0},
	"brake": {"axis": 2, "raw_released": 1.0, "raw_pressed": -1.0, "deadzone": 0.03, "saturation": 0.98, "gamma": 1.0},
	# Raw button index -> Input Map action.
	"buttons": {
		0: "toggle_camera",
		2: "toggle_reverse",
		5: "reset_vehicle",
		6: "shift_down",
		7: "shift_up",
	},
}

# Written to the Input Map too, so the action path (see VehicleController) applies the same deadzones.
var pad_config := {"trigger_deadzone": 0.05, "stick_deadzone": 0.10}
# Set by VehicleController from its StickSteerExponent.
var pad_stick_exponent := 1.5

var throttle := 0.0
var brake := 0.0
var steering := 0.0

var wheel_throttle := 0.0
var wheel_brake := 0.0
var wheel_steering := 0.0
var pad_throttle := 0.0
var pad_brake := 0.0
var pad_steering := 0.0

var wheel_device := NO_DEVICE
var pad_device := NO_DEVICE
# True while any wheel control is outside its deadzone.
var wheel_engaged := false

# Last raw value per wheel axis. An axis has no entry until its first event: the engine reports 0.0 until then, which
# on a -1..1 pedal at rest would read as half throttle.
var _wheel_raw := {}
var _forced_wheel := NO_DEVICE
var _forced_pad := NO_DEVICE
var _wheel_button_bindings: Array[Dictionary] = []
var _probe := false
var _probe_axes := {}


func _ready() -> void:
	process_mode = Node.PROCESS_MODE_ALWAYS
	_probe = OS.get_cmdline_user_args().has("--joy-probe")
	Input.joy_connection_changed.connect(_on_joy_connection_changed)
	apply_config()
	_probe_list_devices()


## Maps a raw pedal value to 0..1 from its calibration. Direction comes from released/pressed, so inverted pedals work.
static func normalize_pedal(raw: float, released: float, pressed: float, deadzone := 0.0, saturation := 1.0, gamma := 1.0) -> float:
	if is_equal_approx(released, pressed) or saturation <= deadzone:
		return 0.0
	var travel := clampf(inverse_lerp(released, pressed, raw), 0.0, 1.0)
	return pow(clampf(inverse_lerp(deadzone, saturation, travel), 0.0, 1.0), gamma)


## Maps a raw steering value to -1..1 (positive = left), each side scaled on its own for off-centre potentiometers.
static func normalize_steering(raw: float, raw_left: float, raw_center: float, raw_right: float, deadzone := 0.0) -> float:
	var offset := raw - raw_center
	var left_span := raw_left - raw_center
	var right_span := raw_right - raw_center
	if offset * left_span > 0.0:
		return _remove_deadzone(offset / left_span, deadzone)
	if offset * right_span > 0.0:
		return -_remove_deadzone(offset / right_span, deadzone)
	return 0.0


## Forces device ids instead of detecting them (NO_DEVICE = detect). Use it if the wheel's USB id or name does not match.
func force_devices(wheel: int, pad: int) -> void:
	_forced_wheel = wheel
	_forced_pad = pad
	apply_config()


## Re-detects the devices and re-applies wheel_config and pad_config. Call it after changing either at runtime.
func apply_config() -> void:
	var wheel := _forced_wheel
	var pad := _forced_pad
	var connected := Input.get_connected_joypads()
	if wheel == NO_DEVICE:
		for id in connected:
			if id != pad and _is_sidewinder(id):
				wheel = id
				break
	if pad == NO_DEVICE:
		for id in connected:
			if id != wheel and Input.is_joy_known(id):
				pad = id
				break
	if wheel != wheel_device:
		_wheel_raw.clear()
	wheel_device = wheel
	pad_device = pad
	_apply_pad_deadzones()
	_rebind_actions()
	if wheel_device != NO_DEVICE and (int(wheel_config["throttle"]["axis"]) == UNMAPPED or int(wheel_config["brake"]["axis"]) == UNMAPPED):
		push_warning("InputManager: wheel pedals are not mapped yet. Run with -- --joy-probe, then set wheel_config.")


func _input(event: InputEvent) -> void:
	if event is InputEventJoypadMotion:
		if event.device == wheel_device:
			_wheel_raw[int(event.axis)] = event.axis_value
		if _probe:
			_probe_axis(event)
	elif event is InputEventJoypadButton and _probe:
		print("[joy-probe] device %d button %d %s" % [event.device, event.button_index, "down" if event.pressed else "up"])


func _process(_delta: float) -> void:
	_read_wheel()
	_read_pad()
	throttle = maxf(wheel_throttle, pad_throttle)
	brake = maxf(wheel_brake, pad_brake)
	steering = wheel_steering if absf(wheel_steering) >= absf(pad_steering) else pad_steering


func _read_wheel() -> void:
	var cfg: Dictionary = wheel_config["steering"]
	var raw_steering: Variant = _wheel_raw.get(int(cfg["axis"]))
	wheel_steering = 0.0
	if raw_steering != null:
		wheel_steering = normalize_steering(raw_steering, cfg["raw_left"], cfg["raw_center"], cfg["raw_right"], cfg["deadzone"])
	wheel_throttle = _read_pedal(wheel_config["throttle"])
	wheel_brake = _read_pedal(wheel_config["brake"])
	wheel_engaged = wheel_throttle > 0.0 or wheel_brake > 0.0 or wheel_steering != 0.0


func _read_pedal(cfg: Dictionary) -> float:
	var raw: Variant = _wheel_raw.get(int(cfg["axis"]))
	if raw == null:
		return 0.0
	return normalize_pedal(raw, cfg["raw_released"], cfg["raw_pressed"], cfg["deadzone"], cfg["saturation"], cfg["gamma"])


func _read_pad() -> void:
	pad_throttle = 0.0
	pad_brake = 0.0
	pad_steering = 0.0
	if pad_device == NO_DEVICE:
		return
	var trigger_deadzone: float = pad_config["trigger_deadzone"]
	pad_throttle = _remove_deadzone(Input.get_joy_axis(pad_device, JOY_AXIS_TRIGGER_RIGHT), trigger_deadzone)
	pad_brake = _remove_deadzone(Input.get_joy_axis(pad_device, JOY_AXIS_TRIGGER_LEFT), trigger_deadzone)
	var stick := Input.get_joy_axis(pad_device, JOY_AXIS_LEFT_X)
	pad_steering = -signf(stick) * pow(_remove_deadzone(absf(stick), pad_config["stick_deadzone"]), pad_stick_exponent)


static func _remove_deadzone(value: float, deadzone: float) -> float:
	return clampf((value - deadzone) / (1.0 - deadzone), 0.0, 1.0)


func _on_joy_connection_changed(_device: int, _connected: bool) -> void:
	apply_config()
	_probe_list_devices()


func _is_sidewinder(device: int) -> bool:
	var info := Input.get_joy_info(device)
	if int(info.get("vendor_id", "0")) == SIDEWINDER_VENDOR_ID and int(info.get("product_id", "0")) == SIDEWINDER_PRODUCT_ID:
		return true
	return Input.get_joy_name(device).containsn("sidewinder")


func _apply_pad_deadzones() -> void:
	for action in ["throttle", "brake"]:
		if InputMap.has_action(action):
			InputMap.action_set_deadzone(action, pad_config["trigger_deadzone"])
	for action in ["steer_left", "steer_right"]:
		if InputMap.has_action(action):
			InputMap.action_set_deadzone(action, pad_config["stick_deadzone"])


# The Input Map's joypad events use "all devices" and standard-layout ids. A raw device's ids mean something else:
# without this, wheel button 4 would restart the car (Back) and its axis 5 would open the throttle (RT). While a wheel
# is present those events listen to the pad alone, and the wheel only triggers the buttons listed in wheel_config.
func _rebind_actions() -> void:
	for binding in _wheel_button_bindings:
		InputMap.action_erase_event(binding["action"], binding["event"])
	_wheel_button_bindings.clear()

	var pad_events_device := ALL_DEVICES
	if wheel_device != NO_DEVICE:
		pad_events_device = pad_device if pad_device != NO_DEVICE else UNREACHABLE_DEVICE
	for action in InputMap.get_actions():
		for event in InputMap.action_get_events(action):
			if event is InputEventJoypadButton or event is InputEventJoypadMotion:
				event.device = pad_events_device

	if wheel_device == NO_DEVICE:
		return
	var buttons: Dictionary = wheel_config["buttons"]
	for button in buttons:
		var action: String = buttons[button]
		if not InputMap.has_action(action):
			push_warning("InputManager: wheel button %d maps to unknown action '%s'." % [button, action])
			continue
		var event := InputEventJoypadButton.new()
		event.device = wheel_device
		event.button_index = button
		InputMap.action_add_event(action, event)
		_wheel_button_bindings.append({"action": action, "event": event})


func _probe_axis(event: InputEventJoypadMotion) -> void:
	var key := "%d:%d" % [event.device, event.axis]
	var seen: Dictionary = _probe_axes.get_or_add(key, {"min": event.axis_value, "max": event.axis_value, "last": INF})
	seen["min"] = minf(seen["min"], event.axis_value)
	seen["max"] = maxf(seen["max"], event.axis_value)
	if absf(event.axis_value - seen["last"]) >= 0.02:
		seen["last"] = event.axis_value
		print("[joy-probe] device %d axis %d raw %+.3f  (seen %+.3f .. %+.3f)" % [event.device, event.axis, event.axis_value, seen["min"], seen["max"]])


func _probe_list_devices() -> void:
	if not _probe:
		return
	for id in Input.get_connected_joypads():
		print("[joy-probe] device %d '%s' known=%s %s" % [id, Input.get_joy_name(id), Input.is_joy_known(id), Input.get_joy_info(id)])
	print("[joy-probe] wheel=%d pad=%d" % [wheel_device, pad_device])
