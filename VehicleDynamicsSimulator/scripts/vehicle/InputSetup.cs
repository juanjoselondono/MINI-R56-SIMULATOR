using Godot;

namespace Simulator.Vehicle;

/// <summary>Registers the driving actions (keyboard + gamepad) at runtime so project.godot stays readable.</summary>
public static class InputSetup
{
    public const string Throttle = "throttle";
    public const string Brake = "brake";
    public const string SteerLeft = "steer_left";
    public const string SteerRight = "steer_right";
    public const string ShiftUp = "shift_up";
    public const string ShiftDown = "shift_down";
    public const string ResetVehicle = "reset_vehicle";
    public const string Pause = "pause_sim";
    public const string ToggleCamera = "toggle_camera";
    public const string ToggleAutoShift = "toggle_autoshift";
    public const string ToggleHeadlights = "toggle_headlights";

    public static void EnsureActions()
    {
        Add(Throttle, Key(Godot.Key.W), Key(Godot.Key.Up), Axis(JoyAxis.TriggerRight, 1));
        Add(Brake, Key(Godot.Key.S), Key(Godot.Key.Down), Axis(JoyAxis.TriggerLeft, 1));
        Add(SteerLeft, Key(Godot.Key.A), Key(Godot.Key.Left), Axis(JoyAxis.LeftX, -1));
        Add(SteerRight, Key(Godot.Key.D), Key(Godot.Key.Right), Axis(JoyAxis.LeftX, 1));
        Add(ShiftUp, Key(Godot.Key.E), Button(JoyButton.RightShoulder));
        Add(ShiftDown, Key(Godot.Key.Q), Button(JoyButton.LeftShoulder));
        Add(ResetVehicle, Key(Godot.Key.R), Button(JoyButton.Back));
        Add(Pause, Key(Godot.Key.P), Button(JoyButton.Start));
        Add(ToggleCamera, Key(Godot.Key.C), Button(JoyButton.Y));
        Add(ToggleAutoShift, Key(Godot.Key.T));
        Add(ToggleHeadlights, Key(Godot.Key.H));
    }

    private static void Add(string action, params InputEvent[] events)
    {
        if (InputMap.HasAction(action)) return;
        InputMap.AddAction(action, 0.15f);
        foreach (var e in events) InputMap.ActionAddEvent(action, e);
    }

    private static InputEvent Key(Key k) => new InputEventKey { PhysicalKeycode = k };
    private static InputEvent Button(JoyButton b) => new InputEventJoypadButton { ButtonIndex = b };
    private static InputEvent Axis(JoyAxis a, float dir) => new InputEventJoypadMotion { Axis = a, AxisValue = dir };
}
