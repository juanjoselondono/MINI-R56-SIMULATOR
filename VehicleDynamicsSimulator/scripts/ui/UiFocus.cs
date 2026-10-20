using Godot;

namespace Simulator.UI;

/// <summary>Driving keys must not act while the user types into a text or number field.</summary>
public static class UiFocus
{
    public static bool IsTyping(Viewport? viewport)
        => viewport?.GuiGetFocusOwner() is LineEdit or TextEdit;
}
