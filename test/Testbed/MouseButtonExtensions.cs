using Testbed.Abstractions;

namespace Testbed;

public static class MouseButtonExtensions
{
    public static Testbed.Abstractions.MouseButton Convert(this OpenTK.Windowing.GraphicsLibraryFramework.MouseButton mouseButton)
    {
        return (Testbed.Abstractions.MouseButton)mouseButton;
    }

    public static Testbed.Abstractions.MouseInputEventArgs Convert(this OpenTK.Windowing.Common.MouseButtonEventArgs e)
    {
        return new MouseInputEventArgs((MouseButton)e.Button, (InputAction)e.Action, (KeyModifiers)e.Modifiers);
    }
}

public static class KeyboardExtensions
{
    public static Testbed.Abstractions.KeyModifiers Convert(this OpenTK.Windowing.GraphicsLibraryFramework.KeyModifiers keyModifiers)
    {
        return (KeyModifiers)keyModifiers;
    }
}