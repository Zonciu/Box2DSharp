namespace Testbed.Abstractions
{
    public readonly struct MouseInputEventArgs
    {
        /// <summary>
        /// Initializes a new instance of the <see cref="T:OpenTK.Windowing.Common.MouseButtonEventArgs" /> struct.
        /// </summary>
        /// <param name="button">The mouse button for the event.</param>
        /// <param name="action">The action of the mouse button.</param>
        /// <param name="modifiers">The key modifiers held during the mouse button's action.</param>
        public MouseInputEventArgs(MouseButton button, InputAction action, KeyModifiers modifiers)
        {
            Button = button;
            Action = action;
            Modifiers = modifiers;
        }

        public MouseButton Button { get; }

        public InputAction Action { get; }

        public KeyModifiers Modifiers { get; }

        public bool IsPressed => Action != InputAction.Release;
    }
}