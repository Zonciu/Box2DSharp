using System;
using System.Collections.Generic;
using Testbed.Abstractions;
using UnityEngine.InputSystem;
using UMouseButton = UnityEngine.UIElements.MouseButton;
using UModifiers = UnityEngine.EventModifiers;

namespace Box2DSharp.Testbed.Unity
{
    public class UnityInput : IInput
    {
        public UnityInput()
        {
            SetupKeyCodeMap();
            SetupUnityKeyCodeMap();
        }

        /// <inheritdoc />
        public bool IsKeyDown(KeyCodes key)
        {
            return Keyboard.current[KeyCodeMap[key]].wasPressedThisFrame;
        }

        /// <inheritdoc />
        public bool IsKeyPressed(KeyCodes key)
        {
            return Keyboard.current[KeyCodeMap[key]].isPressed;
        }

        /// <inheritdoc />
        public bool IsKeyUp(KeyCodes key)
        {
            return Keyboard.current[KeyCodeMap[key]].wasReleasedThisFrame;
        }

        /// <inheritdoc />
        public bool IsMouseDown(MouseButton button)
        {
            switch (button)
            {
            case MouseButton.Left:
                return Mouse.current.leftButton.wasPressedThisFrame;
            case MouseButton.Right:
                return Mouse.current.rightButton.wasPressedThisFrame;
            case MouseButton.Middle:
                return Mouse.current.middleButton.wasPressedThisFrame;
            default:
                throw new ArgumentOutOfRangeException(nameof(button), button, null);
            }
        }

        /// <inheritdoc />
        public bool IsMouseClicked(MouseButton button)
        {
            switch (button)
            {
            case MouseButton.Left:
                return Mouse.current.leftButton.isPressed;
            case MouseButton.Right:
                return Mouse.current.rightButton.isPressed;
            case MouseButton.Middle:
                return Mouse.current.middleButton.isPressed;
            default:
                throw new ArgumentOutOfRangeException(nameof(button), button, null);
            }
        }

        /// <inheritdoc />
        public bool IsMouseUp(MouseButton button)
        {
            switch (button)
            {
            case MouseButton.Left:
                return Mouse.current.leftButton.wasReleasedThisFrame;
            case MouseButton.Right:
                return Mouse.current.rightButton.wasReleasedThisFrame;
            case MouseButton.Middle:
                return Mouse.current.middleButton.wasReleasedThisFrame;
            default:
                throw new ArgumentOutOfRangeException(nameof(button), button, null);
            }
        }

        public static Key GetKeyCode(KeyCodes keyCodes)
        {
            return KeyCodeMap[keyCodes];
        }

        public static KeyCodes GetKeyCode(Key key)
        {
            return UnityKeyCodeMap.TryGetValue(key, out var keyCode) ? keyCode : 0;
        }

        public static UMouseButton GetMouse(MouseButton mouse)
        {
            return MouseButtonMap[(int)mouse];
        }

        public static KeyModifiers GetKeyModifiers(Keyboard keyboard)
        {
            KeyModifiers value = 0;
            if (keyboard.shiftKey.wasPressedThisFrame)
            {
                value |= KeyModifiers.Shift;
            }

            if (keyboard.ctrlKey.wasPressedThisFrame)
            {
                value |= KeyModifiers.Control;
            }

            if (keyboard.altKey.wasPressedThisFrame)
            {
                value |= KeyModifiers.Alt;
            }

            if (keyboard.leftWindowsKey.wasPressedThisFrame || keyboard.rightWindowsKey.wasPressedThisFrame)
            {
                value |= KeyModifiers.Super;
            }

            if (keyboard.capsLockKey.wasPressedThisFrame)
            {
                value |= KeyModifiers.CapsLock;
            }

            if (keyboard.numLockKey.wasPressedThisFrame)
            {
                value |= KeyModifiers.NumLock;
            }

            return value;
        }

        public static KeyModifiers GetKeyModifiers(UModifiers keyModifiers)
        {
            KeyModifiers value = 0;
            if (keyModifiers.HasFlag(UModifiers.Shift))
            {
                value |= KeyModifiers.Shift;
            }

            if (keyModifiers.HasFlag(UModifiers.Control))
            {
                value |= KeyModifiers.Control;
            }

            if (keyModifiers.HasFlag(UModifiers.Alt))
            {
                value |= KeyModifiers.Alt;
            }

            if (keyModifiers.HasFlag(UModifiers.FunctionKey))
            {
                value |= KeyModifiers.Super;
            }

            if (keyModifiers.HasFlag(UModifiers.CapsLock))
            {
                value |= KeyModifiers.CapsLock;
            }

            if (keyModifiers.HasFlag(UModifiers.Numeric))
            {
                value |= KeyModifiers.NumLock;
            }

            return value;
        }

        public static readonly Dictionary<KeyCodes, Key> KeyCodeMap = new Dictionary<KeyCodes, Key>();

        public static readonly Dictionary<Key, KeyCodes> UnityKeyCodeMap = new Dictionary<Key, KeyCodes>();

        public static readonly UMouseButton[] MouseButtonMap =
        {
            UMouseButton.LeftMouse,
            UMouseButton.RightMouse,
            UMouseButton.MiddleMouse
        };

        private void SetupKeyCodeMap()
        {
            KeyCodeMap[KeyCodes.D0] = Key.Digit0;
            KeyCodeMap[KeyCodes.D1] = Key.Digit1;
            KeyCodeMap[KeyCodes.D2] = Key.Digit2;
            KeyCodeMap[KeyCodes.D3] = Key.Digit3;
            KeyCodeMap[KeyCodes.D4] = Key.Digit4;
            KeyCodeMap[KeyCodes.D5] = Key.Digit5;
            KeyCodeMap[KeyCodes.D6] = Key.Digit6;
            KeyCodeMap[KeyCodes.D7] = Key.Digit7;
            KeyCodeMap[KeyCodes.D8] = Key.Digit8;
            KeyCodeMap[KeyCodes.D9] = Key.Digit9;
            KeyCodeMap[KeyCodes.Comma] = Key.Comma;
            KeyCodeMap[KeyCodes.Period] = Key.Period;
            KeyCodeMap[KeyCodes.LeftControl] = Key.LeftCtrl;
            KeyCodeMap[KeyCodes.RightControl] = Key.RightCtrl;
            KeyCodeMap[KeyCodes.LeftShift] = Key.LeftShift;
            KeyCodeMap[KeyCodes.RightShift] = Key.RightShift;
            KeyCodeMap[KeyCodes.LeftAlt] = Key.LeftAlt;
            KeyCodeMap[KeyCodes.RightAlt] = Key.RightAlt;
            KeyCodeMap[KeyCodes.A] = Key.A;
            KeyCodeMap[KeyCodes.B] = Key.B;
            KeyCodeMap[KeyCodes.C] = Key.C;
            KeyCodeMap[KeyCodes.D] = Key.D;
            KeyCodeMap[KeyCodes.E] = Key.E;
            KeyCodeMap[KeyCodes.F] = Key.F;
            KeyCodeMap[KeyCodes.G] = Key.G;
            KeyCodeMap[KeyCodes.H] = Key.H;
            KeyCodeMap[KeyCodes.I] = Key.I;
            KeyCodeMap[KeyCodes.J] = Key.J;
            KeyCodeMap[KeyCodes.K] = Key.K;
            KeyCodeMap[KeyCodes.L] = Key.L;
            KeyCodeMap[KeyCodes.M] = Key.M;
            KeyCodeMap[KeyCodes.N] = Key.N;
            KeyCodeMap[KeyCodes.O] = Key.O;
            KeyCodeMap[KeyCodes.P] = Key.P;
            KeyCodeMap[KeyCodes.Q] = Key.Q;
            KeyCodeMap[KeyCodes.R] = Key.R;
            KeyCodeMap[KeyCodes.S] = Key.S;
            KeyCodeMap[KeyCodes.T] = Key.T;
            KeyCodeMap[KeyCodes.U] = Key.U;
            KeyCodeMap[KeyCodes.V] = Key.V;
            KeyCodeMap[KeyCodes.W] = Key.W;
            KeyCodeMap[KeyCodes.X] = Key.X;
            KeyCodeMap[KeyCodes.Y] = Key.Y;
            KeyCodeMap[KeyCodes.Z] = Key.Z;
        }

        private void SetupUnityKeyCodeMap()
        {
            UnityKeyCodeMap[Key.Digit0] = KeyCodes.D0;
            UnityKeyCodeMap[Key.Digit1] = KeyCodes.D1;
            UnityKeyCodeMap[Key.Digit2] = KeyCodes.D2;
            UnityKeyCodeMap[Key.Digit3] = KeyCodes.D3;
            UnityKeyCodeMap[Key.Digit4] = KeyCodes.D4;
            UnityKeyCodeMap[Key.Digit5] = KeyCodes.D5;
            UnityKeyCodeMap[Key.Digit6] = KeyCodes.D6;
            UnityKeyCodeMap[Key.Digit7] = KeyCodes.D7;
            UnityKeyCodeMap[Key.Digit8] = KeyCodes.D8;
            UnityKeyCodeMap[Key.Digit9] = KeyCodes.D9;
            UnityKeyCodeMap[Key.Comma] = KeyCodes.Comma;
            UnityKeyCodeMap[Key.Period] = KeyCodes.Period;
            UnityKeyCodeMap[Key.LeftCtrl] = KeyCodes.LeftControl;
            UnityKeyCodeMap[Key.RightCtrl] = KeyCodes.RightControl;
            UnityKeyCodeMap[Key.LeftShift] = KeyCodes.LeftShift;
            UnityKeyCodeMap[Key.RightShift] = KeyCodes.RightShift;
            UnityKeyCodeMap[Key.LeftAlt] = KeyCodes.LeftAlt;
            UnityKeyCodeMap[Key.RightAlt] = KeyCodes.RightAlt;
            UnityKeyCodeMap[Key.A] = KeyCodes.A;
            UnityKeyCodeMap[Key.B] = KeyCodes.B;
            UnityKeyCodeMap[Key.C] = KeyCodes.C;
            UnityKeyCodeMap[Key.D] = KeyCodes.D;
            UnityKeyCodeMap[Key.E] = KeyCodes.E;
            UnityKeyCodeMap[Key.F] = KeyCodes.F;
            UnityKeyCodeMap[Key.G] = KeyCodes.G;
            UnityKeyCodeMap[Key.H] = KeyCodes.H;
            UnityKeyCodeMap[Key.I] = KeyCodes.I;
            UnityKeyCodeMap[Key.J] = KeyCodes.J;
            UnityKeyCodeMap[Key.K] = KeyCodes.K;
            UnityKeyCodeMap[Key.L] = KeyCodes.L;
            UnityKeyCodeMap[Key.M] = KeyCodes.M;
            UnityKeyCodeMap[Key.N] = KeyCodes.N;
            UnityKeyCodeMap[Key.O] = KeyCodes.O;
            UnityKeyCodeMap[Key.P] = KeyCodes.P;
            UnityKeyCodeMap[Key.Q] = KeyCodes.Q;
            UnityKeyCodeMap[Key.R] = KeyCodes.R;
            UnityKeyCodeMap[Key.S] = KeyCodes.S;
            UnityKeyCodeMap[Key.T] = KeyCodes.T;
            UnityKeyCodeMap[Key.U] = KeyCodes.U;
            UnityKeyCodeMap[Key.V] = KeyCodes.V;
            UnityKeyCodeMap[Key.W] = KeyCodes.W;
            UnityKeyCodeMap[Key.X] = KeyCodes.X;
            UnityKeyCodeMap[Key.Y] = KeyCodes.Y;
            UnityKeyCodeMap[Key.Z] = KeyCodes.Z;
        }
    }
}