using System;
using System.Collections.Generic;
using OpenTK.Windowing.Desktop;
using OpenTK.Windowing.GraphicsLibraryFramework;
using Testbed.Abstractions;
using KeyModifiers = Testbed.Abstractions.KeyModifiers;
using MouseButton = Testbed.Abstractions.MouseButton;
using TKMouseButton = OpenTK.Windowing.GraphicsLibraryFramework.MouseButton;
using TKKeyModifiers = OpenTK.Windowing.GraphicsLibraryFramework.KeyModifiers;

namespace Testbed
{
    public class Input : IInput
    {
        private GameWindow _gameWindow;

        static Input()
        {
            var keyCount = Enum.GetValues(typeof(KeyCodes)).Length;
            KeyMap = new Dictionary<KeyCodes, Keys>(keyCount);
            KeyCodeMap = new Dictionary<Keys, KeyCodes>(keyCount);
            SetupKeyMap();
            SetupKeyCodeMap();
        }

        public Input(GameWindow gameWindow)
        {
            _gameWindow = gameWindow;
        }

        /// <inheritdoc />
        public bool IsKeyDown(KeyCodes key)
        {
            return _gameWindow.IsKeyDown(KeyMap[key]);
        }

        /// <inheritdoc />
        public bool IsKeyPressed(KeyCodes key)
        {
            return _gameWindow.IsKeyPressed(KeyMap[key]);
        }

        /// <inheritdoc />
        public bool IsKeyUp(KeyCodes key)
        {
            return _gameWindow.IsKeyReleased(KeyMap[key]);
        }

        /// <inheritdoc />
        public bool IsMouseDown(MouseButton button)
        {
            return _gameWindow.IsMouseButtonDown(MouseButtonMap[(int)button]);
        }

        /// <inheritdoc />
        public bool IsMouseClicked(MouseButton button)
        {
            return _gameWindow.IsMouseButtonPressed(MouseButtonMap[(int)button]);
        }

        /// <inheritdoc />
        public bool IsMouseUp(MouseButton button)
        {
            return _gameWindow.IsMouseButtonReleased(MouseButtonMap[(int)button]);
        }

        public static Keys GetKey(KeyCodes keyCodes)
        {
            return KeyMap[keyCodes];
        }

        public static KeyCodes GetKeyCode(Keys key)
        {
            return KeyCodeMap.TryGetValue(key, out var keyCode) ? keyCode : 0;
        }

        public static TKMouseButton GetMouse(MouseButton mouse)
        {
            return MouseButtonMap[(int)mouse];
        }

        public static KeyModifiers GetKeyModifiers(TKKeyModifiers keyModifiers)
        {
            KeyModifiers value = 0;
            if (keyModifiers.HasFlag(TKKeyModifiers.Shift))
            {
                value |= KeyModifiers.Shift;
            }

            if (keyModifiers.HasFlag(TKKeyModifiers.Control))
            {
                value |= KeyModifiers.Control;
            }

            if (keyModifiers.HasFlag(TKKeyModifiers.Alt))
            {
                value |= KeyModifiers.Alt;
            }

            if (keyModifiers.HasFlag(TKKeyModifiers.Super))
            {
                value |= KeyModifiers.Super;
            }

            if (keyModifiers.HasFlag(TKKeyModifiers.CapsLock))
            {
                value |= KeyModifiers.CapsLock;
            }

            if (keyModifiers.HasFlag(TKKeyModifiers.NumLock))
            {
                value |= KeyModifiers.NumLock;
            }

            return value;
        }

        public static readonly Dictionary<KeyCodes, Keys> KeyMap;

        public static readonly Dictionary<Keys, KeyCodes> KeyCodeMap;

        public static readonly TKMouseButton[] MouseButtonMap =
        {
            TKMouseButton.Left,
            TKMouseButton.Right,
            TKMouseButton.Middle
        };

        private static void SetupKeyMap()
        {
            KeyMap[KeyCodes.A] = Keys.A;
            KeyMap[KeyCodes.B] = Keys.B;
            KeyMap[KeyCodes.C] = Keys.C;
            KeyMap[KeyCodes.D] = Keys.D;
            KeyMap[KeyCodes.E] = Keys.E;
            KeyMap[KeyCodes.F] = Keys.F;
            KeyMap[KeyCodes.G] = Keys.G;
            KeyMap[KeyCodes.H] = Keys.H;
            KeyMap[KeyCodes.I] = Keys.I;
            KeyMap[KeyCodes.J] = Keys.J;
            KeyMap[KeyCodes.K] = Keys.K;
            KeyMap[KeyCodes.L] = Keys.L;
            KeyMap[KeyCodes.M] = Keys.M;
            KeyMap[KeyCodes.N] = Keys.N;
            KeyMap[KeyCodes.O] = Keys.O;
            KeyMap[KeyCodes.P] = Keys.P;
            KeyMap[KeyCodes.Q] = Keys.Q;
            KeyMap[KeyCodes.R] = Keys.R;
            KeyMap[KeyCodes.S] = Keys.S;
            KeyMap[KeyCodes.T] = Keys.T;
            KeyMap[KeyCodes.U] = Keys.U;
            KeyMap[KeyCodes.V] = Keys.V;
            KeyMap[KeyCodes.W] = Keys.W;
            KeyMap[KeyCodes.X] = Keys.X;
            KeyMap[KeyCodes.Y] = Keys.Y;
            KeyMap[KeyCodes.Z] = Keys.Z;

            KeyMap[KeyCodes.Comma] = Keys.Comma;
            KeyMap[KeyCodes.Period] = Keys.Period;
            KeyMap[KeyCodes.LeftControl] = Keys.LeftControl;
            KeyMap[KeyCodes.LeftShift] = Keys.LeftShift;
            KeyMap[KeyCodes.LeftAlt] = Keys.LeftAlt;
            KeyMap[KeyCodes.RightControl] = Keys.RightControl;
            KeyMap[KeyCodes.RightShift] = Keys.RightShift;
            KeyMap[KeyCodes.RightAlt] = Keys.RightAlt;

            KeyMap[KeyCodes.D0] = Keys.D0;
            KeyMap[KeyCodes.D1] = Keys.D1;
            KeyMap[KeyCodes.D2] = Keys.D2;
            KeyMap[KeyCodes.D3] = Keys.D3;
            KeyMap[KeyCodes.D4] = Keys.D4;
            KeyMap[KeyCodes.D5] = Keys.D5;
            KeyMap[KeyCodes.D6] = Keys.D6;
            KeyMap[KeyCodes.D7] = Keys.D7;
            KeyMap[KeyCodes.D8] = Keys.D8;
            KeyMap[KeyCodes.D9] = Keys.D9;
        }

        private static void SetupKeyCodeMap()
        {
            KeyCodeMap[Keys.A] = KeyCodes.A;
            KeyCodeMap[Keys.B] = KeyCodes.B;
            KeyCodeMap[Keys.C] = KeyCodes.C;
            KeyCodeMap[Keys.D] = KeyCodes.D;
            KeyCodeMap[Keys.E] = KeyCodes.E;
            KeyCodeMap[Keys.F] = KeyCodes.F;
            KeyCodeMap[Keys.G] = KeyCodes.G;
            KeyCodeMap[Keys.H] = KeyCodes.H;
            KeyCodeMap[Keys.I] = KeyCodes.I;
            KeyCodeMap[Keys.J] = KeyCodes.J;
            KeyCodeMap[Keys.K] = KeyCodes.K;
            KeyCodeMap[Keys.L] = KeyCodes.L;
            KeyCodeMap[Keys.M] = KeyCodes.M;
            KeyCodeMap[Keys.N] = KeyCodes.N;
            KeyCodeMap[Keys.O] = KeyCodes.O;
            KeyCodeMap[Keys.P] = KeyCodes.P;
            KeyCodeMap[Keys.Q] = KeyCodes.Q;
            KeyCodeMap[Keys.R] = KeyCodes.R;
            KeyCodeMap[Keys.S] = KeyCodes.S;
            KeyCodeMap[Keys.T] = KeyCodes.T;
            KeyCodeMap[Keys.U] = KeyCodes.U;
            KeyCodeMap[Keys.V] = KeyCodes.V;
            KeyCodeMap[Keys.W] = KeyCodes.W;
            KeyCodeMap[Keys.X] = KeyCodes.X;
            KeyCodeMap[Keys.Y] = KeyCodes.Y;
            KeyCodeMap[Keys.Z] = KeyCodes.Z;

            KeyCodeMap[Keys.Comma] = KeyCodes.Comma;
            KeyCodeMap[Keys.Period] = KeyCodes.Period;
            KeyCodeMap[Keys.LeftControl] = KeyCodes.LeftControl;
            KeyCodeMap[Keys.LeftShift] = KeyCodes.LeftShift;
            KeyCodeMap[Keys.LeftAlt] = KeyCodes.LeftAlt;
            KeyCodeMap[Keys.RightControl] = KeyCodes.RightControl;
            KeyCodeMap[Keys.RightShift] = KeyCodes.RightShift;
            KeyCodeMap[Keys.RightAlt] = KeyCodes.RightAlt;

            KeyCodeMap[Keys.D0] = KeyCodes.D0;
            KeyCodeMap[Keys.D1] = KeyCodes.D1;
            KeyCodeMap[Keys.D2] = KeyCodes.D2;
            KeyCodeMap[Keys.D3] = KeyCodes.D3;
            KeyCodeMap[Keys.D4] = KeyCodes.D4;
            KeyCodeMap[Keys.D5] = KeyCodes.D5;
            KeyCodeMap[Keys.D6] = KeyCodes.D6;
            KeyCodeMap[Keys.D7] = KeyCodes.D7;
            KeyCodeMap[Keys.D8] = KeyCodes.D8;
            KeyCodeMap[Keys.D9] = KeyCodes.D9;
        }
    }
}