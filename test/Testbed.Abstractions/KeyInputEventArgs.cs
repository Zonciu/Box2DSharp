namespace Testbed.Abstractions
{
    public readonly struct KeyInputEventArgs
    {
        public readonly KeyCodes Key;

        public readonly KeyModifiers Modifiers;

        public readonly bool IsRepeat;

        public bool Alt => Modifiers.IsSet(KeyModifiers.Alt);

        public bool Control => Modifiers.IsSet(KeyModifiers.Control);

        public bool Shift => Modifiers.IsSet(KeyModifiers.Shift);

        public bool Command => Modifiers.IsSet(KeyModifiers.Super);

        public KeyInputEventArgs(KeyCodes key, KeyModifiers modifiers, bool isRepeat)
        {
            Key = key;
            Modifiers = modifiers;
            IsRepeat = isRepeat;
        }
    }
}