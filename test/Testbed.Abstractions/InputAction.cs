namespace Testbed.Abstractions
{
    public enum InputAction
    {
        /// <summary>The key or mouse button was released.</summary>
        Release,

        /// <summary>The key or mouse button was pressed.</summary>
        Press,

        /// <summary>The key was held down until it repeated.</summary>
        Repeat,
    }
}