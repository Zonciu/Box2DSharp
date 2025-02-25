using System.Numerics;

namespace Testbed.Abstractions;

public readonly struct MouseMoveEventArgs(Vector2 position, Vector2 delta)
{
    public MouseMoveEventArgs(float x, float y, float deltaX, float deltaY)
        : this(new Vector2(x, y), new Vector2(deltaX, deltaY))
    { }

    /// <summary>
    /// Gets the new X position produced by this event.
    /// This position is relative to the top-left corner of the contents of the window.
    /// </summary>
    public float X => this.Position.X;

    /// <summary>
    /// Gets the new Y position produced by this event.
    /// This position is relative to the top-left corner of the contents of the window.
    /// </summary>
    public float Y => this.Position.Y;

    /// <summary>
    /// Gets the new position produced by this event.
    /// This position is relative to the top-left corner of the contents of the window.
    /// </summary>
    public Vector2 Position { get; } = position;

    /// <summary>Gets the change in X position since the last event.</summary>
    public float DeltaX => this.Delta.X;

    /// <summary>Gets the change in Y position since the last event.</summary>
    public float DeltaY => this.Delta.Y;

    /// <summary>Gets the change in position since the last event.</summary>
    public Vector2 Delta { get; } = delta;
}