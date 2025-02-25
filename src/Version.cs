namespace Box2DSharp
{
    /// <summary>
    /// Version numbering scheme.
    /// See https://semver.org/
    /// </summary>
    public struct Version
    {
        /// Significant changes
        public int Major;

        /// Incremental changes
        public int Minor;

        /// Bug fixes
        public int Revision;

        public Version(int major, int minor, int revision)
        {
            Major = major;
            Minor = minor;
            Revision = revision;
        }
    }
}