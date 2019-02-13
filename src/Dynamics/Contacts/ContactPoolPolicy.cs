using Box2DSharp.Common;

namespace Box2DSharp.Dynamics.Contacts
{
    public class ContactPoolPolicy<TContact> : IPooledObjectPolicy<TContact>
        where TContact : Contact, new()
    {
        /// <inheritdoc />
        public TContact Create()
        {
            return new TContact();
        }

        /// <inheritdoc />
        public bool Return(TContact obj)
        {
            obj.Reset();
            return true;
        }
    }
}