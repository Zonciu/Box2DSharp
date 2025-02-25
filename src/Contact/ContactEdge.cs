namespace Box2DSharp
{
    /// <summary>
    /// A contact edge is used to connect bodies and contacts together
    /// in a contact graph where each body is a node and each contact
    /// is an edge. A contact edge belongs to a doubly linked list
    /// maintained in each attached body. Each contact has two contact
    /// edges, one for each attached body.
    /// 在接触图中，每个刚体是一个节点，每个接触点是一条边，接触边用于将刚体和接触点关联在一起。接触边属于每个关联刚体中的双链表。每个接触都有两条接触边，接触点关联的两个刚体各一条。
    /// </summary>
    public struct ContactEdge
    {
        /// <summary>
        /// 刚体Id
        /// </summary>
        public int BodyId;

        /// <summary>
        /// 
        /// </summary>
        public int PrevKey;

        /// <summary>
        /// 
        /// </summary>
        public int NextKey;
    }
}