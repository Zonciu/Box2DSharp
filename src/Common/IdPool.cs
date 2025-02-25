using System.Diagnostics;

namespace Box2DSharp
{
    public class IdPool
    {
        private B2Array<int> _freeArray;

        private int _nextIndex;

        public IdPool(int capacity = 32)
        {
            _freeArray = new(capacity);
        }

        public void DestroyIdPool()
        {
            _freeArray.Dispose();
            _freeArray = null!;
        }

        public int AllocId()
        {
            int id;
            if (_freeArray.Count > 0)
            {
                id = _freeArray.Last();
                _freeArray.Pop();
            }
            else
            {
                id = _nextIndex;
                _nextIndex += 1;
            }

            return id;
        }

        public void FreeId(int id)
        {
            Debug.Assert(_nextIndex > 0);
            Debug.Assert(0 <= id && id < _nextIndex);

            if (id == _nextIndex)
            {
                _nextIndex -= 1;
                return;
            }

            _freeArray.Push(id);
        }

        public void ValidateFreeId(int id)
        {
            if (!Core.B2Validate)
            {
                return;
            }

            int freeCount = _freeArray.Count;
            for (int i = 0; i < freeCount; ++i)
            {
                if (_freeArray[i] == id)
                {
                    return;
                }
            }

            Debug.Assert(false);
        }

        public int GetIdCount()
        {
            return _nextIndex - _freeArray.Count;
        }

        public int GetIdCapacity()
        {
            return _nextIndex;
        }

        public int GetIdBytes()
        {
            return _freeArray.Capacity * sizeof(int);
        }
    }
}