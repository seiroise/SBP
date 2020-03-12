using UnityEngine;

namespace SBP
{
    public struct Node
    {
        public Vector2 position;
        public Vector2 prevPosition;
        public Vector2 acceleration;

        private float _mass;
        public float mass { get { return _mass; } }
        private float _invMass;
        public float invMass { get { return _invMass; } }

        public float damping;
        public bool isEnabled;
        public bool isStatic;

        public void SetParameters(Vector2 position, float mass, float damping)
        {
            this.position = position;
            this.prevPosition = position;

            SetMass(mass);

            this.damping = damping;
        }

        public void SetMass(float mass)
        {
            this._mass = Mathf.Max(mass, 1e-3f);
            this._invMass = 1f / mass;
        }
    }

    public struct Edge
    {
        public int a;
        public int b;
        public float restLength;
        public float stiffness;
        public bool isEnabled;

        public void SetParameters(int a, int b, float restLength, float stiffness)
        {
            this.a = a;
            this.b = b;
            this.restLength = restLength;
            this.stiffness = stiffness;
        }
    }

    public struct AABB
    {
        private Vector2 _min;
        private Vector2 _max;

        public Vector2 min
        {
            get { return _min; }
        }
        public Vector2 max
        {
            get { return _max; }
        }

        public AABB(Vector2 min, Vector2 max)
        {
            this._min = min;
            this._max = max;
        }

        public static AABB CreateAABB(Vector2 center, Vector2 extents)
        {
            extents.x = Mathf.Max(extents.x, 0f);
            extents.y = Mathf.Max(extents.y, 0f);
            Vector2 min = center - extents * 0.5f;
            return new AABB(min, min + extents);
        }

        public void Encapsulate(Vector2 p)
        {
            if (p.x < _min.x)
            {
                _min.x = p.x;
            }
            else if (_max.x < p.x)
            {
                _max.x = p.x;
            }
            if (p.y < _min.y)
            {
                _min.y = p.y;
            }
            else if (_max.y < p.y)
            {
                _max.y = p.y;
            }
        }
    }
}