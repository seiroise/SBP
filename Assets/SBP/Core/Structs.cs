using UnityEngine;
using System;

namespace SBP
{
    [Serializable]
    public class Node
    {
        public Vector2 position;
        public Vector2 prevPosition;
        public Vector2 acceleration;

        private float _mass;
        public float mass { get { return _mass; } }
        private float _invMass;
        public float invMass { get { return _invMass; } }

        public float damping;
        public bool isEnabled = true;
        public bool isStatic = false;

        public Node(Vector2 position, float mass, float damping)
        {
            SetParameters(position, mass, damping);
        }

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

        /// <summary>
        /// 前回のフレームからの速度(m/frame)を返す。
        /// </summary>
        /// <returns></returns>
        public Vector2 GetFrameVelocity()
        {
            return (position - prevPosition);
        }

        /// <summary>
        /// 前回のフレームからの移動距離から単位時間あたりの速度を返す。
        /// </summary>
        /// <param name="dt"></param>
        /// <returns></returns>
        public Vector2 GetSecondVelocity(float dt)
        {
            return (position - prevPosition) / dt;
        }

        /// <summary>
        /// なんというかポジションベースの物理特有って感じ。
        /// </summary>
        /// <param name="force"></param>
        public void ApplyForce(Vector2 force)
        {
            prevPosition -= force;
        }
    }

    [Serializable]
    public class Edge
    {
        public Node a;
        public Node b;
        public float restLength;
        // public float stiffness;
        public bool isEnabled = true;

        public Edge(Node a, Node b)
        {
            SetParameters(a, b);
        }

        public void SetParameters(Node a, Node b)
        {
            this.a = a;
            this.b = b;
            this.restLength = Vector2.Distance(a.position, b.position);
        }

        public float GetInvMass()
        {
            return 1f / (a.mass + b.mass);
        }
    }

    [Serializable]
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
        public Vector2 center
        {
            get { return (min + max) * 0.5f; }
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

        public void ShrinkZero()
        {
            _min = _max = Vector2.zero;
        }
    }
}