using UnityEngine;
using System.Collections.Generic;

namespace SBP
{
    public class SBPSimulator : MonoBehaviour
    {

        const float EPSILON = 1e-3f;

        [Range(1, 10)]
        public int edgeUpdateIterations = 4;
        public Vector2 gravity = new Vector2(0f, 9.8f);

        private List<Node> _nodes;
        private List<Edge> _edges;
        private List<Convex> _convexes;

        #region Life Cycle

        private void Awake()
        {
            _nodes = new List<Node>();
            _edges = new List<Edge>();
            _convexes = new List<Convex>();
        }

        private void Update()
        {
            float dt = Time.deltaTime;

            AccumulateForces();
            UpdateNodes(dt);

            for (int i = 0; i < edgeUpdateIterations; ++i)
            {
                UpdateEdges();
            }

        }

        #endregion

        #region Simulation

        private void AccumulateForces()
        {
            for (int i = 0; i < _nodes.Count; ++i)
            {
                Node n = _nodes[i];
                if (n.isEnabled && !n.isStatic)
                {
                    n.acceleration = gravity;
                }
            }
        }

        private void UpdateNodes(float dt)
        {
            for (int i = 0; i < _nodes.Count; ++i)
            {
                Node n = _nodes[i];
                if (n.isEnabled && !n.isStatic)
                {
                    Vector2 delta = n.position - n.prevPosition;
                    n.prevPosition = n.position;
                    n.position += delta * (1 + n.damping);
                    n.position += n.acceleration * dt * dt;
                }
            }
        }

        private void UpdateEdges()
        {
            for (int i = 0; i < _edges.Count; ++i)
            {
                UpdateEdge(i);
            }
        }

        private void UpdateEdge(int i)
        {
            Edge e = _edges[i];

            if (!e.isEnabled)
            {
                // 無効なedge
                return;
            }

            Node a = _nodes[e.a];
            Node b = _nodes[e.b];

            if (a.isStatic && b.isStatic)
            {
                // 両者ともにstatic
                return;
            }

            Vector2 aPos = a.position;
            Vector2 bPos = b.position;

            // 移動量を求める。
            Vector2 d = bPos - aPos;
            float length = d.magnitude;
            float diff = (e.restLength - length) / length * e.stiffness;
            Vector2 offset = d * (diff * 0.5f);

            if (a.isStatic)
            {
                b.position = bPos + offset;
            }
            else if (b.isStatic)
            {
                a.position = aPos - offset;
            }
            else
            {
                // 質量の比を反映する。
                float mm = a.mass + b.mass;
                float ma = a.mass / mm;
                float mb = b.mass / mm;

                // 座標を更新
                a.position = aPos - offset * mb;
                b.position = bPos + offset * ma;
            }

            _nodes[e.a] = a;
            _nodes[e.b] = b;
        }

        #endregion

        #region Collision

        /// <summary>
        /// 衝突判定を行うための事前処理
        /// 凸包のAABBのmin.xの順にソートする。
        /// </summary>
        void SortConvexesForSweepAndPrune()
        {
            _convexes.Sort((a, b) =>
            {
                float delta = a.aabb.min.x - b.aabb.min.x;
                if (Mathf.abs(delta) < EPSILON)
                {
                    return 0;
                }
                else
                {
                    return delta;
                }
            });
        }

        /// <summary>
        /// 凸包同士の衝突判定のブロードフェイズ
		/// x軸に従ってソートされているはずなのでその特定を活かす。	
        /// </summary>
        void CheckBroadPhaseCollision()
        {
            for (int i = 0; i < _convexes.Count; ++i)
            {
                for (int j = i + 1; j < _convexes.Count; ++j)
                {
                    Convex a = _convexes[i];
                    Convex b = _convexes[j];

                    if (a.aabb.max.x < b.aabb.min.x)
                    {
                        // 事前にソートしているはずなので、これ以降は衝突しない。
                        break;
                    }


                }
            }
        }

        /// <summary>
        /// 凸包同士の衝突を判定する。
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        void CheckAndHandleCollision(Convex a, Convex b, ref CollisionInfo info)
        {
            if (a.aabb.max.y < b.aabb.min.y ||
                a.aabb.min.y > b.aabb.max.y ||
                a.aabb.max.x < b.aabb.min.x ||
                a.aabb.min.x > b.aabb.max.x ||
                (a.isStatic && b.isStatic))
            {
                return;
            }

            if (CollideConvexes(a, b, ref info))
            {

            }
        }

        /// <summary>
        /// 凸包同士の衝突判定
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        bool CollideConvexes(Convex a, Convex b, ref CollisionInfo info)
        {
            info.a = a;
            info.b = b;

            for (int i = 0, j = a.indices.Count - 1; i < a.indices.Count; j = i++)
            {
                Node n1 = _nodes[a.indices[i]];
                Node n2 = _nodes[a.indices[j]];

                Vector2 p1 = n1.position;
                Vector2 p2 = n2.position;

                Vector2 normal = (p1 - p2).normalized;
            }

            return true;
        }

        #endregion
    }
}