using UnityEngine;
using System.Collections.Generic;

#if UNITY_EDITOR
using UnityEditor;
#endif

namespace SBP
{
    public class SBPSimulator : MonoBehaviour
    {

        const float EPSILON = 1e-3f;

        [Range(1, 10)]
        public int edgeUpdateIterations = 4;
		[Range(1, 10)]
		public int collisionCheckIterations = 4;

        public Vector2 gravity = new Vector2(0f, 9.8f);

		[Header("World Bounds")]

		public bool isWorldBoundsEnabled = true;
		public Vector2 minWorldBounds = new Vector2(-10f, -10f);
		public Vector2 maxWorldBounds = new Vector2(10f, 10f);

		[Header("Debug")]

		public Color worldBoundsColor = Color.red;

		public bool drawNodesOnGizmos = true;
		public Color nodeColor = Color.white;
		public bool drawEdgesOnGizmos = true;
		public Color edgeColor = Color.green;
		public bool collisionInfo = true;

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

			// エッジの更新
            for (int i = 0; i < edgeUpdateIterations; ++i)
            {
                UpdateEdges();
            }

			// 凸包の衝突判定
			SortConvexesForSweepAndPrune();
			for (int i = 0; i < collisionCheckIterations; ++i)
			{
				CheckBroadPhaseCollision();
			}

			// シミュレーション範囲の制限
			if (isWorldBoundsEnabled)
			{
				KeepNodesInWorldBounds();
			}

			// ノードの更新
			AccumulateForces();
			UpdateNodes(dt);

			// 凸包の領域更新
			UpdateConvexBounds();
		}

		#endregion

		#region Utilities

		public Node AddNode(Node n)
		{
			_nodes.Add(n);
			return n;
		}

		public Edge AddEdge(Edge e)
		{
			_edges.Add(e);
			return e;
		}

		public Convex AddConvex(Convex c)
		{
			_convexes.Add(c);
			return c;
		}

		public Convex AddTriangle(Vector2 center, float size)
		{
			size = Mathf.Max(1e-3f, size);
			Vector2 ap = center + size * Utilities.GetAngledVector(90f * Mathf.Deg2Rad);
			Vector2 bp = center + size * Utilities.GetAngledVector(210f * Mathf.Deg2Rad);
			Vector2 cp = center + size * Utilities.GetAngledVector(-30f * Mathf.Deg2Rad);

			Node a = AddNode(new Node(ap, 1f, 0f));
			Node b = AddNode(new Node(bp, 1f, 0f));
			Node c = AddNode(new Node(cp, 1f, 0f));
			Node x = AddNode(new Node(center, 1f, 0f));

			Edge ab = AddEdge(new Edge(a, b));
			Edge bc = AddEdge(new Edge(b, c));
			Edge ca = AddEdge(new Edge(c, a));

			Edge xa = AddEdge(new Edge(x, a));
			Edge xb = AddEdge(new Edge(x, b));
			Edge xc = AddEdge(new Edge(x, c));

			Convex triangle = new Convex();
			triangle.AddNode(a);
			triangle.AddNode(b);
			triangle.AddNode(c);
			triangle.RecalculateBounds();

			return AddConvex(triangle);
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

			Node a = e.a;
			Node b = e.b;

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
			float diff = (e.restLength - length) / length;// * e.stiffness;
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

            // _nodes[e.a] = a;
			// _nodes[e.b] = b;
        }

		private void UpdateConvexBounds()
		{
			for (int i = 0; i < _convexes.Count; ++i)
			{
				_convexes[i].RecalculateBounds();
			}
		}

		private void KeepNodesInWorldBounds()
		{
			for (int i = 0; i < _nodes.Count; ++i)
			{
				Vector2 p = _nodes[i].position;
				p.x = Mathf.Clamp(p.x, minWorldBounds.x, maxWorldBounds.x);
				p.y = Mathf.Clamp(p.y, minWorldBounds.y, maxWorldBounds.y);
				_nodes[i].position = p;
			}
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
                if (Mathf.Abs(delta) < EPSILON)
                {
                    return 0;
                }
                else
                {
                    return delta > 0 ? 1 : -1;
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

					CollisionInfo info = new CollisionInfo();
					CheckNarrowPhaseCollision(a, b, ref info);
                }
            }
        }

        /// <summary>
        /// 凸包同士の衝突を判定する。
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        void CheckNarrowPhaseCollision(Convex a, Convex b, ref CollisionInfo info)
        {
			// 2つの凸包について細かく判定
            if (a.aabb.max.y < b.aabb.min.y ||
                a.aabb.min.y > b.aabb.max.y ||
                a.aabb.max.x < b.aabb.min.x ||
                a.aabb.min.x > b.aabb.max.x ||
                (a.isStatic && b.isStatic))
            {
                return;
            }

            if (CollideConvexes(a, b, ref info) && CollideConvexes(b, a, ref info))
            {
				CollisionResponse(info);
            }
        }

        /// <summary>
        /// 凸包同士の衝突判定
		/// 衝突判定には分離軸定理を用いる。
		/// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        bool CollideConvexes(Convex a, Convex b, ref CollisionInfo info)
        {
            info.a = a;
            info.b = b;

			float minDepth = info.depth;
			Vector2 collisionNormal = new Vector2();
			bool foundBestNormal = false;
			Node edgeA = null, edgeB = null;

			// 衝突時のノーマルとその深さを取得する。
            for (int i = 0, j = a.collisionNodes.Count - 1; i < a.collisionNodes.Count; j = i++)
            {
                Node n1 = _nodes[i];
                Node n2 = _nodes[j];

                Vector2 p1 = n1.position;
                Vector2 p2 = n2.position;

				// 面の法線
				Vector2 normal = new Vector2(p2.y - p1.y, p1.x - p2.x).normalized;

				float aMin, aMax, bMin, bMax, distance, center;
				ProjectConvexToAxis(a, normal, out aMin, out aMax);
				ProjectConvexToAxis(b, normal, out bMin, out bMax);
				MeasureDistanceInterval(aMin, aMax, bMin, bMax, out distance, out center);

				Vector2 centersDirection = a.aabb.center - b.aabb.center;
				float normalResponceProjection = Vector2.Dot(centersDirection, normal);

				if (distance > 0)
				{
					// 分離軸に射影した点が交差していない場合は、2つの凸法を分離するための軸が必ず存在する。
					return false;
				}
				else if (normalResponceProjection > 0 && Mathf.Abs(distance) < minDepth)
				{
					minDepth = Mathf.Abs(distance);
					collisionNormal = normal;
					foundBestNormal = true;
					edgeA = n1;
					edgeB = n2;
				}
			}

			if (foundBestNormal)
			{
				float maxDistance = 0f;
				Node firstNodeOnCorrectSide = null;

				for (int i = 0; i < b.collisionNodes.Count; i++)
				{
					// もう一つの凸包の頂点と選択された辺との位置関係を確認する。
					if (Utilities.IsClockwise(b.collisionNodes[i].position, edgeA.position, edgeB.position))
					{
						// 時計回りに位置する場合は無視
						continue;
					}

					// エッジからの距離を求める。
					float distanceToEdgeFromPoint = Utilities.DistanceToSegment(b.collisionNodes[i].position, edgeA.position, edgeB.position);
					if (distanceToEdgeFromPoint > maxDistance)
					{
						maxDistance = distanceToEdgeFromPoint;
						info.node = b.collisionNodes[i];
					}

					firstNodeOnCorrectSide = b.collisionNodes[i];
				}

				if (info.node == null)
				{
					info.node = firstNodeOnCorrectSide != null ? firstNodeOnCorrectSide : b.collisionNodes[0];
				}

				info.axis = collisionNormal;
				info.edgeA = edgeA;
				info.edgeB = edgeB;
				info.depth = minDepth;
			}


            return true;
        }

		/// <summary>
		/// 軸(axis)に凸包(convex)を射影したときの、最小値(min)と最大値(max)を求める。
		/// </summary>
		/// <param name="convex"></param>
		/// <param name="axis"></param>
		/// <param name="min"></param>
		/// <param name="max"></param>
		void ProjectConvexToAxis(Convex convex, Vector2 axis, out float min, out float max)
		{
			Node n = convex.collisionNodes[0];
			min = max = Vector2.Dot(n.position, axis);

			for (int i = 1; i < convex.collisionNodes.Count; ++i)
			{
				n = convex.collisionNodes[i];
				float t = Vector2.Dot(axis, n.position);
				if (t < min)
				{
					min = t;
				}
				else if (t > max)
				{
					max = t;
				}
			}
		}

		/// <summary>
		/// 2つの最小と最大の間の距離を求める。
		/// </summary>
		/// <param name="aMin"></param>
		/// <param name="aMax"></param>
		/// <param name="bMin"></param>
		/// <param name="bMax"></param>
		/// <param name="distance"></param>
		/// <param name="center"></param>
		void MeasureDistanceInterval(float aMin, float aMax, float bMin, float bMax, out float distance, out float center)
		{
			if (aMin < bMin)
			{
				distance = bMin - aMax;
				center = aMax + distance * 0.5f;
			}
			else
			{
				distance = aMin - bMax;
				center = bMax + distance * 0.5f;
			}
		}

		/// <summary>
		/// 衝突情報をもとにそれに対する反応を処理する。
		/// </summary>
		/// <param name="info"></param>
		void CollisionResponse(CollisionInfo info)
		{
			Vector2 collisionVector = info.axis * info.depth;
			Node edgeA = info.edgeA;
			Node edgeB = info.edgeB;
			float t = 0f;
			if (Mathf.Abs(edgeA.position.x - edgeB.position.x) > Mathf.Abs(edgeA.position.y - edgeB.position.y))
			{
				// エッジがx軸方向に膨らんでいる場合
				t = (info.node.position.x - collisionVector.x - edgeA.position.x) / (edgeB.position.x - edgeA.position.x);
			}
			else
			{
				// エッジがy軸方向に膨らんでいる場合
				t = (info.node.position.y - collisionVector.y - edgeA.position.y) / (edgeB.position.y - edgeA.position.y);
			}

			// グラフの形状がわからない場合はgeogebraなどで確認すると良い。
			float lambda = 1f / (t * t + (1f - t) * (1f - t));

			// エッジの合計の重さの逆数を求める。
			float invMassEdge = 0f;
			if (info.edgeA.mass * info.edgeB.mass > 0)
			{
				invMassEdge = 1f / (info.edgeA.mass + info.edgeB.mass);
			}

			collisionVector /= (info.node.invMass + invMassEdge);

			// エッジの移動
			info.edgeA.position += collisionVector * ((1f - t) * lambda * invMassEdge);
			info.edgeB.position += collisionVector * (t * lambda * invMassEdge);

			// 衝突点の移動
			info.node.position += collisionVector * -info.node.invMass;

			// 摩擦の適用

			// 衝突している凸包の領域の更新
			info.a.RecalculateBounds();
			info.b.RecalculateBounds();
		}

		#endregion

		#region Debug

#if UNITY_EDITOR

		private void OnDrawGizmos()
		{
			if (isWorldBoundsEnabled)
			{
				Gizmos.color = worldBoundsColor;
				Vector3 center = (minWorldBounds + maxWorldBounds) * 0.5f;
				Vector3 size = maxWorldBounds - minWorldBounds;
				Gizmos.DrawWireCube(center, size);
			}

			if (Application.isPlaying)
			{
				if (drawEdgesOnGizmos)
				{
					Gizmos.color = edgeColor;
					DrawEdgesOnGizmos();
				}
				if (drawNodesOnGizmos)
				{
					Gizmos.color = nodeColor;
					DrawNodesOnGizmos();
				}
			}
		}

		private void DrawNodesOnGizmos()
		{
			for (int i = 0; i < _nodes.Count; ++i)
			{
				Gizmos.DrawWireSphere(_nodes[i].position, 0.1f);
			}
		}

		private void DrawEdgesOnGizmos()
		{
			for (int i = 0; i < _edges.Count; ++i)
			{
				Gizmos.DrawLine(_edges[i].a.position, _edges[i].b.position);
			}
		}
#endif

#endregion
	}
}