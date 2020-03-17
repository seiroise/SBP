using UnityEngine;
using System.Collections.Generic;

#if UNITY_EDITOR
using UnityEditor;
#endif

namespace SBP
{
    public class SBPSimulator : MonoBehaviour
    {

        static readonly float EPSILON = 1e-3f;
        static readonly float LARGE = 1e+8f;
        static readonly float FRICTION_RATIO = 0.5f;
        static readonly float FRICTION_MIN = 0.3f;

        static readonly Vector3 DEBUG_CUBE_S = Vector3.one * 0.05f;
        static readonly Vector3 DEBUG_CUBE_M = Vector3.one * 0.1f;

        [Range(1, 10)]
        public int edgeUpdateIterations = 4;
        [Range(1, 10)]
        public int collisionCheckIterations = 4;

        public Vector2 gravity = new Vector2(0f, 9.8f);

        [Header("World Bounds")]

        public bool isWorldBoundsEnabled = true;
        public Vector2 maxWorldBounds = new Vector2(10f, 10f);
        public Vector2 minWorldBounds = new Vector2(-10f, -10f);

        [Header("Debug")]

        public Color worldBoundsColor = Color.red;

        public bool drawNodesOnGizmos = true;
        public Color nodeColor = Color.white;
        public bool drawEdgesOnGizmos = true;
        public Color edgeColor = Color.green;
        public bool drawNormalsOnGizmos = false;
        public Color normalColor = Color.yellow;
        public bool drawCollisionInfo = false;

        private List<Node> _nodes;
        private List<Edge> _edges;
        private List<Convex> _convexes;

        // 衝突情報を保持しておくため
        private int _collisionInfoIndex = 0;
        private List<CollisionInfo> _collisionInfoList;

        // SATの結果を保持する構造体。２つあれば事足りるので保持しておく。
        // マルチスレッド化する場合は、スレッドごとにこのペアが必要になる。
        // マルチスレッド化出来るのか...？
        private SATResult satResultAB;
        private SATResult satResultBA;

        #region Life Cycle

        private void Awake()
        {
            _nodes = new List<Node>();
            _edges = new List<Edge>();
            _convexes = new List<Convex>();

            _collisionInfoList = new List<CollisionInfo>();
        }

        private void Update()
        {
            float dt = Time.deltaTime;
            dt = 0.016f;

            // ノードの更新
            AccumulateForces();
            UpdateNodes(dt);

            // エッジの更新
            for (int i = 0; i < edgeUpdateIterations; ++i)
            {
                UpdateEdges();
            }

            // 凸包の衝突判定
            SortConvexesForSweepAndPrune();	// 本当はこれも反復中にやったほうがいいんだろうけどね...
            for (int i = 0; i < collisionCheckIterations; ++i)
            {
                ProcessCollision();
            }

            // シミュレーション範囲の制限
            if (isWorldBoundsEnabled)
            {
                KeepNodesInWorldBounds();
            }

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

        public Convex AddConvex(Convex c, bool addNodes, bool addEdges)
        {
            _convexes.Add(c);
            if (addNodes)
            {
                for (int i = 0; i < c.collisionNodes.Count; ++i)
                {
                    AddNode(c.collisionNodes[i]);
                }
                for (int i = 0; i < c.helperNodes.Count; ++i)
                {
                    AddNode(c.helperNodes[i]);
                }
            }
            if (addEdges)
            {
                for (int i = 0; i < c.edges.Count; ++i)
                {
                    AddEdge(c.edges[i]);
                }
            }
            return c;
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

        /// <summary>
        /// エッジによるノードの更新
        /// </summary>
        /// <param name="i"></param>
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

        /// <summary>
        /// 凸包の境界情報を更新する。
        /// </summary>
        private void UpdateConvexBounds()
        {
            for (int i = 0; i < _convexes.Count; ++i)
            {
                _convexes[i].RecalculateBounds();
            }
        }

        /// <summary>
        /// ノードを設定されている境界内に留める。
        /// </summary>
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

        /// <summary>
        /// 衝突情報を格納するためのオブジェクトを取得する。
        /// </summary>
        private CollisionInfo GetCollisionInfo()
        {
            CollisionInfo infoObject = null;
            if (_collisionInfoIndex < _collisionInfoList.Count)
            {
                infoObject = _collisionInfoList[_collisionInfoIndex];
                infoObject.Reset();
            }
            else
            {
                infoObject = new CollisionInfo();
                _collisionInfoList.Add(infoObject);
            }
            _collisionInfoIndex++;
            return infoObject;
        }

        /// <summary>
        /// 衝突情報を格納するためのオブジェクト番号のリセット
        /// </summary>
        private void ResetCollisionInfoIndex()
        {
            _collisionInfoIndex = 0;
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
            min = max = Vector2.Dot(axis, n.position);

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
        /// ２つの範囲の共通部分の量を測る。(符号付き)
        /// </summary>
        /// <param name="aMin"></param>
        /// <param name="aMax"></param>
        /// <param name="bMin"></param>
        /// <param name="bMax"></param>
        /// <param name="signedAmount"></param>
        void MeasureOverlappedSignedAmount(float aMin, float aMax, float bMin, float bMax, out float signedAmount)
        {
            if (aMin < bMin)
            {
                signedAmount = aMax - bMin;
                // center = (bMin + aMax) * 0.5f;
            }
            else
            {
                signedAmount = bMax - aMin;
                // center = (aMin + bMax) * 0.5f;
            }
        }

        #endregion

        #region Broad phase Collision detection

        /// <summary>
        /// 衝突処理
        /// </summary>
        void ProcessCollision()
        {
            ResetCollisionInfoIndex();
            CheckBroadPhaseCollision();
            // ProcessCollisionInfoList();
        }

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

                    CheckNarrowPhaseCollision2(a, b);
                }
            }
        }

        #endregion

#if false
        #region Narrow phase collision detection 1

        /// <summary>
        /// 凸包同士の衝突を判定する。
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        void CheckNarrowPhaseCollision(Convex a, Convex b)
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

            CollisionInfo info = GetCollisionInfo();
            if (CollideConvexes(a, b, ref info) && CollideConvexes(b, a, ref info))
            {
                CollisionResponse(info);
                _collisionInfoList.Add(info);
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
            // aのエッジにbのノードが挿入されているかどうかを判定する。
            info.convexA = a;
            info.convexB = b;

            float minDepth = info.depth;
            Vector2 collisionNormal = new Vector2();
            bool foundBestNormal = false;
            Node edgeA = null, edgeB = null;

            // 衝突時のノーマルとその深さを取得する。
            for (int i = 0, j = a.collisionNodes.Count - 1; i < a.collisionNodes.Count; j = i++)
            {
                Node n1 = a.collisionNodes[i];
                Node n2 = a.collisionNodes[j];

                Vector2 p1 = n1.position;
                Vector2 p2 = n2.position;

                // 面の法線
                // 2点間の法線を求める賢い方法やね。
                // Vector2 normal = new Vector2(p1.y - p2.y, p2.x - p1.x).normalized;
                Vector2 normal = new Vector2(p2.y - p1.y, p1.x - p2.x).normalized;
                // Vector2 normal = Utilities.RotateVector(p1 - p2, 90f * Mathf.Deg2Rad).normalized;

                // 法線に対して2つの凸包の頂点を射影し、その最大値と最小値を求める。
                // Aの最大値とBの最小値、またはAの最小値とBの最大値の符号付き距離(signed distance)を計算する。
                // 距離が負の場合はその辺n1,n2に対してその距離の絶対値だけめり込みが起きていることになる。
                float aMin, aMax, bMin, bMax, signedAmount;
                ProjectConvexToAxis(a, normal, out aMin, out aMax);
                ProjectConvexToAxis(b, normal, out bMin, out bMax);
                MeasureOverlappedSignedAmount(aMin, aMax, bMin, bMax, out signedAmount);

                // 中心座標間の差分ベクトルと法線との内積を求める。
                Vector2 centersDirection = a.aabb.center - b.aabb.center;
                float normalResponceProjection = Vector2.Dot(centersDirection, normal);

                if (signedAmount <= 0)
                {
                    // 分離軸に射影した点が交差していない場合は、2つの凸法を分離するための軸が必ず存在する。
                    return false;
                }
                else if (normalResponceProjection > 0 && signedAmount < minDepth)
                {
                    // 法線と中心間の差分ベクトルとの向きが90度未満で、
                    // めり込み距離が現在記録しているものよりも小さい場合。
                    minDepth = Mathf.Abs(signedAmount);
                    collisionNormal = normal;
                    foundBestNormal = true;
                    edgeA = n2;
                    edgeB = n1;
                }
            }

            if (foundBestNormal)
            {
                float maxDistance = 1e+5f;
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
                    if (distanceToEdgeFromPoint < maxDistance)
                    {
                        maxDistance = distanceToEdgeFromPoint;
                        info.penetratedNode = b.collisionNodes[i];
                    }

                    firstNodeOnCorrectSide = b.collisionNodes[i];
                }

                if (info.penetratedNode == null)
                {
                    info.penetratedNode = firstNodeOnCorrectSide != null ? firstNodeOnCorrectSide : b.collisionNodes[0];
                }

                info.axis = collisionNormal;
                info.edgeNodeA = edgeA;
                info.edgeNodeB = edgeB;
                info.depth = minDepth;
            }
            return true;
        }

        #endregion
#endif

        #region Narrow phase collision detection 2

        /// <summary>
        /// 二凸包間の衝突判定
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <param name="info"></param>
        void CheckNarrowPhaseCollision2(Convex a, Convex b)
        {
            // 2つの凸包が衝突しているかの少し粗い確認
            if (a.aabb.max.y < b.aabb.min.y ||
                a.aabb.min.y > b.aabb.max.y ||
                a.aabb.max.x < b.aabb.min.x ||
                a.aabb.min.x > b.aabb.max.x ||
                (a.isStatic && b.isStatic))
            {
                return;
            }

            // ２つの凸包に対して分離線が定義出来るかの確認
            if (CheckCollisionWithSAT(a, b, ref satResultAB) && CheckCollisionWithSAT(b, a, ref satResultBA))
            {
                // ２つの凸包間でそれぞれ分離軸が定義できないので、２つの凸包は衝突している。
                CollisionInfo info = GetCollisionInfo();

                if (satResultAB.penetration < satResultBA.penetration)
                {
                    // FindShallowestPenetratedNode(b, ref satResultAB);
                    FindDeepestPenetratedNode(b, ref satResultAB);
                    info.convexA = a;
                    info.convexB = b;
                    info.edgeNodeA = satResultAB.edgeNodeA;
                    info.edgeNodeB = satResultAB.edgeNodeB;
                    info.penetratedNode = satResultAB.penetratedNode;
                    info.depth = satResultAB.penetration;
                    info.axis = satResultAB.axis;
                }
                else
                {
                    // FindShallowestPenetratedNode(a, ref satResultBA);
                    FindDeepestPenetratedNode(a, ref satResultBA);
                    info.convexA = b;
                    info.convexB = a;
                    info.edgeNodeA = satResultBA.edgeNodeA;
                    info.edgeNodeB = satResultBA.edgeNodeB;
                    info.penetratedNode = satResultBA.penetratedNode;
                    info.depth = satResultBA.penetration;
                    info.axis = satResultBA.axis;
                }
                _collisionInfoList.Add(info);
                CollisionResponse(info);
            }
        }

        /// <summary>
        /// SAT(分離軸定理)による衝突の検出
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <param name="info"></param>
        bool CheckCollisionWithSAT(Convex a, Convex b, ref SATResult result)
        {
            // ２つの凸包を分離する分離線が存在する場合、それは２つの凸包のいずれかの辺に平行である。
            // これが凸包同士の分離軸定理なので、これをもとに衝突判定を行い、それを解決するための情報を集める。
            // そのためにまずは最も重なりの少ない分離軸を探索する。
            // その後、その分離軸に対して最も深くめり込んでいる頂点を処理する。

            result.penetration = LARGE;
            result.edgeNodeA = null;
            result.edgeNodeB = null;

            for (int i = 0, j = a.collisionNodes.Count - 1; i < a.collisionNodes.Count; j = i++)
            {
                Node edgeNodeA = a.collisionNodes[j];
                Node edgeNodeB = a.collisionNodes[i];

                Vector2 pa = edgeNodeA.position;
                Vector2 pb = edgeNodeB.position;

                Vector2 axis = new Vector2(pb.y - pa.y, pa.x - pb.x).normalized;

                float aMin, aMax, bMin, bMax, signedAmount;
                ProjectConvexToAxis(a, axis, out aMin, out aMax);
                ProjectConvexToAxis(b, axis, out bMin, out bMax);
                MeasureOverlappedSignedAmount(aMin, aMax, bMin, bMax, out signedAmount);

                if (signedAmount <= 0f)
                {
                    // 重なりが存在しない場合はその範囲に分離線が定義できてしまうので、衝突しない。
                    return false;
                }

                if (signedAmount < result.penetration)
                {
                    // 重なりが最も小さい場合はその量と辺を構成するノードを記録する。
                    result.penetration = signedAmount;
                    result.edgeNodeA = edgeNodeA;
                    result.edgeNodeB = edgeNodeB;
                    result.axis = axis;
                }
            }
            return true;
        }

        /// <summary>
        /// 最も浅いノードを返す。
        /// </summary>
        /// <param name="convex"></param>
        /// <param name="result"></param>
        void FindShallowestPenetratedNode(Convex convex, ref SATResult result)
        {
            Vector2 pa = result.edgeNodeA.position;
            Vector2 pb = result.edgeNodeB.position;

            float minDistance = LARGE;
            Node bestNode = null;

            for (int i = 0; i < convex.collisionNodes.Count; ++i)
            {
                Node n = convex.collisionNodes[i];
                float distance = Utilities.DistanceToSegment(n.position, pa, pb);
                if (distance < minDistance)
                {
                    minDistance = distance;
                    bestNode = n;
                }
            }
            result.penetratedNode = bestNode;
        }

        /// <summary>
        /// 最も深くめり込んでいるノードを返す。
        /// </summary>
        /// <param name="convex"></param>
        /// <param name="result"></param>
        void FindDeepestPenetratedNode(Convex convex, ref SATResult result)
        {
            Vector2 pa = result.edgeNodeA.position;
            Vector2 pb = result.edgeNodeB.position;
            Vector2 mid = (pa + pb) * 0.5f;

            float minDistance = LARGE;
            Node bestNode = null;

            for (int i = 0; i < convex.collisionNodes.Count; ++i)
            {
                // 軸方向に射影した場合に、最も小さいものが一番めり込んでいる。
                Node n = convex.collisionNodes[i];
                float distance = Vector2.Dot(n.position - mid, result.axis);
                if (distance < minDistance)
                {
                    minDistance = distance;
                    bestNode = n;
                }
            }
            result.penetratedNode = bestNode;
        }

        #endregion

        #region Collision Responce

        /// <summary>
        /// 検出された衝突情報への処理。
        /// </summary>
        void ProcessCollisionInfoList()
        {
            for (int i = 0; i < _collisionInfoIndex; ++i)
            {
                CollisionResponse(_collisionInfoList[i]);
            }
        }

        /// <summary>
        /// 衝突情報をもとにそれに対する反応を処理する。
        /// </summary>
        /// <param name="info"></param>
        void CollisionResponse(CollisionInfo info)
        {
            Vector2 collisionVector = info.axis * info.depth;
            Node edgeA = info.edgeNodeA;
            Node edgeB = info.edgeNodeB;
            float t = 0f;
            if (Mathf.Abs(edgeA.position.x - edgeB.position.x) > Mathf.Abs(edgeA.position.y - edgeB.position.y))
            {
                // エッジがx軸方向に膨らんでいる場合
                t = (info.penetratedNode.position.x - collisionVector.x - edgeA.position.x) / (edgeB.position.x - edgeA.position.x);
            }
            else
            {
                // エッジがy軸方向に膨らんでいる場合
                t = (info.penetratedNode.position.y - collisionVector.y - edgeA.position.y) / (edgeB.position.y - edgeA.position.y);
            }

            // グラフの形状がわからない場合はgeogebraなどで確認すると良い。
            float lambda = 1f / (t * t + (1f - t) * (1f - t));

            // エッジの合計の重さの逆数を求める。
            float invMassEdge = 0f;
            if (info.edgeNodeA.mass * info.edgeNodeB.mass > 0)
            {
                invMassEdge = 1f / (info.edgeNodeA.mass + info.edgeNodeB.mass);
            }

            collisionVector /= (info.penetratedNode.invMass + invMassEdge);

            info.pa = info.edgeNodeA.position + (collisionVector * ((1f - t) * lambda * invMassEdge));
            info.pb = info.edgeNodeB.position + (collisionVector * (t * lambda * invMassEdge));
            info.pn = info.penetratedNode.position - (collisionVector * info.penetratedNode.invMass);

            // エッジの移動
            info.edgeNodeA.position -= collisionVector * ((1f - t) * lambda * invMassEdge);
            info.edgeNodeB.position -= collisionVector * (t * lambda * invMassEdge);

            // 衝突点の移動
            info.penetratedNode.position += collisionVector * info.penetratedNode.invMass;

            // 摩擦の適用
            ApplyFriction(info);

            // 衝突している凸包の領域の更新
            info.convexA.RecalculateBounds();
            info.convexB.RecalculateBounds();
        }

        /// <summary>
        /// 衝突判定をもとにその接線方向に働く摩擦を適応する。
        /// </summary>
        /// <param name="info"></param>
        private void ApplyFriction(CollisionInfo info)
        {
            Node a = info.edgeNodeA;
            Node b = info.edgeNodeB;
            Node n = info.penetratedNode;
            Vector2 tangentDirection = (b.position - a.position).normalized;

            // ノードとエッジの速度。
            // エッジの速度は構成ノードの平均速度とする。
            Vector2 nodeVelocity = n.GetFrameVelocity();
            Vector2 edgeVelocity = (a.GetFrameVelocity() + b.GetFrameVelocity()) * 0.5f;

            // 摩擦割合
            // 値域 : [FRICTION_MIN,1]
            float frictionRatio = Mathf.Max(Mathf.Min(info.depth / FRICTION_RATIO, 1f), FRICTION_MIN);

            // ノードとエッジの接線方向の速度
            // 速度を表すベクトル => velocity, 速度を表すスカラー => speed
            // という使い分け。
            float tangentNodeSpeed = Vector2.Dot(nodeVelocity, tangentDirection);
            float tangentEdgeSpeed = Vector2.Dot(edgeVelocity, tangentDirection);

            // エッジの質量の逆数
            float invMassEdge = 1f / (a.mass + b.mass);

            // 速度差
            float speedDelta = tangentNodeSpeed - tangentEdgeSpeed;
            float deltaToApply = (speedDelta * frictionRatio) / (n.invMass + invMassEdge);

            // 反映
            n.ApplyForce(tangentDirection * (-deltaToApply * n.invMass));
            a.ApplyForce(tangentDirection * (deltaToApply * a.invMass));
            b.ApplyForce(tangentDirection * (deltaToApply * b.invMass));
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
				if (drawNormalsOnGizmos)
				{
					Gizmos.color = normalColor;
					DrawConvexNormalsOnGizmos();
				}
				if (drawCollisionInfo)
				{
					DrawCollisionInfoList();
				}
			}
		}

		private void DrawNodesOnGizmos()
		{
			for (int i = 0; i < _nodes.Count; ++i)
			{
				Gizmos.DrawCube(_nodes[i].position, DEBUG_CUBE_S);
			}
		}

		private void DrawEdgesOnGizmos()
		{
			for (int i = 0; i < _edges.Count; ++i)
			{
				Gizmos.DrawLine(_edges[i].a.position, _edges[i].b.position);
			}
		}

		private void DrawConvexNormalsOnGizmos()
		{
			for (int k = 0; k < _convexes.Count; ++k)
			{
				Convex c = _convexes[k];
				for (int i = 0, j = c.collisionNodes.Count - 1; i < c.collisionNodes.Count; j = i++)
				{
					Node a = c.collisionNodes[j];
					Node b = c.collisionNodes[i];
					Vector2 mid = (a.position + b.position) * 0.5f;
					Vector2 normal = new Vector2(b.position.y - a.position.y, a.position.x - b.position.x).normalized;
					Gizmos.DrawLine(mid, mid + normal);
				}
			}
		}

		private void DrawCollisionInfoList()
		{
			for (int i = 0; i < _collisionInfoIndex; ++i)
			{
				DrawCollisionInfo(_collisionInfoList[i]);
			}
		}

		private void DrawCollisionInfo(CollisionInfo info)
		{
			if (info == null)
			{
				return;
			}

			Gizmos.color = nodeColor * 0.8f;
			Gizmos.DrawCube(info.penetratedNode.position, DEBUG_CUBE_M);
			Gizmos.DrawLine(info.penetratedNode.position, info.pn);

			Gizmos.color = edgeColor * 0.8f;
			Gizmos.DrawCube(info.edgeNodeA.position, DEBUG_CUBE_M);
			Gizmos.DrawLine(info.edgeNodeA.position, info.pa);
			Gizmos.DrawCube(info.edgeNodeB.position, DEBUG_CUBE_M);
			Gizmos.DrawLine(info.edgeNodeB.position, info.pb);

			Gizmos.color = Color.cyan;
			Vector2 mid = (info.edgeNodeA.position + info.edgeNodeB.position) * 0.5f;
			Gizmos.DrawLine(mid, mid + info.axis);

			Gizmos.DrawLine(info.edgeNodeA.position, info.edgeNodeA.prevPosition);
			Gizmos.DrawLine(info.edgeNodeB.position, info.edgeNodeB.prevPosition);
			Gizmos.DrawLine(info.penetratedNode.position, info.penetratedNode.prevPosition);
		}

#endif

        #endregion
    }
}