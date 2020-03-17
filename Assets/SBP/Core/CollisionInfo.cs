using UnityEngine;

namespace SBP
{
    /// <summary>
    /// コリジョンについての情報
    /// </summary>
    public class CollisionInfo
    {
        /// <summary>
        /// 衝突判定のある凸包A
        /// </summary>
        public Convex convexA = null;

        /// <summary>
        /// 凸包Aの衝突判定のある辺を構成するノードA
        /// </summary>
        public Node edgeNodeA = null;

        /// <summary>
        /// 凸包Aの衝突判定のある辺を構成するノードB
        /// </summary>
        public Node edgeNodeB = null;

        /// <summary>
        /// 衝突判定のある凸包B
        /// </summary>
        public Convex convexB = null;

        /// <summary>
        /// 凸包Aにめり込んでいる凸包Bのノード
        /// </summary>
        public Node penetratedNode = null;

        /// <summary>
        ///ノードの
        /// </summary>
        public float depth = 1e+8f;

        /// <summary>
        /// 分離軸
        /// </summary>
        public Vector2 axis;

        public Vector2 pa;
        public Vector2 pb;
        public Vector2 pn;

        public void Reset()
        {
            convexA = null;
            convexB = null;
            edgeNodeA = null;
            edgeNodeB = null;
            penetratedNode = null;
            depth = 1e+8f;
        }
    }
}