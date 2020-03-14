using UnityEngine;

namespace SBP
{
    /// <summary>
    /// ２つの凸包間のSATの結果を格納する。
    /// </summary>
    public struct SATResult
    {
        /// <summary>
        /// 分離線に平行な凸包の辺を構成するノードA
        /// </summary>
        public Node edgeNodeA;

        /// <summary>
        /// 分離線に平行な凸包の辺を構成するノードB
        /// </summary>
        public Node edgeNodeB;

        /// <summary>
        /// めり込み度合い
        /// </summary>
        public float penetration;

        /// <summary>
        /// 分離軸
        /// </summary>
        public Vector2 axis;

        /// <summary>
        /// 最も浅い位置に存在するノード
        /// </summary>
        public Node penetratedNode;
    }
}