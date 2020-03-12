using UnityEngine;
using System.Collections.Generic;

namespace SBP
{
    /// <summary>
    /// 複数のNodeで構成された凸包
    /// </summary>
    public class Convex
    {

        /// <summary>
        /// 凸包の衝突判定用の境界を構築するノードの番号
        /// </summary>
        public List<int> indices;

        /// <summary>
        /// 凸包を完全に包含する矩形領域
        /// </summary>
        public AABB aabb = new AABB();

        /// <summary>
        /// 凸包が静的であるかどうか。
        /// </summary>
        public bool isStatic = false;

        /// <summary>
        /// 静的な場合、ノードの座標はこの値で上書きされる。
        /// </summary>
        public List<Vector2> nodePositions;

        /*
        public void SetNodes(List<int> nodes)
        {
            this.indices = nodes;
            RecalculateBounds();
        }

        public void SetStatic(bool isStatic)
        {
            this.isStatic = isStatic;
            if (!isStatic)
            {
                return;
            }

            // nodePositions = new List<Vector2>();
            // for (int i = 0; i < nodes.Count; ++i)
            // {
            //     nodePositions.Add(nodes[i].position);
            // }
        }

        public void RecalculateBounds()
        {
            // for (int i = 0; i < nodes.Count; ++i)
            // {
            //    aabb.Encapsulate(nodes[i].position);
            // }
        }
        */
    }
}