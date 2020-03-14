using UnityEngine;
using System;
using System.Collections.Generic;

namespace SBP
{
    /// <summary>
    /// 複数のNodeで構成された凸包
    /// </summary>
    [Serializable]
    public class Convex
    {

        /// <summary>
        /// 凸包の衝突判定を構成するノードのリスト
        /// </summary>
		public List<Node> collisionNodes;

        /// <summary>
        /// コリジョン判定以外のノードのリスト
        /// </summary>
        public List<Node> helperNodes;

        /// <summary>
        /// 凸包を完全に包含する矩形領域
        /// </summary>
        public AABB aabb = new AABB();

        /// <summary>
        /// 凸包が静的であるかどうか。
        /// </summary>
        public bool isStatic = false;

        public Convex()
        {
            collisionNodes = new List<Node>();
            helperNodes = new List<Node>();
        }

        public void AddCollisionNode(Node n)
        {
            collisionNodes.Add(n);
        }

        public void AddHelperNode(Node n)
        {
            helperNodes.Add(n);
        }

        public void RecalculateBounds()
        {
            for (int i = 0; i < collisionNodes.Count; ++i)
            {
                aabb.Encapsulate(collisionNodes[i].position);
            }
        }

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
		*/
    }
}