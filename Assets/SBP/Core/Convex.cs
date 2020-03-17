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
        /// エッジ
        /// </summary>
        public List<Edge> edges;

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

        public static Convex LoadFromConvexData(SBPConvexData data, Vector2 translation, float rotation, Vector2 scale)
        {
            if (data == null)
            {
                return null;
            }

            // ノードを作成
            List<Node> nodes = new List<Node>();
            for (int i = 0; i < data.nodes.Length; ++i)
            {
                SBPNodeData nodeData = data.nodes[i];
                Vector2 position = nodeData.position;
                position *= scale;
                position = Utilities.RotateVector(position, rotation);
                position += translation;

                Node n = new Node(position, nodeData.mass, nodeData.damping);
                nodes.Add(n);
            }

            // エッジを作成
            List<Edge> edges = new List<Edge>();
            for (int i = 0; i < data.edges.Length; ++i)
            {
                SBPEdgeData edgeData = data.edges[i];
                Edge e = new Edge(nodes[edgeData.aIdx], nodes[edgeData.bIdx]);
                edges.Add(e);
            }

            // 凸包を作成
            Convex convex = new Convex();
            int offset = data.helperNodeOffset;

            for (int i = 0; i < nodes.Count; ++i)
            {
                if (i < offset)
                {
                    convex.AddCollisionNode(nodes[i]);
                }
                else
                {
                    convex.AddHelperNode(nodes[i]);
                }
            }

            convex.edges = edges;
            convex.RecalculateBounds();
            return convex;
        }
    }
}