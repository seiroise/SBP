using UnityEngine;
using System;

namespace SBP
{
    /// <summary>
    /// 保存用のノードのデータ
    /// </summary>
    [Serializable]
    public struct SBPNodeData
    {
        public Vector2 position;
        public float mass;
        public float damping;

        public SBPNodeData(Vector2 position, float mass = 1f, float damping = 0f)
        {
            this.position = position;
            this.mass = mass;
            this.damping = damping;
        }
    }

    /// <summary>
    /// 保存用のエッジのデータ
    /// </summary>
    [Serializable]
    public struct SBPEdgeData
    {
        public int aIdx;
        public int bIdx;

        public SBPEdgeData(int aIdx, int bIdx)
        {
            this.aIdx = aIdx;
            this.bIdx = bIdx;
        }
    }

    [CreateAssetMenu(menuName = "SBP/Convex", fileName = "SBPConvexData")]
    public class SBPConvexData : ScriptableObject
    {

        /// <summary>
        /// 形状を構成するノード
        /// </summary>
        public SBPNodeData[] nodes;

        /// <summary>
        /// ノード同士の接続情報	
        /// </summary>
        public SBPEdgeData[] edges;

        /// <summary>
        /// 補助用のノードの開始インデックス
        /// helperNodeOffset = 5ならば0-4が衝突判定用、5~が補助用のノードとなる。
        /// </summary>
        public int helperNodeOffset;
    }
}