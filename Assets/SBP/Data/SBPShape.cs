using UnityEngine;

namespace SBP
{
    [CreateAssetMenu(menuName = "SBP/Shape", fileName = "SBPShape")]
    public class SBPShape : ScriptableObject
    {

        /// <summary>
        /// 形状を構成するノード
        /// </summary>
        public Node[] nodes;

        /// <summary>
        /// ノード同士の接続情報、２つで1セット
		/// edgeIndices[2n]とedgeIndices[2n + 1]でエッジが作成される。	
        /// </summary>
        public int[] edgeIndices;

        /// <summary>
        /// 補助用のノードの開始インデックス
        /// helperNodeOffset = 5ならば0-4が衝突判定用、5~が補助用のノードとなる。
        /// </summary>
        public int helperNodeOffset;
    }
}