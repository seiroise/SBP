using UnityEngine;

namespace SBP
{
    public static class ConvexUtilities
    {
        public static readonly Vector3 NODE_SIZE_ON_GIZMOS = new Vector3(0.1f, 0.1f, 0f);

        public static void PreviewConvexOnGizmos(SBPConvexData convexData, Vector2 translation, float rotation, Vector2 scale, Color nodeColor, Color edgeColor)
        {
            if (!convexData)
            {
                return;
            }

            Gizmos.color = edgeColor;
            for (int i = 0; i < convexData.edges.Length; ++i)
            {
                SBPNodeData n1 = convexData.nodes[convexData.edges[i].aIdx];
                SBPNodeData n2 = convexData.nodes[convexData.edges[i].bIdx];
                Vector2 p1 = Utilities.RotateVector(n1.position, rotation) * scale + translation;
                Vector2 p2 = Utilities.RotateVector(n2.position, rotation) * scale + translation;
                Gizmos.DrawLine(p1, p2);
            }

            Gizmos.color = nodeColor;
            for (int i = 0; i < convexData.nodes.Length; ++i)
            {
                SBPNodeData n = convexData.nodes[i];
                Vector2 p = Utilities.RotateVector(n.position, rotation) * scale + translation;
                Gizmos.DrawCube(p, NODE_SIZE_ON_GIZMOS);
            }
        }
    }
}