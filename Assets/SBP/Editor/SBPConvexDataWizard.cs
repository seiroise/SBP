using UnityEngine;
using UnityEditor;

namespace SBP
{
    public class SBPConvexDataWizard : ScriptableWizard
    {
        public enum InnerEdgeConstructionType
        {
            None,
            ConnectWithCenter,
        }

        public string path;
        [Range(3, 24)]
        public int corner = 3;
        public bool addCenterNode = true;

        public static void SetPolygon(ref SBPConvexData data, int corner, float size, float startAngle, InnerEdgeConstructionType innerEdgeConstructionType)
        {
            if (data == null)
            {
                return;
            }

            corner = Mathf.Max(corner, 3);
            size = Mathf.Max(size, 1e-3f);

            if (corner % 2 == 0)
            {
                // 偶数角の場合
                int h = corner / 2;

                data.nodes = new SBPNodeData[corner];
                data.edges = new SBPEdgeData[corner + h];

                float angleDelta = 360f / corner;

                for (int i = 0; i < corner; ++i)
                {
                    float angle = angleDelta * i + startAngle;
                    data.nodes[i] = new SBPNodeData(Utilities.GetAngledVectorWithDegree(angle));
                }

                for (int i = 0, j = corner - 1; i < corner; j = i++)
                {
                    data.edges[i] = new SBPEdgeData(i, j);
                }
                for (int i = 0; i < h; ++i)
                {
                    data.edges[corner + i] = new SBPEdgeData(i, i + h);
                }
                data.helperNodeOffset = corner;
            }
            else
            {
                // 奇数角の場合は中心点が必要
                data.nodes = new SBPNodeData[corner + 1];
                data.edges = new SBPEdgeData[corner * 2];

                float angleDelta = 360f / corner;

                for (int i = 0; i < corner; ++i)
                {
                    float angle = angleDelta * i + startAngle;
                    data.nodes[i] = new SBPNodeData(Utilities.GetAngledVectorWithDegree(angle));
                }
                data.nodes[corner] = new SBPNodeData(Vector2.zero);

                for (int i = 0, j = corner - 1; i < corner; j = i++)
                {
                    data.edges[i * 2] = new SBPEdgeData(i, j);
                    data.edges[i * 2 + 1] = new SBPEdgeData(i, corner);
                }
                data.helperNodeOffset = corner;
            }
        }

        [MenuItem("Window/SBP Convex Data Wizard")]
        private static void Open()
        {
            SBPConvexDataWizard wiz = DisplayWizard<SBPConvexDataWizard>("SBP Convex Data Wizard");
            wiz.path = GetFocusFolderPath();
        }

        private void OnWizardCreate()
        {
            SBPConvexData obj = SBPConvexData.CreateInstance("SBPConvexData") as SBPConvexData;
            SetPolygon(ref obj, corner, 1f, 0f, InnerEdgeConstructionType.ConnectWithCenter);
            // SetTriangle(ref obj, 1f, 0f);
            AssetDatabase.CreateAsset(obj, path + "/SBPConvexData.asset");
        }

        private static string GetFocusFolderPath()
        {
            foreach (Object obj in Selection.GetFiltered(typeof(DefaultAsset), SelectionMode.DeepAssets))
            {
                if (obj is DefaultAsset)
                {
                    string path = AssetDatabase.GetAssetPath(obj);
                    if (AssetDatabase.IsValidFolder(path))
                    {
                        return path;
                    }
                }
            }
            return "Assets";
        }
    }
}