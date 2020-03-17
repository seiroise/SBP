using UnityEngine;

namespace SBP
{
    public class ConvexSpawner : MonoBehaviour
    {
        public SBPSimulator simulator;

        public SBPConvexData convexData;

        public float size = 1f;
        public bool syncPosition = false;

        private Convex _mine = null;

        private void Start()
        {
            if (simulator && convexData)
            {
                Convex convex = Convex.LoadFromConvexData(convexData, transform.position, transform.eulerAngles.z * Mathf.Deg2Rad, transform.localScale);
                _mine = simulator.AddConvex(convex, true, true);
                // _mine = simulator.AddTriangle(transform.position, size);
            }
        }

        private void Update()
        {
            if (_mine != null && syncPosition)
            {
                _mine.helperNodes[0].position = transform.position;
            }
        }

        private void OnDrawGizmos()
        {
            if (Application.isPlaying)
            {
                return;
            }

            if (convexData)
            {
                ConvexUtilities.PreviewConvexOnGizmos(convexData, transform.position, transform.eulerAngles.z * Mathf.Deg2Rad, transform.localScale, Color.green, Color.cyan);
            }
        }

    }
}