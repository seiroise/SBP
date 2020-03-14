using UnityEngine;

namespace SBP
{
    public class ConvexSpawner : MonoBehaviour
    {
        public SBPSimulator simulator;

        public float size = 1f;
        public bool syncPosition = false;

        private Convex _mine = null;

        private void Start()
        {
            if (simulator)
            {
                _mine = simulator.AddTriangle(transform.position, size);
            }
        }

        private void Update()
        {
            if (_mine != null && syncPosition)
            {
                _mine.helperNodes[0].position = transform.position;
            }
        }


    }
}