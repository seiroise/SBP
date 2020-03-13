using UnityEngine;

namespace SBP
{
    public class CollisionInfo
    {
        public Convex a = null;
        public Convex b = null;
		public float depth = 1e+8f;
		public Node node = null;
		public Vector2 axis;
		public Node edgeA = null;
		public Node edgeB = null;
    }
}