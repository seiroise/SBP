using UnityEngine;

namespace SBP
{
	public class ConvexSpawner : MonoBehaviour
	{
		public SBPSimulator simulator;

		public float size = 1f;

		private void Start()
		{
			if (simulator)
			{
				simulator.AddTriangle(transform.position, size);
			}
		}
	}
}