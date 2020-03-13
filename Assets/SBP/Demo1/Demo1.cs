using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace SBP
{
	public class Demo1 : MonoBehaviour
	{
		public Transform a;
		public Transform b;
		public Transform p;

		public bool sign = false;
		public float dist = 0f;

		private void Update()
		{
			if (a && b && p)
			{
				sign = Utilities.IsClockwise(p.position, a.position, b.position);
				dist = Utilities.DistanceToSegment(p.position, a.position, b.position);
			}
		}

		private void OnDrawGizmos()
		{
			if (a && b)
			{
				Gizmos.DrawLine(a.position, b.position);
			}
			if (a && p)
			{
				Gizmos.DrawLine(a.position, p.position);
			}
		}
	}
}