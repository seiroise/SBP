using UnityEngine;

namespace SBP
{
	public static class Utilities
	{

		/// <summary>
		/// 線分abに対して線分apが時計回りの方向に存在する場合はtrueを返す。
		/// 点pが点a, bを通る直線上に存在する場合もtrueを返す。
		/// </summary>
		/// <param name="p"></param>
		/// <param name="a"></param>
		/// <param name="b"></param>
		/// <returns></returns>
		public static bool IsClockwise(Vector2 p, Vector2 a, Vector2 b)
		{
			Vector2 d1 = b - a;
			Vector2 d2 = p - a;
			float t = d1.x * d2.y - d2.x * d1.y;
			return t <= 0f;
		}

		/// <summary>
		/// 線分abと点pの最短距離を求める。
		/// 点a = 点bの場合ゼロ除算が発生する。
		/// </summary>
		/// <param name="p"></param>
		/// <param name="a"></param>
		/// <param name="b"></param>
		/// <returns></returns>
		public static float DistanceToSegment(Vector2 p, Vector2 a, Vector2 b)
		{
			Vector2 d1 = b - a;
			Vector2 d2 = p - a;
			float l = d1.magnitude;
			Vector2 d3 = d1 / l;
			float t = Mathf.Clamp(Vector2.Dot(d3, d2), 0f, l);
			return Vector2.Distance(d3 * t, d2);
		}

		/// <summary>
		/// 指定した角度(radian)を向いた単位ベクトルを返す
		/// </summary>
		/// <param name="radian"></param>
		/// <returns></returns>
		public static Vector2 GetAngledVector(float radian)
		{
			return new Vector2(Mathf.Cos(radian), Mathf.Sin(radian));
		}
	}
}