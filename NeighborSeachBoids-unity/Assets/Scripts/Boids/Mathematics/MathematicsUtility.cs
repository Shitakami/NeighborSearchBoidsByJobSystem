using Unity.Mathematics;

namespace Boids.Mathematics
{
    internal static class MathematicsUtility
    {
        internal static float3 Limit(float3 vec, float max)
        {
            var lenght = math.sqrt(math.dot(vec, vec));
            return lenght > max
                ? vec * (max / lenght)
                : vec;
        }

        internal static int3 CalculateGridIndex(float3 position, float3 minGridPoint, float gridScale, int3 gridCount)
        {
            // MEMO: 範囲外のものは範囲内のGridに収める
            return new int3(
                (int) math.clamp((position.x - minGridPoint.x) / gridScale, 0, gridCount.x),
                (int) math.clamp((position.y - minGridPoint.y) / gridScale, 0, gridCount.y),
                (int) math.clamp((position.z - minGridPoint.z) / gridScale, 0, gridCount.z)
            );
        }
    }
}