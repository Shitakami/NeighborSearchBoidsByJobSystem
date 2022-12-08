using System.Text;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;

namespace Boids.Debugger
{
    internal static class DebugUtility
    {
        public static void ShowBoidsDataArray(NativeArray<BoidsData> boidsDatas)
        {
            var sb = new StringBuilder();
            foreach (var t in boidsDatas)
            {
                sb.AppendLine($"Position:{t.Position}, Velocity:{t.Velocity}");
            }
            
            Debug.Log(sb);
        }
        
        private static void ShowNeighborInstances(
            int3 gridCount,
            NativeMultiHashMap<int3, int> grid, 
            float3 simulationAreaCenter, 
            float gridScale,
            BoidsData[] boidsDatas,
            float cohesionAffectedRadiusSqr)
        {
            var ownIndex = 0;
            var ownPosition = boidsDatas[ownIndex].Position;
            
            var origin = new int3(
                (int) math.clamp((ownPosition.x - simulationAreaCenter.x) / gridScale, 0, gridCount.x),
                (int) math.clamp((ownPosition.y - simulationAreaCenter.y) / gridScale, 0, gridCount.y),
                (int) math.clamp((ownPosition.z - simulationAreaCenter.z) / gridScale, 0, gridCount.z)
            );

            int minX = origin.x - 1 < 0 ? 0 : origin.x - 1;
            int minY = origin.y - 1 < 0 ? 0 : origin.y - 1;
            int minZ = origin.z - 1 < 0 ? 0 : origin.z - 1;

            int maxX = origin.x + 1 >= gridCount.x ? origin.x : origin.x + 1;
            int maxY = origin.y + 1 >= gridCount.y ? origin.y : origin.y + 1;
            int maxZ = origin.z + 1 >= gridCount.z ? origin.z : origin.z + 1;
            
            var instances = new StringBuilder();
            var inInstances = new StringBuilder("\nin\n");
            var outInstances = new StringBuilder("\nout\n");
            
            for (int x = minX; x <= maxX; ++x)
            for (int y = minY; y <= maxY; ++y)
            for (int z = minZ; z <= maxZ; ++z)
            {
                var key = new int3(x, y, z);

                for (var success = grid.TryGetFirstValue(key, out var targetIndex, out var iterator);
                     success;
                     success = grid.TryGetNextValue(out targetIndex, ref iterator))
                {
                    var targetPosition = boidsDatas[targetIndex].Position;
                    var diff = ownPosition - targetPosition;
                    var distance = math.distance(ownPosition, targetPosition);

                    instances.AppendLine($"{targetIndex} : distance = {distance}");

                    var distanceSqr = math.dot(diff, diff);

                    if (distanceSqr < cohesionAffectedRadiusSqr)
                    {
                        inInstances.AppendLine($"{targetIndex}");
                    }
                    else
                    {
                        outInstances.AppendLine($"{targetIndex}");
                    }
                }
            }
            
            Debug.Log(instances.ToString() + inInstances + outInstances);
        }
        
        private static void ShowNativeParallelMultiHashMap(NativeMultiHashMap<int3, int> hashMap, int3 indexes, int neighborSearchGridCount)
        {
            var maxIndex = neighborSearchGridCount;

            var sb = new StringBuilder();
            
            // for (int x = 0; x <= maxIndex.x; ++x)
            // for (int y = 0; y <= maxIndex.y; ++y)
            // for (int z = 0; z <= maxIndex.z; ++z)
            // {
            // var key = new int3(x, y, z);
            var key = indexes;

            var subSb = new StringBuilder();

            for (var success = hashMap.TryGetFirstValue(key, out var targetIndex, out var iterator);
                 success;
                 success = hashMap.TryGetNextValue(out targetIndex, ref iterator))
            {
                subSb.AppendLine(targetIndex.ToString());
            }

            if (subSb.Length != 0)
            {
                sb.AppendLine($"{key}:\n" + subSb);
            }
            // }

            Debug.Log(sb);
        }
    }
}