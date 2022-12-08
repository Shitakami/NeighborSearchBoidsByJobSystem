using Boids.Mathematics;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace Boids.Job
{
    [BurstCompile]
    internal struct NeighborSearchBoidsSimulatorJob : IJobParallelFor
    {
        [ReadOnly] private readonly float _cohesionWeight;
        [ReadOnly] private readonly float _cohesionAffectedRadiusSqr;
        [ReadOnly] private readonly float _separateWeight;
        [ReadOnly] private readonly float _separateAffectedRadiusSqr;
        [ReadOnly] private readonly float _alignmentWeight;
        [ReadOnly] private readonly float _alignmentAffectedRadiusSqr;

        [ReadOnly] private readonly float _maxSpeed;
        [ReadOnly] private readonly float _maxForceSteer;

        [ReadOnly] private NativeMultiHashMap<int3, int> _grid;
        [ReadOnly] private readonly float _gridScale;
        [ReadOnly] private readonly int3 _gridCount;
        [ReadOnly] private readonly float3 _minGridPoint;

        [ReadOnly] private readonly NativeArray<BoidsData> _boidsDatasRead;
        [WriteOnly] private NativeArray<float3> _boidsSteerWrite;

        public NeighborSearchBoidsSimulatorJob(
            float cohesionWeight,
            float cohesionAffectedRadiusSqr,
            float separateWeight,
            float separateAffectedRadiusSqr,
            float alignmentWeight,
            float alignmentAffectedRadiusSqr,
            float maxSpeed,
            float maxForceSteer,
            NativeMultiHashMap<int3, int> grid,
            float gridScale,
            int3 gridCount,
            float3 minGridPoint,
            NativeArray<BoidsData> boidsDatasRead,
            NativeArray<float3> boidsSteerWrite
        )
        {
            _cohesionWeight = cohesionWeight;
            _cohesionAffectedRadiusSqr = cohesionAffectedRadiusSqr;
            _separateWeight = separateWeight;
            _separateAffectedRadiusSqr = separateAffectedRadiusSqr;
            _alignmentWeight = alignmentWeight;
            _alignmentAffectedRadiusSqr = alignmentAffectedRadiusSqr;
            _maxSpeed = maxSpeed;
            _maxForceSteer = maxForceSteer;
            _grid = grid;
            _gridScale = gridScale;
            _gridCount = gridCount;
            _minGridPoint = minGridPoint;
            _boidsDatasRead = boidsDatasRead;
            _boidsSteerWrite = boidsSteerWrite;
        }

        public void Execute(int ownIndex)
        {
            var ownPosition = _boidsDatasRead[ownIndex].Position;
            var ownVelocity = _boidsDatasRead[ownIndex].Velocity;

            var cohesionPositionSum = new float3();
            var cohesionTargetCount = 0;

            var separateRepluseSum = new float3();
            var separateTargetCount = 0;

            var alignmentVelocitySum = new float3();
            var alignmentTargetCount = 0;

            var gridIndex = MathematicsUtility.CalculateGridIndex(ownPosition, _minGridPoint, _gridScale, _gridCount);

            int minX = gridIndex.x - 1 < 0 ? 0 : gridIndex.x - 1;
            int minY = gridIndex.y - 1 < 0 ? 0 : gridIndex.y - 1;
            int minZ = gridIndex.z - 1 < 0 ? 0 : gridIndex.z - 1;

            int maxX = gridIndex.x + 1 >= _gridCount.x ? gridIndex.x : gridIndex.x + 1;
            int maxY = gridIndex.y + 1 >= _gridCount.y ? gridIndex.y : gridIndex.y + 1;
            int maxZ = gridIndex.z + 1 >= _gridCount.z ? gridIndex.z : gridIndex.z + 1;

            for (int x = minX; x <= maxX; ++x)
            for (int y = minY; y <= maxY; ++y)
            for (int z = minZ; z <= maxZ; ++z)
            {
                var key = new int3(x, y, z);

                for (var success = _grid.TryGetFirstValue(key, out var targetIndex, out var iterator);
                     success;
                     success = _grid.TryGetNextValue(out targetIndex, ref iterator))
                {
                    if (ownIndex == targetIndex)
                    {
                        continue;
                    }

                    var targetPosition = _boidsDatasRead[targetIndex].Position;
                    var targetVelocity = _boidsDatasRead[targetIndex].Velocity;

                    var diff = ownPosition - targetPosition;
                    var distanceSqr = math.dot(diff, diff);

                    if (distanceSqr <= _cohesionAffectedRadiusSqr)
                    {
                        cohesionPositionSum += targetPosition;
                        cohesionTargetCount++;
                    }

                    if (distanceSqr <= _separateAffectedRadiusSqr)
                    {
                        separateRepluseSum += math.normalize(diff) / math.sqrt(distanceSqr); // 距離に反比例する相手から自分への力
                        separateTargetCount++;
                    }

                    if (distanceSqr <= _alignmentAffectedRadiusSqr)
                    {
                        alignmentVelocitySum += targetVelocity;
                        alignmentTargetCount++;
                    }
                }
            }

            var cohesionSteer = new float3();
            if (cohesionTargetCount > 0)
            {
                var cohesionPositionAverage = cohesionPositionSum / cohesionTargetCount;
                var cohesionDirection = cohesionPositionAverage - ownPosition;
                var cohesionVelocity = math.normalize(cohesionDirection) * _maxSpeed;
                cohesionSteer = MathematicsUtility.Limit(cohesionVelocity - ownVelocity, _maxForceSteer);
            }

            var separateSteer = new float3();
            if (separateTargetCount > 0)
            {
                var separateRepulseAverage = separateRepluseSum / separateTargetCount;
                var separateVelocity = math.normalize(separateRepulseAverage) * _maxSpeed;
                separateSteer = MathematicsUtility.Limit(separateVelocity - ownVelocity, _maxForceSteer);
            }

            var alignmentSteer = new float3();
            if (alignmentTargetCount > 0)
            {
                var alignmentVelocityAverage = alignmentVelocitySum / alignmentTargetCount;
                var alignmentVelocity = math.normalize(alignmentVelocityAverage) * _maxSpeed;
                alignmentSteer = MathematicsUtility.Limit(alignmentVelocity - ownVelocity, _maxForceSteer);
            }

            _boidsSteerWrite[ownIndex] =
                cohesionSteer * _cohesionWeight +
                separateSteer * _separateWeight +
                alignmentSteer * _alignmentWeight;
        }
    }
}