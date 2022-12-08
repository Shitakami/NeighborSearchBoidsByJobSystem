using Boids.Mathematics;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEditor.U2D;

namespace Boids.Job
{
    [BurstCompile]
    internal struct AllSearchBoidsSimulatorJob : IJobParallelFor
    {
        [ReadOnly] private readonly float _cohesionWeight;
        [ReadOnly] private readonly float _cohesionAffectedRadiusSqr;
        [ReadOnly] private readonly float _separateWeight;
        [ReadOnly] private readonly float _separateAffectedRadiusSqr;
        [ReadOnly] private readonly float _alignmentWeight;
        [ReadOnly] private readonly float _alignmentAffectedRadiusSqr;

        [ReadOnly] private readonly float _maxSpeed;
        [ReadOnly] private readonly float _maxForceSteer;

        [ReadOnly] private readonly NativeArray<BoidsData> _boidsDatasRead;
        [WriteOnly] private NativeArray<float3> _boidsSteerWrite;

        public AllSearchBoidsSimulatorJob(
            float cohesionWeight,
            float cohesionAffectedRadiusSqr,
            float separateWeight,
            float separateAffectedRadiusSqr,
            float alignmentWeight,
            float alignmentAffectedRadiusSqr,
            float maxSpeed,
            float maxForceSteer,
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

            for (int targetIndex = 0; targetIndex < _boidsDatasRead.Length; ++targetIndex)
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