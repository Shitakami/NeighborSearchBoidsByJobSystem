using Boids.Job;
using Boids.Settings;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

namespace Boids
{
    public class NeighborSearchBoidsSimulator
    {
        private readonly NeighborSearchBoidsSetting _neighborSearchBoidsSetting;

        public NeighborSearchBoidsSimulator(NeighborSearchBoidsSetting boidsSetting)
        {
            _neighborSearchBoidsSetting = boidsSetting;
        }

        public void Calculate(ref NativeArray<Matrix4x4> instanceMatricesArray, BoidsData[] boidsDatas)
        {
            var instanceCount = boidsDatas.Length;
            var boidsDataArray = new NativeArray<BoidsData>(instanceCount, Allocator.TempJob);
            var gridMultiHashMap = new NativeMultiHashMap<int3, int>(instanceCount, Allocator.TempJob);

            boidsDataArray.CopyFrom(boidsDatas);

            var registerInstanceToGridJob = new RegisterNeighborSearchGridJob
            (
                gridMultiHashMap.AsParallelWriter(),
                boidsDataArray,
                _neighborSearchBoidsSetting.MinNeighborSearchGridPoint,
                _neighborSearchBoidsSetting.NeighborSearchGridScale,
                _neighborSearchBoidsSetting.NeighborSearchGridCount
            );

            var registerInstanceToGridHandler = registerInstanceToGridJob.Schedule(instanceCount, 0);
            registerInstanceToGridHandler.Complete();
            
            var boidsSteerArray = new NativeArray<float3>(instanceCount, Allocator.TempJob);
            
            var boidsJob = new NeighborSearchBoidsSimulatorJob
            (
                _neighborSearchBoidsSetting.CohesionWeight,
                _neighborSearchBoidsSetting.CohesionAffectedRadiusSqr,
                _neighborSearchBoidsSetting.SeparateWeight,
                _neighborSearchBoidsSetting.SeparateAffectedRadiusSqr,
                _neighborSearchBoidsSetting.AlignmentWeight,
                _neighborSearchBoidsSetting.AlignmentAffectedRadiusSqr,
                _neighborSearchBoidsSetting.MaxSpeed,
                _neighborSearchBoidsSetting.MaxSteerForce,
                gridMultiHashMap,
                _neighborSearchBoidsSetting.NeighborSearchGridScale,
                _neighborSearchBoidsSetting.NeighborSearchGridCount,
                _neighborSearchBoidsSetting.MinNeighborSearchGridPoint,
                boidsDataArray,
                boidsSteerArray
            );

            var boidsJobHandler = boidsJob.Schedule(instanceCount, 0);
            boidsJobHandler.Complete();

            gridMultiHashMap.Dispose();

            var applySteerForce = new ApplySteerForceJob
            (
                boidsDataArray,
                boidsSteerArray,
                instanceMatricesArray,
                _neighborSearchBoidsSetting.SimulationAreaCenter,
                _neighborSearchBoidsSetting.SimulationAreaScaleHalf,
                _neighborSearchBoidsSetting.AvoidSimulationAreaWeight,
                Time.deltaTime,
                _neighborSearchBoidsSetting.MaxSpeed,
                _neighborSearchBoidsSetting.InstanceScale
            );

            var applySteerForceHandler = applySteerForce.Schedule(instanceCount, 0);
            applySteerForceHandler.Complete();
            
            boidsDataArray.CopyTo(boidsDatas);

            boidsDataArray.Dispose();
            boidsSteerArray.Dispose();
        }
    }
}