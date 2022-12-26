using Boids.Job;
using Boids.Settings;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

namespace Boids
{
    public class AllSearchBoidsSimulator
    {
        private readonly AllSearchBoidsSetting _allSearchBoidsSetting;

        public AllSearchBoidsSimulator(AllSearchBoidsSetting boidsSetting)
        {
            _allSearchBoidsSetting = boidsSetting;
        }

        public void Calculate(NativeArray<Matrix4x4> instanceMatricesArray, BoidsData[] boidsDatas)
        {
            var instanceCount = boidsDatas.Length;
            var boidsDataArray = new NativeArray<BoidsData>(instanceCount, Allocator.TempJob);
            var boidsSteerArray = new NativeArray<float3>(instanceCount, Allocator.TempJob);

            boidsDataArray.CopyFrom(boidsDatas);
            
            var boidsJob = new AllSearchBoidsSimulatorJob
            (
                _allSearchBoidsSetting.CohesionWeight,
                _allSearchBoidsSetting.CohesionAffectedRadiusSqr,
                _allSearchBoidsSetting.SeparateWeight,
                _allSearchBoidsSetting.SeparateAffectedRadiusSqr,
                _allSearchBoidsSetting.AlignmentWeight,
                _allSearchBoidsSetting.AlignmentAffectedRadiusSqr,
                _allSearchBoidsSetting.MaxSpeed,
                _allSearchBoidsSetting.MaxSteerForce,
                boidsDataArray,
                boidsSteerArray
            );

            var boidsJobHandler = boidsJob.Schedule(instanceCount, 0);
            boidsJobHandler.Complete();

            var applySteerForce = new ApplySteerForceJob
            (
                boidsDataArray,
                boidsSteerArray,
                instanceMatricesArray,
                _allSearchBoidsSetting.SimulationAreaCenter,
                _allSearchBoidsSetting.SimulationAreaScale,
                _allSearchBoidsSetting.AvoidSimulationAreaWeight,
                Time.deltaTime,
                _allSearchBoidsSetting.MaxSpeed,
                _allSearchBoidsSetting.InstanceScale
            );

            var applySteerForceHandler = applySteerForce.Schedule(instanceCount, 0);
            applySteerForceHandler.Complete();
            
            boidsDataArray.CopyTo(boidsDatas);

            boidsDataArray.Dispose();
            boidsSteerArray.Dispose();
        }
    }
}