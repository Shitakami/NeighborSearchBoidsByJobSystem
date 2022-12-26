using Boids.Job;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using Random = UnityEngine.Random;

namespace Boids
{
    public static class BoidsInitializer
    {
        public static void In(BoidsData[] boidsDatas, Vector3 simulationAreaCenter, Vector3 simulationAreaScale, float initializeVelocity)
        {
            for (var i = 0; i < boidsDatas.Length; ++i)
            {
                var randPosition = Random.insideUnitSphere;
                boidsDatas[i].Position = new float3(
                    simulationAreaScale.x * randPosition.x + simulationAreaCenter.x,
                    simulationAreaScale.y * randPosition.y + simulationAreaCenter.y,
                    simulationAreaScale.z * randPosition.z + simulationAreaCenter.z
                );
                boidsDatas[i].Velocity = Random.insideUnitSphere * initializeVelocity;
            }
            
            // MEMO: JobSystemで初期化するようにしたかった。
            // が、そうすると RenderMeshInstanced で Invalid AABB とエラーが発生し、正しくインスタンスが表示されなくなった。
            // 一旦問題は解決せずに、MainThreadで実行する
            //
            // var instanceCount = boidsDatas.Length;
            // var boidsDatasArray = new NativeArray<BoidsData>(instanceCount, Allocator.TempJob);
            //
            // var boidsInitializeJob = new BoidsInitializeJob
            // {
            //     BoidsDatasWrite = boidsDatasArray,
            //     SimulationAreaCenter = simulationAreaCenter,
            //     SimulationAreaScale = simulationAreaScale,
            //     InitializeVelocity = initializeVelocity,
            //     Random = new Unity.Mathematics.Random((uint)Random.Range(1, 10000))
            // };
            //
            // var boidsInitializeJobHandler = boidsInitializeJob.Schedule(instanceCount, 0);
            // boidsInitializeJobHandler.Complete();
            //
            // boidsDatasArray.CopyTo(boidsDatas);
            // boidsDatasArray.Dispose();
        }
    }
}