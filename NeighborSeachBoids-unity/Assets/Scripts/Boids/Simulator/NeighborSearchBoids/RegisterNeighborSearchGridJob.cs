using Boids.Mathematics;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace Boids.Job
{
    internal struct RegisterNeighborSearchGridJob : IJobParallelFor
    {
        [WriteOnly] private NativeMultiHashMap<int3, int>.ParallelWriter _gridWriter;
        [ReadOnly] private readonly NativeArray<BoidsData> _boidsDatasRead;
        [ReadOnly] private readonly float3 _minGridPoint;
        [ReadOnly] private readonly float _gridScale;
        [ReadOnly] private readonly int3 _gridCount;
        
        public RegisterNeighborSearchGridJob(
            NativeMultiHashMap<int3, int>.ParallelWriter gridWriter,
            NativeArray<BoidsData> boidsDatasRead,
            float3 minGridPoint,
            float gridScale,
            int3 gridCount)
        {
            _gridWriter = gridWriter;
            _boidsDatasRead = boidsDatasRead;
            _minGridPoint = minGridPoint;
            _gridScale = gridScale;
            _gridCount = gridCount;
        }

        public void Execute(int index)
        {
            var boidsDataPosition = _boidsDatasRead[index].Position;

            var gridIndex = MathematicsUtility.CalculateGridIndex(boidsDataPosition, _minGridPoint, _gridScale, _gridCount);

            _gridWriter.Add(gridIndex, index);
        }
    }
}