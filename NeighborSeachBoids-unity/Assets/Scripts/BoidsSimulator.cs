using System;
using Boids;
using Boids.Settings;
using Unity.Collections;
using UnityEngine;
using UnityEngine.Rendering;

namespace BoidsSimulator
{
    public class BoidsSimulator : MonoBehaviour
    {
        private enum BoidsSimulationType
        {
            AllSearch,
            NeighborSearch,
        }

        [Header("シミュレーション法")]
        [SerializeField] private BoidsSimulationType _boidsSimulationType;

        [SerializeField] private int _instanceCount;

        [Header("描画関係")]
        [SerializeField] private Mesh _mesh;
        [SerializeField] private Material _material;
        
        private BoidsData[] _boidsDatas;
        private RenderParams _renderParams;

        [Header("AllSearchSetting")]
        [SerializeField] private AllSearchBoidsSetting _allSearchBoidsSetting;

        [Header("NeighborSearchSetting")]
        [SerializeField] private NeighborSearchBoidsSetting _neighborSearchBoidsSetting;

        private void Start()
        {
            InitializeBoidsInstance();
            _renderParams = new RenderParams(_material) { receiveShadows = true, shadowCastingMode = ShadowCastingMode.On };
        }

        private void Update()
        {
            switch (_boidsSimulationType)
            {
                case BoidsSimulationType.AllSearch:
                    UpdateBoidsByAllSearch();
                    break;
                case BoidsSimulationType.NeighborSearch:
                    UpdateBoidsByNeighborSearch();
                    break;
                default:
                    throw new ArgumentOutOfRangeException();
            }
        }

        private void UpdateBoidsByNeighborSearch()
        {
            var matricesArray = new NativeArray<Matrix4x4>(_instanceCount, Allocator.TempJob);

            var neighborSearchBoidsSimulator = new NeighborSearchBoidsSimulator(_neighborSearchBoidsSetting);
            neighborSearchBoidsSimulator.Calculate(matricesArray, _boidsDatas);
            
            RendererUtility.InstanceRenderUtility.DrawAll(_mesh, _renderParams, matricesArray);

            matricesArray.Dispose();
        }

        private void UpdateBoidsByAllSearch()
        {
            var allSearchBoidsSimulator = new AllSearchBoidsSimulator(_allSearchBoidsSetting);
            var matricesArray = new NativeArray<Matrix4x4>(_instanceCount, Allocator.TempJob);
            
            allSearchBoidsSimulator.Calculate(matricesArray, _boidsDatas);

            RendererUtility.InstanceRenderUtility.DrawAll(_mesh, _renderParams, matricesArray);
            
            matricesArray.Dispose();
        }

        private void InitializeBoidsInstance()
        {
            _boidsDatas = new BoidsData[_instanceCount];

            BoidsInitializer.In(_boidsDatas, _allSearchBoidsSetting.SimulationAreaCenter, _allSearchBoidsSetting.SimulationAreaScale, 0.1f);
        }
    }
}