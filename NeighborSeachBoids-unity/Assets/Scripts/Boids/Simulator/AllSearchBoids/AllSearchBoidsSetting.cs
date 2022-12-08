using Unity.Mathematics;
using UnityEngine;

namespace Boids.Settings
{
    [CreateAssetMenu(fileName = "AllSearchBoidsSetting", menuName = "Boids/AllSearchSetting")]
    public class AllSearchBoidsSetting : ScriptableObject
    {
        [Header("結合")]
        [SerializeField] private float _cohesionWeight;
        [SerializeField] private float _cohesionAffectedRadius;

        [Header("分離")]
        [SerializeField] private float _separationWeight;
        [SerializeField] private float _separationAffectedRadius;

        [Header("整列")]
        [SerializeField] private float _alignmentWeight;
        [SerializeField] private float _alignmentAffectedRadius;

        [Header("シミュレーション空間")]
        [SerializeField] private Vector3 _simulationAreaCenter;
        [SerializeField] private Vector3 _simulationAreaScale;
        [SerializeField] private float _avoidSimulationAreaWeight;

        [Space(20)]
        [SerializeField] private float _maxSpeed;
        [SerializeField] private float _maxSteerForce;

        [Header("個体のスケール")]
        [SerializeField] private float3 _instanceScale;

        public float CohesionWeight => _cohesionWeight;
        public float CohesionAffectedRadiusSqr => _cohesionAffectedRadius * _cohesionAffectedRadius;

        public float SeparateWeight => _separationWeight;
        public float SeparateAffectedRadiusSqr => _separationAffectedRadius * _separationAffectedRadius;

        public float AlignmentWeight => _alignmentWeight;
        public float AlignmentAffectedRadiusSqr => _alignmentAffectedRadius * _alignmentAffectedRadius;

        public Vector3 SimulationAreaCenter => _simulationAreaCenter;
        public Vector3 SimulationAreaScale => _simulationAreaScale / 2; // MEMO: 計算では 1/2 の方が書きやすいため
        public float AvoidSimulationAreaWeight => _avoidSimulationAreaWeight;

        public float MaxSpeed => _maxSpeed;
        public float MaxSteerForce => _maxSteerForce;

        public float3 InstanceScale => _instanceScale;
    }
}