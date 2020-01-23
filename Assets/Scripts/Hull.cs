using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Physics;
using Unity.Transforms;
using UnityEngine;

public struct LocalPlayer : IComponentData
{
}

public class Hull : MonoBehaviour, IConvertGameObjectToEntity
{
    public Transform playerTransform;

    public void Convert(Entity entity, EntityManager dstManager, GameObjectConversionSystem conversionSystem)
    {
        // Adding local player tag
        dstManager.AddComponentData(entity, new LocalPlayer());
        dstManager.AddComponentData(entity, new CopyTransformToGameObject());
        dstManager.AddComponentObject(entity, playerTransform);
    }
}

public class HullSailingSystem : JobComponentSystem
{
    private NativeArray<int> _currentEngineStage;
    private EntityQuery _localPlayer;
    private NativeArray<bool> _recentEngineStageChange;
    private NativeArray<float> _stageChangeTime;

    protected override void OnCreate()
    {
        // Cached access to a set of ComponentData based on a specific query 
        _localPlayer = GetEntityQuery(typeof(PhysicsVelocity), typeof(PhysicsMass),
            typeof(Rotation), ComponentType.ReadOnly<LocalPlayer>());

        // Creating output values. Unity must output from jobs using Native Arrays.
        _recentEngineStageChange = new NativeArray<bool>(new[] {false}, Allocator.Persistent);
        _stageChangeTime = new NativeArray<float>(new[] {0f}, Allocator.Persistent);
        _currentEngineStage = new NativeArray<int>(new[] {0}, Allocator.Persistent);
    }

    protected override void OnDestroy()
    {
        _recentEngineStageChange.Dispose();
        _stageChangeTime.Dispose();
        _currentEngineStage.Dispose();
    }

    protected override JobHandle OnUpdate(JobHandle inputDeps)
    {
        var mainJobChain = new ThrottleJob
        {
            Forward = Input.GetKey(KeyCode.W),
            Back = Input.GetKey(KeyCode.S),
            CurrentStage = _currentEngineStage,
            CurrentTime = UnityEngine.Time.realtimeSinceStartup,
            StageChangeTime = _stageChangeTime,
            RecentEngineStageChange = _recentEngineStageChange
        }.Schedule(inputDeps);

        mainJobChain = new MovementJob
        {
            Left = Input.GetKey(KeyCode.A),
            Right = Input.GetKey(KeyCode.D),
            CurrentStage = _currentEngineStage,
            DeltaTime = UnityEngine.Time.deltaTime
        }.Schedule(_localPlayer, mainJobChain);

        return mainJobChain;
    }

    [BurstCompile]
    private struct ThrottleJob : IJob
    {
        [ReadOnly] public bool Forward, Back;
        [ReadOnly] public float CurrentTime;

        public NativeArray<bool> RecentEngineStageChange;
        public NativeArray<float> StageChangeTime;
        public NativeArray<int> CurrentStage;

        public void Execute()
        {
            if (!RecentEngineStageChange[0])
            {
                if (Forward || Back)
                {
                    RecentEngineStageChange[0] = true;
                    StageChangeTime[0] = CurrentTime;
                }

                if (Forward)
                    CurrentStage[0]++;

                if (Back)
                    CurrentStage[0]--;

                CurrentStage[0] = math.clamp(CurrentStage[0], -1, 4);
            }
            else if (CurrentTime > StageChangeTime[0] + 0.5f)
            {
                RecentEngineStageChange[0] = false;
            }
        }
    }

    [BurstCompile]
    private struct MovementJob : IJobForEach<PhysicsVelocity, PhysicsMass, Rotation, LocalPlayer>
    {
        [ReadOnly] public NativeArray<int> CurrentStage;
        [ReadOnly] public bool Left, Right;
        [ReadOnly] public float DeltaTime;

        public void Execute(ref PhysicsVelocity shipPhysics, ref PhysicsMass shipMass, ref Rotation shipRotation,
            [ReadOnly] ref LocalPlayer c1)
        {
            var rotation = 0f;
            if (Left)
                rotation--;
            if (Right)
                rotation++;

            shipPhysics.Angular += new float3(0, rotation * shipMass.InverseMass, 0);

            shipPhysics.Linear = math.lerp(shipPhysics.Linear,
                math.forward(shipRotation.Value) * CurrentStage[0], DeltaTime * 10f);
        }
    }
}