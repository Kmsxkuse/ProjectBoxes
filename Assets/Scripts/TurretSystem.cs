using System;
using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Physics;
using Unity.Physics.Systems;
using Unity.Rendering;
using Unity.Transforms;
using UnityEngine;

public class TurretSystem : JobComponentSystem
{
    private Transform _mainCamTransform;

    private NativeArray<float3> _outputTest, _cameraEndpoint;
    private EntityQuery _turrets;

    protected override void OnCreate()
    {
        _turrets = GetEntityQuery(typeof(Rotation), ComponentType.ReadOnly<WorldRenderBounds>(),
            ComponentType.ReadOnly<TurretTag>());

        _outputTest = new NativeArray<float3>(new[] {new float3(), new float3()}, Allocator.Persistent);
        _cameraEndpoint = new NativeArray<float3>(1, Allocator.Persistent);
    }

    protected override void OnDestroy()
    {
        _outputTest.Dispose();
        _cameraEndpoint.Dispose();
    }

    protected override void OnStartRunning()
    {
        if (Camera.main == null)
            throw new Exception("Camera not found!");

        _mainCamTransform = Camera.main.transform;
    }

    protected override JobHandle OnUpdate(JobHandle inputDeps)
    {
        Debug.DrawLine(_outputTest[0], _outputTest[1], Color.red);

        var mainJobChain = new CameraEndpoint
        {
            CollisionWorld = World.DefaultGameObjectInjectionWorld
                .GetExistingSystem<BuildPhysicsWorld>().PhysicsWorld.CollisionWorld,
            CameraLocation = _mainCamTransform.position,
            CameraDirection = _mainCamTransform.forward,
            Endpoint = _cameraEndpoint
        }.Schedule(inputDeps);

        mainJobChain = new TurretDirection
        {
            Endpoint = _cameraEndpoint,
            DeltaTime = Time.DeltaTime,
            OutputTest = _outputTest
        }.ScheduleSingle(_turrets, mainJobChain);

        return mainJobChain;
    }

    [BurstCompile]
    private struct CameraEndpoint : IJob
    {
        [ReadOnly] public CollisionWorld CollisionWorld;
        [ReadOnly] public float3 CameraLocation, CameraDirection;

        [WriteOnly] public NativeArray<float3> Endpoint;

        public void Execute()
        {
            var raycastInput = new RaycastInput
            {
                Start = CameraLocation,
                End = CameraDirection * 500 + CameraLocation,
                Filter = new CollisionFilter
                {
                    BelongsTo = 1u << 2,
                    CollidesWith = 1u << 1,
                    GroupIndex = 0
                }
            };

            if (!CollisionWorld.CastRay(raycastInput, out var terrainHit))
                return;

            Endpoint[0] = terrainHit.Position;
        }
    }

    [BurstCompile]
    private struct TurretDirection : IJobForEach<Rotation, WorldRenderBounds, TurretTag>
    {
        [ReadOnly] public NativeArray<float3> Endpoint;
        [ReadOnly] public float DeltaTime;

        [WriteOnly] public NativeArray<float3> OutputTest;

        public void Execute(ref Rotation turretRotation, [ReadOnly] ref WorldRenderBounds turretCenter,
            [ReadOnly] ref TurretTag turretTag)
        {
            OutputTest[0] = Endpoint[0];
            OutputTest[1] = turretCenter.Value.Center;
            turretRotation.Value = math.mul(math.normalize(turretRotation.Value),
                quaternion.AxisAngle(math.up(), 0.1f * DeltaTime));
        }
    }
}