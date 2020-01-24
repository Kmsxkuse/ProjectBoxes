using System;
using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Physics;
using Unity.Physics.Extensions;
using Unity.Transforms;
using UnityEngine;

public struct BoatTag : IComponentData
{
}

public struct TriangleData : IComponentData
{
    // Vertex Points
    public readonly float3 P1, P2, P3;

    public TriangleData(float3 p1, float3 p2, float3 p3) : this()
    {
        P1 = p1;
        P2 = p2;
        P3 = p3;
    }
}

public struct TriangleGlobals
{
    // Global Points
    public readonly float3 G1, G2, G3;
    public readonly float3 Center;

    public TriangleGlobals(LocalToWorld localToWorld, TriangleData triangleData)
    {
        G1 = math.transform(localToWorld.Value, triangleData.P1);
        G2 = math.transform(localToWorld.Value, triangleData.P2);
        G3 = math.transform(localToWorld.Value, triangleData.P3);
        Center = (G1 + G2 + G3) / 3f;
    }

    public TriangleGlobals(float3 g1, float3 g2, float3 g3)
    {
        G1 = g1;
        G2 = g2;
        G3 = g3;
        Center = (G1 + G2 + G3) / 3f;
    }

    public float3 Normal()
    {
        return math.normalizesafe(math.cross(G2 - G1, G3 - G1));
    }

    public float DistanceToSurface()
    {
        return math.abs(Center.y);
    }

    public float SurfaceArea()
    {
        var a = math.distance(G1, G2);
        var c = math.distance(G3, G1);
        var u = G2 - G1;
        var v = G3 - G1;
        return a * c * math.sin(math.acos(math.dot(u, v) / (math.length(u) * math.length(v)))) / 2f;
    }
}

public class BuoyancySystem : JobComponentSystem
{
    private const float WaterRho = 1027f;

    private Entity _boat;

    //private NativeQueue<TestForce> _debugForces;
    private EntityQuery _triangles, _boatPhysics;
    private NativeQueue<TriangleGlobals> _underwaterTriangles;

    protected override void OnStartRunning()
    {
        var boatLookup = GetEntityQuery(typeof(BoatTag),
            ComponentType.ReadOnly<LocalToWorld>());
        using (var boatArray = boatLookup.ToEntityArray(Allocator.TempJob))
        {
            if (boatArray.Length > 1)
                throw new Exception("ERROR: More than 1 player controlled boat detected.");

            _boat = boatArray[0];
        }

        _boatPhysics = GetEntityQuery(typeof(PhysicsVelocity),
            ComponentType.ReadOnly<PhysicsMass>(), ComponentType.ReadOnly<Translation>(),
            ComponentType.ReadOnly<Rotation>(), ComponentType.ReadOnly<BoatTag>());

        _triangles = GetEntityQuery(ComponentType.ReadOnly<TriangleData>());

        _underwaterTriangles = new NativeQueue<TriangleGlobals>(Allocator.Persistent);
        //_debugForces = new NativeQueue<TestForce>(Allocator.Persistent);
    }

    protected override void OnDestroy()
    {
        _underwaterTriangles.Dispose();
        //_debugForces.Dispose();
    }

    protected override JobHandle OnUpdate(JobHandle inputDeps)
    {
        //while (_debugForces.TryDequeue(out var force)) Debug.DrawRay(force.Origin, force.Force, force.Color);

        var mainJobChain = new CalculateValues
        {
            L2W = EntityManager.GetComponentData<LocalToWorld>(_boat),
            UnderwaterTriangles = _underwaterTriangles.AsParallelWriter()
        }.Schedule(_triangles, inputDeps);

        mainJobChain = new ApplyBuoyancyForces
        {
            DeltaTime = Time.fixedDeltaTime,
            UnderwaterTriangles = _underwaterTriangles
            //DebugForces = _debugForces
        }.ScheduleSingle(_boatPhysics, mainJobChain);

        return mainJobChain;
    }

    private struct TestForce
    {
        public readonly float3 Force;
        public readonly float3 Origin;
        public readonly Color32 Color;

        public TestForce(float3 force, float3 origin, Color32 color)
        {
            Force = force;
            Origin = origin;
            Color = color;
        }
    }

    private struct VertexData
    {
        public readonly float3 Position;
        public readonly float Height;
        public readonly int Index;

        public VertexData(float3 position, float height, int index)
        {
            Position = position;
            Height = height;
            Index = index;
        }
    }

    [BurstCompile]
    private struct CalculateValues : IJobForEach<TriangleData>
    {
        [ReadOnly] public LocalToWorld L2W;

        [WriteOnly] public NativeQueue<TriangleGlobals>.ParallelWriter UnderwaterTriangles;

        public void Execute([ReadOnly] ref TriangleData triangleData)
        {
            var currentGlobals = new TriangleGlobals(L2W, triangleData);

            // TODO: Add water mesh height calculation for wave physics.
            // Assuming water is flat and calm
            var h1 = new VertexData(currentGlobals.G1, currentGlobals.G1.y, 0);
            var h2 = new VertexData(currentGlobals.G2, currentGlobals.G2.y, 1);
            var h3 = new VertexData(currentGlobals.G3, currentGlobals.G3.y, 2);

            // All above water, ignore.
            if (h1.Height > 0 && h2.Height > 0 && h3.Height > 0)
                return;

            // All below water.
            if (h1.Height < 0 && h2.Height < 0 && h3.Height < 0)
            {
                UnderwaterTriangles.Enqueue(currentGlobals);
                return;
            }

            // Sorting points in descending order
            if (h1.Height < h3.Height)
                Swap(ref h1, ref h3);

            if (h1.Height < h2.Height)
                Swap(ref h1, ref h2);

            if (h2.Height < h3.Height)
                Swap(ref h2, ref h3);

            // Mathematics for the following two originated from:
            // https://gamasutra.com/view/news/237528/Water_interaction_model_for_boats_in_video_games.php
            // And overall code structure and commenting adapted from:
            // https://www.habrador.com/tutorials/unity-boat-tutorial/3-buoyancy/

            if (h2.Height < 0)
                AddTrianglesOneAboveWater(h1, h2, h3);
            else
                AddTrianglesTwoAboveWater(h1, h2, h3);
        }

        private static void Swap(ref VertexData left, ref VertexData right)
        {
            var temp = left;
            left = right;
            right = temp;
        }

        private void AddTrianglesOneAboveWater(VertexData aboveHigh, VertexData underMid, VertexData underLow)
        {
            //H is always at position 0
            var highPosition = aboveHigh.Position;

            //Left of H is M
            //Right of H is L

            //Find the index of M
            var mIndex = aboveHigh.Index - 1;
            if (mIndex < 0) mIndex = 2;

            //We also need the heights to water
            var highHeight = aboveHigh.Height;

            //This means M is at position 1 in the List
            if (underMid.Index == mIndex)
                Swap(ref underLow, ref underMid);

            var mPosition = underLow.Position;
            var lPosition = underMid.Position;

            var mHeight = underLow.Height;
            var lHeight = underMid.Height;

            //Now we can calculate where we should cut the triangle to form 2 new triangles
            //because the resulting area will always form a square

            //Point I_M
            var mh = highPosition - mPosition;

            var tM = -mHeight / (highHeight - mHeight);

            var miM = tM * mh;

            var iM = miM + mPosition;

            //Point I_L
            var lh = highPosition - lPosition;

            var tL = -lHeight / (highHeight - lHeight);

            var liL = tL * lh;

            var iL = liL + lPosition;

            //Save the data, such as normal, area, etc      
            //2 triangles below the water  
            UnderwaterTriangles.Enqueue(new TriangleGlobals(mPosition, iM, iL));
            UnderwaterTriangles.Enqueue(new TriangleGlobals(mPosition, iL, lPosition));
        }

        private void AddTrianglesTwoAboveWater(VertexData aboveHigh, VertexData aboveMid, VertexData underLow)
        {
            //H and M are above the water
            //H is after the vertice that's below water, which is L
            //So we know which one is L because it is last in the sorted list
            var lowPosition = underLow.Position;

            //Find the index of H
            var hIndex = underLow.Index + 1;
            if (hIndex > 2) hIndex = 0;

            //We also need the heights to water
            var lowHeight = underLow.Height;

            //This means that H is at position 1 in the list
            if (aboveMid.Index == hIndex)
                Swap(ref aboveHigh, ref aboveMid);

            var hPosition = aboveHigh.Position;
            var mPosition = aboveMid.Position;

            var hHeight = aboveHigh.Height;
            var mHeight = aboveMid.Height;

            //Now we can find where to cut the triangle

            //Point J_M
            var lm = mPosition - lowPosition;

            var tM = -lowHeight / (mHeight - lowHeight);

            var ljM = tM * lm;

            var jM = ljM + lowPosition;

            //Point J_H
            var lh = hPosition - lowPosition;

            var tH = -lowHeight / (hHeight - lowHeight);

            var ljH = tH * lh;

            var jH = ljH + lowPosition;

            //Save the data, such as normal, area, etc
            //1 triangle below the water
            UnderwaterTriangles.Enqueue(new TriangleGlobals(lowPosition, jH, jM));
        }
    }

    [BurstCompile]
    private struct ApplyBuoyancyForces : IJobForEach<PhysicsVelocity, PhysicsMass, Translation, Rotation, BoatTag>
    {
        [ReadOnly] public float DeltaTime;

        // There should be only one thread running for application of forces.
        public NativeQueue<TriangleGlobals> UnderwaterTriangles;
        //public NativeQueue<TestForce> DebugForces;

        public void Execute(ref PhysicsVelocity physicsVelocity, [ReadOnly] ref PhysicsMass physicsMass,
            [ReadOnly] ref Translation translation, [ReadOnly] ref Rotation rotation, [ReadOnly] ref BoatTag c0)
        {
            while (UnderwaterTriangles.TryDequeue(out var triangle))
                //var force = BuoyancyForce(WaterRho, triangle);

                physicsVelocity.ApplyImpulse(physicsMass, translation, rotation,
                    BuoyancyForce(WaterRho, triangle) * DeltaTime, triangle.Center);

            // DEBUG
            //DebugForces.Enqueue(new TestForce(triangle.Normal() * 3f, triangle.Center, Color.white));
            //DebugForces.Enqueue(new TestForce(math.normalize(force) * -3f, triangle.Center, Color.blue));
        }

        private static float3 BuoyancyForce(float rho, TriangleGlobals triangle)
        {
            //Buoyancy is a hydrostatic force - it's there even if the water isn't flowing or if the boat stays still

            // F_buoyancy = rho * g * V
            // rho - density of the mediaum you are in
            // g - gravity
            // V - volume of fluid directly above the curved surface 

            // V = z * S * n 
            // z - distance to surface
            // S - surface area
            // n - normal to the surface

            var buoyancyForce = rho * Physics.gravity.y * triangle.DistanceToSurface()
                                * triangle.SurfaceArea() * triangle.Normal();

            //The vertical component of the hydrostatic forces don't cancel out but the horizontal do
            buoyancyForce.x = 0f;
            buoyancyForce.z = 0f;

            return buoyancyForce;
        }
    }
}