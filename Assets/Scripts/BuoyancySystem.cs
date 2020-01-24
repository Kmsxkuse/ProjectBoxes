using System;
using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Physics;
using Unity.Physics.Extensions;
using Unity.Transforms;

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
    
    public TriangleGlobals(LocalToWorld localToWorld, TriangleData triangleData)
    {
        G1 = math.transform(localToWorld.Value, triangleData.P1);
        G2 = math.transform(localToWorld.Value, triangleData.P2);
        G3 = math.transform(localToWorld.Value, triangleData.P3);
    }

    public TriangleGlobals(float3 g1, float3 g2, float3 g3)
    {
        G1 = g1;
        G2 = g2;
        G3 = g3;
    }

    public float3 Center()
    {
        return (G1 + G2 + G3) / 3f;
    }

    public float DistanceToSurface(float3 center)
    {
        return math.abs(center.y);
    }

    public float3 Normal()
    {
        return math.normalize(math.cross(G2 - G1, G3 - G1));
    }

    public float SurfaceArea()
    {
        var a = math.distance(G1, G2);
        var c = math.distance(G3, G1);
        var u = G2 - G1;
        var v = G3 - G1;
        return a * c * math.sin(math.degrees(math.acos(math.dot(u, v) 
                                                        / (math.length(u) * math.length(v))))) / 2f;
    }
}

public class BuoyancySystem : JobComponentSystem
{
    private Entity _boat;
    private EntityQuery _triangles, _boatPhysics;
    private NativeQueue<TriangleGlobals> _underwaterTriangles;

    private const float WaterRho = 1027f;
    // TODO: Replace this constant with variable from Physics World.
    private const float DebugGravity = 9.81f;
    
    protected override void OnStartRunning()
    {
        var boatLookup = GetEntityQuery(typeof(BoatTag));
        using (var boatArray = boatLookup.ToEntityArray(Allocator.Temp))
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
    }

    protected override void OnDestroy()
    {
        _underwaterTriangles.Dispose();
    }

    protected override JobHandle OnUpdate(JobHandle inputDeps)
    {
        var mainJobChain = new CalculateValues
        {
            L2W = EntityManager.GetComponentData<LocalToWorld>(_boat),
            UnderwaterTriangles = _underwaterTriangles.AsParallelWriter()
        }.Schedule(_triangles, inputDeps);
        
        mainJobChain = new ApplyBuoyancyForces
        {
            DeltaTime = UnityEngine.Time.deltaTime,
            UnderwaterTriangles = _underwaterTriangles
        }.Schedule(_boatPhysics, mainJobChain);

        return mainJobChain;
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
            var high = aboveHigh.Position;
            var heightHigh = aboveHigh.Height;

            var midIndex = aboveHigh.Index - 1 > 0 ? 0 : 2;

            if (underMid.Index != midIndex)
                Swap(ref underMid, ref underLow);
                
            var mid = underMid.Position;
            var low = underLow.Position;

            var heightMid = underMid.Height;
            var heightLow = underLow.Height;
            
            // Point interMiddle
            var ratio = -heightMid / (heightHigh - heightMid);

            var interpolatedPoint = ratio * (high - mid);

            var interMiddle = interpolatedPoint + mid;
            
            // Point interLow
            ratio = -heightLow / (heightHigh - heightLow);

            interpolatedPoint = ratio * (high - low);

            var interLow = interpolatedPoint + low;
            
            // Two triangles forming a quadrilateral underwater
            UnderwaterTriangles.Enqueue(new TriangleGlobals(mid, interMiddle, interLow));
            UnderwaterTriangles.Enqueue(new TriangleGlobals(mid, interLow, low));
        }

        private void AddTrianglesTwoAboveWater(VertexData aboveHigh, VertexData aboveMid, VertexData underLow)
        {
            var low = underLow.Position;
            var heightLow = underLow.Height;
            
            var highIndex = underLow.Index + 1 > 2 ? 0 : 1;
            
            // == not a typo.
            if (aboveMid.Index == highIndex)
                Swap(ref aboveMid, ref aboveHigh);

            var mid = aboveMid.Position;
            var high = aboveHigh.Position;

            var heightMid = aboveMid.Height;
            var heightHigh = aboveHigh.Height;
            
            // Point interMiddle
            var ratio = -heightLow / (heightMid - heightLow);

            var interpolatedPoint = ratio * (mid - low);

            var interMiddle = interpolatedPoint + low;
            
            // Point interLow
            ratio = -heightLow / (heightHigh - heightLow);

            interpolatedPoint = ratio * (high - low);

            var interHigh = interpolatedPoint + low;
            
            // Only one triangle underwater
            UnderwaterTriangles.Enqueue(new TriangleGlobals(low, interHigh, interMiddle));
        }
    }

    [BurstCompile]
    private struct ApplyBuoyancyForces : IJobForEach<PhysicsVelocity, PhysicsMass, Translation, Rotation, BoatTag>
    {
        [ReadOnly] public float DeltaTime;
        
        public NativeQueue<TriangleGlobals> UnderwaterTriangles;
        
        public void Execute(ref PhysicsVelocity physicsVelocity, [ReadOnly] ref PhysicsMass physicsMass,
            [ReadOnly] ref Translation translation, [ReadOnly] ref Rotation rotation, [ReadOnly] ref BoatTag c0)
        {
            while (UnderwaterTriangles.TryDequeue(out var triangle))
            {
                physicsVelocity.ApplyImpulse(physicsMass, translation, rotation, 
                    BuoyancyForce(WaterRho, triangle) * DeltaTime, triangle.Center());
            }
        }
        
        private float3 BuoyancyForce(float rho, TriangleGlobals triangle)
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
            
            var buoyancyForce = rho * DebugGravity 
                                    * triangle.DistanceToSurface(triangle.Center()) 
                                    * triangle.SurfaceArea() * triangle.Normal();

            //The vertical component of the hydrostatic forces don't cancel out but the horizontal do
            buoyancyForce.x = 0f;
            buoyancyForce.z = 0f;

            return buoyancyForce;
        }
    }
}