using System;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEngine;

public struct BoatTag : IComponentData
{
}

/// <summary>
///     Package of data for each triangle.
/// </summary>
public struct TriangleData : IBufferElementData
{
    // The corners of this triangle in global coordinates
    public float3 P1;
    public float3 P2;
    public float3 P3;

    // The center of the triangle
    public float3 Center;

    // The distance to the surface from the center of the triangle
    public float DistanceToSurface;

    // The normal to the triangle
    public float3 Normal;

    // The area of the triangle
    public float Area;

    public TriangleData(float3 p1, float3 p2, float3 p3)
    {
        P1 = p1;
        P2 = p2;
        P3 = p3;

        // Center of the triangle
        Center = (p1 + p2 + p3) / 3f;

        // Distance to the surface from the center of the triangle
        DistanceToSurface =
            Mathf.Abs(DistanceToWater
                .TempDistanceToWater(Center)); //WaterController.current.DistanceToWater(Center, Time.time));

        // Normal of the triangle
        Normal = math.normalizesafe(math.cross(p2 - p1, p3 - p1));

        // Area of the triangle
        var a = math.distance(p1, p2);

        var c = math.distance(p3, p1);

        Area = a * c * math.sin(AngleDeg(p2 - p1, p3 - p1)) / 2f;

        float AngleDeg(float3 vec1, float3 vec2)
        {
            return math.degrees(math.acos(math.dot(vec1, vec2) / (math.length(vec1) * math.length(vec2))));
        }
    }
}

/// <summary>
///     Coordinates of a boat mesh vertex
/// </summary>
public struct BoatVertices : IBufferElementData
{
    private float3 _vertex;

    public static implicit operator float3(BoatVertices boatVertices)
    {
        return boatVertices._vertex;
    }

    public static implicit operator BoatVertices(float3 float3)
    {
        return new BoatVertices {_vertex = float3};
    }
}

/// <summary>
///     Coordinates of a boat mesh vertex in Global Coordinates
/// </summary>
public struct BoatGlobalVertices : IBufferElementData
{
    private float3 _vertex;

    public static implicit operator float3(BoatGlobalVertices boatVertices)
    {
        return boatVertices._vertex;
    }

    public static implicit operator BoatGlobalVertices(float3 float3)
    {
        return new BoatGlobalVertices {_vertex = float3};
    }

    public static implicit operator BoatVertices(BoatGlobalVertices boatVertices)
    {
        return boatVertices._vertex;
    }

    public static implicit operator BoatGlobalVertices(BoatVertices boatVertex)
    {
        return new BoatGlobalVertices {_vertex = boatVertex};
    }
}

/// <summary>
///     Positions in allVerticesArray, such as 0, 3, 5, to build triangles
/// </summary>
public struct BoatTriangles : IBufferElementData
{
    private int _triangle;

    public static implicit operator int(BoatTriangles boatTriangles)
    {
        return boatTriangles._triangle;
    }

    public static implicit operator BoatTriangles(int intValue)
    {
        return new BoatTriangles {_triangle = intValue};
    }
}

/// <summary>
///     Find all the distances to water once because some triangles share vertices, so reused.
/// </summary>
public struct DistanceToWater : IBufferElementData
{
    private float _distance;

    public static implicit operator float(DistanceToWater distanceToWater)
    {
        return distanceToWater._distance;
    }

    public static implicit operator DistanceToWater(float floatValue)
    {
        return new DistanceToWater {_distance = floatValue};
    }

    /// <summary>
    ///     Find the distance from a vertex to water.
    ///     <para>Make sure the position is in global coordinates.</para>
    ///     Positive if above water.
    ///     Negative if below water.
    /// </summary>
    /// <param name="position"></param>
    /// <returns></returns>
    public static float TempDistanceToWater(float3 position)
    {
        const float waterHeight = 0;

        return position.y - waterHeight;
    }
}

public class Buoyancy : MonoBehaviour, IConvertGameObjectToEntity
{
    public void Convert(Entity entity, EntityManager dstManager, GameObjectConversionSystem conversionSystem)
    {
        dstManager.AddComponentData(entity, new BoatTag());

        //Init the arrays and lists
        var triangles = Array.ConvertAll(GetComponent<MeshFilter>().mesh.triangles,
            i => (BoatTriangles) i);
        var vertices = Array.ConvertAll(GetComponent<MeshFilter>().mesh.vertices,
            v => (BoatVertices) (float3) v);

        using (var tempTriangles = new NativeArray<BoatTriangles>(triangles, Allocator.Temp))
        {
            dstManager.AddBuffer<BoatTriangles>(entity).AddRange(tempTriangles);
        }

        using (var tempVertices = new NativeArray<BoatVertices>(vertices, Allocator.Temp))
        {
            dstManager.AddBuffer<BoatVertices>(entity).AddRange(tempVertices);
        }

        dstManager.AddBuffer<BoatGlobalVertices>(entity);

        dstManager.AddBuffer<TriangleData>(entity);
        dstManager.AddBuffer<DistanceToWater>(entity);
    }
}

// Runs everything during Fixed Update. After transforms were communicated to game objects.
[UpdateInGroup(typeof(LateSimulationSystemGroup))]
public class BuoyancySystem : JobComponentSystem
{
    // Adapted from Unity Buoyancy Tutorial: https://www.habrador.com/tutorials/unity-boat-tutorial/3-buoyancy/

    private Entity _boatEntity;
    private EntityManager _entityManager;

    protected override void OnStartRunning()
    {
        _entityManager = World.EntityManager;
    }

    protected override JobHandle OnUpdate(JobHandle inputDeps)
    {
        var boatVerticesArray = _entityManager.GetBuffer<BoatVertices>(_boatEntity).AsNativeArray();
        var boatGlobalVerticesArray = _entityManager.GetBuffer<BoatGlobalVertices>(_boatEntity).AsNativeArray();
        var distanceToWaterArray = _entityManager.GetBuffer<DistanceToWater>(_boatEntity).AsNativeArray();

        var mainJobChain = new GenerateUnderwaterMesh
        {
            BoatVertices = boatVerticesArray,
            LocalToWorld = _entityManager.GetComponentData<LocalToWorld>(_boatEntity),
            BoatGlobalVertices = boatGlobalVerticesArray,
            DistanceToWater = distanceToWaterArray
        }.Schedule(boatVerticesArray.Length, 32, inputDeps);

        var boatTrianglesBuffer = _entityManager.GetBuffer<BoatTriangles>(_boatEntity);

        // List that will store the data we need to sort the vertices based on distance to water.
        var vertexDataPlaceholders = new NativeArray<VertexData>(boatTrianglesBuffer.Length,
            Allocator.TempJob, NativeArrayOptions.UninitializedMemory);


        return mainJobChain;
    }

    // Find all the distances to water once because some triangles share vertices, so reuse.
    private struct GenerateUnderwaterMesh : IJobParallelFor
    {
        [ReadOnly] public NativeArray<BoatVertices> BoatVertices;
        [ReadOnly] public LocalToWorld LocalToWorld;

        [WriteOnly] public NativeArray<BoatGlobalVertices> BoatGlobalVertices;
        [WriteOnly] public NativeArray<DistanceToWater> DistanceToWater;

        public void Execute(int index)
        {
            var globalPosition = math.transform(LocalToWorld.Value, BoatVertices[index]);
            BoatGlobalVertices[index] = globalPosition;
            DistanceToWater[index] = global::DistanceToWater.TempDistanceToWater(globalPosition);
        }
    }

    // Help container to store triangle data so we can sort the distances
    private struct VertexData
    {
        // An index so we can form clockwise triangles.
        public int Index;

        // The distance to water from this vertex.
        public float Distance;

        // The global position of the vertex.
        public float3 GlobalVertexPos;
    }

    // Determine which of the triangles are underwater.
    private struct ProcessTriangles : IJobParallelFor
    {
        [DeallocateOnJobCompletion] public NativeArray<VertexData> VerticesData;

        [ReadOnly] public NativeArray<DistanceToWater> DistanceToWater;
        [ReadOnly] public NativeArray<BoatGlobalVertices> BoatGlobalVertices;
        [ReadOnly] public NativeArray<BoatTriangles> BoatTriangles;

        // Possible candidate for write only?
        public NativeQueue<TriangleData>.ParallelWriter UnderwaterTriangles;

        // Loop through all the triangles (3 vertices = 1 triangle).
        public void Execute(int triangle)
        {
            // Underwater tripwires.
            var notUnderwater = true;
            var allUnderwater = true;

            // Loop through the 3 vertices.
            var placeholder = new VertexData();
            for (var x = 0; x < 3; x++)
            {
                var cursor = BoatTriangles[triangle * 3 + x];
                placeholder.Distance = DistanceToWater[cursor];
                placeholder.Index = x;
                placeholder.GlobalVertexPos = BoatGlobalVertices[cursor];

                VerticesData[triangle * 3 + x] = placeholder;

                if (placeholder.Distance < 0f)
                    notUnderwater = false;
                else
                    allUnderwater = false;
            }

            // All vertices are above the water.
            if (notUnderwater)
                return;

            // Create the triangles that are below the waterline.
            var v0 = VerticesData[triangle * 3];
            var v1 = VerticesData[triangle * 3 + 1];
            var v2 = VerticesData[triangle * 3 + 2];

            // All vertices are underwater
            if (allUnderwater)
            {
                UnderwaterTriangles.Enqueue(new TriangleData(v0.GlobalVertexPos, v1.GlobalVertexPos,
                    v2.GlobalVertexPos));
                return;
            }

            // 1 or 2 vertices are below the water.
            // Sort the vertices in descending order by distance to water.
            if (v0.Distance < v2.Distance)
                Swap(ref v0, ref v2);
            if (v0.Distance < v1.Distance)
                Swap(ref v0, ref v1);
            if (v1.Distance < v2.Distance)
                Swap(ref v1, ref v2);

            void Swap(ref VertexData lhs, ref VertexData rhs)
            {
                var temp = lhs;
                lhs = rhs;
                rhs = temp;
            }

            // One vertex is above the water, the other two are below.
            if (v1.Distance < 0)
                AddTrianglesOneAboveWater(v0, v1, v2);
            else
                AddTrianglesTwoAboveWater(v0, v1, v2);
        }

        // Build the new triangles where one of the old vertices is above the water.
        private void AddTrianglesOneAboveWater(VertexData aboveHigh, VertexData belowMid, VertexData belowLow)
        {
            var h = aboveHigh.GlobalVertexPos;

            // Left of H is M.
            // Right of H is L.

            // Find the index of M.
            var mIndex = aboveHigh.Index - 1;
            if (mIndex < 0)
                mIndex = 2;

            // We also need the heights to water.
            var hH = aboveHigh.Distance;
            var hM = 0f;
            var hL = 0f;

            var m = float3.zero;
            var l = float3.zero;

            // This means M is the below mid point
            if (belowMid.Index == mIndex)
            {
                m = belowMid.GlobalVertexPos;
                l = belowLow.GlobalVertexPos;

                hM = belowMid.Distance;
                hL = belowLow.Distance;
            }
            else
            {
                m = belowLow.GlobalVertexPos;
                l = belowMid.GlobalVertexPos;

                hM = belowLow.Distance;
                hL = belowMid.Distance;
            }

            // Now we can calculate where we should cut the triangle to form 2 new triangles
            // because the resulting area will always form a square.

            // Point iM
            var tM = -hM / (hH - hM);

            var iM = tM * (h - m) + m;

            //Point iL
            var tL = -hL / (hH - hL);

            var iL = tL * (h - l) + l;

            UnderwaterTriangles.Enqueue(new TriangleData(m, iM, iL));
            UnderwaterTriangles.Enqueue(new TriangleData(m, iL, l));
        }

        // Build the new triangles where two of the old vertices are above the water.
        private void AddTrianglesTwoAboveWater(VertexData aboveHigh, VertexData aboveMid, VertexData belowLow)
        {
            // H and M are above the water.
            // H is after the vertex that's below water, which is L.
            // So we know which one is L because it is last in the sorted list.
            var l = belowLow.GlobalVertexPos;

            // Left of H is M.
            // Right of H is L.

            // Find the index of H.
            var hIndex = belowLow.Index + 1;
            if (hIndex > 2)
                hIndex = 0;

            // We also need the heights to water.
            var hL = belowLow.Distance;
            var hM = 0f;
            var hH = 0f;

            var m = float3.zero;
            var h = float3.zero;

            // This means M is the below mid point.
            if (aboveMid.Index == hIndex)
            {
                h = aboveMid.GlobalVertexPos;
                m = aboveHigh.GlobalVertexPos;

                hH = aboveMid.Distance;
                hM = aboveHigh.Distance;
            }
            else
            {
                h = aboveHigh.GlobalVertexPos;
                m = aboveMid.GlobalVertexPos;

                hH = aboveHigh.Distance;
                hM = aboveMid.Distance;
            }

            // Now we can find where to cut the triangle.

            // Point jM
            var tM = -hL / (hM - hL);

            var jM = tM * (m - l) + l;


            // Point jH
            var tH = -hL / (hH - hL);

            var jH = tH * (h - l) + l;

            // 1 triangle below the water
            UnderwaterTriangles.Enqueue(new TriangleData(l, jH, jM));
        }

        private struct AddUnderwaterForces : IJobParallelFor
        {
            public void Execute(int index)
            {
            }
        }
    }
}