using Unity.Entities;
using UnityEngine;

public class BoatPhysicsBootstrap : MonoBehaviour, IConvertGameObjectToEntity
{
    public void Convert(Entity entity, EntityManager dstManager, GameObjectConversionSystem conversionSystem)
    {
        dstManager.AddComponentData(entity, new BoatTag());

        var boatVertices = GetComponent<MeshFilter>().mesh.vertices;
        var boatTriangles = GetComponent<MeshFilter>().mesh.triangles;

        var triangleArchetype = dstManager.CreateArchetype(typeof(TriangleData));

        for (var triangle = 0; triangle < boatTriangles.Length / 3; triangle++)
        {
            var cursor = triangle * 3;
            var placeholderTriangleData = new TriangleData(boatVertices[boatTriangles[cursor]],
                boatVertices[boatTriangles[cursor + 1]], boatVertices[boatTriangles[cursor + 2]]);

            dstManager.SetComponentData(dstManager.CreateEntity(triangleArchetype), placeholderTriangleData);
        }
    }
}