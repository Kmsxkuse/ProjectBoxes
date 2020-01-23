using Unity.Entities;
using UnityEngine;

public struct TurretTag : IComponentData
{
}

public class Turret : MonoBehaviour, IConvertGameObjectToEntity
{
    public void Convert(Entity entity, EntityManager dstManager, GameObjectConversionSystem conversionSystem)
    {
        dstManager.AddComponentData(entity, new TurretTag());
    }
}