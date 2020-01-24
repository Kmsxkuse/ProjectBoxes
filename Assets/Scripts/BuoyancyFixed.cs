using System.Collections.Generic;
using Unity.Entities;
using UnityEngine;

public class BuoyancyFixed : MonoBehaviour
{
    private IEnumerable<ComponentSystemBase> _simSystems;

    private void Start()
    {
        World.DefaultGameObjectInjectionWorld.GetOrCreateSystem<SimulationSystemGroup>().Enabled = false;
        _simSystems = World.DefaultGameObjectInjectionWorld.GetOrCreateSystem<SimulationSystemGroup>().Systems;
    }

    private void FixedUpdate()
    {
        foreach (var sys in _simSystems) sys.Update();
    }
}