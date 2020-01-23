using System;
using Unity.Mathematics;
using UnityEngine;

public class CamCode : MonoBehaviour
{
    private Vector3 _offset;
    public float azimuthalSpeed = 8f;
    public Transform playerTransform;
    public float polarSpeed = 2f;

    [NonSerialized] public float3 ToPosition;

    private void Update()
    {
        var playerPosition = playerTransform.position;
        Transform camTransform;
        (camTransform = transform).RotateAround(playerPosition, Vector3.up, Input.GetAxis("Mouse X") * azimuthalSpeed);
        camTransform.eulerAngles -= new Vector3(Input.GetAxis("Mouse Y") * polarSpeed, 0, 0);
        _offset = camTransform.position - playerPosition;
    }

    private void LateUpdate()
    {
        transform.position = playerTransform.position + _offset;
    }
}