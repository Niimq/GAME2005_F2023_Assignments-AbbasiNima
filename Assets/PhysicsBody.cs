using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PhysicsBody : MonoBehaviour
{
    public Vector3 velocity = Vector3.zero;
    public float gravityScale = 1.0f;
    public float friction = 0.0f;

    public PhysicsShape shape = null;
    public void Awake()
    {
        shape = GetComponent<PhysicsShape>();
    }
}
