using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PhysicsBody : MonoBehaviour
{
    public Vector3 velocity = Vector3.zero;
    public float gravityScale = 1.0f;
    public float friction = 0.0f;

   // [Range[]]
    public float mass = 1.0f;
    public PhysicsShape shape = null;

    public Vector3 NetForce
    {
        get;
        private set; // we can't set it but we can get it. getter and setter!
    }

    public void AddForce(Vector3 force)
    {
        NetForce += force;
    }

    public void ResetForces()
    { 
        NetForce = Vector3.zero;
    }

    public void Awake()
    {
        shape = GetComponent<PhysicsShape>();
    }
}
