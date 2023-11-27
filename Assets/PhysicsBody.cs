using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PhysicsBody : MonoBehaviour
{
    public Vector3 velocity = Vector3.zero;
    public float gravityScale = 1.0f;
    public float Damping = 1.0f;
    public float frictionCoefficient = 0.05f;

    private Vector3 netForce;

    // Physics engines often store inverse of mass for performance reasons
    // (it's faster when attempting to divide by mass to instead multiply by cached inverse)
    public float mass = 1.0f;
    public float massInverse = 1.0f;
    public PhysicsShape shape = null;

    public float Mass //Property (feature of C# that lets you make variable-like getter and setter)
    { 
        get { return mass; }
        set 
        {
            if (value <= 0.000001f)
            {
                throw new System.Exception("Stop it! " + name + " SHould not have a mass of 0 or less.");
            }
            mass = value;
            massInverse = 1.0f / mass;
        }
    }

    public float MassInverse
    {
        get { return massInverse; } // public getter
        private set { massInverse = value; } // a private setter
    }

    public Vector3 NetForce
    {
        get { return netForce; }
        private set { netForce = value; } // we can't set it but we can get it. getter and setter!
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
