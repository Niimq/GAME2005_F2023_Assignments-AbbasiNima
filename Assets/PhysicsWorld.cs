using System.Collections.Generic;
using UnityEngine;

public class PhysicsWorld : MonoBehaviour
{
    public bool DebugMode = true;
    public float dt = 1 / 30;
    public float t = 0.0f;
    public List<PhysicsBody> bodies;
    public Vector3 gravity = new Vector3(0f, -9.8f, 0f);

    // Start is called before the first frame update
    void Start()
    {
        bodies = new List<PhysicsBody>();
        dt = Time.fixedDeltaTime;
    }

    void AddNewBodiesFromScene()
    {
        PhysicsBody[] bodiesFound = FindObjectsOfType<PhysicsBody>();
        foreach (PhysicsBody body in bodiesFound)
        {
            if (!bodies.Contains(body))
            {
                bodies.Add(body);
            }
        }
    }

    private void ApplyKinematics()
    {

        //Do gravity
        foreach (PhysicsBody body in bodies)
        {
            // Apply acceleration due to gravity
          //  body.velocity += gravity * body.gravityScale * dt;

            // Damp Motion
            body.velocity *= (1.0f - (body.friction * dt));

            // Do kinematics
            body.transform.position += body.velocity * dt;
        }

    }
    public bool CheckCollisionBetweenSphere(PhysicsShapeSphere shapeA, PhysicsShapeSphere shapeB)
    {
        Vector3 displacements = shapeA.transform.position - shapeB.transform.position;
        float distance = displacements.magnitude;
        if (distance < shapeA.radius + shapeB.radius)
        {
            return true;
        }
        else
        {
            return false;
        } 
    }

    public bool CheckCollisionBetween(PhysicsBody bodyA, PhysicsBody bodyB)
    {
        // Figure out what type of collision to perform e.g.
        // Sphere-Plane collision detection ...
        // Sphere-Sphere Collision Detections.
        // etc.
        if (bodyA.shape == null || bodyB.shape == null) return false;
        else if (bodyA.shape.GetShapeType() == PhysicsShape.Type.Sphere
            && bodyB.shape.GetShapeType() == PhysicsShape.Type.Sphere) 
        {
           return CheckCollisionBetweenSphere((PhysicsShapeSphere)bodyA.shape, (PhysicsShapeSphere)bodyB.shape);
        }
        else 
        {
            throw new System.Exception("UnKnow Shape Type!"); // throwing an exception to know if we did not implement the new shape's collision
        }
        
    }

    private void CheckCollisions()
    {
        if (DebugMode)
        {
            for (int i = 0; i < bodies.Count; i++)
            {
                bodies[i].GetComponent<Renderer>().material.SetColor("_Color", Color.white);
            }
        }

        for (int i = 0; i < bodies.Count; i++)
        {
            PhysicsBody bodyA = bodies[i];

            

            // Check if it collides with each other object
            for (int j = i + 1; j < bodies.Count; j++)
            {
                PhysicsBody bodyB = bodies[j];

                // Check for collision between A and B
                bool isColiding = CheckCollisionBetween(bodyA, bodyB);
                if (DebugMode)
                {
                    if (isColiding) 
                    {
                        bodyA.GetComponent<Renderer>().material.SetColor("_Color", Color.red);
                        bodyB.GetComponent<Renderer>().material.SetColor("_Color", Color.red);
                    }
                }
            }
        }
    }

    private void FixedUpdate()
    {
        AddNewBodiesFromScene();

        // Apply kinematics
        ApplyKinematics();

        // Check collisions
        CheckCollisions();

        t += dt;
    }
}
