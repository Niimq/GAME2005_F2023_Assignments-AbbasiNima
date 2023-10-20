using System.Collections.Generic;
using UnityEngine;

public class PhysicsWorld : MonoBehaviour
{
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

    private void FixedUpdate()
    {
        AddNewBodiesFromScene();

        //Do gravity
        foreach (PhysicsBody body in bodies)
        {
            // Apply acceleration due to gravity
            body.velocity += gravity * body.gravityScale * dt;
            
            // Damp Motion
            body.velocity *= (1.0f - (body.friction * dt));
           
            // Do kinematics
            body.transform.position += body.velocity * dt;
        }
        t += dt;
    }
}
