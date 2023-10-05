using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PhysicsWorld : MonoBehaviour
{
    public float dt = 1/30;
    public float t = 0.0f;
    public List<PhysicsBody> bodies = new List<PhysicsBody>();
    public Vector3 gravity = new Vector3 (0f, -9.8f, 0f);

    // Start is called before the first frame update
    void Start()
    {
        dt = Time.fixedDeltaTime;
    }
    
    void CheckForNewObjects()
    {
       PhysicsBody [] bodiesFound = FindObjectsOfType<PhysicsBody>();
        foreach(PhysicsBody body in bodiesFound)
        {
            if (!bodies.Contains(body))
            {
                bodies.Add(body);
            }
        }
    }

    private void FixedUpdate()
    {
        CheckForNewObjects();

        //Do gravity
        foreach (PhysicsBody body in bodies)
        {
            body.velocity += gravity * body.gravityScale * dt;
            // Do kinematics
            body.transform.position += body.velocity * dt;
        }
        t += dt;

        // Do Drag
        foreach(PhysicsBody body in bodies)
        {
            body.velocity += -(-body.drag * body.velocity) * dt;
        }
    }
}
