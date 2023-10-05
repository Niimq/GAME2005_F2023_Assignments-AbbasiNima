using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PhysicsWorld : MonoBehaviour
{
    public float dt = 1/30;
    public float t = 0.0f;
    public List<Physics_Library> bodies;
    public Vector3 gravity = new Vector3 (0f, -9.8f, 0f);

    // Start is called before the first frame update
    void Start()
    {
        dt = Time.fixedDeltaTime;
    }

    private void FixedUpdate()
    {
        //Do gravity
        foreach (Physics_Library body in bodies)
        {
            body.velocity += gravity * body.gravityScale * dt;
            // Do kinematics
            body.transform.position += body.velocity * dt;
        }
        t += dt;

        // Do Drag
        foreach(Physics_Library body in bodies)
        {
            body.velocity += -(-body.drag * body.velocity) * dt;
        }
    }
}
