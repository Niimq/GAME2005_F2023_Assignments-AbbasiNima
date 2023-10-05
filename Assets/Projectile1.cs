using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Projectile : MonoBehaviour
{
    public float launchVelocity;
    public float launchAngle;
    public float launchHeight;

    public List<Physics_Library> bodies;
    public Physics_Library body;

    void LaunchBall()
    {
        // 1. Update velX and velY based on horizontal and vertical components of launch velocity & launch angle
        // 2. Assign position to new launch height and re-launch the ball!
        foreach (Physics_Library body in bodies)
        {
             body.velocity.x = launchVelocity * Mathf.Cos(launchAngle * Mathf.Deg2Rad);
             body.velocity.y = launchVelocity * Mathf.Sin(launchAngle * Mathf.Deg2Rad);
             transform.position = new Vector3(0.0f, launchHeight, 0.0f);
            Instantiate(body, transform.position, Quaternion.identity );
        }    
    }

    void FixedUpdate()
    {
        // Restart the launch
        if (Input.GetKey(KeyCode.Space))
        {
         
           
           LaunchBall();
        }
    }
}
