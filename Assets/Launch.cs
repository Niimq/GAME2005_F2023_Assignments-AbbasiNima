using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Launch : MonoBehaviour
{
    public float speed = 0.0f; // launchVelocity
    public float elevationAngleDegrees = 30.0f; // launchAngle
    public float launchHeight;
    public GameObject projectilePrefab;

    void Shoot()
    {
        // 1. Update velX and velY based on horizontal and vertical components of launch velocity & launch angle
        // 2. Assign position to new launch height and re-launch the ball!

         GameObject NewObject = Instantiate(projectilePrefab);
        NewObject.transform.position = transform.position;

        PhysicsBody body = NewObject.GetComponent<PhysicsBody>();
        body.velocity = new Vector3 (
           Mathf.Cos(elevationAngleDegrees * Mathf.Deg2Rad) * speed,
           Mathf.Sin(elevationAngleDegrees * Mathf.Deg2Rad) * speed,
            0.0f);
    }

    private void FixedUpdate()
    {
        if (Input.GetKeyDown(KeyCode.Space))
        {
            Shoot();
        }
    }
}
