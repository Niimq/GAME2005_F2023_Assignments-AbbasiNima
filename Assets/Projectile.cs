using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Projectile : MonoBehaviour
{
    public float launchVelocity;
    public float launchAngle;
    public float launchHeight;

    private float velX = 10.0f;  // Doesn't change between frames
    private float velY = 10.0f;  // Under constant acceleration (gravity)

    void Start()
    {
        LaunchBall();
    }

    void LaunchBall()
    {
        Debug.Log("Launch!");

        // 1. Update velX and velY based on horizontal and vertical components of launch velocity & launch angle
        float angleDeg2Rad = launchAngle * Mathf.PI / 180;
        // Applying trigonemtery
        velX = Mathf.Cos(angleDeg2Rad) * launchVelocity;
        velY = Mathf.Sin(angleDeg2Rad) * launchVelocity;
        
        // 2. Assign position to new launch height and re-launch the ball!
        transform.position = new Vector3(transform.position.x, transform.position.y, transform.position.z);
    }

    private void Update()
    {
         // Restart the launch
        if (Input.GetKey(KeyCode.Space))
        {
            LaunchBall();
        }
    }

    void FixedUpdate()
    {
       

        float dt = Time.fixedDeltaTime;
        float acc = Physics.gravity.y;

        // Update velocity using acceleration over time (velX remains constant)
        velY = velY + acc * dt;
        // acceleration = change in velocity/ time
        // m/s^2
        //0 - 60kph / 10s <---- 60kph faster in 10s
        // 6kph/s rate of acceleration
        // 6000m/h/s
        // ~1.67 m/s/s

        // v = change in position / time
        // a = change in velocity / time
        // a = change in velocity / time - change in velocity / time / time

        // v1 = v0 + a * t

        // Update position using velocity over time
        transform.position = new Vector3(
            transform.position.x + velX * dt,
            transform.position.y + velY * dt,
            transform.position.z
        );
    }
}
