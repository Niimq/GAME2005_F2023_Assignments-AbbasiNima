using System.Collections; // These are our includes
using System.Collections.Generic;
using UnityEngine; // It's basically a namespace

public class HelloScript : MonoBehaviour
{
    public float X, Y;
    public float A, B;
    public float lineLength = 5.0f;
    public float t = 0.0f;
    public float dt = 1.0f / 60.0f;

    // Start is called before the first frame update
    void Start()
    {
        Debug.Log("Hello World!");
    }

    // Update is called once per frame
    void Update()
    {
        Debug.DrawLine(transform.position, transform.position 
            + new Vector3(lineLength, 0.0f, 0.0f), Color.red
            );
        Debug.DrawLine(transform.position, transform.position
           + new Vector3(0.0f, lineLength, 0.0f), Color.green
           );
        Debug.DrawLine(transform.position, transform.position
           + new Vector3(0.0f, 0.0f, lineLength), Color.blue
           );
    }

    private void FixedUpdate()
    {
        X = X + -Mathf.Sin(t * A) * A * B * dt;
        Y = Y + -Mathf.Cos(t * A) * A * B * dt;

        transform.position = new Vector3(X, Y, transform.position.z);

        t += dt;
    }
}
