using System;
using System.Collections;
using System.Collections.Generic;
using System.Drawing;
using UnityEngine;
using UnityEngine.SocialPlatforms.GameCenter;
using UnityEngine.UIElements;

public class PhysicsShapeAABB : PhysicsShape
{
    public Vector3 centerpoint = new Vector3().normalized;
    
    
    public Vector3 mMin;
    public Vector3 mMax;
    
    public float x = 0.0f;
    public float y = 0.0f;
    public float z = 0.0f;
   

    public override Type GetShapeType()
    {
        return PhysicsShape.Type.Box;
    }

    // Start is called before the first frame update
    void Start()
    {
        
    }
    private void updateMinMax()
    {
        mMin.x = centerpoint.x - x;
        mMin.y = centerpoint.y - y;
        mMin.z = centerpoint.z - z;

        mMax.x = centerpoint.x + x;
        mMax.y = centerpoint.y + y;
        mMax.z = centerpoint.z + z;
    }
    void UpdateMinMax(Vector3 point)
    {

        centerpoint.x = point.x;
        centerpoint.y = point.y;
        centerpoint.z = point.z;
    }

    // Update is called once per frame
    void Update()
    {
       UpdateMinMax(transform.position);
        updateMinMax();
    }
}
