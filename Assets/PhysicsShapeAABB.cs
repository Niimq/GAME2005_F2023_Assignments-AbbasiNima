using System;
using System.Collections;
using System.Collections.Generic;
using System.Drawing;
using UnityEngine;
using UnityEngine.SocialPlatforms.GameCenter;
using UnityEngine.UIElements;

public class PhysicsShapeAABB : PhysicsShape
{
    public Vector3 point = new Vector3().normalized;
    
    public Vector3 mMax;
    public Vector3 mMin;

    public override Type GetShapeType()
    {
        return PhysicsShape.Type.Box;
    }

    // Start is called before the first frame update
    void Start()
    {
        
    }

    void UpdateMinMax(Vector3 point)
    {
       
        // Update each component seperately
        mMax.x = Mathf.Max(mMax.x, point.x);
        mMax.y = Mathf.Max(mMax.y, point.y);
        mMax.z = Mathf.Max(mMax.z, point.z);

        mMin.x = Mathf.Min(mMin.x, point.x);
        mMin.y = Mathf.Min(mMin.y, point.y);
        mMin.z = Mathf.Min(mMin.z, point.z);
    }

    // Update is called once per frame
    void Update()
    {
    }
}
