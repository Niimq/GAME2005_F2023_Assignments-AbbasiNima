using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PhysicsShapePlane : PhysicsShape
{
    public override Type GetShapeType()
    {
        return PhysicsShape.Type.Plane;
    }
}
