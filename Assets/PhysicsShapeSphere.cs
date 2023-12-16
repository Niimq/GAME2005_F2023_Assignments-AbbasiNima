using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PhysicsShapeSphere : PhysicsShape
{
    public float radius = 0.5f;

   public override Type GetShapeType()
    {
        return PhysicsShape.Type.Sphere;
    }

}
