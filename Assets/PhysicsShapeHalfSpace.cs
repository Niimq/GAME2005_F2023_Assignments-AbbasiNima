using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PhysicsShapeHalfSpace : PhysicsShape
{
    public override Type GetShapeType()
    {
        return PhysicsShape.Type.halfspace;
    }
}
