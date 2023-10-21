using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public abstract class PhysicsShape : MonoBehaviour
{
    public enum Type
    {
        Sphere = 0
    }
    public abstract Type GetShapeType();
}
