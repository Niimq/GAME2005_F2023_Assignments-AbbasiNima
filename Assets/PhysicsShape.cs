using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public abstract class PhysicsShape : MonoBehaviour
{
    public enum Type
    {
        Sphere = 0,
        halfspace,
        Plane
    }

    // Enum for material types
    public enum MaterialType
    {
        Steel = 1,
        Wood = 2,
        Rubber = 3,
        Cloth = 4,
        Stone = 5
    }

    [SerializeField] private MaterialType materialType;  // Expose material type in the Unity Editor

    // Getter and setter but can only be set in here
    public float Bounciness = 0.0f;

    public void UpdateBounciness()
    {

        // Set Bounciness based on material type during initialization
        switch (materialType)
        {
            case MaterialType.Steel:
                Bounciness = 0.0f;
                break;
            case MaterialType.Wood:
                Bounciness = 0.5f;
                break;
            case MaterialType.Rubber:
                Bounciness = 0.8f;
                break;
            case MaterialType.Cloth:
                Bounciness = 0.4f;
                break;
            case MaterialType.Stone:
                Bounciness = 0.1f;
                break;
            default:
                Bounciness = 0.0f; // Default value
                break;
        }
        // Also did it with if's but doesn't work 
        if (materialType == MaterialType.Steel)
            Bounciness = 0.0f;
        else if(materialType == MaterialType.Wood)
            Bounciness = 0.5f;
        else if(materialType == MaterialType.Rubber)
            Bounciness = 0.8f;
        else if (materialType == MaterialType.Cloth)
            Bounciness = 0.4f;
        else if (materialType == MaterialType.Stone)
            Bounciness = 0.1f;
    }
    public abstract Type GetShapeType();
}
