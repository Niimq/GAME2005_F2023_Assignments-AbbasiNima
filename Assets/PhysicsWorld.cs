using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;

public class PhysicsWorld : MonoBehaviour
{
    public bool DebugMode = true;
    public float dt = 1 / 30;
    public float t = 0.0f;
    public List<PhysicsBody> bodies;
    public Vector3 gravity = new Vector3(0f, -9.8f, 0f);

    // Start is called before the first frame update
    void Start()
    {
        bodies = new List<PhysicsBody>();
        dt = Time.fixedDeltaTime;
    }

    void AddNewBodiesFromScene()
    {
        PhysicsBody[] bodiesFound = FindObjectsOfType<PhysicsBody>();
        foreach (PhysicsBody body in bodiesFound)
        {
            if (!bodies.Contains(body))
            {
                bodies.Add(body);
            }
        }
    }

    private void ApplyKinematics()
    {

        //Do gravity
        foreach (PhysicsBody body in bodies)
        {
            // Apply acceleration due to gravity
          //  body.velocity += gravity * body.gravityScale * dt;

            // Damp Motion
            body.velocity *= (1.0f - (body.friction * dt));

            // Do kinematics
            body.transform.position += body.velocity * dt;
        }

    }
    public bool CheckCollisionBetweenSphere(PhysicsShapeSphere shapeA, PhysicsShapeSphere shapeB)
    {
        //1. Determine displacement between spheres (difference in position)
        Vector3 displacements = shapeA.transform.position - shapeB.transform.position;
        //2. Get distance by taking length of the displacement
        float distance = displacements.magnitude;

        //3. If the distance is less than the sum of the radiii, then overlapping.
        if (distance < shapeA.radius + shapeB.radius)
        {
            return true;
        }
        else
        {
            return false;
        } 
    }
    public bool CheckCollisionsBetweenSpherePlane(PhysicsShapeSphere sphere, PhysicsShapePlane plane)
    {
        //?????

        // Let a sphere be defined by:
        //A postion of the center of the sphere
        //A radius

        // Let a plane be defined by:
        // A point anywhere on the plane ( we can use transform.position for this)
        // The orientation of the plane as a quaternion or euler engles (rotation around x, y, and z)

        //1. Find the normal vector perpendicular to the plane
        //      rotate a basis vector e.g. (0,0,1) by the orientation of the object
        Vector3 normal = plane.transform.rotation * new Vector3(0, 1, 0);

        //2. Find the displacement from the point on the plane to the center of the sphere by vector subtraction
        //      Vec3 displacement = sphere.position - plane.position
        Vector3 displacement = sphere.transform.position - plane.transform.position;

        //3. Find the scalar projection of the displacement vector onto the normal vector using dot product
        //      float projection = Dot(displacement, plane.normal)
        float projection = Vector3.Dot(displacement, normal);

        //4. For a PLANE, if the length of the projection is less than the sphere radius, they are overlapping
        //      bool isColliding = abs(projection) < sphere.radius
        bool isColliding = Mathf.Abs(projection) < sphere.radius;
        return isColliding;
    }

    public bool CheckCollisionsBetweenSphereHalfSpace(PhysicsShapeSphere sphere, PhysicsShapeHalfSpace halfSpace)
    {
        //?????

        // Let a sphere be defined by:
        //A postion of the center of the sphere
        //A radius

        // Let a plane be defined by:
        // A point anywhere on the plane ( we can use transform.position for this)
        // The orientation of the plane as a quaternion or euler engles (rotation around x, y, and z)

        //1. Find the normal vector perpendicular to the plane
        //      rotate a basis vector e.g. (0,0,1) by the orientation of the object
        //Vector3 normal = halfSpace.transform.up;
        Vector3 normal = halfSpace.transform.rotation * new Vector3(0, 1, 0);
        

        //2. Find the displacement from the point on the plane to the center of the sphere by vector subtraction
        //      Vec3 displacement = sphere.position - plane.position
        Vector3 displacement = sphere.transform.position - halfSpace.transform.position;

        //3. Find the scalar projection of the displacement vector onto the normal vector using dot product
        //      float projection = Dot(displacement, plane.normal)
        float projection = Vector3.Dot(displacement, normal);

        //4. For a HALFSPACE, if the projection is less than the sphere radius, they are overlapping
        //      bool isColliding = projection < sphere.radius
        bool isColliding = projection < sphere.radius;

        return isColliding;
    }

    public bool CheckCollisionBetween(PhysicsBody bodyA, PhysicsBody bodyB)
    {
        PhysicsShape.Type ShapeOfA = bodyA.shape.GetShapeType();
        PhysicsShape.Type ShapeOfB = bodyB.shape.GetShapeType();



        if (bodyA.shape == null || bodyB.shape == null) return false;
        else if (ShapeOfA == PhysicsShape.Type.Sphere
            && ShapeOfB == PhysicsShape.Type.Sphere)
        {
            return CheckCollisionBetweenSphere((PhysicsShapeSphere)bodyA.shape, (PhysicsShapeSphere)bodyB.shape);
        }
        else if (ShapeOfA == PhysicsShape.Type.Sphere
            && ShapeOfB == PhysicsShape.Type.halfspace)
        {
            return CheckCollisionsBetweenSphereHalfSpace((PhysicsShapeSphere)bodyA.shape, (PhysicsShapeHalfSpace)bodyB.shape);
        }
        // Half-Space collision with sphere and itself
        else if (ShapeOfA == PhysicsShape.Type.halfspace
            && ShapeOfB == PhysicsShape.Type.Sphere)
        {
            return CheckCollisionsBetweenSphereHalfSpace((PhysicsShapeSphere)bodyB.shape, (PhysicsShapeHalfSpace)bodyA.shape);
        }
        else if (ShapeOfA == PhysicsShape.Type.halfspace
            && ShapeOfB == PhysicsShape.Type.halfspace)
        {
            return false;
        }
        // Plane collision with it self and also halfspace
        else if (ShapeOfA == PhysicsShape.Type.Plane
            && ShapeOfB == PhysicsShape.Type.Sphere)
        { 
            return CheckCollisionsBetweenSpherePlane((PhysicsShapeSphere)bodyB.shape, (PhysicsShapePlane)bodyA.shape);
        }
        else if (ShapeOfA == PhysicsShape.Type.Sphere
            && ShapeOfB == PhysicsShape.Type.Plane)
        {
            return CheckCollisionsBetweenSpherePlane((PhysicsShapeSphere)bodyA.shape, (PhysicsShapePlane)bodyB.shape);
        }
        else if (ShapeOfA == PhysicsShape.Type.Plane
            && ShapeOfB == PhysicsShape.Type.halfspace)
        { return false; }
        else if (ShapeOfA == PhysicsShape.Type.halfspace
            && ShapeOfB == PhysicsShape.Type.Plane)
        { return false; }
        else if (ShapeOfA == PhysicsShape.Type.Plane
            && ShapeOfB == PhysicsShape.Type.Plane)
        { return false; }
        else
        {
            throw new System.Exception("UnKnow Shape Type!"); // throwing an exception to know if we did not implement the new shape's collision
        }
        
    }

    private void CheckCollisions()
    {
        if (DebugMode)
        {
            for (int i = 0; i < bodies.Count; i++)
            {
                bodies[i].GetComponent<Renderer>().material.SetColor("_Color", Color.white);
            }
        }

        for (int i = 0; i < bodies.Count; i++)
        {
            PhysicsBody bodyA = bodies[i];

            

            // Check if it collides with each other object
            for (int j = i + 1; j < bodies.Count; j++)
            {
                PhysicsBody bodyB = bodies[j];

                // Check for collision between A and B
                bool isColiding = CheckCollisionBetween(bodyA, bodyB);
                if (DebugMode)
                {
                    if (isColiding) 
                    {
                        bodyA.GetComponent<Renderer>().material.SetColor("_Color", Color.red);
                        bodyB.GetComponent<Renderer>().material.SetColor("_Color", Color.red);
                    }
                }
            }
        }
    }

    private void FixedUpdate()
    {
        AddNewBodiesFromScene();

        // Apply kinematics
        ApplyKinematics();

        // Check collisions
        CheckCollisions();

        t += dt;
    }
}
