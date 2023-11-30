using System.Collections.Generic;
using System.Collections.ObjectModel;
using Unity.VisualScripting;
using UnityEngine;

public class PhysicsWorld : MonoBehaviour
{
    public bool DebugMode = true;
    public float dt = 1 / 30;
    public float t = 0.0f;
    public List<PhysicsBody> bodies;
    public Vector3 gravity = new Vector3(0f, -9.8f, 0f);
    public bool colliding;

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
    Vector3 GetGravityForce(PhysicsBody body)
    { 
        return gravity * body.mass * body.gravityScale;
    }

    private void ResetNetForces()
    {
        foreach (PhysicsBody body in bodies)
        {
            body.ResetForces();
        }
    }

    private void ApplyKinematics()
    {

        //Do gravity
        foreach (PhysicsBody body in bodies)
        {
             // Do kinematics
             body.transform.position += body.velocity * dt;

             // Velocity
             Debug.DrawLine(body.transform.position, body.transform.position + body.velocity, Color.red);
            
        }

    }

    private void applyAcceleration()
    {
        foreach (PhysicsBody body in bodies)
        {
            // Gravity
            Vector3 GravityForce = GetGravityForce(body);
            body.AddForce(GravityForce);
            // Gravity does not scale based on mass
            // let g = -10
            // let m = 5
            // Fg * m * g = -50
            // a = Fg / m
            // a = -50 / 5 = -10
            // Test with m = 1 (will get same a)
            // Fg = -10 * 1 = -10
            // a = Fg / m = -10 / 1 = -10
            // Solve by applying gravitation acceleration directly!
            // Note that this is physically incorrect because Fg is no longer proportional to mass,
            // so this will break the moment anything depends on Fg
            //Vector3 Fg = gravity * body.gravityScale * body.mass;
            //Vector3 Fn = -Fg;   // TODO -- handle normal force at an angle
            //Vector3 fNet = Fg + Fn;

            // Acceleration
            Vector3 acceleration = body.NetForce / body.mass;// / body.mass;

            //Gravity force
            Debug.DrawLine(body.transform.position, body.transform.position + GetGravityForce(body), Color.yellow);

            // Net force
            Debug.DrawLine(body.transform.position, body.transform.position + body.NetForce, Color.blue);

            Debug.Log(acceleration);
            // Change velocity based on acceleration
            body.velocity += acceleration * dt;

            // Damp Motion
            body.velocity *= (1.0f - (body.Damping * dt));
    }   }

    public bool CheckCollisionBetweenSphere(PhysicsShapeSphere shapeA, PhysicsShapeSphere shapeB)
    {
        //1. Determine displacement between spheres (difference in position)
        Vector3 displacements = shapeA.transform.position - shapeB.transform.position;
        //2. Get distance by taking length of the displacement
        float distance = displacements.magnitude;

        //3. If the distance is less than the sum of the radiii, then overlapping.
        if (distance < shapeA.radius + shapeB.radius)
        {
            /*displacements.Normalize();
            
            Vector3 v1 = shapeA.GetComponent<PhysicsBody>().velocity;
            float X1 = Vector3.Dot(displacements, v1);
            v1.x = displacements.x * X1;
            v1.y = displacements.y * X1;

            Vector3 displacement2 = displacements * -1;
            Vector3 v2 = shapeB.GetComponent<PhysicsBody>().velocity;
            float X2 = Vector3.Dot(displacement2, v2);
            v2.x = displacement2.x * X1;
            v2.y = displacement2.y - v2.x;

            shapeA.GetComponent<PhysicsBody>().velocity = v1;
            shapeB.GetComponent<PhysicsBody>().velocity = v2;
        */

            //shapeA.GetComponent<PhysicsBody>().gravityScale = 0;
            //shapeB.GetComponent<PhysicsBody>().gravityScale = 0;
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
        bool isColliding = Mathf.Abs(projection) <= sphere.radius;
        colliding = isColliding;
        PhysicsBody SphereBody = sphere.GetComponent<PhysicsBody>();

        float fgDotNormal = Vector3.Dot(GetGravityForce(SphereBody), normal); // is gravity opposing the surface normal?

        if (isColliding)
        {
            Vector3 mtv = (sphere.radius - projection) * normal;
            sphere.transform.position += mtv;

            if (fgDotNormal > 0.0f)
            { 
            
            }
            else
            {
                Vector3 FGravityPerp = fgDotNormal * normal * dt;

                Vector3 NormalForce = -FGravityPerp;

                sphere.GetComponent<PhysicsBody>().AddForce(NormalForce);

                Debug.DrawLine(sphere.transform.position, sphere.transform.position + NormalForce, Color.green);

                float frictionCoefficient = SphereBody.frictionCoefficient;

                // Calculate friction force magnitude from coefficient of friction and normal force magnitude
                float frictionMagnitudeMax = frictionCoefficient * NormalForce.magnitude;

                // Kinetic friction works to reduce relative velocity
                Vector3 VelocitySphereRelativeToPlane = SphereBody.velocity - plane.GetComponent<PhysicsBody>().velocity;
                Vector3 VelocityOutOfPlane = Vector3.Dot(VelocitySphereRelativeToPlane, normal) * normal;

                // Make sure friction Does not apply out of plane
                Vector3 VelocityInPlane = VelocitySphereRelativeToPlane - VelocityOutOfPlane;

                // Direction opposite of velocity, length of 1
                Vector3 FrictionDirection = -VelocityInPlane.normalized;

                // Combine magnitude of friction with direction opposing sliding motion
                Vector3 ForceFriction = frictionMagnitudeMax * FrictionDirection;

                SphereBody.AddForce(ForceFriction);

                Debug.DrawLine(sphere.transform.position, sphere.transform.position + ForceFriction, new Color(1.0f, 0.65f, 0.0f));
            }
        }

        // Determine forces acting on the object


        return isColliding;
    }

    public bool CheckCollisionsBetweenSphereHalfSpace(PhysicsShapeSphere sphere, PhysicsShapeHalfSpace halfSpace)
    {
        //?????

        // Let a sphere be defined by:
        //A postion of the center of the sphere
        //A radius
        Vector3 FrictionForce = Vector3.zero;
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
        bool isColliding = projection <= sphere.radius;
        colliding = isColliding;

        PhysicsBody SphereBody = sphere.GetComponent<PhysicsBody>();

        float fgDotNormal = Vector3.Dot(GetGravityForce(SphereBody), normal);

        if (isColliding) 
        {
            Vector3 mtv = (sphere.radius - projection) * normal;
            sphere.transform.position += mtv;
            if (fgDotNormal > 0.0f)
            {

            }
            else
            {
                Vector3 FGravityPerp = fgDotNormal * normal * dt;

                Vector3 NormalForce = -FGravityPerp;

                sphere.GetComponent<PhysicsBody>().AddForce(NormalForce);

                Debug.DrawLine(sphere.transform.position, sphere.transform.position + NormalForce, Color.green);

                float frictionCoefficient = SphereBody.frictionCoefficient;

                // Calculate friction force magnitude from coefficient of friction and normal force magnitude
                float frictionMagnitudeMax = frictionCoefficient * NormalForce.magnitude;

                // Kinetic friction works to reduce relative velocity
                Vector3 VelocitySphereRelativeToPlane = SphereBody.velocity - halfSpace.GetComponent<PhysicsBody>().velocity;
                Vector3 VelocityOutOfPlane = Vector3.Dot(VelocitySphereRelativeToPlane, normal) * normal;

                // Make sure friction Does not apply out of plane
                Vector3 VelocityInPlane = VelocitySphereRelativeToPlane - VelocityOutOfPlane;

                // Direction opposite of velocity, length of 1
                Vector3 FrictionDirection = -VelocityInPlane.normalized;

                // Combine magnitude of friction with direction opposing sliding motion
                Vector3 ForceFriction = frictionMagnitudeMax * FrictionDirection;

                SphereBody.AddForce(ForceFriction);

                Debug.DrawLine(sphere.transform.position, sphere.transform.position + ForceFriction, new Color(1.0f, 0.65f, 0.0f));
                //TODO: Make sure friction does not cause the sphere to speed up relative to the plane
            }

        }

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

        // Set all net forces to 0 -- should do before adding any forces for the frame
        ResetNetForces();

        // Change position based on veloctiy and time
        ApplyKinematics();

        // Check collisions, apply forces due to collisions
        CheckCollisions();

        // Change net acceleration based on net force.. also adds gravity
        applyAcceleration();

        t += dt;
    }
}
