using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ForceTestCube : MonoBehaviour
{
    public int variant = 0;
    public Rigidbody rb;
    private int hadTriggerPrevFrame=0;
    private Vector3 lastColPoint = Vector3.zero;
    Vector3 inertiaTensorWS = Vector3.one;
    // Start is called before the first frame update
    void Start()
    {
        rb.automaticInertiaTensor = false;
        rb.inertiaTensor = new Vector3(1,2,1);
        inertiaTensorWS = Quaternion.Inverse(rb.rotation) * (rb.inertiaTensor);  // WARNING: this might be wrong!!!
        inertiaTensorWS = new Vector3(Mathf.Abs(inertiaTensorWS.x), Mathf.Abs(inertiaTensorWS.y), Mathf.Abs(inertiaTensorWS.z));
        Debug.Log("varinat" + variant + " inertiaTensor: " + rb.inertiaTensor);
        //rb.velocity = Vector3.up;
        rb.angularVelocity = new Vector3(0, 0, 0);
        lastColPoint = rb.position+Vector3.up;
        rb.AddForceAtPosition(Vector3.forward, lastColPoint, ForceMode.Impulse);
        hadTriggerPrevFrame = 2;

        //rb.AddForce(Vector3.forward, ForceMode.Impulse);
        //rb.AddTorque(new Vector3(0,-1,0), ForceMode.Impulse);

        Debug.Log("added dir= " + rb.GetAccumulatedForce()*Time.fixedDeltaTime + "  added Torque" + rb.GetAccumulatedTorque()*Time.fixedDeltaTime);
        Debug.Log("added to Point Vel (self-estimated)=" + GetAddedPointVelocity(lastColPoint));
    }

    // Update is called once per frame
    void Update()
    {
        
    }
    private void FixedUpdate()
    {
        inertiaTensorWS = Quaternion.Inverse(rb.rotation) * (rb.inertiaTensor);  // WARNING: this might be wrong!!!
        inertiaTensorWS = new Vector3(Mathf.Abs(inertiaTensorWS.x), Mathf.Abs(inertiaTensorWS.y), Mathf.Abs(inertiaTensorWS.z));
        Debug.Log("global iTensor=" + inertiaTensorWS);

        if (hadTriggerPrevFrame == 2)
        {
            hadTriggerPrevFrame = 1;
        }
        else if (hadTriggerPrevFrame == 1)
        {
            hadTriggerPrevFrame = 0;
            Debug.Log("Vel at Point after Col = " + rb.GetPointVelocity(lastColPoint));
            Debug.Log("Vel dir after Col = " + rb.velocity);
            Debug.Log("angular Velocity= " + rb.angularVelocity);

            //in local Space, because Unitys inertiaTensor is in local space
            Vector3 localPoint = lastColPoint - rb.worldCenterOfMass;
            //localPoint = new Vector3(localPoint.x * transform.localScale.x, localPoint.y * transform.localScale.y, localPoint.z * transform.localScale.z);
            Vector3 localDir = Vector3.forward;

            Vector3 speedPerAngularVelocity = Vector3.Cross(localPoint, localDir); //how much one unit roation around each axis would move the point in direction of force
            Vector3 angularDirections = new Vector3(speedPerAngularVelocity.x > 0 ? 1 : speedPerAngularVelocity.x < 0 ? -1 : 0,
                                                    speedPerAngularVelocity.y > 0 ? 1 : speedPerAngularVelocity.y < 0 ? -1 : 0,
                                                    speedPerAngularVelocity.z > 0 ? 1 : speedPerAngularVelocity.z < 0 ? -1 : 0);
            speedPerAngularVelocity = new Vector3(speedPerAngularVelocity.x == 0 ? 0.00000000000000000000000001f : Mathf.Abs(speedPerAngularVelocity.x), //prevent Division by Zeros for an orthogonal rotation axis
                                                  speedPerAngularVelocity.y == 0 ? 0.00000000000000000000000001f : Mathf.Abs(speedPerAngularVelocity.y),
                                                  speedPerAngularVelocity.z == 0 ? 0.00000000000000000000000001f : Mathf.Abs(speedPerAngularVelocity.z));
            Vector3 s = speedPerAngularVelocity;//short name
            float PVx = rb.angularVelocity.x * s.x * angularDirections.x;
            float PVy = rb.angularVelocity.y * s.y * angularDirections.y;
            float PVz = rb.angularVelocity.z * s.z * angularDirections.z;
            Debug.Log("Vel from rotations after Col: x=" + PVx + ", y=" + PVy + ", z=" + PVz);

        }
    }

    Vector3 GetAddedPointVelocity(Vector3 worldPoint)
    {
        Vector3 rot = rb.GetAccumulatedTorque() * Time.fixedDeltaTime;
        Vector3 vFromRotation = Vector3.Cross(new Vector3(rot.x/inertiaTensorWS.x,rot.y/inertiaTensorWS.y,rot.z/inertiaTensorWS.z), worldPoint - rb.centerOfMass); //THIS CROSS IS WRONG HERE!!!!
        Vector3 vFromDir = rb.GetAccumulatedTorque() * Time.fixedDeltaTime / rb.mass;


        return vFromRotation+vFromDir;
        //TODO: testen, ob das stimmt und danach in Car.cs einfügen
    }
}
