using System;
using System.Collections;
using System.Collections.Generic;
using UnityEditor.PackageManager;
//using Unity.VisualScripting;
//using UnityEditor;
using UnityEngine;
//using UnityEngine.UIElements;
//using static UnityEditor.Experimental.GraphView.GraphView;
//using static UnityEngine.UI.Image;


public class Car : MonoBehaviour
{
    [SerializeField] SpeedupCurve speedupCurve;
    public SpeedupCurve SpeedupCurve { get => speedupCurve; set => speedupCurve = value; }


    [SerializeField] bool showFrontWheelSettings;
    [SerializeField] Vector3 frontRightWheelCenter = new Vector3(1, -0.3f, 1);
    [SerializeField] float frontWheelRadius = 0.25f;
    [SerializeField] bool frontUse3DWheelPhysics = false;
    [SerializeField] float frontWheelInwardThickness = 0.1f;
    [SerializeField] float frontWheelOutwardThickness = 0;
    [SerializeField] int frontWheelShapeAccuracy = 4;
    [SerializeField] float frontWheelSuspensionDistanceToLiftCarWeight = 0.1f;
    [SerializeField] float frontWheelDamping = 0.4f;
    [SerializeField] float frontSuspHardCap = 0.2f;


    [SerializeField] bool showRearWheelSettings;
    [SerializeField] Vector3 rearRightWheelCenter = new Vector3(1, -0.3f, -1);
    [SerializeField] float rearWheelRadius = 0.25f;
    [SerializeField] bool rearUse3DWheelPhysics = false;
    [SerializeField] float rearWheelInwardThickness = 0.1f;
    [SerializeField] float rearWheelOutwardThickness = 0;
    [SerializeField] int rearWheelShapeAccuracy = 4;
    [SerializeField] float rearWheelSuspensionDistanceToLiftCarWeight = 0.1f;
    [SerializeField] float rearWheelDamping = 0.4f;
    [SerializeField] float rearSuspHardCap = 0.2f;

    [SerializeField] bool springsByDefaultGravity = true;
    [SerializeField] float springsByOtherValue = -10f;
    [SerializeField] float lateralAttackHeightLift = 0f; //AUSWIRKUNGEN NOCH NICHT IMPLEMENTIERT
    [SerializeField] float longitudalAttackHeightLift = 0f; //AUSWIRKUNGEN NOCH NICHT IMPLEMENTIERT
    [SerializeField] float frontAntiRollBar = 0f;
    [SerializeField] float rearAntiRollBar = 0f;
    //[SerializeField] float frontAntiRollBar = 0f;


    [SerializeField] LayerMask solidGround;
    [SerializeField] LayerMask looseGround;

    [SerializeField]
    private bool endlessFrontWheelGrip = false;
    [SerializeField]
    private bool endlessBackWheelGrip = false;
    [SerializeField] private Grip frontWheelGrip;
    [SerializeField] private Grip backWheelGrip;

    //public Vector3 FrontRightWheelCenter { get => frontRightWheelCenter;}
    //public Vector3 RearRightWheelCenter { get => rearRightWheelCenter; }
    //public float FrontWheelRadius { get => frontWheelRadius;}
    //public float RearWheelRadius { get => rearWheelRadius; }

    //public tester[] testers;


    //parameters to acess by array
    Vector3[] wheelCenters = new Vector3[4];
    Vector3[] wheelCentersStartPoint = new Vector3[4];
    Vector3[] wheelCentersEndPoint = new Vector3[4];
    Vector3[] wheel3DCenters = new Vector3[4];
    Vector3[] wheel3DCentersStartPoint = new Vector3[4];
    Vector3[] wheel3DCentersEndPoint = new Vector3[4];
    float[] wheel3DThicknesses = new float[4];
    float[] upwardSuspensionCap;
    Vector3[] upwardSuspensionCapVec;
    float[] looseSpringOffsetLength;
    float[] wheelRadii;
    float[] inwardWheelThicknesses;
    float[] outwardWheelThicknesses;
    //Dependent Parameters
    Rigidbody rb;
    float[] springPower = new float[4];
    
    //float frontSpringPower = 0;
    //float rearSpringPower = 0;
    float usedGravityForSprings = 0;
    LayerMask combinedGroundLayers;


    //Active Parameters
    float[] previousSpringCompressions = new float[4];





    // Start is called before the first frame update
    void Start()
    {
        UpdateDependendParameters();
        UpdateArrayAccessibleParameters();
    }

    public void UpdateDependendParameters() //Note: This Method is also called when you make changes in Inspector during play mode
    {
        rb = GetComponent<Rigidbody>();
        usedGravityForSprings = springsByDefaultGravity ? Physics.gravity.magnitude : Mathf.Abs(springsByOtherValue);
        float distCenterFrontWheel = Mathf.Abs(frontRightWheelCenter.z - rb.centerOfMass.z);
        float distCenterRearWheel = Mathf.Abs(rearRightWheelCenter.z - rb.centerOfMass.z);
        float frontSpringWeightRatio = distCenterRearWheel / (distCenterFrontWheel + distCenterRearWheel);
        float rearSpringWeightRatio = 1 - frontSpringWeightRatio;
        springPower[0] = usedGravityForSprings * frontSpringWeightRatio * 0.5f / frontWheelSuspensionDistanceToLiftCarWeight;
        springPower[1] = springPower[0];
        springPower[2] = usedGravityForSprings * rearSpringWeightRatio * 0.5f / rearWheelSuspensionDistanceToLiftCarWeight;
        springPower[3] = springPower[2];
        combinedGroundLayers = solidGround | looseGround;
    }

    public void UpdateArrayAccessibleParameters()
    {
        wheelCenters = new Vector3[] { frontRightWheelCenter + 2 * Vector3.left * frontRightWheelCenter.x, frontRightWheelCenter, rearRightWheelCenter + 2 * Vector3.left * rearRightWheelCenter.x, rearRightWheelCenter };
        Vector3 fr3DOffset = Vector3.right * (frontWheelOutwardThickness - frontWheelInwardThickness) / 2;
        Vector3 br3DOffset = Vector3.right * (rearWheelOutwardThickness - rearWheelInwardThickness) / 2;
        wheel3DCenters = new Vector3[] { wheelCenters[0] - fr3DOffset, wheelCenters[1] + fr3DOffset, wheelCenters[2] - br3DOffset, wheelCenters[3] + br3DOffset };
        upwardSuspensionCap = new float[] { frontSuspHardCap, frontSuspHardCap, rearSuspHardCap,rearSuspHardCap };
        upwardSuspensionCapVec = new Vector3[] { Vector3.up * frontSuspHardCap, Vector3.up * frontSuspHardCap, Vector3.up * rearSuspHardCap, Vector3.up * rearSuspHardCap };
        looseSpringOffsetLength = new float[] { frontWheelSuspensionDistanceToLiftCarWeight, frontWheelSuspensionDistanceToLiftCarWeight, rearWheelSuspensionDistanceToLiftCarWeight, rearWheelSuspensionDistanceToLiftCarWeight };
        wheelRadii = new float[] { frontWheelRadius, frontWheelRadius, rearWheelRadius, rearWheelRadius };
        inwardWheelThicknesses = new float[] {frontWheelInwardThickness, frontWheelInwardThickness, rearWheelInwardThickness, rearWheelInwardThickness };
        outwardWheelThicknesses = new float[] { frontWheelOutwardThickness, frontWheelOutwardThickness, rearWheelOutwardThickness, rearWheelOutwardThickness };
        for(int i=0; i < 4; i++)
        {
            wheelCentersStartPoint[i] = wheelCenters[i] + upwardSuspensionCapVec[i] + Vector3.up * wheelRadii[i];
            wheelCentersEndPoint[i] = wheelCenters[i] + Vector3.down * (looseSpringOffsetLength[i] + wheelRadii[i]);
            wheel3DCentersStartPoint[i] = wheel3DCenters[i] + upwardSuspensionCapVec[i] + Vector3.up * wheelRadii[i];
            wheel3DCentersEndPoint[i] = wheel3DCenters[i] + Vector3.down * looseSpringOffsetLength[i];
            wheel3DThicknesses[i] = i < 2 ? frontWheelInwardThickness + frontWheelOutwardThickness : rearWheelInwardThickness + rearWheelOutwardThickness;
        }
    }

    void Update() { }


    private void FixedUpdate()
    {
        float[] steeringAngles = new float[] {0,0,0,0};
        //testSteering !!!!!!!!!
        steeringAngles[0] = 30 * Input.GetAxis("Horizontal");
        steeringAngles[1] = 30 * Input.GetAxis("Horizontal");


        //float currentTime= Time.realtimeSinceStartup;
        Vector3[] hitPoints;
        Vector3[] hitNormals;
        float[] absoluteSpringCompressions;
        float[] springCompressions;
        int[] collidedGroundType;
        //Vector3 relUp = transform.rotation * Vector3.up;

        FindGroundInteraction(out hitPoints, out hitNormals, out absoluteSpringCompressions, out collidedGroundType);
        ApplyAntiRollBar(absoluteSpringCompressions, out springCompressions);

        float[] suspPowers = new float[4];
        for (int i = 0; i < 4; i++)
        {
            //SUSPENSION
            //suspension
            suspPowers[i] = springCompressions[i] * springPower[i];
            //damping
            float deltaSpringCompression = absoluteSpringCompressions[i] - previousSpringCompressions[i];
            float dampingCoefficient = i < 2 ? frontWheelDamping : rearWheelDamping;
            float dampingValue = dampingCoefficient * deltaSpringCompression / Time.fixedDeltaTime * springCompressions[i];
            Vector3 verticalForce = hitNormals[i] * (suspPowers[i] + dampingValue) * rb.mass;
            rb.AddForceAtPosition(verticalForce, hitPoints[i]);
            //handle hard bump when wheelSprings are compressed over the maximum
            if (absoluteSpringCompressions[i] > looseSpringOffsetLength[i] + upwardSuspensionCap[i])
            {

                //if it is not already in a state of extending the spring again
                if (Vector3.Angle( rb.GetPointVelocity(hitPoints[i]), hitNormals[i]) > 90)
                {
                    Vector3 angularChange;
                    Vector3 directionalChange;
                    //ImpulseNeededToStopDirectionalMovementAtPoint(hitPoints[i], hitNormals[i], out angularChange, out directionalChange, out _);    //TESTWEISE AUSKOMMENTIERT!!!!!!
                    //rb.angularVelocity += angularChange;
                    //rb.velocity += directionalChange;
                }

            }



            //SIDEWARD FRICTION
            //if (i == 0)
            //{
                Vector3 directionFoward = rb.rotation * (Quaternion.Euler(0, steeringAngles[i], 0) * Vector3.forward);
                Vector3 rOnGround = Vector3.Cross(hitNormals[i], directionFoward);
                //Debug.DrawRay(hitPoints[i], rOnGround.normalized);
                Vector3 slideDirection = (rOnGround * ((Vector3.Angle(rb.GetPointVelocity(hitPoints[i]), rOnGround) > 90) ? -1 : 1)).normalized;
                Debug.DrawRay(hitPoints[i], -slideDirection.normalized, Color.black);
                //if (i == 0) Debug.DrawRay(hitPoints[i], -slideDirection.normalized * 0.1f, Color.blue, 0.3f);
                //Debug.DrawRay(hitPoints[i], rb.GetPointVelocity(hitPoints[i]), Color.cyan);
                float maxStopImpuse = verticalForce.magnitude * 2.3f * Time.fixedDeltaTime;//verticalForce.magnitude*0.1f * Time.fixedDeltaTime; //TEST VALUE!!!
                Vector3 angularChange1;
                Vector3 directionalChange1;
                float neededImpulseToStop;
                ImpulseNeededToStopDirectionalMovementAtPoint(hitPoints[i], -slideDirection, out angularChange1, out directionalChange1, out neededImpulseToStop);
                //Debug.Log("maxStopImpulse= " + maxStopImpuse + " neededImpulseToStop=" + neededImpulseToStop);
                if (maxStopImpuse < neededImpulseToStop/3) //REMOVE /10
                {
                    rb.AddForceAtPosition(-slideDirection*maxStopImpuse, hitPoints[i], ForceMode.Impulse);

                    //rb.AddForceAtPosition(-slideDirection * neededImpulseToStop , hitPoints[i], ForceMode.Impulse);

                    //Debug.Log("currentImp= " + maxStopImpuse + "  to stop Imp= " + neededImpulseToStop);

                    //rb.angularVelocity += angularChange1; //TEST
                    //rb.velocity += directionalChange1; //TEST
                }
                else
                {
                    //Debug.Log("picked full stop");
                    rb.AddForceAtPosition(-slideDirection * neededImpulseToStop/3, hitPoints[i], ForceMode.Impulse); //REMOVE /10
                    //rb.angularVelocity += angularChange1;
                    //rb.velocity += directionalChange1;
                }
            //}

        }





        previousSpringCompressions = absoluteSpringCompressions;
        //float timeUsed = (Time.realtimeSinceStartup - currentTime) / Time.fixedDeltaTime;
        //Debug.Log("Timeratio used is " + timeUsed);




        //TESTDRIVING
        if (Input.GetKey(KeyCode.W))
        {
            for (int i = 0; i < 4; i++)
            {
                Vector3 sideDirectionR = rb.rotation * (Quaternion.Euler(0, steeringAngles[i], 0) * Vector3.right);
                Vector3 fowardOnGround = Vector3.Cross(sideDirectionR, hitNormals[i]).normalized;
                float speedAtWheel = Vector3.Dot(rb.GetPointVelocity(hitPoints[i]), fowardOnGround);
                Debug.DrawRay(hitPoints[i], fowardOnGround * speedAtWheel);
                Debug.Log("speedAtWheel" + i + " = " + speedAtWheel);
                float acceleration = speedupCurve.GetAccelerationValueOfSpeed(speedAtWheel);
                Debug.Log("acelerationAtWheel" + i + " = " + acceleration);
                rb.AddForceAtPosition(fowardOnGround * acceleration/4f, hitPoints[i]);
            }


            //rb.velocity +=  rb.rotation * Vector3.forward * Time.fixedDeltaTime * 4;
        }else if (Input.GetKey(KeyCode.S))
        {
            for (int i = 0; i < 4; i++)
            {
                Vector3 sideDirectionR = rb.rotation * (Quaternion.Euler(0, steeringAngles[i], 0) * Vector3.right);
                Vector3 fowardOnGround = Vector3.Cross(sideDirectionR, hitNormals[i]).normalized;
                float speedAtWheel = Vector3.Dot(rb.GetPointVelocity(hitNormals[i]), fowardOnGround);
                float acceleration = speedupCurve.GetAccelerationValueOfSpeed(speedAtWheel);
                rb.AddForceAtPosition(-fowardOnGround * acceleration / 4f, hitPoints[i]);
            }
        }
    }





    private void FindGroundInteraction(out Vector3[] hitPoints, out Vector3[] hitNormals, out float[] springCompressions, out int[] collidedGroundType)
    {
        hitPoints = new Vector3[4];
        hitNormals = new Vector3[4];
        springCompressions = new float[] {0,0,0,0};
        collidedGroundType = new int[] { 0, 0, 0, 0 }; //0 = no; 1 = solidGround; 2 = looseGround

        for(int i = 0; i < 4; i++)
        {
            bool use3DCollisions = i < 2 ? frontUse3DWheelPhysics : rearUse3DWheelPhysics;

            //Vector3 startPoint = rb.position + rb.rotation * (wheelCenters[i] + upwardsSuspensionCap[i]+ Vector3.up*wheelRadii[i]);


            if (!use3DCollisions)
            {
                //Vector3 endPoint = rb.position + rb.rotation * (wheelCenters[i] + Vector3.down * (looseSpringOffsetLength[i] + wheelRadii[i]));
                Vector3 startPoint = transform.TransformPoint(wheelCentersStartPoint[i]);
                Vector3 endPoint = transform.TransformPoint(wheelCentersEndPoint[i]);
                Vector3 startToEnd = endPoint - startPoint;
                RaycastHit hit;
                Ray ray = new Ray(startPoint, startToEnd);
                Debug.DrawRay(startPoint, startToEnd, UnityEngine.Color.red);
                if (Physics.Raycast(ray, out hit, startToEnd.magnitude, combinedGroundLayers))
                {
                    hitPoints[i] = hit.point;
                    hitNormals[i] = hit.normal;
                    //Debug.DrawRay(hit.point, hit.normal, UnityEngine.Color.green);
                    collidedGroundType[i] = (solidGround == (solidGround | (1 << hit.transform.gameObject.layer))) ? 1 : 2;
                    springCompressions[i] = (hit.point - endPoint).magnitude;
                }
            }
            else
            {
                //HIER KOMPLEXE 3D KOLLISIONEN
                Vector3 startPoint = transform.TransformPoint(wheel3DCentersStartPoint[i]);
                Vector3 endPoint = transform.TransformPoint(wheel3DCentersEndPoint[i]);
                Vector3 startToEnd = endPoint - startPoint;
                Vector3 localEndPoint = transform.InverseTransformPoint(endPoint);

                //localSpace Positions
                int usedShapeAccuracy = (i < 2 ? frontWheelShapeAccuracy : rearWheelShapeAccuracy);
                int side = i % 2 == 0 ? -1 : 1; //serves as positive multiplier for right side, negative for left side
                float degreeCoveredPerBoxCast = 180 / usedShapeAccuracy;


                float zScale = 2 * wheelRadii[i] * Mathf.Tan(degreeCoveredPerBoxCast / 2 * Mathf.Deg2Rad);

                //the goal is to find the cylinder-approximating-box, which would lead to the biggest spring compression, thus being at the highest ground relative to the circular wheel shape.
                for (int j = 0; j < usedShapeAccuracy; j++)
                {
                    float xRot = -90 + (j + 0.5f) * degreeCoveredPerBoxCast;
                    Vector3 boxCenter = startPoint + rb.rotation * (Quaternion.Euler(xRot, 0, 0) * Vector3.down * wheelRadii[i] * 0.5f); //+ localXOffsetVec); // WAS 0.01f before!!!!!
                    Debug.DrawRay(boxCenter, new Vector3(wheel3DThicknesses[i], wheelRadii[i], zScale), UnityEngine.Color.red, 0.02f);
                    //Gizmos.DrawCube(boxCenter, startToEnd);
                    //ExtDebug.DrawBoxCastBox(boxCenter, new Vector3(wheel3DThicknesses[i], wheelRadii[i], zScale),  rb.rotation * Quaternion.Euler(xRot, 0, 0), startToEnd, startToEnd.magnitude, Color.red);
                    ExtDebug.DrawBox(boxCenter + rb.rotation * (Vector3.down * wheelRadii[i] + upwardSuspensionCapVec[i]) , new Vector3(wheel3DThicknesses[i], wheelRadii[i], zScale)*0.5f, rb.rotation * Quaternion.Euler(xRot, 0, -1*side), Color.red);
                    //ExtDebug.DrawBox(rb.position, Vector3.one, rb.rotation, Color.blue);
                    RaycastHit hit;

                    if (Physics.BoxCast(boxCenter, new Vector3(wheel3DThicknesses[i], wheelRadii[i], zScale)*0.5f, startToEnd, out hit, rb.rotation * Quaternion.Euler(xRot, 0, -1 * side), startToEnd.magnitude, combinedGroundLayers))
                    {
                        //Debug.DrawRay(hit.point, hit.normal, UnityEngine.Color.blue);
                        Vector3 hitLocalSpace = transform.InverseTransformPoint(hit.point);
                        float zOffsetFromCenter = hitLocalSpace.z - wheelCenters[i].z;
                        //Debug.Log("zOffset= " + zOffsetFromCenter);
                        float lostCompressionHeightByZOffset = wheelRadii[i] - Mathf.Sqrt(wheelRadii[i] * wheelRadii[i] - zOffsetFromCenter * zOffsetFromCenter);
                        //Debug.Log("lostCompressionHeight" + lostCompressionHeightByZOffset);
                        float theoreticalSpringCompression = hitLocalSpace.y + wheelRadii[i] - localEndPoint.y - lostCompressionHeightByZOffset;
                        //Debug.Log("theoCompression" + i + "= " + theoreticalSpringCompression + "  prev= " + springCompressions[i]);
                        if (theoreticalSpringCompression > springCompressions[i])
                        {
                            springCompressions[i] = theoreticalSpringCompression;
                            hitPoints[i] = hit.point;
                            hitNormals[i] = hit.normal;
                            collidedGroundType[i] = (solidGround == (solidGround | (1 << hit.transform.gameObject.layer))) ? 1 : 2;
                            if(hit.rigidbody == rb) { Debug.LogError("A Wheel collided downward with its corresponding car. You should NOT pick a Layer as solidGround or looseGround which is part of your Car's colliders"); }
                        }
                    }
                }
            }
            Debug.DrawRay(hitPoints[i], hitNormals[i] * springCompressions[i], UnityEngine.Color.green);
            //Debug.Log("Spring compression[" + i + "]= " + springCompressions[i]);
        }
    }


    private void ApplyAntiRollBar(float[] absoluteSpringCompressions, out float[] springCompressions)
    {
        springCompressions = new float[4];

        for (int i = 0; i < 4; i+=2) {
            float combinedCompression = absoluteSpringCompressions[0+i] + absoluteSpringCompressions[1+i];

            if (combinedCompression == 0)
            {
                springCompressions[0 + i] = 0;
                springCompressions[1 + i] = 0;
                continue;
            }
            float usedRollbarValue = i == 0 ? frontAntiRollBar : rearAntiRollBar;
            float ratioL = absoluteSpringCompressions[0 + i] / combinedCompression;
            float ratioR = absoluteSpringCompressions[1 + i] / combinedCompression;
            springCompressions[0 + i] = absoluteSpringCompressions[0 + i] * (1 + (ratioL - 0.5f) * usedRollbarValue*2);
            springCompressions[1 + i] = absoluteSpringCompressions[1 + i] * (1 + (ratioR - 0.5f) * usedRollbarValue*2);
        }
    }

    private void ImpulseNeededToStopDirectionalMovementAtPoint(Vector3 Point, Vector3 ImpulseDirToCancleCurrent, out Vector3 angularChange, out Vector3 directionalChange, out float neededImpulse)
    {
        //part of the currentSpeed going in opposide direction as the counteracting impulse 
        float neededV = Mathf.Abs(Vector3.Dot(rb.GetPointVelocity(Point), -ImpulseDirToCancleCurrent.normalized));
        if (neededV == 0)
        {
            neededImpulse = 0;
            angularChange = Vector3.zero;
            directionalChange = Vector3.zero;
            return;
        }
        //Debug.Log("Point vel:" + rb.GetPointVelocity(Point));
        //Debug.Log("vel to cancle out: " + neededV);

        ////in local Space, because Unitys inertiaTensor is in local space   //OLD LOCAL SPACE   <----
        //Vector3 localPoint = transform.InverseTransformPoint(Point);
        ////localPoint = new Vector3(localPoint.x * transform.localScale.x, localPoint.y * transform.localScale.y, localPoint.z * transform.localScale.z);
        //Vector3 localDir = transform.InverseTransformDirection(ImpulseDirToCancleCurrent.normalized);
        //Vector3 speedPerAngularVelocity = Vector3.Cross(localPoint, localDir); //how much one unit roation around each axis would move the point in direction of force


        Vector3 speedPerAngularVelocity = Vector3.Cross(Point - rb.worldCenterOfMass, ImpulseDirToCancleCurrent.normalized); //how much one unit roation around each axis would move the point in direction of force
        Vector3 angularDirections = new Vector3(speedPerAngularVelocity.x > 0 ? 1 : speedPerAngularVelocity.x < 0 ? -1 : 0,
                                                speedPerAngularVelocity.y > 0 ? 1 : speedPerAngularVelocity.y < 0 ? -1 : 0,
                                                speedPerAngularVelocity.z > 0 ? 1 : speedPerAngularVelocity.z < 0 ? -1 : 0);
        speedPerAngularVelocity = new Vector3(speedPerAngularVelocity.x == 0 ? 0.00000000000000000000000001f : Mathf.Abs(speedPerAngularVelocity.x), //prevent Division by Zeros for an orthogonal rotation axis
                                              speedPerAngularVelocity.y == 0 ? 0.00000000000000000000000001f : Mathf.Abs(speedPerAngularVelocity.y),
                                              speedPerAngularVelocity.z == 0 ? 0.00000000000000000000000001f : Mathf.Abs(speedPerAngularVelocity.z));
        //Debug.Log("speedPerAngularVelocity = " + speedPerAngularVelocity);
        if (rb.inertiaTensor.x == 0 || rb.inertiaTensor.y == 0 || rb.inertiaTensor.z == 0) Debug.LogError("It is not allowed to lock the rigidbodys rotation while using this script");


        Vector3 s = speedPerAngularVelocity;//short name
        Vector3 j = Quaternion.Inverse(rb.rotation) * (rb.inertiaTensor); //inertiaTensor in global Space // WARNING: this might be wrong!!!
        j = new Vector3(Mathf.Abs(j.x), Mathf.Abs(j.y), Mathf.Abs(j.z));             // ""
        //Vector3 j = rb.inertiaTensor; //short name


        //current parts from rotational(xyz) and directional movement, that move against the direction-to-cancle-out at the relevant position
        float PVx = rb.angularVelocity.x * s.x * angularDirections.x;
        float PVy = rb.angularVelocity.y * s.y * angularDirections.y;
        float PVz = rb.angularVelocity.z * s.z * angularDirections.z;
        float PVd = Vector3.Dot(rb.velocity, ImpulseDirToCancleCurrent.normalized);
        //Debug.Log("neededV=" + neededV + " all PV added =" + (PVx + PVy + PVz + PVd));

        //all following interim results happen to appear very often in the main final equation, so they are calculated once here
        float Mx = j.x / (s.x * s.x);
        float My = j.y / (s.y * s.y);
        float Mz = j.z / (s.z * s.z);
        float Md = rb.mass;
        //interim result
        float PId = Md * PVd; //previous directional Impulse
        //interim results
        float MdDivMx = Md / Mx;
        float MdDivMy = Md / My;
        float MdDivMz = Md / Mz;
        //interim results
        float PIdDivMx = PId / Mx; //previouse directional Impulse divided by mass-equivalent of xRotation
        float PIdDivMy = PId / My;
        float PIdDivMz = PId / Mz;
        //interim result
        float leftoverVd = neededV + PVx + PVy + PVz;

        //wanted delta velocities
        float d; 
        float x;
        float y;
        float z;

        //unfortunately each of the 4 parts of the formular is different depending if the new velocity leads to a direction-change of the rotational axis or directional movement. It is only known which forular part is correct AFTER calculating it
        //the formular part for a direction-change is only possible when the corresponding previous v is negative, therefor unnecesarry calculations are prevented by "...&& PV_ < 0)"
        for (int dTwist=1; dTwist == 1 || (dTwist == -1 && PVd < 0); dTwist-=2)
        {
            for (int xTwist = 1; xTwist == 1 || (xTwist == -1 && PVx < 0); xTwist -= 2)
            {
                for (int yTwist = 1; yTwist == 1 || (yTwist == -1 && PVy < 0); yTwist -= 2)
                {
                    for (int zTwist = 1; zTwist == 1 || (zTwist == -1 && PVz < 0); zTwist -= 2)
                    {
                        d = (leftoverVd - dTwist * (xTwist * PIdDivMx + yTwist * PIdDivMy + zTwist * PIdDivMz)) / (1 + xTwist * MdDivMx + yTwist * MdDivMy + zTwist * MdDivMz);
                        if (d < 0) continue; //force is not supposed to cause a delta velocity into the opposide direction
                        if (PVd < 0 && dTwist == 1 && PVd + d > 0) continue; //can not apply the formular for no-direction-change (by having the multiplier dTwist set to 1) if it turns out to cause a direction change
                        if (dTwist == -1 && PVd + d < 0) continue; //can not apply direction-change-formular (by dTwist = -1) if it turns out not to cause a direction change
                        x = dTwist * xTwist * (d * MdDivMx + PIdDivMx) - PVx;
                        if (x < 0) continue;
                        if (PVx < 0 && xTwist == 1 && PVx + x > 0) continue;
                        if (xTwist == -1 && PVx + x < 0) continue;
                        y = dTwist * yTwist * (d * MdDivMy + PIdDivMy) - PVy;
                        if (y < 0) continue;
                        if (PVy < 0 && yTwist == 1 && PVy + y > 0) continue;
                        if (yTwist == -1 && PVy + y < 0) continue;
                        z = dTwist * zTwist * (d * MdDivMz + PIdDivMz) - PVz;
                        if (z < 0) continue;
                        if (PVz < 0 && zTwist == 1 && PVz + z > 0) continue;
                        if (zTwist == -1 && PVz + z < 0) continue;
                    }
                }
            }
        }








        float multiplierAllSpeeds = 2 * j.x * j.y * j.z * Md * neededV / (s.x * s.x * j.y * j.z * Md + s.y * s.y * j.x * j.z * Md + s.z * s.z * j.x * j.y * Md + j.x * j.y * j.z);
        multiplierAllSpeeds = Mathf.Abs(multiplierAllSpeeds);
        //speed provided by each rotation
        float vx = (s.x * s.x) / (2 * j.x) * multiplierAllSpeeds;
        float vy = (s.y * s.y) / (2 * j.y) * multiplierAllSpeeds;
        float vz = (s.z * s.z) / (2 * j.z) * multiplierAllSpeeds;
        float vdir = 1 / (2 * Md) * multiplierAllSpeeds;


        //angular Velocities
        float wx = vx / s.x;
        float wy = vy / s.y;
        float wz = vz / s.z;
        //Debug.Log("vx= " + vx + "  vy= " + vy + "  vz= " + vz + "  vdir= " + vdir);
        //Debug.Log("wx= " + wx + "  wy= " + wy + "  wz= " + wz + "  vdir= "+vdir + "  allMultiplier= "+multiplierAllSpeeds+"  neededV= "+neededV);
        //Debug.Log("neededFullMassImpulse= " + (neededV * mass));
        //kinetic Energy of each speed
        float kx = 0.5f * j.x * Mathf.Pow(wx, 2);
        float ky = 0.5f * j.y * Mathf.Pow(wy, 2);
        float kz = 0.5f * j.z * Mathf.Pow(wz, 2); //2
        float kdir = 0.5f * Md * Mathf.Pow(vdir, 2); //2
        float totalK = kx + ky + kz + kdir; //4  //2
        float scaledK = Mathf.Sqrt(Mathf.Pow(kx, 2) + Mathf.Pow(ky, 2) + Mathf.Pow(kz, 2) + Mathf.Pow(kdir, 2));
        float scaler = scaledK / totalK;
        neededImpulse = (wx * j.x + wy * j.y + wz * j.z + vdir * Md) * scaler;
        //needsCancle = neededImpulse < ImpulseDirToCancleCurrent.magnitude;
        angularChange = transform.TransformDirection(new Vector3(wx * angularDirections.x, wy * angularDirections.y, wz * angularDirections.z));
        directionalChange = ImpulseDirToCancleCurrent.normalized * vdir;
        //Debug.Log("calcedImpulse= " + neededImpulse);
    }



}
