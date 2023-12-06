using System;
using System.Collections;
using System.Collections.Generic;
using UnityEditor.PackageManager;
//using Unity.VisualScripting;
//using UnityEditor;
using UnityEngine;
using UnityEngine.UIElements;
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

    Vector3 inertiaTensorWS = Vector3.one;



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
        Debug.Log("newFramePointVel = " + rb.GetPointVelocity(Vector3.one));
        inertiaTensorWS = GetInertiaTensorInWorldSpace();
        //Debug.Log("inertiaTensorLocal: " + rb.inertiaTensor + ",  in WS: " + inertiaTensorWS);
        float[] steeringAngles = new float[] { 0, 0, 0, 0 };
        //testSteering !!!!!!!!!
        steeringAngles[0] = 20 * Input.GetAxis("Horizontal");
        steeringAngles[1] = 20 * Input.GetAxis("Horizontal");


        //float currentTime= Time.realtimeSinceStartup;
        Vector3[] hitPoints;
        Vector3[] hitNormals;
        Vector3[] hitPointForLateral;
        Vector3[] hitPointForLongitudal;
        float[] absoluteSpringCompressions;
        float[] springCompressions;
        int[] collidedGroundType;
        //Vector3 relUp = transform.rotation * Vector3.up;

        FindGroundInteraction(out hitPoints, out hitNormals, out absoluteSpringCompressions, out collidedGroundType);
        hitPointForLateral = new Vector3[] { GetLiftedPoint(hitPoints[0], lateralAttackHeightLift), GetLiftedPoint(hitPoints[1], lateralAttackHeightLift) , GetLiftedPoint(hitPoints[2], lateralAttackHeightLift) , GetLiftedPoint(hitPoints[3], lateralAttackHeightLift) };
        hitPointForLongitudal = new Vector3[] { GetLiftedPoint(hitPoints[0], longitudalAttackHeightLift), GetLiftedPoint(hitPoints[1], longitudalAttackHeightLift), GetLiftedPoint(hitPoints[2], longitudalAttackHeightLift), GetLiftedPoint(hitPoints[3], longitudalAttackHeightLift) };

        ApplyAntiRollBar(absoluteSpringCompressions, out springCompressions);

        float[] suspPowers = new float[4];
        Vector3[] verticalForces = new Vector3[4];
        Vector3[] verticalForcesWithoutDamping = new Vector3[4];
        for (int i = 0; i < 4; i++)
        {
            //SUSPENSION
            //suspension
            suspPowers[i] = springCompressions[i] * springPower[i];
            //damping
            float deltaSpringCompression = absoluteSpringCompressions[i] - previousSpringCompressions[i];
            float dampingCoefficient = i < 2 ? frontWheelDamping : rearWheelDamping;
            float dampingValue = dampingCoefficient * deltaSpringCompression / Time.fixedDeltaTime * springCompressions[i];
            verticalForcesWithoutDamping[i] = hitNormals[i] * suspPowers[i] * rb.mass;
            verticalForces[i] = hitNormals[i] * (suspPowers[i] + dampingValue) * rb.mass;
            rb.AddForceAtPosition(verticalForces[i], hitPoints[i]);
            //handle hard bump when wheelSprings are compressed over the maximum
            if (absoluteSpringCompressions[i] > looseSpringOffsetLength[i] + upwardSuspensionCap[i])
            {

                //if it is not already in a state of extending the spring again
                if (Vector3.Angle(rb.GetPointVelocity(hitPoints[i]), hitNormals[i]) > 90)
                {
                    Vector3 angularChange;
                    Vector3 directionalChange;
                    //ImpulseNeededToStopDirectionalMovementAtPoint(hitPoints[i], hitNormals[i], out angularChange, out directionalChange, out _);    //TESTWEISE AUSKOMMENTIERT!!!!!!
                    //rb.angularVelocity += angularChange;
                    //rb.velocity += directionalChange;
                }

            }



            

        }

        //TESTDRIVING
        Vector3[] fowardOnGround = new Vector3[4];
        float[] speedAtWheel = new float[4];
        for (int i = 0; i < 4; i++)
        {
            Vector3 sideDirectionR = rb.rotation * (Quaternion.Euler(0, steeringAngles[i], 0) * Vector3.right);
            fowardOnGround[i] = Vector3.Cross(sideDirectionR, hitNormals[i]).normalized;
            speedAtWheel[i] = Vector3.Dot(GetActualPointVelocity(hitPointForLongitudal[i]), fowardOnGround[i]);
            Debug.DrawRay(hitPointForLongitudal[i], fowardOnGround[i] * speedAtWheel[i]);
        }
        float averatgeWheelSpeed = (speedAtWheel[0] + speedAtWheel[1] + speedAtWheel[2] + speedAtWheel[3]) / 4f;
        if (Input.GetKey(KeyCode.W))
        {
            float acceleration = speedupCurve.GetAccelerationValueOfSpeed(Mathf.Abs(averatgeWheelSpeed)); //Remove Mathf.abs and replace with other curve later!!
            for (int i = 0; i < 4; i++)
            {
                rb.AddForceAtPosition(fowardOnGround[i] * acceleration / 4f, hitPointForLongitudal[i]);
            }


            //rb.velocity +=  rb.rotation * Vector3.forward * Time.fixedDeltaTime * 4;
        }
        else if (Input.GetKey(KeyCode.S))
        {
            float acceleration = speedupCurve.GetAccelerationValueOfSpeed(Mathf.Abs(averatgeWheelSpeed)); //Remove Mathf.abs and replace with other curve later!!
            for (int i = 0; i < 4; i++)
            {
                rb.AddForceAtPosition(-fowardOnGround[i] * acceleration / 4f, hitPointForLongitudal[i]);
            }
        }


        //SIDEWARD FRICTION
        Vector3[] slideDirections = new Vector3[4];
        float[] slideSpeeds = new float[4];
        (float maxV, float maxVDir, Vector3 maxW, Vector3 rotToV, Vector3 vDirection)[] impulseProperties = new (float maxV, float maxVDir, Vector3 maxW, Vector3 rotToV, Vector3 vDirection)[4];
        for (int i = 0; i < 4; i++)
        {
            //SIDEWARD FRICTION
            
            Vector3 directionFoward = rb.rotation * (Quaternion.Euler(0, steeringAngles[i], 0) * Vector3.forward);
            Vector3 rOnGround = Vector3.Cross(hitNormals[i], directionFoward);
            //Debug.DrawRay(hitPoints[i], rOnGround.normalized);
            slideDirections[i] = (rOnGround * ((Vector3.Angle(GetActualPointVelocity(hitPointForLateral[i]), rOnGround) > 90) ? -1 : 1)).normalized;
            slideSpeeds[i] = Vector3.Dot(GetActualPointVelocity(hitPointForLateral[i]), slideDirections[i]); // CHECK IF THIS IS CORRECT!!!!
            Debug.DrawRay(hitPointForLateral[i], -slideDirections[i].normalized, Color.black);
            //if (i == 0) Debug.DrawRay(hitPoints[i], -slideDirection.normalized * 0.1f, Color.blue, 0.3f);
            //Debug.DrawRay(hitPoints[i], rb.GetPointVelocity(hitPoints[i]), Color.cyan);

            float maxStopImpulse;
            if ((i < 2 && endlessFrontWheelGrip) || (i >= 2 && endlessBackWheelGrip))
            {
                maxStopImpulse = slideSpeeds[i]* rb.mass / 4; 

            }
            else
            {
                maxStopImpulse = verticalForces[i].magnitude * 20.9f * Time.fixedDeltaTime;//verticalForce.magnitude*0.1f * Time.fixedDeltaTime; //TEST VALUE!!!

            }
           
            impulseProperties[i] = ImpulseNeededToStopDirectionalMovementAtPoint(hitPointForLateral[i], -slideDirections[i], maxStopImpulse); // , out angularChange1, out directionalChange1, out neededImpulseToStop);
                                                                                                                                    //Debug.Log("IMP PROBS[" + i + "]  maxV=" + impulseProperties[i].maxV + ", maxVDir=" + impulseProperties[i].maxVDir + ", maxW=" + impulseProperties[i].maxW + ", rotToV=" + impulseProperties[i].rotToV + ", vDirection=" + impulseProperties[i].vDirection);

            //Vector3 angularChange1;
            //Vector3 directionalChange1;
            //float neededImpulseToStop;
            //Debug.Log("maxStopImpulse= " + maxStopImpuse + " neededImpulseToStop=" + neededImpulseToStop);
            //if (maxStopImpuse < neededImpulseToStop * 0.25f) //REMOVE /10
            //{
            //    rb.AddForceAtPosition(-slideDirections[i] * maxStopImpuse, hitPoints[i], ForceMode.Impulse);

            //    //rb.AddForceAtPosition(-slideDirection * neededImpulseToStop , hitPoints[i], ForceMode.Impulse);

            //    //Debug.Log("currentImp= " + maxStopImpuse + "  to stop Imp= " + neededImpulseToStop);

            //    //rb.angularVelocity += angularChange1; //TEST
            //    //rb.velocity += directionalChange1; //TEST
            //}
            //else
            //{
            //    //Debug.Log("picked full stop");
            //    //rb.AddForceAtPosition(-slideDirection * neededImpulseToStop * 0.4f, hitPoints[i], ForceMode.Impulse); //REMOVE /10
            //    //rb.angularVelocity += angularChange1;
            //    //rb.velocity += directionalChange1;
            //    rb.AddForceAtPosition(-slideDirection * maxStopImpuse, hitPoints[i], ForceMode.Impulse); // TEST to always use maximum
            //}
        }
        float[,] maxsConversionsToMyV = new float[4,4]; //How much would the maximum impulse of another Wheel impact my own directional velocity [toMyV,fromOtherRatio]
        for(int i = 0; i < 4; i++)
        {
            //Debug.Log("maxVTotal= " + impulseProperties[i].maxV);
            //Debug.Log("maxVDir= " + impulseProperties[i].maxVDir);
            //Debug.Log("maxV From OWN rotation= " + (impulseProperties[i].maxW.x * impulseProperties[i].rotToV.x + impulseProperties[i].maxW.y * impulseProperties[i].rotToV.y + impulseProperties[i].maxW.z * impulseProperties[i].rotToV.z));
            for (int j = 0; j < 4; j++)
            {
                if (i == j)
                {
                    maxsConversionsToMyV[i,j] = 0; //exclude conversion from self
                }
                else if (impulseProperties[j].maxV == 0)
                {
                    maxsConversionsToMyV[i, j] = 0; //no conversion from wheel, which is not on ground
                }
                else
                {
                    float fromOtherMaxV = Vector3.Dot(impulseProperties[j].maxVDir * impulseProperties[j].vDirection , impulseProperties[i].vDirection.normalized); //Projection of other directionalV on my directionalV
                    //Debug.Log("fromOtherV: " + fromOtherMaxV);
                    float fromOtherMaxW = impulseProperties[j].maxW.x * impulseProperties[i].rotToV.x + impulseProperties[j].maxW.y * impulseProperties[i].rotToV.y + impulseProperties[j].maxW.z * impulseProperties[i].rotToV.z;
                    maxsConversionsToMyV[i,j] = fromOtherMaxV + fromOtherMaxW;
                }
                //Debug.Log("maxConversionsToMyV[" + i + "," + j + "]=" + maxsConversionsToMyV[i, j]);
            }
        }
        float[] plannedRatios = new float[4] {0,0,0,0};
        float[] prevIterationPR = new float[4] {-2,-2,-2,-2};
        //every wheels Friction is limited to only stop the wheels lateral movement and can not make it move into the opposide direction
        //this loop chooses this correct possible friction impulse with respect to all interactions with other wheels
        for (int iterationCount = 0; iterationCount < 100; iterationCount++)
        {

            for (int i = 0; i < 4; i++)
            {
                if (impulseProperties[i].maxV == 0) //ignore Wheels without friction/ without contact to floor
                {
                    plannedRatios[i] = 0;
                    continue;
                }
                float vFromOtherWheels = maxsConversionsToMyV[i,0] * plannedRatios[0] + maxsConversionsToMyV[i, 1] * plannedRatios[1] + maxsConversionsToMyV[i, 2] * plannedRatios[2] + maxsConversionsToMyV[i, 3] * plannedRatios[3];
                float neededOwnV = slideSpeeds[i] - vFromOtherWheels;
                plannedRatios[i] = neededOwnV / impulseProperties[i].maxV; //* 0.9f; //DELETE *0.9f LATER!!!!
                if (plannedRatios[i] > 1) plannedRatios[i] = 1;
                if (plannedRatios[i] < -1) plannedRatios[i] = -1;
            }

            if (plannedRatios[0] == prevIterationPR[0] && plannedRatios[1] == prevIterationPR[1] && plannedRatios[2] == prevIterationPR[2] && plannedRatios[3] == prevIterationPR[3])
            {
                //Debug.Log("Left after "+iterationCount+" iterations");
                break;
            }
            //if (iterationCount == 99) Debug.LogError("exit iteration with emergency exit, prevIterationPR=" + prevIterationPR + " plannedRatios=" + plannedRatios);
            prevIterationPR[0] = plannedRatios[0]; prevIterationPR[1] = plannedRatios[1]; prevIterationPR[2] = plannedRatios[2]; prevIterationPR[3] = plannedRatios[3];
        }

        //Apply friction-impulses of all wheels
        Vector3 angularVelChange = Vector3.zero;
        Vector3 directionalVelChange = Vector3.zero;
        for (int i = 0; i < 4; i++)
        {
            angularVelChange += plannedRatios[i] * impulseProperties[i].maxW;
            directionalVelChange += plannedRatios[i] * impulseProperties[i].maxVDir * impulseProperties[i].vDirection;
            //onlyForDebug
            float vFromOtherWheels = maxsConversionsToMyV[i, 0] * plannedRatios[0] + maxsConversionsToMyV[i, 1] * plannedRatios[1] + maxsConversionsToMyV[i, 2] * plannedRatios[2] + maxsConversionsToMyV[i, 3] * plannedRatios[3];
            //Debug.Log("estimatedV[" + i + "] with ratio " + plannedRatios[i] + ", slideSpeed="+slideSpeeds[i] +", selfV="+ impulseProperties[i].maxV * plannedRatios[i] +", vFromOthers="+ vFromOtherWheels +",   ->v: "+ (impulseProperties[i].maxV * plannedRatios[i]+ vFromOtherWheels- slideSpeeds[i]));
        }
        //Debug.Log("planned Ratios: " + plannedRatios[0] + ", " + plannedRatios[1] + ", " + plannedRatios[2] + ", " + plannedRatios[3]);
        //Debug.Log("angularChange= " + angularVelChange + ",   directionalChange=" + directionalVelChange);
        rb.AddTorque(new Vector3(angularVelChange.x*inertiaTensorWS.x, angularVelChange.y * inertiaTensorWS.y, angularVelChange.z * inertiaTensorWS.z), ForceMode.Impulse);
        rb.AddForce(directionalVelChange * rb.mass, ForceMode.Impulse);


        previousSpringCompressions = absoluteSpringCompressions;
        //float timeUsed = (Time.realtimeSinceStartup - currentTime) / Time.fixedDeltaTime;
        //Debug.Log("Timeratio used is " + timeUsed);



        Debug.Log("estimatedVelNextFrame = " + GetActualPointVelocity(Vector3.one));
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

    private (float maxV, float maxVDir, Vector3 maxW, Vector3 rotToV, Vector3 vDirection) ImpulseNeededToStopDirectionalMovementAtPoint(Vector3 hitPoint, Vector3 ImpulseDirToCancleCurrent, float impulsePower) //, out Vector3 angularChange, out Vector3 directionalChange, out float neededImpulse)
    {
        //part of the currentSpeed going in opposide direction as the counteracting impulse 
        //float neededV = Mathf.Abs(Vector3.Dot(rb.GetPointVelocity(Point), -ImpulseDirToCancleCurrent.normalized));
        //if (neededV == 0)
        //{
        //    neededImpulse = 0;
        //    angularChange = Vector3.zero;
        //    directionalChange = Vector3.zero;
        //    return;
        //}
        //Debug.Log("Point vel:" + rb.GetPointVelocity(Point));
        //Debug.Log("vel to cancle out: " + neededV);

        ////in local Space, because Unitys inertiaTensor is in local space   //OLD LOCAL SPACE   <----
        //Vector3 localPoint = transform.InverseTransformPoint(Point);
        ////localPoint = new Vector3(localPoint.x * transform.localScale.x, localPoint.y * transform.localScale.y, localPoint.z * transform.localScale.z);
        //Vector3 localDir = transform.InverseTransformDirection(ImpulseDirToCancleCurrent.normalized);
        //Vector3 speedPerAngularVelocity = Vector3.Cross(localPoint, localDir); //how much one unit roation around each axis would move the point in direction of force


        Vector3 speedPerAngularVelocity = Vector3.Cross(hitPoint - rb.worldCenterOfMass, ImpulseDirToCancleCurrent.normalized); //how much one unit roation around each axis would move the point in direction of force
        Vector3 angularDirections = new Vector3(speedPerAngularVelocity.x > 0 ? 1 : speedPerAngularVelocity.x < 0 ? -1 : 0,
                                                speedPerAngularVelocity.y > 0 ? 1 : speedPerAngularVelocity.y < 0 ? -1 : 0,
                                                speedPerAngularVelocity.z > 0 ? 1 : speedPerAngularVelocity.z < 0 ? -1 : 0);
        speedPerAngularVelocity = new Vector3(speedPerAngularVelocity.x == 0 ? 0.00000000000000000000000001f : Mathf.Abs(speedPerAngularVelocity.x), //prevent Division by Zeros for an orthogonal rotation axis
                                              speedPerAngularVelocity.y == 0 ? 0.00000000000000000000000001f : Mathf.Abs(speedPerAngularVelocity.y),
                                              speedPerAngularVelocity.z == 0 ? 0.00000000000000000000000001f : Mathf.Abs(speedPerAngularVelocity.z));
        //Debug.Log("speedPerAngularVelocity = " + speedPerAngularVelocity);
        if (rb.inertiaTensor.x == 0 || rb.inertiaTensor.y == 0 || rb.inertiaTensor.z == 0) Debug.LogError("It is not allowed to lock the rigidbodys rotation while using this script");


        Vector3 s = speedPerAngularVelocity;//short name
        Vector3 j = inertiaTensorWS; //short name



        //all following interim results happen to appear very often in the main final equation, so they are calculated once here
        //float Mx = j.x / (s.x * s.x);
        //float My = j.y / (s.y * s.y);
        //float Mz = j.z / (s.z * s.z);
        float Md = rb.mass;



        //float multiplierAllSpeeds = 2 * j.x * j.y * j.z * Md * neededV / (s.x * s.x * j.y * j.z * Md + s.y * s.y * j.x * j.z * Md + s.z * s.z * j.x * j.y * Md + j.x * j.y * j.z);
        //multiplierAllSpeeds = Mathf.Abs(multiplierAllSpeeds);
        ////speed provided by each rotation
        //float vx = (s.x * s.x) / (2 * j.x) * multiplierAllSpeeds;
        //float vy = (s.y * s.y) / (2 * j.y) * multiplierAllSpeeds;
        //float vz = (s.z * s.z) / (2 * j.z) * multiplierAllSpeeds;
        //float vdir = 1 / (2 * Md) * multiplierAllSpeeds;



        ////relative speed Split:         //OLD PROPABLY DELETE COMPLETELY
        //float vx = (s.x * s.x) / j.x;
        //float vy = (s.y * s.y) / j.y;
        //float vz = (s.z * s.z) / j.z;
        //float vdir = 1 / Md;
        //float totalImpulseCost = vx * j.x / s.x + vy * j.y / s.y + vz * j.z / s.z + 1; //the 1 being vdir*md;

        
        //relative impulse putten into the v
        float relImpulseX = s.x / j.x;
        float relImpulseY = s.y / j.y;
        float relImpulseZ = s.z / j.z;
        float relImpulseDir = 1 / Md;
        float scaler = impulsePower / (relImpulseX + relImpulseY + relImpulseZ + relImpulseDir);
        float impulseX = relImpulseX * scaler;
        float impulseY = relImpulseY * scaler;
        float impulseZ = relImpulseZ * scaler;
        float impulseDir = impulsePower;

        //angular Velocities
        float wx = angularDirections.x * impulseX / j.x;
        float wy = angularDirections.y * impulseY / j.y;
        float wz = angularDirections.z * impulseZ / j.z;

        float vx = impulseX * s.x / j.x;
        float vy = impulseY * s.y / j.y;
        float vz = impulseZ * s.z / j.z;
        float vdir = impulseDir / Md;
        float totalV = vx + vy + vz + vdir;

        //(float maxV, Vector3 maxW, Vector3 rotToV) output = (totalV, new Vector3(wx, wy, wz), new Vector3(s.x*angularDirections.x, s.y * angularDirections.y, s.y * angularDirections.y));
        return (totalV, vdir, new Vector3(wx, wy, wz), new Vector3(s.x * angularDirections.x, s.y * angularDirections.y, s.z * angularDirections.z), ImpulseDirToCancleCurrent.normalized);



        ////Debug.Log("vx= " + vx + "  vy= " + vy + "  vz= " + vz + "  vdir= " + vdir);
        ////Debug.Log("wx= " + wx + "  wy= " + wy + "  wz= " + wz + "  vdir= "+vdir + "  allMultiplier= "+multiplierAllSpeeds+"  neededV= "+neededV);
        ////Debug.Log("neededFullMassImpulse= " + (neededV * mass));

        ////kinetic Energy of each speed
        //float kx = 0.5f * j.x * Mathf.Pow(wx, 2);
        //float ky = 0.5f * j.y * Mathf.Pow(wy, 2);
        //float kz = 0.5f * j.z * Mathf.Pow(wz, 2); //2
        ////float kdir = 0.5f * Md * Mathf.Pow(vdir, 2); //2
        //float kdir = 0.5f * Md * Mathf.Pow(vdir, 2);
        //float totalK = kx + ky + kz + kdir; //4  //2
        //float scaledK = Mathf.Sqrt(Mathf.Pow(kx, 2) + Mathf.Pow(ky, 2) + Mathf.Pow(kz, 2) + Mathf.Pow(kdir, 2));
        //float scaler = scaledK / totalK;
        ////neededImpulse = (wx * j.x + wy * j.y + wz * j.z + vdir * Md) * scaler;
        //neededImpulse = (wx * j.x + wy * j.y + wz * j.z + vdir * Md) * scaler;
        ////needsCancle = neededImpulse < ImpulseDirToCancleCurrent.magnitude;
        ////angularChange = transform.TransformDirection(new Vector3(wx * angularDirections.x, wy * angularDirections.y, wz * angularDirections.z));
        //angularChange = new Vector3(wx * angularDirections.x, wy * angularDirections.y, wz * angularDirections.z);
        ////directionalChange = ImpulseDirToCancleCurrent.normalized * vdir;
        //directionalChange = ImpulseDirToCancleCurrent.normalized * vdir;
        ////Debug.Log("calcedImpulse= " + neededImpulse);

    }

    //based on Nathan Reeds answer on https://gamedev.stackexchange.com/questions/70355/inertia-tensor-and-world-coordinate-conversion (05.12.23)
    Vector3 GetInertiaTensorInWorldSpace()
    {
        // Erhalte die Rotationsmatrix der Welt
        Matrix4x4 worldRotationMatrix = Matrix4x4.Rotate(rb.rotation);

        // Erhalte die lokale Trägheitstensor-Matrix
        Matrix4x4 localInertiaTensor = Matrix4x4.zero;
        localInertiaTensor.m00 = rb.inertiaTensor.x;
        localInertiaTensor.m11 = rb.inertiaTensor.y;
        localInertiaTensor.m22 = rb.inertiaTensor.z;

        // Transformiere den lokalen Trägheitstensor in Weltkoordinaten
        Matrix4x4 worldInertiaTensorMatrix = worldRotationMatrix * localInertiaTensor * worldRotationMatrix.inverse;

        // Extrahiere die skalierten Hauptachsen
        Vector3 worldInertiaTensor = new Vector3(worldInertiaTensorMatrix.m00, worldInertiaTensorMatrix.m11, worldInertiaTensorMatrix.m22);

        return worldInertiaTensor;
    }

    Vector3 GetActualPointVelocity(Vector3 point)
    {
        Vector3 pointVelDir = rb.velocity + rb.GetAccumulatedForce() * Time.fixedDeltaTime / rb.mass;
        Vector3 rotImpulse = rb.GetAccumulatedTorque() * Time.fixedDeltaTime;
        Vector3 addedRot = new Vector3(rotImpulse.x / inertiaTensorWS.x, rotImpulse.y / inertiaTensorWS.y, rotImpulse.z / inertiaTensorWS.z);
        Vector3 pointVelRot = Vector3.Cross(rb.angularVelocity+addedRot,point - rb.worldCenterOfMass);
        return (pointVelDir + pointVelRot + Physics.gravity*Time.fixedDeltaTime); //gravity is not counted in accumulatedForce, so it is dont here
    }

    Vector3 GetLiftedPoint(Vector3 point, float lift) //lift point up towards the center of mass
    {
        Vector3 pointInLocalSpace = transform.InverseTransformPoint(point);
        Vector3 liftedPointInLocalSpace = new Vector3(pointInLocalSpace.x, Mathf.Lerp(pointInLocalSpace.y,rb.centerOfMass.y, lift), pointInLocalSpace.z);
        return transform.TransformPoint(liftedPointInLocalSpace); //=liftedPointInWorldSpace
    }
}
