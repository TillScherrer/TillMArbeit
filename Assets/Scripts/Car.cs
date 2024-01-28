using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.ConstrainedExecution;
using TMPro;
using UnityEditor.PackageManager;
//using Unity.VisualScripting;
//using UnityEditor;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.UIElements;
//using UnityEngine.UIElements;
//using static UnityEditor.Experimental.GraphView.GraphView;
//using static UnityEngine.UI.Image;


public class Car : MonoBehaviour
{
    
    public SpeedupCurve SpeedupCurve { get => speedupCurve; set => speedupCurve = value; }
    public SolidGrounGrip FrontGripOnSolidGround { get => frontGripOnSolidGround; set => frontGripOnSolidGround = value; }
    public LooseGrounGrip FrontGripOnLooseGround { get => frontGripOnLooseGround; set => frontGripOnLooseGround = value; }
    public SolidGrounGrip RearGripOnSolidGround { get => rearGripOnSolidGround; set => rearGripOnSolidGround = value; }
    public LooseGrounGrip RearGripOnLooseGround { get => rearGripOnLooseGround; set => rearGripOnLooseGround = value; }
    //public Vector3[] Wheel3DCenters { get => wheel3DCenters;}

    //acceleration
    [SerializeField] SpeedupCurve speedupCurve;
    [SerializeField] float rearOrFrontPowered = 1;

    //gear Settings
    [SerializeField] int numberOfGears = 1;
    public List<float> gearIsBestAtRelSpeed = new List<float>();
    [SerializeField] float gearImpactOnAcceleration = 0;
    float gearScaleAtMinimum = 1;
    float gearScaleDiff = 0;
    [SerializeField] float gearOutDuration = 0.1f;
    [SerializeField] float shiftDuration = 0.1f;
    [SerializeField] float gearInDuration = 0.1f;
    [SerializeField] float shiftingImpactOnAcceleration = 0;
    [SerializeField] GearShiftMode gearShiftMode = GearShiftMode.Automatic;

    //decceleration Settings
    [SerializeField] float slowByAirResistanceAt30ms = 0.5f;
    [SerializeField] float airResistanceExponent = 2f;
    [SerializeField] float slowByRollFriction = 0.05f;

    //breaking Settings
    [SerializeField] float breakMaxSlowdown = 10f;
    [SerializeField] float breakAppliedToFrontWheels = 1f;
    [SerializeField] float breakAppliedToRearWheels = 1f;
    [SerializeField] float breakLessToOptimiseGrip = 1f;
    [SerializeField] float handBreakMaxSlowdown = 20f;
    [SerializeField] float handBreakAppliedToFrontWheels = 1f;
    [SerializeField] float handBreakAppliedToRearWheels = 1f;
    [SerializeField] float handBreakLessToOptimiseGrip = 0f;


    //front wheel physical position and spring settings
    [SerializeField] bool showFrontWheelSettings;
    [SerializeField] Vector3 frontRightWheelCenter = new Vector3(1, -0.3f, 1);
    [SerializeField] float frontWheelRadius = 0.25f;
    //[SerializeField] float frontWheelRatioOfCarsWeight = 0.01f; //car ca. 1%, Monstertruck ca. 10%
    [SerializeField] float frontWheelRotationHoldsRatioOfSpeed = 0.025f; //car ca. 2.5%, Monstertruck ca. 15%?
    [SerializeField] bool frontUse3DWheelPhysics = false;
    [SerializeField] float frontWheelInwardThickness = 0.1f;
    [SerializeField] float frontWheelOutwardThickness = 0;
    [SerializeField] int frontWheelShapeAccuracy = 4;
    [SerializeField] float frontWheelSuspensionDistanceToLiftCarWeight = 0.1f;
    [SerializeField] float frontWheelDamping = 0.4f;
    [SerializeField] float frontSuspHardCap = 0.2f;

    //rear wheel physical position and spring settings
    [SerializeField] bool showRearWheelSettings;
    [SerializeField] Vector3 rearRightWheelCenter = new Vector3(1, -0.3f, -1);
    [SerializeField] float rearWheelRadius = 0.25f;
    //[SerializeField] float rearWheelRatioOfCarsWeight = 0.01f;
    [SerializeField] float rearWheelRotationHoldsRatioOfSpeed = 0.025f;
    [SerializeField] bool rearUse3DWheelPhysics = false;
    [SerializeField] float rearWheelInwardThickness = 0.1f;
    [SerializeField] float rearWheelOutwardThickness = 0;
    [SerializeField] int rearWheelShapeAccuracy = 4;
    [SerializeField] float rearWheelSuspensionDistanceToLiftCarWeight = 0.1f;
    [SerializeField] float rearWheelDamping = 0.4f;
    [SerializeField] float rearSuspHardCap = 0.2f;

    //spring context
    [SerializeField] bool springsByDefaultGravity = true;
    [SerializeField] float springsByOtherValue = -10f;
    [SerializeField] float lateralAttackHeightLift = 0f;
    [SerializeField] float longitudalAttackHeightLift = 0f;

    //anti-roll-bar
    [SerializeField] float frontAntiRollBar = 0f;
    [SerializeField] float rearAntiRollBar = 0f;
    //[SerializeField] float frontAntiRollBar = 0f;


    [SerializeField] LayerMask solidGround;
    [SerializeField] LayerMask looseGround;

    [SerializeField] float scaleGripWithSpringCompression = 1;
    [SerializeField] float scaleGripWithDampingCompression = 0;
    [SerializeField] float spreadGripFromNormalForceOnAllWheels = 0;

    [SerializeField] bool sameGripSettingsForRearWheel;
    [SerializeField]
    private bool endlessLengthwiseGrip = false;
    [SerializeField]
    private bool endlessSidewaysGrip = false;
    //[SerializeField] private Grip frontWheelGrip;
    //[SerializeField] private Grip backWheelGrip;

    [SerializeField] SolidGrounGrip frontGripOnSolidGround;
    [SerializeField] LooseGrounGrip frontGripOnLooseGround;

    [SerializeField] SolidGrounGrip rearGripOnSolidGround = new SolidGrounGrip();
    [SerializeField] LooseGrounGrip rearGripOnLooseGround = new LooseGrounGrip();

    [SerializeField] float FwSgLengthwiseGrip;
    [SerializeField] float RwSgLengthwiseGrip;
    [SerializeField] float FwSgSidewaysGrip;
    [SerializeField] float RwSgSidewaysGrip;
    [SerializeField] float FwLgLengthwiseGrip;
    [SerializeField] float RwLgLengthwiseGrip;
    [SerializeField] float FwLgSidewaysGrip;
    [SerializeField] float RwLgSidewaysGrip;
    [SerializeField] bool useSameGripMultipliersAsSolidGround;

    //Steering Section
    [SerializeField] float ackermanSteering = 0;
    [SerializeField] float maxSteerChangePerSecond = 3;
    [SerializeField] float frontSteerAtZeroSpeed = 30;
    [SerializeField] float frontSteerAt30MS = 15;
    [SerializeField] float rearSteerAtZeroSpeed = -10;
    [SerializeField] float rearSteerAt30MS = 5;
    

    //Input Section
    [SerializeField] bool breakAtOtherDirectionInput = true;
    [SerializeField] FadingKey ThrottleKey = new FadingKey();
    [SerializeField] FadingKey BackwardThrottleKey = new FadingKey();
    [SerializeField] KeyCode SteerLeftKey = KeyCode.A;
    [SerializeField] KeyCode SteerRightKey = KeyCode.S;
    [SerializeField] FadingKey BreakKey = new FadingKey();
    [SerializeField] FadingKey HandbreakKey = new FadingKey();
    [SerializeField] KeyCode GearShiftKey = KeyCode.LeftShift;
    [SerializeField] KeyCode GearShiftUpKey = KeyCode.LeftShift;
    [SerializeField] KeyCode GearShiftDownKey = KeyCode.CapsLock;
    [SerializeField] KeyCode NitroKey = KeyCode.None;

    //Audio Section
    [SerializeField] AudioSource audioForEngine = null;
    [SerializeField] float deltaPitchAtLowGearSpeed = 0.1f;
    [SerializeField] float deltaPitchAtHighGearSpeed = 0.1f;
    [SerializeField] float amplifyDeltaPitchPerGear = -0.3f;
    [SerializeField] float volumeDropOffAtGearShift = 1.0f;
    [SerializeField] float volumeScaleWithThrottleInput = 1.0f;
    [SerializeField] float volumeDropOffAtLowGearEffectiveness = 1.0f;
    float selectedBasePitchForEngine;
    float selectedBaseVolumeForEngine;


    //GUI Section
    [SerializeField] GameObject sliderToShowShift;
    UnityEngine.UI.Slider guiSliderForShifting;
    [SerializeField] GameObject sliderToShowClutch;
    UnityEngine.UI.Slider guiSliderForClutch;
    [SerializeField] GameObject guiGear;
    TextMeshProUGUI guiGearTextMeshPro;
    [SerializeField] GameObject guiSpeed;
    TextMeshProUGUI guiSpeedTextMeshPro;

    //Visualisation
    public Transform[] visualWheels = new Transform[4];
    [SerializeField] bool[] placeOnPhysicalWheel = new bool[] {false,false,false,false};


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
    float[] steeringAngles = new float[] { 0, 0, 0, 0 };
    Vector3[] visualWheelLocalPositions = new Vector3[4];
    Quaternion[] visualWheelLocalRotations = new Quaternion[4];
    
    //Dependent Parameters
    Rigidbody rb;
    

    //float frontSpringPower = 0;
    //float rearSpringPower = 0;
    float usedGravityForSprings = 0;
    LayerMask combinedGroundLayers;


    //Active Parameters
    float[] springPower = new float[4];
    float[] absoluteSpringCompressions = new float[4];
    float[] previousSpringCompressions = new float[4];
    float[] currentWheelSpinningSpeeds = new float[] {0,0,0,0};
    float currentAverageWheelSpinningSpeed = 0;
    float[] visualWheelCurrentXRot = new float[4];
    float[] visualWheelRPS = new float[4];
    int currentGear = 0;
    int aimedGear = 0;
    int prevGear = 0;
    float timeInCurrentGearState = 0;
    GearShiftState gearShiftState = GearShiftState.Geared;
    float accelerationScalerByClutch = 1;
    Vector3 inertiaTensorWS = Vector3.one;
    float frontSteeringAngle = 0;
    float rearSteeringAngle = 0;



    //input bools
    bool inputThrottle = false;
    bool inputBackwardThrottle = false;
    bool inputBreake = false;
    bool inputGearShiftDown = false;
    bool inputGearShiftUp = false;
    bool inputNitro = false;


    //delete after developement is done
    float testBasicSlowdown = 2;
    float testBreakingSlowdown = 8;


    // Start is called before the first frame update
    void Start()
    {
        UpdateDependendParameters();
        UpdateArrayAccessibleParameters(false);
        ValidateGearSettings();
        if (audioForEngine != null)
        {
            selectedBasePitchForEngine = audioForEngine.pitch;
            selectedBaseVolumeForEngine = audioForEngine.volume;
        }
        if (sliderToShowShift != null) guiSliderForShifting = sliderToShowShift.GetComponent<UnityEngine.UI.Slider>();
        if (sliderToShowClutch != null) guiSliderForClutch = sliderToShowClutch.GetComponent<UnityEngine.UI.Slider>();
        if (guiGear != null) guiGearTextMeshPro = guiGear.GetComponent<TextMeshProUGUI>();
        if (guiSpeed != null) guiSpeedTextMeshPro = guiSpeed.GetComponent<TextMeshProUGUI>();
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

    public void UpdateArrayAccessibleParameters(bool calledFromInspector)
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

        if (calledFromInspector && Application.isPlaying) return; //following should not be changed from the inspector during play-Mode

        for(int i = 0; i < 4; i++)
        {
            visualWheelLocalPositions[i] = transform.InverseTransformPoint(visualWheels[i].position);
            visualWheelLocalRotations[i] = Quaternion.Inverse(transform.rotation)*visualWheels[i].rotation;
            
        }
    }

    void Update() {

        //INPUT
        float currentSpeed = Vector3.Dot(rb.velocity, transform.forward); //TO CHANGE: change this to Wheel Spin Speed later
        currentAverageWheelSpinningSpeed = GetAverageWheelSpinningSpeed();
        if(gearShiftMode == GearShiftMode.Automatic || numberOfGears == 0)
        {
            ThrottleKey.IsPressed = (currentSpeed >= -0.01f) && Input.GetKey(ThrottleKey.KeyboardInput);
            BackwardThrottleKey.IsPressed = (currentSpeed <= 0.11f) && Input.GetKey(BackwardThrottleKey.KeyboardInput); //PUT TO 0.01 VALUE LATER !!!!!
            BreakKey.IsPressed = Input.GetKey(BreakKey.KeyboardInput) || (currentSpeed > 0.01 && Input.GetKey(BackwardThrottleKey.KeyboardInput)) || (currentSpeed < -0.01 && Input.GetKey(ThrottleKey.KeyboardInput));
        }
        else
        {
            ThrottleKey.IsPressed = Input.GetKey(ThrottleKey.KeyboardInput) && ((currentSpeed >= -0.01f && currentGear > -1) || (currentSpeed <= 0.01f && currentGear < 0)); //must have right gear for moving direction-
            BackwardThrottleKey.IsPressed = false;
            BreakKey.IsPressed = Input.GetKey(BreakKey.KeyboardInput) || (currentSpeed >= -0.01f && currentGear < 0) || (currentSpeed <= 0.01f && currentGear > -1); ; // -because the wrong gear also counts as breaking
        }

        
        HandbreakKey.IsPressed = Input.GetKey(HandbreakKey.KeyboardInput);
        ThrottleKey.Update();
        BackwardThrottleKey.Update();
        BreakKey.Update();
        HandbreakKey.Update();
        //gear shift input
        ManageGearShiftInput(currentSpeed);


        //TEST OF VISUAL WHEELS
        for (int i = 0; i < 4; i++)
        {
            visualWheelRPS[i] = currentSpeed / ((i < 2 ? frontWheelRadius : rearWheelRadius) * 2 * Mathf.PI); //WIRD DURCH WHEEL SPEED ERSETZT
            visualWheelCurrentXRot[i] += visualWheelRPS[i] * 360 * Time.deltaTime;
            if (visualWheelCurrentXRot[i] > 360) visualWheelCurrentXRot[i] -= 360;
            if (visualWheelCurrentXRot[i] < 0) visualWheelCurrentXRot[i] += 360;
            visualWheels[i].position =    transform.TransformPoint(visualWheelLocalPositions[i]) + transform.up * (absoluteSpringCompressions[i] - (i<2?frontWheelSuspensionDistanceToLiftCarWeight:rearWheelSuspensionDistanceToLiftCarWeight));
            visualWheels[i].rotation =  transform.rotation * Quaternion.Euler(visualWheelCurrentXRot[i], steeringAngles[i], 0) * visualWheelLocalRotations[i];  //visualWheelLocalRotations[i] = Quaternion.Inverse(transform.rotation) * visualWheels[i].rotation;
            //Debug.Log("wheel[" + i + "] steering Angle is " + steeringAngles[i]);
        }
    }


    private void FixedUpdate()
    {
        //update World Space inertia tensor
        inertiaTensorWS = GetInertiaTensorInWorldSpace();




        //float currentTime= Time.realtimeSinceStartup;
        Vector3[] hitPoints;
        Vector3[] hitNormals;
        Vector3[] hitPointForLateral;
        Vector3[] hitPointForLongitudal;
        
        float[] springCompressions;
        int[] collidedGroundType;
        //Vector3 relUp = transform.rotation * Vector3.up;

        FindGroundInteraction(out hitPoints, out hitNormals, out absoluteSpringCompressions, out collidedGroundType);
        hitPointForLateral = new Vector3[] { GetLiftedPoint(hitPoints[0], lateralAttackHeightLift), GetLiftedPoint(hitPoints[1], lateralAttackHeightLift) , GetLiftedPoint(hitPoints[2], lateralAttackHeightLift) , GetLiftedPoint(hitPoints[3], lateralAttackHeightLift) };
        hitPointForLongitudal = new Vector3[] { GetLiftedPoint(hitPoints[0], longitudalAttackHeightLift), GetLiftedPoint(hitPoints[1], longitudalAttackHeightLift), GetLiftedPoint(hitPoints[2], longitudalAttackHeightLift), GetLiftedPoint(hitPoints[3], longitudalAttackHeightLift) };

        ApplyAntiRollBar(absoluteSpringCompressions, out springCompressions);

        //float[] suspPowers = new float[4];
        float[] springCompressionForce = GetSpringCompressionForce(springCompressions);
        float[] dampingForces = new float[4];
        Vector3[] verticalForces = new Vector3[4];
        //Vector3[] verticalForcesWithoutDamping = new Vector3[4];
        for (int i = 0; i < 4; i++)
        {
           

            //SUSPENSION
            //suspension
            //suspPowers[i] = springCompressions[i] * springPower[i];
            //damping
            float deltaSpringCompression = absoluteSpringCompressions[i] - previousSpringCompressions[i];
            float dampingCoefficient = i < 2 ? frontWheelDamping : rearWheelDamping;
            dampingForces[i] = dampingCoefficient * deltaSpringCompression / Time.fixedDeltaTime * springCompressions[i] * rb.mass;
            //verticalForcesWithoutDamping[i] = hitNormals[i] * suspPowers[i] * rb.mass;

            //verticalForces[i] = hitNormals[i] * (suspPowers[i] + dampingValue) * rb.mass;
            verticalForces[i] = hitNormals[i] * (springCompressionForce[i]+ dampingForces[i]);

            rb.AddForceAtPosition(verticalForces[i], hitPoints[i]);
            //handle hard bump when wheelSprings are compressed over the maximum
            if (absoluteSpringCompressions[i] > looseSpringOffsetLength[i] + upwardSuspensionCap[i])
            {
                float currentSpeedAlongLoadingDirection =  Vector3.Dot(rb.GetPointVelocity(hitPoints[i]), -transform.up); //Projection of other directionalV on my directionalV
                //if it is not already in a state of extending the spring again
                if(currentSpeedAlongLoadingDirection>0) //hardstop is only needed, when it is not already in a state of extending the spring again
                {
                    (float vTotal, float vFromDir, Vector3 w, Vector3 rotToV, Vector3 treatedDirection) effectOfFullImpulseStop;
                    effectOfFullImpulseStop = CalculateEffectOfImpulseOnDirectionalMovementAtPoint(hitPoints[i], transform.up, currentSpeedAlongLoadingDirection * rb.mass);
                    float scalerToReachAimedSpeed = currentSpeedAlongLoadingDirection / effectOfFullImpulseStop.vTotal;
                    Vector3 angularChange = effectOfFullImpulseStop.w * scalerToReachAimedSpeed;
                    Vector3 directionalChange = effectOfFullImpulseStop.vFromDir * effectOfFullImpulseStop.treatedDirection * scalerToReachAimedSpeed;
                    rb.AddRelativeTorque(new Vector3(angularChange.x * inertiaTensorWS.x, angularChange.y * inertiaTensorWS.y, angularChange.z * inertiaTensorWS.z), ForceMode.Impulse);
                    rb.AddForce(directionalChange * rb.mass, ForceMode.Impulse);


                    //GANZ ALT
                    //ImpulseNeededToStopDirectionalMovementAtPoint(hitPoints[i], hitNormals[i], out angularChange, out directionalChange, out _);    //TESTWEISE AUSKOMMENTIERT!!!!!!
                    //rb.angularVelocity += angularChange;
                    //rb.velocity += directionalChange;
                }

            }



            

        }

        //TESTDRIVING
        float currentSpeed = Vector3.Dot(rb.velocity, transform.forward);
        float currentRelSpeed = speedupCurve.GetTimeInCurve(currentSpeed);
        //Gear shifting - input has already been managed in "Update()"
        ComputeGearShiftState();
        accelerationScalerByClutch = Mathf.Lerp(1, GetClutchValue(), shiftingImpactOnAcceleration);
        float accelerationScalerBySuitabilityOfGear = GetAccelerationScaleByGear(currentGear, currentSpeed); //TO CHANGE: depend on wheel spin speed later
        float totalGearAccelerationMultiplier = accelerationScalerByClutch * accelerationScalerBySuitabilityOfGear;
        //set steering Angle
        float frontSteerDifFor30ms = frontSteerAtZeroSpeed-frontSteerAt30MS;
        float currentMaxFrontSteerAngle = frontSteerAt30MS + frontSteerDifFor30ms * (-1 + 2 / Mathf.Pow(2, Mathf.Abs(currentSpeed)/30)); //TO DO: DIESE FORMEL RICHTIG MACHEN, AUCH BEI DER ANZEIGE IM INSPECTOR TOOLTIP!
        //float currentMaxFrontSteerAngle = frontSteerAt30MS + frontSteerDifFor30ms * (2 * Mathf.Pow(2, -30/ Mathf.Abs(currentSpeed))-1);
        //richtige formel: rearSteerAt30MS + rearSteerDifFor30ms * (2 * Mathf.Pow(2, -30speed / abs(current)) - 1))
        UpdateSteerAngle(ref frontSteeringAngle, currentMaxFrontSteerAngle);
        float rearSteerDifFor30ms = rearSteerAtZeroSpeed - rearSteerAt30MS;
        //float currentMaxRearSteerAngle = (rearSteerAt30MS + rearSteerDifFor30ms * (2 * -Mathf.Pow(2, -Mathf.Abs(currentSpeed) / 30) - 1)); //diese formel ist vermultich einfach falsch?
        float currentMaxRearSteerAngle = rearSteerAt30MS + rearSteerDifFor30ms * (-1 + 2 / Mathf.Pow(2, Mathf.Abs(currentSpeed)/ 30));
        UpdateSteerAngle(ref rearSteeringAngle, currentMaxRearSteerAngle);
        //TO DO: NOW USE ACKERMAN TO SET INDIVIDUAL STEERING ANGLE OF WHEELS
        
        steeringAngles[0] = frontSteeringAngle;//20 * Input.GetAxis("Horizontal");
        steeringAngles[1] = frontSteeringAngle;//20 * Input.GetAxis("Horizontal");
        steeringAngles[2] = rearSteeringAngle;
        steeringAngles[3] = rearSteeringAngle;
        //Debug.Log("front Steering Angle = " + frontSteeringAngle);
        //Debug.Log("rear Steering Angle = " + rearSteeringAngle);

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
        bool usingBackwardGear = currentGear < 0;
        float gearDirection = usingBackwardGear ? -1 : 1;
        bool wheelsSpinBackwards = averatgeWheelSpeed < 0;
        float wheelSpinningDirectionalMultiplier = wheelsSpinBackwards ? -1 : 1;
        float moveDirectionMultiplier = currentSpeed < 0 ? -1 : 1;
                                                                                                                                        //VORISCHT: BISHER IST ALLES FOLGENDE NUR FÜR VORWÄRTS DREHENDE RÄTER IMPLEMENTIERT!!!!!!!!!!
        float basicDecceleration = -testBasicSlowdown; //Replace with actual current basic slowdown
        float breakingDecceleration = -testBreakingSlowdown; //Replace with actual current breaking slowdown
        float maxThrottleAcceleration = speedupCurve.GetAccelerationValueForSpeed(Mathf.Abs(averatgeWheelSpeed)) * totalGearAccelerationMultiplier; //Remove Mathf.abs and replace with other curve later!!
        float combinedAcceleration;

        bool canPassZeroByAcceleration; //TO DO: muss noch in logik eingebaut werden, nur beschleunigungne können aktiv richtungswechsel initiieren

        if (HandbreakKey.Value > 0)
        {
            //not implemented yet
            combinedAcceleration = -wheelSpinningDirectionalMultiplier;
        }
        else if(BreakKey.Value > 0)
        {
            float deccelerationWithoutBreake;
            if (maxThrottleAcceleration + basicDecceleration < 0)
            {
                deccelerationWithoutBreake = maxThrottleAcceleration + basicDecceleration;
                if (deccelerationWithoutBreake < -testBreakingSlowdown) deccelerationWithoutBreake = -testBreakingSlowdown;
            }
            else
            {
                deccelerationWithoutBreake = basicDecceleration;
            }
            combinedAcceleration = Mathf.Lerp(deccelerationWithoutBreake,-testBreakingSlowdown, BreakKey.Value) * wheelSpinningDirectionalMultiplier;    
        }
        else if ( (gearShiftMode == GearShiftMode.Automatic || numberOfGears==0) ? (currentGear >=0 && ThrottleKey.Value > 0) : ThrottleKey.Value > 0) //TO DO: BACKWARD CASES
        { 
            
            if (maxThrottleAcceleration > 0) //Gear can create Acceleration for current wheel speed
            {
                combinedAcceleration = Mathf.Lerp(basicDecceleration * moveDirectionMultiplier, maxThrottleAcceleration * gearDirection, ThrottleKey.Value);
            }
            else if (maxThrottleAcceleration> breakingDecceleration) //Gear is so bad for current speed that it slows it down, but less than the breaking decceleration
            {
                float minusFromBasicDecceleration = Mathf.Lerp(basicDecceleration, 0, ThrottleKey.Value);
                combinedAcceleration = maxThrottleAcceleration + minusFromBasicDecceleration;
                if (combinedAcceleration < breakingDecceleration) combinedAcceleration = breakingDecceleration;
                combinedAcceleration *= wheelSpinningDirectionalMultiplier;

            }
            else //gear is so bad that it would slow down more than breaking, but we use breaking as a limit
            {
                combinedAcceleration = breakingDecceleration * wheelSpinningDirectionalMultiplier;
            }
        }
        else if (BackwardThrottleKey.Value > 0) //THIS HAS TO BE REPLACED WITH A CONCEPT FOR DRIVING BACKWARDS //TO DO: BACKWARD CASES
        {
            if (maxThrottleAcceleration > 0) //Gear can create Acceleration for current wheel speed
            {
                combinedAcceleration = Mathf.Lerp(basicDecceleration, maxThrottleAcceleration, BackwardThrottleKey.Value);
            }
            else if (maxThrottleAcceleration > breakingDecceleration) //Gear is so bad for current speed that it slows it down, but less than the breaking decceleration
            {
                float minusFromBasicDecceleration = Mathf.Lerp(basicDecceleration, 0, BackwardThrottleKey.Value);
                combinedAcceleration = maxThrottleAcceleration + minusFromBasicDecceleration;
                if (combinedAcceleration < breakingDecceleration) combinedAcceleration = breakingDecceleration;
            }
            else //gear is so bad that it would slow down more than breaking, but we use breaking as a limit
            {
                combinedAcceleration = breakingDecceleration;
            }
            combinedAcceleration *= -1;
        }
        else //only rolling- and air resistance
        {
            float deccelerationWithoutBreake;
            if (maxThrottleAcceleration + basicDecceleration < 0)
            {
                deccelerationWithoutBreake = maxThrottleAcceleration + basicDecceleration;
                if (deccelerationWithoutBreake < -testBreakingSlowdown) deccelerationWithoutBreake = -testBreakingSlowdown;
            }
            else
            {
                deccelerationWithoutBreake = basicDecceleration;
            }
            combinedAcceleration = Mathf.Lerp(deccelerationWithoutBreake, -testBreakingSlowdown, BreakKey.Value) * wheelSpinningDirectionalMultiplier;
        }
        for (int i = 0; i < 4; i++)
        {
            rb.AddForceAtPosition(fowardOnGround[i] * combinedAcceleration / 4f, hitPointForLongitudal[i]);
        }


        //SIDEWARD FRICTION
        float[] normalForcesUsedForGrip = GetNormalForcesManipulatedForGrip(springCompressions, dampingForces);
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
            if ((i < 2 && endlessLengthwiseGrip) || (i >= 2 && endlessSidewaysGrip))
            {
                maxStopImpulse = slideSpeeds[i]* rb.mass / 4; 

            }
            else
            {
                maxStopImpulse = normalForcesUsedForGrip[i] * 1.5f * Time.fixedDeltaTime;//verticalForce.magnitude*0.1f * Time.fixedDeltaTime; //TEST VALUE!!!

            }
           
            impulseProperties[i] = CalculateEffectOfImpulseOnDirectionalMovementAtPoint(hitPointForLateral[i], -slideDirections[i], maxStopImpulse); 
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
        //rb.AddTorque(new Vector3(angularVelChange.x*inertiaTensorWS.x, angularVelChange.y * inertiaTensorWS.y, angularVelChange.z * inertiaTensorWS.z), ForceMode.Impulse);
        //local
        rb.AddRelativeTorque(new Vector3(angularVelChange.x * inertiaTensorWS.x, angularVelChange.y * inertiaTensorWS.y, angularVelChange.z * inertiaTensorWS.z), ForceMode.Impulse);
        rb.AddForce(directionalVelChange * rb.mass, ForceMode.Impulse);


        previousSpringCompressions = absoluteSpringCompressions;
        //float timeUsed = (Time.realtimeSinceStartup - currentTime) / Time.fixedDeltaTime;
        //Debug.Log("Timeratio used is " + timeUsed);



        //Manage Audio
        ManageAudioAndGUI(currentGear, currentSpeed); //TO CHANGE: USE WHEEL SPIN INSTEAD


        //Debug.Log("estimatedVelNextFrame = " + GetActualPointVelocity(Vector3.one));
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
                    //Debug.DrawRay(boxCenter, new Vector3(wheel3DThicknesses[i], wheelRadii[i], zScale), UnityEngine.Color.red, 0.02f);
                    //Gizmos.DrawCube(boxCenter, startToEnd);
                    //ExtDebug.DrawBoxCastBox(boxCenter, new Vector3(wheel3DThicknesses[i], wheelRadii[i], zScale),  rb.rotation * Quaternion.Euler(xRot, 0, 0), startToEnd, startToEnd.magnitude, Color.red);
                    //ExtDebug.DrawBox(boxCenter + rb.rotation * (Vector3.down * wheelRadii[i] + upwardSuspensionCapVec[i]) , new Vector3(wheel3DThicknesses[i], wheelRadii[i], zScale)*0.5f, rb.rotation * Quaternion.Euler(xRot, 0, -1*side), Color.red);
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

    float[] GetNormalForcesManipulatedForGrip(float[] springCompressions, float[] dampingForces)
    {
        float[] gripImpactScaledSpringCompressions = GetSpringCompressionsWhichGotScaledByTheirImpactOnGrip(springCompressions);
        float[] usedForcesBySpringCompression = GetSpringCompressionForce(gripImpactScaledSpringCompressions);
        float[] combinedUsedForces = new float[4];
        float averageForce = 0;
        for(int i = 0; i < 4; i++)
        {
            combinedUsedForces[i] = usedForcesBySpringCompression[i] + dampingForces[i] * scaleGripWithDampingCompression;
            averageForce += combinedUsedForces[i];
        }
        averageForce *= 0.25f;
        float[] combinedAndBalancedForce = new float[4];
        for(int i = 0;i < 4; i++)
        {
            combinedAndBalancedForce[i] = Mathf.Lerp(combinedUsedForces[i], averageForce, spreadGripFromNormalForceOnAllWheels);
        }
        return combinedAndBalancedForce;

    }
    float[] GetSpringCompressionForce(float[] springCompressions)
    {
        float[] normalForceOfSpringCompression = new float[4];
        for (int i = 0; i < 4; i++)
        {
            normalForceOfSpringCompression[i] = springCompressions[i] * springPower[i] * rb.mass;
        }
        return normalForceOfSpringCompression;
    }
    
    float[] GetSpringCompressionsWhichGotScaledByTheirImpactOnGrip(float[] springCompressions)
    {
        float[] scaledSpringCompressions = new float[4];
        for(int i = 0; i < 4; i++)
        {
            //Debug.Log("springCompressions= "+springCompressions[i]);
            float oneCompressionLength = (i < 2 ? frontWheelSuspensionDistanceToLiftCarWeight : rearWheelSuspensionDistanceToLiftCarWeight);
            if (springCompressions[i] == 0) //Not on Ground
            {
                scaledSpringCompressions[i] = 0;
            }
            else 
            {
                float currentCompressionRatio = springCompressions[i] / oneCompressionLength;
                if(currentCompressionRatio < scaleGripWithSpringCompression / 2)
                {
                    float firstGradient = 1/(scaleGripWithSpringCompression);
                    scaledSpringCompressions[i] = firstGradient * springCompressions[i];
                }
                else
                {
                    float secondGradient = scaleGripWithSpringCompression;
                    float yStartingPoint = (1 - scaleGripWithSpringCompression);
                    scaledSpringCompressions[i] = yStartingPoint * oneCompressionLength + secondGradient * springCompressions[i];
                }
            }
            //Debug.Log("ScaledSpringCompressions= " + scaledSpringCompressions[i]);
        }
        return scaledSpringCompressions;
    }

    private (float vTotal, float vFromDir, Vector3 w, Vector3 rotToV, Vector3 treatedDirection) CalculateEffectOfImpulseOnDirectionalMovementAtPoint(Vector3 hitPoint, Vector3 ImpulseDirToCancleCurrent, float impulsePower) 
    {
        ////in local Space, because Unitys inertiaTensor is in local space
        Vector3 localPoint = transform.InverseTransformPoint(hitPoint);

        Vector3 localDir = transform.InverseTransformDirection(ImpulseDirToCancleCurrent.normalized);

        //local
        Vector3 speedPerAngularVelocity = Vector3.Cross(localPoint - rb.centerOfMass, localDir); //how much one unit roation around each axis would move the point in direction of force
        Vector3 angularDirections = new Vector3(speedPerAngularVelocity.x > 0 ? 1 : speedPerAngularVelocity.x < 0 ? -1 : 0,
                                                speedPerAngularVelocity.y > 0 ? 1 : speedPerAngularVelocity.y < 0 ? -1 : 0,
                                                speedPerAngularVelocity.z > 0 ? 1 : speedPerAngularVelocity.z < 0 ? -1 : 0);

        speedPerAngularVelocity = new Vector3(Mathf.Abs(speedPerAngularVelocity.x), //durch nur abs ersetzt
                                              Mathf.Abs(speedPerAngularVelocity.y),
                                              Mathf.Abs(speedPerAngularVelocity.z));

        if (rb.inertiaTensor.x == 0 || rb.inertiaTensor.y == 0 || rb.inertiaTensor.z == 0) Debug.LogError("It is not allowed to lock the rigidbodys rotation while using this script");

        Vector3 s = speedPerAngularVelocity;//short name
        Vector3 j = inertiaTensorWS; //short name


        //relative impulse putten into the v
        float relImpulseX = s.x;// / j.x;
        float relImpulseY = s.y;// / j.y;
        float relImpulseZ = s.z;// / j.z;
        float scaler = impulsePower;// / (relImpulseX + relImpulseY + relImpulseZ + relImpulseDir);
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
        float vdir = impulseDir / rb.mass;
        float totalV = vx + vy + vz + vdir;

        return (totalV, vdir, new Vector3(wx, wy, wz), new Vector3(s.x * angularDirections.x, s.y * angularDirections.y, s.z * angularDirections.z), ImpulseDirToCancleCurrent.normalized);
        
    }

    //Method based on "Nathan Reeds" answer on https://gamedev.stackexchange.com/questions/70355/inertia-tensor-and-world-coordinate-conversion (05.12.23)
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

    public void ValidateGearSettings()
    {
        if(numberOfGears<0)numberOfGears=0;
        if(numberOfGears>16)numberOfGears=16;
        for(int i=0; i < numberOfGears; i++)
        {
            if (i+1 > gearIsBestAtRelSpeed.Count)
            {
                gearIsBestAtRelSpeed.Add(i==0?0: gearIsBestAtRelSpeed[i - 1]);
            }

            if (i > 0)
            {
                if(gearIsBestAtRelSpeed[i] < gearIsBestAtRelSpeed[i - 1])
                {
                    gearIsBestAtRelSpeed[i] = gearIsBestAtRelSpeed[i - 1];
                }
            }
        }
        gearScaleAtMinimum = MinAccelerationScaleLookupTable.GetMinAccelerationScale(gearImpactOnAcceleration);
        gearScaleDiff = 1+gearImpactOnAcceleration - gearScaleAtMinimum;
    }
    public float GetAccelerationScaleByGear(int currentGear, float currentSpeed)//toTo: Rückwärtsgang
    {
        if (numberOfGears == 0) return 1;
        if (gearImpactOnAcceleration == 0) return 1;
        return 1 + gearImpactOnAcceleration - gearScaleDiff * GetGearUneffectivenessRating(currentGear, currentSpeed);
    }

    float GetGearUneffectivenessRating(int currentGear, float currentSpeed)//Done: Rückwärtsgang
    {
        float twistDirMultiplier = (currentGear < 0 && currentSpeed > 0) || (currentGear > -1 && currentSpeed < 0) ? -1 : 1;
        if (numberOfGears == 0) return 1;
        if (currentGear < 0) currentGear = 0;
        float currenSpeedRatio = speedupCurve.GetTimeInCurve(currentSpeed);
        float offsetFromOptimum = Mathf.Abs(currenSpeedRatio * twistDirMultiplier - gearIsBestAtRelSpeed[currentGear]); //offset is higher when the gear points into another direction than driving
        float ratioOfRecomendedShiftMoment = currenSpeedRatio > gearIsBestAtRelSpeed[currentGear] ? GetShiftUpMoment(currentGear) : GetShiftDownMoment(currentGear);
        float distToReccomendedShiftMoment = Mathf.Abs(ratioOfRecomendedShiftMoment - gearIsBestAtRelSpeed[currentGear]);
        return offsetFromOptimum / distToReccomendedShiftMoment;
    }





    float GetShiftUpMoment(int currentGear)//toTo: Rückwärtsgang //vermutlich nicht nötig
    {
        if (currentGear < 0) currentGear = 0;
        return currentGear == (numberOfGears - 1) ? 1 : (gearIsBestAtRelSpeed[currentGear] + gearIsBestAtRelSpeed[currentGear + 1]) / 2;
    }
    float GetShiftDownMoment(int currentGear)//toTo: Rückwärtsgang //vermutlich nicht nötig
    {
        if (currentGear < 0) currentGear = 0;
        return currentGear == 0 ? 0 : (gearIsBestAtRelSpeed[currentGear] + gearIsBestAtRelSpeed[currentGear - 1]) / 2;
    }

    int GetBestGearForCurrentRelSpeed(float relSpeed)
    {
        if (relSpeed < 0) return -1;

        float lowestDif = 100;
        int gearOfLowestDif = 0;
        for(int g=0; g<numberOfGears; g++)
        {
            float difToGearsOptimum = Mathf.Abs(relSpeed - gearIsBestAtRelSpeed[g]);
            if (difToGearsOptimum < lowestDif)
            {
                lowestDif = difToGearsOptimum;
                gearOfLowestDif = g;
            }
        }
        return gearOfLowestDif;
    }

    void UpdateSteerAngle(ref float currentAngle, float maxAngle)
    {
        float maxAngleDirection = maxAngle < 0 ? -1 : 1;

        float maxDegreeChange = Mathf.Abs(maxAngle * (maxSteerChangePerSecond) * Time.fixedDeltaTime);

        int steerInput = 0;
        if (Input.GetKey(SteerRightKey)) steerInput++;
        if (Input.GetKey(SteerLeftKey)) steerInput--;

        if (steerInput == 0)
        {
            if (Mathf.Abs(currentAngle) < maxDegreeChange) currentAngle = 0;
            else if (currentAngle > 0) currentAngle -= maxDegreeChange;
            else currentAngle += maxDegreeChange;
        }
        else if (steerInput == 1) currentAngle += maxDegreeChange * maxAngleDirection;
        else if (steerInput == -1) currentAngle -= maxDegreeChange * maxAngleDirection;


        if (currentAngle > Mathf.Abs(maxAngle)) currentAngle = Mathf.Abs(maxAngle);
        if (currentAngle < -Mathf.Abs(maxAngle)) currentAngle = -Mathf.Abs(maxAngle);
    }

    void ManageGearShiftInput(float currentSpeed)
    {
        //bool drivingBackwards = currentSpeed < 0;
        float currentRelSpeed = speedupCurve.GetTimeInCurve(currentSpeed);

        if (gearShiftMode == GearShiftMode.Automatic || numberOfGears == 0)
        {
            int bestGear = GetBestGearForCurrentRelSpeed(currentRelSpeed); //TO DO: weitere Regeln für automatik Schaltung nötig
            if (bestGear == 0) // in this case input and speed decides if the backward-gear or gear0 should be used
            {
                if (currentGear > 0)
                {
                    aimedGear = 0;
                }
                else if (currentSpeed > -0.01f && ThrottleKey.IsPressed)
                {
                    aimedGear = 0;
                    //if (ThrottleKey.IsPressed)
                    //{
                    //    aimedGear = 0;
                    //}
                    //else if (BackwardThrottleKey.IsPressed)
                    //{
                    //    aimedGear = -1;
                    //}

                }
                else if(currentSpeed < 0.01f && BackwardThrottleKey.IsPressed)
                {
                    aimedGear = -1;
                } 
            }
            else //all other gears are far from zero, so it is earsier to decide
            {
                if (currentSpeed > 0)
                {
                    aimedGear = bestGear;
                }
                else
                {
                    aimedGear = -1;
                }
            }

        }
        else if (gearShiftMode == GearShiftMode.ManualClickUpDown)
        {
            if (Input.GetKeyDown(KeyCode.LeftShift) && aimedGear < numberOfGears-1) aimedGear++; //shift gear up
            if (Input.GetKeyDown(KeyCode.CapsLock) && aimedGear > -1) aimedGear--; //shift gear down
        }
        else if (gearShiftMode == GearShiftMode.ManualOneClick)
        {
            if (Input.GetKeyDown(KeyCode.LeftShift))
            {
                int bestGear = GetBestGearForCurrentRelSpeed(currentRelSpeed);
                if (bestGear == aimedGear) //force a shift even if it is not necesarry
                {
                    bool planGearShiftUp = gearIsBestAtRelSpeed[aimedGear] < currentRelSpeed;
                    if (aimedGear < 0) aimedGear++;
                    else
                    {
                        if (planGearShiftUp && aimedGear < numberOfGears) aimedGear++; //shift gear up
                        if (!planGearShiftUp && aimedGear > -1) aimedGear--; //shift gear down
                    }
                }
                else //shift to best gear
                {
                    aimedGear = bestGear;
                }

                

            }
        }
    }

    void ComputeGearShiftState()
    {
        float remainingTime = Time.fixedDeltaTime; //the time which is simulated this physics update
        
        while (remainingTime > 0)
        {
            switch (gearShiftState)
            {
                case GearShiftState.Geared:
                    if (currentGear == aimedGear) //just keep in current gear
                    {
                        timeInCurrentGearState += remainingTime;
                        remainingTime = 0;

                    }
                    else //initialise gear shifting Process
                    {
                        gearShiftState = GearShiftState.GearingOut;
                        timeInCurrentGearState = 0;
                    }
                    break;

                case GearShiftState.GearingOut:
                    //Debug.Log("gearing Out (current=" + currentGear + ", aimed=" + aimedGear + ")");
                    if (currentGear == aimedGear) //= decision to stop the gear shifting process -> jump to equivalent time to gear in again
                    {
                        float relativeTimeInGearOutProcess = timeInCurrentGearState / gearOutDuration;
                        float relativeTimeInGearInProcess = 1 - relativeTimeInGearOutProcess;
                        timeInCurrentGearState = relativeTimeInGearInProcess * gearInDuration;
                        gearShiftState = GearShiftState.GearingIn;
                    }
                    else
                    {
                        if (timeInCurrentGearState + remainingTime >= gearOutDuration) //reaches next GearShiftState within this frame
                        {
                            remainingTime -= gearOutDuration - timeInCurrentGearState;
                            timeInCurrentGearState = 0;
                            gearShiftState = GearShiftState.Ungeared;
                        }
                        else //keep Gearing Out
                        {
                            timeInCurrentGearState += remainingTime;
                            remainingTime = 0;
                        }
                    }
                    break;


                case GearShiftState.Ungeared:
                    //Debug.Log("ungeared (current=" + currentGear + ", aimed=" + aimedGear + ")");
                    if (currentGear != aimedGear) //this State only needs to reset its stage timer when the user changes the aimed gear by then
                    {
                        prevGear = currentGear;
                        currentGear = aimedGear;
                        timeInCurrentGearState = 0;
                    }
                    if (timeInCurrentGearState + remainingTime >= shiftDuration) //reaches next GearShiftState within this frame
                    {
                        remainingTime -= shiftDuration - timeInCurrentGearState;
                        timeInCurrentGearState = 0;
                        gearShiftState = GearShiftState.GearingIn;
                    }
                    else //keep Shifting in the ungeared state
                    {
                        timeInCurrentGearState += remainingTime;
                        remainingTime = 0;
                    }
                    break;

                case GearShiftState.GearingIn:
                    //Debug.Log("gearing in (current=" + currentGear + ", aimed=" + aimedGear + ")");
                    if (currentGear != aimedGear) //= decision to change the gear again, before it could fully gear in from previous shift -> jump to equivalent time of gearing out again
                    {
                        float relativeTimeInGearInProcess = timeInCurrentGearState / gearInDuration;
                        float relativeTimeInGearOutProcess = 1 - relativeTimeInGearInProcess;
                        timeInCurrentGearState = relativeTimeInGearOutProcess * gearInDuration;
                        gearShiftState = GearShiftState.GearingOut;
                    }
                    else
                    {
                        if (timeInCurrentGearState + remainingTime >= gearInDuration) //reaches next GearShiftState within this frame
                        {
                            remainingTime -= gearInDuration - timeInCurrentGearState;
                            timeInCurrentGearState = remainingTime;
                            remainingTime = 0;
                            gearShiftState = GearShiftState.Geared;
                        }
                        else //keep Gearing Out
                        {
                            timeInCurrentGearState += remainingTime;
                            remainingTime = 0;
                        }
                    }
                    break;

                default:
                    Debug.LogError("Error: invalid GearShiftState");
                    break;
            }
        }
    }

    float GetClutchValue()
    {
        float clutchState = 1;
        switch (gearShiftState)
        {
            case GearShiftState.Geared:
                clutchState = 1;
                break;
            case GearShiftState.GearingOut:
                clutchState = 1- timeInCurrentGearState / gearOutDuration;
                break;
            case GearShiftState.Ungeared:
                clutchState = 0;
                break;
            case GearShiftState.GearingIn:
                clutchState = timeInCurrentGearState / gearInDuration;
                break;
            default:
                Debug.LogError("Error: invalid GearShiftState");
                break;
        }
        return clutchState;
    }



    void ManageAudioAndGUI(int currentGear, float currentSpeed)//toTo: Rückwärtsgang   
    {
        if (audioForEngine == null) return;
        bool isInNegativeGear = currentGear < 0;
        if(currentGear <0) currentGear = 0;

        float pitchRawChange = 0;
        float totalPitch = selectedBasePitchForEngine;

        if (numberOfGears != 0) {
            float currenSpeedRatio = speedupCurve.GetTimeInCurve(currentSpeed);
            float currentDeltaPitchAmplification = 1 + currentGear * amplifyDeltaPitchPerGear;
            if (currenSpeedRatio > gearIsBestAtRelSpeed[currentGear])
            {
                float nextMinScaleReachedAt = GetShiftUpMoment(currentGear);
                float distMaxToMin = nextMinScaleReachedAt - gearIsBestAtRelSpeed[currentGear];                
                float currentRatioDistanceToMax = currenSpeedRatio - gearIsBestAtRelSpeed[currentGear];
                pitchRawChange = currentRatioDistanceToMax / distMaxToMin;
                totalPitch *= (1 + deltaPitchAtHighGearSpeed * currentDeltaPitchAmplification * pitchRawChange);
            }
            else
            {
                float previousMinScaleReachedAt = GetShiftDownMoment(currentGear);
                float distMinToMax = gearIsBestAtRelSpeed[currentGear] - previousMinScaleReachedAt;
                float currentRatioDistanceToMax = gearIsBestAtRelSpeed[currentGear] - currenSpeedRatio;
                pitchRawChange = -currentRatioDistanceToMax / distMinToMax;
                totalPitch /= (1 + (deltaPitchAtLowGearSpeed * currentDeltaPitchAmplification * -pitchRawChange));
            }
        }


        float totalVolume = selectedBaseVolumeForEngine;
        float volumeScaledByThrottle = Mathf.Lerp(1, Mathf.Max(ThrottleKey.Value, BackwardThrottleKey.Value), volumeScaleWithThrottleInput);
        float volumeScaledByGearState = Mathf.Lerp(1, GetClutchValue(), volumeDropOffAtGearShift);
        float volumeScaledByGearUneffectiveness = 1 / (1 + GetGearUneffectivenessRating(isInNegativeGear? -1 : currentGear, currentSpeed)*volumeDropOffAtLowGearEffectiveness);
        totalVolume *= (volumeScaledByThrottle * volumeScaledByGearState * volumeScaledByGearUneffectiveness);
        audioForEngine.pitch = totalPitch;
        audioForEngine.volume = totalVolume;

        if (guiSliderForShifting != null)
        {
            float shownValue = 0.5f + 0.25f * pitchRawChange;

            guiSliderForShifting.value = shownValue;
            ColorBlock cb = guiSliderForShifting.colors;
            cb.normalColor = shownValue < 0.2f ? Color.red : shownValue < 0.25 ? Color.yellow : shownValue < 0.75f ? Color.green : shownValue < 0.8f ? Color.yellow : Color.red;
            guiSliderForShifting.colors = cb;
        }
        if(guiSliderForClutch != null)
        {
            guiSliderForClutch.value = GetClutchValue();
        }
        if(guiGear != null)
        {
            guiGearTextMeshPro.text = isInNegativeGear ? "R" : (currentGear + 1).ToString();
        }
        if (guiSpeed != null)
        {
            guiSpeedTextMeshPro.text = (currentSpeed*3.6f).ToString("F0")+" km/h";
        }

    }


    float GetAverageWheelSpinningSpeed()
    {
        float average = 0;
        for(int i = 0; i < 4; i++)
        {
            average += currentWheelSpinningSpeeds[i] * (i < 2 ? rearOrFrontPowered: 1-rearOrFrontPowered) / 2;
        }
        return average;
    }

    //Just visual inspector feedback
    void OnDrawGizmos()
    {
        DrawPhysical3DWheelShape();
    }
    void DrawPhysical3DWheelShape()
    {
        for (int i = (showFrontWheelSettings && frontUse3DWheelPhysics ? 0 : 2); i < (showRearWheelSettings && rearUse3DWheelPhysics ? 4 : 2); i++)
        {
            Vector3 startPoint = transform.TransformPoint(wheel3DCenters[i]);
            int usedShapeAccuracy = (i < 2 ? frontWheelShapeAccuracy : rearWheelShapeAccuracy);
            int side = i % 2 == 0 ? -1 : 1; //serves as positive multiplier for right side, negative for left side
            float degreeCoveredPerBoxCast = 180 / usedShapeAccuracy;
            float usedWheelRadius = i < 2 ? frontWheelRadius : rearWheelRadius;
            //float wheel3DThickness = i < 2 ? frontWheelInwardThickness.floatValue + frontWheelOutwardThickness.floatValue : rearWheelInwardThickness.floatValue + rearWheelOutwardThickness.floatValue;
            float zScale = 2 * usedWheelRadius * Mathf.Tan(degreeCoveredPerBoxCast / 2 * Mathf.Deg2Rad);
            //Quaternion carRotation = car.transform.rotation;
            for (int j = 0; j < usedShapeAccuracy; j++)
            {
                float xRot = -90 + (j + 0.5f) * degreeCoveredPerBoxCast;
                Vector3 boxCenter = startPoint + transform.rotation * (Quaternion.Euler(xRot, 0, 0) * Vector3.down * usedWheelRadius * 0.5f);
                ExtDebug.DrawBox(boxCenter, new Vector3(wheel3DThicknesses[i], usedWheelRadius, zScale) * 0.5f, transform.rotation * Quaternion.Euler(xRot, 0, -1 * side), Color.red);
            }
        }
    }


}



public enum GearShiftMode
{
    Automatic,
    ManualOneClick,
    ManualClickUpDown,
}

public enum GearShiftState
{
    Geared,
    GearingOut,
    Ungeared,
    GearingIn
}






public static class MinAccelerationScaleLookupTable
{
    public static float[] lookupTable = new float[] { //contains 101 values representing 0%-100%
        1.0000000f, 0.9900646f, 0.9802637f, 0.9705906f, 0.961041f, 0.9516158f, 0.9423141f, 0.9331286f, 0.9240597f, 0.9151053f, 0.906265f, 0.8975334f, 0.8889122f, 0.880394f, 0.8719826f, 0.8636732f, 0.8554645f, 0.8473539f, 0.8393438f, 0.8314257f, //0-19
        0.8236029f, 0.8158737f, 0.8082352f, 0.8006854f, 0.7932246f, 0.7858496f, 0.7785599f, 0.7713528f, 0.7642288f, 0.7571859f, 0.7502251f, 0.7433472f, 0.7365316f, 0.7298009f, 0.7231455f, 0.7165642f, 0.7100525f, 0.7036151f, 0.6972478f, 0.6909504f,//20-39
        0.6847193f, 0.6785581f, 0.6724701f, 0.6664329f, 0.6604667f, 0.6545649f, 0.6487243f, 0.6429468f, 0.6372294f, 0.6315734f, 0.6259766f, 0.6204367f, 0.6149564f, 0.6095314f, 0.6041619f, 0.5988486f, 0.5935898f, 0.588385f, 0.5832326f, 0.5781331f, //40-59
        0.5730858f, 0.5680876f, 0.5631714f, 0.5582448f, 0.5533972f, 0.548597f, 0.5438461f, 0.5391414f, 0.5344839f, 0.5298703f, 0.5253034f, 0.5207834f, 0.5163078f, 0.5118713f, 0.5074816f, 0.5031323f, 0.4988257f, 0.4945617f, 0.4903395f, 0.486156f, //60-79
        0.4820137f, 0.4779101f, 0.473846f, 0.469821f, 0.4658359f, 0.4618855f, 0.4579735f, 0.4540997f, 0.4502611f, 0.4464593f, 0.4426928f, 0.4389615f,0.4352644f, 0.4316101f, 0.4279785f, 0.4243813f, 0.4208221f, 0.4172921f, 0.4137953f, 0.4103317f, //80-99
        0.4069004f }; //100

    public static float GetMinAccelerationScale(float gearImpact)
    {
        float lookupTableGranularity = lookupTable.Length - 1;
        if (gearImpact == 0) return 1;
        else if(gearImpact == 1) return lookupTable[lookupTable.Length - 1];
        else
        {
            float relativeValue = gearImpact*lookupTableGranularity;
            float byClosestLowerValue = lookupTable[Mathf.FloorToInt(relativeValue)];
            float byClosestUpperValue = lookupTable[Mathf.CeilToInt(relativeValue)];
            float scale = relativeValue % 1;
            return Mathf.Lerp(byClosestLowerValue, byClosestUpperValue, scale);
        }
    }
}


