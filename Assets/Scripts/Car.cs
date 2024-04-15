//using System;
//using System.Collections;
using System.Collections.Generic;
//using System.Runtime.ConstrainedExecution;
using TMPro;
//using Unity.VisualScripting;
//using UnityEditor.PackageManager;
//using Unity.VisualScripting;
//using UnityEditor;
using UnityEngine;
using UnityEngine.UI;
//using UnityEngine.UIElements;
//using UnityEngine.UIElements;
//using static UnityEditor.Experimental.GraphView.GraphView;
//using static UnityEngine.UI.Image;


public class Car : MonoBehaviour
{

    public SpeedupCurve SpeedupCurve { get => speedupCurve; set => speedupCurve = value; }
    public SolidGrounGrip FrontGripOnSolidGround { get => gripOnSolidGround; set => gripOnSolidGround = value; }
    public LooseGrounGrip FrontGripOnLooseGround { get => gripOnLooseGround; set => gripOnLooseGround = value; }
    //public SolidGrounGrip RearGripOnSolidGround { get => rearGripOnSolidGround; set => rearGripOnSolidGround = value; }
    //public LooseGrounGrip RearGripOnLooseGround { get => rearGripOnLooseGround; set => rearGripOnLooseGround = value; }
    //public Vector3[] Wheel3DCenters { get => wheel3DCenters;}

    //acceleration
    [SerializeField] SpeedupCurve speedupCurve;
    [SerializeField] float rearOrFrontPowered = 1;
    [SerializeField] float accelerationRestrainToOptimiseGrip = 0.5f; //Todo: inspector

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
    [SerializeField] float breakRestrainToOptimiseGrip = 1f;
    [SerializeField] float breakLimitedByGrip = 1f;
    [SerializeField] float handBreakMaxSlowdown = 20f;
    [SerializeField] float handBreakAppliedToFrontWheels = 1f;
    [SerializeField] float handBreakRestrainToOptimiseGrip = 0f;
    [SerializeField] float handBreakLimitedByGrip = 1f;


    //front wheel physical position and spring settings
    [SerializeField] bool showWheelSettings;
    [SerializeField] Vector3 frontRightWheelCenter = new Vector3(1, -0.3f, 1);
    [SerializeField] float frontWheelRadius = 0.25f;
    //[SerializeField] float frontWheelRatioOfCarsWeight = 0.01f; //car ca. 1%, Monstertruck ca. 10%
    [SerializeField] float frontWheelRotationHoldsRatioOfSpeed = 0.025f; //car ca. 2.5%, Monstertruck ca. 15%?
    [SerializeField] bool frontUse3DWheelPhysics = false;
    [SerializeField] float frontWheelInwardThickness = 0.1f;
    [SerializeField] float frontWheelOutwardThickness = 0;
    [SerializeField] int frontWheelShapeAccuracy = 4;
    [SerializeField] float frontWheelSuspensionDistanceToLiftCarWeight = 0.1f;
    [SerializeField] float frontSuspHardCap = 0.2f;

    //rear wheel physical position and spring settings
    [SerializeField] Vector3 rearRightWheelCenter = new Vector3(1, -0.3f, -1);
    [SerializeField] float rearWheelRadius = 0.25f;
    [SerializeField] float rearWheelRotationHoldsRatioOfSpeed = 0.025f;
    [SerializeField] bool rearUse3DWheelPhysics = false;
    [SerializeField] float rearWheelInwardThickness = 0.1f;
    [SerializeField] float rearWheelOutwardThickness = 0;
    [SerializeField] int rearWheelShapeAccuracy = 4;
    [SerializeField] float rearWheelSuspensionDistanceToLiftCarWeight = 0.1f;
    [SerializeField] float rearSuspHardCap = 0.2f;

    //spring context
    [SerializeField] bool springsByDefaultGravity = true;
    [SerializeField] float springsByOtherValue = -10f;
    [SerializeField] float lateralAttackHeightLift = 0f;
    [SerializeField] float longitudalAttackHeightLift = 0f;

    //balance
    [SerializeField] float damping = 0.4f;
    [SerializeField] float antiJumpSuspension = 0f;
    [SerializeField] float frontAntiRollBar = 0f;
    [SerializeField] float rearAntiRollBar = 0f;
    [SerializeField] TurnUpwardsField TurnUpwardsWithZeroWheels = new TurnUpwardsField();
    [SerializeField] TurnUpwardsField TurnUpwardsWithOneOrTwoWheels = new TurnUpwardsField();
    [SerializeField] TurnUpwardsField TurnUpwardsWithThreeOrFourWheels = new TurnUpwardsField();
    [SerializeField] float minFlightDurationForCorrection = 1.5f;
    [SerializeField] float maxAngularVelCorrectionInAir = 0f;
    [SerializeField] bool fixFlightRotationAtLeavingGround = false;


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

    [SerializeField] float lengthwiseGripAffectedBySidewaysSlip = 1;
    [SerializeField] float sidewaysGripAffectedByLengthwiseSlip = 1;

    [SerializeField] SolidGrounGrip gripOnSolidGround;
    [SerializeField] LooseGrounGrip gripOnLooseGround;

    //[SerializeField] SolidGrounGrip rearGripOnSolidGround = new SolidGrounGrip();
    //[SerializeField] LooseGrounGrip rearGripOnLooseGround = new LooseGrounGrip();

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
    //Steering Balance
    [SerializeField] float frontSteerTowardsSlide = 0;
    [SerializeField] float maxFrontAngleTowardsSlide = 0;
    [SerializeField] float rearSteerTowardsSlide = 0;
    [SerializeField] float maxRearAngleTowardsSlide = 0;
    //Arcady Steer
    [SerializeField] float disableNaturalSteering = 0;
    [SerializeField] float maxAnglesPSChangeAtArcadySteering = 50f;
    //Air Steer
    [SerializeField] float maxAnglesPSChangeAtAirSteering = 0f;


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
    [SerializeField] bool[] placeOnPhysicalWheel = new bool[] { false, false, false, false };


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
    float[] rawSteeringAngles = new float[] { 0, 0, 0, 0 };
    float[] prevOffAngles = new float[] { 0, 0, 0, 0 };
    float[] steeringAngles = new float[] { 0, 0, 0, 0 };
    Vector3[] visualWheelLocalPositions = new Vector3[4];
    Quaternion[] visualWheelLocalRotations = new Quaternion[4];
    //float[] solideGroundLenghthwiseGripMultipliers = new float[4];
    //float[] solideGroundSidewaysGripMultipliers = new float[4];
    //float[] looseGroundLenghthwiseGripMultipliers = new float[4];
    //float[] looseGroundSidewaysGripMultipliers = new float[4];

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
    float[] lengthwiseForceAtWheels = new float[] { 0, 0, 0, 0 };
    float[] sidewayGripAtWheels = new float[] { 0, 0, 0, 0 };
    float currentAverageWheelSpinningSpeed = 0;
    float[] visualWheelCurrentXRot = new float[4];
    float[] visualWheelRPS = new float[4];
    float[] longitudalRatioAndDirectionalScalingGripAtPreviouseFrame = new float[] {0.1f,0.1f,0.1f,0.1f}; //TO DO: SAVE FROM PREVIOUSE FRAME
    int currentGear = 0;
    int aimedGear = 0;
    int prevGear = 0;
    float timeInCurrentGearState = 0;
    GearShiftState gearShiftState = GearShiftState.Geared;
    float accelerationScalerByClutch = 1;
    Vector3 inertiaTensorWS = Vector3.one;
    float frontSteeringAngle = 0;
    float rearSteeringAngle = 0;

    int numberOfGroundedWheels = 0;
    float timeInAir = 0;
    bool triggeredGroundLeftAlready = false;
    bool justLeftGround = false;
    float lastContactWithGround = 0;


    //input bools
    //bool inputThrottle = false;
    //bool inputBackwardThrottle = false;
    //bool inputBreake = false;
    //bool inputGearShiftDown = false;
    //bool inputGearShiftUp = false;
    //bool inputNitro = false;


    //delete after developement is done
    //float testBasicSlowdown = 2;
    //float testBreakingSlowdown = 8;


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


        //make grip multiplier accessable by array
        



        if (calledFromInspector && Application.isPlaying) return; //after this comes, what should not be changed from the inspector during play-Mode

        for(int i = 0; i < 4; i++)
        {
            visualWheelLocalPositions[i] = transform.InverseTransformPoint(visualWheels[i].position);
            visualWheelLocalRotations[i] = Quaternion.Inverse(transform.rotation)*visualWheels[i].rotation;
            
        }





    }

    void Update() {

        //INPUT
        //float currentSpeed = Vector3.Dot(rb.velocity, transform.forward); //TO CHANGE: change this to Wheel Spin Speed later
        currentAverageWheelSpinningSpeed = GetAverageWheelSpinningSpeed();
        if(gearShiftMode == GearShiftMode.Automatic || numberOfGears == 0)
        {
            ThrottleKey.IsPressed = (currentAverageWheelSpinningSpeed >= -0.01f) && Input.GetKey(ThrottleKey.KeyboardInput);
            BackwardThrottleKey.IsPressed = (currentAverageWheelSpinningSpeed <= 0.01f) && Input.GetKey(BackwardThrottleKey.KeyboardInput); //PUT TO 0.01 VALUE LATER !!!!!
            
            BreakKey.IsPressed = Input.GetKey(BreakKey.KeyboardInput) || (currentAverageWheelSpinningSpeed > 0.01 && Input.GetKey(BackwardThrottleKey.KeyboardInput)) || (currentAverageWheelSpinningSpeed < -0.01 && Input.GetKey(ThrottleKey.KeyboardInput));
            if (BackwardThrottleKey.IsPressed && BackwardThrottleKey.KeyboardInput == BreakKey.KeyboardInput) BreakKey.IsPressed = false;
        }
        else
        {
            ThrottleKey.IsPressed = Input.GetKey(ThrottleKey.KeyboardInput) && ((currentAverageWheelSpinningSpeed >= -0.01f && currentGear > -1) || (currentAverageWheelSpinningSpeed <= 0.01f && currentGear < 0)); //must have right gear for moving direction-
            BackwardThrottleKey.IsPressed = false;
            BreakKey.IsPressed = Input.GetKey(BreakKey.KeyboardInput) || (currentAverageWheelSpinningSpeed >= 0.01f && currentGear < 0) || (currentAverageWheelSpinningSpeed <= -0.01f && currentGear > -1); ; // -because the wrong gear also counts as breaking
        }

        //Debug.Log("Throttle=" + ThrottleKey.IsPressed + ", BackwardThrottle=" + BackwardThrottleKey.IsPressed + ", Break=" + BreakKey.IsPressed);
        
        HandbreakKey.IsPressed = Input.GetKey(HandbreakKey.KeyboardInput);
        ThrottleKey.Update();
        BackwardThrottleKey.Update();
        BreakKey.Update();
        HandbreakKey.Update();
        //gear shift input
        ManageGearShiftInput(currentAverageWheelSpinningSpeed);


        //TEST OF VISUAL WHEELS
        for (int i = 0; i < 4; i++)
        {
            visualWheelRPS[i] = currentWheelSpinningSpeeds[i] / ((i < 2 ? frontWheelRadius : rearWheelRadius) * 2 * Mathf.PI); 
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
        ComputeGroundAndAirStatus();
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
            float dampingCoefficient = i < 2 ? damping : damping;
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
                }
            }
        }




        //STEERING
        float currentSpeed = Vector3.Dot(rb.velocity, transform.forward);
        float frontSteerDifFor30ms = frontSteerAtZeroSpeed - frontSteerAt30MS; //front
        float currentMaxFrontSteerAngle = frontSteerAt30MS + frontSteerDifFor30ms * (-1 + 2 / Mathf.Pow(2, Mathf.Abs(currentSpeed) / 30));
        UpdateRawSteerAngle(ref frontSteeringAngle, currentMaxFrontSteerAngle);
        float rearSteerDifFor30ms = rearSteerAtZeroSpeed - rearSteerAt30MS;    //rear
        float currentMaxRearSteerAngle = rearSteerAt30MS + rearSteerDifFor30ms * (-1 + 2 / Mathf.Pow(2, Mathf.Abs(currentSpeed) / 30));
        UpdateRawSteerAngle(ref rearSteeringAngle, currentMaxRearSteerAngle);
        UpdateSteeringAngles(hitPoints);
        

        //GEAR SHIFTING  
        ComputeGearShiftState(); //(input has already been managed in "Update()". Therefor the registered input is computed here)

        //TESTDRIVING
        currentAverageWheelSpinningSpeed = GetAverageWheelSpinningSpeed();
        //float currentRelSpeed = speedupCurve.GetTimeInCurve(currentSpeed);        
        //accelerationScalerByClutch = Mathf.Lerp(1, GetClutchValue(), shiftingImpactOnAcceleration);
        //float accelerationScalerBySuitabilityOfGear = GetAccelerationScaleByGear(currentGear, currentSpeed); //TO CHANGE: depend on wheel spin speed later
        //float totalGearAccelerationMultiplier = accelerationScalerByClutch * accelerationScalerBySuitabilityOfGear;
        //set steering Angle (front)
        

        Vector3[] fowardOnGround = new Vector3[4];
        float[] fowardSpeedOverGroundAtWheel = new float[4];
        Vector3[] sideSlideDirections = new Vector3[4];
        float[] sideSlideSpeeds = new float[4];
        for (int i = 0; i < 4; i++)
        {
            //forward
            Vector3 sideDirectionR = rb.rotation * (Quaternion.Euler(0, steeringAngles[i], 0) * Vector3.right);
            if (hitNormals[i] == Vector3.zero)// = if this wheel is not in contact with the ground, take local foward vector of wheel instead
            {
                fowardOnGround[i] = Vector3.Cross(sideDirectionR, transform.up).normalized;
            }
            else
            {
                fowardOnGround[i] = Vector3.Cross(sideDirectionR, hitNormals[i]).normalized;
            }
            fowardSpeedOverGroundAtWheel[i] = Vector3.Dot(GetActualPointVelocity(hitPointForLongitudal[i]), fowardOnGround[i]);
            //Debug.DrawRay(hitPointForLongitudal[i], fowardOnGround[i] * fowardSpeedOverGroundAtWheel[i]);

            //sideward
            Vector3 directionFoward = rb.rotation * (Quaternion.Euler(0, steeringAngles[i], 0) * Vector3.forward);
            Vector3 rOnGround = Vector3.Cross(hitNormals[i], directionFoward);
            sideSlideDirections[i] = (rOnGround * ((Vector3.Angle(GetActualPointVelocity(hitPointForLateral[i]), rOnGround) > 90) ? -1 : 1)).normalized;
            sideSlideSpeeds[i] = Vector3.Dot(GetActualPointVelocity(hitPointForLateral[i]), sideSlideDirections[i]); // CHECK IF THIS IS CORRECT!!!!
            //Debug.DrawRay(hitPointForLateral[i], -sideSlideDirections[i].normalized, Color.black);
            //if (i == 0) Debug.DrawRay(hitPoints[i], -slideDirection.normalized * 0.1f, Color.blue, 0.3f);
            //Debug.DrawRay(hitPoints[i], rb.GetPointVelocity(hitPoints[i]), Color.cyan);
        }



        (float atFullThrottle, float atNoThrottle) motorAcceleration = GetMotorAccelerationAbs();
        //Note: "motorAcceleration.atNoThrottle" is always 0 or negative, whereas ".atFullThrottle" it is usualy positive but could also turn out negative if a very unsuitable gear is used.
        ActionMode actionMode;
        int forceDirection; //default Foward
        float frontWheelsPowerShare;
        float totalAvailablePower;
        float usedSlipPreventer;
        bool wheelsSpinBackwards = currentAverageWheelSpinningSpeed < 0;
        int wheelSpinningDirectionalMultiplier = wheelsSpinBackwards ? -1 : 1;
        //Debug.Log("wheelSpin direction = " + wheelSpinningDirectionalMultiplier + ", avSpeed = " + currentAverageWheelSpinningSpeed);
        float[] normalForcesUsedForGrip = GetNormalForcesManipulatedForGrip(springCompressions, dampingForces);
        float[] rollingFrictioSlowdownForEachWheel = GetRollingFrictionSlowdonwForEachWheelAbs(normalForcesUsedForGrip);
        float totalRollingFrictionSlowdown = rollingFrictioSlowdownForEachWheel[0] + rollingFrictioSlowdownForEachWheel[1] + rollingFrictioSlowdownForEachWheel[2] + rollingFrictioSlowdownForEachWheel[3];

        if (HandbreakKey.Value > 0)
        {
            actionMode = ActionMode.Handbreak;
            forceDirection = -wheelSpinningDirectionalMultiplier; //breaking applies force into the opposite direction of the current wheel Spin.
            
            //the breaking from the handbreak and the motor are combined here - as well as the proportional distribution to front or rear wheels.
            //However the combined slow can not exceed the maximum slow from the handbreak. This way the player can not abuse shifting into a way-too-low gear to get unintended good breaking power.
            float handbreakBreakPower = handBreakMaxSlowdown*HandbreakKey.Value;
            float motorBreakPower = -motorAcceleration.atNoThrottle;
            float ratioOfMotorBreak = motorBreakPower / (motorBreakPower + handbreakBreakPower);
            frontWheelsPowerShare = handBreakAppliedToFrontWheels * (1-ratioOfMotorBreak) + motorBreakPower * ratioOfMotorBreak;
            totalAvailablePower = handbreakBreakPower + motorBreakPower;
            usedSlipPreventer = handBreakRestrainToOptimiseGrip;
            if (totalAvailablePower > handBreakMaxSlowdown) totalAvailablePower = handBreakMaxSlowdown;

        }
        else if (BreakKey.Value > 0)//breaking works like handbreaking just with other values
        {
            actionMode = ActionMode.Break;
            forceDirection = -wheelSpinningDirectionalMultiplier;
            float breakBreakPower = breakMaxSlowdown * BreakKey.Value;
            float motorBreakPower = -motorAcceleration.atNoThrottle;
            float ratioOfMotorBreak = motorBreakPower / (motorBreakPower + breakBreakPower);
            frontWheelsPowerShare = breakAppliedToFrontWheels * (1 - ratioOfMotorBreak) + motorBreakPower * ratioOfMotorBreak;
            totalAvailablePower = breakBreakPower + motorBreakPower;
            usedSlipPreventer = breakRestrainToOptimiseGrip;
            if (totalAvailablePower > breakMaxSlowdown) totalAvailablePower = breakMaxSlowdown;
        }
        else if ((gearShiftMode == GearShiftMode.Automatic || numberOfGears == 0) ? (currentGear >= 0 && ThrottleKey.Value > 0) : ThrottleKey.Value > 0) //TO DO: BACKWARD CASES
        {
            float acceleration = motorAcceleration.atFullThrottle * ThrottleKey.Value + motorAcceleration.atNoThrottle * (1-ThrottleKey.Value);
            frontWheelsPowerShare = rearOrFrontPowered;

            if (acceleration > 0) 
            {
                actionMode = ActionMode.AccelerateByMotor;
                totalAvailablePower = acceleration;
                forceDirection = currentGear<0?-1:1;
                usedSlipPreventer = accelerationRestrainToOptimiseGrip;
            }
            else
            {
                actionMode = ActionMode.DeccelerateByMotor;
                forceDirection = -wheelSpinningDirectionalMultiplier;
                totalAvailablePower = -acceleration;
                usedSlipPreventer = 0;
                if (totalAvailablePower > breakMaxSlowdown) totalAvailablePower = breakMaxSlowdown;
                
            }
            
        }
        else if (BackwardThrottleKey.Value > 0) //THIS HAS TO BE REPLACED WITH A CONCEPT FOR DRIVING BACKWARDS //TO DO: BACKWARD CASES     //-maybe already done
        {
            float acceleration = motorAcceleration.atFullThrottle * BackwardThrottleKey.Value + motorAcceleration.atNoThrottle * (1 - BackwardThrottleKey.Value);
            frontWheelsPowerShare = rearOrFrontPowered;

            if (acceleration > 0)
            {
                actionMode = ActionMode.AccelerateByMotor;
                forceDirection = -1;
                totalAvailablePower = acceleration;
                usedSlipPreventer = accelerationRestrainToOptimiseGrip;
            }
            else
            {
                actionMode = ActionMode.DeccelerateByMotor;
                forceDirection = -wheelSpinningDirectionalMultiplier;
                totalAvailablePower = -acceleration;
                usedSlipPreventer = 0;
                if (totalAvailablePower > breakMaxSlowdown) totalAvailablePower = breakMaxSlowdown;
            }
        }
        else //rolling
        {
            frontWheelsPowerShare = rearOrFrontPowered;
            actionMode = ActionMode.DeccelerateByMotor;
            forceDirection = -wheelSpinningDirectionalMultiplier;
            totalAvailablePower = -motorAcceleration.atNoThrottle;
            usedSlipPreventer = 0;
            if (totalAvailablePower > breakMaxSlowdown) totalAvailablePower = breakMaxSlowdown;
        }


        //Debug.Log("totalAvailablePower " + totalAvailablePower);
        //Debug.Log("forceDirection " + forceDirection);
        //Debug.Log("actionMode " + actionMode);
        //Debug.Log("frontWheelsPowerShare " + frontWheelsPowerShare);
        //Debug.Log(" "+);

        //TO DO: FÜR VORDER UND HINTERRADPAAR ZUSÄTZLICH NO THROTTLE DRAUF PACKEN, FALLS MAXIMUM DADURCH NICHT ÜBERSCHRITTEN WIRD
        //float aimedAccelerationDirection = currentGear < 0 ? -1 : 1;
        float[] powerShareForWheelRotation = new float[4];
        float[] powerShareForMovement = new float[4];
        float[] availablePowerForFrame = new float[4];
        //float[] totalDirectionalPower = new float[4];
        bool[] canLeadToDirectionChange = new bool[] {false,false,false,false};


        float fWPowerReservedForBothWheelSpins = frontWheelRotationHoldsRatioOfSpeed * 2; //first of all a part of the estimated power distribution does not depend on grip because it goes into canging the rotation of the wheels (without moving the car itself)
        if(fWPowerReservedForBothWheelSpins > frontWheelsPowerShare) fWPowerReservedForBothWheelSpins = frontWheelsPowerShare; //not more than the entire front wheel power can be powered into pure wheel rotation
        //float fWRemainingPowerShare = frontWheelsPowerShare - fWPowerReservedForBothWheelSpins;
        //(same for rear wheels)
        float rearWheelsPowerShare = 1 - frontWheelsPowerShare;
        float rWPowerReservedForBothWheelSpins = rearWheelRotationHoldsRatioOfSpeed * 2;
        if (rWPowerReservedForBothWheelSpins > rearWheelsPowerShare) rWPowerReservedForBothWheelSpins = rearWheelsPowerShare;
        //float rWRemainingPowerShare = rearWheelsPowerShare - rWPowerReservedForBothWheelSpins;
        for (int i = 0; i < 4; i += 2)//has two iterations. The first one treats the left and right front wheel and the second one treats both rear wheels
        {
            //now split spin power between left and right wheel by the difference of their longitudal slip ratio to balance out unreasonably diffrent spinnging wheels (=by how their individual rotation matches the individual directional movement over the ground)
            float leftRelSpinShare;
            float rightRelSpinShare;

            float leftWheelLongitudalSlipRatio = GetDirectionalLongitudalSlipRatio(currentWheelSpinningSpeeds[0 + i], fowardSpeedOverGroundAtWheel[0 + i], forceDirection);
            float rightWheelLongitudalSlipRatio = GetDirectionalLongitudalSlipRatio(currentWheelSpinningSpeeds[1 + i], fowardSpeedOverGroundAtWheel[1 + i], forceDirection);
            //Debug.Log("leftWheelLongitudalSlipRatio " + leftWheelLongitudalSlipRatio);
            //Debug.Log("rightWheelLongitudalSlipRatio " + rightWheelLongitudalSlipRatio);


            leftRelSpinShare = rightWheelLongitudalSlipRatio - leftWheelLongitudalSlipRatio + 0.5f;
            if (leftRelSpinShare < 0) leftRelSpinShare = 0;
            else if (leftRelSpinShare > 1) leftRelSpinShare = 1;
            rightRelSpinShare = 1 - leftRelSpinShare;

            //Debug.Log("leftRelSpinShare = " + leftRelSpinShare);

            //Debug.Log("rearWheelsPowerShare " + rearWheelsPowerShare);
            //Debug.Log("frontWheelsPowerShare " + frontWheelsPowerShare);

            powerShareForWheelRotation[0+i] = (i < 2 ? fWPowerReservedForBothWheelSpins : rWPowerReservedForBothWheelSpins) * leftRelSpinShare;
            powerShareForWheelRotation[1+i] = (i < 2 ? fWPowerReservedForBothWheelSpins : rWPowerReservedForBothWheelSpins) * rightRelSpinShare;
            //Debug.Log("powerShareForWheelRotation[0+i] " + powerShareForWheelRotation[0 + i]);
            //Debug.Log("powerShareForWheelRotation[1+i] " + powerShareForWheelRotation[1 + i]);

            //the remaining motorPower is splitten along the wheels scaled proportional to their estimated available longitudal grip force
            float remainingMotorShare = (i < 2 ? frontWheelsPowerShare : rearWheelsPowerShare) - (i < 2 ? fWPowerReservedForBothWheelSpins : rWPowerReservedForBothWheelSpins);
            float leftRelGrip  = normalForcesUsedForGrip[0 + i] * longitudalRatioAndDirectionalScalingGripAtPreviouseFrame[0 + i];
            float rightRelGrip = normalForcesUsedForGrip[1 + i] * longitudalRatioAndDirectionalScalingGripAtPreviouseFrame[1 + i];
            if (leftRelGrip == 0 && rightRelGrip == 0) //prevent division by 0
            {
                leftRelGrip = 0.5f;
                rightRelGrip = 0.5f;
            }
            else if (leftRelGrip < 0 || rightRelGrip < 0) //this happens when it did not touch the ground previous physic frame. Therefore only the relation between normal forces for grip is used
            {
                leftRelGrip = normalForcesUsedForGrip[0 + i];
                rightRelGrip = normalForcesUsedForGrip[1 + i];
            }
            float leftRelShareForMovement = leftRelGrip / (leftRelGrip + rightRelGrip);
            float rightRelShareForMovement = 1 - leftRelShareForMovement;
            powerShareForMovement[0 + i] = remainingMotorShare * leftRelShareForMovement;
            powerShareForMovement[1 + i] = remainingMotorShare * rightRelShareForMovement;

            //combine motor-/break-power with slow from rolling friction. So far we used the power available for one second. To get the power available for this physicFrame we multiply the result with Time.fixedDeltaTime;
            availablePowerForFrame[0 + i] = ((powerShareForWheelRotation[0 + i] + powerShareForMovement[0 + i]) * forceDirection * totalAvailablePower + rollingFrictioSlowdownForEachWheel[0 + i] * (-wheelSpinningDirectionalMultiplier))*Time.fixedDeltaTime;
            availablePowerForFrame[1 + i] = ((powerShareForWheelRotation[1 + i] + powerShareForMovement[1 + i]) * forceDirection * totalAvailablePower + rollingFrictioSlowdownForEachWheel[1 + i] * (-wheelSpinningDirectionalMultiplier))*Time.fixedDeltaTime;
            if(actionMode == ActionMode.AccelerateByMotor) //only accelerating could lead to a direction change. All types of breaking can not (variable is false by default)
            {
                //accelerating can lead to a direction change, but only when the rolling-friction-slowdown does not cancle it out, because that would make it a form of breaking in total
                canLeadToDirectionChange[0 + i] = availablePowerForFrame[0 + i] * forceDirection < 0 ? false : true; 
                canLeadToDirectionChange[1 + i] = availablePowerForFrame[1 + i] * forceDirection < 0 ? false : true;
            }
        }

        //TO DO: ADD DECCELERATION FROM AIR RESISTANCE AND USE VELOCITY WITH ADDED VALUE FOR GRIP EVALUATION (maybe this should aleready be done before power distribution)


        for(int i=0; i<4; i++) //for every wheel
        {
            //Debug.Log("iteration: " + i + "dirChangePossible:"+canLeadToDirectionChange[i]);
            //Debug.Log("availablePowerForFrame initially is: " + availablePowerForFrame[i]);
            float availablePower = availablePowerForFrame[i]; //(note: can be negative)
            float lengthwiseSpeedOverGround = fowardSpeedOverGroundAtWheel[i];
            float spinningSpeed = currentWheelSpinningSpeeds[i];
            if(actionMode != ActionMode.AccelerateByMotor)
            {
                //ensures that breaking does not lead to acceleration. Therefore it must be working against the spinning speed or working against the movement when wheel spin is already lockt to zero
                availablePower = Mathf.Abs(availablePower) * (spinningSpeed > 0 ? -1 : spinningSpeed < 0 ? 1 : lengthwiseSpeedOverGround > 0 ? -1 : lengthwiseSpeedOverGround < 0 ? 1 : 0);
            }
            int availablePowerDirection = availablePower < 0 ? -1 : availablePower > 0 ? 1 : lengthwiseSpeedOverGround < spinningSpeed ? 1 : -1;
            //Debug.Log("availablePower= " + availablePower);
            //Debug.Log("spinningSpeed before = " + spinningSpeed);
            //the gripOptimiser uses less of the force to get a better slip ratio leading to better grip. At acceleration this prevents the wheels from spinning to fast and at breaking it prevents the wheels to lock too hard
            //float gripOptimiser = actionMode == ActionMode.AccelerateByMotor ? limitSpeedForGrip : actionMode == ActionMode.Break ? breakLessToOptimiseGrip : actionMode == ActionMode.Handbreak ? handBreakLessToOptimiseGrip : 0;

            //Szenario 0: Suprisingly no force has to be applied at all. Therefor only the current wheel spin needs to be computed.
            //if (availablePower == 0) {

            //    forceDirection = lengthwiseSpeedOverGround < spinningSpeed ? 1 : -1;
            //} 

            //bool fowardPower = availablePower > 0;
            float powerRatioForRotation = i < 2 ? frontWheelRotationHoldsRatioOfSpeed : rearWheelRotationHoldsRatioOfSpeed;

            //SZENARIO 0: The wheel spin is not even equal to speed over ground into the direction of force applied. In that case power is spend first to make the wheel spin match.
            // If all force will be spend without fixing the opposing spin direction, go on with Szenario 1. If the wrong spin direction is surmountable and force is still remaining, continiue with Szenario 2.
            // At the rear case, where the force is cancled out perfectly and the spin also matches the ground, nothing has to be done.
            //if (fowardPower)
            //{
            if (lengthwiseSpeedOverGround * availablePowerDirection > spinningSpeed * availablePowerDirection)
            {
                float neededPowerToMakeSpinMatch = powerRatioForRotation * (lengthwiseSpeedOverGround - spinningSpeed);
                if (neededPowerToMakeSpinMatch * availablePowerDirection >= availablePower * availablePowerDirection)
                {
                    spinningSpeed = Mathf.Lerp(spinningSpeed, lengthwiseSpeedOverGround, availablePower / neededPowerToMakeSpinMatch);
                    availablePower = 0;
                    if(float.IsNaN(spinningSpeed)) { Debug.LogError("invalide SpinningSpeed in SZENARIO 0"); }
                    //the entire force from friction/break/motor was already used to make the wheelspin match more to the speed over the ground,
                    //so the force direction is no longer decided from their direction. Instead the direction is foward or backward depending on the spin relative to its speed over ground
                    availablePowerDirection = lengthwiseSpeedOverGround < spinningSpeed ? 1 : -1;
                    // -> this will lead to Szenario 1
                    //Debug.Log("Szenario 0.0");
                }
                else
                {
                    spinningSpeed = lengthwiseSpeedOverGround;
                    availablePower -= neededPowerToMakeSpinMatch;
                    // -> this will lead to Szenario 2
                    //Debug.Log("Szenario 0.1");
                }
                //Debug.Log("spinningSpeed after S0 = " + spinningSpeed+", availablePower after = "+availablePower);

            }
            //}
            //else //(note: the available power can be negative for backward power. Keep this in mind when looking at the operators of scopes like this, where fowardPower == false)
            //{
            //    if (lengthwiseSpeedOverGround < spinningSpeed)
            //    {
            //        float neededPowerToMakeSpinMatch = powerRatioForRotation * (lengthwiseSpeedOverGround - spinningSpeed);
            //        if (neededPowerToMakeSpinMatch <= availablePower)
            //        {
            //            spinningSpeed = Mathf.Lerp(spinningSpeed, lengthwiseSpeedOverGround, availablePower / neededPowerToMakeSpinMatch);
            //            goto beforeGripByOnlySpinOffset;
            //        }
            //        else
            //        {
            //            spinningSpeed = lengthwiseSpeedOverGround;
            //            availablePower -= neededPowerToMakeSpinMatch;
            //        }
            //    }
            //}

            //ansonsten gibt es:
            // wheelspin, grip, power at:
            //  (unbekannte Position in Reihenfolge). current wheel spin
            //  2. same wheel spin as ground
            //  3. best grip wheel spin
            //  4. full power into spin 

            //PREPARATION for Szenario 1 and 2:

            //do it for current wheel spinn:
            float wheelRotationHoldsRatioOfSpeed = i < 2 ? frontWheelRotationHoldsRatioOfSpeed : rearWheelRotationHoldsRatioOfSpeed;

            (float lengthwise, float sideways) availableGripWithCurrentSpin = GetSpecificGripPower(spinningSpeed, lengthwiseSpeedOverGround, sideSlideSpeeds[i], availablePowerDirection, collidedGroundType[i], normalForcesUsedForGrip[i], i < 2);

            //find spinning speed for best slip ratio (the slip ratio, where the best grip is reached)
            float totalCurrentSpeedOverGround = Mathf.Sqrt(lengthwiseSpeedOverGround * lengthwiseSpeedOverGround + sideSlideSpeeds[i] * sideSlideSpeeds[i]);
            float bestSlipRatio = collidedGroundType[i] == 1 ? gripOnSolidGround.slipRatioOfMaxGrip : 2; //ToDo: case für keinen ground einfügen und gripOnLooseGround.slipRatioOfMaxGrip einfügen.
            //float bestSlip = bestSlipRatio * totalCurrentSpeedOverGround;
            //float bestLengthwiseSlip = Mathf.Sqrt(bestSlip * bestSlip - sideSlideSpeeds[i] * sideSlideSpeeds[i]);
            //if (float.IsNaN(bestLengthwiseSlip)) { bestLengthwiseSlip = 0;} //this means the best slip ratio can not even be reached //To Do: create special case for this or threat it in other cases
            //float plannedSpinningSpeedForBestSlipRatio = lengthwiseSpeedOverGround + availablePowerDirection * bestLengthwiseSlip;

            float slipAngleRadiants = GetAbsSlipAngle(lengthwiseSpeedOverGround, sideSlideSpeeds[i]) * Mathf.Deg2Rad;
            float minSlipRatioByAngle = Mathf.Sqrt(2 - 2 * Mathf.Cos(slipAngleRadiants));
            float plannedK;
            if(minSlipRatioByAngle >= bestSlipRatio)
            {
                // when the best slip Ratio can not be reached, even without longitudal slip, because lateral slip already creates a too high value,
                // at least the best available Slip is reached with K = 1 
                plannedK = 1;
            }
            else
            {
                plannedK = Mathf.Cos(slipAngleRadiants) + 0.5f * Mathf.Sqrt(2) * Mathf.Sqrt(Mathf.Cos(slipAngleRadiants * 2) + 2 * bestSlipRatio * bestSlipRatio - 1);
            }

            float plannedSpinningSpeedForBestSlipRatio = lengthwiseSpeedOverGround + Mathf.Abs(lengthwiseSpeedOverGround) * (plannedK - 1) * availablePowerDirection;

            float availablePowerForBestSlipRatio = availablePower + (spinningSpeed - plannedSpinningSpeedForBestSlipRatio) * wheelRotationHoldsRatioOfSpeed;
            (float lengthwise, float sideways) availableGripPowerWithBestSlipRatioSpin = GetSpecificGripPower(plannedSpinningSpeedForBestSlipRatio, lengthwiseSpeedOverGround, sideSlideSpeeds[i], availablePowerDirection, collidedGroundType[i], normalForcesUsedForGrip[i], i < 2);

            //float availablePowerForSameSpinAsGround = availablePower + (spinningSpeed - lengthwiseSpeedOverGround) * wheelRotationHoldsRatioOfSpeed;
            //(float lengthwise, float sideways) availableGripPowerWithSameSpinAsGround = GetSpecificGripPower(lengthwiseSpeedOverGround, lengthwiseSpeedOverGround, sideSlideSpeeds[i], forceDirection, collidedGroundType[i], normalForcesUsedForGrip[i], i < 2);



            float plannedSpin = spinningSpeed;
            float powerWithPlannedSpin = availablePower;
            (float lengthwise, float sideways) availableGripWithPlannedSpin = (0,0);


            //To DO: double check, if it realy works for all cases, where force is backwards
            //SZENARIO 1: when there is more Grip, than the  force to change the cars speed, the offset from wheel spin to speed over ground becomes smaler till it matches in speed or force
            //the difference of the wheel spin and available power between the current conditions and what would be, if the wheels spin as fast as the lenghwise speed over the ground
            if (availableGripWithCurrentSpin.lengthwise > availablePower * availablePowerDirection)
            {  
                float spinDif = lengthwiseSpeedOverGround - spinningSpeed;
                float powerDif = (spinningSpeed - lengthwiseSpeedOverGround) * wheelRotationHoldsRatioOfSpeed;

                bool gripIsHigher = true;
                //aproximate the wheel spin, where the available power == available grip. This is done by 20 iterations, leading to an accuracy of spinDif/1,048,576
                for (int j = 1; j < 21; j++)
                {
                    if (gripIsHigher)
                    {
                        plannedSpin += spinDif / Mathf.Pow(2, j);
                        powerWithPlannedSpin += powerDif / Mathf.Pow(2, j);
                    }
                    else
                    {
                        plannedSpin -= spinDif / Mathf.Pow(2, j);
                        powerWithPlannedSpin -= powerDif / Mathf.Pow(2, j);
                    }

                    availableGripWithPlannedSpin = GetSpecificGripPower(plannedSpin, lengthwiseSpeedOverGround, sideSlideSpeeds[i], availablePowerDirection, collidedGroundType[i], normalForcesUsedForGrip[i], i < 2);
                    if (availableGripWithPlannedSpin.lengthwise > powerWithPlannedSpin * availablePowerDirection)
                    {
                        gripIsHigher = true;
                    }
                    else if (availableGripWithPlannedSpin.lengthwise < powerWithPlannedSpin * availablePowerDirection)
                    {
                        gripIsHigher = false;
                    }
                    else
                    {
                        break; //reached spin where grip matches power
                    }
                }
                if (float.IsNaN(plannedSpin)) { Debug.LogError("invalide SpinningSpeed in SZENARIO 1"); }
                //Debug.Log("Szenario 1");
            }
            //SZENARIO 2: when there is more force to change the cars speed, than grip to apply this force, the offset rises till the forces match or all excess energy turns into wheel spin (unless specifically regulated)
            else if (availableGripWithCurrentSpin.lengthwise < availablePower * availablePowerDirection)
            {                                                                                   
                //To Do: bei folgendem darauf achten, dass es auch für rückwärts force funktioniert
                if (spinningSpeed * availablePowerDirection < plannedSpinningSpeedForBestSlipRatio * availablePowerDirection) //having a lower slip, than the perfect slip ratio
                {
                    if (availableGripPowerWithBestSlipRatioSpin.lengthwise > availablePowerForBestSlipRatio * availablePowerDirection) //if the grip is yet not good enough at the current spin, but would be, if it spins at the perfect slip ratio, find the spinSpeed between thouse, where grip and power are equal.
                    {
                        //the difference of the wheel spin and available power between the current conditions and what would be, if the wheels spin as fast as needed for the perfect slip ratio
                        float spinDif = plannedSpinningSpeedForBestSlipRatio - spinningSpeed;
                        float powerDif = (spinningSpeed - plannedSpinningSpeedForBestSlipRatio) * wheelRotationHoldsRatioOfSpeed;

                        bool gripIsHigher = false;
                        //aproximate the wheel spin, where the available power == available grip. This is done by 20 iterations, leading to an accuracy of spinDif/1,048,576
                        for (int j = 1; j < 21; j++)
                        {
                            if (gripIsHigher)
                            {
                                plannedSpin -= spinDif / Mathf.Pow(2, j);
                                powerWithPlannedSpin -= powerDif / Mathf.Pow(2, j);
                            }
                            else
                            {
                                plannedSpin += spinDif / Mathf.Pow(2, j);
                                powerWithPlannedSpin += powerDif / Mathf.Pow(2, j);
                            }

                            availableGripWithPlannedSpin = GetSpecificGripPower(plannedSpin, lengthwiseSpeedOverGround, sideSlideSpeeds[i], availablePowerDirection, collidedGroundType[i], normalForcesUsedForGrip[i], i < 2);
                            if (availableGripWithPlannedSpin.lengthwise > powerWithPlannedSpin * availablePowerDirection)
                            {
                                gripIsHigher = true;
                            }
                            else if (availableGripWithPlannedSpin.lengthwise < powerWithPlannedSpin * availablePowerDirection)
                            {
                                gripIsHigher = false;
                            }
                            else
                            {
                                break; //reached spin where grip matches power
                            }
                        }
                        if (float.IsNaN(plannedSpin)) { Debug.LogError("invalide SpinningSpeed in SZENARIO 2.0"); }
                        //Debug.Log("Szenario 2.0");
                    }
                    else //if even the perfect slip ratio grip is not good enought to use to power, put energy, which can not be used otherwise, into wheel spin
                    {
                        //since the perfect slip ratio grip at least happens during this frame (allthrough only for a moment), the power is converted into speed by that grip, while remaining power might change the wheel spin
                        powerWithPlannedSpin = availableGripPowerWithBestSlipRatioSpin.lengthwise * availablePowerDirection;
                        availableGripWithPlannedSpin = availableGripPowerWithBestSlipRatioSpin;
                        float remainingPower = availablePower - availableGripPowerWithBestSlipRatioSpin.lengthwise * availablePowerDirection;
                        float fullPoweredSpin = plannedSpinningSpeedForBestSlipRatio + remainingPower/ wheelRotationHoldsRatioOfSpeed;
                        plannedSpin = Mathf.Lerp(fullPoweredSpin, spinningSpeed, usedSlipPreventer);
                        if (float.IsNaN(plannedSpin)) { Debug.LogError("invalide SpinningSpeed in SZENARIO 2.1"); }
                        //Debug.Log("Szenario 2.1");
                    }
                }
                else //already having a higher slip, than the perfect slip ratio
                {
                    //first of all the current access slip above perfect slip ratio is reduced depending on the usedSlipPrevention - in a way, that the ratio of the usedSlipPrevention is applied over one second.
                    //as an example: when the usedSlipPrevention is 0.5, around 29% of access slip is prevented in half a second, 50% in one second, 75% in two seconds and around 34% in four seconds
                    float spinDifToPerfectSlipRatio = plannedSpinningSpeedForBestSlipRatio - spinningSpeed;
                    float spinChangeForSlipReduction = spinDifToPerfectSlipRatio * (1-Mathf.Pow(1-usedSlipPreventer, Time.fixedDeltaTime)); //(0 without slip prevention and as big as the spinDifToPerfectSlipRatio at full slip prevention)
                    plannedSpin = spinningSpeed + spinChangeForSlipReduction;
                    //now put current access available power into wheel spin (reduced by usedSlipPrevention)
                    float remainingPower = availablePower - availableGripWithCurrentSpin.lengthwise * availablePowerDirection;
                    float fullPoweredSpin = spinningSpeed + remainingPower / wheelRotationHoldsRatioOfSpeed;
                    plannedSpin = Mathf.Lerp(fullPoweredSpin, plannedSpin, usedSlipPreventer);
                    //use the new spin or the previous spin depending on what is lower
                    availableGripWithPlannedSpin = GetSpecificGripPower(spinningSpeed * availablePowerDirection < plannedSpin * availablePowerDirection ? spinningSpeed : plannedSpin,
                        lengthwiseSpeedOverGround, sideSlideSpeeds[i], forceDirection, collidedGroundType[i], normalForcesUsedForGrip[i], i < 2);
                    if (float.IsNaN(plannedSpin)) { Debug.LogError("invalide SpinningSpeed in SZENARIO 2.2"); }
                    //Debug.Log("Szenario 2.2");
                }


                
            }
            else //SZENARIO x: the aviable grip at the current wheel spinning speed fits exactly the power used to change the cars speed. Therefore the wheel spin does not change at all
            {
                availableGripWithPlannedSpin = GetSpecificGripPower(plannedSpin, lengthwiseSpeedOverGround, sideSlideSpeeds[i], availablePowerDirection, collidedGroundType[i], normalForcesUsedForGrip[i], i < 2);
                if (float.IsNaN(plannedSpin)) { Debug.LogError("invalide SpinningSpeed in SZENARIO 3"); }
                //Debug.Log("Szenario 3");
            }


            //all forms of breaking or slowdown can slow the wheel spin down or even lock them, but should not be able to accelerate the spin into the opposite direction.
            //The following lines prevent cases, where the previous code wrongfully created that behaviour
            if (!canLeadToDirectionChange[i] && (plannedSpin*lengthwiseSpeedOverGround<0) && (spinningSpeed * plannedSpin < 0 || (spinningSpeed==0 && plannedSpin !=0)))
            {
                plannedSpin = 0;
                availableGripWithPlannedSpin = GetSpecificGripPower(plannedSpin, lengthwiseSpeedOverGround, sideSlideSpeeds[i], availablePowerDirection, collidedGroundType[i], normalForcesUsedForGrip[i], i < 2);
                powerWithPlannedSpin = availablePowerForFrame[i] + currentWheelSpinningSpeeds[i] * wheelRotationHoldsRatioOfSpeed;
                if (float.IsNaN(plannedSpin)) { Debug.LogError("invalide SpinningSpeed in SZENARIO anti-change-correction"); }
                //Debug.Log("Szenario anti-change-correction");
            }


            ////This part of the code is only called when the car does not apply any force to the wheel and therefore the offset spin to the ground has not been computet yet.
            ////In this case access spin gets (partly) balanced out (depending on grip) to better match the wheels speed over the ground. The force of the changed wheel spin changes the cars speed into the opposed direction.
            ////example: if wheels spin faster, than how fast they move over the ground, they accelerate the car and their own spin is slowed down in return. If they spin too slow, their rotation is accelerated on cost of the car's speed.



            if (powerWithPlannedSpin > availableGripWithPlannedSpin.lengthwise) { powerWithPlannedSpin = availableGripWithPlannedSpin.lengthwise; }
            else if (powerWithPlannedSpin < -availableGripWithPlannedSpin.lengthwise) { powerWithPlannedSpin = -availableGripWithPlannedSpin.lengthwise; }

            lengthwiseForceAtWheels[i] = powerWithPlannedSpin;
            sidewayGripAtWheels[i] = availableGripWithPlannedSpin.sideways;
            currentWheelSpinningSpeeds[i] = plannedSpin;
            
            //Debug.Log("lengthwiseForceAtWheels=" + lengthwiseForceAtWheels[i]);
            //Debug.Log("sidewayGrip=" + availableGripWithPlannedSpin.sideways);
            //Debug.Log("spin=" + plannedSpin);
            if (float.IsNaN(plannedSpin)) { Debug.LogError("wrongSpinMESSAGE works"); }

            if (normalForcesUsedForGrip[i] == 0)
            {
                longitudalRatioAndDirectionalScalingGripAtPreviouseFrame[i] = -1; //this indicates, that the ground was not touched and is marked with "-1" to be treated differently;
            }
            else
            {
                longitudalRatioAndDirectionalScalingGripAtPreviouseFrame[i] = availableGripWithPlannedSpin.lengthwise / normalForcesUsedForGrip[i];
            }
            //Debug.Log("fowardOnGround is: " + fowardOnGround[i] + "  hitPointForLongitudal is: " + hitPointForLongitudal[i]);
            rb.AddForceAtPosition(fowardOnGround[i] * lengthwiseForceAtWheels[i], hitPointForLongitudal[i], ForceMode.Impulse);
            Debug.DrawRay(hitPoints[i], (fowardOnGround[i] * lengthwiseForceAtWheels[i] - sideSlideDirections[i] * sidewayGripAtWheels[i])*10, Color.blue);
            //Debug.Log("lengthwise Grip: "+ lengthwiseForceAtWheels[i] + ", Side Grip is: " + sidewayGripAtWheels[i]);
            //Debug.Log("With direction - lengthwise Grip: " + fowardOnGround[i] * lengthwiseForceAtWheels[i] + ", Side Grip is: " + -sideSlideDirections[i] * sidewayGripAtWheels[i]);
        }
        currentAverageWheelSpinningSpeed = GetAverageWheelSpinningSpeed();
        

       //float rearWheelMotorShare = 1 - rearOrFrontPowered;



       //float averatgeWheelSpeed = (speedAtWheel[0] + speedAtWheel[1] + speedAtWheel[2] + speedAtWheel[3]) / 4f;
       //bool usingBackwardGear = currentGear < 0;
       //float gearDirection = usingBackwardGear ? -1 : 1;

       //float moveDirectionMultiplier = currentSpeed < 0 ? -1 : 1;
       ////VORISCHT: BISHER IST ALLES FOLGENDE NUR FÜR VORWÄRTS DREHENDE RÄTER IMPLEMENTIERT - oder doch schon?
       //float combinedAcceleration;
       //float basicDecceleration = -testBasicSlowdown; //Replace with actual current basic slowdown
       //float breakingDecceleration = -testBreakingSlowdown; //Replace with actual current breaking slowdown
       //float maxThrottleAcceleration = speedupCurve.GetAccelerationValueForSpeed(Mathf.Abs(currentAverageWheelSpinningSpeed)) * totalGearAccelerationMultiplier; //Remove Mathf.abs and replace with other curve later!!


       //bool canPassZeroByAcceleration; //TO DO: muss noch in logik eingebaut werden, nur beschleunigungne können aktiv richtungswechsel initiieren

       //if (HandbreakKey.Value > 0)
       //{
       //    //not implemented yet
       //    combinedAcceleration = -wheelSpinningDirectionalMultiplier;
       //}
       //else if(BreakKey.Value > 0)
       //{
       //    float deccelerationWithoutBreake;
       //    if (maxThrottleAcceleration + basicDecceleration < 0)
       //    {
       //        deccelerationWithoutBreake = maxThrottleAcceleration + basicDecceleration;
       //        if (deccelerationWithoutBreake < -testBreakingSlowdown) deccelerationWithoutBreake = -testBreakingSlowdown;
       //    }
       //    else
       //    {
       //        deccelerationWithoutBreake = basicDecceleration;
       //    }
       //    combinedAcceleration = Mathf.Lerp(deccelerationWithoutBreake,-testBreakingSlowdown, BreakKey.Value) * wheelSpinningDirectionalMultiplier;    
       //}
       //else if ( (gearShiftMode == GearShiftMode.Automatic || numberOfGears==0) ? (currentGear >=0 && ThrottleKey.Value > 0) : ThrottleKey.Value > 0) //TO DO: BACKWARD CASES
       //{ 

       //    if (maxThrottleAcceleration > 0) //Gear can create Acceleration for current wheel speed
       //    {
       //        combinedAcceleration = Mathf.Lerp(basicDecceleration * moveDirectionMultiplier, maxThrottleAcceleration * gearDirection, ThrottleKey.Value);
       //    }
       //    else if (maxThrottleAcceleration> breakingDecceleration) //Gear is so bad for current speed that it slows it down, but less than the breaking decceleration
       //    {
       //        float minusFromBasicDecceleration = Mathf.Lerp(basicDecceleration, 0, ThrottleKey.Value);
       //        combinedAcceleration = maxThrottleAcceleration + minusFromBasicDecceleration;
       //        if (combinedAcceleration < breakingDecceleration) combinedAcceleration = breakingDecceleration;
       //        combinedAcceleration *= wheelSpinningDirectionalMultiplier;

       //    }
       //    else //gear is so bad that it would slow down more than breaking, but we use breaking as a limit
       //    {
       //        combinedAcceleration = breakingDecceleration * wheelSpinningDirectionalMultiplier;
       //    }
       //}
       //else if (BackwardThrottleKey.Value > 0) //THIS HAS TO BE REPLACED WITH A CONCEPT FOR DRIVING BACKWARDS //TO DO: BACKWARD CASES
       //{
       //    if (maxThrottleAcceleration > 0) //Gear can create Acceleration for current wheel speed
       //    {
       //        combinedAcceleration = Mathf.Lerp(basicDecceleration, maxThrottleAcceleration, BackwardThrottleKey.Value);
       //    }
       //    else if (maxThrottleAcceleration > breakingDecceleration) //Gear is so bad for current speed that it slows it down, but less than the breaking decceleration
       //    {
       //        float minusFromBasicDecceleration = Mathf.Lerp(basicDecceleration, 0, BackwardThrottleKey.Value);
       //        combinedAcceleration = maxThrottleAcceleration + minusFromBasicDecceleration;
       //        if (combinedAcceleration < breakingDecceleration) combinedAcceleration = breakingDecceleration;
       //    }
       //    else //gear is so bad that it would slow down more than breaking, but we use breaking as a limit
       //    {
       //        combinedAcceleration = breakingDecceleration;
       //    }
       //    combinedAcceleration *= -1;
       //}
       //else //only rolling- and air resistance
       //{
       //    float deccelerationWithoutBreake;
       //    if (maxThrottleAcceleration + basicDecceleration < 0)
       //    {
       //        deccelerationWithoutBreake = maxThrottleAcceleration + basicDecceleration;
       //        if (deccelerationWithoutBreake < -testBreakingSlowdown) deccelerationWithoutBreake = -testBreakingSlowdown;
       //    }
       //    else
       //    {
       //        deccelerationWithoutBreake = basicDecceleration;
       //    }
       //    combinedAcceleration = Mathf.Lerp(deccelerationWithoutBreake, -testBreakingSlowdown, BreakKey.Value) * wheelSpinningDirectionalMultiplier;
       //}
       //for (int i = 0; i < 4; i++)
       //{
       //    rb.AddForceAtPosition(fowardOnGround[i] * combinedAcceleration / 4f, hitPointForLongitudal[i]);
       //}




       //SIDEWARD FRICTION

       (float maxV, float maxVDir, Vector3 maxW, Vector3 rotToV, Vector3 vDirection)[] impulseProperties = new (float maxV, float maxVDir, Vector3 maxW, Vector3 rotToV, Vector3 vDirection)[4];
        for (int i = 0; i < 4; i++)
        {
            //SIDEWARD FRICTION
            
            

            float maxStopImpulse;
            if (endlessSidewaysGrip)
            {
                maxStopImpulse = sideSlideSpeeds[i]* rb.mass / 4; 

            }
            else
            {
                //maxStopImpulse = normalForcesUsedForGrip[i] * 1.5f * Time.fixedDeltaTime;//verticalForce.magnitude*0.1f * Time.fixedDeltaTime; //TEST VALUE!!!
                maxStopImpulse = sidewayGripAtWheels[i];
                Debug.DrawRay(hitPoints[i],  - sideSlideDirections[i] * sidewayGripAtWheels[i] * 10, Color.green);
            }
           
            impulseProperties[i] = CalculateEffectOfImpulseOnDirectionalMovementAtPoint(hitPointForLateral[i], -sideSlideDirections[i], maxStopImpulse); 
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
                float neededOwnV = sideSlideSpeeds[i] - vFromOtherWheels;
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

        //Compute Ground-Air-Status
        PerformInAirBalance();

        //Manage Audio
        ManageAudioAndGUI(currentGear, currentAverageWheelSpinningSpeed, currentSpeed); //TO CHANGE: USE WHEEL SPIN INSTEAD


        //Debug.Log("estimatedVelNextFrame = " + GetActualPointVelocity(Vector3.one));
    }

















    private void FindGroundInteraction(out Vector3[] hitPoints, out Vector3[] hitNormals, out float[] springCompressions, out int[] collidedGroundType)
    {
        hitPoints = new Vector3[4];
        hitNormals = new Vector3[4];
        springCompressions = new float[] {0,0,0,0};
        collidedGroundType = new int[] { 0, 0, 0, 0 }; //0 = no; 1 = solidGround; 2 = looseGround
        int numberOfContacts = 0;

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
                    numberOfContacts++;
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
                if (springCompressions[i] != 0) numberOfContacts++;
            }
            Debug.DrawRay(hitPoints[i], hitNormals[i] * springCompressions[i], UnityEngine.Color.green);
            //Debug.Log("Spring compression[" + i + "]= " + springCompressions[i]);
        }
        numberOfGroundedWheels = numberOfContacts;
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

    void UpdateRawSteerAngle(ref float currentAngle, float maxAngle)
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

    void UpdateSteeringAngles(Vector3[] hitPoints)
    {
        // try to calculate the local position of the turning center from the Bicycle-Model to apply Ackermann-Steering
        float zDistBetweenRearAndFrontWheels = frontRightWheelCenter.z - rearRightWheelCenter.z;
        float frontZPerX = Mathf.Sin(frontSteeringAngle * Mathf.Deg2Rad);
        float rearZPerX = Mathf.Sin(rearSteeringAngle * Mathf.Deg2Rad);
        float linesGettingCloserOnYPerX = frontZPerX - rearZPerX;
        if(Mathf.Abs(linesGettingCloserOnYPerX) < 0.0000000000000001f)
        {
            //can not apply ackerman steering when front and rear wheels are parallel
            rawSteeringAngles[0] = frontSteeringAngle;
            rawSteeringAngles[1] = frontSteeringAngle;
            rawSteeringAngles[2] = rearSteeringAngle;
            rawSteeringAngles[3] = rearSteeringAngle;
        }
        else
        {
            // calculate the local position of the turning center
            float xOfTurningCenter = zDistBetweenRearAndFrontWheels / linesGettingCloserOnYPerX; // a positive value tells how far the turning center is on the right side, a negative for the left side
            float zDistFromFrontWheels = xOfTurningCenter * frontZPerX;
            float zDistFromRearWheels = xOfTurningCenter * rearZPerX;
            
            // calculate the individually adapted steering angles of ackermann steering
            float rFrontAckermannAngle = Mathf.Atan(zDistFromFrontWheels / (xOfTurningCenter - frontRightWheelCenter.x)) * Mathf.Rad2Deg;
            float lFrontAckermannAngle = Mathf.Atan(zDistFromFrontWheels / (xOfTurningCenter + frontRightWheelCenter.x)) * Mathf.Rad2Deg;
            float rRearAckermannAngle = Mathf.Atan(zDistFromRearWheels / (xOfTurningCenter + rearRightWheelCenter.x)) * Mathf.Rad2Deg;
            float lRearAckermannAngle = Mathf.Atan(zDistFromRearWheels / (xOfTurningCenter - rearRightWheelCenter.x)) * Mathf.Rad2Deg;

            //Lerp between basic steering Angle and the adapted value for Ackermann Steering. A negative value of Ackermann Steering creates "Anti-Ackermann-Steering" (which is the literal opposide)
            rawSteeringAngles[0] = Mathf.LerpUnclamped(frontSteeringAngle, rFrontAckermannAngle, ackermanSteering);
            rawSteeringAngles[1] = Mathf.LerpUnclamped(frontSteeringAngle, lFrontAckermannAngle, ackermanSteering);
            rawSteeringAngles[2] = Mathf.LerpUnclamped(rearSteeringAngle, rRearAckermannAngle, ackermanSteering);
            rawSteeringAngles[3] = Mathf.LerpUnclamped(rearSteeringAngle, lRearAckermannAngle, ackermanSteering);

        }




        //rawSteeringAngles[0] = frontSteeringAngle;
        //rawSteeringAngles[1] = frontSteeringAngle;
        //rawSteeringAngles[2] = rearSteeringAngle;
        //rawSteeringAngles[3] = rearSteeringAngle;



        //from the rawSteeringAngle some adjustment towards the sliding direction happens here to get the definite steeringAngle
        for(int i=0; i<4; i++)
        {
            Vector3 fowardAtRawSteer = rb.rotation * (Quaternion.Euler(0, rawSteeringAngles[i], 0) * Vector3.forward);
            Vector3 velAtWheel = rb.GetPointVelocity(hitPoints[i]);
            Vector3 upwardPartOfVelAtWheel = Vector3.Dot(velAtWheel, transform.up)*transform.up;
            float singedDeltaAngle = Vector3.SignedAngle(fowardAtRawSteer, velAtWheel, transform.up);
            //handle driving backwards
            if (singedDeltaAngle > 90) singedDeltaAngle -= 180;
            else if (singedDeltaAngle < -90) singedDeltaAngle += 180;
            float plannedOffAngle = singedDeltaAngle * (i < 2 ? frontSteerTowardsSlide : rearSteerTowardsSlide);
            //the allignment towards sliding direction fades out at speeds below 1
            if((velAtWheel-upwardPartOfVelAtWheel).magnitude < 2)
            {
                plannedOffAngle = Mathf.Lerp(0, plannedOffAngle, velAtWheel.magnitude/2);
            }
            float usedMaxSteeringTowardsSlideAngle = i < 2 ? maxFrontAngleTowardsSlide : maxRearAngleTowardsSlide;
            if(plannedOffAngle>usedMaxSteeringTowardsSlideAngle) plannedOffAngle = usedMaxSteeringTowardsSlideAngle;
            if (plannedOffAngle < -usedMaxSteeringTowardsSlideAngle) plannedOffAngle = -usedMaxSteeringTowardsSlideAngle;
            //Debug.Log("plannedOffAngle" + plannedOffAngle);

            //prevent sudden changes (especially important when sliding around 90 degree sideways, which would lead to sudden corrections towards driving foward and backward)
            float maxOffAngleChangeThisPhysicsFrame = 200 * Time.fixedDeltaTime; //can change maximum 200 Degree per second;
            if (plannedOffAngle - prevOffAngles[i] > maxOffAngleChangeThisPhysicsFrame) plannedOffAngle = prevOffAngles[i] + maxOffAngleChangeThisPhysicsFrame;
            if (plannedOffAngle - prevOffAngles[i] < -maxOffAngleChangeThisPhysicsFrame) plannedOffAngle = prevOffAngles[i] - maxOffAngleChangeThisPhysicsFrame;
            prevOffAngles[i] = plannedOffAngle;

            steeringAngles[i] = rawSteeringAngles[i] + plannedOffAngle;

            Debug.DrawRay(hitPoints[i], fowardAtRawSteer.normalized * 2, Color.gray);
            Debug.DrawRay(hitPoints[i], rb.rotation * (Quaternion.Euler(0, steeringAngles[i], 0) * Vector3.forward).normalized * 2, Color.white);
        }
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



    void ManageAudioAndGUI(int currentGear, float currentWheelSpinSpeed, float currentSpeed)//toTo: Rückwärtsgang   
    {
        if (audioForEngine == null) return;
        bool isInNegativeGear = currentGear < 0;
        if(currentGear <0) currentGear = 0;

        float pitchRawChange = 0;
        float totalPitch = selectedBasePitchForEngine;

        if (numberOfGears != 0) {
            float currenSpeedRatio = speedupCurve.GetTimeInCurve(currentWheelSpinSpeed);
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
        float volumeScaledByGearUneffectiveness = 1 / (1 + GetGearUneffectivenessRating(isInNegativeGear? -1 : currentGear, currentWheelSpinSpeed)*volumeDropOffAtLowGearEffectiveness);
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
            average += currentWheelSpinningSpeeds[i] / 4; //* (i < 2 ? rearOrFrontPowered: 1-rearOrFrontPowered) / 2;
        }
        return average;
    }

    float GetAirResistanceSlowdownForSpeedAbs(float speed)
    {
        return slowByAirResistanceAt30ms  * Mathf.Pow(Mathf.Abs(speed/30), airResistanceExponent);
    }

    float[] GetRollingFrictionSlowdonwForEachWheelAbs(float[] normalForcesUsedForGrip)
    {
        float[] rollingFriction = new float[4];
        for(int i = 0; i < 4; i++)
        {
            rollingFriction[i] = normalForcesUsedForGrip[i] * slowByRollFriction;
        }
        return rollingFriction;
    }

    (float atFullThrottle, float atNoThrottle) GetMotorAccelerationAbs()
    {
        float atFullThrottle;
        float atNoThrottle;
        //float spinDirectionMultiplier = currentAverageWheelSpinningSpeed < 0 ? -1 : 1;
        accelerationScalerByClutch = Mathf.Lerp(1, GetClutchValue(), shiftingImpactOnAcceleration);

        if (Mathf.Abs(currentAverageWheelSpinningSpeed) < speedupCurve.topSpeed) //below max Speed
        {
            float relSpinSpeed = speedupCurve.GetTimeInCurve(Mathf.Abs(currentAverageWheelSpinningSpeed));
            float baseAccelerationAtCurrentSpeed = speedupCurve.GetAccelerationValueForSpeed(Mathf.Abs(currentAverageWheelSpinningSpeed));
            float accelerationScalerBySuitabilityOfGear = GetAccelerationScaleByGear(currentGear, currentAverageWheelSpinningSpeed);
            float gearsOptimumRelSpeed = currentGear < 0 ? gearIsBestAtRelSpeed[0] : gearIsBestAtRelSpeed[currentGear];
            bool spinFasterThanOptimum = (relSpinSpeed > gearsOptimumRelSpeed);

            float airResistanceToOvercome = GetAirResistanceSlowdownForSpeedAbs(Mathf.Abs(currentAverageWheelSpinningSpeed));

            atFullThrottle = (baseAccelerationAtCurrentSpeed * accelerationScalerBySuitabilityOfGear + airResistanceToOvercome + slowByRollFriction);// * spinDirectionMultiplier; //(To DO: adjust slowByRollingFriction for current wheel spin)

            if (spinFasterThanOptimum) //The motor can only create a slowdown when the gear is too low for the current speed
            {
                atNoThrottle = baseAccelerationAtCurrentSpeed * (accelerationScalerBySuitabilityOfGear - 1);// * spinDirectionMultiplier;
            }
            else
            {
                atNoThrottle = 0;
                if (atFullThrottle < 0) atFullThrottle = 0;
            }

            
        }
        else //above max Speed
        {
            float theoreticalAccelerationAtMaxSpeed = speedupCurve.GetAccelerationValueForSpeed(Mathf.Abs(speedupCurve.topSpeed));
            float accelerationScalerBySuitabilityOfGear = GetAccelerationScaleByGear(currentGear, currentAverageWheelSpinningSpeed);
            //float freeBadGearRatio = 1 - gearScaleAtMinimum;
            float punishmentForBadGearRatio = (accelerationScalerBySuitabilityOfGear - gearScaleAtMinimum) * theoreticalAccelerationAtMaxSpeed; //is 0 or negative because the accelerationScalerBSOG is <= gearScaleAtMinimum
            float airResistanceToOvercomeAtMaxSpeed = GetAirResistanceSlowdownForSpeedAbs(speedupCurve.topSpeed);

            atFullThrottle = (airResistanceToOvercomeAtMaxSpeed + slowByRollFriction + punishmentForBadGearRatio);// *spinDirectionMultiplier; //(TO DO: adjust slowByRollingFriction to topSpeed)
            atNoThrottle = punishmentForBadGearRatio;// *spinDirectionMultiplier;
        }


        return (atFullThrottle*accelerationScalerByClutch, atNoThrottle*accelerationScalerByClutch);
    }

    (float slilpRatio, bool usesFowardForce) GetSlipRatio(float fowardSpeedOverGround, float sidewaysSlip, float wheelSpin, int intendedDirection)
    {
        bool usesFowardForce = fowardSpeedOverGround < wheelSpin || (fowardSpeedOverGround==wheelSpin && intendedDirection>0);
        //bool useSlipRatioForBreaking = Mathf.Abs(fowardSpeedOverGround) > Mathf.Abs(wheelSpin) || (Mathf.Abs(fowardSpeedOverGround) == Mathf.Abs(wheelSpin) && (wheelSpin*intendedDirection<0));
        bool oppositeDirection = fowardSpeedOverGround * wheelSpin < 0;
        //float slipMagnitude = Mathf.Sqrt(fowardSpeedOverGround*fowardSpeedOverGround+sidewaysSlip*sidewaysSlip);

        //the original formula does not tread slip Angles above 90 Degreee correctly, therefore the Absolute value of "fowardSpeedOverGround" is used
        float slipAngle = GetAbsSlipAngle(fowardSpeedOverGround, sidewaysSlip);

        float slipRatio;

        
        float K = Mathf.Abs(wheelSpin) / Mathf.Abs(fowardSpeedOverGround);
        if (K < 1) K = 2 - K; // The original formula does not work correctly for breaking (aka K < 1). This fixes the issue by treating it equal to acceleration
        // |1-K| would be the lengthwise slipRatio
        // the slipAngle can also increase the slipRatio and leads to a SlipRatio of 1 by itself at 90 Degree
        slipRatio = Mathf.Sqrt(Mathf.Pow(K * Mathf.Sin(slipAngle*Mathf.Deg2Rad), 2) + Mathf.Pow(1 - K * Mathf.Cos(slipAngle*Mathf.Deg2Rad), 2));

        // the original formula does not tread movement and wheel spin in different directions correctly.
        // The opposite-direction-wheel speed has more slip than a locked-wheel-break and is therefore set to maximum slip
        if (oppositeDirection) slipRatio = 1;

        slipRatio = Mathf.Clamp01(slipRatio);

        return (slipRatio, usesFowardForce);
    }

    float GetAbsSlipAngle(float fowardSpeedOverGround, float sidewaysSlip)
    {
        return Vector2.Angle(Vector2.up, new Vector2(sidewaysSlip, Mathf.Abs(fowardSpeedOverGround)));
    }

    float GetRawGripBySlipRatioAbs (float slipRatio, int groundType)
    {
        if (groundType < 0 || groundType > 2) Debug.LogError("groundType" + groundType + " is not valide!");
        return groundType == 0 ? 0: groundType == 1? gripOnSolidGround.curve.Evaluate(Mathf.Abs(slipRatio)) : gripOnLooseGround.curve.Evaluate(Mathf.Abs(slipRatio));
    }

    //Just visual inspector feedback
    void OnDrawGizmos()
    {
        DrawPhysical3DWheelShape();
    }
    void DrawPhysical3DWheelShape()
    {
        for (int i = (showWheelSettings && frontUse3DWheelPhysics ? 0 : 2); i < (showWheelSettings && rearUse3DWheelPhysics ? 4 : 2); i++)
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

    float GetDirectionalLongitudalSlipRatio(float spinSpeed, float groundSpeed, int wantedDirection)
    {
        //the slipRatio in this method is calculated with a bias to apply force into a direction
        float slipRatio;
        spinSpeed *= wantedDirection;
        groundSpeed *= wantedDirection;

        //Debug.Log("spin Speed = " + spinSpeed);
        //Debug.Log("ground Speed = " + groundSpeed);

        //If angle a=0 Then Mathf.Sqrt((K*sin(a))^2 + (1- K*cos(a))^2 ) = |1-K| .  Since K = |spinSpeed| / |groundSpeed|. It is the same like |spinSpeed-groundSpeed|/groundSpeed

        if (spinSpeed >= 0)
        {
            if (groundSpeed >= 0)
            {
                slipRatio = (spinSpeed-groundSpeed)/groundSpeed;
                if(slipRatio > 1) slipRatio = 1;
                if(slipRatio < 0) slipRatio = 0;
            }
            else
            {
                slipRatio = 1;
            }
        }
        else
        {
            if (groundSpeed >= 0)
            {
                slipRatio=0;
            }
            else
            {

                slipRatio =  (groundSpeed-spinSpeed)/ groundSpeed;
                if (slipRatio > 1) slipRatio = 1;
                if (slipRatio < 0) slipRatio = 0;
            }
        }
        return slipRatio;
    }

    float GetUsedGripMultiplier(int groundType, bool frontWheel, bool lengthwise) //returns 0 when no ground collision happened
    {
        float gripMultiplier;

        if(groundType == 0)
        {
            return 0;//no ground collision
        }
        else if(groundType == 1 || useSameGripMultipliersAsSolidGround)
        {
            if(frontWheel || sameGripSettingsForRearWheel)
            {
                if (lengthwise)
                {
                    return FwSgLengthwiseGrip;
                }
                else
                {
                    return FwSgSidewaysGrip;
                }
                
            }
            else
            {
                if (lengthwise)
                {
                    return RwSgLengthwiseGrip;
                }
                else
                {
                    return RwSgSidewaysGrip;
                }
            }
        }
        else if(groundType == 2)
        {
            if (frontWheel || sameGripSettingsForRearWheel)
            {
                if (lengthwise)
                {
                    return FwLgLengthwiseGrip;
                }
                else
                {
                    return FwLgSidewaysGrip;
                }

            }
            else
            {
                if (lengthwise)
                {
                    return RwLgLengthwiseGrip;
                }
                else
                {
                    return RwLgSidewaysGrip;
                }
            }
        }
        else
        {
            Debug.LogError(groundType + " is no valide GroundType");
            return -1;
        }

    }

    (float lenghthwise, float sideways) GetSpecificGripPower(float spinningSpeed, float speedOverGround, float sideSlideSpeed, int forceDirection, int collidedGroundType, float normalForcesUsedForGrip, bool isFrontWheel)
    {
        if (float.IsNaN(spinningSpeed))
        {
            Debug.LogError("invalide spinningSpeed: NaN");
        }
        float lengthwiseSlip = Mathf.Abs(spinningSpeed - speedOverGround);

        float sideSlip = Mathf.Abs(sideSlideSpeed);

        //division by zero is prevented the following way: If the lengthwise slip is 0 and the sideways is not, the angle is 90°. If both slips are zero, they get equal shares being 45°
        float slipAngle = lengthwiseSlip==0 ? (sideSlip == 0 ? Mathf.PI / 4 : Mathf.PI / 4 ) : Mathf.Atan(sideSlip / lengthwiseSlip);

        float lengthwiseScaler = Mathf.Cos(slipAngle);
        float sidewardScaler = Mathf.Sin(slipAngle);

        lengthwiseScaler = Mathf.Lerp(1, lengthwiseScaler, lengthwiseGripAffectedBySidewaysSlip);
        sidewardScaler = Mathf.Lerp(1, sidewardScaler, sidewaysGripAffectedByLengthwiseSlip);

        if (lengthwiseScaler < 0.05f) lengthwiseScaler = 0.05f;
        if (sidewardScaler < 0.05f) sidewardScaler = 0.05f;


        //(float slipRatio, bool usesFowardForce) slipRatio = GetSlipRatio(speedOverGround, sideSlip, spinningSpeed, forceDirection);
        //float slipRatio = ratAndDir.slipRatio;
        //bool usesFowardForce = ratAndDir.usesFowardForce;
        float slipRatio = GetSlipRatio(speedOverGround, sideSlip, spinningSpeed, forceDirection).slilpRatio;
        float rawGrip = GetRawGripBySlipRatioAbs(slipRatio, collidedGroundType);
        float availableLengthwiseGripPower = rawGrip * lengthwiseScaler * normalForcesUsedForGrip * GetUsedGripMultiplier(collidedGroundType, isFrontWheel, true) * Time.fixedDeltaTime;
        float availableSidewaysGripPower = rawGrip * sidewardScaler * normalForcesUsedForGrip * GetUsedGripMultiplier(collidedGroundType, isFrontWheel, false) * Time.fixedDeltaTime;
        
        if(float.IsNaN(availableSidewaysGripPower))
        {
            Debug.LogError("invalide Sideway Grip: NaN");
        }
        if (float.IsNaN(availableLengthwiseGripPower))
        {
            Debug.LogError("invalide lengthwiseGripPower: NaN");
        }

        return (availableLengthwiseGripPower, availableSidewaysGripPower);
    }



    void PerformInAirBalance()
    {
        bool doInstantCorrection = justLeftGround && fixFlightRotationAtLeavingGround;
        if ((timeInAir < minFlightDurationForCorrection || maxAngularVelCorrectionInAir==0) && !doInstantCorrection) return;

        Vector3 startingCenter = transform.position;

        float timeStepDuration = 0.05f; //predicted time per ray in seconds
        int numberOfTimesteps = doInstantCorrection ? 300 : 30; // 300 * 0.05s = 15s predicted duration at leaving ground and 25*0.05s = 1.5s predicted duration in Air
        Vector3[] v0 = new Vector3[numberOfTimesteps + 1];
        Vector3[,] dotCurves = new Vector3[3, numberOfTimesteps + 1]; //It will contain three "curves", each represented by a row of points.
                                                                      // The points will be connected by raycasts to form the curve and check where the curve collides with terrain.
        Vector3[] hitPos = new Vector3[3];
        float[] hitT = new float[3]; //a curve represents the car's predicted movement. "hitT[curveIndex]" saves the estimated time from each curve to hit terrain.
        Quaternion[] moveDirRotAtIndex = new Quaternion[numberOfTimesteps + 1];

        for (int i = 0; i <= numberOfTimesteps; i++)
        {
            moveDirRotAtIndex[i] = Quaternion.LookRotation(rb.velocity + i * timeStepDuration * Physics.gravity, Vector3.up);
            v0[i] = startingCenter + rb.velocity * (float)i * timeStepDuration + 0.5f * Physics.gravity * (float)i * (float)i * timeStepDuration * timeStepDuration;
            if (i > 0) Debug.DrawLine(v0[i - 1], v0[i], Color.black, Time.fixedDeltaTime);
        }

        //Vector3[] DirectionOfHittingRays = new Vector3[]{new Vector3(), new Vector3(), new Vector3()}; //NOTE: this is the direction of the casted ray, not the hitten surface's
        RaycastHit hit;
        for (int curveIndex = 0; curveIndex < 3; curveIndex++)
        {
            //each line has a baseOffset to seperate it's position from the other lines. For each point in a line this Offset is Rotatet into the estimated move direction at that point
            Vector3 lineBaseOffset = curveIndex == 0 ? new Vector3(0, 1.5f, 0)
                                   : curveIndex == 1 ? new Vector3(-0.866f, -0.5f, 0)
                                                    : new Vector3(0.866f, -0.5f, 0);
            dotCurves[curveIndex, 0] = v0[0] + moveDirRotAtIndex[0] * lineBaseOffset;
            for (int i = 0; i < numberOfTimesteps; i++)
            {
                dotCurves[curveIndex, i + 1] = v0[i + 1] + moveDirRotAtIndex[i + 1] * lineBaseOffset;
                float rayLength = Vector3.Distance(dotCurves[curveIndex, i], dotCurves[curveIndex, i + 1]);
                Debug.DrawLine(dotCurves[curveIndex, i], dotCurves[curveIndex, i + 1], Color.magenta, Time.fixedDeltaTime);
                if (Physics.Raycast(dotCurves[curveIndex, i], dotCurves[curveIndex, i + 1] - dotCurves[curveIndex, i], out hit, rayLength, combinedGroundLayers))
                {
                    hitPos[curveIndex] = hit.point;
                    hitT[curveIndex] = i * timeStepDuration + (hit.point - dotCurves[curveIndex, i]).magnitude / rayLength * timeStepDuration;
                    //DirectionOfHittingRays[curveIndex] = dotCurves[curveIndex, i + 1] - dotCurves[curveIndex, i];
                    break;
                }
            }
        }

        float minDurationTillHit = 0.05f;
        if (hitT[0] > minDurationTillHit && hitT[1] > minDurationTillHit && hitT[2] > minDurationTillHit)
        {
            float unreducedTimeToContact = Mathf.Min(hitT[0], hitT[1], hitT[2]);
            float speedAtContact = (rb.velocity + unreducedTimeToContact * Physics.gravity).magnitude;
            float earlyerArrival = 0.3f / speedAtContact; //the wheels arrive earlyer at the ground than the car, so time needs to be subtracted.
            float timeToContact = unreducedTimeToContact - earlyerArrival;
            if (timeToContact > minDurationTillHit)
            {
                Vector3 contactNormal = Vector3.Cross(hitPos[2] - hitPos[0], hitPos[1] - hitPos[0]);
                float estimadedYRotOnArival = rb.rotation.eulerAngles.y + rb.angularVelocity.y * Mathf.Rad2Deg * timeToContact;

                float AdaptationForWantedXRot = Vector3.Angle(Quaternion.Euler(0, estimadedYRotOnArival, 0) * Vector3.forward, contactNormal) - 90;
                Quaternion aimedRotation = Quaternion.Euler(AdaptationForWantedXRot, 0, 0) * Quaternion.LookRotation(Quaternion.Euler(0, estimadedYRotOnArival, 0) * Vector3.forward, contactNormal);
                //Vector3 wantedRotation = Quaternion.Lerp(rb.rotation, aimedRotation, Time.fixedDeltaTime / timeToContact).eulerAngles;

                //Goal orientated Version
                Vector3 wantedAngularVelChange = Vector3.zero;
                Quaternion expectedRotChange = Quaternion.Euler(
                        rb.angularVelocity.x * Mathf.Rad2Deg * timeToContact,
                        rb.angularVelocity.y * Mathf.Rad2Deg * timeToContact,
                        rb.angularVelocity.z * Mathf.Rad2Deg * timeToContact
                        );
                //A * B * C = D      means    expected * (CHANGE * current) = aimed 
                //B = A^-1 * D * C^-1
                Quaternion delta = Quaternion.Inverse(expectedRotChange) * aimedRotation * Quaternion.Inverse(rb.rotation);


                //the angle is calculated by an adaptation of the solution from DMGregory at https://gamedev.stackexchange.com/questions/147409/rotate-from-current-rotation-to-another-using-angular-velocity (15.04.2024)
                float angle; Vector3 axis;
                delta.ToAngleAxis(out angle, out axis);
                // We get an infinite axis in the event that our rotation is already aligned.
                if (!float.IsInfinity(axis.x))
                {
                    if (angle > 180f)
                        angle -= 360f;

                    //Goal orientated version
                    wantedAngularVelChange = (0.98f * angle * Mathf.Deg2Rad / timeToContact) * axis.normalized;
                    wantedAngularVelChange = new Vector3(wantedAngularVelChange.x, 0, wantedAngularVelChange.z);
                }
                else
                {
                    Debug.Log("axis is infinity (should propably not)");
                }

                if (doInstantCorrection && timeToContact > 0.7f)
                {
                    rb.angularVelocity += wantedAngularVelChange; // instant rotation speed change when leaving ground
                }
                else if (timeInAir >= minFlightDurationForCorrection)
                {
                    float angularChangeLimitScale = wantedAngularVelChange.magnitude > maxAngularVelCorrectionInAir * Time.fixedDeltaTime ? maxAngularVelCorrectionInAir * Time.fixedDeltaTime / wantedAngularVelChange.magnitude : 1;
                    rb.angularVelocity += wantedAngularVelChange * angularChangeLimitScale; // continiouse rotation speed change in the air, limited by maxAngularVelCorrectionInAir.
                }
            }
        }
    }

    private void ComputeGroundAndAirStatus()
    {
        if (numberOfGroundedWheels > 0)
        {
            lastContactWithGround = 0;
            timeInAir = 0;
            triggeredGroundLeftAlready = false;
            justLeftGround = false;
        }
        else
        {
            lastContactWithGround += Time.fixedDeltaTime;
        }
        if (lastContactWithGround > Time.fixedDeltaTime * 4)
        {
            timeInAir += Time.fixedDeltaTime;
            if (!triggeredGroundLeftAlready)
            {
                justLeftGround = true;
                triggeredGroundLeftAlready = true;
            }
            else
            {
                justLeftGround = false;
            }
        }
    }

    private void OnCollisionStay(Collision collision)
    {
        if (((1 << collision.gameObject.layer) & combinedGroundLayers) != 0)
        {
            lastContactWithGround = 0;
            timeInAir = 0;
            triggeredGroundLeftAlready = false;
            justLeftGround = false;
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

enum ShouldGet
{
    All,
    Nothing,
    Value
}

enum ActionMode
{
    AccelerateByMotor,
    DeccelerateByMotor,
    Break,
    Handbreak
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


