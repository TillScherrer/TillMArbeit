using System.Collections.Generic;
using TMPro;
using UnityEngine;
using UnityEngine.UI;



public class Car : MonoBehaviour
{

    //accessed from CarEditor to apply custom manipulation
    public SpeedupCurve SpeedupCurve { get => speedupCurve; set => speedupCurve = value; }
    public SolidGrounGrip FrontGripOnSolidGround { get => gripOnSolidGround; set => gripOnSolidGround = value; }
    public LooseGrounGrip FrontGripOnLooseGround { get => gripOnLooseGround; set => gripOnLooseGround = value; }


    //foldouts for CarEditor
    [SerializeField] bool showAccelerationSettings;
    [SerializeField] bool showGearSettings;
    [SerializeField] bool showBreakSettings;
    [SerializeField] bool showWheelSettings;
    [SerializeField] bool showBalanceSettings;
    [SerializeField] bool showGripSetting;
    [SerializeField] bool showSteeringSettings;
    [SerializeField] bool showInputSettings;
    [SerializeField] bool showAudioSettings;
    [SerializeField] bool showOverlaySettings;
    [SerializeField] bool showVisualisationSettings;


    //ACCELERATION
    [SerializeField] SpeedupCurve speedupCurve;
    [SerializeField] float rearOrFrontPowered = 1;
    [SerializeField] float accelerationRestrainToOptimiseGrip = 0.5f; //Todo: inspector

    //GEARS
    [SerializeField] int numberOfGears = 1;
    [SerializeField] float gearImpactOnAcceleration = 0;
    [SerializeField] float gearOutDuration = 0.1f;
    [SerializeField] float shiftDuration = 0.1f;
    [SerializeField] float gearInDuration = 0.1f;
    [SerializeField] float shiftingImpactOnAcceleration = 0;
    [SerializeField] GearShiftMode gearShiftMode = GearShiftMode.Automatic;


    //BRAKING
    //breaking Settings
    [SerializeField] float breakMaxSlowdown = 10f;
    [SerializeField] float breakAppliedToFrontWheels = 1f;
    [SerializeField] float breakRestrainToOptimiseGrip = 1f;
    [SerializeField] float breakLimitedByGrip = 1f;
    [SerializeField] float handBreakMaxSlowdown = 20f;
    [SerializeField] float handBreakAppliedToFrontWheels = 1f;
    [SerializeField] float handBreakRestrainToOptimiseGrip = 0f;
    [SerializeField] float handBreakLimitedByGrip = 1f;
    //decceleration Settings
    [SerializeField] float slowByAirResistanceAt30ms = 0.5f;
    [SerializeField] float airResistanceExponent = 2f;
    [SerializeField] float slowByRollFriction = 0.05f;

    //SUSPENSION
    //front wheel physical position and spring settings
    [SerializeField] Vector3 frontRightWheelCenter = new Vector3(1, -0.3f, 1);
    [SerializeField] float frontWheelRadius = 0.25f;
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

    //BALANCE
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

    //GRIP
    //layers for ground types
    [SerializeField] LayerMask solidGround;
    [SerializeField] LayerMask looseGround;
    //grip's scaling with nomral force
    [SerializeField] float scaleGripWithSpringCompression = 1;
    [SerializeField] float scaleGripWithDampingCompression = 0;
    [SerializeField] float spreadGripFromNormalForceOnAllWheels = 0;
    //
    [SerializeField] bool sameGripSettingsForRearWheel;
    //endless grip option
    [SerializeField] private bool endlessLengthwiseGrip = false;
    [SerializeField] private bool endlessSidewaysGrip = false;
    //dependence of front- and side-grip
    [SerializeField] float lengthwiseGripAffectedBySidewaysSlip = 1;
    [SerializeField] float sidewaysGripAffectedByLengthwiseSlip = 1;
    //grip curves for both ground types
    [SerializeField] SolidGrounGrip gripOnSolidGround;
    [SerializeField] LooseGrounGrip gripOnLooseGround;
    //grip at wheel at ground type for direction
    [SerializeField] float FwSgLengthwiseGrip;
    [SerializeField] float RwSgLengthwiseGrip;
    [SerializeField] float FwSgSidewaysGrip;
    [SerializeField] float RwSgSidewaysGrip;
    [SerializeField] float FwLgLengthwiseGrip;
    [SerializeField] float RwLgLengthwiseGrip;
    [SerializeField] float FwLgSidewaysGrip;
    [SerializeField] float RwLgSidewaysGrip;
    [SerializeField] bool useSameGripMultipliersAsSolidGround;

    //STEERING
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


    //INPUT
    //[SerializeField] bool breakAtOtherDirectionInput = true;
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

    //AUDIO
    [SerializeField] AudioSource audioForEngine = null;
    [SerializeField] float deltaPitchAtLowGearSpeed = 0.1f;
    [SerializeField] float deltaPitchAtHighGearSpeed = 0.1f;
    [SerializeField] float amplifyDeltaPitchPerGear = -0.3f;
    [SerializeField] float volumeDropOffAtGearShift = 1.0f;
    [SerializeField] float volumeScaleWithThrottleInput = 1.0f;
    [SerializeField] float volumeDropOffAtLowGearEffectiveness = 1.0f;
    [SerializeField] AudioSource audioForSlip = null;

    //OVERLAY
    [SerializeField] GameObject sliderToShowShift;
    UnityEngine.UI.Slider guiSliderForShifting;
    [SerializeField] GameObject sliderToShowClutch;
    UnityEngine.UI.Slider guiSliderForClutch;
    [SerializeField] GameObject guiGear;
    TextMeshProUGUI guiGearTextMeshPro;
    [SerializeField] GameObject guiSpeed;
    TextMeshProUGUI guiSpeedTextMeshPro;

    //VISUALS
    public Transform[] visualWheels = new Transform[4];
    [SerializeField] bool[] placeOnPhysicalWheel = new bool[] { false, false, false, false };
    [SerializeField] Transform visualDebugCar;
    [SerializeField] GameObject particlePrefab;


    //SETUP PARAMETERS
    public List<float> gearIsBestAtRelSpeed = new List<float>();
    float gearScaleAtMinimum = 1;
    float gearScaleDiff = 0;
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
    float[] springAccelerationPerM = new float[4];
    float usedGravityForSprings = 0;
    float extraEffortForWheelSpin;
    LayerMask combinedGroundLayers;
    Vector3[] visualWheelLocalPositions = new Vector3[4];
    Quaternion[] visualWheelLocalRotations = new Quaternion[4];
    float selectedBasePitchForEngine;
    float selectedBaseVolumeForEngine;
    //References (connected on setup)
    Rigidbody rb;
    CustomGravityReciver gravityReciver;
    GameObject[] particleGameObjects = new GameObject[4];
    ParticleSystem[] particleSystems = new ParticleSystem[4];

    //ACTIVE CHANGING PARAMETERS EACH FIXED UPDATE
    //spring and ground contact
    Vector3[] hitPoints = new Vector3[4];
    Vector3[] hitNormals = new Vector3[4];
    Vector3[] hitPointForLateral;
    Vector3[] hitPointForLongitudal;
    float[] absoluteSpringCompressions = new float[4];
    float[] previousAbsoluteSpringCompressions = new float[4];
    float[] springCompressions;
    int[] collidedGroundType;
    float[] normalForcesUsedForGrip;
    int numberOfGroundedWheels = 0;
    float timeInAir = 0;
    bool triggeredGroundLeftAlready = false;
    bool justLeftGround = false;
    float lastContactWithGround = 0;
    Vector3 previouslyAcceptedGroundNormal = Vector3.up;
    Vector3 currentGroundNormal = Vector3.up;
    //movement
    float currentFowardSpeed = 0;
    float[] fowardSpeedOverGroundAtWheel = new float[4];
    float[] sideSlideSpeeds = new float[4];
    Vector3[] fowardOnGroundAtWheel = new Vector3[4];
    Vector3[] sideSlideDirections = new Vector3[4];
    //acceleration & breaking
    float[] currentWheelSpinningSpeeds = new float[] {0,0,0,0};
    float[] lengthwiseForceAtWheels = new float[] { 0, 0, 0, 0 };
    float[] sidewayGripAtWheels = new float[] { 0, 0, 0, 0 };
    float currentAverageWheelSpinningSpeed = 0;
    //gears
    float[] longitudalRatioAndDirectionalScalingGripAtPreviouseFrame = new float[] {0.1f,0.1f,0.1f,0.1f}; //TO DO: SAVE FROM PREVIOUSE FRAME
    int currentGear = 0;
    int aimedGear = 0;
    int prevGear = 0;
    float timeInCurrentGearState = 0;
    GearShiftState gearShiftState = GearShiftState.Geared;
    float accelerationScalerByClutch = 1;
    //steering
    Vector3 inertiaTensorWS = Vector3.one;
    float frontSteeringAngle = 0;
    float rearSteeringAngle = 0;
    float turningRadius = Mathf.Infinity;
    float[] rawSteeringAngles = new float[] { 0, 0, 0, 0 };
    float[] prevOffAngles = new float[] { 0, 0, 0, 0 };
    float[] steeringAngles = new float[] { 0, 0, 0, 0 };
    //visuals
    float[] visualWheelCurrentXRot = new float[4];
    float[] visualWheelRPS = new float[4];
    


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
        if (sliderToShowShift != null) guiSliderForShifting = sliderToShowShift.GetComponent<Slider>();
        if (sliderToShowClutch != null) guiSliderForClutch = sliderToShowClutch.GetComponent<Slider>();
        if (guiGear != null) guiGearTextMeshPro = guiGear.GetComponent<TextMeshProUGUI>();
        if (guiSpeed != null) guiSpeedTextMeshPro = guiSpeed.GetComponent<TextMeshProUGUI>();

        if(particlePrefab != null)
        {
            for(int i = 0; i < 4; i++)
            {
                particleGameObjects[i] = Instantiate(particlePrefab);
                particleSystems[i] = particleGameObjects[i].GetComponent<ParticleSystem>();
            }
        }
    }

    void Update() {

        //Rotate and turn visual Wheels (zero impact on physical calculations)
        for (int i = 0; i < 4; i++)
        {
            visualWheelRPS[i] = currentWheelSpinningSpeeds[i] / ((i < 2 ? frontWheelRadius : rearWheelRadius) * 2 * Mathf.PI);
            visualWheelCurrentXRot[i] += visualWheelRPS[i] * 360 * Time.deltaTime;
            if (visualWheelCurrentXRot[i] > 360) visualWheelCurrentXRot[i] -= 360;
            if (visualWheelCurrentXRot[i] < 0) visualWheelCurrentXRot[i] += 360;
            visualWheels[i].position = transform.TransformPoint(visualWheelLocalPositions[i]) + transform.up * (absoluteSpringCompressions[i] - (i < 2 ? frontWheelSuspensionDistanceToLiftCarWeight : rearWheelSuspensionDistanceToLiftCarWeight));
            visualWheels[i].rotation = transform.rotation * Quaternion.Euler(visualWheelCurrentXRot[i], steeringAngles[i], 0) * visualWheelLocalRotations[i];
        }

        //Get Input
        currentAverageWheelSpinningSpeed = GetAverageWheelSpinningSpeed();
        if(gearShiftMode == GearShiftMode.Automatic || numberOfGears == 0)
        {
            ThrottleKey.IsPressed = (currentAverageWheelSpinningSpeed >= -0.01f) && Input.GetKey(ThrottleKey.KeyboardInput);
            BackwardThrottleKey.IsPressed = (currentAverageWheelSpinningSpeed <= 0.01f) && Input.GetKey(BackwardThrottleKey.KeyboardInput);
            
            BreakKey.IsPressed = Input.GetKey(BreakKey.KeyboardInput) || (currentAverageWheelSpinningSpeed > 0.01 && Input.GetKey(BackwardThrottleKey.KeyboardInput)) || (currentAverageWheelSpinningSpeed < -0.01 && Input.GetKey(ThrottleKey.KeyboardInput));
            if (BackwardThrottleKey.IsPressed && BackwardThrottleKey.KeyboardInput == BreakKey.KeyboardInput) BreakKey.IsPressed = false;
        }
        else
        {
            ThrottleKey.IsPressed = Input.GetKey(ThrottleKey.KeyboardInput) && ((currentAverageWheelSpinningSpeed >= -0.01f && currentGear > -1) || (currentAverageWheelSpinningSpeed <= 0.01f && currentGear < 0)); //must have right gear for moving direction-
            BackwardThrottleKey.IsPressed = false;
            BreakKey.IsPressed = Input.GetKey(BreakKey.KeyboardInput) || (currentAverageWheelSpinningSpeed >= 0.01f && currentGear < 0) || (currentAverageWheelSpinningSpeed <= -0.01f && currentGear > -1); ; // -because the wrong gear also counts as breaking
        }
        HandbreakKey.IsPressed = Input.GetKey(HandbreakKey.KeyboardInput);

        //make input dynamic (Keyboard gives binary input, which is interpolated over time)
        ThrottleKey.Update();
        BackwardThrottleKey.Update();
        BreakKey.Update();
        HandbreakKey.Update();
        //gear shift input (register manual input or emulate input with automatic gear shift)
        ManageGearShiftInput(currentAverageWheelSpinningSpeed);      
    }


    private void FixedUpdate()
    {
        //update World Space inertia tensor
        inertiaTensorWS = GetInertiaTensorInWorldSpace();

        //GROUND INTERACTION
        FindGroundInteraction(out hitPoints, out hitNormals, out absoluteSpringCompressions, out collidedGroundType);
        ComputeGroundAndAirStatus();
        hitPointForLateral = new Vector3[] { GetLiftedPoint(hitPoints[0], lateralAttackHeightLift), GetLiftedPoint(hitPoints[1], lateralAttackHeightLift) , GetLiftedPoint(hitPoints[2], lateralAttackHeightLift) , GetLiftedPoint(hitPoints[3], lateralAttackHeightLift) };
        hitPointForLongitudal = new Vector3[] { GetLiftedPoint(hitPoints[0], longitudalAttackHeightLift), GetLiftedPoint(hitPoints[1], longitudalAttackHeightLift), GetLiftedPoint(hitPoints[2], longitudalAttackHeightLift), GetLiftedPoint(hitPoints[3], longitudalAttackHeightLift) };
        //SUSPENSION AND GETTING NORMAL FORCE WHICH IS USED FOR GRIP
        ApplyAntiRollBar(absoluteSpringCompressions, out springCompressions);
        ApplySpringForcesAndSetNormalForceUsedForGrip();


        //STEERING
        currentFowardSpeed = Vector3.Dot(rb.velocity, transform.forward);
        float frontSteerDifFor30ms = frontSteerAtZeroSpeed - frontSteerAt30MS; //front
        float currentMaxFrontSteerAngle = frontSteerAt30MS + frontSteerDifFor30ms * (-1 + 2 / Mathf.Pow(2, Mathf.Abs(currentFowardSpeed) / 30));
        UpdateRawSteerAngle(ref frontSteeringAngle, currentMaxFrontSteerAngle);
        float rearSteerDifFor30ms = rearSteerAtZeroSpeed - rearSteerAt30MS;    //rear
        float currentMaxRearSteerAngle = rearSteerAt30MS + rearSteerDifFor30ms * (-1 + 2 / Mathf.Pow(2, Mathf.Abs(currentFowardSpeed) / 30));
        UpdateRawSteerAngle(ref rearSteeringAngle, currentMaxRearSteerAngle);
        UpdateSteeringAngles(hitPoints);
        
        //UpdateHorizontalMovementParameters (this just updates some often used variables)
        for (int i = 0; i < 4; i++)
        {
            //forward
            Vector3 sideDirectionR = rb.rotation * (Quaternion.Euler(0, steeringAngles[i], 0) * Vector3.right);
            if (hitNormals[i] == Vector3.zero)// = if this wheel is not in contact with the ground, take local foward vector of wheel instead
            {
                fowardOnGroundAtWheel[i] = Vector3.Cross(sideDirectionR, transform.up).normalized;
            }
            else
            {
                fowardOnGroundAtWheel[i] = Vector3.Cross(sideDirectionR, hitNormals[i]).normalized;
            }
            fowardSpeedOverGroundAtWheel[i] = Vector3.Dot(GetActualPointVelocity(hitPointForLongitudal[i]), fowardOnGroundAtWheel[i]);

            //sideward
            Vector3 directionFoward = rb.rotation * (Quaternion.Euler(0, steeringAngles[i], 0) * Vector3.forward);
            Vector3 rOnGround = Vector3.Cross(hitNormals[i], directionFoward);
            sideSlideDirections[i] = (rOnGround * ((Vector3.Angle(GetActualPointVelocity(hitPointForLateral[i]), rOnGround) > 90) ? -1 : 1)).normalized;
            sideSlideSpeeds[i] = Vector3.Dot(GetActualPointVelocity(hitPointForLateral[i]), sideSlideDirections[i]);
        }

        //GEAR SHIFTING  
        ComputeGearShiftState(); //(input has already been managed in "Update()". Therefor the registered input is computed here)

        //ACCELERATION AND BREAKING
        rb.velocity -= GetAirResistanceSlowdownForSpeedAbs(rb.velocity.magnitude) * Time.fixedDeltaTime * rb.velocity.normalized;
        currentAverageWheelSpinningSpeed = GetAverageWheelSpinningSpeed();
        
        bool wheelsSpinBackwards = currentAverageWheelSpinningSpeed < 0;
        int wheelSpinningDirectionalMultiplier = wheelsSpinBackwards ? -1 : 1;
        ComputeLengthwiseInputToGetFrontAndRearPower(out ActionMode actionMode, out int forceDirection, out float frontWheelsPowerShare, out float totalAvailablePower, out float usedSlipPreventer, wheelSpinningDirectionalMultiplier);

        SplitAvailableImpulseOnLeftAndRightWheels(out float[] availableImpulseForFrame, out bool[] canLeadToDirectionChange, actionMode, frontWheelsPowerShare, forceDirection, totalAvailablePower, wheelSpinningDirectionalMultiplier);

        ComputeImpulsesWithWheelSpinAndResultingFriction(availableImpulseForFrame, actionMode, forceDirection, usedSlipPreventer, canLeadToDirectionChange);

        
        float[] maxLengthwiseImpulse = new float[4];
        Vector3[] directionToApplyLengthwiseForce = new Vector3[4];
        bool[] isBreaking = new bool[4];
        float[] currentMovementAgainstForceDir= new float[4];
        for (int i = 0; i < 4; i++)
        {
            directionToApplyLengthwiseForce[i] = (fowardOnGroundAtWheel[i] * lengthwiseForceAtWheels[i]).normalized;
            maxLengthwiseImpulse[i] = (fowardOnGroundAtWheel[i] * lengthwiseForceAtWheels[i]).magnitude;
            isBreaking[i] = !canLeadToDirectionChange[i];

            currentMovementAgainstForceDir[i] = Vector3.Dot(GetActualPointVelocity(hitPointForLongitudal[i]), -directionToApplyLengthwiseForce[i].normalized);
        }
        //apply lengthwise friciton
        BalanceAndApplyImpulses(hitPointForLongitudal, directionToApplyLengthwiseForce, maxLengthwiseImpulse, currentMovementAgainstForceDir, isBreaking);


        //SIDEWARD FRICTION
        float[] maxStopImpulse = new float[4];
        Vector3[] directionToApplyBreakingForce = new Vector3[4];
        for (int i = 0; i < 4; i++)
        {
            directionToApplyBreakingForce[i] = -sideSlideDirections[i];
            if (endlessSidewaysGrip)
            {
                maxStopImpulse[i] = sideSlideSpeeds[i] * rb.mass / 4;

            }
            else
            {
                maxStopImpulse[i] = sidewayGripAtWheels[i];
                
            }

            //reduce sidewardForce when the side of the wheel is on the ground
            float wheelSidewardAngle = Mathf.Abs(Vector3.SignedAngle(hitNormals[i], transform.up, transform.forward));
            if(wheelSidewardAngle > 75)
            {
                maxStopImpulse[i] = 0;
            }
            else if (wheelSidewardAngle > 50)
            {
                maxStopImpulse[i] *= (75 - wheelSidewardAngle) / 25f;
            }
            //Debug.DrawRay(hitPoints[i], sideSlideDirections[i].normalized * -maxStopImpulse[i] / Time.fixedDeltaTime * 0.2f, Color.green);
        }

        bool[] sidewardForceIsABreakingForce = new bool[] {true,true,true,true};
        BalanceAndApplyImpulses(hitPointForLateral, directionToApplyBreakingForce, maxStopImpulse, sideSlideSpeeds, sidewardForceIsABreakingForce);


        ApplyArcadySteeringOrAirSteering();


        ApplyTurnUpwards();
        //Compute Ground-Air-Status
        PerformInAirBalance();

        //Manage Audio
        ManageAudioOverlayAndVisualParticles(currentGear, currentAverageWheelSpinningSpeed, currentFowardSpeed);

        previousAbsoluteSpringCompressions = absoluteSpringCompressions;
    }




    float GetAverageWheelSpinningSpeed()
    {
        float average = 0;
        for (int i = 0; i < 4; i++)
        {
            average += currentWheelSpinningSpeeds[i] / 4; //* (i < 2 ? rearOrFrontPowered: 1-rearOrFrontPowered) / 2;
        }
        return average;
    }

    void ManageGearShiftInput(float currentSpeed)
    {
        float currentRelSpeed = speedupCurve.GetTimeInCurve(currentSpeed);

        if (gearShiftMode == GearShiftMode.Automatic || numberOfGears == 0)
        {
            int bestGear = GetBestGearForCurrentRelSpeed(currentRelSpeed); //TO DO: weitere Regeln für automatik Schaltung nötig

            // the shifting-up process does take some speed, but the car should not shift down immediately after shifting up
            bool isQualifiedToShiftDown = false;
            if (gearImpactOnAcceleration == 0)
            {
                if (timeInCurrentGearState > gearInDuration + shiftDuration + gearOutDuration + 1 || bestGear < currentGear - 1)
                {
                    isQualifiedToShiftDown = true;
                }
            }
            else
            {
                //Debug.Log("Gear Uneffectiveness " + (GetGearUneffectivenessRating(currentGear, currentSpeed)));
                //Debug.Log("gearScaleDif=  " + gearScaleDiff);
                if (GetGearUneffectivenessRating(currentGear, currentSpeed) > 1.3f)
                {
                    isQualifiedToShiftDown = true;
                }
            }


            if (bestGear == 0) // in this case input and speed decides if the backward-gear or gear0 should be used
            {
                if (currentGear > 0)
                {
                    if (isQualifiedToShiftDown)
                    {
                        aimedGear = 0;
                    }
                }
                else if (currentSpeed > -0.1f && ThrottleKey.IsPressed)
                {
                    aimedGear = 0;
                }
                else if (currentSpeed < 0.1f && BackwardThrottleKey.IsPressed)
                {
                    aimedGear = -1;
                }
            }
            else //all other gears are far from zero, so it is earsier to decide
            {
                if (currentSpeed > 0)
                {
                    if(bestGear < aimedGear)
                    {
                        if (isQualifiedToShiftDown)
                        {
                            aimedGear = bestGear;
                        }
                    }
                    else
                    {
                        aimedGear = bestGear;
                    }
                }
                else
                {
                    aimedGear = -1;
                }
            }

        }
        else if (gearShiftMode == GearShiftMode.ManualClickUpDown)
        {
            if (Input.GetKeyDown(KeyCode.LeftShift) && aimedGear < numberOfGears - 1) aimedGear++; //shift gear up
            if (Input.GetKeyDown(KeyCode.CapsLock) && aimedGear > -1) aimedGear--; //shift gear down
        }
        else if (gearShiftMode == GearShiftMode.ManualOneClick)
        {
            if (Input.GetKeyDown(KeyCode.LeftShift))
            {
                int bestGear = GetBestGearForCurrentRelSpeed(currentRelSpeed);
                if (bestGear == aimedGear) //force a shift since the user did the input, even if it is not necesarry for best speed
                {
                    bool planGearShiftUp = gearIsBestAtRelSpeed[aimedGear] < currentRelSpeed;
                    if (aimedGear < 0) aimedGear++;
                    else
                    {
                        if (planGearShiftUp && aimedGear < numberOfGears-1) aimedGear++; //shift gear up
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

    Vector3 GetInertiaTensorInWorldSpace()
    {
        //Method based on "Nathan Reeds" answer on https://gamedev.stackexchange.com/questions/70355/inertia-tensor-and-world-coordinate-conversion (05.12.23)

        Matrix4x4 worldRotationMatrix = Matrix4x4.Rotate(rb.rotation);

        Matrix4x4 localInertiaTensor = Matrix4x4.zero;
        localInertiaTensor.m00 = rb.inertiaTensor.x;
        localInertiaTensor.m11 = rb.inertiaTensor.y;
        localInertiaTensor.m22 = rb.inertiaTensor.z;

        // transform localInertiaTensor into world space
        Matrix4x4 worldInertiaTensorMatrix = worldRotationMatrix * localInertiaTensor * worldRotationMatrix.inverse;

        // extract scaled main axis
        Vector3 worldInertiaTensor = new Vector3(worldInertiaTensorMatrix.m00, worldInertiaTensorMatrix.m11, worldInertiaTensorMatrix.m22);

        return worldInertiaTensor;
    }
    private void FindGroundInteraction(out Vector3[] hitPoints, out Vector3[] hitNormals, out float[] absoluteSpringCompressions, out int[] collidedGroundType)
    {
        hitPoints = new Vector3[4];
        hitNormals = new Vector3[4];
        absoluteSpringCompressions = new float[] { 0, 0, 0, 0 };
        collidedGroundType = new int[] { 0, 0, 0, 0 }; //0 = no; 1 = solidGround; 2 = looseGround
        int numberOfContacts = 0;
        Vector3 combinedNormalVector = Vector3.zero;

        for (int i = 0; i < 4; i++)
        {
            bool use3DCollisions = i < 2 ? frontUse3DWheelPhysics : rearUse3DWheelPhysics;

            if (!use3DCollisions)
            {
                //collision detection by ray
                Vector3 startPoint = transform.TransformPoint(wheelCentersStartPoint[i]);
                Vector3 endPoint = transform.TransformPoint(wheelCentersEndPoint[i]);
                Vector3 startToEnd = endPoint - startPoint;
                Ray ray = new Ray(startPoint, startToEnd);
                Debug.DrawRay(startPoint, startToEnd, UnityEngine.Color.red);
                if (Physics.Raycast(ray, out RaycastHit hit, startToEnd.magnitude, combinedGroundLayers))
                {
                    hitPoints[i] = hit.point;
                    hitNormals[i] = hit.normal;
                    collidedGroundType[i] = (solidGround == (solidGround | (1 << hit.transform.gameObject.layer))) ? 1 : 2;
                    absoluteSpringCompressions[i] = (hit.point - endPoint).magnitude;
                    numberOfContacts++;
                }
            }
            else
            {
                //collision detection by combined 3D-Shape (Many cubes approximate one cylinder)
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

                    if (Physics.BoxCast(boxCenter, new Vector3(wheel3DThicknesses[i], wheelRadii[i], zScale) * 0.5f, startToEnd, out hit, rb.rotation * Quaternion.Euler(xRot, 0, -10 * side), startToEnd.magnitude, combinedGroundLayers))
                    {
                        Vector3 hitLocalSpace = transform.InverseTransformPoint(hit.point);
                        float zOffsetFromCenter = hitLocalSpace.z - wheelCenters[i].z;
                        float lostCompressionHeightByZOffset = wheelRadii[i] - Mathf.Sqrt(wheelRadii[i] * wheelRadii[i] - zOffsetFromCenter * zOffsetFromCenter);
                        float theoreticalSpringCompression = hitLocalSpace.y + wheelRadii[i] - localEndPoint.y - lostCompressionHeightByZOffset;
                        if (theoreticalSpringCompression > absoluteSpringCompressions[i])
                        {
                            absoluteSpringCompressions[i] = theoreticalSpringCompression;
                            hitPoints[i] = hit.point;
                            hitNormals[i] = hit.normal;
                            collidedGroundType[i] = (solidGround == (solidGround | (1 << hit.transform.gameObject.layer))) ? 1 : 2;
                            if (hit.rigidbody == rb) { Debug.LogError("A Wheel collided downward with its corresponding car. You should NOT pick a Layer as solidGround or looseGround which is part of your Car's colliders"); }
                        }
                    }
                }
                if (absoluteSpringCompressions[i] != 0) numberOfContacts++;
            }
            Debug.DrawRay(hitPoints[i], hitNormals[i] * absoluteSpringCompressions[i], UnityEngine.Color.green);
            combinedNormalVector += hitNormals[i];
        }
        numberOfGroundedWheels = numberOfContacts;
        if (numberOfContacts == 0)
        {
            currentGroundNormal = previouslyAcceptedGroundNormal.normalized;
        }
        else
        {
            currentGroundNormal = (combinedNormalVector / numberOfContacts).normalized;
            previouslyAcceptedGroundNormal = currentGroundNormal;
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
        if (lastContactWithGround > Time.fixedDeltaTime * 2)
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
            springCompressions[0 + i] = absoluteSpringCompressions[0 + i] * (1 + (ratioL * 2 - 1) * usedRollbarValue);
            springCompressions[1 + i] = absoluteSpringCompressions[1 + i] * (1 + (ratioR * 2 - 1) * usedRollbarValue);
        }
    }

    void ApplySpringForcesAndSetNormalForceUsedForGrip()
    {
        float[] springCompressionForce = GetSpringCompressionForce(springCompressions);
        float[] dampingForces = new float[4];
        Vector3[] verticalForces = new Vector3[4];
        for (int i = 0; i < 4; i++)
        {
            //SUSPENSION
            //damping
            float deltaSpringCompression = absoluteSpringCompressions[i] - previousAbsoluteSpringCompressions[i];
            float speedOfCompression = deltaSpringCompression / Time.fixedDeltaTime;
            float ratioOfAverageCompression = (springCompressions[i] / looseSpringOffsetLength[i]);
            dampingForces[i] = damping * speedOfCompression * ratioOfAverageCompression * rb.mass;

            verticalForces[i] = hitNormals[i] * (springCompressionForce[i] + dampingForces[i]);

            float currentSpeedAwayFromGroundNormal = Vector3.Dot(GetActualPointVelocity(hitPoints[i]), hitNormals[i]);

            //reduce supsension power when already moving upwards, if the anti-jump-option is on
            if (currentSpeedAwayFromGroundNormal > 0)
            {
                //enables full effect at 0.1 m/s upward speed
                float scalerFromUpSpeed = Mathf.Lerp(0, 1, currentSpeedAwayFromGroundNormal * 10);
                float reduction = Mathf.Lerp(0, 1, scalerFromUpSpeed * antiJumpSuspension);
                verticalForces[i] *= (1f - reduction);
            }

            float suspensionAngle = Vector3.Angle(hitNormals[i], transform.up);
            //reduce sping power at very high angles (scaling from 1 to zero between 50 and 90 degree)
            if (suspensionAngle > 50)
            {
                verticalForces[i] *= (90f - suspensionAngle) / 40f;
            }
            

            rb.AddForceAtPosition(verticalForces[i], hitPoints[i]);

            //the downward movement can be fully stopped as a hard bump, when ether springs are compressed over their maximum, or partially when the angle is very high
            float downwardMovementHardStop = 0;
            //stop downward movmenet partially when the angle between suspension and ground is too high
            if (suspensionAngle > 70) downwardMovementHardStop = 1 - ((90f - suspensionAngle) / 20f);
            //stop downward movement completely when the spring is compressed over its limit
            if (absoluteSpringCompressions[i] > looseSpringOffsetLength[i] + upwardSuspensionCap[i]) downwardMovementHardStop = 1;
            if (downwardMovementHardStop != 0)
            {
                //if it is not already in a state of extending the spring again
                if (currentSpeedAwayFromGroundNormal < 0) //hardstop is only needed, when it is not already in a state of extending the spring again
                {
                    (float vTotal, float vFromDir, Vector3 w, Vector3 rotToV, Vector3 treatedDirection) effectOfFullImpulseStop;
                    effectOfFullImpulseStop = CalculateEffectOfImpulseOnDirectionalMovementAtPoint(hitPoints[i], hitNormals[i], -currentSpeedAwayFromGroundNormal * rb.mass);
                    float scalerToReachAimedSpeed = -currentSpeedAwayFromGroundNormal / effectOfFullImpulseStop.vTotal;
                    Vector3 angularChange = effectOfFullImpulseStop.w * scalerToReachAimedSpeed * downwardMovementHardStop;
                    Vector3 directionalChange = effectOfFullImpulseStop.vFromDir * effectOfFullImpulseStop.treatedDirection * scalerToReachAimedSpeed * downwardMovementHardStop;
                    rb.AddRelativeTorque(new Vector3(angularChange.x * inertiaTensorWS.x, angularChange.y * inertiaTensorWS.y, angularChange.z * inertiaTensorWS.z), ForceMode.Impulse);
                    rb.AddForce(directionalChange * rb.mass, ForceMode.Impulse);
                }
            }
        }
        normalForcesUsedForGrip = GetNormalForcesManipulatedForGrip(springCompressions, dampingForces);
    }

    float[] GetSpringCompressionForce(float[] springCompressions)
    {
        float[] normalForceOfSpringCompression = new float[4];
        for (int i = 0; i < 4; i++)
        {
            normalForceOfSpringCompression[i] = springCompressions[i] * springAccelerationPerM[i] * rb.mass;
        }
        return normalForceOfSpringCompression;
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

    float[] GetSpringCompressionsWhichGotScaledByTheirImpactOnGrip(float[] springCompressions)
    {
        float[] scaledSpringCompressions = new float[4];
        for(int i = 0; i < 4; i++)
        {
            float oneCompressionLength = (i < 2 ? frontWheelSuspensionDistanceToLiftCarWeight : rearWheelSuspensionDistanceToLiftCarWeight);
            if (springCompressions[i] == 0) //Not on ground
            {
                scaledSpringCompressions[i] = 0;
            }
            else //on ground
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
        }
        return scaledSpringCompressions;
    }

    private (float vTotal, float vFromDir, Vector3 w, Vector3 rotToV, Vector3 treatedDirection) CalculateEffectOfImpulseOnDirectionalMovementAtPoint(Vector3 hitPoint, Vector3 ImpulseDirToCancleCurrent, float impulsePower) 
    {
        //in local Space, because Unitys inertiaTensor is in local space
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


    public void UpdateDependendParameters() //Note: This Method is also called when you make changes in Inspector during play mode
    {
        rb = GetComponent<Rigidbody>();
        gravityReciver = GetComponent<CustomGravityReciver>();
        usedGravityForSprings = springsByDefaultGravity ? Physics.gravity.magnitude : Mathf.Abs(springsByOtherValue);
        float distCenterFrontWheel = Mathf.Abs(frontRightWheelCenter.z - rb.centerOfMass.z);
        float distCenterRearWheel = Mathf.Abs(rearRightWheelCenter.z - rb.centerOfMass.z);
        float frontSpringWeightRatio = distCenterRearWheel / (distCenterFrontWheel + distCenterRearWheel);
        float rearSpringWeightRatio = 1 - frontSpringWeightRatio;
        springAccelerationPerM[0] = usedGravityForSprings * frontSpringWeightRatio * 0.5f / frontWheelSuspensionDistanceToLiftCarWeight;
        springAccelerationPerM[1] = springAccelerationPerM[0];
        springAccelerationPerM[2] = usedGravityForSprings * rearSpringWeightRatio * 0.5f / rearWheelSuspensionDistanceToLiftCarWeight;
        springAccelerationPerM[3] = springAccelerationPerM[2];
        combinedGroundLayers = solidGround | looseGround;
        extraEffortForWheelSpin = (1 + 2 * frontWheelRotationHoldsRatioOfSpeed + 2 * rearWheelRotationHoldsRatioOfSpeed);
    }

    public void UpdateArrayAccessibleParameters(bool calledFromInspector)
    {
        wheelCenters = new Vector3[] { frontRightWheelCenter + 2 * Vector3.left * frontRightWheelCenter.x, frontRightWheelCenter, rearRightWheelCenter + 2 * Vector3.left * rearRightWheelCenter.x, rearRightWheelCenter };
        Vector3 fr3DOffset = Vector3.right * (frontWheelOutwardThickness - frontWheelInwardThickness) / 2;
        Vector3 br3DOffset = Vector3.right * (rearWheelOutwardThickness - rearWheelInwardThickness) / 2;
        wheel3DCenters = new Vector3[] { wheelCenters[0] - fr3DOffset, wheelCenters[1] + fr3DOffset, wheelCenters[2] - br3DOffset, wheelCenters[3] + br3DOffset };
        upwardSuspensionCap = new float[] { frontSuspHardCap, frontSuspHardCap, rearSuspHardCap, rearSuspHardCap };
        upwardSuspensionCapVec = new Vector3[] { Vector3.up * frontSuspHardCap, Vector3.up * frontSuspHardCap, Vector3.up * rearSuspHardCap, Vector3.up * rearSuspHardCap };
        looseSpringOffsetLength = new float[] { frontWheelSuspensionDistanceToLiftCarWeight, frontWheelSuspensionDistanceToLiftCarWeight, rearWheelSuspensionDistanceToLiftCarWeight, rearWheelSuspensionDistanceToLiftCarWeight };
        wheelRadii = new float[] { frontWheelRadius, frontWheelRadius, rearWheelRadius, rearWheelRadius };

        for (int i = 0; i < 4; i++)
        {
            wheelCentersStartPoint[i] = wheelCenters[i] + upwardSuspensionCapVec[i] + Vector3.up * wheelRadii[i];
            wheelCentersEndPoint[i] = wheelCenters[i] + Vector3.down * (looseSpringOffsetLength[i] + wheelRadii[i]);
            wheel3DCentersStartPoint[i] = wheel3DCenters[i] + upwardSuspensionCapVec[i] + Vector3.up * wheelRadii[i];
            wheel3DCentersEndPoint[i] = wheel3DCenters[i] + Vector3.down * looseSpringOffsetLength[i];
            wheel3DThicknesses[i] = i < 2 ? frontWheelInwardThickness + frontWheelOutwardThickness : rearWheelInwardThickness + rearWheelOutwardThickness;
        }

        if (calledFromInspector && Application.isPlaying) return; //everything after here in this method will not be changed from the inspector during play-Mode

        for (int i = 0; i < 4; i++)
        {
            visualWheelLocalPositions[i] = transform.InverseTransformPoint(visualWheels[i].position);
            visualWheelLocalRotations[i] = Quaternion.Inverse(transform.rotation) * visualWheels[i].rotation;
        }
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

    public float GetAccelerationScaleByGear(int currentGear, float currentSpeed)
    {
        if (numberOfGears == 0) return 1;
        if (gearImpactOnAcceleration == 0) return 1;
        return 1 + gearImpactOnAcceleration - gearScaleDiff * GetGearUneffectivenessRating(currentGear, currentSpeed);
    }

    float GetGearUneffectivenessRating(int currentGear, float currentSpeed)
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


    float GetShiftUpMoment(int currentGear)
    {
        if (currentGear < 0) currentGear = 0;
        return currentGear == (numberOfGears - 1) ? 1 : (gearIsBestAtRelSpeed[currentGear] + gearIsBestAtRelSpeed[currentGear + 1]) / 2;
    }
    float GetShiftDownMoment(int currentGear)
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
            turningRadius = Mathf.Infinity;
        }
        else
        {
            // calculate the local position of the turning center
            float xOfTurningCenter = zDistBetweenRearAndFrontWheels / linesGettingCloserOnYPerX; // a positive value tells how far the turning center is on the right side, a negative for the left side
            float zDistFromFrontWheels = xOfTurningCenter * frontZPerX;
            float zDistFromRearWheels = xOfTurningCenter * rearZPerX;

            // calculate the individually adapted steering angles of ackermann steering
            float rFrontAckermannAngle = Mathf.Atan(zDistFromFrontWheels / (xOfTurningCenter + frontRightWheelCenter.x)) * Mathf.Rad2Deg;
            float lFrontAckermannAngle = Mathf.Atan(zDistFromFrontWheels / (xOfTurningCenter - frontRightWheelCenter.x)) * Mathf.Rad2Deg;
            float rRearAckermannAngle = Mathf.Atan(zDistFromRearWheels / (xOfTurningCenter - rearRightWheelCenter.x)) * Mathf.Rad2Deg;
            float lRearAckermannAngle = Mathf.Atan(zDistFromRearWheels / (xOfTurningCenter + rearRightWheelCenter.x)) * Mathf.Rad2Deg;


            //Lerp between basic steering Angle and the adapted value for Ackermann Steering. A negative value of Ackermann Steering creates "Anti-Ackermann-Steering" (which is the literal opposide)
            rawSteeringAngles[0] = Mathf.LerpUnclamped(frontSteeringAngle, rFrontAckermannAngle, ackermanSteering);
            rawSteeringAngles[1] = Mathf.LerpUnclamped(frontSteeringAngle, lFrontAckermannAngle, ackermanSteering);
            rawSteeringAngles[2] = Mathf.LerpUnclamped(rearSteeringAngle, rRearAckermannAngle, ackermanSteering);
            rawSteeringAngles[3] = Mathf.LerpUnclamped(rearSteeringAngle, lRearAckermannAngle, ackermanSteering);

            //Save the dist from Center for Arcady-Steering and Air-Steering
            float zDistFromCenter = zDistFromFrontWheels - frontRightWheelCenter.z;
            float distFromCenter = Mathf.Sqrt(zDistFromCenter * zDistFromCenter + xOfTurningCenter * xOfTurningCenter);
            if (xOfTurningCenter < 0) distFromCenter = -distFromCenter; //negative Radius for turning left
            turningRadius = distFromCenter;

        }

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

    void ApplyArcadySteeringOrAirSteering()
    {
        //use the air steering or arcady steering depending if at least one wheel is on the ground. Nothing needs to be done here if the possible change is set to 0
        float usedMaxAngularChangePerSecond = numberOfGroundedWheels == 0 ? maxAnglesPSChangeAtAirSteering : maxAnglesPSChangeAtArcadySteering;
        if (usedMaxAngularChangePerSecond == 0) return;

        Vector3 velocityWithoutLocalYSpeed = rb.velocity - Vector3.Dot(rb.velocity, rb.rotation * Vector3.up) * (rb.rotation * Vector3.up);
        float fowardOrBackward = Vector3.Dot(rb.velocity, rb.rotation * Vector3.forward)<0? -1:1;



        float wantedRotationInRadiansPerSecond;
        if(turningRadius == Mathf.Infinity)
        {
            wantedRotationInRadiansPerSecond = 0;
        }
        else
        {
            wantedRotationInRadiansPerSecond = velocityWithoutLocalYSpeed.magnitude* fowardOrBackward / turningRadius;
        }

        float currentLocalYRoation = (transform.InverseTransformDirection(rb.angularVelocity).normalized * rb.angularVelocity.magnitude).y;
        float plannedAPSChange = wantedRotationInRadiansPerSecond - currentLocalYRoation;
        float maxRPSChangeThisFrame = usedMaxAngularChangePerSecond * Time.fixedDeltaTime * Mathf.Deg2Rad;
        plannedAPSChange = Mathf.Clamp(plannedAPSChange, -maxRPSChangeThisFrame, maxRPSChangeThisFrame);

        Vector3 angularVelocityToChange = transform.TransformDirection(Vector3.up).normalized * plannedAPSChange;

        rb.angularVelocity += angularVelocityToChange;
    }

    void ApplyTurnUpwards()
    {
        //The car is only turned upwards if a wheel or the cars collider is in contact with the ground
        if(numberOfGroundedWheels == 0 && lastContactWithGround > Time.fixedDeltaTime) { return; }

        TurnUpwardsField usedTurnAction = numberOfGroundedWheels == 0 ? TurnUpwardsWithZeroWheels : numberOfGroundedWheels < 3 ? TurnUpwardsWithOneOrTwoWheels : TurnUpwardsWithThreeOrFourWheels;
        rb.velocity -= currentGroundNormal * usedTurnAction.downwardAcceleration * Time.fixedDeltaTime;

        //no speed change means the rotation correction is deactivated
        if (usedTurnAction.maxRotSpeedChange == 0) return;

        float offAngleFromNormalInDegree = Vector3.SignedAngle(currentGroundNormal, transform.up, transform.forward);

        //the correction does not happen, when the car is already turend upwards enough;
        if (Mathf.Abs(offAngleFromNormalInDegree) <= usedTurnAction.toleratedAngle) return;

        float currentLocalZRoation = (transform.InverseTransformDirection(rb.angularVelocity).normalized * rb.angularVelocity.magnitude).z;
        float correctionDir = offAngleFromNormalInDegree > 0 ? -1 : 1;
        float maxRotSpeedInRadiants = usedTurnAction.maxRotSpeed * Mathf.Deg2Rad;

        //nothing needs to be done when the car is already rotating with the maximum rotaiton speed or faster into the wanted direction
        if (currentLocalZRoation * correctionDir >= maxRotSpeedInRadiants) return;

        float plannedZRotationChange = usedTurnAction.maxRotSpeedChange * Mathf.Deg2Rad * correctionDir * Time.fixedDeltaTime;

        if(offAngleFromNormalInDegree > 0)
        {
            if(currentLocalZRoation + plannedZRotationChange < -maxRotSpeedInRadiants)
            {
                //when having a positive OffSet and therefore a negative Correction, the negative correction plus the current rotation should not acceed the maximum negative Rotation
                plannedZRotationChange = -maxRotSpeedInRadiants - currentLocalZRoation;
            }
        }
        else
        {
            //when having a negative OffSet and therefore a positive Correction, the positive correction plus the current rotation should not acceed the maximum positive Rotation
            if (currentLocalZRoation + plannedZRotationChange > maxRotSpeedInRadiants)
            {
                plannedZRotationChange = maxRotSpeedInRadiants - currentLocalZRoation;
            }
        }

        Vector3 angularVelocityToChange = transform.TransformDirection(Vector3.forward).normalized * plannedZRotationChange;
        rb.angularVelocity += angularVelocityToChange;
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

    void ManageAudioOverlayAndVisualParticles(int currentGear, float currentWheelSpinSpeed, float currentSpeed)  
    {
        //Audio
        bool isInNegativeGear = currentGear < 0;
        if(currentGear <0) currentGear = 0;

        float pitchRawChange = 0;
        float totalPitch = selectedBasePitchForEngine;

        if (audioForEngine != null)
        {
            if (numberOfGears != 0)
            {
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
            float volumeScaledByGearUneffectiveness = 1 / (1 + GetGearUneffectivenessRating(isInNegativeGear ? -1 : currentGear, currentWheelSpinSpeed) * volumeDropOffAtLowGearEffectiveness);
            totalVolume *= (volumeScaledByThrottle * volumeScaledByGearState * volumeScaledByGearUneffectiveness);
            audioForEngine.pitch = totalPitch;
            audioForEngine.volume = totalVolume;
        }

        //Sliding Tire Sound and Smoke
        float numberOfSlips = 0;
        float combinedSlipRatio = 0;
        for(int i= 0; i < 4; i++)
        {
            int dir = fowardSpeedOverGroundAtWheel[i] > currentWheelSpinningSpeeds[i] ? 1 : -1;
            float slipRatio = GetSlipRatio(fowardSpeedOverGroundAtWheel[i], sideSlideSpeeds[i], currentWheelSpinningSpeeds[i], dir).slilpRatio;

            bool slipsEnough = Mathf.Abs(fowardSpeedOverGroundAtWheel[i]) > 1 && slipRatio > gripOnSolidGround.slipRatioOfMaxGrip + 0.02f && absoluteSpringCompressions[i] > 0;
            if (slipsEnough)
            {
                numberOfSlips++;
                combinedSlipRatio += (slipRatio - gripOnSolidGround.slipRatioOfMaxGrip);
            }


            if (particlePrefab != null)
            {
                if (slipsEnough)
                {
                    if(!particleSystems[i].isPlaying) particleSystems[i].Play();
                    particleGameObjects[i].transform.position = hitPoints[i];
                    particleGameObjects[i].transform.rotation = Quaternion.LookRotation(fowardOnGroundAtWheel[i]*-dir, transform.up) * Quaternion.Euler(-50,0,0);
                    particleSystems[i].enableEmission = true;
                }
                else
                {
                    particleSystems[i].enableEmission = false;
                }
            }

            

        }
        if (audioForSlip != null)
        {
            if(numberOfSlips > 0)
            {
                audioForSlip.volume = (0.2f*numberOfSlips + combinedSlipRatio/6f);
                if (!audioForSlip.isPlaying) audioForSlip.Play();
            }
            else
            {
                //audioForSlip.Pause();
                audioForSlip.volume -= 2 * Time.fixedDeltaTime;
                if (audioForEngine.volume <= 0)
                {
                    audioForSlip.Pause();
                }
            }
        }

        //Overlay
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


    float GetAirResistanceSlowdownForSpeedAbs(float speed)
    {
        return slowByAirResistanceAt30ms  * Mathf.Pow(Mathf.Abs(speed/30), airResistanceExponent);
    }

    float[] GetRollingFrictionSlowdonwForEachWheelAbs(float[] normalForcesUsedForGrip)
    {
        float[] rollingFriction = new float[4];
        for(int i = 0; i < 4; i++)
        {
            rollingFriction[i] = normalForcesUsedForGrip[i] * slowByRollFriction / usedGravityForSprings;
        }
        return rollingFriction;
    }

    void ComputeLengthwiseInputToGetFrontAndRearPower(out ActionMode actionMode, out int forceDirection, out float frontWheelsPowerShare, out float totalAvailablePower, out float usedSlipPreventer, int wheelSpinningDirectionalMultiplier)
    {
        //Note: "motorAcceleration.atNoThrottle" is always 0 or negative, whereas ".atFullThrottle" it is usualy positive but could also turn out negative if a very unsuitable gear is used.
        (float atFullThrottle, float atNoThrottle) motorAcceleration = GetMotorAccelerationAbs();

        if (HandbreakKey.Value > 0)
        {
            actionMode = ActionMode.Handbreak;
            forceDirection = -wheelSpinningDirectionalMultiplier; //breaking applies force into the opposite direction of the current wheel Spin.

            //the breaking from the handbreak and the motor are combined here - as well as the proportional distribution to front or rear wheels.
            //However the combined slow can not exceed the maximum slow from the handbreak. This way the player can not abuse shifting into a way-too-low gear to get unintended good breaking power.
            float handbreakBreakPower = handBreakMaxSlowdown * HandbreakKey.Value * extraEffortForWheelSpin;
            float motorBreakPower = -motorAcceleration.atNoThrottle;
            float ratioOfMotorBreak = motorBreakPower / (motorBreakPower + handbreakBreakPower);
            frontWheelsPowerShare = handBreakAppliedToFrontWheels * (1 - ratioOfMotorBreak) + motorBreakPower * ratioOfMotorBreak;
            totalAvailablePower = handbreakBreakPower + motorBreakPower;
            usedSlipPreventer = handBreakRestrainToOptimiseGrip;
            if (totalAvailablePower > handBreakMaxSlowdown) totalAvailablePower = handBreakMaxSlowdown;
        }
        else if (BreakKey.Value > 0)//breaking works like handbreaking just with other values
        {
            actionMode = ActionMode.Break;
            forceDirection = -wheelSpinningDirectionalMultiplier;
            float breakBreakPower = breakMaxSlowdown * BreakKey.Value * extraEffortForWheelSpin;
            float motorBreakPower = -motorAcceleration.atNoThrottle;
            float ratioOfMotorBreak = motorBreakPower / (motorBreakPower + breakBreakPower);
            frontWheelsPowerShare = breakAppliedToFrontWheels * (1 - ratioOfMotorBreak) + motorBreakPower * ratioOfMotorBreak;
            totalAvailablePower = breakBreakPower + motorBreakPower;
            usedSlipPreventer = breakRestrainToOptimiseGrip;
            if (totalAvailablePower > breakMaxSlowdown) totalAvailablePower = breakMaxSlowdown;
        }
        else if ((gearShiftMode == GearShiftMode.Automatic || numberOfGears == 0) ? (currentGear >= 0 && ThrottleKey.Value > 0) : ThrottleKey.Value > 0)
        {
            float acceleration = motorAcceleration.atFullThrottle * ThrottleKey.Value + motorAcceleration.atNoThrottle * (1 - ThrottleKey.Value);
            frontWheelsPowerShare = rearOrFrontPowered;

            if (acceleration > 0)
            {
                actionMode = ActionMode.AccelerateByMotor;
                totalAvailablePower = acceleration;
                forceDirection = currentGear < 0 ? -1 : 1;
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
        else if (BackwardThrottleKey.Value > 0)
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
    }

    void SplitAvailableImpulseOnLeftAndRightWheels(out float[] availableImpulseForFrame, out bool[] canLeadToDirectionChange, ActionMode actionMode, float frontWheelsPowerShare, int forceDirection
        ,float totalAvailablePower, int wheelSpinningDirectionalMultiplier)
    {
        availableImpulseForFrame = new float[4];
        canLeadToDirectionChange = new bool[] { false, false, false, false };

        float[] rollingFrictioSlowdownForEachWheel = GetRollingFrictionSlowdonwForEachWheelAbs(normalForcesUsedForGrip);

        float fWPowerReservedForBothWheelSpins = frontWheelRotationHoldsRatioOfSpeed * 2; //first of all a part of the estimated power distribution does not depend on grip because it goes into canging the rotation of the wheels (without moving the car itself)
        if (fWPowerReservedForBothWheelSpins > frontWheelsPowerShare) fWPowerReservedForBothWheelSpins = frontWheelsPowerShare; //not more than the entire front wheel power can be powered into pure wheel rotation
        //(same for rear wheels)
        float rearWheelsPowerShare = 1 - frontWheelsPowerShare;
        float rWPowerReservedForBothWheelSpins = rearWheelRotationHoldsRatioOfSpeed * 2;
        if (rWPowerReservedForBothWheelSpins > rearWheelsPowerShare) rWPowerReservedForBothWheelSpins = rearWheelsPowerShare;
        float[] powerShareForWheelRotation = new float[4];
        float[] powerShareForMovement = new float[4];

        for (int i = 0; i < 4; i += 2)//has two iterations. The first one treats the left and right front wheel and the second one treats both rear wheels
        {
            //now split spin power between left and right wheel by the difference of their longitudal slip ratio to balance out unreasonably diffrent spinnging wheels (=by how their individual rotation matches the individual directional movement over the ground)
            float leftRelSpinShare;
            float rightRelSpinShare;

            float leftWheelLongitudalSlipRatio = GetDirectionalLongitudalSlipRatio(currentWheelSpinningSpeeds[0 + i], fowardSpeedOverGroundAtWheel[0 + i], forceDirection);
            float rightWheelLongitudalSlipRatio = GetDirectionalLongitudalSlipRatio(currentWheelSpinningSpeeds[1 + i], fowardSpeedOverGroundAtWheel[1 + i], forceDirection);


            leftRelSpinShare = rightWheelLongitudalSlipRatio - leftWheelLongitudalSlipRatio + 0.5f;
            if (leftRelSpinShare < 0) leftRelSpinShare = 0;
            else if (leftRelSpinShare > 1) leftRelSpinShare = 1;
            rightRelSpinShare = 1 - leftRelSpinShare;


            powerShareForWheelRotation[0 + i] = (i < 2 ? fWPowerReservedForBothWheelSpins : rWPowerReservedForBothWheelSpins) * leftRelSpinShare;
            powerShareForWheelRotation[1 + i] = (i < 2 ? fWPowerReservedForBothWheelSpins : rWPowerReservedForBothWheelSpins) * rightRelSpinShare;


            //the remaining motorPower is splitten along the wheels scaled proportional to their estimated available longitudal grip force
            float remainingMotorShare = (i < 2 ? frontWheelsPowerShare : rearWheelsPowerShare) - (i < 2 ? fWPowerReservedForBothWheelSpins : rWPowerReservedForBothWheelSpins);
            float leftNormalForceForGrip = normalForcesUsedForGrip[0 + i];// / (normalForcesUsedForGrip[0 + i] + normalForcesUsedForGrip[1 + i]);
            float rightNormalForceForGrip = normalForcesUsedForGrip[1 + i];// / (normalForcesUsedForGrip[0 + i] + normalForcesUsedForGrip[1 + i]);
            if (leftNormalForceForGrip == 0 && rightNormalForceForGrip == 0) //prevent division by 0
            {
                leftNormalForceForGrip = 0.5f;
                rightNormalForceForGrip = 0.5f;
            }
            else if (leftNormalForceForGrip < 0 || rightNormalForceForGrip < 0) //this happens when it did not touch the ground previous physic frame. Therefore only the relation between normal forces for grip is used
            {
                leftNormalForceForGrip = normalForcesUsedForGrip[0 + i];
                rightNormalForceForGrip = normalForcesUsedForGrip[1 + i];
            }
            float leftRelShareForMovement = leftNormalForceForGrip / (leftNormalForceForGrip + rightNormalForceForGrip);
            float rightRelShareForMovement = 1 - leftRelShareForMovement;
            powerShareForMovement[0 + i] = remainingMotorShare * leftRelShareForMovement;
            powerShareForMovement[1 + i] = remainingMotorShare * rightRelShareForMovement;

            //combine motor-/break-power with slow from rolling friction. So far we used the power available for one second. To get the power available for this physicFrame we multiply the result with Time.fixedDeltaTime;
            availableImpulseForFrame[0 + i] = ((powerShareForWheelRotation[0 + i] + powerShareForMovement[0 + i]) * forceDirection * totalAvailablePower + rollingFrictioSlowdownForEachWheel[0 + i] * (-wheelSpinningDirectionalMultiplier)) * Time.fixedDeltaTime;
            availableImpulseForFrame[1 + i] = ((powerShareForWheelRotation[1 + i] + powerShareForMovement[1 + i]) * forceDirection * totalAvailablePower + rollingFrictioSlowdownForEachWheel[1 + i] * (-wheelSpinningDirectionalMultiplier)) * Time.fixedDeltaTime;
            if (actionMode == ActionMode.AccelerateByMotor) //only accelerating could lead to a direction change. All types of breaking can not (variable is false by default)
            {
                //accelerating can lead to a direction change, but only when the rolling-friction-slowdown does not cancle it out, because that would make it a form of breaking in total
                canLeadToDirectionChange[0 + i] = availableImpulseForFrame[0 + i] * forceDirection < 0 ? false : true;
                canLeadToDirectionChange[1 + i] = availableImpulseForFrame[1 + i] * forceDirection < 0 ? false : true;
            }
        }
    }

    void ComputeImpulsesWithWheelSpinAndResultingFriction(float[] availableImpulseForFrame, ActionMode actionMode, int forceDirection, float usedSlipPreventer, bool[] canLeadToDirectionChange)
    {
        for (int i = 0; i < 4; i++) //for every wheel
        {
            float availablePower = availableImpulseForFrame[i]; //(note: can be negative)
            float lengthwiseSpeedOverGround = fowardSpeedOverGroundAtWheel[i];
            float spinningSpeed = currentWheelSpinningSpeeds[i];
            if (actionMode != ActionMode.AccelerateByMotor)
            {
                //ensures that breaking does not lead to acceleration. Therefore it must be working against the spinning speed or working against the movement when wheel spin is already lockt to zero
                availablePower = Mathf.Abs(availablePower) * (spinningSpeed > 0 ? -1 : spinningSpeed < 0 ? 1 : lengthwiseSpeedOverGround > 0 ? -1 : lengthwiseSpeedOverGround < 0 ? 1 : 0);
            }
            int availablePowerDirection = availablePower < 0 ? -1 : availablePower > 0 ? 1 : lengthwiseSpeedOverGround < spinningSpeed ? 1 : -1;
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
                    if (float.IsNaN(spinningSpeed)) { Debug.LogError("invalide SpinningSpeed in SZENARIO 0"); }
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
                    // -> this will lead to Szenario 1 or 2
                    //Debug.Log("Szenario 0.1");
                }
                //Debug.Log("spinningSpeed after S0 = " + spinningSpeed+", availablePower after = "+availablePower);

            }

            //PREPARATION for Szenario 1 and 2:
            //do it for current wheel spinn:
            float wheelRotationHoldsRatioOfSpeed = i < 2 ? frontWheelRotationHoldsRatioOfSpeed : rearWheelRotationHoldsRatioOfSpeed;

            (float lengthwise, float sideways) availableGripWithCurrentSpin = GetSpecificGripPower(spinningSpeed, lengthwiseSpeedOverGround, sideSlideSpeeds[i], availablePowerDirection, collidedGroundType[i], normalForcesUsedForGrip[i], i < 2);

            //find spinning speed for best slip ratio (the slip ratio, where the best grip is reached)
            float bestSlipRatio = collidedGroundType[i] == 1 ? gripOnSolidGround.slipRatioOfMaxGrip : 2;

            float slipAngleRadiants = GetAbsSlipAngle(lengthwiseSpeedOverGround, sideSlideSpeeds[i]) * Mathf.Deg2Rad;
            float minSlipRatioByAngle = Mathf.Sqrt(2 - 2 * Mathf.Cos(slipAngleRadiants));
            float plannedK;
            if (minSlipRatioByAngle >= bestSlipRatio)
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


            float plannedSpin = spinningSpeed;
            float impulseWithPlannedSpin = availablePower;
            (float lengthwise, float sideways) availableGripWithPlannedSpin = (0, 0);


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
                        impulseWithPlannedSpin += powerDif / Mathf.Pow(2, j);
                    }
                    else
                    {
                        plannedSpin -= spinDif / Mathf.Pow(2, j);
                        impulseWithPlannedSpin -= powerDif / Mathf.Pow(2, j);
                    }

                    availableGripWithPlannedSpin = GetSpecificGripPower(plannedSpin, lengthwiseSpeedOverGround, sideSlideSpeeds[i], availablePowerDirection, collidedGroundType[i], normalForcesUsedForGrip[i], i < 2);
                    if (availableGripWithPlannedSpin.lengthwise > impulseWithPlannedSpin * availablePowerDirection)
                    {
                        gripIsHigher = true;
                    }
                    else if (availableGripWithPlannedSpin.lengthwise < impulseWithPlannedSpin * availablePowerDirection)
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
                if (spinningSpeed * availablePowerDirection < plannedSpinningSpeedForBestSlipRatio * availablePowerDirection) //having a lower slip, than the perfect slip ratio
                {
                    //if the grip is yet not good enough at the current spin, but would be, if it spins at the perfect slip ratio, find the spinSpeed between thouse, where grip and power are equal.
                    if (availableGripPowerWithBestSlipRatioSpin.lengthwise > availablePowerForBestSlipRatio * availablePowerDirection)
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
                                impulseWithPlannedSpin -= powerDif / Mathf.Pow(2, j);
                            }
                            else
                            {
                                plannedSpin += spinDif / Mathf.Pow(2, j);
                                impulseWithPlannedSpin += powerDif / Mathf.Pow(2, j);
                            }

                            availableGripWithPlannedSpin = GetSpecificGripPower(plannedSpin, lengthwiseSpeedOverGround, sideSlideSpeeds[i], availablePowerDirection, collidedGroundType[i], normalForcesUsedForGrip[i], i < 2);
                            if (availableGripWithPlannedSpin.lengthwise > impulseWithPlannedSpin * availablePowerDirection)
                            {
                                gripIsHigher = true;
                            }
                            else if (availableGripWithPlannedSpin.lengthwise < impulseWithPlannedSpin * availablePowerDirection)
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
                        impulseWithPlannedSpin = availableGripPowerWithBestSlipRatioSpin.lengthwise * availablePowerDirection;
                        availableGripWithPlannedSpin = availableGripPowerWithBestSlipRatioSpin;
                        float remainingPower = availablePower - availableGripPowerWithBestSlipRatioSpin.lengthwise * availablePowerDirection;
                        float fullPoweredSpin = plannedSpinningSpeedForBestSlipRatio + remainingPower / wheelRotationHoldsRatioOfSpeed;
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
                    float spinChangeForSlipReduction = spinDifToPerfectSlipRatio * (1 - Mathf.Pow(1 - usedSlipPreventer, Time.fixedDeltaTime)); //(0 without slip prevention and as big as the spinDifToPerfectSlipRatio at full slip prevention)
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
            else //SZENARIO 3: the aviable grip at the current wheel spinning speed fits exactly the power used to change the cars speed. Therefore the wheel spin does not change at all
            {
                availableGripWithPlannedSpin = GetSpecificGripPower(plannedSpin, lengthwiseSpeedOverGround, sideSlideSpeeds[i], availablePowerDirection, collidedGroundType[i], normalForcesUsedForGrip[i], i < 2);
                if (float.IsNaN(plannedSpin)) { Debug.LogError("invalide SpinningSpeed in SZENARIO 3"); }
                //Debug.Log("Szenario 3");
            }


            //all forms of breaking or slowdown can slow the wheel spin down or even lock them, but should not be able to accelerate the spin into the opposite direction.
            //The following lines prevent cases, where the previous code wrongfully created that behaviour
            if (!canLeadToDirectionChange[i] && (plannedSpin * lengthwiseSpeedOverGround < 0) && (spinningSpeed * plannedSpin < 0 || (spinningSpeed == 0 && plannedSpin != 0)))
            {
                plannedSpin = 0;
                availableGripWithPlannedSpin = GetSpecificGripPower(plannedSpin, lengthwiseSpeedOverGround, sideSlideSpeeds[i], availablePowerDirection, collidedGroundType[i], normalForcesUsedForGrip[i], i < 2);
                impulseWithPlannedSpin = availableImpulseForFrame[i] + currentWheelSpinningSpeeds[i] * wheelRotationHoldsRatioOfSpeed;
                if (float.IsNaN(plannedSpin)) { Debug.LogError("invalide SpinningSpeed in SZENARIO anti-change-correction"); }
                //Debug.Log("Szenario anti-change-correction");
            }


            ////This part of the code is only called when the car does not apply any force to the wheel and therefore the offset spin to the ground has not been computet yet.
            ////In this case access spin gets (partly) balanced out (depending on grip) to better match the wheels speed over the ground. The force of the changed wheel spin changes the cars speed into the opposed direction.
            ////example: if wheels spin faster, than how fast they move over the ground, they accelerate the car and their own spin is slowed down in return. If they spin too slow, their rotation is accelerated on cost of the car's speed.
            if (impulseWithPlannedSpin > availableGripWithPlannedSpin.lengthwise) { impulseWithPlannedSpin = availableGripWithPlannedSpin.lengthwise; }
            else if (impulseWithPlannedSpin < -availableGripWithPlannedSpin.lengthwise) { impulseWithPlannedSpin = -availableGripWithPlannedSpin.lengthwise; }

            lengthwiseForceAtWheels[i] = impulseWithPlannedSpin;
            sidewayGripAtWheels[i] = availableGripWithPlannedSpin.sideways;
            currentWheelSpinningSpeeds[i] = plannedSpin;

            if (float.IsNaN(plannedSpin)) { Debug.LogError("invalide Spin"); }
        }
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

            atFullThrottle = baseAccelerationAtCurrentSpeed * accelerationScalerBySuitabilityOfGear * extraEffortForWheelSpin + airResistanceToOvercome + slowByRollFriction;

            if (spinFasterThanOptimum) //The motor can only create a slowdown when the gear is too low for the current speed
            {
                atNoThrottle = baseAccelerationAtCurrentSpeed * (accelerationScalerBySuitabilityOfGear - 1) * extraEffortForWheelSpin;// * spinDirectionMultiplier;
                if(atNoThrottle>0) atNoThrottle = 0;
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
            float punishmentForBadGearRatio = (accelerationScalerBySuitabilityOfGear - gearScaleAtMinimum) * theoreticalAccelerationAtMaxSpeed * extraEffortForWheelSpin; //is 0 or negative because the accelerationScalerBSOG is <= gearScaleAtMinimum
            float airResistanceToOvercomeAtMaxSpeed = GetAirResistanceSlowdownForSpeedAbs(speedupCurve.topSpeed);

            atFullThrottle = (airResistanceToOvercomeAtMaxSpeed + slowByRollFriction + punishmentForBadGearRatio);
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

    float GetDirectionalLongitudalSlipRatio(float spinSpeed, float groundSpeed, int wantedDirection)
    {
        //the slipRatio in this method is calculated with a bias to apply force into a direction
        float slipRatio;
        spinSpeed *= wantedDirection;
        groundSpeed *= wantedDirection;

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

        if (endlessLengthwiseGrip) availableLengthwiseGripPower = 1000 * Time.fixedDeltaTime;

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

    private void BalanceAndApplyImpulses(Vector3[] posToApplyForce, Vector3[] directionToApplyForce, float[] maxForce, float[] currentSpeedAgainstForceDirection, bool[] isBreakingForce)
    {
        (float maxV, float maxVDir, Vector3 maxW, Vector3 rotToV, Vector3 vDirection)[] impulseProperties = new (float maxV, float maxVDir, Vector3 maxW, Vector3 rotToV, Vector3 vDirection)[4];
        for (int i = 0; i < 4; i++)
        {
            impulseProperties[i] = CalculateEffectOfImpulseOnDirectionalMovementAtPoint(posToApplyForce[i], directionToApplyForce[i], maxForce[i]);

            //the impact of the rotation by applying force at a certain position is reduced depending on how much the natural steering is disabled for aracde steering
            impulseProperties[i].maxW = impulseProperties[i].maxW * (1f - disableNaturalSteering);
            impulseProperties[i].maxV = Mathf.Lerp(impulseProperties[i].maxV, impulseProperties[i].maxVDir, disableNaturalSteering);
        }
        float[,] maxsConversionsToMyV = new float[4, 4]; //How much would the maximum impulse of another Wheel impact my own directional velocity [toMyV,fromOtherRatio]
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                if (i == j)
                {
                    maxsConversionsToMyV[i, j] = 0; //exclude conversion from self
                }
                else if (impulseProperties[j].maxV == 0)
                {
                    maxsConversionsToMyV[i, j] = 0; //no conversion from wheel, which is not on ground
                }
                else
                {
                    float fromOtherMaxV = Vector3.Dot(impulseProperties[j].maxVDir * impulseProperties[j].vDirection, impulseProperties[i].vDirection.normalized); //Projection of other directionalV on my directionalV
                    float fromOtherMaxW = impulseProperties[j].maxW.x * impulseProperties[i].rotToV.x + impulseProperties[j].maxW.y * impulseProperties[i].rotToV.y + impulseProperties[j].maxW.z * impulseProperties[i].rotToV.z;
                    maxsConversionsToMyV[i, j] = fromOtherMaxV + fromOtherMaxW;
                }
            }
        }

        //breaking forces start with a ratio of 0. That way the aproximation does not oscillate
        float[] plannedRatios = new float[4] { 0, 0, 0, 0 };
        float[] prevIterationPR = new float[4] { -2, -2, -2, -2 };

        int numberOfAccelerationForces = 0;
        for (int i = 0; i < 4; i++)
        {
            if (!isBreakingForce[i])
            {
                numberOfAccelerationForces++;
                plannedRatios[i] = 1;
            }
        }

        //every wheels Friction is limited to only stop the wheels lateral movement and can not make it move into the opposide direction
        //this loop chooses this correct possible friction impulse with respect to all interactions with other wheels
        if (numberOfAccelerationForces < 4) //aproximation is only needed for breaking forces. So this part is skipped if every wheel accelerates.
        {
            for (int iterationCount = 0; iterationCount < 100; iterationCount++)
            {

                for (int i = 0; i < 4; i++)
                {
                    if (!isBreakingForce[i])
                    {
                        continue; //no need to limit forces, which are no breaking forces
                    }

                    if (impulseProperties[i].maxV == 0) //ignore Wheels without friction/ without contact to floor
                    {
                        plannedRatios[i] = 0;
                        continue;
                    }
                    float vFromOtherWheels = maxsConversionsToMyV[i, 0] * plannedRatios[0] + maxsConversionsToMyV[i, 1] * plannedRatios[1] + maxsConversionsToMyV[i, 2] * plannedRatios[2] + maxsConversionsToMyV[i, 3] * plannedRatios[3];
                    float neededOwnV = currentSpeedAgainstForceDirection[i] - vFromOtherWheels;
                    plannedRatios[i] = neededOwnV / impulseProperties[i].maxV;
                    if (plannedRatios[i] > 1) plannedRatios[i] = 1;
                    if (plannedRatios[i] < -1) plannedRatios[i] = -1;
                }

                if (plannedRatios[0] == prevIterationPR[0] && plannedRatios[1] == prevIterationPR[1] && plannedRatios[2] == prevIterationPR[2] && plannedRatios[3] == prevIterationPR[3])
                {
                    break;
                }
                prevIterationPR[0] = plannedRatios[0]; prevIterationPR[1] = plannedRatios[1]; prevIterationPR[2] = plannedRatios[2]; prevIterationPR[3] = plannedRatios[3];
            }
        }


        //Apply friction-impulses of all wheels
        Vector3 angularVelChange = Vector3.zero;
        Vector3 directionalVelChange = Vector3.zero;
        for (int i = 0; i < 4; i++)
        {
            angularVelChange += plannedRatios[i] * impulseProperties[i].maxW;
            directionalVelChange += plannedRatios[i] * impulseProperties[i].maxVDir * impulseProperties[i].vDirection;

            Debug.DrawRay(posToApplyForce[i] + transform.up * 0.05f, impulseProperties[i].maxV * impulseProperties[i].vDirection * 0.5f, Color.blue);
        }
        rb.AddRelativeTorque(new Vector3(angularVelChange.x * inertiaTensorWS.x, angularVelChange.y * inertiaTensorWS.y, angularVelChange.z * inertiaTensorWS.z), ForceMode.Impulse);
        rb.AddForce(directionalVelChange * rb.mass, ForceMode.Impulse);
    }
    void PerformInAirBalance()
    {
        bool doInstantCorrection = justLeftGround && fixFlightRotationAtLeavingGround;
        if ((timeInAir < minFlightDurationForCorrection || maxAngularVelCorrectionInAir==0) && !doInstantCorrection) return;

        Vector3 startingCenter = transform.position;

        float timeStepDuration = 0.05f; //predicted time per ray in seconds
        int numberOfTimesteps = doInstantCorrection ? 300 : 30; // 300 * 0.05s = 15s predicted duration at leaving ground and 25*0.05s = 1.5s predicted duration in Air
        Vector3[] flightPositions = new Vector3[numberOfTimesteps + 1];
        Vector3[,] dotCurves = new Vector3[3, numberOfTimesteps + 1]; //It will contain three "curves", each represented by a row of points.
                                                                      // The points will be connected by raycasts to form the curve and check where the curve collides with terrain.
        Vector3[] hitPos = new Vector3[3];
        float[] hitT = new float[3]; //a curve represents the car's predicted movement. "hitT[curveIndex]" saves the estimated time from each curve to hit terrain.
        Quaternion[] moveDirRotAtIndex = new Quaternion[numberOfTimesteps + 1];
        float[] speedAtTimestep = new float[numberOfTimesteps + 1];

        //i=0 is calculated outside loop
        bool usesCustomGravity = (gravityReciver != null);
        moveDirRotAtIndex[0] = Quaternion.LookRotation(rb.velocity, Vector3.up);
        flightPositions[0] = startingCenter;
        Vector3 gravityAtPrevPoint = usesCustomGravity ? gravityReciver.GetGravityForPosition(flightPositions[0]) : Physics.gravity;
        Vector3 velocityAtPrevTimestep = rb.velocity; //+ gravityReciver.GetGravityForPosition(v0[0]) * 0.5f;
        speedAtTimestep[0] = velocityAtPrevTimestep.magnitude;

        for (int i = 1; i <= numberOfTimesteps; i++)
        {
            if (usesCustomGravity && gravityReciver.CurveSpace)
            {
                //chagne previous velocity with gravity change if curved space is enabled
                Vector3 gravityTwoStepsAgo = gravityAtPrevPoint;
                gravityAtPrevPoint = gravityReciver.GetGravityForPosition(flightPositions[i - 1]);

                float angle = Vector3.Angle(gravityTwoStepsAgo, gravityAtPrevPoint);
                if (angle != 0 && angle < 10)
                {
                    velocityAtPrevTimestep = Quaternion.FromToRotation(gravityTwoStepsAgo, gravityAtPrevPoint) * velocityAtPrevTimestep;
                }
            }
            else
            {
                gravityAtPrevPoint = usesCustomGravity ? gravityReciver.GetGravityForPosition(flightPositions[i - 1]) : Physics.gravity;
            }
            moveDirRotAtIndex[i] = Quaternion.LookRotation(velocityAtPrevTimestep, -gravityAtPrevPoint);
            flightPositions[i] = flightPositions[i - 1] + velocityAtPrevTimestep * timeStepDuration + 0.5f * gravityAtPrevPoint * timeStepDuration * timeStepDuration;
            Debug.DrawLine(flightPositions[i - 1], flightPositions[i], Color.black, Time.fixedDeltaTime);

            velocityAtPrevTimestep += gravityAtPrevPoint * timeStepDuration;
            velocityAtPrevTimestep -= GetAirResistanceSlowdownForSpeedAbs(velocityAtPrevTimestep.magnitude) * timeStepDuration * velocityAtPrevTimestep.normalized;
            speedAtTimestep[i] = velocityAtPrevTimestep.magnitude; //velocityAtPrevTimestep is the velocity of the current timestep already here
        }


        for (int curveIndex = 0; curveIndex < 3; curveIndex++)
        {
            //each line has a baseOffset to seperate it's position from the other lines. For each point in a line this Offset is Rotatet into the estimated move direction at that point
            Vector3 lineBaseOffset = curveIndex == 0 ? new Vector3(0, 1.5f, 0)
                                   : curveIndex == 1 ? new Vector3(-0.866f, -0.5f, 0)
                                                    : new Vector3(0.866f, -0.5f, 0);
            dotCurves[curveIndex, 0] = flightPositions[0] + moveDirRotAtIndex[0] * lineBaseOffset;
            for (int i = 0; i < numberOfTimesteps; i++)
            {
                dotCurves[curveIndex, i + 1] = flightPositions[i + 1] + moveDirRotAtIndex[i + 1] * lineBaseOffset;
                float rayLength = Vector3.Distance(dotCurves[curveIndex, i], dotCurves[curveIndex, i + 1]);
                Debug.DrawLine(dotCurves[curveIndex, i], dotCurves[curveIndex, i + 1], Color.magenta, Time.fixedDeltaTime);
                if (Physics.Raycast(dotCurves[curveIndex, i], dotCurves[curveIndex, i + 1] - dotCurves[curveIndex, i], out RaycastHit hit, rayLength, combinedGroundLayers))
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
            float speedAtContact = gravityReciver == null ? (rb.velocity + unreducedTimeToContact * Physics.gravity).magnitude : speedAtTimestep[Mathf.FloorToInt(unreducedTimeToContact/timeStepDuration)];
            float earlyerArrival = 0.3f / speedAtContact; //the wheels arrive earlyer at the ground than the car, so time needs to be subtracted.
            float timeToContact = unreducedTimeToContact - earlyerArrival;
            if (timeToContact < minDurationTillHit) return;


            Vector3 contactNormal = Vector3.Cross(hitPos[2] - hitPos[0], hitPos[1] - hitPos[0]);
            Vector3 coaseImpactPos = Vector3.Lerp(flightPositions[Mathf.FloorToInt(timeToContact / timeStepDuration)], flightPositions[Mathf.CeilToInt(timeToContact / timeStepDuration)], (timeToContact / timeStepDuration)%1);


            //DEBUG DELETE LATER
            Debug.DrawRay(hitPos[0], contactNormal, Color.red, Time.fixedDeltaTime);
            Debug.DrawRay(hitPos[1], contactNormal, Color.red, Time.fixedDeltaTime);
            Debug.DrawRay(hitPos[2], contactNormal, Color.red, Time.fixedDeltaTime);


            ////> Since full rotations are not included in our aimed rotation, they are added here to get the correct deltaRotation later:
            //int numberOfExtraRotations = Mathf.RoundToInt(rotMagnitudeDone / 360);
            //Quaternion extraRotationsAsQuaternion = Quaternion.Euler(numberOfExtraRotations * 2 * Mathf.PI / timeToContact * rb.angularVelocity.normalized);
            //aimedRotation = extraRotationsAsQuaternion * aimedRotation;
            ////< Since unitys Quaternions only hold rotations up to one full rotation, it is even like that not possibel to include them. We would need another math library or change everyhting to matrix4x4

            float rotMagnitudeDone = rb.angularVelocity.magnitude * Mathf.Rad2Deg * timeToContact;
            Quaternion rotChangeWithCurrentAngVel = Quaternion.AngleAxis(rotMagnitudeDone, rb.angularVelocity.normalized);
            Quaternion rotOnArrivalWithCurretnAV = rotChangeWithCurrentAngVel * rb.rotation;
            Vector3 aimedforward = Vector3.Cross(rotOnArrivalWithCurretnAV * Vector3.right, contactNormal);
            Quaternion aimedRotation = Quaternion.LookRotation(aimedforward, contactNormal);

            //visual debugging
            if (visualDebugCar != null)
            {
                visualDebugCar.SetPositionAndRotation(coaseImpactPos, aimedRotation);
            }

            if (doInstantCorrection && timeToContact > 0.7f)
            {
                //since expectedToWantedRot * rotOnArrivalWithCurretnAV = aimedRotation
                Quaternion expextedToWantedRot = aimedRotation * Quaternion.Inverse(rotOnArrivalWithCurretnAV);
                Quaternion wantedAngularVelQuaternion = expextedToWantedRot * rotChangeWithCurrentAngVel;

                //Vector3 axis;
                wantedAngularVelQuaternion.ToAngleAxis(out float angle, out Vector3 axis);
                if (float.IsInfinity(axis.x)) return; //infinite axis means rot is already alligned
                                                      //if (angle > 180f) angle -= 360f;
                Vector3 wantedAngularVelocity = (angle * Mathf.Deg2Rad / timeToContact) * axis.normalized;
                rb.angularVelocity = wantedAngularVelocity;
            }
            else if (timeInAir >= minFlightDurationForCorrection)
            {

                Quaternion expectedRotChange = Quaternion.Euler(
                        rb.angularVelocity.x * Mathf.Rad2Deg * timeToContact,
                        rb.angularVelocity.y * Mathf.Rad2Deg * timeToContact,
                        rb.angularVelocity.z * Mathf.Rad2Deg * timeToContact
                        );

                //A * B * C = D      means    changeWithCurrentAV * (AVChange * currentRot) = aimed 
                //B = A^-1 * D * C^-1
                Quaternion AVChange = Quaternion.Inverse(expectedRotChange) * aimedRotation * Quaternion.Inverse(rb.rotation); //WITH OLD AIMED ROTATION


                //the angle is calculated by an adaptation of the solution from DMGregory at https://gamedev.stackexchange.com/questions/147409/rotate-from-current-rotation-to-another-using-angular-velocity (15.04.2024)
                float angle; Vector3 axis;
                AVChange.ToAngleAxis(out angle, out axis);
                // We get an infinite axis in the event that our rotation is already aligned.
                if (float.IsInfinity(axis.x)) return; //infinite axis means rot is already alligned
                if (angle > 180f) angle -= 360f;

                //Goal orientated version
                Vector3 wantedAngularVelChange = (0.98f * angle * Mathf.Deg2Rad / timeToContact) * axis.normalized;
                wantedAngularVelChange = new Vector3(wantedAngularVelChange.x, 0, wantedAngularVelChange.z);

                float angularChangeLimitScale = wantedAngularVelChange.magnitude > maxAngularVelCorrectionInAir * Time.fixedDeltaTime ? maxAngularVelCorrectionInAir * Time.fixedDeltaTime / wantedAngularVelChange.magnitude : 1;
                rb.angularVelocity += wantedAngularVelChange * angularChangeLimitScale; // continiouse rotation speed change in the air, limited by maxAngularVelCorrectionInAir.
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
            previouslyAcceptedGroundNormal = collision.contacts[0].normal;
        }
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


