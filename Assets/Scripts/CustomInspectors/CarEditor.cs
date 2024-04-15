#if UNITY_EDITOR
using UnityEngine;
using UnityEditor;
using System;
//using UnityEngine.UIElements;
using Unity.VisualScripting;
//using UnityEditorInternal;
//using System.Reflection;
//using System.Runtime.ConstrainedExecution;
//using UnityEditor.UIElements;
//using System.Drawing;


//This class serves as the inspector-UI for "Car.cs". Most properties are appear 3 times:  1.Defined as SerializedProperty    2.Fetched from Car.cs in "OnEnable()"    3.Dispalyed and managed in "OnInspectorGUI()"
[CustomEditor(typeof(Car))]
[CanEditMultipleObjects]
public class CarEditor : Editor
{
    Color colorForPerformance = new Color(0.5f, 0.7f, 0.5f); //green
    Color colorForAccuracy = new Color(0.5f, 0.5f, 0.7f); //blue

    Color colorForRealism = new Color(0.8f, 0.8f, 0.0f); //yellow
    Color colorForArcade = new Color(0.9f, 0.3f, 0.3f); //red

    //Information for Background Rectangles
    readonly float lineH = EditorGUIUtility.singleLineHeight;
    float[] sectionHeights = new float[20];
    (float from, float to)[] subrectangles = new (float, float)[5];
    float gearRecTo = 0;
    
    //Background color 0 as texture
    Texture2D tex0;
    //Background color 1 as texture
    Texture2D tex1;

    

    Car car;

    //acceleration
    SerializedProperty speedupCurve;
    SerializedProperty rearOrFrontPowered;
    SerializedProperty accelerationRestrainToOptimiseGrip;

    //gears
    SerializedProperty gearImpactOnAcceleration;
    SerializedProperty numberOfGears;
    SerializedProperty gearOutDuration;
    SerializedProperty shiftDuration;
    SerializedProperty gearInDuration;
    SerializedProperty shiftingImpactOnAcceleration;
    SerializedProperty gearShiftMode;

    //decceleration
    SerializedProperty slowByAirResistanceAt30ms;
    SerializedProperty airResistanceExponent;
    SerializedProperty slowByRollFriction;

    //breaking Settings
    SerializedProperty breakMaxSlowdown;
    SerializedProperty breakAppliedToFrontWheels;
    SerializedProperty breakRestrainToOptimiseGrip;
    SerializedProperty breakLimitedByGrip;
    SerializedProperty handBreakMaxSlowdown;
    SerializedProperty handBreakAppliedToFrontWheels;
    SerializedProperty handBreakRestrainToOptimiseGrip;
    SerializedProperty handBreakLimitedByGrip;


    //physical wheel-position and spring settings
    SerializedProperty showWheelSettings;
    SerializedProperty frontRightWheelCenter;
    SerializedProperty frontWheelRadius;
    SerializedProperty frontWheelRotationHoldsRatioOfSpeed;
    SerializedProperty frontUse3DWheelPhysics;
    SerializedProperty frontWheelInwardThickness;
    SerializedProperty frontWheelOutwardThickness;
    SerializedProperty frontWheelShapeAccuracy;
    SerializedProperty frontSuspDist;
    SerializedProperty frontSuspHardCap;
    //SerializedProperty showRearWheelSettings;
    SerializedProperty rearRightWheelCenter;
    SerializedProperty rearWheelRadius;
    SerializedProperty rearWheelRotationHoldsRatioOfSpeed;
    SerializedProperty rearUse3DWheelPhysics;
    SerializedProperty rearWheelInwardThickness;
    SerializedProperty rearWheelOutwardThickness;
    SerializedProperty rearWheelShapeAccuracy;
    SerializedProperty rearSuspDist;
    SerializedProperty rearSuspHardCap;

    SerializedProperty springsByDefaultGravity;
    SerializedProperty springsByOtherValue;

    //balance
    SerializedProperty damping;
    SerializedProperty antiJumpSuspension;
    SerializedProperty lateralAttackHeightLift;
    SerializedProperty longitudalAttackHeightLift;
    SerializedProperty frontAntiRollBar;
    SerializedProperty rearAntiRollBar;
    SerializedProperty solidGroundLayer;
    SerializedProperty looseGroundLayer;
    SerializedProperty TurnUpwardsWithZeroWheels;
    SerializedProperty TurnUpwardsWithOneOrTwoWheels;
    SerializedProperty TurnUpwardsWithThreeOrFourWheels;
    SerializedProperty minFlightDurationForCorrection;
    SerializedProperty maxAngularVelCorrectionInAir;
    SerializedProperty fixFlightRotationAtLeavingGround;


    SerializedProperty scaleGripWithSpringCompression;
    SerializedProperty scaleGripWithDampingCompression;
    SerializedProperty spreadGripFromNormalForceOnAllWheels;

    SerializedProperty sameGripSettingsForAllWheels;
    SerializedProperty endlessLengthwiseGrip;
    SerializedProperty endlessSidewaysGrip;
    //SerializedProperty frontWheelGrip;
    //SerializedProperty backWheelGrip;

    //SerializedProperty frontGripOnSolidGround;
    //SerializedProperty frontGripOnLooseGround;
    //SerializedProperty rearGripOnSolidGround;
    //SerializedProperty rearGripOnLooseGround;

    //grip             (Fw = Front Wheel, Rw = Rear Wheel, Sg = Solide Ground, Lg = Loose Ground)
    SerializedProperty lengthwiseGripAffectedBySidewaysSlip;
    SerializedProperty sidewaysGripAffectedByLengthwiseSlip;
    SerializedProperty useSameGripMultipliersAsSolidGround;
    SerializedProperty FwSgLengthwiseGrip;
    SerializedProperty RwSgLengthwiseGrip;
    SerializedProperty FwSgSidewaysGrip;
    SerializedProperty RwSgSidewaysGrip;
    SerializedProperty FwLgLengthwiseGrip;
    SerializedProperty RwLgLengthwiseGrip;
    SerializedProperty FwLgSidewaysGrip;
    SerializedProperty RwLgSidewaysGrip;

    //steering
    SerializedProperty ackermanSteering;
    SerializedProperty maxSteerChangePerSecond;
    SerializedProperty frontSteerAtZeroSpeed;
    SerializedProperty frontSteerAt30MS;
    SerializedProperty rearSteerAtZeroSpeed;
    SerializedProperty rearSteerAt30MS;
    //steering balancing
    SerializedProperty frontSteerTowardsSlide;
    SerializedProperty maxFrontAngleTowardsSlide;
    SerializedProperty rearSteerTowardsSlide;
    SerializedProperty maxRearAngleTowardsSlide;
    //Arcady Steer
    SerializedProperty disableNaturalSteering;
    SerializedProperty maxAnglesPSChangeFromArcadySteering;
    //Air Steer
    SerializedProperty maxAnglesPSChangeAtAirSteering;

    //input
    SerializedProperty breakAtOtherDirectionInput; 
    SerializedProperty ThrottleKey;
    SerializedProperty BackwardThrottleKey;
    SerializedProperty SteerLeftKey;
    SerializedProperty SteerRightKey;
    SerializedProperty BreakKey;
    SerializedProperty HandbreakKey;
    SerializedProperty GearShiftKey;
    SerializedProperty GearShiftUpKey;
    SerializedProperty GearShiftDownKey;
    SerializedProperty NitroKey;

    //audio
    SerializedProperty audioForEngine;
    SerializedProperty deltaPitchAtLowGearSpeed;
    SerializedProperty deltaPitchAtHighGearSpeed;
    SerializedProperty amplifyDeltaPitchPerGear;
    SerializedProperty volumeDropOffAtGearShift;
    SerializedProperty volumeScaleWithThrottleInput;
    SerializedProperty volumeDropOffAtLowGearEffectiveness;

    //gui
    SerializedProperty sliderToShowShift;
    SerializedProperty sliderToShowClutch;
    SerializedProperty guiGear;
    SerializedProperty guiSpeed;

    //visualisation
    Transform[] visualWheels;
    SerializedProperty placeOnPhysicalWheel;

    int speedUnit = 0;
    string speedUnitWord = "";
    float speedUnitScaler = 1;
    float pickedOrientationAirSlowdowm = 30f;
    float pickedAirSlowdownAtOrientation = 1f;
    CarGizmos carGizmos;
    
    //Which Settings are folded out:
    bool showAccelerationSettings = false;
    bool showGearSettings = false;
    bool showBreakSettings = false;
    bool showBalanceSettings = false;



    void OnEnable()
    {
        car = target as Car;

        //Background color 0 as texture
        tex0 = new Texture2D(1, 1, TextureFormat.RGBA32, false);
        tex0.SetPixel(0, 0, new Color(0.20f, 0.20f, 0.22f));
        tex0.Apply();
        //Background color 1 as texture
        tex1 = new Texture2D(1, 1, TextureFormat.RGBA32, false);
        tex1.SetPixel(0, 0, new Color(0.17f, 0.17f, 0.19f));
        tex1.Apply();







        //accelleration settings
        speedupCurve = serializedObject.FindProperty("speedupCurve");
        rearOrFrontPowered = serializedObject.FindProperty("rearOrFrontPowered");
        accelerationRestrainToOptimiseGrip = serializedObject.FindProperty("accelerationRestrainToOptimiseGrip");
        //gear settings
        gearImpactOnAcceleration = serializedObject.FindProperty("gearImpactOnAcceleration");
        numberOfGears = serializedObject.FindProperty("numberOfGears");
        gearOutDuration = serializedObject.FindProperty("gearOutDuration");
        shiftDuration = serializedObject.FindProperty("shiftDuration");
        gearInDuration = serializedObject.FindProperty("gearInDuration");
        shiftingImpactOnAcceleration = serializedObject.FindProperty("shiftingImpactOnAcceleration");
        gearShiftMode = serializedObject.FindProperty("gearShiftMode");
        //decceleration settings
        slowByAirResistanceAt30ms = serializedObject.FindProperty("slowByAirResistanceAt30ms");
        airResistanceExponent = serializedObject.FindProperty("airResistanceExponent");
        slowByRollFriction = serializedObject.FindProperty("slowByRollFriction");
        //breaking settings
        breakMaxSlowdown = serializedObject.FindProperty("breakMaxSlowdown");
        breakAppliedToFrontWheels = serializedObject.FindProperty("breakAppliedToFrontWheels");
        breakRestrainToOptimiseGrip = serializedObject.FindProperty("breakRestrainToOptimiseGrip");
        breakLimitedByGrip = serializedObject.FindProperty("breakLimitedByGrip");
        handBreakMaxSlowdown = serializedObject.FindProperty("handBreakMaxSlowdown");
        handBreakAppliedToFrontWheels = serializedObject.FindProperty("handBreakAppliedToFrontWheels");
        handBreakRestrainToOptimiseGrip = serializedObject.FindProperty("handBreakRestrainToOptimiseGrip");
        handBreakLimitedByGrip = serializedObject.FindProperty("handBreakLimitedByGrip");
        //front wheel physical position and spring settings
        showWheelSettings = serializedObject.FindProperty("showWheelSettings");
        frontRightWheelCenter = serializedObject.FindProperty("frontRightWheelCenter");
        frontWheelRadius = serializedObject.FindProperty("frontWheelRadius");
        frontWheelRotationHoldsRatioOfSpeed = serializedObject.FindProperty("frontWheelRotationHoldsRatioOfSpeed");
        frontUse3DWheelPhysics = serializedObject.FindProperty("frontUse3DWheelPhysics");
        frontWheelInwardThickness = serializedObject.FindProperty("frontWheelInwardThickness");
        frontWheelOutwardThickness = serializedObject.FindProperty("frontWheelOutwardThickness");
        frontWheelShapeAccuracy = serializedObject.FindProperty("frontWheelShapeAccuracy");
        frontSuspDist = serializedObject.FindProperty("frontWheelSuspensionDistanceToLiftCarWeight");
        frontSuspHardCap = serializedObject.FindProperty("frontSuspHardCap");
        //rear wheel physical position and spring settings
        rearRightWheelCenter = serializedObject.FindProperty("rearRightWheelCenter");
        rearWheelRadius = serializedObject.FindProperty("rearWheelRadius");
        rearWheelRotationHoldsRatioOfSpeed = serializedObject.FindProperty("rearWheelRotationHoldsRatioOfSpeed");
        rearUse3DWheelPhysics = serializedObject.FindProperty("rearUse3DWheelPhysics");
        rearWheelInwardThickness = serializedObject.FindProperty("rearWheelInwardThickness");
        rearWheelOutwardThickness = serializedObject.FindProperty("rearWheelOutwardThickness");
        rearWheelShapeAccuracy = serializedObject.FindProperty("rearWheelShapeAccuracy");
        rearSuspDist = serializedObject.FindProperty("rearWheelSuspensionDistanceToLiftCarWeight");
        rearSuspHardCap = serializedObject.FindProperty("rearSuspHardCap");
        //spring context
        springsByDefaultGravity = serializedObject.FindProperty("springsByDefaultGravity");
        springsByOtherValue = serializedObject.FindProperty("springsByOtherValue");
        lateralAttackHeightLift  = serializedObject.FindProperty("lateralAttackHeightLift");
        longitudalAttackHeightLift = serializedObject.FindProperty("longitudalAttackHeightLift");
        //balance
        damping = serializedObject.FindProperty("damping");
        antiJumpSuspension = serializedObject.FindProperty("antiJumpSuspension");
        TurnUpwardsWithZeroWheels = serializedObject.FindProperty("TurnUpwardsWithZeroWheels");
        TurnUpwardsWithOneOrTwoWheels = serializedObject.FindProperty("TurnUpwardsWithOneOrTwoWheels");
        TurnUpwardsWithThreeOrFourWheels = serializedObject.FindProperty("TurnUpwardsWithThreeOrFourWheels");
        minFlightDurationForCorrection = serializedObject.FindProperty("minFlightDurationForCorrection");
        maxAngularVelCorrectionInAir = serializedObject.FindProperty("maxAngularVelCorrectionInAir");
        fixFlightRotationAtLeavingGround = serializedObject.FindProperty("fixFlightRotationAtLeavingGround");
        //anti roll bar
        frontAntiRollBar = serializedObject.FindProperty("frontAntiRollBar");
        rearAntiRollBar = serializedObject.FindProperty("rearAntiRollBar");
        //ground layer selection
        solidGroundLayer = serializedObject.FindProperty("solidGround");
        looseGroundLayer = serializedObject.FindProperty("looseGround");
        //grip relation to vertical force
        scaleGripWithSpringCompression = serializedObject.FindProperty("scaleGripWithSpringCompression");
        scaleGripWithDampingCompression = serializedObject.FindProperty("scaleGripWithDampingCompression");
        spreadGripFromNormalForceOnAllWheels = serializedObject.FindProperty("spreadGripFromNormalForceOnAllWheels");
        //grip settings coarse
        sameGripSettingsForAllWheels = serializedObject.FindProperty("sameGripSettingsForRearWheel");
        endlessLengthwiseGrip = serializedObject.FindProperty("endlessLengthwiseGrip");
        endlessSidewaysGrip = serializedObject.FindProperty("endlessSidewaysGrip");
        //grip settings direction dependence scaler
        lengthwiseGripAffectedBySidewaysSlip = serializedObject.FindProperty("lengthwiseGripAffectedBySidewaysSlip");
        sidewaysGripAffectedByLengthwiseSlip = serializedObject.FindProperty("sidewaysGripAffectedByLengthwiseSlip");
        //grip settings fine
        FwSgLengthwiseGrip  = serializedObject.FindProperty("FwSgLengthwiseGrip");
        RwSgLengthwiseGrip = serializedObject.FindProperty("RwSgLengthwiseGrip");
        FwSgSidewaysGrip = serializedObject.FindProperty("FwSgSidewaysGrip");
        RwSgSidewaysGrip = serializedObject.FindProperty("RwSgSidewaysGrip");
        FwLgLengthwiseGrip = serializedObject.FindProperty("FwLgLengthwiseGrip");
        RwLgLengthwiseGrip = serializedObject.FindProperty("RwLgLengthwiseGrip");
        FwLgSidewaysGrip = serializedObject.FindProperty("FwLgSidewaysGrip");
        RwLgSidewaysGrip = serializedObject.FindProperty("RwLgSidewaysGrip");
        useSameGripMultipliersAsSolidGround = serializedObject.FindProperty("useSameGripMultipliersAsSolidGround");

        //steering
        ackermanSteering = serializedObject.FindProperty("ackermanSteering");
        maxSteerChangePerSecond = serializedObject.FindProperty("maxSteerChangePerSecond");
        frontSteerAtZeroSpeed = serializedObject.FindProperty("frontSteerAtZeroSpeed");
        frontSteerAt30MS = serializedObject.FindProperty("frontSteerAt30MS");
        rearSteerAtZeroSpeed = serializedObject.FindProperty("rearSteerAtZeroSpeed");
        rearSteerAt30MS = serializedObject.FindProperty("rearSteerAt30MS");
        //steering balancing
        frontSteerTowardsSlide = serializedObject.FindProperty("frontSteerTowardsSlide");
        maxFrontAngleTowardsSlide = serializedObject.FindProperty("maxFrontAngleTowardsSlide");
        rearSteerTowardsSlide = serializedObject.FindProperty("rearSteerTowardsSlide");
        maxRearAngleTowardsSlide = serializedObject.FindProperty("maxRearAngleTowardsSlide");
        //arcady steering
        disableNaturalSteering = serializedObject.FindProperty("disableNaturalSteering");
        maxAnglesPSChangeFromArcadySteering = serializedObject.FindProperty("maxAnglesPSChangeAtArcadySteering");
        //air steering
        maxAnglesPSChangeAtAirSteering = serializedObject.FindProperty("maxAnglesPSChangeAtAirSteering");

        //input secttion
        breakAtOtherDirectionInput = serializedObject.FindProperty("breakAtOtherDirectionInput");
        ThrottleKey = serializedObject.FindProperty("ThrottleKey");
        BackwardThrottleKey = serializedObject.FindProperty("BackwardThrottleKey");
        SteerLeftKey = serializedObject.FindProperty("SteerLeftKey");
        SteerRightKey = serializedObject.FindProperty("SteerRightKey");
        BreakKey = serializedObject.FindProperty("BreakKey");
        HandbreakKey = serializedObject.FindProperty("HandbreakKey");
        GearShiftKey = serializedObject.FindProperty("GearShiftKey");
        GearShiftUpKey = serializedObject.FindProperty("GearShiftUpKey");
        GearShiftDownKey = serializedObject.FindProperty("GearShiftDownKey");
        NitroKey = serializedObject.FindProperty("NitroKey");

        //audio section
        audioForEngine = serializedObject.FindProperty("audioForEngine");
        deltaPitchAtLowGearSpeed = serializedObject.FindProperty("deltaPitchAtLowGearSpeed");
        deltaPitchAtHighGearSpeed = serializedObject.FindProperty("deltaPitchAtHighGearSpeed");
        amplifyDeltaPitchPerGear = serializedObject.FindProperty("amplifyDeltaPitchPerGear");
        volumeDropOffAtGearShift = serializedObject.FindProperty("volumeDropOffAtGearShift");
        volumeScaleWithThrottleInput = serializedObject.FindProperty("volumeScaleWithThrottleInput");
        volumeDropOffAtLowGearEffectiveness = serializedObject.FindProperty("volumeDropOffAtLowGearEffectiveness");

        //gui section
        sliderToShowShift = serializedObject.FindProperty("sliderToShowShift");
        sliderToShowClutch = serializedObject.FindProperty("sliderToShowClutch");
        guiGear = serializedObject.FindProperty("guiGear");
        guiSpeed = serializedObject.FindProperty("guiSpeed");

        //visualisation section
        visualWheels = car.visualWheels;

            
        placeOnPhysicalWheel = serializedObject.FindProperty("placeOnPhysicalWheel");




        carGizmos = target.GetComponentInChildren<CarGizmos>();
        
    }

    public override void OnInspectorGUI()
    {
        float currentH = 0;
        int sectionCount = 0;
        bool isRepainting = (Event.current.type == EventType.Repaint);

        serializedObject.Update();
        car.SpeedupCurve.ValidateCurve();

        Color darkerThanWhite = new Color(1f, 1f, 0.85f);

        //select Speed Unit
        GUILayout.Label("In which Unit do you measure Speed?", new GUIStyle(EditorStyles.centeredGreyMiniLabel));
        EditorGUILayout.BeginHorizontal();
        {
            GUILayout.Label("", GUILayout.Width(10));
            var oldColor0 = GUI.backgroundColor;
            if (speedUnit==0) GUI.backgroundColor = colorForPerformance;
            bool switchToMS = GUILayout.Button("m/s");
            if (switchToMS) speedUnit = 0;
            GUI.backgroundColor = speedUnit==1 ? colorForPerformance : oldColor0;
            bool switchToKMH = GUILayout.Button("km/h");
            if (switchToKMH) speedUnit = 1;
            GUI.backgroundColor = speedUnit == 2 ? colorForPerformance : oldColor0;
            bool switchToMPH = GUILayout.Button("mph");
            if (switchToMPH) speedUnit = 2;
            GUI.backgroundColor = oldColor0;
        }
        EditorGUILayout.EndHorizontal();
        currentH += lineH * 2 + 3;
        if (speedUnit == 0)
        {
            speedUnitScaler = 1;
            speedUnitWord = "m/s";
        }
        else if (speedUnit == 1)
        {
            speedUnitScaler = 3.6f;
            speedUnitWord = "km/h";
        }
        else
        {
            speedUnitScaler = 2.2369362921f;
            speedUnitWord = "mph";
        }
        





        //GUI.DrawTexture(new Rect(0, 50, Screen.width, 150), tex0, ScaleMode.StretchToFill);

        //acceleration settings
        GUI.DrawTexture(new Rect(0, currentH, Screen.width, sectionHeights[sectionCount]), GetAlternatingBackgroundTex(sectionCount), ScaleMode.StretchToFill);
        GUILayout.BeginHorizontal();
        showAccelerationSettings = EditorGUILayout.Foldout(showAccelerationSettings, "Acceleration");
        GUILayout.Label("", GUILayout.Width(20));
        string accelerationInformation = ("The car reaches " + car.SpeedupCurve.topSpeed*speedUnitScaler +  speedUnitWord+" in " + car.SpeedupCurve.timeNeeded + " seconds");
        GUILayout.Label(accelerationInformation, new GUIStyle(EditorStyles.centeredGreyMiniLabel));
        GUILayout.EndHorizontal();
        if (showAccelerationSettings)
        {
            EditorGUI.indentLevel++;
            car.SpeedupCurve.topSpeed = EditorGUILayout.FloatField("("+speedUnitWord+") Max Speed", car.SpeedupCurve.topSpeed*speedUnitScaler)/speedUnitScaler;
            ClassicUnitConversionFix(ref car.SpeedupCurve.topSpeed);
            car.SpeedupCurve.timeNeeded = EditorGUILayout.FloatField("(s) Time needed", car.SpeedupCurve.timeNeeded);
            EditorGUILayout.CurveField("Speed Curve", car.SpeedupCurve.curve, colorForRealism, new Rect(0, 0f, 1f, 1f));
            currentH += lineH*3 + 8;
            //rearOrFrontPowered.floatValue = 
            //EditorGUILayout.BeginHorizontal();
            //{

            //    //GUILayout.FlexibleSpace();
            //    EditorGUILayout.LabelField("(%) Rear", GUILayout.MaxWidth(75));
            //    rearOrFrontPowered.floatValue = 1 - EditorGUILayout.FloatField("", (1-rearOrFrontPowered.floatValue)*100, GUILayout.MaxWidth(50)) /100;
            //    rearOrFrontPowered.floatValue = EditorGUILayout.Slider("", rearOrFrontPowered.floatValue*100, 0, 100, GUILayout.MaxWidth(Screen.width-310)) /100;
            //    EditorGUILayout.LabelField("Front", GUILayout.MaxWidth(60));
            //    //EditorGUILayout.LabelField("When driving with ", new GUIStyle(EditorStyles.helpBox), GUILayout.MaxWidth(100));
            //    //pickedOrientationAirSlowdowm = EditorGUILayout.FloatField("", pickedOrientationAirSlowdowm * speedUnitScaler, GUILayout.MaxWidth(65)) / speedUnitScaler;
            //    //EditorGUILayout.LabelField(speedUnitWord + ", the air resistance slows it by ", new GUIStyle(EditorStyles.helpBox) { alignment = TextAnchor.MiddleLeft }, GUILayout.MaxWidth(182));
            //    //pickedAirSlowdownAtOrientation = EditorGUILayout.FloatField("", pickedAirSlowdownAtOrientation, GUILayout.MaxWidth(65));
            //    //EditorGUILayout.LabelField(" m/s^2 ", new GUIStyle(EditorStyles.helpBox) { alignment = TextAnchor.MiddleLeft }, GUILayout.MaxWidth(80));
            //    //GUILayout.FlexibleSpace();
            //}
            //EditorGUILayout.EndHorizontal();
            DrawRearFrontSlider(ref rearOrFrontPowered);
            accelerationRestrainToOptimiseGrip.floatValue = EditorGUILayout.Slider("(%) Restrain to optimise Grip", accelerationRestrainToOptimiseGrip.floatValue * 100, 0, 100) / 100;

            EditorGUI.indentLevel--;
        }
        currentH += lineH + 10;
        GUILayout.Space(10);

        //gear settings
        //GUI.DrawTexture(new Rect(0, sectionHeights[sectionCount], Screen.width, sectionHeights[++sectionCount]), GetAlternatingBackgroundTex(sectionCount), ScaleMode.StretchToFill);
        DrawNextSectionAndClosePreviouse(ref sectionCount);
        showGearSettings = EditorGUILayout.Foldout(showGearSettings, "Gears");
        currentH+= lineH;
        if (showGearSettings)
        {
            EditorGUI.indentLevel++;
            currentH += lineH + 4;
            int numberOfGearsBeforeChange = numberOfGears.intValue;
            EditorGUILayout.PropertyField(numberOfGears);
            if (numberOfGears.intValue > 16) numberOfGears.intValue = 16;
            if (numberOfGears.intValue > 0)
            {
                currentH += lineH*2;
                GUI.DrawTexture(new Rect(EditorGUI.indentLevel*30, subrectangles[0].from, Screen.width, subrectangles[0].to- subrectangles[0].from), GetAlternatingBackgroundTex(sectionCount+1), ScaleMode.StretchToFill);
                GUILayout.Label("Where in the Acceleration-Curve is the Gear's peak Performance? (Ratio within Acceleration Time)", new GUIStyle(EditorStyles.centeredGreyMiniLabel));            
            }
            car.ValidateGearSettings();
            if(numberOfGears.intValue > 0)
            {
                gearImpactOnAcceleration.floatValue = EditorGUILayout.Slider("Gear Impact on Acceleration", gearImpactOnAcceleration.floatValue, 0, 1);
                if (isRepainting) subrectangles[0].from = GUILayoutUtility.GetLastRect().yMax;
                GUI.Label(GUILayoutUtility.GetLastRect(), new GUIContent("", "Around 0.2 is a realistic behaviour. If you set this to 0.2, the car acceleration is multiplied by 1.2 at the perfectly fitting speed of a gear, but the acceleration is accordingly worse, the further away it is from its optimum, in a way that the total acceleration time is preserved, when you shift at the right moment. Set this to 0 if you only use gears for the soundsystem"));
                EditorGUI.indentLevel++;
                for (int i = 0; i < numberOfGears.intValue && i < car.gearIsBestAtRelSpeed.Count; i++)
                {

                    car.gearIsBestAtRelSpeed[i] = EditorGUILayout.Slider("Gear " + (i + 1) + ":", car.gearIsBestAtRelSpeed[i], 0, 1);
                    currentH += lineH + 2;
                }
                //gearRecTo = currentH;
                EditorGUI.indentLevel--;
                if (isRepainting) subrectangles[0].to = GUILayoutUtility.GetLastRect().yMax + 75;
                if (numberOfGears.intValue == numberOfGearsBeforeChange) //(if statement prevents calculations with unvalidated gears)
                {
                    GUI.DrawTexture(new Rect(Screen.width * 0.36f - 21, subrectangles[0].to-75, Screen.width * 0.44f - 59, 75), CreateGearTexture((int)(Screen.width * 0.44f - 52), 75), ScaleMode.ScaleAndCrop);
                }
                
                GUILayout.Space(85);
                //currentH += 85;

                GUILayout.Label("A Gear Shift takes " + (gearOutDuration.floatValue + shiftDuration.floatValue + gearInDuration.floatValue) + " Seconds in total", new GUIStyle(EditorStyles.centeredGreyMiniLabel));
                gearOutDuration.floatValue = EditorGUILayout.Slider("(s) Gear Out Duration", gearOutDuration.floatValue, 0, 0.9f);
                shiftDuration.floatValue = EditorGUILayout.Slider("(s) Shift Duration", shiftDuration.floatValue, 0, 0.9f);
                gearInDuration.floatValue = EditorGUILayout.Slider("(s) Gear In Duration", gearInDuration.floatValue, 0, 0.9f);
                float totalShiftingTime = gearOutDuration.floatValue + shiftDuration.floatValue + gearInDuration.floatValue;
                
                shiftingImpactOnAcceleration.floatValue = EditorGUILayout.Slider("(%) Shifting Impact On Acceleration", shiftingImpactOnAcceleration.floatValue*100, 0, 100)/100;
                GUI.Label(GUILayoutUtility.GetLastRect(), new GUIContent("", "100% is a realistic behaviour, where there is no acceleration when the gears are out while shifting. However you can set this value lower, so process of shifting does not impact the acceleration too much."));
                //shifting visualisation curve
                EditorGUILayout.CurveField("(Shift Visualisation:)", new AnimationCurve(new Keyframe[] {
                    new Keyframe(0, 1, 0, 0, 0, 0) ,
                    new Keyframe(gearOutDuration.floatValue, 1 - shiftingImpactOnAcceleration.floatValue, 0, 0, 0, 0) ,
                    new Keyframe(gearOutDuration.floatValue+shiftDuration.floatValue, 1 - shiftingImpactOnAcceleration.floatValue, 0, 0, 0, 0) ,
                    new Keyframe(gearOutDuration.floatValue + shiftDuration.floatValue+gearInDuration.floatValue, 1, 0, 0, 0, 0) ,
                    new Keyframe(totalShiftingTime > 1 ? totalShiftingTime : 1, 1, 0, 0, 0, 0) ,
                }), Color.white, new Rect(0, 0f, totalShiftingTime > 1 ? totalShiftingTime : 1, 1f));

                EditorGUILayout.PropertyField(gearShiftMode);
                //currentH += lineH * 7 + 5;
            }
            EditorGUI.indentLevel--;
            
        }
        GUILayout.Space(10);


        //Deceleration
        DrawNextSectionAndClosePreviouse(ref sectionCount);
        showBreakSettings = EditorGUILayout.Foldout(showBreakSettings, "Deceleration");
        if (showBreakSettings)
        {
            //braking
            EditorGUILayout.LabelField("Braking", EditorStyles.miniBoldLabel);
            breakMaxSlowdown.floatValue = EditorGUILayout.FloatField("break deceleration", breakMaxSlowdown.floatValue);
            DrawRearFrontSlider(ref breakAppliedToFrontWheels);
            breakRestrainToOptimiseGrip.floatValue = EditorGUILayout.Slider("(%) Restrain to optimise Grip", breakRestrainToOptimiseGrip.floatValue * 100, 0, 100) / 100;
            breakLimitedByGrip.floatValue = EditorGUILayout.Slider("(%) Breake limited by Grip", breakLimitedByGrip.floatValue * 100, 0, 100) / 100;
            GUILayout.Space(10);

            //handbrake
            EditorGUILayout.LabelField("Handbreak", EditorStyles.miniBoldLabel);
            handBreakMaxSlowdown.floatValue = EditorGUILayout.FloatField("handbreak deceleration", handBreakMaxSlowdown.floatValue);
            DrawRearFrontSlider(ref handBreakAppliedToFrontWheels);
            handBreakRestrainToOptimiseGrip.floatValue = EditorGUILayout.Slider("(%) Restrain to optimise Grip", handBreakRestrainToOptimiseGrip.floatValue * 100, 0, 100) / 100;
            handBreakLimitedByGrip.floatValue = EditorGUILayout.Slider("(%) Breake limited by Grip", handBreakLimitedByGrip.floatValue * 100, 0, 100) / 100;
            GUILayout.Space(10);

            //Permanent deceleration factors
            EditorGUILayout.LabelField("Permanent deceleration factors", EditorStyles.miniBoldLabel);
            EditorGUILayout.BeginHorizontal();
            {
                GUILayout.FlexibleSpace();

                EditorGUILayout.LabelField("When driving with ", new GUIStyle(EditorStyles.helpBox), GUILayout.MaxWidth(100));
                pickedOrientationAirSlowdowm = EditorGUILayout.FloatField("", pickedOrientationAirSlowdowm * speedUnitScaler, GUILayout.MaxWidth(65))/speedUnitScaler;
                EditorGUILayout.LabelField(speedUnitWord + ", the air resistance slows it by ", new GUIStyle(EditorStyles.helpBox) { alignment = TextAnchor.MiddleLeft }, GUILayout.MaxWidth(182));
                pickedAirSlowdownAtOrientation = EditorGUILayout.FloatField("", pickedAirSlowdownAtOrientation, GUILayout.MaxWidth(65));
                EditorGUILayout.LabelField(" m/s^2 ", new GUIStyle(EditorStyles.helpBox) { alignment = TextAnchor.MiddleLeft }, GUILayout.MaxWidth(80));
                GUILayout.FlexibleSpace();
            }
            EditorGUILayout.EndHorizontal();
            // (This is just a Tooltip to explain the air resistence)
            slowByAirResistanceAt30ms.floatValue = pickedAirSlowdownAtOrientation * Mathf.Pow(30, airResistanceExponent.floatValue) / Mathf.Pow(pickedOrientationAirSlowdowm, airResistanceExponent.floatValue);
            GUI.Label(GUILayoutUtility.GetLastRect(), new GUIContent("", "The Slowdown from Air Resistance rises by the exponent of "+airResistanceExponent.floatValue+" with speed. " +
                "With you current Settings the Air Resistance slows the car by" + Environment.NewLine
            + (pickedAirSlowdownAtOrientation * Mathf.Pow(15, airResistanceExponent.floatValue) / Mathf.Pow(pickedOrientationAirSlowdowm, airResistanceExponent.floatValue)).ToString("F2") + " m/s^2 at " + (15 * speedUnitScaler).ToString("F2") + speedUnitWord + ", " + Environment.NewLine
            + slowByAirResistanceAt30ms.floatValue.ToString("F2") + " m/s^2 at " + (30 * speedUnitScaler).ToString("F2") + speedUnitWord+ " and "+ Environment.NewLine +
             (pickedAirSlowdownAtOrientation * Mathf.Pow(60, airResistanceExponent.floatValue) / Mathf.Pow(pickedOrientationAirSlowdowm, airResistanceExponent.floatValue)).ToString("F2") + " m/s^2 at " + (60 * speedUnitScaler).ToString("F2") + speedUnitWord));
            
            airResistanceExponent.floatValue = EditorGUILayout.Slider("Air Resistance Exponent", airResistanceExponent.floatValue, 1, 2);
            GUI.Label(GUILayoutUtility.GetLastRect(), new GUIContent("", "Exponent should be 2 for Realistic Air Resistance Behaviour! (F = Area * Const * Speed^2) If you pick 1 the slowdown scales lineary with speed"));
            slowByRollFriction.floatValue = EditorGUILayout.FloatField("Slow by Rolling Friction", slowByRollFriction.floatValue);
        }
        GUILayout.Space(10);

        



        //front wheel Position and Spring Settings
        DrawNextSectionAndClosePreviouse(ref sectionCount);
        showWheelSettings.boolValue = EditorGUILayout.Foldout(showWheelSettings.boolValue, "Wheels & Suspension");
        if (showWheelSettings.boolValue)
        {
            //front wheels & suspension
            EditorGUILayout.LabelField("Front Wheels", EditorStyles.miniBoldLabel);
            EditorGUI.indentLevel++;
            frontRightWheelCenter.vector3Value  = EditorGUILayout.Vector3Field("PositionR", frontRightWheelCenter.vector3Value);
            frontWheelRadius.floatValue         = EditorGUILayout.FloatField("Radius", frontWheelRadius.floatValue);
            frontWheelRotationHoldsRatioOfSpeed.floatValue = EditorGUILayout.Slider("Holds energetic Ratio of Speed as Rotation", frontWheelRotationHoldsRatioOfSpeed.floatValue * 100f, 1, 15)/100f;
            GUI.Label(GUILayoutUtility.GetLastRect(), new GUIContent("", "% of the kinectic energy used to rotate one front wheel. When a Car moves foward, not all kinetic energy is foward movement. Each wheel stores around 2.5% of the car's total kinetic energy as rotation. Overproportial wheels like thouse of monstertrucks can take up to 15% of the vehicle's total kinetic energy each." +
                " Why is this Ratio relevant? Example: If wheels spin very fast in the air and the car hits the ground again, excess wheel speed is converted into foward movement accordingly. Low Ratios on the other hand use less of the motor power to adapt the current wheel spin"));
            frontSuspDist.floatValue            = EditorGUILayout.FloatField("Loose Spring Range", frontSuspDist.floatValue);
            frontSuspHardCap.floatValue         = EditorGUILayout.FloatField("Upward Suspension Stop", frontSuspHardCap.floatValue);

            var oldColor = GUI.backgroundColor;
            EditorGUILayout.BeginHorizontal();
            {
                GUILayout.Label("", GUILayout.Width(10));
                
                if (!frontUse3DWheelPhysics.boolValue) GUI.backgroundColor = colorForPerformance;
                bool switchToRayPhysics = GUILayout.Button("Use performant Raycast");
                if (switchToRayPhysics) frontUse3DWheelPhysics.boolValue = false;
                GUI.backgroundColor = frontUse3DWheelPhysics.boolValue ? GUI.backgroundColor = colorForAccuracy : oldColor;
                bool switchTo3DWheel = GUILayout.Button("3D Wheel Collider");
                if (switchTo3DWheel) frontUse3DWheelPhysics.boolValue = true;
            }
            EditorGUILayout.EndHorizontal();

            if (frontUse3DWheelPhysics.boolValue)
            {
                EditorGUI.indentLevel++;
                frontWheelInwardThickness.floatValue    = EditorGUILayout.FloatField("Inward Thickness", frontWheelInwardThickness.floatValue);
                frontWheelOutwardThickness.floatValue   = EditorGUILayout.FloatField("Outward Thickness", frontWheelOutwardThickness.floatValue);
                frontWheelShapeAccuracy.intValue        = EditorGUILayout.IntSlider("Shape Accuracy", frontWheelShapeAccuracy.intValue, 4, 12);
                EditorGUI.indentLevel--;

            }
            GUI.backgroundColor = oldColor;
            GUILayout.Space(5);

            //rear wheels & suspension
            EditorGUILayout.LabelField("Rear Wheels", EditorStyles.miniBoldLabel);
            rearRightWheelCenter.vector3Value = EditorGUILayout.Vector3Field("PositionR", rearRightWheelCenter.vector3Value);
            rearWheelRadius.floatValue = EditorGUILayout.FloatField("Radius", rearWheelRadius.floatValue);
            rearWheelRotationHoldsRatioOfSpeed.floatValue = EditorGUILayout.Slider("Holds energetic Ratio of Speed as Rotation", rearWheelRotationHoldsRatioOfSpeed.floatValue * 100f, 1, 15) / 100f;
            GUI.Label(GUILayoutUtility.GetLastRect(), new GUIContent("", "% of the kinectic energy used to rotate one rear wheel. When a Car moves foward, not all kinetic energy is foward movement. Each wheel stores around 2.5% of the car's total kinetic energy as rotation. Overproportial wheels like thouse of monstertrucks can take up to 15% of the vehicle's total kinetic energy each." +
                " Why is this Ratio relevant? Example: If wheels spin very fast in the air and the car hits the ground again, excess wheel speed is converted into foward movement accordingly. Low Ratios on the other hand use less of the motor power to adapt the current wheel spin"));
            rearSuspDist.floatValue = EditorGUILayout.FloatField("Loose Spring Range", rearSuspDist.floatValue);
            //rearDamping.floatValue=  EditorGUILayout.Slider("Damping", rearDamping.floatValue, 0, 10);
            rearSuspHardCap.floatValue = EditorGUILayout.FloatField("Upward Suspension Stop", rearSuspHardCap.floatValue);

            EditorGUILayout.BeginHorizontal();
            {
                GUILayout.Label("", GUILayout.Width(10));
                if (!rearUse3DWheelPhysics.boolValue) GUI.backgroundColor = colorForPerformance;
                bool switchToRayPhysics = GUILayout.Button("Use performant Raycast");
                if (switchToRayPhysics) rearUse3DWheelPhysics.boolValue = false;
                GUI.backgroundColor = rearUse3DWheelPhysics.boolValue ? GUI.backgroundColor = colorForAccuracy : oldColor;
                bool switchTo3DWheel = GUILayout.Button("3D Wheel Collider");
                if (switchTo3DWheel) rearUse3DWheelPhysics.boolValue = true;
            }
            EditorGUILayout.EndHorizontal();

            if (rearUse3DWheelPhysics.boolValue)
            {
                EditorGUI.indentLevel++;
                rearWheelInwardThickness.floatValue = EditorGUILayout.FloatField("Inward Thickness", rearWheelInwardThickness.floatValue);
                rearWheelOutwardThickness.floatValue = EditorGUILayout.FloatField("Outward Thickness", rearWheelOutwardThickness.floatValue);
                rearWheelShapeAccuracy.intValue = EditorGUILayout.IntSlider("Shape Accuracy", rearWheelShapeAccuracy.intValue, 4, 12);
                EditorGUI.indentLevel--;

            }
            
            GUI.backgroundColor = oldColor;
            EditorGUI.indentLevel--;

            GUILayout.Space(15);

            //DrawNextSectionAndClosePreviouse(ref sectionCount);
            GUILayout.Label("For which Gravity did you adjust the Suspension Springs?", new GUIStyle(EditorStyles.centeredGreyMiniLabel));
            var previousColor = GUI.backgroundColor;
            EditorGUILayout.BeginHorizontal();
            {
                if (springsByDefaultGravity.boolValue) GUI.backgroundColor = colorForPerformance;
                bool switchToDefaultGravity = GUILayout.Button("Szene Default Gravity");
                if (switchToDefaultGravity) springsByDefaultGravity.boolValue = true;
                GUI.backgroundColor = springsByDefaultGravity.boolValue ? previousColor : colorForPerformance;
                bool switchToCustomGravity = GUILayout.Button("Custom Gravity Value");
                if (switchToCustomGravity) springsByDefaultGravity.boolValue = false;
            }
            EditorGUILayout.EndHorizontal();
            EditorGUI.indentLevel++;
            EditorGUILayout.BeginHorizontal();
            {
                GUILayout.FlexibleSpace();
                if (springsByDefaultGravity.boolValue)
                {
                    EditorGUILayout.LabelField("Answer: I Adjusted Spings for " + Physics.gravity.magnitude + "m/s^2 Gravity", new GUIStyle(EditorStyles.helpBox), GUILayout.MaxWidth(320));
                }
                else
                {
                    EditorGUILayout.LabelField("Answer: I Adjusted Spings for", new GUIStyle(EditorStyles.helpBox), GUILayout.MaxWidth(160));
                    springsByOtherValue.floatValue = EditorGUILayout.FloatField("", springsByOtherValue.floatValue, GUILayout.MaxWidth(65));
                    EditorGUILayout.LabelField("m/s^2 Gravity", new GUIStyle(EditorStyles.helpBox) { alignment = TextAnchor.MiddleLeft }, GUILayout.MaxWidth(80));
                }
                GUILayout.FlexibleSpace();
            }
            EditorGUILayout.EndHorizontal();
            GUI.backgroundColor = previousColor;
            EditorGUI.indentLevel--;
        }
        
        GUILayout.Space(10);


        //BALANCE
        DrawNextSectionAndClosePreviouse(ref sectionCount);
        showBalanceSettings = EditorGUILayout.Foldout(showBalanceSettings, "Balance");
        if (showBalanceSettings)
        {
            damping.floatValue = EditorGUILayout.Slider("Damping", damping.floatValue, 0, 10);
            antiJumpSuspension.floatValue = EditorGUILayout.Slider("(%) Anti-Jump Suspension", antiJumpSuspension.floatValue * 100, 0, 100) / 100;
            GUI.Label(GUILayoutUtility.GetLastRect(), new GUIContent("", "When a wheel drives fast over a bump it might throw the car upwards on that side. With anti-jump-suspension a spring looses % of its power, while the car is already moving upward on that local position, preventing that behaviour"));
            frontAntiRollBar.floatValue = EditorGUILayout.Slider("(%) Front Anti-Roll Bar", frontAntiRollBar.floatValue * 100, 0, 100) / 100;
            rearAntiRollBar.floatValue = EditorGUILayout.Slider("(%) Rear Anti-Roll Bar", rearAntiRollBar.floatValue * 100, 0, 100) / 100;
            lateralAttackHeightLift.floatValue = EditorGUILayout.Slider("(%) Lift lateral-attackpoint", lateralAttackHeightLift.floatValue * 100, 0, 100) / 100;
            GUI.Label(GUILayoutUtility.GetLastRect(), new GUIContent("", "0% is a realistic behavior, where sideward grip applies force at ground height (potentionally flipping the car sideward), whereas 100% prevents this behaviour by lifting the attackpoint to the center of mass's height")); //tooltip to previous https://www.reddit.com/r/Unity3D/comments/45bjwc/tooltip_on_custom_inspectorproperties/ by againey
            longitudalAttackHeightLift.floatValue = EditorGUILayout.Slider("(%) Lift longitudal-attackpoint", longitudalAttackHeightLift.floatValue * 100, 0, 100) / 100;
            GUI.Label(GUILayoutUtility.GetLastRect(), new GUIContent("", "0% is a realistic behavior, where grip of accelerating and breaking applies force at ground height (potentionally flipping the car foward/backward), whereas 100% prevents this behaviour by lifting the attackpoint to the center of mass's height"));
            GUILayout.Label("Define how/if the car turns up again when it landed bad", new GUIStyle(EditorStyles.centeredGreyMiniLabel));
            EditorGUILayout.PropertyField(TurnUpwardsWithZeroWheels);
            EditorGUILayout.PropertyField(TurnUpwardsWithOneOrTwoWheels);
            EditorGUILayout.PropertyField(TurnUpwardsWithThreeOrFourWheels);
            EditorGUILayout.LabelField("Correct Air Rotation to land straigth:", EditorStyles.miniBoldLabel);
            minFlightDurationForCorrection.floatValue = EditorGUILayout.FloatField("(s) Start Correction after flight Duration:", minFlightDurationForCorrection.floatValue);
            maxAngularVelCorrectionInAir.floatValue = EditorGUILayout.FloatField("(°/s^2) Max Rotation speed change:", maxAngularVelCorrectionInAir.floatValue);
            EditorGUILayout.PropertyField(fixFlightRotationAtLeavingGround);
        }
        GUILayout.Space(15);

        //GRIP - NORMAL FORCE RELATION
        DrawNextSectionAndClosePreviouse(ref sectionCount);
        EditorGUILayout.LabelField("Each Wheel's Grip is scaled (%) by what?", new GUIStyle(EditorStyles.centeredGreyMiniLabel));
        scaleGripWithSpringCompression.floatValue = EditorGUILayout.Slider("By its Spring Compression", scaleGripWithSpringCompression.floatValue * 100, 0, 100) / 100;
        scaleGripWithDampingCompression.floatValue = EditorGUILayout.Slider("By its Damping Compression", scaleGripWithDampingCompression.floatValue * 100, 0, 100) / 100;
        spreadGripFromNormalForceOnAllWheels.floatValue = EditorGUILayout.Slider("By other Wheels' Compressions", spreadGripFromNormalForceOnAllWheels.floatValue * 100, 0, 100) / 100;
        GUILayout.Space(15);



        DrawNextSectionAndClosePreviouse(ref sectionCount);
        EditorGUILayout.BeginHorizontal();
        {
            var prevColor01 = GUI.backgroundColor;
            if (!sameGripSettingsForAllWheels.boolValue) GUI.backgroundColor = colorForRealism;
            bool switchToDynamicGrip = GUILayout.Button("Different-", GUILayout.Width(70));
            if (switchToDynamicGrip) sameGripSettingsForAllWheels.boolValue = false;
            GUI.backgroundColor = sameGripSettingsForAllWheels.boolValue ? GUI.backgroundColor = colorForArcade : prevColor01;
            bool switchToEndlessGrip = GUILayout.Button("Same-", GUILayout.Width(70));
            if (switchToEndlessGrip) sameGripSettingsForAllWheels.boolValue = true;
            GUI.backgroundColor = sameGripSettingsForAllWheels.boolValue ? GUI.backgroundColor = prevColor01 : colorForRealism;
            EditorGUILayout.LabelField("-Grip Settings for Front and Rear Wheels");
            GUI.backgroundColor = prevColor01;
        }
        EditorGUILayout.EndHorizontal();
        //Endless Grip Buttons
        EditorGUILayout.BeginHorizontal();
        {
            var prevColor2 = GUI.backgroundColor;
            if (!endlessLengthwiseGrip.boolValue) GUI.backgroundColor = colorForRealism;
            bool switchToDynamicGrip = GUILayout.Button("Dynamic-");
            if (switchToDynamicGrip) endlessLengthwiseGrip.boolValue = false;
            GUI.backgroundColor = endlessLengthwiseGrip.boolValue ? GUI.backgroundColor = colorForArcade : prevColor2;
            bool switchToEndlessGrip = GUILayout.Button("Endless-");
            if (switchToEndlessGrip) endlessLengthwiseGrip.boolValue = true;
            GUI.backgroundColor = endlessLengthwiseGrip.boolValue ? GUI.backgroundColor = prevColor2 : colorForRealism;
            EditorGUILayout.LabelField("-Lengthwise Grip", GUILayout.Width(110));
            GUI.backgroundColor = prevColor2;
        }
        EditorGUILayout.EndHorizontal();
        EditorGUILayout.BeginHorizontal();
        {
            var prevColor3 = GUI.backgroundColor;
            if (!endlessSidewaysGrip.boolValue) GUI.backgroundColor = colorForRealism;
            bool switchToDynamicGrip = GUILayout.Button("Dynamic-");
            if (switchToDynamicGrip) endlessSidewaysGrip.boolValue = false;
            GUI.backgroundColor = endlessSidewaysGrip.boolValue ? GUI.backgroundColor = colorForArcade : prevColor3;
            bool switchToEndlessGrip = GUILayout.Button("Endless-");
            if (switchToEndlessGrip) endlessSidewaysGrip.boolValue = true;
            GUI.backgroundColor = endlessSidewaysGrip.boolValue ? GUI.backgroundColor = prevColor3 : colorForRealism;
            EditorGUILayout.LabelField("-Sideways Grip", GUILayout.Width(110));
            GUI.backgroundColor = prevColor3;
        }
        EditorGUILayout.EndHorizontal();

        if (!endlessLengthwiseGrip.boolValue) lengthwiseGripAffectedBySidewaysSlip.floatValue = EditorGUILayout.Slider("lengthwise Grip affected by sideways Slip", lengthwiseGripAffectedBySidewaysSlip.floatValue * 100, 0, 100) / 100;
        if (!endlessSidewaysGrip.boolValue) sidewaysGripAffectedByLengthwiseSlip.floatValue = EditorGUILayout.Slider("sideways Grip affected by lengthwise Slip", sidewaysGripAffectedByLengthwiseSlip.floatValue * 100, 0, 100) / 100;

        //Solid Ground Grip Settings
        EditorGUILayout.LabelField("Grip Settings on Solid Ground", EditorStyles.miniBoldLabel);
        EditorGUILayout.PropertyField(solidGroundLayer);
        car.FrontGripOnSolidGround.ValidateCurve();
        EditorGUILayout.CurveField("Grip at SlidingSpeed", car.FrontGripOnSolidGround.curve, colorForRealism, new Rect(0, 0f, 2f, 1f));
        EditorGUI.indentLevel++;
        if(!endlessLengthwiseGrip.boolValue)
        {
            if (sameGripSettingsForAllWheels.boolValue)
            {
                FwSgLengthwiseGrip.floatValue = EditorGUILayout.FloatField("Lengthwise Grip", FwSgLengthwiseGrip.floatValue);
            }
            else
            {
                EditorGUILayout.BeginHorizontal();
                {
                    EditorGUILayout.LabelField("Lengthwise Grip     Fw:", GUILayout.Width(150));
                    FwSgLengthwiseGrip.floatValue = EditorGUILayout.FloatField("", FwSgLengthwiseGrip.floatValue, GUILayout.MaxWidth(70));
                    EditorGUILayout.LabelField("   Rw:", GUILayout.Width(50));
                    RwSgLengthwiseGrip.floatValue = EditorGUILayout.FloatField("", RwSgLengthwiseGrip.floatValue, GUILayout.MaxWidth(70));
                }
                EditorGUILayout.EndHorizontal();
            }         
        }
        if (!endlessSidewaysGrip.boolValue)
        {
            if (sameGripSettingsForAllWheels.boolValue)
            {
                FwSgSidewaysGrip.floatValue = EditorGUILayout.FloatField("Sideways Grip", FwSgSidewaysGrip.floatValue);
            }
            else
            {               
                EditorGUILayout.BeginHorizontal();
                {
                    EditorGUILayout.LabelField("Sideways Grip        Fw:", GUILayout.Width(150));
                    FwSgSidewaysGrip.floatValue = EditorGUILayout.FloatField("", FwSgSidewaysGrip.floatValue, GUILayout.MaxWidth(70));
                    EditorGUILayout.LabelField("   Rw:", GUILayout.Width(50));
                    RwSgSidewaysGrip.floatValue = EditorGUILayout.FloatField("", RwSgSidewaysGrip.floatValue, GUILayout.MaxWidth(70));
                }
                EditorGUILayout.EndHorizontal();
            }
        }
        EditorGUI.indentLevel--;

        //Loose Ground Grip Settings
        EditorGUILayout.LabelField("Grip Settings on Solid Loose", EditorStyles.miniBoldLabel);
        EditorGUILayout.PropertyField(looseGroundLayer);
        car.FrontGripOnLooseGround.ValidateCurve();
        EditorGUILayout.CurveField("Grip at SlidingSpeed", car.FrontGripOnLooseGround.curve, colorForRealism, new Rect(0, 0f, 2f, 1f));
        if (!endlessLengthwiseGrip.boolValue || !endlessSidewaysGrip.boolValue)
        {
            useSameGripMultipliersAsSolidGround.boolValue = EditorGUILayout.Toggle("useSameGripMultipliersAsSolidGround", useSameGripMultipliersAsSolidGround.boolValue);
            if (!useSameGripMultipliersAsSolidGround.boolValue)
            {
                EditorGUI.indentLevel++;
                if (!endlessLengthwiseGrip.boolValue)
                {
                    if (sameGripSettingsForAllWheels.boolValue)
                    {
                        FwLgLengthwiseGrip.floatValue = EditorGUILayout.FloatField("Lengthwise Grip", FwLgLengthwiseGrip.floatValue);
                    }
                    else
                    {
                        EditorGUILayout.BeginHorizontal();
                        {
                            EditorGUILayout.LabelField("Lengthwise Grip     Fw:", GUILayout.Width(150));
                            FwLgLengthwiseGrip.floatValue = EditorGUILayout.FloatField("", FwLgLengthwiseGrip.floatValue, GUILayout.MaxWidth(70));
                            EditorGUILayout.LabelField("   Rw:", GUILayout.Width(50));
                            RwLgLengthwiseGrip.floatValue = EditorGUILayout.FloatField("", RwLgLengthwiseGrip.floatValue, GUILayout.MaxWidth(70));
                        }
                        EditorGUILayout.EndHorizontal();
                    }
                }
                if (!endlessSidewaysGrip.boolValue)
                {
                    if (sameGripSettingsForAllWheels.boolValue)
                    {
                        FwLgSidewaysGrip.floatValue = EditorGUILayout.FloatField("Sideways Grip", FwLgSidewaysGrip.floatValue);
                    }
                    else
                    {
                        EditorGUILayout.BeginHorizontal();
                        {
                            EditorGUILayout.LabelField("Sideways Grip        Fw:", GUILayout.Width(150));
                            FwLgSidewaysGrip.floatValue = EditorGUILayout.FloatField("", FwLgSidewaysGrip.floatValue, GUILayout.MaxWidth(70));
                            EditorGUILayout.LabelField("   Rw:", GUILayout.Width(50));
                            RwLgSidewaysGrip.floatValue = EditorGUILayout.FloatField("", RwLgSidewaysGrip.floatValue, GUILayout.MaxWidth(70));
                        }
                        EditorGUILayout.EndHorizontal();
                    }
                }
                EditorGUI.indentLevel--;
            }
        }
        GUILayout.Space(15);




        //STEERING SETTINGS
        DrawNextSectionAndClosePreviouse(ref sectionCount);
        EditorGUILayout.LabelField("Steering Settings", EditorStyles.boldLabel);
        ackermanSteering.floatValue = EditorGUILayout.Slider("(%) Ackerman Steering", ackermanSteering.floatValue * 100, -100, 100) / 100;
        GUI.Label(GUILayoutUtility.GetLastRect(), new GUIContent("", "ackerman schmackermann!"));
        maxSteerChangePerSecond.floatValue = EditorGUILayout.Slider("(%/s) Steering Change Per Second", maxSteerChangePerSecond.floatValue * 100, 0, 2000) / 100;
        GUI.Label(GUILayoutUtility.GetLastRect(), new GUIContent("", "The steering angle is always changed continiously. With this variable on 1000%-Change-Per-Second it takes 0.1 Seconds of Steering-Input to change the Wheels from a neutral state (0°) to the maximum Steering Angle"));
        frontSteerAtZeroSpeed.floatValue = EditorGUILayout.Slider("(°) Max Front Steer", frontSteerAtZeroSpeed.floatValue, 0, 45);
        GUI.Label(GUILayoutUtility.GetLastRect(), new GUIContent("", "How many Degrees can the front wheels steer, while the car is standing?"));
        frontSteerAt30MS.floatValue = EditorGUILayout.Slider("(°) MFS at "+(30*speedUnitScaler).ToString("F2") + " "+speedUnitWord, frontSteerAt30MS.floatValue, 0, 45);
        float frontSteerDifFor30ms = frontSteerAt30MS.floatValue - frontSteerAtZeroSpeed.floatValue;
        GUI.Label(GUILayoutUtility.GetLastRect(), new GUIContent("", "How many Degrees can the front wheels steer, while driving at "+(30*speedUnitScaler).ToString("F2") + " " +speedUnitWord+ "?"
            + " (It is recommendet to set a lower Steering Angle for higher Speed. With you current Settings the front wheels steer " + Environment.NewLine
            + (frontSteerAt30MS.floatValue + frontSteerDifFor30ms * (2* Mathf.Pow(2,-1/0.5f)-1)) +"° at "+(15*speedUnitScaler).ToString("F2")+ speedUnitWord+", "+Environment.NewLine
            + frontSteerAt30MS.floatValue +"° at " + (30 * speedUnitScaler).ToString("F2") + speedUnitWord  +Environment.NewLine +
            "and " + (frontSteerAt30MS.floatValue + frontSteerDifFor30ms * (2 * Mathf.Pow(2, -1/2f) - 1)) + "° at " + (60 * speedUnitScaler).ToString("F2") + speedUnitWord + ")"));
        rearSteerAtZeroSpeed.floatValue = EditorGUILayout.Slider("(°) Max Rear Steer", rearSteerAtZeroSpeed.floatValue, -45, 45);
        GUI.Label(GUILayoutUtility.GetLastRect(), new GUIContent("", "How many Degrees can the rear wheels steer, while the car is standing?"));
        rearSteerAt30MS.floatValue = EditorGUILayout.Slider("(°) MRS at " + (30 * speedUnitScaler).ToString("F2") + " " + speedUnitWord, rearSteerAt30MS.floatValue, -45, 45);
        float rearSteerDifFor30ms = rearSteerAt30MS.floatValue - rearSteerAtZeroSpeed.floatValue;
        GUI.Label(GUILayoutUtility.GetLastRect(), new GUIContent("", "How many Degrees can the rear wheels steer, while driving at " + (30 * speedUnitScaler).ToString("F2") + " " + speedUnitWord + "?"
            + " (It is recommendet to set a lower Steering Angle for higher Speed. With you current Settings the front wheels steer " + Environment.NewLine
            + (rearSteerAt30MS.floatValue + rearSteerDifFor30ms * (2 * Mathf.Pow(2, -1 / 0.5f) - 1)) + "° at " + (15 * speedUnitScaler).ToString("F2") + speedUnitWord + ", " + Environment.NewLine
            + rearSteerAt30MS.floatValue + "° at " + (30 * speedUnitScaler).ToString("F2") + speedUnitWord + Environment.NewLine +
            "and " + (rearSteerAt30MS.floatValue + rearSteerDifFor30ms * (2 * Mathf.Pow(2, -1 / 2f) - 1)) + "° at " + (60 * speedUnitScaler).ToString("F2") + speedUnitWord + ")"));
        //Balancing Steer
        EditorGUILayout.LabelField("Balancing Steer", EditorStyles.miniBoldLabel);
        frontSteerTowardsSlide.floatValue = EditorGUILayout.Slider("(%) fw Steer Towards Slide", frontSteerTowardsSlide.floatValue * 100, 0, 100) / 100;
        maxFrontAngleTowardsSlide.floatValue = EditorGUILayout.FloatField("(°) fw Max Steer Towards Slide", maxFrontAngleTowardsSlide.floatValue);
        rearSteerTowardsSlide.floatValue = EditorGUILayout.Slider("(%) rw Steer Towards Slide", rearSteerTowardsSlide.floatValue * 100, 0, 100) / 100;
        maxRearAngleTowardsSlide.floatValue = EditorGUILayout.FloatField("(°) rw Max Steer Towards Slide", maxRearAngleTowardsSlide.floatValue);
        //Arcady Steering
        EditorGUILayout.LabelField("Arcady Steering", EditorStyles.miniBoldLabel);
        disableNaturalSteering.floatValue = EditorGUILayout.Slider("(%) Disable natural Steering", disableNaturalSteering.floatValue * 100, 0, 100) / 100;
        GUI.Label(GUILayoutUtility.GetLastRect(), new GUIContent("", "It is possible to replace the steering with arcady steering here. " +
            "By disableing the natural steering, you can remove the impact on angular velocity, which would result from grip at the wheels' contact points."));
        maxAnglesPSChangeFromArcadySteering.floatValue = EditorGUILayout.FloatField("(°/s^2) Arcady angular acceleration", maxAnglesPSChangeFromArcadySteering.floatValue);
        GUI.Label(GUILayoutUtility.GetLastRect(), new GUIContent("", "This variable defines how fast the angular velocity can change per second on the ground to match the wanted angular velocity." +
            " This wanted Angular velocity is defined by your current speed divided by the theoretical length of the ground-turning-cicle, which results from your current steer. Note that this is no directly natural behaviour and is therefore called arcady"));
        //Air Steering
        EditorGUILayout.LabelField("Air Steering", EditorStyles.miniBoldLabel);
        maxAnglesPSChangeAtAirSteering.floatValue = EditorGUILayout.FloatField("(°/s^2) Angular acceleration in Air", maxAnglesPSChangeAtAirSteering.floatValue);
        GUI.Label(GUILayoutUtility.GetLastRect(), new GUIContent("", "This variable defines how fast the angular velocity can change per second in the air to match the wanted angular velocity." +
            " This wanted Angular velocity is defined by your current speed divided by the theoretical length of the ground-turning-cicle, which results from your current steer."));
        GUILayout.Space(15);
        //if (isRepainting) sectionHeights[sectionCount] = GUILayoutUtility.GetLastRect().yMax;


        //INPUT SETTINGS
        EditorGUILayout.LabelField("Input Settings", EditorStyles.boldLabel);
        EditorGUILayout.PropertyField(breakAtOtherDirectionInput);
        EditorGUILayout.PropertyField(ThrottleKey);
        EditorGUILayout.PropertyField(BackwardThrottleKey);
        EditorGUILayout.PropertyField(SteerLeftKey);
        EditorGUILayout.PropertyField(SteerRightKey);
        EditorGUILayout.PropertyField(BreakKey);
        EditorGUILayout.PropertyField(HandbreakKey);
        EditorGUILayout.PropertyField(GearShiftKey);
        EditorGUILayout.PropertyField(GearShiftUpKey);
        EditorGUILayout.PropertyField(GearShiftDownKey);
        EditorGUILayout.PropertyField(NitroKey);
        GUILayout.Space(15);


        //AUDIO SETTINGS
        EditorGUILayout.LabelField("Audio Settings", EditorStyles.boldLabel);
        EditorGUILayout.PropertyField(audioForEngine);
        deltaPitchAtLowGearSpeed.floatValue = EditorGUILayout.Slider("(%) Pitch down at low Gear Speed", deltaPitchAtLowGearSpeed.floatValue *100, 0, 100) / 100;
        deltaPitchAtHighGearSpeed.floatValue = EditorGUILayout.Slider("(%) Pitch up at high Gear Speed", deltaPitchAtHighGearSpeed.floatValue * 100, 0, 100) / 100;
        amplifyDeltaPitchPerGear.floatValue = EditorGUILayout.Slider("(%) Amplify Delta Pitch per Gear", amplifyDeltaPitchPerGear.floatValue * 100, 0, 100)/100;
        volumeDropOffAtGearShift.floatValue = EditorGUILayout.Slider("(%) Volume Dropoff at Gear Shift", volumeDropOffAtGearShift.floatValue * 100, 0, 100) / 100;
        volumeScaleWithThrottleInput.floatValue = EditorGUILayout.Slider("(%) Volume scales with Throttle Input", volumeScaleWithThrottleInput.floatValue * 100, 0, 100) / 100;
        volumeDropOffAtLowGearEffectiveness.floatValue = EditorGUILayout.Slider("Volume Dropoff at low Gear Effectivness", volumeDropOffAtLowGearEffectiveness.floatValue, 0, 1);
        GUILayout.Space(15);


        //GUI SETTINGS
        EditorGUILayout.LabelField("Gui Settings", EditorStyles.boldLabel);
        EditorGUILayout.PropertyField(sliderToShowShift);
        EditorGUILayout.PropertyField(sliderToShowClutch);
        EditorGUILayout.PropertyField(guiGear);
        EditorGUILayout.PropertyField(guiSpeed);
        GUILayout.Space(15);


        //VISUAL SETTINGS
        for (int i = 0; i < 4; i++)
        {
            PlaceVisualWheelOnPhysicalWheelPosition(i);
        }
        //hier kommt ausklappding drum
        for (int i = 0; i < 4; i++)
        {
            InsertVisualWheel(i);
        }


        ClosePreviouseSection(sectionCount);

        //prevent user from setting invalide values
        if (frontRightWheelCenter.vector3Value.x<0) frontRightWheelCenter.vector3Value += Vector3.left*frontRightWheelCenter.vector3Value.x;
        if (frontRightWheelCenter.vector3Value.z < 0) frontRightWheelCenter.vector3Value += Vector3.back * frontRightWheelCenter.vector3Value.z;
        if (frontWheelRadius.floatValue < 0.01f) frontWheelRadius.floatValue = 0.01f;
        if (frontSuspDist.floatValue < 0.01f) frontSuspDist.floatValue = 0.01f;
        if (frontSuspHardCap.floatValue < 0.01f) frontSuspHardCap.floatValue = 0.01f;
        if (frontWheelInwardThickness.floatValue < 0.01f) frontWheelInwardThickness.floatValue = 0.01f;
        if (frontWheelOutwardThickness.floatValue < 0.01f) frontWheelOutwardThickness.floatValue = 0.01f;

        if (rearRightWheelCenter.vector3Value.x < 0) rearRightWheelCenter.vector3Value += Vector3.left * rearRightWheelCenter.vector3Value.x;
        if (rearRightWheelCenter.vector3Value.z > 0) rearRightWheelCenter.vector3Value += Vector3.back * rearRightWheelCenter.vector3Value.z;
        if (rearWheelRadius.floatValue < 0.01f) rearWheelRadius.floatValue = 0.01f;
        if (rearSuspDist.floatValue < 0.01f) rearSuspDist.floatValue = 0.01f;
        if (rearSuspHardCap.floatValue < 0.01f) rearSuspHardCap.floatValue = 0.01f;
        if (rearWheelInwardThickness.floatValue < 0.01f) rearWheelInwardThickness.floatValue = 0.01f;
        if (rearWheelOutwardThickness.floatValue < 0.01f) rearWheelOutwardThickness.floatValue = 0.01f;
        if (springsByOtherValue.floatValue < 0.01f) springsByOtherValue.floatValue = 0.01f;

        PreventBeingInBothLayerMasks();
        



        serializedObject.ApplyModifiedProperties();
        EditorGUILayout.LabelField("(Below this object)");


        //Gizmo Visualisation
        //carGizmos.VisualiseWheels(frontRightWheelCenter.vector3Value, rearRightWheelCenter.vector3Value, frontWheelRadius.floatValue, rearWheelRadius.floatValue, frontSuspDist.floatValue, rearSuspDist.floatValue);
        carGizmos.VisualiseWheels(showWheelSettings.boolValue, frontRightWheelCenter.vector3Value, frontSuspDist.floatValue, frontWheelRadius.floatValue, frontSuspHardCap.floatValue,
                                  frontUse3DWheelPhysics.boolValue, frontWheelInwardThickness.floatValue, frontWheelOutwardThickness.floatValue,
                                  showWheelSettings.boolValue, rearRightWheelCenter.vector3Value, rearSuspDist.floatValue, rearWheelRadius.floatValue, rearSuspHardCap.floatValue,
                                  rearUse3DWheelPhysics.boolValue, rearWheelInwardThickness.floatValue, rearWheelOutwardThickness.floatValue);


        //if (EditorApplication.isPlaying)
        {
            car.UpdateDependendParameters();
            car.UpdateArrayAccessibleParameters(true);
        }

    }


    public static bool IsInLayerMask(int layerMask, int layer)
    {
        return (layerMask & (1 << layer)) != 0;
    }

    void PreventBeingInBothLayerMasks()
    {
        //LayerMask solidLayer = EditorGUILayout.MaskField(InternalEditorUtility.LayerMaskToConcatenatedLayersMask(solidGroundLayer), InternalEditorUtility.layers);
        //solidGroundLayer.
        LayerMask solidLayerCopy = solidGroundLayer.intValue;

        for (int i = 0; i < 32; i++)
        {
            if (IsInLayerMask(looseGroundLayer.intValue, i)){
                //Debug.Log("Layer" + i + "is in loose Layer!");
            }
            if (IsInLayerMask(looseGroundLayer.intValue, i) && IsInLayerMask(solidGroundLayer.intValue, i))
            {
                Debug.Log("A Layer can not be solid ground and loose ground at once. Accordingly Layer" + i + " ("+ LayerMask.LayerToName(i) + ") has been removed from the solid Ground mask!");
                solidLayerCopy &= ~(1 << i);
            }
        }
        solidGroundLayer.intValue = solidLayerCopy;
    }

    void ClassicUnitConversionFix(ref float value)
    {
        if(value%1 < 0.0001) value -= value%1;
    }


    Texture2D CreateGearTexture(int xRes, int yRes)
    {
        //int resolution = 100;
        Texture2D tex = new Texture2D(xRes, yRes, TextureFormat.RGBA32, false);

        Color grayBar = new Color(0.73f, 0.73f, 0.73f);

        //Draw Baseline
        for (int x = 0; x < xRes; x++)
        {

            int y = yRes / 2;
            tex.SetPixel(x, y, Color.black);
        }

        //Draw vertical line for each gears peak ratio
        for (int i = 0; i < numberOfGears.intValue; i++)
        {
            int relBestSpeedInXResolution = (int)(car.gearIsBestAtRelSpeed[i]*xRes);
            for(int y = 0; y<yRes-1; y++)
            {
                tex.SetPixel(relBestSpeedInXResolution, y, grayBar);
            }
            //White fade at the top of the Grey bar to indicate relation to the sliders above
            tex.SetPixel(relBestSpeedInXResolution, yRes - 1, Color.white);
            tex.SetPixel(relBestSpeedInXResolution, yRes - 2, new Color(0.95f,0.95f,0.95f));
            tex.SetPixel(relBestSpeedInXResolution, yRes - 2, new Color(0.8f, 0.8f, 0.8f));
        }


        //Draw lines to represent multiplier per ratio for each gear
        float topSpeed = car.SpeedupCurve.topSpeed;
        for(int i = 0; i< numberOfGears.intValue; i++)
        {
            int prevY = (int)(car.GetAccelerationScaleByGear(i, car.SpeedupCurve.curve.Evaluate(0) * topSpeed) / 2 * yRes);
            if (prevY < 0) prevY = 0;
            Color gearColor = new Color(0, 1 /*- ((float)i/(numberOfGears.intValue-1))*/, (float)i / (numberOfGears.intValue-1));
            for (int x = 0; x<xRes; x++)
            {
                float scaler = car.GetAccelerationScaleByGear(i, car.SpeedupCurve.curve.Evaluate((x + 1f) / xRes) * topSpeed);
                if(scaler > 0 && scaler <= 2)
                {
                    int y = (int)(scaler / 2 * yRes);
                    int yDif = y-prevY;
                    if (yDif == 0)
                    {
                        tex.SetPixel(x, y, gearColor);
                    }
                    else if(yDif > 0)
                    {
                        for(int h = y; h > prevY; h--)
                        {
                            tex.SetPixel(x, h, gearColor);
                        }
                    }
                    else
                    {
                        for (int h = y; h < prevY; h++)
                        {
                            tex.SetPixel(x, h, gearColor);
                        }
                    }
                    prevY = y;
                }
                
            }
        }
        

        tex.Apply();
        return tex;
    }

    Texture2D GetAlternatingBackgroundTex(int sectionIndex)
    {
        if(sectionIndex%2 == 0)
        {
            return tex0;
        }
        else
        {
            return tex1;
        }
    }


    void DrawNextSectionAndClosePreviouse(ref int sectionCount)
    {
        ClosePreviouseSection(sectionCount);
        if (Event.current.type == EventType.Repaint)  GUI.DrawTexture(new Rect(0, sectionHeights[sectionCount], Screen.width, sectionHeights[++sectionCount]), GetAlternatingBackgroundTex(sectionCount), ScaleMode.StretchToFill);
    }
    void ClosePreviouseSection(int sectionCount)
    {
        if (Event.current.type == EventType.Repaint) sectionHeights[sectionCount] = GUILayoutUtility.GetLastRect().yMax;
    }

    void InsertVisualWheel(int index)
    {
        string wheelName = (index % 2 == 0 ? "Left " : "Right ") + (index < 2 ? "Front " : "Rear ") + "Wheel:";
        EditorGUILayout.BeginHorizontal();
        {
            EditorGUILayout.LabelField(wheelName, GUILayout.Width(130));
            visualWheels[index] = EditorGUILayout.ObjectField(visualWheels[index], typeof(Transform),  GUILayout.MaxWidth(70)) as Transform;
            EditorGUILayout.LabelField("place at physcial pos.", GUILayout.Width(130));
            placeOnPhysicalWheel.GetArrayElementAtIndex(index).boolValue = EditorGUILayout.Toggle("", placeOnPhysicalWheel.GetArrayElementAtIndex(index).boolValue, GUILayout.MaxWidth(70));
        }
        EditorGUILayout.EndHorizontal();
    }

    void PlaceVisualWheelOnPhysicalWheelPosition(int index)
    {
        if (Application.isPlaying) { return; }
        if (placeOnPhysicalWheel.GetArrayElementAtIndex(index).boolValue)
        {
            if (visualWheels[index] == null) return;
            Vector3 xPosChange = index%2==0? (index < 2 ? frontRightWheelCenter.vector3Value.x : rearRightWheelCenter.vector3Value.x) * Vector3.left *2 : Vector3.zero;
            Vector3 relPosition = (index < 2 ? frontRightWheelCenter.vector3Value : rearRightWheelCenter.vector3Value) + xPosChange;
            visualWheels[index].position = car.transform.TransformPoint(relPosition);
        }
        
    }

    void DrawRearFrontSlider(ref SerializedProperty value)
    {
        EditorGUILayout.BeginHorizontal();
        {
            EditorGUILayout.LabelField("(%) Rear", GUILayout.MaxWidth(75));
            value.floatValue = 1 - EditorGUILayout.FloatField("", (1 - value.floatValue) * 100, GUILayout.MaxWidth(50)) / 100;
            value.floatValue = EditorGUILayout.Slider("", value.floatValue * 100, 0, 100, GUILayout.MaxWidth(Screen.width - 310)) / 100;
            EditorGUILayout.LabelField("Front", GUILayout.MaxWidth(60));
        }
        EditorGUILayout.EndHorizontal();
    }

}

#endif


//EditorGUILayout.BeginHorizontal();
//EditorGUILayout.LabelField("Loose Spring Range");
//frontSuspDist.floatValue = EditorGUILayout.FloatField("", frontSuspDist.floatValue, GUILayout.Width(50));
//EditorGUILayout.EndHorizontal();


