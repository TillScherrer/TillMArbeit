using UnityEngine;
using UnityEditor;
using System;
using UnityEngine.UIElements;
using Unity.VisualScripting;
using UnityEditorInternal;
using System.Reflection;

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
    float[] sectionHeights = new float[10];
    float gearRecTo = 0;
    
    //Background color 0 as texture
    Texture2D tex0;
    //Background color 1 as texture
    Texture2D tex1;
    

    Car car;

    //acceleration
    SerializedProperty speedupCurve;

    //gears
    SerializedProperty gearImpactOnAcceleration;
    SerializedProperty numberOfGears;
    SerializedProperty gearOutDuration;
    SerializedProperty shiftDuration;
    SerializedProperty gearInDuration;
    SerializedProperty shiftingImpactOnAcceleration;
    //SerializedProperty hasAutomaticShifting;
    SerializedProperty gearShiftMode;

    //physical wheel-position and spring settings
    SerializedProperty showFrontWheelSettings;
    SerializedProperty frontRightWheelCenter;
    SerializedProperty frontWheelRadius;
    SerializedProperty frontUse3DWheelPhysics;
    SerializedProperty frontWheelInwardThickness;
    SerializedProperty frontWheelOutwardThickness;
    SerializedProperty frontWheelShapeAccuracy;
    SerializedProperty frontSuspDist;
    SerializedProperty frontDamping;
    SerializedProperty frontSuspHardCap;
    SerializedProperty showRearWheelSettings;
    SerializedProperty rearRightWheelCenter;
    SerializedProperty rearWheelRadius;
    SerializedProperty rearUse3DWheelPhysics;
    SerializedProperty rearWheelInwardThickness;
    SerializedProperty rearWheelOutwardThickness;
    SerializedProperty rearWheelShapeAccuracy;
    SerializedProperty rearSuspDist;
    SerializedProperty rearDamping;
    SerializedProperty rearSuspHardCap;

    SerializedProperty springsByDefaultGravity;
    SerializedProperty springsByOtherValue;
    SerializedProperty lateralAttackHeightLift;
    SerializedProperty longitudalAttackHeightLift;
    SerializedProperty frontAntiRollBar;
    SerializedProperty rearAntiRollBar;

    SerializedProperty solidGroundLayer;
    SerializedProperty looseGroundLayer;


    SerializedProperty scaleGripWithSpringCompression;
    SerializedProperty scaleGripWithDampingCompression;
    SerializedProperty spreadGripFromNormalForceOnAllWheels;

    SerializedProperty sameGripSettingsForRearWheel;
    SerializedProperty endlessFrontWheelGrip;
    SerializedProperty endlessBackWheelGrip;
    //SerializedProperty frontWheelGrip;
    //SerializedProperty backWheelGrip;

    SerializedProperty frontGripOnSolidGround;
    SerializedProperty frontGripOnLooseGround;
    SerializedProperty rearGripOnSolidGround;
    SerializedProperty rearGripOnLooseGround;


    //steering
    SerializedProperty ackermanSteering;
    SerializedProperty maxSteerChangePerSecond;
    SerializedProperty frontSteerAtZeroSpeed;
    SerializedProperty frontSteerAt30MS;
    SerializedProperty rearSteerAtZeroSpeed;
    SerializedProperty rearSteerAt30MS;
    

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



    int speedUnit = 0;
    string speedUnitWord = "";
    float speedUnitScaler = 1;
    CarGizmos carGizmos;
    
    //Which Settings are folded out:
    bool showAccelerationSettings = false;
    bool showGearSettings = false;

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
        //gear settings
        gearImpactOnAcceleration = serializedObject.FindProperty("gearImpactOnAcceleration");
        numberOfGears = serializedObject.FindProperty("numberOfGears");
        gearOutDuration = serializedObject.FindProperty("gearOutDuration");
        shiftDuration = serializedObject.FindProperty("shiftDuration");
        gearInDuration = serializedObject.FindProperty("gearInDuration");
        shiftingImpactOnAcceleration = serializedObject.FindProperty("shiftingImpactOnAcceleration");
        //hasAutomaticShifting = serializedObject.FindProperty("hasAutomaticShifting");
        gearShiftMode = serializedObject.FindProperty("gearShiftMode");
       //front wheel physical position and spring settings
       showFrontWheelSettings = serializedObject.FindProperty("showFrontWheelSettings");
        frontRightWheelCenter = serializedObject.FindProperty("frontRightWheelCenter");
        frontWheelRadius = serializedObject.FindProperty("frontWheelRadius");
        frontUse3DWheelPhysics = serializedObject.FindProperty("frontUse3DWheelPhysics");
        frontWheelInwardThickness = serializedObject.FindProperty("frontWheelInwardThickness");
        frontWheelOutwardThickness = serializedObject.FindProperty("frontWheelOutwardThickness");
        frontWheelShapeAccuracy = serializedObject.FindProperty("frontWheelShapeAccuracy");
        frontSuspDist = serializedObject.FindProperty("frontWheelSuspensionDistanceToLiftCarWeight");
        frontDamping = serializedObject.FindProperty("frontWheelDamping");
        frontSuspHardCap = serializedObject.FindProperty("frontSuspHardCap");
        //rear wheel physical position and spring settings
        showRearWheelSettings = serializedObject.FindProperty("showRearWheelSettings");
        rearRightWheelCenter = serializedObject.FindProperty("rearRightWheelCenter");
        rearWheelRadius = serializedObject.FindProperty("rearWheelRadius");
        rearUse3DWheelPhysics = serializedObject.FindProperty("rearUse3DWheelPhysics");
        rearWheelInwardThickness = serializedObject.FindProperty("rearWheelInwardThickness");
        rearWheelOutwardThickness = serializedObject.FindProperty("rearWheelOutwardThickness");
        rearWheelShapeAccuracy = serializedObject.FindProperty("rearWheelShapeAccuracy");
        rearSuspDist = serializedObject.FindProperty("rearWheelSuspensionDistanceToLiftCarWeight");
        rearDamping = serializedObject.FindProperty("rearWheelDamping");
        rearSuspHardCap = serializedObject.FindProperty("rearSuspHardCap");
        //spring context
        springsByDefaultGravity = serializedObject.FindProperty("springsByDefaultGravity");
        springsByOtherValue = serializedObject.FindProperty("springsByOtherValue");
        lateralAttackHeightLift  = serializedObject.FindProperty("lateralAttackHeightLift");
        longitudalAttackHeightLift = serializedObject.FindProperty("longitudalAttackHeightLift");
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
        sameGripSettingsForRearWheel = serializedObject.FindProperty("sameGripSettingsForRearWheel");
        endlessFrontWheelGrip = serializedObject.FindProperty("endlessFrontWheelGrip");
        endlessBackWheelGrip = serializedObject.FindProperty("endlessBackWheelGrip");
        //grip settings fine
        frontGripOnSolidGround = serializedObject.FindProperty("frontGripOnSolidGround");
        frontGripOnLooseGround = serializedObject.FindProperty("frontGripOnLooseGround");
        rearGripOnSolidGround = serializedObject.FindProperty("rearGripOnSolidGround");
        rearGripOnLooseGround = serializedObject.FindProperty("rearGripOnLooseGround");

        //steering
        ackermanSteering = serializedObject.FindProperty("ackermanSteering");
        maxSteerChangePerSecond = serializedObject.FindProperty("maxSteerChangePerSecond");
        frontSteerAtZeroSpeed = serializedObject.FindProperty("frontSteerAtZeroSpeed");
        frontSteerAt30MS = serializedObject.FindProperty("frontSteerAt30MS");
        rearSteerAtZeroSpeed = serializedObject.FindProperty("rearSteerAtZeroSpeed");
        rearSteerAt30MS = serializedObject.FindProperty("rearSteerAt30MS");
        

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


        carGizmos = target.GetComponentInChildren<CarGizmos>();
    }

    public override void OnInspectorGUI()
    {
        float currentH = 0;
        int sectionCount = 0;

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
        GUI.DrawTexture(new Rect(0, currentH, Screen.width, sectionHeights[sectionCount]), tex0, ScaleMode.StretchToFill);
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
            EditorGUI.indentLevel--;
        }
        currentH += lineH + 10;
        sectionHeights[sectionCount] = currentH;
        GUILayout.Space(10);

        //gear settings
        GUI.DrawTexture(new Rect(0, sectionHeights[sectionCount], Screen.width, sectionHeights[++sectionCount]), tex1, ScaleMode.StretchToFill);
        showGearSettings = EditorGUILayout.Foldout(showGearSettings, "Gears");
        currentH+= lineH;
        if (showGearSettings)
        {
            EditorGUI.indentLevel++;
            currentH += lineH + 4;
            EditorGUILayout.PropertyField(numberOfGears);
            if (numberOfGears.intValue > 0)
            {
                currentH += lineH;
                GUI.DrawTexture(new Rect(EditorGUI.indentLevel*30, currentH, Screen.width, gearRecTo-currentH), tex0, ScaleMode.StretchToFill);
                GUILayout.Label("Where in the Acceleration-Curve is the Gear's peak Performance? (Ratio within Acceleration Time)", new GUIStyle(EditorStyles.centeredGreyMiniLabel));            
            }
            car.ValidateGearSettings();
            EditorGUI.indentLevel++;
            for (int i = 0; i<numberOfGears.intValue && i < car.gearIsBestAtRelSpeed.Count; i++)
            {
                
                car.gearIsBestAtRelSpeed[i] = EditorGUILayout.Slider("Gear "+(i+1)+":", car.gearIsBestAtRelSpeed[i], 0, 1);
                currentH += lineH+2;
            }
            gearRecTo = currentH;
            EditorGUI.indentLevel--;
            if (numberOfGears.intValue > 0)
            {
                gearImpactOnAcceleration.floatValue = EditorGUILayout.Slider("Gear Impact on Acceleration", gearImpactOnAcceleration.floatValue, 0, 1);
                GUI.Label(GUILayoutUtility.GetLastRect(), new GUIContent("", "If you set this to 0.2, the car accelerates 1.2 times faster at optimum speed of a gears, but accordingly worse, the further away it is from the optimum, in a way that the total acceleration time is preserved. Set this to 0 if you only use gears for the soundsystem"));
                gearOutDuration.floatValue = EditorGUILayout.Slider("(s) Gear Out Duration", gearOutDuration.floatValue, 0, 0.9f);
                shiftDuration.floatValue = EditorGUILayout.Slider("(s) Shift Duration", shiftDuration.floatValue, 0, 0.9f);
                gearInDuration.floatValue = EditorGUILayout.Slider("(s) Gear In Duration", gearInDuration.floatValue, 0, 0.9f);
                float totalShiftingTime = gearOutDuration.floatValue + shiftDuration.floatValue + gearInDuration.floatValue;
                GUILayout.Label("A Gear Shift takes " + totalShiftingTime + " Seconds in total", new GUIStyle(EditorStyles.centeredGreyMiniLabel));
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



                //GUILayout.Label("Shifting Mode:", new GUIStyle(EditorStyles.centeredGreyMiniLabel));               
                //EditorGUILayout.BeginHorizontal();
                //{
                //    var previousColoru = GUI.backgroundColor;
                //    if (hasAutomaticShifting.boolValue) GUI.backgroundColor = colorForRealism;
                //    bool switchToDefaultGravity = GUILayout.Button("Automatic");
                //    if (switchToDefaultGravity) hasAutomaticShifting.boolValue = true;
                //    GUI.backgroundColor = hasAutomaticShifting.boolValue ? previousColoru : colorForRealism;
                //    bool switchToCustomGravity = GUILayout.Button("Manuell");
                //    if (switchToCustomGravity) hasAutomaticShifting.boolValue = false;
                //    GUI.backgroundColor = previousColoru;
                //}
                //EditorGUILayout.EndHorizontal();
                //currentH += lineH * 9+3;
                EditorGUILayout.PropertyField(gearShiftMode);
                currentH += lineH * 8 + 5;
            }
            EditorGUI.indentLevel--;
            
        }
        currentH += 11;
        sectionHeights[sectionCount] = currentH;
        GUILayout.Space(10);

        //front wheel Position and Spring Settings
        GUI.DrawTexture(new Rect(0, sectionHeights[sectionCount], Screen.width, sectionHeights[++sectionCount] - sectionHeights[sectionCount-1]), tex0, ScaleMode.StretchToFill);
        showFrontWheelSettings.boolValue = EditorGUILayout.Foldout(showFrontWheelSettings.boolValue, "Front Wheel Settings");
        if (showFrontWheelSettings.boolValue)
        {
            EditorGUI.indentLevel++;
            frontRightWheelCenter.vector3Value  = EditorGUILayout.Vector3Field("PositionR", frontRightWheelCenter.vector3Value);
            frontWheelRadius.floatValue         = EditorGUILayout.FloatField("Radius", frontWheelRadius.floatValue);
            frontSuspDist.floatValue            = EditorGUILayout.FloatField("Loose Spring Range", frontSuspDist.floatValue);
            frontDamping.floatValue             = EditorGUILayout.Slider("Damping", frontDamping.floatValue, 0, 10);
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
            currentH += lineH * 6+16;

            if (frontUse3DWheelPhysics.boolValue)
            {
                EditorGUI.indentLevel++;
                frontWheelInwardThickness.floatValue    = EditorGUILayout.FloatField("Inward Thickness", frontWheelInwardThickness.floatValue);
                frontWheelOutwardThickness.floatValue   = EditorGUILayout.FloatField("Outward Thickness", frontWheelOutwardThickness.floatValue);
                frontWheelShapeAccuracy.intValue        = EditorGUILayout.IntSlider("Shape Accuracy", frontWheelShapeAccuracy.intValue, 4, 12);
                currentH += lineH * 3+6;
                EditorGUI.indentLevel--;
            }
            GUI.backgroundColor = oldColor;
            EditorGUI.indentLevel--;
        }
        currentH += lineH + 5;
        sectionHeights[sectionCount] = currentH;
        GUILayout.Space(5);

        //rear wheel Position and Spring Settings
        GUI.DrawTexture(new Rect(0, sectionHeights[sectionCount], Screen.width, sectionHeights[++sectionCount] - sectionHeights[sectionCount-1]), tex1, ScaleMode.StretchToFill);
        showRearWheelSettings.boolValue = EditorGUILayout.Foldout(showRearWheelSettings.boolValue, "Rear Wheel Settings");
        if (showRearWheelSettings.boolValue)
        {
            EditorGUI.indentLevel++;
            rearRightWheelCenter.vector3Value = EditorGUILayout.Vector3Field("PositionR", rearRightWheelCenter.vector3Value);
            rearWheelRadius.floatValue = EditorGUILayout.FloatField("Radius", rearWheelRadius.floatValue);
            rearSuspDist.floatValue = EditorGUILayout.FloatField("Loose Spring Range", rearSuspDist.floatValue);
            rearDamping.floatValue=  EditorGUILayout.Slider("Damping", rearDamping.floatValue, 0, 10);
            rearSuspHardCap.floatValue = EditorGUILayout.FloatField("Upward Suspension Stop", rearSuspHardCap.floatValue);

            var oldColor = GUI.backgroundColor;
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
            currentH += lineH * 6 + 16;

            if (rearUse3DWheelPhysics.boolValue)
            {
                EditorGUI.indentLevel++;
                rearWheelInwardThickness.floatValue = EditorGUILayout.FloatField("Inward Thickness", rearWheelInwardThickness.floatValue);
                rearWheelOutwardThickness.floatValue = EditorGUILayout.FloatField("Outward Thickness", rearWheelOutwardThickness.floatValue);
                rearWheelShapeAccuracy.intValue = EditorGUILayout.IntSlider("Shape Accuracy", rearWheelShapeAccuracy.intValue, 4, 12);
                currentH += lineH * 3 + 6;
                EditorGUI.indentLevel--;
            }
            GUI.backgroundColor = oldColor;
            EditorGUI.indentLevel--;
        }
        currentH += lineH + 10;
        sectionHeights[sectionCount] = currentH;
        GUILayout.Space(10);

        GUI.DrawTexture(new Rect(0, sectionHeights[sectionCount], Screen.width, sectionHeights[++sectionCount] - sectionHeights[sectionCount - 1]), tex0, ScaleMode.StretchToFill);
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
        EditorGUILayout.EndHorizontal();
        GUI.backgroundColor = previousColor;
        EditorGUI.indentLevel--;
        currentH += lineH * 3+5+10;
        sectionHeights[sectionCount] = currentH;
        GUILayout.Space(10);

        GUI.DrawTexture(new Rect(0, sectionHeights[sectionCount], Screen.width, sectionHeights[++sectionCount] - sectionHeights[sectionCount - 1]), tex1, ScaleMode.StretchToFill);
        lateralAttackHeightLift.floatValue = EditorGUILayout.Slider("(%) Lift lateral-attackpoint", lateralAttackHeightLift.floatValue*100, 0, 100)/100;
        GUI.Label(GUILayoutUtility.GetLastRect(), new GUIContent("", "0% is a realistic behavior, where sideward grip applies force at ground height (potentionally flipping the car sideward), whereas 100% prevents this behaviour by lifting the attackpoint to the center of mass's height")); //tooltip to previous https://www.reddit.com/r/Unity3D/comments/45bjwc/tooltip_on_custom_inspectorproperties/ by againey
        longitudalAttackHeightLift.floatValue = EditorGUILayout.Slider("(%) Lift longitudal-attackpoint", longitudalAttackHeightLift.floatValue * 100, 0, 100) / 100;
        GUI.Label(GUILayoutUtility.GetLastRect(), new GUIContent("", "0% is a realistic behavior, where grip of accelerating and breaking applies force at ground height (potentionally flipping the car foward/backward), whereas 100% prevents this behaviour by lifting the attackpoint to the center of mass's height"));
        frontAntiRollBar.floatValue = EditorGUILayout.Slider("(%) Front Anti-Roll Bar", frontAntiRollBar.floatValue * 100, 0, 100) / 100;
        rearAntiRollBar.floatValue = EditorGUILayout.Slider("(%) Rear Anti-Roll Bar", rearAntiRollBar.floatValue * 100, 0, 100) / 100;
        currentH += lineH * 4+15;
        sectionHeights[5] = currentH;
        GUILayout.Space(15);

        GUI.DrawTexture(new Rect(0, sectionHeights[5], Screen.width, sectionHeights[6] - sectionHeights[5]), tex0, ScaleMode.StretchToFill);
        EditorGUILayout.LabelField("Each Wheel's Grip is scaled (%) by what?", new GUIStyle(EditorStyles.centeredGreyMiniLabel));
        scaleGripWithSpringCompression.floatValue = EditorGUILayout.Slider("By its Spring Compression", scaleGripWithSpringCompression.floatValue * 100, 0, 100) / 100;
        scaleGripWithDampingCompression.floatValue = EditorGUILayout.Slider("By its Damping Compression", scaleGripWithDampingCompression.floatValue * 100, 0, 100) / 100;
        spreadGripFromNormalForceOnAllWheels.floatValue = EditorGUILayout.Slider("By other Wheels' Compressions", spreadGripFromNormalForceOnAllWheels.floatValue * 100, 0, 100) / 100;
        currentH += lineH * 4 + 25;
        sectionHeights[6] = currentH;
        GUILayout.Space(15);

        GUI.DrawTexture(new Rect(0, sectionHeights[6], Screen.width, sectionHeights[7] - sectionHeights[6]), tex1, ScaleMode.StretchToFill);
        EditorGUILayout.PropertyField(solidGroundLayer);
        EditorGUILayout.PropertyField(looseGroundLayer);
        currentH += lineH * 2 + 10;
        sectionHeights[7] = currentH;

        //endlessGrip.editable = false;
        //EditorGUILayout.PropertyField(endlessFrontWheelGrip);

        //FRONT WHEEL GRIP
        var prevColor2 = GUI.backgroundColor;
        EditorGUILayout.BeginHorizontal();
        {
            if (!endlessFrontWheelGrip.boolValue) GUI.backgroundColor = colorForRealism;
            bool switchToDynamicGrip = GUILayout.Button("Dynamic-");
            if (switchToDynamicGrip) endlessFrontWheelGrip.boolValue = false;
            GUI.backgroundColor = endlessFrontWheelGrip.boolValue ? GUI.backgroundColor = colorForArcade : prevColor2;
            bool switchToEndlessGrip = GUILayout.Button("Endless-");
            if (switchToEndlessGrip) endlessFrontWheelGrip.boolValue = true;
            GUI.backgroundColor = endlessFrontWheelGrip.boolValue ? GUI.backgroundColor = prevColor2 : colorForRealism;
            EditorGUILayout.LabelField("Front Wheel Grip", GUILayout.Width(110));
        }
        EditorGUILayout.EndHorizontal();
        if (!endlessFrontWheelGrip.boolValue)
        {
                EditorGUI.indentLevel++;
            car.FrontGripOnSolidGround.ValidateCurve();
            car.FrontGripOnLooseGround.ValidateCurve();

            EditorGUILayout.LabelField("Settings for FRONT wheel on SOLID Ground:");
            car.FrontGripOnSolidGround.maxGrip = EditorGUILayout.FloatField("Grip Multiplicator", car.FrontGripOnSolidGround.maxGrip);
            car.FrontGripOnSolidGround.definedUpToSlideSpeed = EditorGUILayout.FloatField("Sliding-speed Scale", car.FrontGripOnSolidGround.definedUpToSlideSpeed);
            EditorGUILayout.CurveField("Grip at SlidingSpeed",car.FrontGripOnSolidGround.curve, colorForRealism, new Rect(0, 0f, 1f, 1f));

            EditorGUILayout.LabelField("Settings for FRONT wheel on LOOSE Ground:");
            car.FrontGripOnLooseGround.maxGrip = EditorGUILayout.FloatField("Grip Multiplicator", car.FrontGripOnLooseGround.maxGrip);
            car.FrontGripOnLooseGround.definedUpToSlideSpeed = EditorGUILayout.FloatField("Sliding-speed Scale", car.FrontGripOnLooseGround.definedUpToSlideSpeed);
            EditorGUILayout.CurveField("Grip at SlidingSpeed", car.FrontGripOnLooseGround.curve, colorForRealism + new Color(0.1f,0.1f,0.1f), new Rect(0, 0f, 1f, 1f));
                EditorGUI.indentLevel--;
        }
        GUI.backgroundColor = prevColor2;




        //REAR WHEEL GRIP
        var prevColor3 = GUI.backgroundColor;
        EditorGUILayout.BeginHorizontal();
        {
            if (!endlessBackWheelGrip.boolValue) GUI.backgroundColor = colorForRealism;
            bool switchToDynamicGrip = GUILayout.Button("Dynamic-");
            if (switchToDynamicGrip) endlessBackWheelGrip.boolValue = false;
            GUI.backgroundColor = endlessBackWheelGrip.boolValue ? GUI.backgroundColor = colorForArcade : prevColor2;
            bool switchToEndlessGrip = GUILayout.Button("Endless-");
            if (switchToEndlessGrip) endlessBackWheelGrip.boolValue = true;
            GUI.backgroundColor = endlessBackWheelGrip.boolValue ? GUI.backgroundColor = prevColor2 : colorForRealism;
            
            EditorGUILayout.LabelField("Rear Wheel Grip", GUILayout.Width(110));
        }
        EditorGUILayout.EndHorizontal();
        if (!endlessBackWheelGrip.boolValue)
        {
            EditorGUI.indentLevel++;

            sameGripSettingsForRearWheel.boolValue = EditorGUILayout.Toggle("Same Dynamic Grip Settings", sameGripSettingsForRearWheel.boolValue);
            if(sameGripSettingsForRearWheel.boolValue && endlessFrontWheelGrip.boolValue)
            {
                EditorGUILayout.LabelField("-> Endless Rear Grip, just like Front Wheels");
            }
            
            if (!sameGripSettingsForRearWheel.boolValue)
            {
                car.RearGripOnSolidGround.ValidateCurve();
                car.RearGripOnLooseGround.ValidateCurve();

                EditorGUILayout.LabelField("Settings for REAR wheel on SOLID Ground:");
                car.RearGripOnSolidGround.maxGrip = EditorGUILayout.FloatField("Grip Multiplicator", car.RearGripOnSolidGround.maxGrip);
                car.RearGripOnSolidGround.definedUpToSlideSpeed = EditorGUILayout.FloatField("Sliding-speed Scale", car.RearGripOnSolidGround.definedUpToSlideSpeed);
                EditorGUILayout.CurveField("Grip at SlidingSpeed", car.RearGripOnSolidGround.curve, colorForRealism, new Rect(0, 0f, 1f, 1f));

                EditorGUILayout.LabelField("Settings for REAR wheel on LOOSE Ground:");
                car.RearGripOnLooseGround.maxGrip = EditorGUILayout.FloatField("Grip Multiplicator", car.RearGripOnLooseGround.maxGrip);
                car.RearGripOnLooseGround.definedUpToSlideSpeed = EditorGUILayout.FloatField("Sliding-speed Scale", car.RearGripOnLooseGround.definedUpToSlideSpeed);
                EditorGUILayout.CurveField("Grip at SlidingSpeed", car.RearGripOnLooseGround.curve, colorForRealism + new Color(0.1f, 0.1f, 0.1f), new Rect(0, 0f, 1f, 1f));
            }
            EditorGUI.indentLevel--;
        }
        GUI.backgroundColor = prevColor3;
        GUILayout.Space(15);


        //EditorGUILayout.PropertyField(endlessBackWheelGrip);
        //if (!endlessBackWheelGrip.boolValue)
        //{
        //    EditorGUI.indentLevel++;
        //    EditorGUILayout.PropertyField(rearGripOnSolidGround);
        //    EditorGUILayout.PropertyField(rearGripOnLooseGround);
        //    car.RearGripOnSolidGround.ValidateCurve();
        //    car.RearGripOnLooseGround.ValidateCurve();
        //    EditorGUI.indentLevel--;
        //}

        //STEERING SETTINGS
        ackermanSteering.floatValue = EditorGUILayout.Slider("(%) Ackerman Steering", ackermanSteering.floatValue * 100, 0, 100) / 100;
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
        rearSteerAtZeroSpeed.floatValue = EditorGUILayout.Slider("(°) Max Rear Steer", rearSteerAtZeroSpeed.floatValue, -45, 45); ;
        rearSteerAt30MS.floatValue = EditorGUILayout.Slider("(°) MRS at " + (30 * speedUnitScaler).ToString("F2") + " " + speedUnitWord, rearSteerAt30MS.floatValue, -45, 45);


        //INPUT SETTINGS
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
        carGizmos.VisualiseWheels(showFrontWheelSettings.boolValue, frontRightWheelCenter.vector3Value, frontSuspDist.floatValue, frontWheelRadius.floatValue, frontSuspHardCap.floatValue,
                                  frontUse3DWheelPhysics.boolValue, frontWheelInwardThickness.floatValue, frontWheelOutwardThickness.floatValue,
                                  showRearWheelSettings.boolValue, rearRightWheelCenter.vector3Value, rearSuspDist.floatValue, rearWheelRadius.floatValue, rearSuspHardCap.floatValue,
                                  rearUse3DWheelPhysics.boolValue, rearWheelInwardThickness.floatValue, rearWheelOutwardThickness.floatValue);


        if (EditorApplication.isPlaying)
        {
            car.UpdateDependendParameters();
            car.UpdateArrayAccessibleParameters();
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
}




//EditorGUILayout.BeginHorizontal();
//EditorGUILayout.LabelField("Loose Spring Range");
//frontSuspDist.floatValue = EditorGUILayout.FloatField("", frontSuspDist.floatValue, GUILayout.Width(50));
//EditorGUILayout.EndHorizontal();


