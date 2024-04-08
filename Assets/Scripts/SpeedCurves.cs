
using System;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

[Serializable]
public class CurveHolder
{

    public float timeNeeded = 2;
    public float topSpeed = 1;

    public AnimationCurve curve = new AnimationCurve();
    [HideInInspector]
    public AnimationCurve invertedCurve = new AnimationCurve();
    [HideInInspector]
    public float minGradient = 0.01f; //vorher 0.025f
    [HideInInspector]
    public virtual bool Rising { get { return false; } }
    [HideInInspector]
    public virtual int Orientation { get { return 0; } } //start hight added to value

    //[HideInInspector]
    //public float curveIntegral = 0.5f;
    //private readonly float accuracyOfIntegral = 30f;

    [HideInInspector]
    public float[] rawAccelerationValues;
    private readonly int accuracyOfAccelerationValues = 100;

    public float GetAccelerationValueForSpeed(float speed)
    {
        //BEDINGUNG BEI SPEED UNTER 0 FEHLT!!!!!!!!!!!!!!!!!!!!!!!
        //if(speed<0) return rawAccelerationValues[0]; //TEMPORARY SOLUTION, MIGHT BE REPLACED WITH BREAKING WHILE BACKWARDS DRIVING
        speed = Mathf.Abs(speed);
        if (speed > topSpeed) return 0;
        float timeInCurve = GetTimeInCurve(speed);
        //Debug.Log("time in curve= " + timeInCurve);
        float sampleValue = timeInCurve * (accuracyOfAccelerationValues + 1);
        int fromSample = Mathf.FloorToInt(sampleValue);
        int toSample = Mathf.CeilToInt(sampleValue);
        if (fromSample > accuracyOfAccelerationValues) fromSample = accuracyOfAccelerationValues;
        if (toSample > accuracyOfAccelerationValues) toSample = accuracyOfAccelerationValues;
        //Debug.Log("from sample= " + fromSample);
        //Debug.Log("to sample= " + toSample);
        //Debug.Log("accel at sample= " + rawAccelerationValues[fromSample]);
        float scaleBetween = sampleValue - fromSample; //scale from 0 to 1 between the two samples
        return Mathf.Lerp(rawAccelerationValues[fromSample], rawAccelerationValues[toSample], scaleBetween)*topSpeed/timeNeeded;
    }

    public float GetTimeInCurve(float speed)
    {
        speed = Mathf.Abs(speed);
        return invertedCurve.Evaluate(speed / topSpeed);
    }

    public void ValidateRawAccelerationValues()
    {
        //with an accuracy of 100 there are 101 samples from 0/100 to 100/100 on the curve
        //sample zero is the speed difference to the first one. Every other sample is the speed difference to the previouse one.
        rawAccelerationValues = new float[accuracyOfAccelerationValues+1];
        rawAccelerationValues[0] = (curve.Evaluate((float)1 / accuracyOfAccelerationValues) - curve.Evaluate((float)(0) / accuracyOfAccelerationValues)) * accuracyOfAccelerationValues;
        for (int i = 1; i <= accuracyOfAccelerationValues; i++)
        {
            rawAccelerationValues[i] = (curve.Evaluate((float)i / accuracyOfAccelerationValues)- curve.Evaluate((float)(i-1) / accuracyOfAccelerationValues)) * accuracyOfAccelerationValues;
            //Debug.Log("raw accel value [" + i + "] = " + rawAccelerationValues[i]);
        }
    }


    public void CreateInvertedCurve()
    {
        invertedCurve = new AnimationCurve();

        //Set Positions of inverted curve's keys
        for (int i = 0; i < curve.length; i++)
        {
            invertedCurve.AddKey(curve.keys[i].value, curve.keys[i].time);
        }

        //Set tangentes and weights of inverted curve's keys
        for (int i = 0; i < curve.length; i++)
        {
            Keyframe key;
            key = Rising ? invertedCurve.keys[i] : invertedCurve.keys[curve.length - 1 - i];//order projects backwards for falling Curve
            key.weightedMode = WeightedMode.Both;

            Keyframe orginKey = curve.keys[i];
            if (i < curve.length - 1)
            {
                Keyframe followingOrginKey = curve.keys[i + 1];
                float OrginTangentToFollowing = (followingOrginKey.value - orginKey.value) / (followingOrginKey.time - orginKey.time);
                float OrginTangentRatio = orginKey.outTangent / OrginTangentToFollowing;
                if (Rising)
                {
                    key.outTangent = 1f / curve.keys[i].outTangent;
                    key.outWeight = OrginTangentRatio * orginKey.outWeight;
                }
                else
                {
                    key.inTangent = 1f / curve.keys[i].outTangent;
                    key.inWeight = OrginTangentRatio * orginKey.outWeight;
                }

            }
            if (i > 0)
            {


                Keyframe previousOrginKey = curve.keys[i - 1];
                float OrginTangentFromPrevious = (orginKey.value - previousOrginKey.value) / (orginKey.time - previousOrginKey.time);
                float OrginTangentRatio = orginKey.inTangent / OrginTangentFromPrevious;

                if (Rising)
                {
                    key.inTangent = 1f / curve.keys[i].inTangent;
                    key.inWeight = OrginTangentRatio * orginKey.inWeight;
                }
                else
                {
                    key.outTangent = 1f / curve.keys[i].inTangent;
                    key.outWeight = OrginTangentRatio * orginKey.inWeight;
                }

            }
            if (Rising)
            {
                invertedCurve.MoveKey(i, key);
            }
            else
            {
                invertedCurve.MoveKey(curve.length - 1 - i, key);
            }
        }


    }

    //public float GetSpeedByTime(float time)
    //{
    //    float unscaledValue;
    //    if (time < 0)
    //    {
    //        unscaledValue = curve.keys[0].value + curve.keys[0].outTangent * time;
    //    }
    //    else if (time > 1)
    //    {
    //        unscaledValue = curve.keys[curve.length - 1].value + curve.keys[curve.length - 1].inTangent * (time - 1);
    //    }
    //    else
    //    {
    //        unscaledValue = curve.Evaluate(time);
    //    }
    //    return unscaledValue * AverageSpeed / curveIntegral;
    //}

    //public float GetTimeBySpeed(float speed)
    //{
    //    float speedScaledToCurve = speed / AverageSpeed * curveIntegral;

    //    if ((speedScaledToCurve < invertedCurve.keys[0].time)) //speed is on left side out of defined range
    //    {
    //        Keyframe firstKey = invertedCurve.keys[0];
    //        return firstKey.value + (speedScaledToCurve - firstKey.time) * firstKey.outTangent;
    //    }
    //    else if ((speedScaledToCurve > invertedCurve.keys[invertedCurve.length - 1].time))//speed is on right side out of defined range
    //    {
    //        Keyframe lastKey = invertedCurve.keys[invertedCurve.length - 1];
    //        return lastKey.value + (speedScaledToCurve - lastKey.time) * lastKey.inTangent;
    //    }
    //    else//speed within defined range
    //    {
    //        return invertedCurve.Evaluate(speedScaledToCurve);
    //    }
    //}

    //public void OnValidate()
    //{
    //    ValidateCurve();
    //}

    public virtual void ValidateCurve()
    {
        int oneWhenRisingElseZero = Rising ? 1 : 0;
        int positivWhenRisingElseNegative = -1 + (2 * oneWhenRisingElseZero);
        //adds start key
        if (curve.length < 1)
        {
            curve.AddKey(0, 1 - oneWhenRisingElseZero + Orientation);
        }
        //adds end key
        if (curve.length < 2)
        {
            curve.AddKey(1, oneWhenRisingElseZero + Orientation);
        }

        //moves start and end keys to fixed positions
        Keyframe jStart = curve.keys[0];
        jStart.time = 0;
        jStart.value = 1 - oneWhenRisingElseZero + Orientation;
        curve.MoveKey(0, jStart);
        Keyframe jEnd = curve.keys[curve.length - 1];
        jEnd.time = 1;
        jEnd.value = oneWhenRisingElseZero + Orientation;
        curve.MoveKey(curve.length - 1, jEnd);



        for (int i = 0; i < curve.length; i++)
        {
            Keyframe key = curve.keys[i];
            //adjustes keys values if they are not start- nor end-key
            if (i > 0 && i < curve.length - 1)
            {

                Keyframe followingKey = curve.keys[i + 1];

                float timeDiffToFollowing = followingKey.time - key.time;
                float valueDiffToFollowing = (followingKey.value - key.value) * positivWhenRisingElseNegative;

                //adjustes valuse, if its too high above the previous Key

                if (valueDiffToFollowing / timeDiffToFollowing < minGradient)
                {
                    key.value = followingKey.value - timeDiffToFollowing * minGradient * positivWhenRisingElseNegative;
                    //Debug.Log("too low!");

                }
                Keyframe previousKey = curve.keys[i - 1];
                float timeDiffFromPrevious = key.time - previousKey.time;
                float valueDiffFromPrevious = (key.value - previousKey.value) * positivWhenRisingElseNegative;
                if (valueDiffFromPrevious / timeDiffFromPrevious < minGradient)
                {
                    key.value = previousKey.value + timeDiffFromPrevious * minGradient * positivWhenRisingElseNegative;
                }



                //key.value = 1;
            }
            curve.MoveKey(i, key);


        }

        for (int i = 0; i < curve.length; i++)
        {
            Keyframe key = curve.keys[i];
            key.weightedMode = WeightedMode.Both;


            //assign vars and adjust tangents' angles
            if (i < curve.length - 1)
            {

                if (Rising)
                {
                    if (key.outTangent < minGradient) key.outTangent = minGradient;
                    else if (key.outTangent > 1 / minGradient) key.outTangent = 1 / minGradient;
                }
                else
                {
                    if (key.outTangent > -minGradient) key.outTangent = -minGradient;
                    else if (key.outTangent < -1 / minGradient) key.outTangent = -1 / minGradient;
                }
            }
            if (i > 0)
            {

                //winkel anpassen
                if (Rising)
                {
                    if (key.inTangent < minGradient) key.inTangent = minGradient;
                    else if (key.inTangent > 1 / minGradient) key.inTangent = 1 / minGradient;
                }
                else
                {
                    if (key.inTangent > -minGradient) key.inTangent = -minGradient;
                    else if (key.inTangent < -1 / minGradient) key.inTangent = -1 / minGradient;
                }
            }


            //adjust tangents' weights
            if (i < curve.length - 1)
            {
                Keyframe followingKey = curve.keys[i + 1];
                float timeDiffToFollowing = followingKey.time - key.time;
                float valueDiffToFollowing = (followingKey.value - key.value);
                float tangentToFollowing = valueDiffToFollowing / timeDiffToFollowing;
                float weightCap = tangentToFollowing / key.outTangent;
                if (weightCap < key.outWeight)
                {
                    key.outWeight = weightCap;
                }
            }
            if (i > 0)
            {
                Keyframe previousKey = curve.keys[i - 1];
                float timeDiffFromPrevious = key.time - previousKey.time;
                float valueDiffFromPrevious = (key.value - previousKey.value);
                float tangentpreviousKey = valueDiffFromPrevious / timeDiffFromPrevious;
                float weightCap = tangentpreviousKey / key.inTangent;
                if (weightCap < key.inWeight)
                {
                    key.inWeight = weightCap;
                }
            }
            curve.MoveKey(i, key);
        }

        //ValidateCurveIntegral();
        //UpdateAutocalculatedVar();
        CreateInvertedCurve();
        ValidateRawAccelerationValues();
    }

    //public void ValidateCurveIntegral()
    //{
    //    float speedHeap = 0;
    //    float stepByAccuracy = 1 / accuracyOfIntegral;
    //    float halfStepByAccuray = 0.5f / accuracyOfIntegral;

    //    for (float f = 0; f < 1; f += stepByAccuracy)
    //    {
    //        speedHeap += curve.Evaluate(f + halfStepByAccuray);
    //    }
    //    curveIntegral = Mathf.Abs(speedHeap / accuracyOfIntegral);
    //}

    //void UpdateAutocalculatedVar()
    //{
    //    switch (autoCalculate)
    //    {
    //        case AutoCalculate.TimeNeeded:
    //            timeNeeded = distance / topSpeed / curveIntegral;
    //            break;
    //        case AutoCalculate.Distance:
    //            distance = topSpeed * timeNeeded * curveIntegral;
    //            break;
    //        case AutoCalculate.TopSpeed:
    //            topSpeed = distance / timeNeeded / curveIntegral;
    //            break;
    //    }
    //}
}



//[Serializable]
//public class Jump : CurveHolder
//{

//    //[HideInInspector]
//    //public float jumpPower;


//    public override bool Rising { get { return false; } }

//    public bool useTopSpeedToStartJump = true;
//    public SpeedConverterXYToXY speedConvertionOnJumpStart;
//    public int useFallWithIndex = 0;
//    [HideInInspector]
//    public virtual bool IsCustomJump { get { return false; } }

//    override public void ValidateCurve()
//    {
//        base.ValidateCurve();
//        if (useTopSpeedToStartJump) speedConvertionOnJumpStart.ySpeed.addYSpeed = topSpeed;
//    }
//}

//[Serializable]
//public class CustomJump : Jump
//{
//    [Space(7)]
//    public ActionDisabler[] DisabledActionsOnJumpStart;
//    public ControlDisabler[] DisabledControlsOnJumpStart;

//    [Space(7)]
//    public bool doesSelectNextJump = false;
//    public ChooseNextJumpFrom chooseNextJumpFrom = ChooseNextJumpFrom.UsualJumps;
//    public int indexOfNextJump = 0;
//    [HideInInspector]
//    public override bool IsCustomJump { get { return true; } }
//    public enum ChooseNextJumpFrom
//    {
//        UsualJumps,
//        CustomJumps
//    }

//    override public void ValidateCurve()
//    {
//        base.ValidateCurve();
//        foreach (ActionDisabler ad in DisabledActionsOnJumpStart)
//        {
//            ad.ValidateActionDisabler();
//        }
//        foreach (ControlDisabler cd in DisabledControlsOnJumpStart)
//        {
//            cd.ValidateControlDisabler();
//        }
//    }
//}

//[Serializable]
//public class Fall : CurveHolder
//{
//    public override int Orientation { get { return -1; } }
//    public bool useTopSpeedAsMaxFallingSpeed = true;
//    public float maxFallingSpeed = 30f;
//    public float slowDownOverMaxSpeed = 5f;
//    override public void ValidateCurve()
//    {
//        base.ValidateCurve();
//        if (maxFallingSpeed < 0.01f) maxFallingSpeed = 0.01f;
//        if (useTopSpeedAsMaxFallingSpeed) maxFallingSpeed = topSpeed;
//    }

//}

[Serializable]
public class SpeedupCurve : CurveHolder
{
    public override int Orientation { get { return 0; } }
    public override bool Rising { get { return true; } }


}

[Serializable]
public class SlowdownCurve : CurveHolder
{
    public override int Orientation { get { return 0; } }
    public override bool Rising { get { return false; } }

}

//[Serializable]
//public enum AutoCalculate
//{
//    TimeNeeded,
//    Distance,
//    TopSpeed
//}


