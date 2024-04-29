
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


    [HideInInspector]
    public float[] rawAccelerationValues;
    private readonly int accuracyOfAccelerationValues = 100;

    public float GetAccelerationValueForSpeed(float speed)
    {
        speed = Mathf.Abs(speed);
        if (speed > topSpeed) return 0;
        float timeInCurve = GetTimeInCurve(speed);

        float sampleValue = timeInCurve * (accuracyOfAccelerationValues + 1);
        int fromSample = Mathf.FloorToInt(sampleValue);
        int toSample = Mathf.CeilToInt(sampleValue);
        if (fromSample > accuracyOfAccelerationValues) fromSample = accuracyOfAccelerationValues;
        if (toSample > accuracyOfAccelerationValues) toSample = accuracyOfAccelerationValues;
        //Debug.Log("from sample= " + fromSample);
        //Debug.Log("to sample= " + toSample);
        //Debug.Log("accel at sample= " + rawAccelerationValues[fromSample]);
        float scaleBetween = sampleValue - fromSample; //scale from 0 to 1 between the two samples
        return Mathf.Lerp(rawAccelerationValues[fromSample], rawAccelerationValues[toSample], scaleBetween) * topSpeed / timeNeeded;
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
        rawAccelerationValues = new float[accuracyOfAccelerationValues + 1];
        rawAccelerationValues[0] = (curve.Evaluate((float)1 / accuracyOfAccelerationValues) - curve.Evaluate((float)(0) / accuracyOfAccelerationValues)) * accuracyOfAccelerationValues;
        for (int i = 1; i <= accuracyOfAccelerationValues; i++)
        {
            rawAccelerationValues[i] = (curve.Evaluate((float)i / accuracyOfAccelerationValues) - curve.Evaluate((float)(i - 1) / accuracyOfAccelerationValues)) * accuracyOfAccelerationValues;
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
}

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


