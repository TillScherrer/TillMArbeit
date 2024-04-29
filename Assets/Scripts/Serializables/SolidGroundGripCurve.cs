
using System;
using System.Collections.Generic;
using System.Linq;
using UnityEditor;
using UnityEngine;

[Serializable]
public class GripCurve
{

    //public float maxGrip = 2;

    //public float definedUpToSlideSpeed = 5;

    public AnimationCurve curve = new AnimationCurve();


    //public float GetBestSlideSpeed()
    //{
    //    //wenn Solid Ground kurve!!!!!!!!
    //    return (curve.keys[1].time / definedUpToSlideSpeed);
    //}

    public virtual void SetKeyNumber(int keyNumber)
    {
        while (curve.length != keyNumber)
        {
            if (curve.length < keyNumber)
            {
                float usedTime = curve.length < 1 ? 0 : curve.keys[curve.length - 1].time+0.1f;
                curve.AddKey( usedTime, 0.5f);
            }
            else
            {
                curve.RemoveKey(curve.length - 1);
            }
        }
    }

  

    public virtual void ValidateCurve()
    {
        
    }

}

[Serializable]
public class SolidGrounGrip : GripCurve
{
    public float gripAtNoSlide { get; } = 0.3f;
    public float slipRatioOfMaxGrip { get; } = 0.6f;

    public float slipRatioForFullSlide { get; } = 0.8f;
    public float gripInFullSlide { get; } = 0.5f;

    float prevGripByK2 = 0.5f;
    float prevGripByK3 = 0.5f;
    override public void ValidateCurve()
    {
        SetKeyNumber(4);

        Keyframe k0 = curve.keys[0];
        Keyframe k1 = curve.keys[1];
        Keyframe k2 = curve.keys[2];
        Keyframe k3 = curve.keys[3];

        k0.time = 0;
        if (k0.value < 0.05f) k0.value = 0.05f;
        else if (k0.value > 0.95f) k0.value = 0.95f;
        k1.value = 1;
        if (k1.time < 0.05f) k1.time = 0.05f;
        else if (k1.time > 0.95f) k1.time = 0.95f;
        if (k2.value < 0.05f) k2.value = 0.05f;
        else if (k2.value > 0.95f) k2.value = 0.95f;
        if (k2.time < k1.time) k2.time = k1.time+0.02f;
        else if (k2.time > 0.97f) k2.time = 0.97f;
        k3.time = 1;
        if (k3.value < 0.05f) k3.value = 0.05f;
        else if (k3.value > 0.95f) k3.value = 0.95f;

        k0.outWeight = 0;
        k1.inWeight = 0;
        k1.outWeight = 0;
        k2.inWeight = 0;
        k2.outWeight = 0;
        k3.inWeight = 0;

        k0.outTangent = 0;
        k1.inTangent = 0;
        k1.outTangent = 0;
        k2.inTangent = 0;
        k2.outTangent = 0;
        k3.inTangent = 0;

        k0.weightedMode = WeightedMode.Both;
        k1.weightedMode = WeightedMode.Both;
        k2.weightedMode = WeightedMode.Both;
        k3.weightedMode = WeightedMode.Both;

        if (prevGripByK2 != k2.value) k3.value = k2.value;      //Slip Ratio of full Slide was changed by k2 (=change k3)
        else if (prevGripByK3 != k3.value) k2.value = k3.value; //Slip Ratio of full Slide was changed by k2 (=change k2)
        prevGripByK2 = k2.value;
        prevGripByK3 = k3.value;

        curve.MoveKey(0, k0);
        curve.MoveKey(1, k1);
        curve.MoveKey(2, k2);
        curve.MoveKey(3, k3);
        //Debug.Log("validated SolidGroundCurve");
    }
}

[Serializable]
public class LooseGrounGrip : GripCurve
{
    public float gripAtNoSlide { get; } = 0.3f;

    override public void ValidateCurve()
    {
        SetKeyNumber(2);

        float minGradient = 0.001f;
        for (int i = 0; i < curve.length; i++)
        {
            Keyframe key = curve.keys[i];
            key.weightedMode = WeightedMode.Both;


            //assign vars and adjust tangents' angles
            if (i == 0)
            {
                key.time = 0;
                if (key.value < 0.00f) key.value = 0.00f;
                else if (key.value > 0.95f) key.value = 0.95f;
                if (key.outTangent < minGradient) key.outTangent = minGradient;
                else if (key.outTangent > 1 / minGradient) key.outTangent = 1 / minGradient;
            }
            if (i == 1)
            {
                key.time = 1;
                key.value = 1;
                if (key.inTangent < minGradient) key.inTangent = minGradient;
                else if (key.inTangent > 1 / minGradient) key.inTangent = 1 / minGradient;
            }


            //adjust tangents' weights
            if (i == 0)
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
            if (i == 1)
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
    }
}



