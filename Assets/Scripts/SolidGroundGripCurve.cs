
using System;
using System.Collections.Generic;
using System.Linq;
using UnityEditor;
using UnityEngine;

[Serializable]
public class GripCurve
{

    public float maxGrip = 2;

    public float definedUpToSlideSpeed = 5;

    public AnimationCurve curve = new AnimationCurve();


    public float GetBestSlideSpeed()
    {
        //wenn Solid Ground kurve!!!!!!!!
        return (curve.keys[1].time / definedUpToSlideSpeed);
    }

    public virtual void SetKeyNumber(int keyNumber)
    {
        while (curve.length != 3)
        {
            if (curve.length < 3)
            {
                curve.AddKey(curve.keys[curve.length - 1].time + 0.1f, 0.5f);
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
public class SolidGrounGrip : GripCurve
{
    public float gripAtNoSlide { get; } = 0.3f;
    public float maxGripAtRelSlide { get; } = 0.6f;
    public float gripInFullSlide { get; } = 0.5f;
    override public void ValidateCurve()
    {
        SetKeyNumber(3);

        curve.keys[0].time = 0;
        if (curve.keys[0].value < 0.05f) curve.keys[0].value = 0.05f;
        else if (curve.keys[0].value > 0.95f) curve.keys[0].value = 0.95f;
        curve.keys[1].value = 1;
        if (curve.keys[1].time < 0.05f) curve.keys[1].time = 0.05f;
        else if (curve.keys[1].time > 0.95f) curve.keys[1].time = 0.95f;
        curve.keys[2].time = 1;
        if (curve.keys[2].value < 0.05f) curve.keys[2].value = 0.05f;
        else if (curve.keys[2].value > 0.95f) curve.keys[2].value = 0.95f;

        //curve.keys[0].outWeight = 0;
        //curve.keys[1].inWeight = 0;
        //curve.keys[1].outWeight = 0;
        //curve.keys[2].inWeight = 0;

        curve.keys[0].weightedMode = WeightedMode.None;
        curve.keys[1].weightedMode = WeightedMode.None;
        curve.keys[2].weightedMode = WeightedMode.None;

    }
}


public class LooseGrounGrip : GripCurve
{
    public float gripAtNoSlide { get; } = 0.3f;

    override public void ValidateCurve()
    {
        SetKeyNumber(2);

        curve.keys[0].time = 0;
        curve.keys[1].time = 1;
        curve.keys[1].value = 1;

        //float maxTangent = 1 - curve.keys[0].value;
        //if(curve.keys[0].outTangent > maxTangent) curve.keys[0].outTangent = maxTangent;
        //if (curve.keys[1].inTangent < -maxTangent) curve.keys[1].inTangent = -maxTangent;

        //if (curve.keys[0].outTangent + curve.keys[0].value > 1) curve.keys[0].outTangent = 1 - curve.keys[0].value;
        //if (1 - curve.keys[1].inTangent < curve.keys[0].value) curve.keys[0].outTangent = 1 - curve.keys[0].value;
        //curve.keys[0].outWeight = 0;
        //curve.keys[1].inWeight = 0;

        float minGradient = 0;
        for (int i = 0; i < curve.length; i++)
        {
            Keyframe key = curve.keys[i];
            key.weightedMode = WeightedMode.Both;


            //assign vars and adjust tangents' angles
            if (i < curve.length - 1)
            {
                    if (key.outTangent < minGradient) key.outTangent = minGradient;
                    else if (key.outTangent > 1 / minGradient) key.outTangent = 1 / minGradient;
            }
            if (i > 0)
            {

                //winkel anpassen
                    if (key.inTangent < minGradient) key.inTangent = minGradient;
                    else if (key.inTangent > 1 / minGradient) key.inTangent = 1 / minGradient;
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
    }
}



