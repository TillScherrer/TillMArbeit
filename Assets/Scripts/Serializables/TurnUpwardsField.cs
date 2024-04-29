using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[Serializable]
public class TurnUpwardsField
{
    public float toleratedAngle = 5f;
    public float maxRotSpeed = 60f;
    public float maxRotSpeedChange = 100f;
    public float downwardAcceleration = 0f;
}
