using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[Serializable]
public class TurnUpwardsField
{
    [SerializeField]
    float toleratedAngle = 5f;
    [SerializeField]
    float maxRotSpeed = 60f;
    [SerializeField]
    float maxRotSpeedChange = 100f;
    [SerializeField]
    float downwardAcceleration = 0f;
}
