using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GravitySplineManager : MonoBehaviour
{

    GravitySpline[] gravitySplines;

    // Start is called before the first frame update
    void Start()
    {
        gravitySplines = GetComponentsInChildren<GravitySpline>();
    }

    public Vector3 GetGravityForPosition(Vector3 pos)
    {
        Vector3 gravity = Vector3.zero;
        float smallestDist = Mathf.Infinity;

        foreach(GravitySpline gs in gravitySplines)
        {
            (bool isInRange, float distance, Vector3 gravity) gsStats = gs.EvaluateGravityOnPoint(pos);
            if(gsStats.isInRange && gsStats.distance < smallestDist)
            {
                gravity = gsStats.gravity;
            }

        }
        return gravity;
    }
}
