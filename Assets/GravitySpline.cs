using UnityEngine;
using UnityEngine.Splines;

public class GravitySpline : MonoBehaviour
{
    [SerializeField] float range = 5;
    [SerializeField] float splineGravity = -9.81f;
    [SerializeField]
    [Tooltip("Usualy the gravity points to the spline-knots up Vector for a flat 3D-Spline. With this option activated you can create a tube shaped 3D-Spline and create gravity towards its center (or away from the center)")]
    bool gravityTowardsSplineCenter = false;

    //WARNING: too high accuracy or number of iterations creates rounding to zero, resulting in an endless loop; accuracy of 20 with 4 iterations is save
    readonly int accuracy = 20; //needs at least 2, low accuracy with high iterations would be very efficient, but increase the chance to miss a global minimum for a local minimum
    readonly int evaluationIterations = 4; //needs at least 1 (but needs practically more)

    SplineContainer spline;

    // Start is called before the first frame update
    void Start()
    {
        spline = GetComponent<SplineContainer>();

        if(splineGravity == 0)
        {
            Debug.LogError("spline Gravity is not allowed to be zero! (it is allowed to be negative through)");
        }
    }

    public (bool isInRange, float distance, Vector3 gravity) EvaluateGravityOnPoint(Vector3 fromPos)
    {
        Vector3 closestPos = Vector3.positiveInfinity;
        Vector3 upVectorDir = Vector3.zero;

        float totalShortestDist = Mathf.Infinity;

        foreach(Spline sp in  spline.Splines)
        {
            float shortestDist = Mathf.Infinity;
            float tOfBest = -1;

            float tCenter = 0.5f;
            

            for(int i = 0; i< evaluationIterations; i++)
            {
                float tRange = Mathf.Pow(1f/(float)accuracy, i);
                //WARNING: too high accuracy or number of iterations creates rounding to zero, resulting in an endless loop; accuracy of 20 with 4 iterations is save
                for (float t = tCenter - 0.5f * tRange; t <= tCenter + 0.5f * tRange; t += tRange / (float)accuracy) 
                {
                    if (sp.Closed)
                    {
                        // get correct values at closed splines
                        if (t < 0) t += 1;
                        else if (t > 1) t -= 1;
                    }
                    else
                    {
                        // prevent reading invalid values at open splines
                        if (t < 0 || t > 1) continue;
                    }

                    float testedDist = (transform.TransformPoint((Vector3)sp.EvaluatePosition(t)) - fromPos).magnitude;

                    //Debug.Log("evaluated t = " + t);
                    //Debug.DrawRay((Vector3)sp.EvaluatePosition((float)t), Vector3.up * 8);

                    if (testedDist < shortestDist)
                    {
                        shortestDist = testedDist;
                        tOfBest = (float)t;

                    }
                }
                tCenter = tOfBest;
            }

            if(shortestDist < totalShortestDist)
            {
                totalShortestDist = shortestDist;
                closestPos = transform.TransformPoint(sp.EvaluatePosition(tOfBest));
                upVectorDir = sp.EvaluateUpVector(tOfBest);
            }
            
        }

        Vector3 fromClosestPointToReceiverPos = fromPos - closestPos;
        Vector3 gravity = gravityTowardsSplineCenter ? fromClosestPointToReceiverPos.normalized * splineGravity : upVectorDir.normalized * splineGravity;
        float distance = fromClosestPointToReceiverPos.magnitude;
        bool isInRange = distance <= range;


        return (isInRange, distance, gravity);
    }
}
