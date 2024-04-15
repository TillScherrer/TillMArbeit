using UnityEngine;
using UnityEngine.Splines;

public class GravitySpline : MonoBehaviour
{
    [SerializeField] float range = 5;
    [SerializeField] float gravityTowardsSpline = 5;

    //WARNING: too high accuracy or number of iterations creates rounding to zero, resulting in an endless loop; accuracy of 20 with 4 iterations is save
    readonly int accuracy = 20; //needs at least 2, low accuracy with high iterations would be very efficient, but increase the chance to miss a global minimum for a local minimum
    readonly int evaluationIterations = 4; //needs at least 1 (but needs practically more)

    SplineContainer spline;

    // Start is called before the first frame update
    void Start()
    {
        spline = GetComponent<SplineContainer>();

        if(gravityTowardsSpline == 0)
        {
            Debug.LogError("spline Gravity is not allowed to be zero! (it is allowed to be negative through)");
        }

        Debug.Log("transform poin 000 to " + transform.TransformPoint(Vector3.zero));
    }

    // Update is called once per frame
    //void Update()
    //{
    //    //Vector3 pos;
    //    for(float t = 0; t<=1; t+=0.01f)
    //    {
    //        //pos = spline.EvaluatePosition(t);
    //        Debug.DrawRay(spline.EvaluatePosition(t), Vector3.up);
    //        Debug.Log(spline.EvaluatePosition(t));
    //    }


        
    //}



    public (bool isInRange, float distance, Vector3 gravity) EvaluateGravityOnPoint(Vector3 fromPos)
    {
        Vector3 closestPos = Vector3.positiveInfinity;

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
            }
            
        }

        Vector3 fromPosToClosestPoint = closestPos - fromPos;
        Vector3 gravity = fromPosToClosestPoint.normalized * gravityTowardsSpline;
        float distance = fromPosToClosestPoint.magnitude;
        bool isInRange = distance <= range;


        return (isInRange, distance, gravity);
    }
}
