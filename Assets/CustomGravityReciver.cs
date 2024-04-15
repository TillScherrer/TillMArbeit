using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UIElements;

public class CustomGravityReciver : MonoBehaviour
{
    [SerializeField]
    Vector3 basicGravity = new Vector3(0,-9.81f,0);

    [SerializeField]
    float towardsGroundNormalGravity = 9.81f;

    [SerializeField]
    bool ignoreBaseGravityWithinGroundNormalGravity = true;

    [SerializeField]
    GravitySplineManager gravitySplineManager;
    [SerializeField]
    bool useGravitySplines = true;
    [SerializeField]
    bool ignoreBaseGravityWithinGravitySpline = true;
    [SerializeField]
    bool ignoreGroundNormalGravityWithinGravitySpline = true;

    Vector3 currentCustomGravity = Vector3.zero;

    Rigidbody rb;

    public Vector3 CurrentCustomGravity { get => currentCustomGravity;}

    // Start is called before the first frame update
    void Start()
    {
        rb = GetComponent<Rigidbody>();
        if(rb == null)
        {
            Debug.LogError("Rigidbody missing! The CustomGravityReciver component can only be used with a rigidbody.");
        }
        else if(rb.useGravity == true)
        {
            Debug.Log("The CustomGravityReciver deactivated the rigidbody's default use-Gravity-option to replace it.");
            rb.useGravity = false;
        }

        if(useGravitySplines && gravitySplineManager == null)
        {
            Debug.Log("A CustomGravityReciver wants to use Gravity Splines, but you did not assign the parent holding these Splines");
        }
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    private void FixedUpdate()
    {
        currentCustomGravity = GetGravityAtMyCurrentPosition();
        rb.velocity += currentCustomGravity * Time.fixedDeltaTime;
    }


    public Vector3 GetGravityAtMyCurrentPosition()
    {
        return GetGravityForPosition(transform.position);
    }

    public Vector3 GetGravityForPosition(Vector3 position)
    {
        Vector3 gravity = Vector3.zero;

        bool useBaseGravity = true;
        bool useGroundNormalGravity = true;

        // conditionally apply gravity spline gravity
        if (useGravitySplines)
        {
            // splineGravity is zero if no spline in range was found
            Vector3 splineGravity = gravitySplineManager.GetGravityForPosition(position);

            if (splineGravity.magnitude > 0)
            {
                gravity += splineGravity;
                useBaseGravity = !ignoreBaseGravityWithinGravitySpline;
                useGroundNormalGravity = !ignoreGroundNormalGravityWithinGravitySpline;
            }
        }

        // conditionally apply ground normal gravity
        if (useGroundNormalGravity && towardsGroundNormalGravity!=0)
        {
            (bool foundGroudnNormal, Vector3 gravity) groundNormalGravity = SearchForGroundNormalGravity(position);

            if (groundNormalGravity.foundGroudnNormal)
            {
                gravity += groundNormalGravity.gravity;

                if(useBaseGravity == true)
                {
                    useBaseGravity = !ignoreBaseGravityWithinGroundNormalGravity;
                }
            }
        }

        // conditionally apply base gravity
        if (useBaseGravity)
        {
            gravity += basicGravity;
        }

        Debug.DrawRay(position, gravity, Color.black);
        return gravity;
    }




    (bool foundGravitySpline, Vector3 gravity) SearchForGroundNormalGravity(Vector3 position)
    {
        bool found = false;
        Vector3 GroundNormalGravity = Vector3.zero;
        //todo: nach gravity spline suchen



        return (found, GroundNormalGravity);
    }

}
