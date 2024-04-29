using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.UIElements;

public class CustomGravityReciver : MonoBehaviour
{
    [SerializeField]
    Vector3 basicGravity = new Vector3(0,-9.81f,0);

    [SerializeField]
    GravitySplineManager gravitySplineManager;
    [SerializeField]
    bool useGravitySplines = true;
    [SerializeField]
    bool ignoreBaseGravityWithinGravitySpline = true;

    [SerializeField]
    [Tooltip("When the Gravity changes, the same direction change is applied to your velocity. Extreme gravity changes do not trigger this")]
    bool curveSpace = false;


    Vector3 currentCustomGravity = Vector3.zero;

    Rigidbody rb;


    Vector3 previousGravityDir = Physics.gravity;


    public Vector3 CurrentCustomGravity { get => currentCustomGravity;}
    public bool CurveSpace { get => curveSpace;}

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



        if (curveSpace)
        {
            float angle = Vector3.Angle(previousGravityDir, currentCustomGravity);
            if (angle != 0 && angle < 10)
            {
                rb.velocity = Quaternion.FromToRotation(previousGravityDir, currentCustomGravity) * rb.velocity;
            }
        }
        previousGravityDir = currentCustomGravity.normalized;
    }


    public Vector3 GetGravityAtMyCurrentPosition()
    {
        return GetGravityForPosition(transform.position);
    }

    public Vector3 GetGravityForPosition(Vector3 position)
    {
        Vector3 gravity = Vector3.zero;

        bool useBaseGravity = true;

        // conditionally apply gravity spline gravity
        if (useGravitySplines)
        {
            // splineGravity is zero if no spline in range was found
            Vector3 splineGravity = gravitySplineManager.GetGravityForPosition(position);

            if (splineGravity.magnitude > 0)
            {
                gravity += splineGravity;
                useBaseGravity = !ignoreBaseGravityWithinGravitySpline;
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
}
