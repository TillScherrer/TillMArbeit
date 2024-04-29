using Cinemachine;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraFollow : MonoBehaviour
{
    public Transform player;
    [SerializeField] private float maximumDistance = 30;
    [SerializeField] private float minimumDistance = 5;
    [SerializeField] private float yDistToPlayer = 15f;
    [SerializeField] private float camFollowSpeed = 10;
    [SerializeField] private float camFleeSpeed = 12;

    Rigidbody rb;
    CinemachineVirtualCamera cam;
    CustomGravityReciver customGravity;
    float prevZRot = 0;

    private void Start()
    {
        rb = player.GetComponent<Rigidbody>();
        customGravity = player.GetComponent<CustomGravityReciver>();
        cam = GetComponent<CinemachineVirtualCamera>();
    }

    private void Update()
    {
        //use gravity or custom gravity as camera downward orientation
        Vector3 gravity = customGravity != null ? customGravity.CurrentCustomGravity : Physics.gravity;
        if (gravity == Vector3.zero) gravity = Vector3.down; //down as default orientiation without gravity


        Vector3 camToCar = player.position - transform.position;

        float distToCarInGravityDir = Vector3.Dot(gravity.normalized, camToCar);
        Vector3 camToCarInGravityDir = gravity.normalized * distToCarInGravityDir;
        float distToPlayerWithoutLocalYDiff = (camToCar - camToCarInGravityDir).magnitude;

        Vector3 carVelocity = rb.velocity;
        float driveDirToCamAngle = Vector2.SignedAngle(new Vector2(carVelocity.x, carVelocity.z), new Vector2(camToCar.x, camToCar.z));
        //make cam dodge to its right when the angle is negative and to its left when its positive (only needed for small angles)
        float scale = Mathf.Abs(driveDirToCamAngle) / 180f;
        int angleSign = driveDirToCamAngle > 0 ? 1 : -1;
        transform.position += transform.rotation * Vector3.left * angleSign * Time.deltaTime * rb.velocity.magnitude * scale * 2.5f;

        //Debug.Log("cam Update2");


        if (distToPlayerWithoutLocalYDiff > maximumDistance)
        {
            transform.position = Vector3.MoveTowards(transform.position, player.position - camToCarInGravityDir, (distToPlayerWithoutLocalYDiff - maximumDistance) * (Time.deltaTime < 1f / 20 ? Time.deltaTime : 1f / 20) * camFollowSpeed);
        }
        else if (distToPlayerWithoutLocalYDiff < minimumDistance)
        {


            Vector3 minPoint = CalculateMinimumPoint(player.position);
            transform.position = Vector3.MoveTowards(transform.position, minPoint, Time.deltaTime * camFleeSpeed);

        }

        float currentLocalY = Vector3.Dot(camToCar, gravity.normalized);
        float updatedLocalY = Mathf.Lerp(yDistToPlayer, currentLocalY, Mathf.Pow(0.65f, Time.deltaTime));//reaches 35% of plannedHeight over 1 second

        Vector3 posWithZeroLocalY = transform.position + currentLocalY * gravity;
        transform.position = posWithZeroLocalY - updatedLocalY * gravity;

        float wantedZRot = Quaternion.LookRotation(camToCar, -gravity).eulerAngles.z;
        if (wantedZRot > 180) wantedZRot -= 360;
        else if (wantedZRot < -180) wantedZRot += 360;

        if (wantedZRot > 90 && prevZRot < -90)
        {
            prevZRot += 360;
        }
        else if (wantedZRot < -90 && prevZRot > 90)
        {
            prevZRot -= 360;
        }
        float updatedRot = Mathf.Lerp(wantedZRot, prevZRot, Mathf.Pow(0.35f, Time.deltaTime));//reaches 65% of wantedZRot over 1 second;
        cam.m_Lens.Dutch = updatedRot > 180 ? updatedRot - 360 : updatedRot < -180 ? updatedRot + 360 : updatedRot;
        prevZRot = updatedRot;

        //adapt FOV
        float speedInGravityDir = Vector3.Dot(gravity.normalized, rb.velocity);
        float speedWithoutLocalY = (rb.velocity - gravity.normalized * distToCarInGravityDir).magnitude;
        float plannedFOVwidth = speedWithoutLocalY < 2 ? 45 : 45 + (Mathf.Sqrt(speedWithoutLocalY - 2) * 5);
        cam.m_Lens.FieldOfView = plannedFOVwidth;
    }


    private Vector3 CalculateMinimumPoint(Vector3 p)
    {
        Vector3 dir = transform.position - p;
        Vector3 point = dir.normalized * minimumDistance;
        //point = new Vector3(point.x, transform.position.y, point.z);
        return point;
    }
}