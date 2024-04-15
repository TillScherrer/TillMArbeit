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
    //private Quaternion previousCameraRotation;


    private void Awake()
    {
        //player = GameObject.FindGameObjectWithTag("Player").transform;
        //cameraPreRot = transform.rotation;
    }

    private void Start()
    {
        rb = player.GetComponent<Rigidbody>();
        customGravity = player.GetComponent<CustomGravityReciver>();
        cam = GetComponent<CinemachineVirtualCamera>();
    }

    private void Update()
    {
        //Debug.Log("cam Update1");

        Vector3 camToCar = player.position - transform.position;

        //float distance = CalculatePlayerDistance();
        float horzDistToPlayer = Mathf.Sqrt(camToCar.x * camToCar.x + camToCar.z * camToCar.z);

        Vector3 carVelocity = player.GetComponent<Rigidbody>().velocity;
        float driveDirToCamAngle = Vector2.SignedAngle(new Vector2(carVelocity.x, carVelocity.z), new Vector2(camToCar.x, camToCar.z));
        //make cam dodge to its right when the angle is negative and to its left when its positive (only needed for small angles)
        float scale = Mathf.Abs(driveDirToCamAngle) / 180f;//Mathf.Sqrt(Mathf.Abs(driveDirToCamAngle))/ Mathf.Sqrt(180);
        int angleSign = driveDirToCamAngle > 0 ? 1 : -1;
        transform.position += transform.rotation * Vector3.left * angleSign * Time.deltaTime * player.GetComponent<Rigidbody>().velocity.magnitude * scale * 2.5f;

        //Debug.Log("cam Update2");


        if (horzDistToPlayer > maximumDistance)
        {
            transform.position = Vector3.MoveTowards(transform.position, new Vector3(player.position.x, transform.position.y, player.position.z), (horzDistToPlayer - maximumDistance) * (Time.deltaTime < 1f / 20 ? Time.deltaTime : 1f / 20) * camFollowSpeed);
        }
        else if (horzDistToPlayer < minimumDistance)
        {


            Vector3 minPoint = CalculateMinimumPoint(player.position);
            transform.position = Vector3.MoveTowards(transform.position, minPoint, Time.deltaTime * camFleeSpeed);

        }

        //Debug.Log("cam Update3");
        Vector3 gravity = customGravity != null ? customGravity.CurrentCustomGravity : Physics.gravity;

        float currentLocalY = Vector3.Dot(camToCar, gravity.normalized);

        float updatedLocalY = Mathf.Lerp(yDistToPlayer, currentLocalY, Mathf.Pow(0.65f, Time.deltaTime));//reaches 35% of plannedHeight over 1 second

        Vector3 posWithZeroLocalY = transform.position + currentLocalY * gravity;
        transform.position = posWithZeroLocalY - updatedLocalY * gravity;
        //transform.position = new Vector3(transform.position.x, updatedLocalY, transform.position.z);

        //Debug.Log("cam Update4");

       




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
        float horzSpeed = new Vector2(rb.velocity.x, rb.velocity.z).magnitude;
        float plannedFOVwidth = horzSpeed < 2 ? 45 : 45 + (Mathf.Sqrt(horzSpeed - 2) * 5);
        cam.m_Lens.FieldOfView = plannedFOVwidth;
    }



    private Vector3 CalculateMinimumPoint(Vector3 p)
    {
        Vector3 dir = transform.position - p;
        Vector3 point = dir.normalized * minimumDistance;
        point = new Vector3(point.x, transform.position.y, point.z);
        return point;
    }
}