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

    //private Quaternion previousCameraRotation;


    private void Awake()
    {
        //player = GameObject.FindGameObjectWithTag("Player").transform;
        //cameraPreRot = transform.rotation;
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


        float camHeight = Mathf.Lerp(player.position.y + yDistToPlayer, transform.position.y, Mathf.Pow(0.2f, Time.deltaTime));//reaches 80% of plannedHeight over 1 second
        transform.position = new Vector3(transform.position.x, camHeight, transform.position.z);

        //Debug.Log("cam Update4");

    }



    private Vector3 CalculateMinimumPoint(Vector3 p)
    {
        Vector3 dir = transform.position - p;
        Vector3 point = dir.normalized * minimumDistance;
        point = new Vector3(point.x, transform.position.y, point.z);
        return point;
    }
}