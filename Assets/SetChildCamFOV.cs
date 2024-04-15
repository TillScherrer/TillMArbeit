using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UIElements;

public class SetChildCamFOV : MonoBehaviour
{
    Camera meCam;
    Camera childCam;

    // Start is called before the first frame update
    void Start()
    {
        meCam = GetComponent<Camera>();
        childCam = GetComponentsInChildren<Camera>()[1];
        if (meCam == null || childCam == null)
        {
            Debug.LogError("FOVBySpeed skript on camera did not find the camera component and/or its child's camera component");
        }
    }

    // Update is called once per frame
    void Update()
    {
        //Debug.Log("my FOV = " + meCam.fieldOfView);
        childCam.fieldOfView = meCam.fieldOfView;
        //Debug.Log("child FOV = " + childCam.fieldOfView);
    }
}
