using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraController : MonoBehaviour
{
    public Transform targetDrone;
    public Transform targetCam;
    public float smoothing = 60f;

    Vector3 offset;

    // Start is called before the first frame update
    void Start()
    {
        offset = transform.position - targetCam.position;
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    // 如果启用 Behaviour，则在每一帧都将调用 LateUpdate
    private void LateUpdate()
    {
        Vector3 targetCamPos = targetCam.position + offset;
        transform.position = targetCamPos;
        Quaternion rot1 = transform.rotation;
        transform.LookAt(targetDrone);
        //Quaternion rot2 = transform.rotation;
        //transform.rotation = Quaternion.Lerp(rot1, rot2, 5f * Time.deltaTime);
    }
}
