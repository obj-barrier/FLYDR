using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class StartCameraController : MonoBehaviour
{
    public GameObject ground;

    // Start is called before the first frame update
    void Start()
    {
        transform.LookAt(ground.transform.position);
    }

    // Update is called once per frame
    void Update()
    {
        transform.LookAt(ground.transform.position);
        transform.RotateAround(ground.transform.position, Vector3.up, 0.1f);
    }
}
