using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Runtime.InteropServices;
using UnityEngine.Rendering;

public class CPPBehavior : MonoBehaviour
{
    private bool spaceKeyPressed = false;
    private LineRenderer lineRenderer;
    public Collider validBounds;
    private Collider robotCollider;
    // OMPL 
    [SerializeField] State goal;
    private UnityOMPLInterface OMLInterface;
    
    void Start()
    {
        robotCollider = GetComponent<Collider>();
        OMLInterface = new UnityOMPLInterface(IsStateValid, GetState, RestoreState);
        
        lineRenderer = GetComponent<LineRenderer>();
        lineRenderer.positionCount = 0;
        lineRenderer.startColor = Color.red;
        lineRenderer.endColor = Color.red;
        lineRenderer.startWidth = 0.1f;
        lineRenderer.endWidth = 0.1f;
    }

    // Update is called once per frame
    void Update()
    {
        if (!Input.GetKey(KeyCode.Space))
        {
            spaceKeyPressed = false;
        }

        if (Input.GetKey(KeyCode.Space) && !spaceKeyPressed)
        {
            spaceKeyPressed = true;
            Debug.Log("Plan Start");
            var points = OMLInterface.plan(goal);
            lineRenderer.positionCount = points.Count;
            int ind = 0;
            foreach (var point in points)
            {
                lineRenderer.SetPosition(ind, new Vector3(point.x, point.y, point.z));
                ind++;
            }
            
            Debug.Log("Done Plan");
        }

    }

    public State GetState()
    {
        State stateStruct = new State();
        stateStruct.x = gameObject.transform.position.x;
        stateStruct.y = gameObject.transform.position.y;
        stateStruct.z = gameObject.transform.position.z;
        return stateStruct; 
    }
    public void RestoreState(State state)
    {
        Vector3 pos = gameObject.transform.position;
        pos.x = state.x;
        pos.y = state.y;
        pos.z = state.z;
        gameObject.transform.position = pos;
    }

    public bool IsStateValid(Vector3 state)
    {
        gameObject.transform.position = state;
        Physics.SyncTransforms();
        
        float robotRadius = 2.0f;
        bool valid = true;
        Collider[] colliders = Physics.OverlapSphere(state, robotRadius);
        foreach (Collider collider in colliders) {
            if (collider.gameObject == gameObject)
            {
                continue;
            }
            if (collider == validBounds)
            {
                if (!robotCollider.bounds.Intersects(collider.bounds))
                {
                    valid = false;
                    break;
                }
                continue;
            }
            bool doesIntersect = robotCollider.bounds.Intersects(collider.bounds);
            if (doesIntersect)
            {
                valid = false;
                break;
            }
        }

        return valid;
    }

}
