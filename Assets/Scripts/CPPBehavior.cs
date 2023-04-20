using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Runtime.InteropServices;
using UnityEngine.Rendering;

public class CPPBehavior : MonoBehaviour
{
    private bool spaceKeyPressed = false;
    private bool collided = false;
    private int count = 0;
    private LineRenderer lineRenderer;
    private Collider robotCollider;
    // OMPL 
    [SerializeField] Transform goal;
    public Collider validBounds;
    [SerializeField] double turnRadius;
    [SerializeField] double inflate;
    [SerializeField] double planTime;

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
        
        var mesh = GetComponent<MeshFilter>().mesh;
        var vertices = mesh.vertices;
        int oo = 0;
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
            bool autoSimulation = Physics.autoSimulation;
            Physics.autoSimulation = false;
            spaceKeyPressed = true;
            Debug.Log("Plan Start");
            State goalState = new State();
            goalState.x = goal.position.x;
            goalState.y = goal.position.y;
            goalState.z = 0;
            count = 0;
            var points = OMLInterface.plan(goalState, turnRadius, planTime);
            lineRenderer.positionCount = points.Count;
            int ind = 0;
            foreach (var point in points)
            {
                lineRenderer.SetPosition(ind, new Vector3(point.x, point.y, point.z*0));
                ind++;
            }
            Physics.autoSimulation = true;
            Debug.Log("Done Plan, total count: " + count);
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
        pos.z = 0;
        //pos.z = state.z;
        gameObject.transform.position = pos;
    }

    //void OnCollisionStay(Collision collision)
    //{
    //    Debug.Log("Do something else here");
    //}
    void OnCollisionStay(Collision collision)
    {
        if (collision.gameObject != validBounds.gameObject)
        {
            collided = true;
            //Debug.Log("Collision: " + collision.collider.name);
        }
      //  else
       // {
        //    collided = false;
        //}

    }
    //void OnCollisionStay(Collision collision)
    //{
    //    collided = true;
     //   Debug.Log("Collision: " + collision.collider.name);
   // }
    public bool IsStateValid(Vector3 state)
    {
        count++;
        var newPosition = new Vector3();
        newPosition = state;
        newPosition.z = 0;
        gameObject.transform.position = newPosition;
        Physics.SyncTransforms();

        float robotRadius = 5.0f;
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
                Vector3 closest = collider.ClosestPoint(gameObject.transform.position);
                if (Vector3.Distance(closest, gameObject.transform.position) < inflate)
                {
                    valid = false;
                    break;
                }
                //Physics.Simulate(Time.fixedDeltaTime);
                //if (collided){
                 //   valid = false;
                 //   collided = false;
                  //  break;
                ///}
            }
        }

        return valid;
    }

}
