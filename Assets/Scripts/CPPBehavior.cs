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

    [SerializeField] Vector2 planningCenter;
    [SerializeField] Vector2 planningSize;

    // OMPL 
    [SerializeField] Transform goal;
    public Collider validBounds;
    [SerializeField] double inflate;
    [SerializeField] double planTime;
    [SerializeField] double dirScalar;
    [SerializeField] private double lambda;
    [SerializeField] private double widthScale;
    [SerializeField] private double numBasisPerMeter;

    private UnityOMPLInterface OMLInterface;
    private Vector3 goalOld;
    private Vector3 stateOld;
    
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

        goalOld = new Vector3();
        stateOld = new Vector3();
        
        var mesh = GetComponent<MeshFilter>().mesh;
        var vertices = mesh.vertices;
        
       // colliders = Physics.OverlapSphere(gameObject.transform.position, 20.0f);
    }

    // Update is called once per frame
    void Update()
    {
        Collider[] collidersTest = Physics.OverlapSphere(gameObject.transform.position, 1.0f);
        
        if (!Input.GetKey(KeyCode.Space))
        {
            spaceKeyPressed = false;
        }

        bool cond = Vector3.Distance(goalOld ,goal.position) > 0.01 
                    || Vector3.Distance(stateOld,gameObject.transform.position) > 0.01;
        if (cond || Input.GetKey(KeyCode.Space) && !spaceKeyPressed)
        {
            goalOld = goal.position;
            stateOld = gameObject.transform.position;
            bool autoSimulation = Physics.autoSimulation;
            Physics.autoSimulation = false;
            spaceKeyPressed = true;
            Debug.Log("Plan Start");
            State goalState = new State();
            goalState.x = goal.position.x;
            goalState.y = goal.position.y;
            goalState.theta = ((float) Math.PI)*goal.transform.eulerAngles.z/180.0f;
            count = 0;
            var points = OMLInterface.plan(goalState, planningCenter, planningSize, planTime, dirScalar, lambda, widthScale, numBasisPerMeter);
            lineRenderer.positionCount = points.Count;
            int ind = 0;
            foreach (var point in points)
            {
                lineRenderer.SetPosition(ind, new Vector3(point.x, point.y, 0));
                ind++;
            }
            Physics.autoSimulation = true;
            Debug.Log("Done Plan, total count: " + count);
        }

    }
    
    void OnDrawGizmosSelected()
    {
        // Draw a semitransparent red cube at the transforms position
        Gizmos.color = new Color(0, 1, 0, 0.15f);
        Vector3 pos = new Vector3();
        pos.x = planningCenter.x;
        pos.y = planningCenter.y;
        Vector3 size = new Vector3();
        size.x = planningSize.x;
        size.y = planningSize.y;
        Gizmos.DrawCube(pos, size);
    }

    public State GetState()
    {
        State stateStruct = new State();
        stateStruct.x = gameObject.transform.position.x;
        stateStruct.y = gameObject.transform.position.y;
        stateStruct.theta = ((float) Math.PI)*gameObject.transform.eulerAngles.z/180.0f;
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
        state.z = 0; // this is hacky
        count++;
        float robotRadius = 1.0f;
        bool valid = true;
        Collider[] colliders = Physics.OverlapSphere(state, robotRadius);
        foreach (Collider collider in colliders) {
            if (collider.gameObject == gameObject)
            {
                continue;
            }

            if (collider == validBounds)
            {
                continue;
            }
            Vector3 closest = collider.ClosestPoint(state);
            if (Vector3.Distance(closest, state) < inflate)
            {
                valid = false;
                break;
            }
        }

        return valid;
    }

}
