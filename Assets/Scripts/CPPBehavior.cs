using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Runtime.InteropServices;

[StructLayout(LayoutKind.Sequential)]
public struct Object
{
    public int x;
    public int y;
}

[System.Serializable]
public struct Goal
{
    public float x;
    public float y;
    public float z;

}

public struct Vec3Struct
{
    public float x;
    public float y;
    public float z;

}

public class CPPBehavior : MonoBehaviour
{
    public Collider validBounds;
    [SerializeField] Goal goal;
    public Collider robotCollider;

    private IntPtr goalPtr;
    private IntPtr statePtr;
    private Vec3Struct stateStruct;
    private IsStateValidDelegate cb;
    
    private bool spaceKeyPressed = false;
    
    public delegate bool IsStateValidDelegate();
    public bool IsStateValid()
    {
        stateStruct = (Vec3Struct) Marshal.PtrToStructure(statePtr, typeof(Vec3Struct));
        Vector3 state = new Vector3(stateStruct.x, stateStruct.y, stateStruct.z);
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
                 continue;
             }
             bool doesIntersect = robotCollider.bounds.Intersects(collider.bounds);
             if (doesIntersect)
             {
                 valid = false;
                 break;
             }
        }

         if (valid)
         {
            // Debug.Log("No intersect ");
         }
         else
         {
             // Debug.Log("Intersect ");
         }

         
        return valid;
    }
    
    //#if UNITY_EDITOR_LINUX
    [DllImport("libUnityLib.so", EntryPoint = "RRTSearch", CallingConvention = CallingConvention.Cdecl)]
    //#endif

    public static extern void RRTSearch(IntPtr gPtr, IntPtr sPtr, IsStateValidDelegate cb);


    void plan()
    {
        stateStruct.x = transform.position.x;
        stateStruct.y = transform.position.y;
        stateStruct.z = transform.position.z;
            
        Marshal.StructureToPtr(goal, goalPtr, false);
        Marshal.StructureToPtr(stateStruct, statePtr, false);
        
        Debug.Log("Starting Plan");
        RRTSearch(goalPtr, statePtr, cb);
        int o = 0;
        Debug.Log("Done Plan");
    }

    void Start()
    {
      // TODO need to free
        goalPtr = Marshal.AllocHGlobal(Marshal.SizeOf(typeof(Goal)));
        statePtr = Marshal.AllocHGlobal(Marshal.SizeOf(typeof(Vec3Struct)));
        cb = new IsStateValidDelegate(IsStateValid);

        robotCollider = GetComponent<Collider>();


        //IntPtr objectsPtr = Marshal.AllocHGlobal(count*Marshal.SizeOf(typeof(Object)));
        //for (var i = 0; i < count; i++)
        //{
        //    Marshal.StructureToPtr(objects[i], objectsPtr+ i*Marshal.SizeOf(typeof(Object)), false);
        //}
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
            plan();
        }

    }
}
