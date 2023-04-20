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

public class UnityOMPLInterface
{
    private IntPtr goalPtr;
    private IntPtr statePtr;
    private IntPtr handle;
    private IntPtr pathPtr;
    private IsStateValidDelegate cb;
    private GameObject gameObject;
    private Func<Vector3, bool> IsStateValidExtern;
    public UnityOMPLInterface(GameObject gameObjectIn, Func<Vector3, bool> IsStateValidExternIn)
    {
        gameObject = gameObjectIn;
        goalPtr = Marshal.AllocHGlobal(Marshal.SizeOf(typeof(Goal)));
        statePtr = Marshal.AllocHGlobal(Marshal.SizeOf(typeof(Vec3Struct)));
        cb = new IsStateValidDelegate(IsStateValid);
        IsStateValidExtern = IsStateValidExternIn;
        handle = Init();
    }
    ~UnityOMPLInterface()
    { 
        Destroy(handle);
    }

    public Vec3Struct[] plan(Goal goal)
    {
        Vec3Struct stateStruct = new Vec3Struct();
        stateStruct.x = gameObject.transform.position.x;
        stateStruct.y = gameObject.transform.position.y;
        stateStruct.z = gameObject.transform.position.z;

        Marshal.StructureToPtr(goal, goalPtr, false);
        Marshal.StructureToPtr(stateStruct, statePtr, false);
        
        Vector3 tmp = gameObject.transform.position;
        bool solved = RRTSearch(handle, goalPtr, statePtr, cb, ref pathPtr);
        gameObject.transform.position = tmp;

        return new Vec3Struct[] { };
    }
    private delegate bool IsStateValidDelegate();

    private bool IsStateValid()
    {
        Vec3Struct stateStruct = (Vec3Struct)Marshal.PtrToStructure(statePtr, typeof(Vec3Struct));
        Vector3 state = new Vector3(stateStruct.x, stateStruct.y, stateStruct.z);
        return IsStateValidExtern(state);
    }

    [DllImport("libUnityLib.so", EntryPoint = "Init", CallingConvention = CallingConvention.Cdecl)] 
    private static extern IntPtr Init();
    [DllImport("libUnityLib.so", EntryPoint = "Destroy", CallingConvention = CallingConvention.Cdecl)] 
    private static extern void Destroy(IntPtr handle);
    [DllImport("libUnityLib.so", EntryPoint = "RRTSearch", CallingConvention = CallingConvention.Cdecl)] 
    private static extern bool RRTSearch(IntPtr handle, IntPtr gPtr, IntPtr sPtr, IsStateValidDelegate cb, ref IntPtr path);

}

public class CPPBehavior : MonoBehaviour
{
    public Collider validBounds;
    [SerializeField] Goal goal;
    private Collider robotCollider;
    private UnityOMPLInterface OMLInterface;

   
    private Vec3Struct stateStruct;

    private bool spaceKeyPressed = false;
    
    public delegate bool IsStateValidDelegate();
    public bool IsStateValid(Vector3 state)
    {
        
        //stateStruct = (Vec3Struct) Marshal.PtrToStructure(statePtr, typeof(Vec3Struct));
        //Vector3 state = new Vector3(stateStruct.x, stateStruct.y, stateStruct.z);
        gameObject.transform.position = state;
        Physics.SyncTransforms();
        // Debug.Log("isValidCalled: " + state.x + ", "+state.y);
        
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
    
   

    void plan()
    {
        //stateStruct.x = transform.position.x;
        //stateStruct.y = transform.position.y;
        //stateStruct.z = transform.position.z;
            
        //Marshal.StructureToPtr(goal, goalPtr, false);
        //Marshal.StructureToPtr(stateStruct, statePtr, false);
        
        //Debug.Log("Starting Plan");
        //Vector3 tmp = gameObject.transform.position;

        OMLInterface.plan(goal);
        //IntPtr solPtr = RRTSearch(goalPtr, statePtr, cb);
        //gameObject.transform.position = tmp;
        //Physics.SyncTransforms();

        int o = 0;
        Debug.Log("Done Plan");
    }

    void Start()
    {
      // TODO need to free
       
        robotCollider = GetComponent<Collider>();
        OMLInterface = new UnityOMPLInterface(gameObject, IsStateValid);


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
            OMLInterface.plan(goal);
            Debug.Log("Done Plan");
        }

    }
}
