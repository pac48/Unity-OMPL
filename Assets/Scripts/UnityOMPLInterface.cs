using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Runtime.InteropServices;
using Random = UnityEngine.Random;

[StructLayout(LayoutKind.Sequential)]
[System.Serializable] 
public struct State
{
    public float x;
    public float y;
    public float theta;

}

[StructLayout(LayoutKind.Sequential)]
public struct CArray {
    public IntPtr data;
    public int size;
};

public delegate bool IsStateValidDelegate(ref State state);
public delegate bool ClosestPointDelegate(ref State state, ref State point);
public delegate bool ClosestPointExternalDelegate(Vector3 state, ref Vector3 point);

[StructLayout(LayoutKind.Sequential)]
public struct RRTSearchInput
{
    public State goal;
    public State state;
    public double radius;
    public State planningCenter;
    public State planningSize;
    public double planTime;
    public int pathResolution;
	public double dirScalar;
	public double lambda;
	public double widthScale;
	public double numBasisPerMeter;
	public IsStateValidDelegate validState;
	public ClosestPointDelegate closestPoint;
}


[StructLayout(LayoutKind.Sequential)]
public struct RRTSearchOutput
{
    public CArray path;
}

public class UnityOMPLInterface
{
    private IntPtr goalPtr;
    private IntPtr statePtr;
    private IntPtr handle;
    private IntPtr pathPtr;
    private int pathLen;
    private IsStateValidDelegate cb;
    private Func<Vector3, bool> IsStateValidExtern;
    private ClosestPointExternalDelegate ClosestPointExtern;
    private Func<State> getState;
    private Action<State> restoreState;
    public UnityOMPLInterface(Func<Vector3, bool> IsStateValidExternIn, ClosestPointExternalDelegate ClosestPointExternIn, Func<State> getStateIn, Action<State> restoreStateIn)
    {
        IsStateValidExtern = IsStateValidExternIn;
        ClosestPointExtern = ClosestPointExternIn;
        getState = getStateIn;
        restoreState = restoreStateIn;
        handle = Init();
    }
    ~UnityOMPLInterface()
    { 
        Destroy(handle);
    }

    public List<State> plan(double radius,State goal, Vector2 planningCenter, Vector2 planningSize, double planTime, int pathResolution, double dirScalar, double lambda, double widthScale, double numBasisPerMeter)
    {
        State stateStruct = getState();
        State tmpState = getState();
        
        RRTSearchInput input = new RRTSearchInput();
        input.radius = radius; 
        input.goal = goal; 
        input.state = tmpState; 
        input.validState = new IsStateValidDelegate(IsStateValid);
        input.closestPoint = new ClosestPointDelegate(GetClosestPoint);
        input.planningCenter.x = planningCenter.x; 
        input.planningCenter.y = planningCenter.y; 
        input.planningSize.x = planningSize.x; 
        input.planningSize.y = planningSize.y;
        input.pathResolution = pathResolution; 
        input.planTime = planTime;
        input.dirScalar = dirScalar;
        input.lambda = lambda;
        input.widthScale = widthScale;
        input.numBasisPerMeter = numBasisPerMeter;
        
        RRTSearchOutput output = new RRTSearchOutput();
 
        bool solved = RRTSearch(handle, ref input, ref output);
        List<State> path = new List<State>();
        if (solved)
        {
            State point = new State();
            IntPtr curPathPtr = output.path.data;
            for (int i=0; i < output.path.size; i++){
                point = Marshal.PtrToStructure<State>(curPathPtr);
                path.Add(point);
                curPathPtr += Marshal.SizeOf(typeof(State));
            }
        }

        restoreState(tmpState);
        
        return path;
    }
    //private delegate bool IsStateValidDelegate();

    private bool IsStateValid(ref State stateStruct)
    {
        Vector3 state = new Vector3(stateStruct.x, stateStruct.y, stateStruct.theta);
        return IsStateValidExtern(state);
    }
    
    private bool GetClosestPoint(ref State stateStruct, ref State pointStruct)
    {
        Vector3 state = new Vector3(stateStruct.x, stateStruct.y, stateStruct.theta);
        Vector3 point = new Vector3();
         bool isPoint = ClosestPointExtern(state, ref point);
         if (isPoint)
         {
             pointStruct.x = point.x;
             pointStruct.y = point.y;
        }
         return isPoint;
    }
    
    

    [DllImport("libUnityLib.so", EntryPoint = "Init", CallingConvention = CallingConvention.Cdecl)] 
    private static extern IntPtr Init();
    [DllImport("libUnityLib.so", EntryPoint = "Destroy", CallingConvention = CallingConvention.Cdecl)] 
    private static extern void Destroy(IntPtr handle);
    [DllImport("libUnityLib.so", EntryPoint = "RRTSearch", CallingConvention = CallingConvention.Cdecl)] 
    private static extern bool RRTSearch(IntPtr handle, ref RRTSearchInput input, ref RRTSearchOutput output);
//    private static extern bool RRTSearch(IntPtr handle, IntPtr gPtr, IntPtr sPtr, IsStateValidDelegate cb, ref IntPtr path, 
//ref int pathLen, double planTime, double lambda, double widthScale, double numBasisPerMeter);

}