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
    public float z;

}

[StructLayout(LayoutKind.Sequential)]
public struct CArray {
    public IntPtr data;
    public int size;
};

public delegate bool IsStateValidDelegate(ref State state);

[StructLayout(LayoutKind.Sequential)]
public struct RRTSearchInput
{
    public State goal;
    public State state;
    public State planningCenter;
    public State planningSize;
	public double planTime;
	public double lambda;
	public double widthScale;
	public double numBasisPerMeter;
	public IsStateValidDelegate cb;
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
    private Func<State> getState;
    private Action<State> restoreState;
    public UnityOMPLInterface(Func<Vector3, bool> IsStateValidExternIn, Func<State> getStateIn, Action<State> restoreStateIn)
    {
        IsStateValidExtern = IsStateValidExternIn;
        getState = getStateIn;
        restoreState = restoreStateIn;
        handle = Init();
    }
    ~UnityOMPLInterface()
    { 
        Destroy(handle);
    }

    public List<State> plan(State goal, Vector2 planningCenter, Vector2 planningSize, double planTime, double lambda, double widthScale, double numBasisPerMeter)
    {
        State stateStruct = getState();
        State tmpState = getState();
        
        RRTSearchInput input = new RRTSearchInput();
        input.goal = goal; 
        input.state = tmpState; 
        input.cb = new IsStateValidDelegate(IsStateValid);
        input.planningCenter.x = planningCenter.x; 
        input.planningCenter.y = planningCenter.y; 
        input.planningSize.x = planningSize.x; 
        input.planningSize.y = planningSize.y; 
        input.planTime = planTime; 
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
        //return false;
        //State stateStruct = (State)Marshal.PtrToStructure(statePtr, typeof(State));
        Vector3 state = new Vector3(stateStruct.x, stateStruct.y, stateStruct.z);
        return IsStateValidExtern(state);
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