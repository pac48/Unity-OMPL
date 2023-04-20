using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Runtime.InteropServices;

[StructLayout(LayoutKind.Sequential)]
[System.Serializable] 
public struct State
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
    private int pathLen;
    private IsStateValidDelegate cb;
    private Func<Vector3, bool> IsStateValidExtern;
    private Func<State> getState;
    private Action<State> restoreState;
    public UnityOMPLInterface(Func<Vector3, bool> IsStateValidExternIn, Func<State> getStateIn, Action<State> restoreStateIn)
    {
        goalPtr = Marshal.AllocHGlobal(Marshal.SizeOf(typeof(State)));
        statePtr = Marshal.AllocHGlobal(Marshal.SizeOf(typeof(State)));
        pathPtr = Marshal.AllocHGlobal(Marshal.SizeOf(typeof(State)));
        pathLen = 0;
        cb = new IsStateValidDelegate(IsStateValid);
        IsStateValidExtern = IsStateValidExternIn;
        getState = getStateIn;
        restoreState = restoreStateIn;
        handle = Init();
    }
    ~UnityOMPLInterface()
    { 
        Destroy(handle);
        Marshal.FreeHGlobal(goalPtr);
        Marshal.FreeHGlobal(statePtr);
        Marshal.FreeHGlobal(pathPtr);
    }

    public List<State> plan(State goal)
    {
        State stateStruct = getState();

        Marshal.StructureToPtr(goal, goalPtr, false);
        Marshal.StructureToPtr(stateStruct, statePtr, false);
        
        State tmpState = getState();
        
        bool solved = RRTSearch(handle, goalPtr, statePtr, cb, ref pathPtr, ref pathLen);
        List<State> path = new List<State>();
        if (solved)
        {
            State point = new State();
            IntPtr curPathPtr = pathPtr;
            for (int i=0; i < pathLen; i++){
                point = Marshal.PtrToStructure<State>(curPathPtr);
                path.Add(point);
                curPathPtr += Marshal.SizeOf(typeof(State));
            }
        }

        restoreState(tmpState);
        
        return path;
    }
    private delegate bool IsStateValidDelegate();

    private bool IsStateValid()
    {
        State stateStruct = (State)Marshal.PtrToStructure(statePtr, typeof(State));
        Vector3 state = new Vector3(stateStruct.x, stateStruct.y, stateStruct.z);
        return IsStateValidExtern(state);
    }

    [DllImport("libUnityLib.so", EntryPoint = "Init", CallingConvention = CallingConvention.Cdecl)] 
    private static extern IntPtr Init();
    [DllImport("libUnityLib.so", EntryPoint = "Destroy", CallingConvention = CallingConvention.Cdecl)] 
    private static extern void Destroy(IntPtr handle);
    [DllImport("libUnityLib.so", EntryPoint = "RRTSearch", CallingConvention = CallingConvention.Cdecl)] 
    private static extern bool RRTSearch(IntPtr handle, IntPtr gPtr, IntPtr sPtr, IsStateValidDelegate cb, ref IntPtr path, ref int pathLen);

}