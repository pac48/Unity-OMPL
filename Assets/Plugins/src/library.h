#pragma once

#include <string>

#if defined(_MSC_VER)
#define MY_LIB_API __declspec(dllexport) // Microsoft
#elif defined(__GNUC__)
#define MY_LIB_API __attribute__((visibility("default"))) // GCC
#else
#define MY_LIB_API // Most compilers export all the symbols by default. We hope for the best here.
#pragma warning Unknown dynamic link import/export semantics.
#endif

typedef bool(*FuncCallBack)();

extern "C" {
MY_LIB_API void RRTSearch(std::intptr_t goalPtrIn, std::intptr_t statePtrIn, FuncCallBack cb);
//MY_LIB_API void RegisterRenderCallback(FuncCallBack cb);
}
