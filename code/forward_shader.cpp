#version 450

#extension GL_ARB_separate_shader_objects : enable
#extension GL_GOOGLE_include_directive : enable

#include "forward_shader.h"

#if VERTEX_SHADER

layout(location = 0) in v3 InPos;

layout(location = 0) out flat v4 OutColor;

void main()
{
    instance_entry Entry = InstanceBuffer[gl_InstanceIndex];
    
    gl_Position = Entry.WVPTransform * V4(InPos, 1);
    OutColor = Entry.Color;
}

#endif

#if FRAGMENT_SHADER

layout(location = 0) in flat v4 InColor;

layout(location = 0) out v4 OutColor;

void main()
{
    OutColor = InColor;
}

#endif
