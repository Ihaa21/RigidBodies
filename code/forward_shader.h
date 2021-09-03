
#define MATH_GLSL 1
#include "..\libs\math\math.h"

//
// NOTE: Scene
//

struct instance_entry
{
    m4 WVTransform;
    m4 WVPTransform;
    v4 Color;
};

layout(set = 0, binding = 0) uniform scene_buffer          
{                                                                   
    v3 CameraPos;                                                   
    u32 NumPointLights;
} SceneBuffer;

layout(set = 0, binding = 1) buffer instance_buffer        
{                                                                   
    instance_entry InstanceBuffer[];                                
};                                                                  
