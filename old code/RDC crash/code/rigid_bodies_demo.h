#pragma once

//#define VALIDATION 1

#include "framework_vulkan\framework_vulkan.h"

#define WIN32_PROFILING
//#define CPU_PROFILING
//#define X86_PROFILING
#include "profiling\profiling.h"

//
// NOTE: Sim handling
//

struct body_array
{
    // TODO: Handle multiple grid cells for one entity
    f32* PosX;
    f32* PosY;
    f32* VelX;
    f32* VelY;
};

//
// NOTE: Render Data
//

struct scene_globals
{
    v3 CameraPos;
    u32 NumPointLights;
};

struct instance_entry
{
    u32 MeshId;
    m4 WVTransform;
    m4 WVPTransform;
    v4 Color;
};

struct gpu_instance_entry
{
    m4 WVTransform;
    m4 WVPTransform;
    v4 Color;
};

struct vbo_mesh
{
    u32 NumVertices;
    VkBuffer Buffer;
};

struct render_scene
{
    // NOTE: General Render Data
    camera Camera;
    VkDescriptorSetLayout SceneDescLayout;
    VkBuffer SceneBuffer;
    VkDescriptorSet SceneDescriptor;

    // NOTE: Scene Meshes
    u32 MaxNumRenderMeshes;
    u32 NumRenderMeshes;
    vbo_mesh* RenderMeshes;
    
    // NOTE: Opaque Instances
    u32 MaxNumOpaqueInstances;
    u32 NumOpaqueInstances;
    instance_entry* OpaqueInstances;
    VkBuffer OpaqueInstanceBuffer;
};

struct demo_state
{
    platform_block_arena PlatformBlockArena;
    linear_arena Arena;
    linear_arena TempArena;

    // NOTE: Samplers
    VkSampler PointSampler;
    VkSampler LinearSampler;

    // NOTE: Rendering Data
    vk_linear_arena RenderTargetArena;
    render_target_entry SwapChainEntry;
    VkImage DepthImage;
    render_target_entry DepthEntry;
    render_target RenderTarget;
    vk_pipeline* RenderPipeline;

    render_scene Scene;

    ui_state UiState;
    
    // NOTE: Mesh Ids
    u32 SquareId;
    u32 CircleId;
};

global demo_state* DemoState;
