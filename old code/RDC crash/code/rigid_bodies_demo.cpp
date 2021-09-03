
#include "rigid_bodies_demo.h"

/*

  NOTE: Based on the below tutorial series:

    - https://www.toptal.com/game/video-game-physics-part-i-an-introduction-to-rigid-body-dynamics
    - https://www.toptal.com/game/video-game-physics-part-ii-collision-detection-for-solid-objects
    - https://www.toptal.com/game/video-game-physics-part-iii-constrained-rigid-body-simulation
  
 */

inline f32 RandFloat()
{
    f32 Result = f32(rand()) / f32(RAND_MAX);
    return Result;
}

//
// NOTE: Asset Storage System
//

inline u32 SceneRenderMeshAdd(render_scene* Scene, vbo_mesh Mesh)
{
    Assert(Scene->NumRenderMeshes < Scene->MaxNumRenderMeshes);

    u32 Result = Scene->NumRenderMeshes++;
    vbo_mesh* RenderMesh = Scene->RenderMeshes + Result;
    *RenderMesh = Mesh;

    return Result;
}

inline void SceneOpaqueInstanceAdd(render_scene* Scene, u32 MeshId, m4 WTransform, v4 Color)
{
    Assert(Scene->NumOpaqueInstances < Scene->MaxNumOpaqueInstances);

    instance_entry* Instance = Scene->OpaqueInstances + Scene->NumOpaqueInstances++;
    Instance->MeshId = MeshId;
    Instance->WVTransform = CameraGetV(&Scene->Camera)*WTransform;
    Instance->WVPTransform = CameraGetP(&Scene->Camera)*Instance->WVTransform;
    Instance->Color = Color;
}

//
// NOTE: Demo Code
//

inline void DemoSwapChainChange(u32 Width, u32 Height)
{
    b32 ReCreate = DemoState->RenderTargetArena.Used != 0;
    VkArenaClear(&DemoState->RenderTargetArena);

    // NOTE: Render Target Data
    RenderTargetEntryReCreate(&DemoState->RenderTargetArena, Width, Height, VK_FORMAT_D32_SFLOAT,
                              VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT, VK_IMAGE_ASPECT_DEPTH_BIT,
                              &DemoState->DepthImage, &DemoState->DepthEntry);
}

inline void DemoAllocGlobals(linear_arena* Arena)
{
    // IMPORTANT: These are always the top of the program memory
    DemoState = PushStruct(Arena, demo_state);
    RenderState = PushStruct(Arena, render_state);
    ProfilerState = PushStruct(Arena, profiler_state);
}

DEMO_INIT(Init)
{
    // NOTE: Init Memory
    {
        linear_arena Arena = LinearArenaCreate(ProgramMemory, ProgramMemorySize);
        DemoAllocGlobals(&Arena);
        *DemoState = {};
        *RenderState = {};
        *ProfilerState = {};
        DemoState->Arena = Arena;
        DemoState->TempArena = LinearSubArena(&DemoState->Arena, MegaBytes(10));
    }

    ProfilerStateCreate(ProfilerFlag_OutputCsv | ProfilerFlag_AutoSetEndOfFrame);

    // NOTE: Init Vulkan
    {
        {
            const char* DeviceExtensions[] =
                {
                    "VK_EXT_shader_viewport_index_layer",
                };
            
            render_init_params InitParams = {};
            InitParams.ValidationEnabled = false;
            InitParams.WindowWidth = WindowWidth;
            InitParams.WindowHeight = WindowHeight;
            InitParams.GpuLocalSize = MegaBytes(10);
            InitParams.DeviceExtensionCount = ArrayCount(DeviceExtensions);
            InitParams.DeviceExtensions = DeviceExtensions;
            VkInit(VulkanLib, hInstance, WindowHandle, &DemoState->Arena, &DemoState->TempArena, InitParams);
        }
    }
    
    // NOTE: Create samplers
    DemoState->PointSampler = VkSamplerCreate(RenderState->Device, VK_FILTER_NEAREST, VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE, VK_BORDER_COLOR_FLOAT_TRANSPARENT_BLACK, 0.0f);
    DemoState->LinearSampler = VkSamplerCreate(RenderState->Device, VK_FILTER_LINEAR, VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE, VK_BORDER_COLOR_FLOAT_TRANSPARENT_BLACK, 0.0f);
        
    // NOTE: Init render target entries
    DemoState->SwapChainEntry = RenderTargetSwapChainEntryCreate(RenderState->WindowWidth, RenderState->WindowHeight,
                                                                 RenderState->SwapChainFormat);

    // NOTE: Init scene system
    {
        render_scene* Scene = &DemoState->Scene;

        Scene->Camera = CameraFlatCreate(V3(0, 0, 0), 4.0f, 2.0f, 1.0f, -1.0f, 1.0f);

        Scene->SceneBuffer = VkBufferCreate(RenderState->Device, &RenderState->GpuArena,
                                            VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                                            sizeof(scene_globals));
        
        Scene->MaxNumRenderMeshes = 1000;
        Scene->RenderMeshes = PushArray(&DemoState->Arena, vbo_mesh, Scene->MaxNumRenderMeshes);

        Scene->MaxNumOpaqueInstances = 50000;
        Scene->OpaqueInstances = PushArray(&DemoState->Arena, instance_entry, Scene->MaxNumOpaqueInstances);
        Scene->OpaqueInstanceBuffer = VkBufferCreate(RenderState->Device, &RenderState->GpuArena,
                                                     VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                                                     sizeof(gpu_instance_entry)*Scene->MaxNumOpaqueInstances);

        // NOTE: Create general descriptor set layouts
        {
            {
                vk_descriptor_layout_builder Builder = VkDescriptorLayoutBegin(&Scene->SceneDescLayout);
                VkDescriptorLayoutAdd(&Builder, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT | VK_SHADER_STAGE_COMPUTE_BIT);
                VkDescriptorLayoutAdd(&Builder, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT | VK_SHADER_STAGE_COMPUTE_BIT);
                VkDescriptorLayoutEnd(RenderState->Device, &Builder);
            }
        }

        // NOTE: Populate descriptors
        Scene->SceneDescriptor = VkDescriptorSetAllocate(RenderState->Device, RenderState->DescriptorPool, Scene->SceneDescLayout);
        VkDescriptorBufferWrite(&RenderState->DescriptorManager, Scene->SceneDescriptor, 0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, Scene->SceneBuffer);
        VkDescriptorBufferWrite(&RenderState->DescriptorManager, Scene->SceneDescriptor, 1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, Scene->OpaqueInstanceBuffer);
    }

    // NOTE: Create render data
    {
        u32 Width = RenderState->WindowWidth;
        u32 Height = RenderState->WindowHeight;
        
        DemoState->RenderTargetArena = VkLinearArenaCreate(RenderState->Device, RenderState->LocalMemoryId, MegaBytes(100));
        DemoSwapChainChange(Width, Height);

        // NOTE: Forward Pass
        {
            render_target_builder Builder = RenderTargetBuilderBegin(&DemoState->Arena, &DemoState->TempArena, Width, Height);
            RenderTargetAddTarget(&Builder, &DemoState->SwapChainEntry, VkClearColorCreate(0, 0, 0, 1));
            RenderTargetAddTarget(&Builder, &DemoState->DepthEntry, VkClearDepthStencilCreate(0, 0));
            
            vk_render_pass_builder RpBuilder = VkRenderPassBuilderBegin(&DemoState->TempArena);
            u32 ColorId = VkRenderPassAttachmentAdd(&RpBuilder, RenderState->SwapChainFormat, VK_ATTACHMENT_LOAD_OP_CLEAR,
                                                    VK_ATTACHMENT_STORE_OP_STORE, VK_IMAGE_LAYOUT_UNDEFINED,
                                                    VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL);
            u32 DepthId = VkRenderPassAttachmentAdd(&RpBuilder, DemoState->DepthEntry.Format, VK_ATTACHMENT_LOAD_OP_CLEAR,
                                                    VK_ATTACHMENT_STORE_OP_DONT_CARE, VK_IMAGE_LAYOUT_UNDEFINED,
                                                    VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL);

            VkRenderPassSubPassBegin(&RpBuilder, VK_PIPELINE_BIND_POINT_GRAPHICS);
            VkRenderPassColorRefAdd(&RpBuilder, ColorId, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL);
            VkRenderPassDepthRefAdd(&RpBuilder, DepthId, VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL);
            VkRenderPassSubPassEnd(&RpBuilder);

            DemoState->RenderTarget = RenderTargetBuilderEnd(&Builder, VkRenderPassBuilderEnd(&RpBuilder, RenderState->Device));
        }
                
        // NOTE: Create PSO
        {
            vk_pipeline_builder Builder = VkPipelineBuilderBegin(&DemoState->TempArena);

            // NOTE: Shaders
            VkPipelineShaderAdd(&Builder, "shader_forward_vert.spv", "main", VK_SHADER_STAGE_VERTEX_BIT);
            VkPipelineShaderAdd(&Builder, "shader_forward_frag.spv", "main", VK_SHADER_STAGE_FRAGMENT_BIT);
                
            // NOTE: Specify input vertex data format
            VkPipelineVertexBindingBegin(&Builder);
            VkPipelineVertexAttributeAdd(&Builder, VK_FORMAT_R32G32B32_SFLOAT, sizeof(v3));
            VkPipelineVertexBindingEnd(&Builder);

            VkPipelineInputAssemblyAdd(&Builder, VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST, VK_FALSE);
            VkPipelineDepthStateAdd(&Builder, VK_TRUE, VK_TRUE, VK_COMPARE_OP_GREATER);
            
            // NOTE: Set the blending state
            VkPipelineColorAttachmentAdd(&Builder, VK_BLEND_OP_ADD, VK_BLEND_FACTOR_ONE, VK_BLEND_FACTOR_ZERO,
                                         VK_BLEND_OP_ADD, VK_BLEND_FACTOR_ONE, VK_BLEND_FACTOR_ZERO);

            VkDescriptorSetLayout DescriptorLayouts[] =
                {
                    DemoState->Scene.SceneDescLayout,
                };
            
            DemoState->RenderPipeline = VkPipelineBuilderEnd(&Builder, RenderState->Device, &RenderState->PipelineManager,
                                                             DemoState->RenderTarget.RenderPass, 0, DescriptorLayouts,
                                                             ArrayCount(DescriptorLayouts));
        }
    }
    
    // NOTE: Upload assets
    vk_commands* Commands = &RenderState->Commands;
    VkCommandsBegin(Commands, RenderState->Device);
    {
        render_scene* Scene = &DemoState->Scene;
        
        // NOTE: Push textures
        vk_image WhiteTexture = {};
        {
            u32 Texels[] =
            {
                0xFFFFFFFF, 
            };
            u32 Dim = 1;
            
            u32 ImageSize = Dim*Dim*sizeof(u32);
            WhiteTexture = VkImageCreate(RenderState->Device, &RenderState->GpuArena, Dim, Dim, VK_FORMAT_R8G8B8A8_UNORM,
                                         VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT, VK_IMAGE_ASPECT_COLOR_BIT);

            u8* GpuMemory = VkCommandsPushWriteImage(Commands, WhiteTexture.Image, Dim, Dim, ImageSize,
                                                     VK_IMAGE_ASPECT_COLOR_BIT, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                                                     BarrierMask(VkAccessFlagBits(0), VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT),
                                                     BarrierMask(VK_ACCESS_SHADER_READ_BIT, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT));

            Copy(Texels, GpuMemory, ImageSize);
        }
        
        // NOTE: Push quad mesh
        {
            vbo_mesh Mesh = {};
            
            f32 Vertices[] = 
                {
                    -0.5, -0.5, 0.0f,
                    0.5, -0.5, 0.0f,
                    0.5,  0.5, 0.0f,
                    -0.5,  0.5, 0.0f,
                };

            Mesh.NumVertices = 4;
            Mesh.Buffer = VkBufferCreate(RenderState->Device, &RenderState->GpuArena,
                                         VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                                         sizeof(Vertices));
            void* GpuMemory = VkCommandsPushWrite(&RenderState->Commands, Mesh.Buffer, sizeof(Vertices),
                                                  BarrierMask(VkAccessFlagBits(0), VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT),
                                                  BarrierMask(VK_ACCESS_VERTEX_ATTRIBUTE_READ_BIT, VK_PIPELINE_STAGE_VERTEX_INPUT_BIT));
            Copy(Vertices, GpuMemory, sizeof(Vertices));

            DemoState->SquareId = SceneRenderMeshAdd(Scene, Mesh);
        }

        // NOTE: Push circle mesh
        {
            vbo_mesh Mesh = {};

            Mesh.NumVertices = 256;
            Mesh.Buffer = VkBufferCreate(RenderState->Device, &RenderState->GpuArena,
                                         VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                                         sizeof(v3) * Mesh.NumVertices);
            v3* GpuMemory = VkCommandsPushWriteArray(&RenderState->Commands, Mesh.Buffer, v3, Mesh.NumVertices,
                                                     BarrierMask(VkAccessFlagBits(0), VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT),
                                                     BarrierMask(VK_ACCESS_VERTEX_ATTRIBUTE_READ_BIT, VK_PIPELINE_STAGE_VERTEX_INPUT_BIT));

            for (u32 Id = 0; Id < Mesh.NumVertices; ++Id)
            {
                f32 Angle = (2.0f * Pi32 / f32(Mesh.NumVertices)) * f32(Id);
                GpuMemory[Id] = 0.5f*V3(Cos(Angle), Sin(Angle), 0.0f);
            }

            DemoState->CircleId = SceneRenderMeshAdd(Scene, Mesh);
        }

        UiStateCreate(RenderState->Device, &DemoState->Arena, &DemoState->TempArena, RenderState->LocalMemoryId,
                      &RenderState->DescriptorManager, &RenderState->PipelineManager, &RenderState->Commands,
                      RenderState->SwapChainFormat, VK_IMAGE_LAYOUT_PRESENT_SRC_KHR, &DemoState->UiState);
    }

    VkDescriptorManagerFlush(RenderState->Device, &RenderState->DescriptorManager);
    VkCommandsSubmit(Commands, RenderState->Device, RenderState->GraphicsQueue);
}

DEMO_DESTROY(Destroy)
{
    // TODO: Remove if we can verify that this is auto destroyed (check recompiling if it calls the destructor)
    ProfilerStateDestroy();
}

DEMO_SWAPCHAIN_CHANGE(SwapChainChange)
{
    VkCheckResult(vkDeviceWaitIdle(RenderState->Device));
    VkSwapChainReCreate(&DemoState->TempArena, WindowWidth, WindowHeight, RenderState->PresentMode);
    
    DemoState->SwapChainEntry.Width = RenderState->WindowWidth;
    DemoState->SwapChainEntry.Height = RenderState->WindowHeight;
    DemoState->Scene.Camera.PerspAspectRatio = f32(RenderState->WindowWidth / RenderState->WindowHeight);
    DemoSwapChainChange(RenderState->WindowWidth, RenderState->WindowHeight);
}

DEMO_CODE_RELOAD(CodeReload)
{
    linear_arena Arena = LinearArenaCreate(ProgramMemory, ProgramMemorySize);
    // IMPORTANT: We are relying on the memory being the same here since we have the same base ptr with the VirtualAlloc so we just need
    // to patch our global pointers here
    DemoAllocGlobals(&Arena);

    VkGetGlobalFunctionPointers(VulkanLib);
    VkGetInstanceFunctionPointers();
    VkGetDeviceFunctionPointers();
}

DEMO_MAIN_LOOP(MainLoop)
{
    {
        CPU_TIMED_BLOCK("MainLoop");
    
        u32 ImageIndex;
        VkCheckResult(vkAcquireNextImageKHR(RenderState->Device, RenderState->SwapChain, UINT64_MAX, RenderState->ImageAvailableSemaphore,
                                            VK_NULL_HANDLE, &ImageIndex));
        DemoState->SwapChainEntry.View = RenderState->SwapChainViews[ImageIndex];

        vk_commands* Commands = &RenderState->Commands;
        VkCommandsBegin(Commands, RenderState->Device);

        // NOTE: Update pipelines
        VkPipelineUpdateShaders(RenderState->Device, &RenderState->CpuArena, &RenderState->PipelineManager);

        RenderTargetUpdateEntries(&DemoState->TempArena, &DemoState->RenderTarget);
    
        // NOTE: Update Ui State
        local_global f32 ModifiedFrameTime = 1.0f / 60.0f;
        {
            ui_state* UiState = &DemoState->UiState;
        
            ui_frame_input UiCurrInput = {};
            UiCurrInput.MouseDown = CurrInput->MouseDown;
            UiCurrInput.MousePixelPos = V2(CurrInput->MousePixelPos);
            UiCurrInput.MouseScroll = CurrInput->MouseScroll;
            Copy(CurrInput->KeysDown, UiCurrInput.KeysDown, sizeof(UiCurrInput.KeysDown));
            UiStateBegin(UiState, FrameTime, RenderState->WindowWidth, RenderState->WindowHeight, UiCurrInput);
            local_global v2 PanelPos = V2(100, 800);
            ui_panel Panel = UiPanelBegin(UiState, &PanelPos, "Shadow Panel");

            {
                UiPanelText(&Panel, "Boid Data:");

#if 0
                UiPanelNextRowIndent(&Panel);
                UiPanelText(&Panel, "FrameTime:");
                UiPanelHorizontalSlider(&Panel, 0.0f, 0.03f, &ModifiedFrameTime);
                UiPanelNumberBox(&Panel, 0.0f, 0.03f, &ModifiedFrameTime);
                UiPanelNextRow(&Panel);
#endif            
            }

            UiPanelEnd(&Panel);

            UiStateEnd(UiState, &RenderState->DescriptorManager);
        }

        // NOTE: Upload scene data
        {
            render_scene* Scene = &DemoState->Scene;
            Scene->NumOpaqueInstances = 0;
            if (!(DemoState->UiState.MouseTouchingUi || DemoState->UiState.ProcessedInteraction))
            {
                CameraUpdate(&Scene->Camera, CurrInput, PrevInput, FrameTime);
            }
            
            // NOTE: Populate scene
            {
                // NOTE: Add Instances
                {
                    CPU_TIMED_BLOCK("Add Instances");

                    m4 Transform = M4Pos(V3(0, 0, 0));
                    SceneOpaqueInstanceAdd(Scene, DemoState->SquareId, Transform, V4(1, 0, 0, 1));
                }
                
                {
                    CPU_TIMED_BLOCK("Upload instances to GPU");
                    gpu_instance_entry* GpuData = VkCommandsPushWriteArray(Commands, Scene->OpaqueInstanceBuffer, gpu_instance_entry, Scene->NumOpaqueInstances,
                                                                           BarrierMask(VkAccessFlagBits(0), VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT),
                                                                           BarrierMask(VK_ACCESS_SHADER_READ_BIT, VK_PIPELINE_STAGE_VERTEX_SHADER_BIT));

                    for (u32 InstanceId = 0; InstanceId < Scene->NumOpaqueInstances; ++InstanceId)
                    {
                        GpuData[InstanceId].WVTransform = Scene->OpaqueInstances[InstanceId].WVTransform;
                        GpuData[InstanceId].WVPTransform = Scene->OpaqueInstances[InstanceId].WVPTransform;
                        GpuData[InstanceId].Color = Scene->OpaqueInstances[InstanceId].Color;
                    }
                }
            }        
        
            {
                scene_globals* Data = VkCommandsPushWriteStruct(Commands, Scene->SceneBuffer, scene_globals,
                                                                BarrierMask(VkAccessFlagBits(0), VK_PIPELINE_STAGE_ALL_COMMANDS_BIT),
                                                                BarrierMask(VK_ACCESS_UNIFORM_READ_BIT, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT));
                *Data = {};
                Data->CameraPos = Scene->Camera.Pos;
            }

            VkCommandsTransferFlush(Commands, RenderState->Device);
        }

        // NOTE: Render Scene
        RenderTargetPassBegin(&DemoState->RenderTarget, Commands, RenderTargetRenderPass_SetViewPort | RenderTargetRenderPass_SetScissor);
        {
            CPU_TIMED_BLOCK("Render Forward");
            render_scene* Scene = &DemoState->Scene;
        
            vkCmdBindPipeline(Commands->Buffer, VK_PIPELINE_BIND_POINT_GRAPHICS, DemoState->RenderPipeline->Handle);
            {
                VkDescriptorSet DescriptorSets[] =
                    {
                        Scene->SceneDescriptor,
                    };
                vkCmdBindDescriptorSets(Commands->Buffer, VK_PIPELINE_BIND_POINT_GRAPHICS, DemoState->RenderPipeline->Layout, 0,
                                        ArrayCount(DescriptorSets), DescriptorSets, 0, 0);
            }

            u32 InstanceId = 0;
            for (; InstanceId < Scene->NumOpaqueInstances; )
            {
                instance_entry* CurrInstance = Scene->OpaqueInstances + InstanceId;
                vbo_mesh* CurrMesh = Scene->RenderMeshes + CurrInstance->MeshId;
            
                VkDeviceSize Offset = 0;
                vkCmdBindVertexBuffers(Commands->Buffer, 0, 1, &CurrMesh->Buffer, &Offset);

                // NOTE: Check how many instances share the same mesh
                u32 NextInstanceId = 1;
                while (NextInstanceId < Scene->NumOpaqueInstances)
                {
                    instance_entry* NextInstance = Scene->OpaqueInstances + NextInstanceId;
                    vbo_mesh* NextMesh = Scene->RenderMeshes + NextInstance->MeshId;

                    if (NextMesh != CurrMesh)
                    {
                        break;
                    }
                    NextInstanceId += 1;
                }
                vkCmdDrawIndexed(Commands->Buffer, CurrMesh->NumVertices, NextInstanceId - InstanceId, 0, 0, InstanceId);

                InstanceId = NextInstanceId;
            }
        }
        RenderTargetPassEnd(Commands);        
        UiStateRender(&DemoState->UiState, RenderState->Device, Commands, DemoState->SwapChainEntry.View);

        VkCommandsEnd(Commands, RenderState->Device);
    
        // NOTE: Render to our window surface
        // NOTE: Tell queue where we render to surface to wait
        VkPipelineStageFlags WaitDstMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
        VkSubmitInfo SubmitInfo = {};
        SubmitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
        SubmitInfo.waitSemaphoreCount = 1;
        SubmitInfo.pWaitSemaphores = &RenderState->ImageAvailableSemaphore;
        SubmitInfo.pWaitDstStageMask = &WaitDstMask;
        SubmitInfo.commandBufferCount = 1;
        SubmitInfo.pCommandBuffers = &Commands->Buffer;
        SubmitInfo.signalSemaphoreCount = 1;
        SubmitInfo.pSignalSemaphores = &RenderState->FinishedRenderingSemaphore;
        VkCheckResult(vkQueueSubmit(RenderState->GraphicsQueue, 1, &SubmitInfo, Commands->Fence));
    
        VkPresentInfoKHR PresentInfo = {};
        PresentInfo.sType = VK_STRUCTURE_TYPE_PRESENT_INFO_KHR;
        PresentInfo.waitSemaphoreCount = 1;
        PresentInfo.pWaitSemaphores = &RenderState->FinishedRenderingSemaphore;
        PresentInfo.swapchainCount = 1;
        PresentInfo.pSwapchains = &RenderState->SwapChain;
        PresentInfo.pImageIndices = &ImageIndex;
        VkResult Result = vkQueuePresentKHR(RenderState->PresentQueue, &PresentInfo);

        switch (Result)
        {
            case VK_SUCCESS:
            {
            } break;

            case VK_ERROR_OUT_OF_DATE_KHR:
            case VK_SUBOPTIMAL_KHR:
            {
                // NOTE: Window size changed
                InvalidCodePath;
            } break;

            default:
            {
                InvalidCodePath;
            } break;
        }
    }

    ProfilerProcessData();
    ProfilerPrintTimeStamps();
}
