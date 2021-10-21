
inline rigid_body_sim_l2 RigidBodySimL2Init(linear_arena* Arena, render_scene* Scene)
{
    rigid_body_sim_l2 Result = {};
    Result.NumRigidBodies = 3;
    Result.RigidBodyArray = PushArray(Arena, rigid_body_l2, Result.NumRigidBodies);

#if 1
    for (u32 RigidBodyId = 0; RigidBodyId < Result.NumRigidBodies; ++RigidBodyId)
    {
        rigid_body_l2* CurrRigidBody = Result.RigidBodyArray + RigidBodyId;
        *CurrRigidBody = {};

#if 0
        //CurrRigidBody->Angle = RigidBodyId == 0 ? 0.45f : 0.0f;
        CurrRigidBody->AngleVel = 0.0f;
        CurrRigidBody->DistFromCenter = RigidBodyId == 0 ? 0.45f : 0.0f;
#else        
        CurrRigidBody->Angle = 2.0f * Pi32 * RandFloat();
        CurrRigidBody->AngleVel = RandFloat();
        CurrRigidBody->DistFromCenter = 2.0f * RandFloat();

        CurrRigidBody->Rotation = 2.0f * Pi32 * RandFloat();
        CurrRigidBody->RotationVel = RandFloat();
#endif
    }
#endif

    // NOTE: GJK/EPA test case
#if 0
    {
        {
            rigid_body_l2* CurrRigidBody = Result.RigidBodyArray + 0;
            CurrRigidBody->Angle = 3.7376986f;
            CurrRigidBody->DistFromCenter = 1.3077182f;
            CurrRigidBody->Rotation = 2.416955f;
        }
        
        {
            rigid_body_l2* CurrRigidBody = Result.RigidBodyArray + 1;
            CurrRigidBody->Angle = 4.0249281f;
            CurrRigidBody->DistFromCenter = 0.58058411f;
            CurrRigidBody->Rotation = 7.8473182f;
        }
    }
#endif
    
    // NOTE: First rigid body is a circle
    {
        rigid_body_l2* CurrRigidBody = Result.RigidBodyArray + 0;
        CurrRigidBody->Type = RigidBodyTypeL2_Circle;
        CurrRigidBody->Circle.Radius = 0.5f;
    }

#if 0
    // NOTE: First rigid body is a circle
    {
        rigid_body_l2* CurrRigidBody = Result.RigidBodyArray + 1;
        CurrRigidBody->Type = RigidBodyTypeL2_Circle;
        CurrRigidBody->Circle.Radius = 0.5f;
    }
#endif
    
#if 1
    // NOTE: Second rigid body is a triangle
    {
        rigid_body_l2* CurrRigidBody = Result.RigidBodyArray + 1;
        polygon_2d* CurrPolygon = &CurrRigidBody->Polygon;
        
        CurrRigidBody->Type = RigidBodyTypeL2_Polygon;
        CurrPolygon->NumVertices = 3;
        CurrPolygon->Vertices = PushArray(Arena, v2, CurrPolygon->NumVertices);

        CurrPolygon->Vertices[0] = 0.5f*V2(Cos((2.0f * Pi32 / 3.0f) * 0.0f), Sin((2.0f * Pi32 / 3.0f) * 0.0f));
        CurrPolygon->Vertices[1] = 0.5f*V2(Cos((2.0f * Pi32 / 3.0f) * 1.0f), Sin((2.0f * Pi32 / 3.0f) * 1.0f));
        CurrPolygon->Vertices[2] = 0.5f*V2(Cos((2.0f * Pi32 / 3.0f) * 2.0f), Sin((2.0f * Pi32 / 3.0f) * 2.0f));
        
        // NOTE: Generate the render mesh
        {
            vbo_mesh Mesh = {};

            Mesh.NumVertices = CurrPolygon->NumVertices + 1;
            Mesh.Buffer = VkBufferCreate(RenderState->Device, &RenderState->GpuArena,
                                         VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                                         sizeof(v3) * Mesh.NumVertices);
            v3* GpuVertices = VkCommandsPushWriteArray(&RenderState->Commands, Mesh.Buffer, v3, Mesh.NumVertices,
                                                       BarrierMask(VkAccessFlagBits(0), VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT),
                                                       BarrierMask(VK_ACCESS_VERTEX_ATTRIBUTE_READ_BIT, VK_PIPELINE_STAGE_VERTEX_INPUT_BIT));
            GpuVertices[0] = V3(CurrPolygon->Vertices[0], 0.0f);
            GpuVertices[1] = V3(CurrPolygon->Vertices[1], 0.0f);
            GpuVertices[2] = V3(CurrPolygon->Vertices[2], 0.0f);
            GpuVertices[3] = V3(CurrPolygon->Vertices[0], 0.0f);

            CurrPolygon->MeshId = SceneRenderMeshAdd(Scene, Mesh);
        }
    }
#endif

#if 1
    // NOTE: Third rigid body is a pentagon
    {
        rigid_body_l2* CurrRigidBody = Result.RigidBodyArray + 2;
        polygon_2d* CurrPolygon = &CurrRigidBody->Polygon;

        CurrRigidBody->Type = RigidBodyTypeL2_Polygon;
        CurrPolygon->NumVertices = 5;
        CurrPolygon->Vertices = PushArray(Arena, v2, CurrPolygon->NumVertices);

        CurrPolygon->Vertices[0] = 0.5f*V2(Cos((2.0f * Pi32 / 5.0f) * 0.0f), Sin((2.0f * Pi32 / 5.0f) * 0.0f));
        CurrPolygon->Vertices[1] = 0.5f*V2(Cos((2.0f * Pi32 / 5.0f) * 1.0f), Sin((2.0f * Pi32 / 5.0f) * 1.0f));
        CurrPolygon->Vertices[2] = 0.5f*V2(Cos((2.0f * Pi32 / 5.0f) * 2.0f), Sin((2.0f * Pi32 / 5.0f) * 2.0f));
        CurrPolygon->Vertices[3] = 0.5f*V2(Cos((2.0f * Pi32 / 5.0f) * 3.0f), Sin((2.0f * Pi32 / 5.0f) * 3.0f));
        CurrPolygon->Vertices[4] = 0.5f*V2(Cos((2.0f * Pi32 / 5.0f) * 4.0f), Sin((2.0f * Pi32 / 5.0f) * 4.0f));

        // NOTE: Generate the render mesh
        {
            vbo_mesh Mesh = {};

            Mesh.NumVertices = CurrPolygon->NumVertices + 1;
            Mesh.Buffer = VkBufferCreate(RenderState->Device, &RenderState->GpuArena,
                                         VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                                         sizeof(v3) * Mesh.NumVertices);
            v3* GpuVertices = VkCommandsPushWriteArray(&RenderState->Commands, Mesh.Buffer, v3, Mesh.NumVertices,
                                                       BarrierMask(VkAccessFlagBits(0), VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT),
                                                       BarrierMask(VK_ACCESS_VERTEX_ATTRIBUTE_READ_BIT, VK_PIPELINE_STAGE_VERTEX_INPUT_BIT));
            GpuVertices[0] = V3(CurrPolygon->Vertices[0], 0.0f);
            GpuVertices[1] = V3(CurrPolygon->Vertices[1], 0.0f);
            GpuVertices[2] = V3(CurrPolygon->Vertices[2], 0.0f);
            GpuVertices[3] = V3(CurrPolygon->Vertices[3], 0.0f);
            GpuVertices[4] = V3(CurrPolygon->Vertices[4], 0.0f);
            GpuVertices[5] = V3(CurrPolygon->Vertices[0], 0.0f);

            CurrPolygon->MeshId = SceneRenderMeshAdd(Scene, Mesh);
        }
    }
#endif
    
    return Result;
}

inline void RigidBodySimUpdate(rigid_body_sim_l2* Sim, f32 FrameTime, render_scene* Scene)
{
    for (u32 RigidBodyId = 0; RigidBodyId < Sim->NumRigidBodies; ++RigidBodyId)
    {
        rigid_body_l2* CurrBody = Sim->RigidBodyArray + RigidBodyId;
        
        // NOTE: Integrate position
        CurrBody->Angle += CurrBody->AngleVel * FrameTime;

        // NOTE: Integrate orientation
        CurrBody->Rotation += CurrBody->RotationVel * FrameTime;
    }

    for (u32 RigidBodyId = 0; RigidBodyId < Sim->NumRigidBodies; ++RigidBodyId)
    {
        rigid_body_l2* CurrBody = Sim->RigidBodyArray + RigidBodyId;

        v2 CurrBodyPos = CurrBody->DistFromCenter * V2(Cos(CurrBody->Angle), Sin(CurrBody->Angle));
        f32 CurrBodyRotation = CurrBody->Rotation;

        // NOTE: Populate draw instances
        m4 Transform = (M4Pos(V3(CurrBodyPos, 0)) *
                        M4Rotation(V3(0.0f, 0.0f, CurrBodyRotation)));

        u32 MeshId = 0;
        switch (CurrBody->Type)
        {
            case RigidBodyTypeL2_Circle:
            {
                MeshId = DemoState->CircleId;
            } break;

            case RigidBodyTypeL2_Polygon:
            {
                MeshId = CurrBody->Polygon.MeshId;
            } break;
        }

        // NOTE: Check if we collide with anything
        b32 Collides = false;
        for (u32 OtherRigidBodyId = RigidBodyId + 1; OtherRigidBodyId < Sim->NumRigidBodies; ++OtherRigidBodyId)
        {
            rigid_body_l2* OtherBody = Sim->RigidBodyArray + OtherRigidBodyId;

            v2 OtherBodyPos = OtherBody->DistFromCenter * V2(Cos(OtherBody->Angle), Sin(OtherBody->Angle));
            f32 OtherBodyRotation = OtherBody->Rotation;

            gjk_support_l2* SupportA = CurrBody->Type == RigidBodyTypeL2_Circle ? SupportCircle2d : SupportConvexPolygon2d;
            gjk_support_l2* SupportB = OtherBody->Type == RigidBodyTypeL2_Circle ? SupportCircle2d : SupportConvexPolygon2d;

            void* GeometryA = CurrBody->Type == RigidBodyTypeL2_Circle ? (void*)&CurrBody->Circle : (void*)&CurrBody->Polygon;
            void* GeometryB = OtherBody->Type == RigidBodyTypeL2_Circle ? (void*)&OtherBody->Circle : (void*)&OtherBody->Polygon;

            //gjk_result_2d CollisionResult = Gjk2d(GeometryA, CurrBodyPos, CurrBodyRotation, SupportA,
            //                                      GeometryB, OtherBodyPos, OtherBodyRotation, SupportB);
            gjk_result_2d CollisionResult = Gjk2d(GeometryB, OtherBodyPos, OtherBodyRotation, SupportB,
                                                  GeometryA, CurrBodyPos, CurrBodyRotation, SupportA);

            DebugPushPoint(CollisionResult.ContactPoint1, V4(1, 0, 0, 1));
            DebugPushPoint(CollisionResult.ContactPoint2, V4(0, 1, 0, 1));
            
            DebugPushLine(CollisionResult.ContactPoint1, CollisionResult.ContactPoint1 - CollisionResult.Normal * CollisionResult.Distance, V4(0, 0, 1, 1));
                
            Collides = Collides || CollisionResult.Intersects;
        }
    
        v4 Color = V4(1, 0, 0, 1);
        if (Collides)
        {
            Color = V4(0, 1, 0, 1);
        }

        SceneOpaqueInstanceAdd(Scene, MeshId, Transform, Color);
    }
}

