
#define GJK_SUPPORT_L2(name) v2 name(v2 Dir, void* Geometry, transform_2d Transform)
typedef GJK_SUPPORT_L2(gjk_support_l2);

GJK_SUPPORT_L2(SupportCircle2d)
{
    circle_2d* Circle = (circle_2d*)Geometry;
    
    v2 NormalizedDir = Normalize(Dir);
    v2 Result = Transform.Pos + Circle->Radius * NormalizedDir;
    
    return Result;
}

GJK_SUPPORT_L2(SupportConvexPolygon2d)
{
    v2 Result = {};

    m2 RotationMat = M2Rotation(Transform.Rotation);
    m2 RotationTransposedMat = Transpose(RotationMat);
    v2 TransformedDir = RotationTransposedMat * Dir;

    polygon_2d* Polygon = (polygon_2d*)Geometry;

    f32 LargestDot = 0.0f;
    for (u32 VertexId = 0; VertexId < Polygon->NumVertices; ++VertexId)
    {
        f32 VertexDotProduct = Dot(TransformedDir, Polygon->Vertices[VertexId]);
        if (VertexDotProduct > LargestDot)
        {
            LargestDot = VertexDotProduct;
            Result = Polygon->Vertices[VertexId];
        }
    }

    Result = RotationMat * Result + Transform.Pos;
    
    return Result;
}

inline v2 Gjk2dGetPerp(v2 LineDir, v2 DirToOrigin)
{
    v2 Result = GetPerp(LineDir);
    if (Dot(Result, DirToOrigin) < 0)
    {
        Result = GetPerp(-LineDir);
    }

    return Result;
}

inline b32 Gjk2d(void* GeometryA, transform_2d TransformA, gjk_support_l2* SupportA,
                 void* GeometryB, transform_2d TransformB, gjk_support_l2* SupportB)
{
    // NOTE: https://www.youtube.com/watch?v=Qupqu1xe7Io&ab_channel=CaseyMuratori
    // NOTE: https://www.youtube.com/watch?v=ajv46BSqcK4&ab_channel=Reducible (secondary, first one is better for code IMO)
    
    b32 Result = false;

    u32 NumFoundPoints = 0;
    v2 FoundPoints[3];

    v2 Direction = V2(1, 0);
    FoundPoints[NumFoundPoints++] = (SupportA(Direction, GeometryA, TransformA) - SupportB(-Direction, GeometryB, TransformB));

    Direction = -FoundPoints[0];

    while (!Result)
    {
        v2 NewPoint = (SupportA(Direction, GeometryA, TransformA) - SupportB(-Direction, GeometryB, TransformB));

        if (Dot(Direction, NewPoint) < 0)
        {
            // NOTE: We didn't make it past the origin, so no intersection
            break;
        }

        FoundPoints[NumFoundPoints++] = NewPoint;

        // NOTE: Notation wise, we say A is the newest point, B is second, C is third, etc
        switch (NumFoundPoints)
        {
            case 2:
            {
                v2 A = FoundPoints[1];
                v2 B = FoundPoints[0];

                v2 AO = V2(0) - A;
                v2 AB = B - A;

                if (Dot(AO, AB) > 0)
                {
                    Direction = Gjk2dGetPerp(AB, AO);
                }
                else
                {
                    NumFoundPoints = 1;
                    FoundPoints[0] = A;
                    Direction = AO;
                }
            } break;

            case 3:
            {
                /* NOTE: We have 4 regions:
                     1) Above triangle edge AC
                     2) Inside triangle
                     3) Outside triangle vert A
                     4) Below triangle edge AB
                 */
                
                v2 A = FoundPoints[2];
                v2 B = FoundPoints[1];
                v2 C = FoundPoints[0];

                v2 AO = V2(0) - A;
                v2 AC = C - A;
                v2 ACPerp = GetPerp(-AC);
                v2 AB = B - A;
                v2 ABPerp = GetPerp(AB);

                if (Dot(AO, ACPerp) > 0)
                {
                    if (Dot(AO, AC) > 0)
                    {
                        // NOTE: We are in region 1
                        NumFoundPoints = 2;
                        FoundPoints[1] = A;
                        FoundPoints[0] = C;
                        Direction = Gjk2dGetPerp(AC, AO);
                    }
                    else
                    {
                        if (Dot(AO, AB) > 0)
                        {
                            // NOTE: We are in region 4
                            NumFoundPoints = 2;
                            FoundPoints[1] = A;
                            FoundPoints[0] = B;
                            Direction = Gjk2dGetPerp(AB, AO);
                        }
                        else
                        {
                            // NOTE: We are in region 3
                            NumFoundPoints = 1;
                            FoundPoints[0] = A;
                            Direction = AO;
                        }
                    }
                }
                else
                {
                    if (Dot(AO, ABPerp) > 0)
                    {
                        if (Dot(AO, AB) > 0)
                        {
                            // NOTE: We are in region 4
                            NumFoundPoints = 2;
                            FoundPoints[1] = A;
                            FoundPoints[0] = B;
                            Direction = Gjk2dGetPerp(AB, AO);
                        }
                        else
                        {
                            // NOTE: We are in region 3
                            NumFoundPoints = 1;
                            FoundPoints[0] = A;
                            Direction = AO;
                        }
                    }
                    else
                    {
                        // NOTE: We are in region 2 so we can exit
                        Result = true;
                    }
                }
            } break;

            default:
            {
                InvalidCodePath;
            } break;
        }
    }

    // NOTE: Begin EPA algorithm, expand our simplex until we found penetration depth
    // TODO: Implement

    return Result;
}


inline rigid_body_sim_l2 RigidBodySimL2Init(linear_arena* Arena, render_scene* Scene)
{
    rigid_body_sim_l2 Result = {};
    Result.NumRigidBodies = 3;
    Result.RigidBodyArray = PushArray(Arena, rigid_body_l2, Result.NumRigidBodies);

    for (u32 RigidBodyId = 0; RigidBodyId < Result.NumRigidBodies; ++RigidBodyId)
    {
        rigid_body_l2* CurrRigidBody = Result.RigidBodyArray + RigidBodyId;
        *CurrRigidBody = {};

        CurrRigidBody->Angle = 2.0f * Pi32 * RandFloat();
        CurrRigidBody->AngleVel = RandFloat();
        CurrRigidBody->DistFromCenter = 2.0f * RandFloat();

        CurrRigidBody->Rotation = 2.0f * Pi32 * RandFloat();
        CurrRigidBody->RotationVel = RandFloat();
    }

    // NOTE: First rigid body is a circle
    {
        rigid_body_l2* CurrRigidBody = Result.RigidBodyArray + 0;
        CurrRigidBody->Type = RigidBodyTypeL2_Circle;
        CurrRigidBody->Circle.Radius = 0.5f;
    }
    
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

        transform_2d TransformA = {};
        TransformA.Pos = V2(Cos(CurrBody->Angle), Sin(CurrBody->Angle));
        TransformA.Rotation = CurrBody->Rotation;

        // NOTE: Populate draw instances
        m4 Transform = (M4Pos(V3(TransformA.Pos, 0)) *
                        M4Rotation(V3(0.0f, 0.0f, TransformA.Rotation)));

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
        for (u32 OtherRigidBodyId = 0; OtherRigidBodyId < Sim->NumRigidBodies; ++OtherRigidBodyId)
        {
            rigid_body_l2* OtherBody = Sim->RigidBodyArray + OtherRigidBodyId;

            if (RigidBodyId != OtherRigidBodyId)
            {
                transform_2d TransformB = {};
                TransformB.Pos = V2(Cos(OtherBody->Angle), Sin(OtherBody->Angle));
                TransformB.Rotation = OtherBody->Rotation;

                gjk_support_l2* SupportA = CurrBody->Type == RigidBodyTypeL2_Circle ? SupportCircle2d : SupportConvexPolygon2d;
                gjk_support_l2* SupportB = OtherBody->Type == RigidBodyTypeL2_Circle ? SupportCircle2d : SupportConvexPolygon2d;

                void* GeometryA = CurrBody->Type == RigidBodyTypeL2_Circle ? (void*)&CurrBody->Circle : (void*)&CurrBody->Polygon;
                void* GeometryB = OtherBody->Type == RigidBodyTypeL2_Circle ? (void*)&OtherBody->Circle : (void*)&OtherBody->Polygon;
                
                Collides = Collides || Gjk2d(GeometryA, TransformA, SupportA, GeometryB, TransformB, SupportB);
            }
        }
    
        v4 Color = V4(1, 0, 0, 1);
        if (Collides)
        {
            Color = V4(0, 1, 0, 1);
        }

        SceneOpaqueInstanceAdd(Scene, MeshId, Transform, Color);
    }
}

