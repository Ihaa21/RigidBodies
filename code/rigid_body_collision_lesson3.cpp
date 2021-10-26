
//
// NOTE: Circle Collision
//

inline collision_result_l3 CircleCollision(v2 PosA, f32 RadiusA, v2 PosB, f32 RadiusB)
{
    collision_result_l3 Result = {};

    // TODO: Handle when PosA == PosB
    
    v2 DistVec = PosB - PosA;
    f32 Dist = Length(DistVec);
    Result.Collides = Dist < (RadiusA + RadiusB);

    if (Result.Collides)
    {
        Result.Depth = (RadiusA + RadiusB) - Dist;
        Result.Normal = DistVec / Dist;
        // NOTE: https://github.com/granj2020/Cirobb-Engine/blob/master/cirobb/Collision.cpp
        // NOTE: Instead of 2 contact points, we generate a single center one
        Result.ContactPoint = PosA + Result.Normal * (RadiusA - RadiusB + Dist) * 0.5f;
    }

    return Result;
}

inline collision_result_l3 CircleToOBB(v2 CirclePos, f32 CircleRadius, v2 BoxPos, v2 BoxDim)
{
    collision_result_l3 Result = {};
    
    v2 Distance = CirclePos - BoxPos;
    v2 localSpace = Distance;
    v2 BoxRadius = BoxDim * 0.5f;

    f32 dx = Abs(localSpace.x) - BoxRadius.x;
    if(dx > CircleRadius) return Result;
  
    f32 dy = Abs(localSpace.y) - BoxRadius.y;
    if(dy > CircleRadius) return Result;
  
    v2 closest = {}; // Closest Point
  
    // 4 Conditions to Find the Closest Point of the Circle.
    if(-localSpace.x > BoxRadius.x) closest.x =  dx; else
        if( localSpace.x > BoxRadius.x) closest.x = -dx;
    if(-localSpace.y > BoxRadius.y) closest.y =  dy; else
        if( localSpace.y > BoxRadius.y) closest.y = -dy;
  
    f32 magnitude = LengthSquared(closest);
    if(magnitude > CircleRadius * CircleRadius) return Result;
  
    if(magnitude) // Shallow Penetration
    {
        magnitude = sqrtf(magnitude); 
        Result.Normal = closest * (1.0f / magnitude);     
    }
    else // Deep Penetration
    {
        magnitude = max(dx, dy);
        Result.Normal = dx > dy ? V2(localSpace.x < 0 ? 1 : -1.0f, 0.0f) : V2(0.0f, localSpace.y < 0 ? 1.0f : -1.0f);
    }

    Result.Collides = true;
    Result.Depth = magnitude - CircleRadius;
    Result.ContactPoint = CirclePos + Result.Normal * magnitude;

    return Result;
}

//
// NOTE: Rigid Body Functions
//

inline rigid_body_l3* RigidBodyL3Init(rigid_body_sim_l3* Sim, v2 Pos, v2 Vel, f32 Angle, f32 AngleVel, f32 InvMass)
{
    Assert(Sim->NumRigidBodies < Sim->MaxNumRigidBodies);
    rigid_body_l3* Result = Sim->RigidBodyArray + Sim->NumRigidBodies++;

    *Result = {};
    Result->InvMass = InvMass;
    Result->RestitutionCoeff = 0.3f;
    
    Result->Pos = Pos;
    Result->Vel = Vel;
    Result->Angle = Angle;
    Result->AngleVel = AngleVel;

    return Result;
}

inline void RigidBodyL3CircleCreate(rigid_body_sim_l3* Sim, v2 Pos, v2 Vel, f32 Angle, f32 AngleVel, f32 InvMass, f32 Radius)
{
    rigid_body_l3* Result = RigidBodyL3Init(Sim, Pos, Vel, Angle, AngleVel, InvMass);
    Result->Type = RigidBodyTypeL3_Circle;
    Result->Circle.Radius = Radius;
    Result->InvInertia = 0.5f * InvMass / (0.25f * Square(Radius)); // NOTE: 1 / (1/4 * m * r^2)
}

inline void RigidBodyL3RectCreate(rigid_body_sim_l3* Sim, linear_arena* Arena, render_scene* Scene, v2 Pos, v2 Vel, f32 Angle,
                                  f32 AngleVel, f32 InvMass, v2 Dim)
{
    rigid_body_l3* Result = RigidBodyL3Init(Sim, Pos, Vel, Angle, AngleVel, InvMass);
    Result->Type = RigidBodyTypeL3_Polygon;
    Result->InvInertia = 12.0f * Result->InvMass / Length(Dim); 

    polygon_2d* CurrPolygon = &Result->Polygon;
    CurrPolygon->NumVertices = 4;
    CurrPolygon->Vertices = PushArray(Arena, v2, CurrPolygon->NumVertices);
    CurrPolygon->Vertices[0] = 0.5f*V2(-Dim.x, Dim.y);
    CurrPolygon->Vertices[1] = 0.5f*V2(-Dim.x, -Dim.y);
    CurrPolygon->Vertices[2] = 0.5f*V2(Dim.x, -Dim.y);
    CurrPolygon->Vertices[3] = 0.5f*V2(Dim.x, Dim.y);
        
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
        GpuVertices[4] = V3(CurrPolygon->Vertices[0], 0.0f);

        CurrPolygon->MeshId = SceneRenderMeshAdd(Scene, Mesh);
    }

}

//
// NOTE: Constraint Hashtable
//

inline constraint_hashtable ConstraintHashTableCreate(u32 MaxNumElements, u32 ProbeLength)
{
    // NOTE: Mostly written from memory but used this as reference: https://benhoyt.com/writings/hash-table-in-c/
    constraint_hashtable Result = {};
    Result.ProbeLength = ProbeLength;
    Result.MaxNumElements = MaxNumElements;
    Result.Keys = (u32*)MemoryAllocate(sizeof(*Result.Keys) * Result.MaxNumElements);
    Result.Values = (constraint_penetration_l3*)MemoryAllocate(sizeof(*Result.Values) * Result.MaxNumElements);
    
    return Result;
}

inline void ConstraintHashTableClear(constraint_hashtable* Table)
{
    // NOTE: Set all keys to default values
    ZeroMem(Table->Keys, sizeof(u32) * Table->MaxNumElements);
    Table->NumElements = 0;
}

inline u32 ConstraintHashTableGetKey(constraint_hashtable* Table, u32 FirstBodyId, u32 SecondBodyId)
{
    u32 Result = ((SecondBodyId & 0xFF) << 16) | (FirstBodyId & 0xFF);
    Result = Mod(Result, Table->MaxNumElements);

    return Result;
}

inline constraint_penetration_l3* ConstraintHashTableAdd(constraint_hashtable* Table, u32 FirstBodyId, u32 SecondBodyId)
{
    Assert(Table->NumElements < Table->MaxNumElements);
    constraint_penetration_l3* Result = 0;
    
    u32 Key = ConstraintHashTableGetKey(Table, FirstBodyId, SecondBodyId);

    // NOTE: Apply linear probing to find a empty spot
    b32 FoundSlot = false;
    for (u32 ProbeId = 0; ProbeId < Table->ProbeLength; ++ProbeId)
    {
        if (Table->Keys[Key + ProbeId] == 0)
        {
            // NOTE: We have a empty spot so add our key
            // NOTE: We store Key + 1 so that we can 0 init the table
            Table->Keys[Key + ProbeId] = Key + 1;
            Result = Table->Values + Key + ProbeId;
            Table->NumElements += 1;
            FoundSlot = true;
            break;
        }
    }

    if (!FoundSlot)
    {
        // TODO: Resize the hashtable
        InvalidCodePath;
    }

    return Result;
}

inline constraint_penetration_l3* ConstraintHashTableGet(constraint_hashtable* Table, u32 FirstBodyId, u32 SecondBodyId)
{
    constraint_penetration_l3* Result = 0;
    
    u32 Key = ConstraintHashTableGetKey(Table, FirstBodyId, SecondBodyId);

    // NOTE: Apply linear probing to find if the key is in a nearby slot
    b32 FoundSlot = false;
    for (u32 ProbeId = 0; ProbeId < Table->ProbeLength; ++ProbeId)
    {
        if (Table->Keys[Key + ProbeId] == Key + 1)
        {
            Result = Table->Values + Key + ProbeId;
            break;
        }
    }

    return Result;
}

//
// NOTE: Penetration Constraint Functions
//

inline void PenetrationConstrantL3Create(rigid_body_sim_l3* Sim, u32 FirstBodyId, u32 SecondBodyId, v2 ContactPoint1,
                                         v2 ContactPoint2, v2 Normal, f32 Depth)
{
    // NOTE: Search if we have this contact from the previous frame
    constraint_penetration_l3* PrevConstraint = 0;
    
#if LESSON3_USE_HASHTABLE

    //PrevConstraint = ConstraintHashTableGet(&Sim->PrevPenetrationConstraintTable, FirstBodyId, SecondBodyId);
    constraint_penetration_l3* Result = ConstraintHashTableAdd(&Sim->PenetrationConstraintTable, FirstBodyId, SecondBodyId);
#if LESSON3_USE_WARMSTART
    if (!(PrevConstraint && PrevConstraint->FirstBodyId == FirstBodyId && PrevConstraint->SecondBodyId == SecondBodyId &&
          LengthSquared(PrevConstraint->ContactPoint - ContactPoint) < Square(0.01f)))
    {
        Result = 0;
    }
#endif
    
#else

#if LESSON3_USE_WARMSTART
    for (u32 ContactId = 0; ContactId < Sim->PrevNumPenetrationConstraints; ++ContactId)
    {
        constraint_penetration_l3* CurrContact = Sim->PrevPenetrationConstraintArray + ContactId;
        if (CurrContact->FirstBodyId == FirstBodyId && CurrContact->SecondBodyId == SecondBodyId &&
            LengthSquared(CurrContact->ContactPoint1 - ContactPoint1) < 0.005f)
        {
            PrevConstraint = CurrContact;
            break;
        }
    }
#endif
    
    Assert(Sim->NumPenetrationConstraints < Sim->MaxNumPenetrationConstraints);
    constraint_penetration_l3* Result = Sim->PenetrationConstraintArray + Sim->NumPenetrationConstraints++;
    
#endif

    if (PrevConstraint)
    {
        *Result = *PrevConstraint;
        Result->ContactPoint1 = ContactPoint1;
        Result->ContactPoint2 = ContactPoint2;
        Result->Normal = Normal;
        Result->Depth = Depth;
    }
    else
    {
        *Result = {};
        Result->FirstBodyId = FirstBodyId;
        Result->SecondBodyId = SecondBodyId;
        Result->ContactPoint1 = ContactPoint1;
        Result->ContactPoint2 = ContactPoint2;
        Result->Normal = Normal;
        Result->Depth = Depth;
    }
}

//
// NOTE: Sim Functions
//

inline rigid_body_sim_l3 RigidBodySimL3Init(linear_arena* Arena, render_scene* Scene)
{    
    rigid_body_sim_l3 Result = {};
    Result.MaxNumRigidBodies = 100;
    Result.RigidBodyArray = PushArray(Arena, rigid_body_l3, Result.MaxNumRigidBodies);

#if LESSON3_USE_HASHTABLE

    Result.PrevPenetrationConstraintTable = ConstraintHashTableCreate(1000, 8);
    Result.PenetrationConstraintTable = ConstraintHashTableCreate(1000, 8);
    
#else
    
    Result.MaxNumPenetrationConstraints = 1000;
    Result.PrevPenetrationConstraintArray = PushArray(Arena, constraint_penetration_l3, Result.MaxNumPenetrationConstraints);
    Result.PenetrationConstraintArray = PushArray(Arena, constraint_penetration_l3, Result.MaxNumPenetrationConstraints);

#endif
    
    // NOTE: Listing various scenes below
    {
#if 0
        // NOTE: n circles shot at each other
        {
            Result.ApplyGravity = false;

            u32 NumCircles = 3;
            for (u32 CircleId = 0; CircleId < NumCircles; ++CircleId)
            {
                f32 Angle = 2.0f * Pi32 * f32(CircleId) / f32(NumCircles);
                v2 Pos = V2(Cos(Angle), Sin(Angle));
                
                RigidBodyL3CircleCreate(&Result, Pos, V2(0) - Pos, 0.0f, 0.0f, 1.0f / 25.0f, 0.25f);
            }
        }
#endif

#if 0
        // NOTE: Stack of circles
        {
            Result.ApplyGravity = true;

            RigidBodyL3CircleCreate(&Result, V2(0.0f, -2.0f), V2(0, 0), 0.0f, 0.0f, 0.0f, 0.25f);
            RigidBodyL3CircleCreate(&Result, V2(0.0f, -1.5f), V2(0, 0), 0.0f, 0.0f, 1.0f / 5.0f, 0.25f);
            RigidBodyL3CircleCreate(&Result, V2(0.0f, -1.0f), V2(0, 0), 0.0f, 0.0f, 1.0f / 5.0f, 0.25f);
            RigidBodyL3CircleCreate(&Result, V2(0.0f, -0.5f), V2(0, 0), 0.0f, 0.0f, 1.0f / 5.0f, 0.25f);
            
        }
#endif
        
#if 1
        // NOTE: Falling circles on a ground 1
        {
            f32 AreaWidth = 4.0f;
            
            Result.ApplyGravity = true;
            RigidBodyL3RectCreate(&Result, Arena, Scene, V2(0.0f, -2.0f), V2(0, 0), 0.0f, 0.0f, 0.0f, V2(100.0f, 0.5f));
            RigidBodyL3RectCreate(&Result, Arena, Scene, V2(0.5f * AreaWidth + 0.5f, -2.0f), V2(0, 0), 0.0f, 0.0f, 0.0f, V2(0.5f, 160.0f));
            RigidBodyL3RectCreate(&Result, Arena, Scene, V2(-0.5f * AreaWidth - 0.5f, -2.0f), V2(0, 0), 0.0f, 0.0f, 0.0f, V2(0.5f, 160.0f));
            
            // NOTE: Falling bodies
            //RigidBodyL3CircleCreate(&Result, V2(-0.6f, 0.0f), V2(0, 0), 0.0f, 0.0f, 1.0f / 105.0f, 0.15f);
            //RigidBodyL3CircleCreate(&Result, V2(-0.6f, -2.0f + 0.15f + 0.25f - 0.01f), V2(0.0f, -0.25f), 0.0f, 0.0f, 1.0f / 105.0f, 0.15f);
            //RigidBodyL3CircleCreate(&Result, V2(0.0f, 0.0f), V2(0, 0), 0.0f, 0.0f, 1.0f / 25.0f, 0.25f);
            //RigidBodyL3CircleCreate(&Result, V2(0.6f, 0.0f), V2(0, 0), 0.0f, 0.0f, 1.0f / 25.0f, 0.25f);

            u32 NumCircles = 40;
            for (u32 CircleId = 0; CircleId < NumCircles; ++CircleId)
            {
                v2 Pos = V2(0.5f * AreaWidth * (2.0f * RandFloat() - 1.0f), 3.0f * RandFloat());
                f32 Radius = RandFloat() * 0.5f;
                RigidBodyL3CircleCreate(&Result, Pos, V2(0, 0), 0.0f, 0.0f, 1.0f / 4.0f, Radius);
            }
        }
#endif
        
#if 0
        // NOTE: Falling circles on a ground 2
        {
            f32 AreaWidth = 1.0f;
            
            Result.ApplyGravity = true;
            RigidBodyL3RectCreate(&Result, Arena, Scene, V2(0.0f, -2.0f), V2(0, 0), 0.0f, 0.0f, 0.0f, V2(100.0f, 0.5f));
            RigidBodyL3RectCreate(&Result, Arena, Scene, V2(0.5f * AreaWidth + 0.5f, -2.0f), V2(0, 0), 0.0f, 0.0f, 0.0f, V2(0.5f, 160.0f));
            RigidBodyL3RectCreate(&Result, Arena, Scene, V2(-0.5f * AreaWidth - 0.5f, -2.0f), V2(0, 0), 0.0f, 0.0f, 0.0f, V2(0.5f, 160.0f));
     
            u32 NumCircles = 3;
            for (u32 CircleId = 0; CircleId < NumCircles; ++CircleId)
            {
                v2 Pos = V2(0.5f * AreaWidth * (2.0f * RandFloat() - 1.0f), 3.0f * RandFloat());
                f32 Radius = AreaWidth / 3.0f;
                RigidBodyL3CircleCreate(&Result, Pos, V2(0, 0), 0.0f, 0.0f, 1.0f / 4.0f, Radius);
            }
        }
#endif
        
#if 0
        // NOTE: Falling circles on a ground 3
        {
            f32 AreaWidth = 1.0f;
            
            Result.ApplyGravity = true;
            RigidBodyL3RectCreate(&Result, Arena, Scene, V2(0.0f, -2.0f), V2(0, 0), 0.0f, 0.0f, 0.0f, V2(100.0f, 0.5f));
            RigidBodyL3RectCreate(&Result, Arena, Scene, V2(0.5f * AreaWidth + 0.5f, -2.0f), V2(0, 0), 0.0f, 0.0f, 0.0f, V2(0.5f, 160.0f));
            RigidBodyL3RectCreate(&Result, Arena, Scene, V2(-0.5f * AreaWidth - 0.5f, -2.0f), V2(0, 0), 0.0f, 0.0f, 0.0f, V2(0.5f, 160.0f));

            f32 Radius = AreaWidth / 3.0f;
            RigidBodyL3CircleCreate(&Result, V2(-0.5f*AreaWidth, 2.0f), V2(0, 0), 0.0f, 0.0f, 1.0f / 4.0f, Radius);
            RigidBodyL3CircleCreate(&Result, V2(0.5f*AreaWidth, 2.0f), V2(0, 0), 0.0f, 0.0f, 1.0f / 4.0f, Radius);
            RigidBodyL3CircleCreate(&Result, V2(0.0f, 4.0f), V2(0, 0), 0.0f, 0.0f, 1.0f / 4.0f, Radius);
        }
#endif
    }
    
    return Result;
}

inline f32 RigidBodyComputeTorqueL3(v2 Force, v2 OffsetPoint)
{
    f32 Result = OffsetPoint.x * Force.y - OffsetPoint.y * Force.x;;
    return Result;
}

inline void RigidBodySimUpdate(rigid_body_sim_l3* Sim, f32 FrameTime, render_scene* Scene)
{    
#if 1
    // NOTE: Integrate forces to get velocities
    for (u32 CurrBodyId = 0; CurrBodyId < Sim->NumRigidBodies; ++CurrBodyId)
    {
        rigid_body_l3* CurrBody = Sim->RigidBodyArray + CurrBodyId;

        f32 Torque = 0.0f;
        v2 Accel = V2(0);
        if (Sim->ApplyGravity && CurrBody->InvMass != 0.0f)
        {
            Accel = 9.81f * V2(0, -1);
        }

        CurrBody->Vel += Accel * FrameTime;
        CurrBody->AngleVel += Torque * CurrBody->InvInertia;

        // NOTE: Damping
        //CurrBody->Vel *= Pow(0.97f, 0.4f);
        CurrBody->AngleVel *= Pow(0.97f, 0.4f);
    }
#endif
    
    // NOTE: Find Collisions
    for (u32 CurrBodyId = 0; CurrBodyId < Sim->NumRigidBodies; ++CurrBodyId)
    {
        rigid_body_l3* CurrBody = Sim->RigidBodyArray + CurrBodyId;
        
        for (u32 OtherBodyId = CurrBodyId + 1; OtherBodyId < Sim->NumRigidBodies; ++OtherBodyId)
        {
            rigid_body_l3* OtherBody = Sim->RigidBodyArray + OtherBodyId;

            if (!(CurrBody->InvMass == 0.0f && OtherBody->InvMass == 0.0f))
            {     
#if LESSON3_USE_GJK

                gjk_support_l2* SupportA = CurrBody->Type == RigidBodyTypeL2_Circle ? SupportCircle2d : SupportConvexPolygon2d;
                gjk_support_l2* SupportB = OtherBody->Type == RigidBodyTypeL2_Circle ? SupportCircle2d : SupportConvexPolygon2d;

                void* GeometryA = CurrBody->Type == RigidBodyTypeL2_Circle ? (void*)&CurrBody->Circle : (void*)&CurrBody->Polygon;
                void* GeometryB = OtherBody->Type == RigidBodyTypeL2_Circle ? (void*)&OtherBody->Circle : (void*)&OtherBody->Polygon;

                gjk_result_2d CollisionResult = Gjk2d(GeometryA, CurrBody->Pos, CurrBody->Angle, SupportA,
                                                      GeometryB, OtherBody->Pos, OtherBody->Angle, SupportB);

                if (CollisionResult.Intersects)
                {
                    DebugPushPoint(CollisionResult.ContactPoint1, V4(1, 0, 1, 1));
                    DebugPushPoint(CollisionResult.ContactPoint2, V4(1, 0, 1, 1));
                    PenetrationConstrantL3Create(Sim, CurrBodyId, OtherBodyId, CollisionResult.ContactPoint1,
                                                 CollisionResult.ContactPoint2, CollisionResult.Normal, -CollisionResult.Distance);

                    if (CurrBody->Type == RigidBodyTypeL3_Circle && OtherBody->Type == RigidBodyTypeL3_Circle)
                    {
                        // NOTE: Draw normals
                        DebugPushLine(CurrBody->Pos, CurrBody->Pos + 5.0f * CollisionResult.Distance* CollisionResult.Normal, V4(0, 0, 0, 1));
                    }
                }
                
#else

                if (CurrBody->Type == RigidBodyTypeL3_Circle && OtherBody->Type == RigidBodyTypeL3_Circle)
                {
                    collision_result_l3 CollisionResult = CircleCollision(CurrBody->Pos, CurrBody->Circle.Radius, OtherBody->Pos, OtherBody->Circle.Radius);

                    if (CollisionResult.Collides)
                    {
                        DebugPushPoint(CollisionResult.ContactPoint, V4(1, 0, 1, 1));
                        PenetrationConstrantL3Create(Sim, CurrBodyId, OtherBodyId, CollisionResult.ContactPoint,
                                                     CollisionResult.ContactPoint, CollisionResult.Normal, -CollisionResult.Depth);
                    }
                }
                else if (CurrBody->Type == RigidBodyTypeL3_Circle && OtherBody->Type == RigidBodyTypeL3_Polygon)
                {
                    v2 BoxDim = Abs(OtherBody->Polygon.Vertices[0] - OtherBody->Polygon.Vertices[2]);
                    collision_result_l3 CollisionResult = CircleToOBB(CurrBody->Pos, CurrBody->Circle.Radius, OtherBody->Pos,
                                                                      BoxDim);

                    if (CollisionResult.Collides)
                    {
                        DebugPushPoint(CollisionResult.ContactPoint, V4(1, 0, 1, 1));
                        PenetrationConstrantL3Create(Sim, CurrBodyId, OtherBodyId, CollisionResult.ContactPoint,
                                                     CollisionResult.ContactPoint, CollisionResult.Normal, -CollisionResult.Depth);
                    }
                }
                else if (CurrBody->Type == RigidBodyTypeL3_Polygon && OtherBody->Type == RigidBodyTypeL3_Circle)
                {
                    v2 BoxDim = Abs(CurrBody->Polygon.Vertices[0] - CurrBody->Polygon.Vertices[2]);
                    collision_result_l3 CollisionResult = CircleToOBB(OtherBody->Pos, OtherBody->Circle.Radius, CurrBody->Pos,
                                                                      BoxDim);

                    if (CollisionResult.Collides)
                    {
                        DebugPushPoint(CollisionResult.ContactPoint, V4(1, 0, 1, 1));
                        PenetrationConstrantL3Create(Sim, OtherBodyId, CurrBodyId, CollisionResult.ContactPoint,
                                                     CollisionResult.ContactPoint, CollisionResult.Normal, -CollisionResult.Depth);
                    }
                }
                
#endif
            }
        }        
    }
    
#if 1
    // NOTE: Sequential-Impulse/Projected-Gauss-Seidel solver
    {
        // TODO: Add friction
        
        // NOTE: Prestep, calculate constants that don't depend on each iteration and apply warm starting
#if LESSON3_USE_HASHTABLE
        constraint_hashtable* ConstraintTable = &Sim->PenetrationConstraintTable;
        for (u32 CurrConstraintId = 0; CurrConstraintId < ConstraintTable->MaxNumElements; ++CurrConstraintId)
        {
            if (ConstraintTable->Keys[CurrConstraintId] == 0)
            {
                continue;
            }
            
            constraint_penetration_l3* Constraint = ConstraintTable->Values + CurrConstraintId;

#else
        for (u32 CurrConstraintId = 0; CurrConstraintId < Sim->NumPenetrationConstraints; ++CurrConstraintId)
        {
            constraint_penetration_l3* Constraint = Sim->PenetrationConstraintArray + CurrConstraintId;
#endif
            
            /*
              NOTE: Penetration constraint derivation:

              [ -n, -RA*n, n, RB*n ] * Diag(1/ma, 1/ia, 1/mb, 1/ib) * Transpose([ -n, -RA*n, n, RB*n ])
              = [ -n/ma, -RA*n/ia, n/mb, RB*n/ib ] * Transpose([ -n, -RA*n, n, RB*n ])
              = Dot(n, n)/ma + (RA*n)*(RA*n)/ia + Dot(n, n)/mb + (RB*n)*(RB*n)/ib
              = (1/ma + 1/mb) + (Dot(RA, n)^2 / ia + Dot(RB, n)^2) / ib)

              ^ The above doesn't depend on velocity so we can precalculate
          
            */
            
            rigid_body_l3* BodyA = Sim->RigidBodyArray + Constraint->FirstBodyId;
            rigid_body_l3* BodyB = Sim->RigidBodyArray + Constraint->SecondBodyId;

#if LESSON3_USE_DERIVED_MATH
            
            v2 RA = Constraint->ContactPoint1 - BodyA->Pos;
            v2 RB = Constraint->ContactPoint2 - BodyB->Pos;
            
            Constraint->InvJMJT = (BodyB->InvMass + BodyA->InvMass +
                                   Square(Cross(-RA, Constraint->Normal)) * BodyA->InvInertia +
                                   Square(Cross(RB, Constraint->Normal)) * BodyB->InvInertia);
            Constraint->InvJMJT = 1.0f / Constraint->InvJMJT;

            // NOTE: Calculate restitution
            v2 Jq2Vec = (BodyB->Vel + Cross(V3(0.0f, 0.0f, BodyB->AngleVel), V3(RB, 0.0f)).xy -
                         BodyA->Vel - Cross(V3(0.0f, 0.0f, BodyA->AngleVel), V3(RA, 0.0f)).xy);
            f32 Jq2 = Dot(Jq2Vec, Constraint->Normal);
            f32 ResitutionCoeff = Max(BodyA->RestitutionCoeff, BodyB->RestitutionCoeff);
            Constraint->Restitution = Jq2 < -1.0f ? Jq2 * ResitutionCoeff : 0.0f;

            // NOTE: Calculate position bias
            f32 BiasSlop = 0.004f;
            f32 BiasFactor = 0.1f;
            Constraint->Bias = Min(0.0f, Constraint->Depth + BiasSlop) * BiasFactor / FrameTime;

            // NOTE: Warm starting
            if (Constraint->AccumImpulse != 0.0f)
            {
                v2 ApplyImpulse = Constraint->AccumImpulse * Constraint->Normal;
                
                BodyA->Vel -= ApplyImpulse * BodyA->InvMass;
                BodyB->Vel += ApplyImpulse * BodyB->InvMass;

                v2 AngleChange1 = Cross(V3(RA, 0.0f), V3(ApplyImpulse, 0.0f)).xy * BodyA->InvInertia;
                v2 AngleChange2 = Cross(V3(RB, 0.0f), V3(ApplyImpulse, 0.0f)).xy * BodyB->InvInertia;
                
                BodyA->AngleVel -= AngleChange1;
                BodyB->AngleVel += AngleChange2;
            }

#else
            
            v2 RA = -(Constraint->ContactPoint1 - BodyA->Pos);
            v2 RB = -(Constraint->ContactPoint2 - BodyB->Pos);
            
            Constraint->InvJMJT = (BodyB->InvMass + BodyA->InvMass +
                                   Square(Cross(Constraint->Normal, RA)) * BodyA->InvInertia +
                                   Square(Cross(Constraint->Normal, RB)) * BodyB->InvInertia);
            Constraint->InvJMJT = 1.0f / Constraint->InvJMJT;

            // NOTE: Calculate restitution
            v2 Jq2Vec = BodyB->Vel + Cross(RB, BodyB->AngleVel) - BodyA->Vel - Cross(RA, BodyA->AngleVel);
            f32 Jq2 = Dot(Jq2Vec, Constraint->Normal);
            f32 ResitutionCoeff = Max(BodyA->RestitutionCoeff, BodyB->RestitutionCoeff);
            Constraint->Restitution = Jq2 < -1.0f ? Jq2 * ResitutionCoeff : 0.0f;

            // NOTE: Calculate position bias
            f32 BiasSlop = 0.004f;
            f32 BiasFactor = 0.1f;
            Constraint->Bias = Min(0.0f, Constraint->Depth + BiasSlop) * BiasFactor / FrameTime;

            // NOTE: Warm starting
            if (Constraint->AccumImpulse != 0.0f)
            {
                v2 ApplyImpulse = Constraint->AccumImpulse * Constraint->Normal;
                
                BodyA->Vel -= ApplyImpulse * BodyA->InvMass;
                BodyB->Vel += ApplyImpulse * BodyB->InvMass;
                BodyA->AngleVel -= Cross(ApplyImpulse, RA) * BodyA->InvInertia;
                BodyB->AngleVel += Cross(ApplyImpulse, RB) * BodyB->InvInertia;
            }

#endif
        }
        
        for (u32 IterationId = 0; IterationId < 12; ++IterationId)
        {
#if LESSON3_USE_HASHTABLE
            for (u32 CurrConstraintId = 0; CurrConstraintId < ConstraintTable->MaxNumElements; ++CurrConstraintId)
            {
                if (ConstraintTable->Keys[CurrConstraintId] == 0)
                {
                    continue;
                }
            
                constraint_penetration_l3* Constraint = ConstraintTable->Values + CurrConstraintId;

#else
            for (u32 CurrConstraintId = 0; CurrConstraintId < Sim->NumPenetrationConstraints; ++CurrConstraintId)
            {
                constraint_penetration_l3* Constraint = Sim->PenetrationConstraintArray + CurrConstraintId;
#endif

                /*
                  NOTE: Penetration Constraitn Derivation:

                  Lambda = -Jq2* - b / JMJT <- This value is precalculated

                    -Jq2* = the derivative of dC/dt with our current velocity

                    [ -n, -RA*n, n, RB*n ] * Transpose(VelA, AngleVelA, VelB, AngleVelB)
                    = Dot(VelB - VelA + Rot(RB, AngleVelB) - Rot(RA, AngleVelA), n)

                    TODO: Why is Jt just the normal? Sorta get it but derive
                    
                 */
                
                // NOTE: We apply the update to both bodies
                rigid_body_l3* BodyA = Sim->RigidBodyArray + Constraint->FirstBodyId;
                rigid_body_l3* BodyB = Sim->RigidBodyArray + Constraint->SecondBodyId;

#if LESSON3_USE_DERIVED_MATH
                
                v2 RA = Constraint->ContactPoint1 - BodyA->Pos;
                v2 RB = Constraint->ContactPoint2 - BodyB->Pos;

                v2 Jq2Vec = (BodyB->Vel + Cross(V3(0.0f, 0.0f, BodyB->AngleVel), V3(RB, 0.0f)).xy -
                             BodyA->Vel - Cross(V3(0.0f, 0.0f, BodyA->AngleVel), V3(RA, 0.0f)).xy);
                f32 Jq2 = Dot(Jq2Vec, Constraint->Normal);
                f32 Lambda = -(Jq2 + Constraint->Restitution + Constraint->Bias) * Constraint->InvJMJT;
                
                // NOTE: Negative means we are inside the object, we don't want to over shoot so we clamp
                f32 SavedAccumImpulse = Constraint->AccumImpulse;
                Constraint->AccumImpulse = Max(Constraint->AccumImpulse + Lambda, 0.0f);
                v2 ApplyImpulse = Constraint->Normal * (Constraint->AccumImpulse - SavedAccumImpulse);

                // NOTE: Apply impulse to our objects
                BodyA->Vel -= ApplyImpulse * BodyA->InvMass;
                BodyB->Vel += ApplyImpulse * BodyB->InvMass;
                
                v2 AngleChange1 = Cross(V3(RA, 0.0f), V3(ApplyImpulse, 0.0f)).xy * BodyA->InvInertia;
                v2 AngleChange2 = Cross(V3(RB, 0.0f), V3(ApplyImpulse, 0.0f)).xy * BodyB->InvInertia;
                
                BodyA->AngleVel -= AngleChange1;
                BodyB->AngleVel += AngleChange2;

#else
                
                v2 RA = -(Constraint->ContactPoint1 - BodyA->Pos);
                v2 RB = -(Constraint->ContactPoint2 - BodyB->Pos);

                f32 Jq2 = Dot(BodyB->Vel + Cross(RB, BodyB->AngleVel) - BodyA->Vel - Cross(RA, BodyA->AngleVel), Constraint->Normal);
                f32 Lambda = -(Jq2 + Constraint->Restitution + Constraint->Bias) * Constraint->InvJMJT;
                
                // NOTE: Negative means we are inside the object, we don't want to over shoot so we clamp
                f32 SavedAccumImpulse = Constraint->AccumImpulse;
                Constraint->AccumImpulse = Max(Constraint->AccumImpulse + Lambda, 0.0f);
                v2 ApplyImpulse = Constraint->Normal * (Constraint->AccumImpulse - SavedAccumImpulse);

                // NOTE: Apply impulse to our objects
                BodyA->Vel -= ApplyImpulse * BodyA->InvMass;
                BodyB->Vel += ApplyImpulse * BodyB->InvMass;
                BodyA->AngleVel -= Cross(ApplyImpulse, RA) * BodyA->InvInertia;
                BodyB->AngleVel += Cross(ApplyImpulse, RB) * BodyB->InvInertia;

#endif
            }
        }
    }
    
    // NOTE: Integrate velocities to get positions
    for (u32 CurrBodyId = 0; CurrBodyId < Sim->NumRigidBodies; ++CurrBodyId)
    {
        rigid_body_l3* CurrBody = Sim->RigidBodyArray + CurrBodyId;

        CurrBody->Pos += CurrBody->Vel * FrameTime;
        CurrBody->Angle += CurrBody->AngleVel * FrameTime;

#if LESSON3_VISUALIZE_VELOCITY
        DebugPushLine(CurrBody->Pos, CurrBody->Pos + CurrBody->Vel, V4(1, 0, 0, 1));
#endif
    }
#endif

    // NOTE: Generate render data
    for (u32 CurrBodyId = 0; CurrBodyId < Sim->NumRigidBodies; ++CurrBodyId)
    {
        rigid_body_l3* CurrBody = Sim->RigidBodyArray + CurrBodyId;
        v4 Color = V4(0, 1, 0, 1);

        f32 Scale = 2.0f * CurrBody->Circle.Radius;
        u32 MeshId = DemoState->CircleId;
        if (CurrBody->Type == RigidBodyTypeL3_Polygon)
        {
            Scale = 1.0f;
            MeshId = CurrBody->Polygon.MeshId;
        }
        
        m4 Transform = (M4Pos(V3(CurrBody->Pos, 0)) *
                        M4Rotation(V3(0.0f, 0.0f, CurrBody->Angle))*
                        M4Scale(V3(Scale)));

        SceneOpaqueInstanceAdd(Scene, MeshId, Transform, Color);
    }

    {
#if LESSON3_USE_HASHTABLE

        // NOTE: Swap the constraint tables
        constraint_hashtable SavedTable = Sim->PrevPenetrationConstraintTable;
        Sim->PrevPenetrationConstraintTable = Sim->PenetrationConstraintTable;
        Sim->PenetrationConstraintTable = SavedTable;

        ConstraintHashTableClear(&Sim->PenetrationConstraintTable);
        
#else

        // NOTE: Swap the constraint arrays
        Sim->PrevNumPenetrationConstraints = Sim->NumPenetrationConstraints;
        Sim->NumPenetrationConstraints = 0;

        {
            constraint_penetration_l3* Temp = Sim->PenetrationConstraintArray;
            Sim->PenetrationConstraintArray = Sim->PrevPenetrationConstraintArray;
            Sim->PrevPenetrationConstraintArray = Temp;
        }
        
#endif
    }
}

