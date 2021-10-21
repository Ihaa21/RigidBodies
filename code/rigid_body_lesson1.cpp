
inline rigid_body_sim_l1 RigidBodySimL1Init(linear_arena* Arena)
{
    rigid_body_sim_l1 Result = {};
    Result.NumRigidBodies = 1;
    Result.RigidBodyArray = PushArray(Arena, rigid_body_l1, Result.NumRigidBodies);

    for (u32 RigidBodyId = 0; RigidBodyId < Result.NumRigidBodies; ++RigidBodyId)
    {
        rigid_body_l1* CurrRigidBody = Result.RigidBodyArray + RigidBodyId;
        *CurrRigidBody = {};
        
        CurrRigidBody->Pos = 2.0f * (2.0f * V2(RandFloat(), RandFloat()) - V2(1.0f));
        CurrRigidBody->Mass = 3.0f;

        CurrRigidBody->Width = 1.0f;
        CurrRigidBody->Height = 0.2f;
        // NOTE: https://en.wikipedia.org/wiki/List_of_moments_of_inertia
        CurrRigidBody->MomentOfInertia = CurrRigidBody->Mass * (Square(CurrRigidBody->Width) + Square(CurrRigidBody->Height)) / 12.0f;
    }
    
    return Result;
}

inline f32 RigidBodyComputeTorqueL1(v2 Force, v2 OffsetPoint)
{
    // NOTE: Cross product in 3d
    f32 Result = OffsetPoint.x * Force.y - OffsetPoint.y * Force.x;;
    return Result;
}

inline void RigidBodySimUpdate(rigid_body_sim_l1* Sim, f32 FrameTime, render_scene* Scene)
{
    for (u32 RigidBodyId = 0; RigidBodyId < Sim->NumRigidBodies; ++RigidBodyId)
    {
        rigid_body_l1* CurrBody = Sim->RigidBodyArray + RigidBodyId;

        // NOTE: Generate force and torque
        v2 Force = V2(0.0f, 5.0f);
        v2 OffsetPoint = V2(CurrBody->Width / 2.0f, CurrBody->Height / 2.0f);
        f32 Torque = RigidBodyComputeTorqueL1(Force, OffsetPoint);

        // NOTE: Apply gravity
        Force += 9.81f * V2(0, -1);
        
        // NOTE: Integrate position
        CurrBody->Vel += (Force / CurrBody->Mass) * FrameTime;
        CurrBody->Pos += CurrBody->Vel * FrameTime;

        // NOTE: Integrate orientation
        float AngleAccel = Torque / CurrBody->MomentOfInertia;
        CurrBody->AngleVel += AngleAccel * FrameTime;
        CurrBody->Angle += CurrBody->AngleVel * FrameTime;

        // NOTE: Populate draw instances
        m4 Transform = (M4Pos(V3(CurrBody->Pos, 0)) *
                        M4Rotation(V3(0.0f, 0.0f, CurrBody->Angle)) *
                        M4Scale(V3(CurrBody->Width, CurrBody->Height, 1.0f)));
        SceneOpaqueInstanceAdd(Scene, DemoState->SquareId, Transform, V4(1, 0, 0, 1));
    }
}
