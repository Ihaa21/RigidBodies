
inline particle_sim_l3 ParticleSimL3Init(linear_arena* Arena)
{
    particle_sim_l3 Result = {};

    Result.Pos = V2(-1, 0);
    Result.Mass = 13.0f;
    
    return Result;
}

inline void ParticleSimUpdate(particle_sim_l3* Sim, f32 FrameTime, render_scene* Scene)
{
    // TODO: There is a bug here where we slowly get lower and lower. But we work in other cases. Why doesn't the cost function get
    // completely respected here? There also isn't a way to define a arbitrary distance from the origin, it gets lost in the derivatives
    // Is that the reason this fails?
    
    // NOTE: Compute external forces (gravity)
    v2 ForceExt = 9.81f * Sim->Mass * V2(0, -1);

    // NOTE: Compute constrained forces
    f32 MultiplierC = (Dot(-ForceExt, Sim->Pos) - Sim->Mass * Dot(Sim->Vel, Sim->Vel)) / Dot(Sim->Pos, Sim->Pos);
    v2 ForceC = MultiplierC * Sim->Pos;

    v2 Force = ForceExt + ForceC;
    
    // NOTE: Integrate position
    Sim->Vel += (Force / Sim->Mass) * FrameTime;
    Sim->Pos += Sim->Vel * FrameTime;

    f32 CostFunc = 0.5f * (Dot(Sim->Pos, Sim->Pos) - 1);
    Assert(Abs(CostFunc) < 0.01f);
    
    // NOTE: Populate draw instances
    m4 Transform = (M4Pos(V3(Sim->Pos, 0)) *
                    M4Scale(V3(0.5f, 0.5f, 1.0f)));
    SceneOpaqueInstanceAdd(Scene, DemoState->SquareId, Transform, V4(1, 0, 0, 1));
}
