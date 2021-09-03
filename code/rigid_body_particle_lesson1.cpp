
inline particle_sim_l1 ParticleSimL1Init(linear_arena* Arena)
{
    particle_sim_l1 Result = {};
    Result.NumParticles = 100;
    Result.ParticleArray = PushArray(Arena, particle_l1, Result.NumParticles);

    for (u32 ParticleId = 0; ParticleId < Result.NumParticles; ++ParticleId)
    {
        particle_l1* CurrParticle = Result.ParticleArray + ParticleId;
        CurrParticle->Pos = 2.0f * (2.0f * V2(RandFloat(), RandFloat()) - V2(1.0f));
        CurrParticle->Vel = 10.0f*V2(0.0f, RandFloat());
        CurrParticle->Mass = 3.0f;
    }
    
    return Result;
}

inline void ParticleSimUpdate(particle_sim_l1* Sim, f32 FrameTime, render_scene* Scene)
{
    for (u32 ParticleId = 0; ParticleId < Sim->NumParticles; ++ParticleId)
    {
        particle_l1* CurrParticle = Sim->ParticleArray + ParticleId;

        // NOTE: Force is just gravity
        v2 Force = 9.81f * V2(0, -1) * CurrParticle->Mass;

        // NOTE: Integrate
        CurrParticle->Vel += (Force / CurrParticle->Mass) * FrameTime;
        CurrParticle->Pos += CurrParticle->Vel * FrameTime;

        // NOTE: Populate draw instances
        SceneOpaqueInstanceAdd(Scene, DemoState->CircleId, M4Pos(V3(CurrParticle->Pos, 0))*M4Scale(V3(0.2f)), V4(1, 0, 0, 1));
    }
}
