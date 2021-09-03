#pragma once

/*

  NOTE: https://www.toptal.com/game/video-game-physics-part-i-an-introduction-to-rigid-body-dynamics

    Newtons 3 Laws:

      1) Inertia, objects like to stay in the same state they are without external forces
      2) F = ma
      3) Every action has a equal reaction (conservation of forces/momentum)
  
 */

struct particle_l1
{
    v2 Pos;
    v2 Vel;
    f32 Mass;
};

struct particle_sim_l1
{
    u32 NumParticles;
    particle_l1* ParticleArray;
};
