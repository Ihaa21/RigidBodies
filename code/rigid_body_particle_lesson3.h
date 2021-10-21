#pragma once

/*

  NOTE: Based on force constraint solver here: https://www.toptal.com/game/video-game-physics-part-iii-constrained-rigid-body-simulation

  * Note we are only doing one particle solver, not multi
  
 */

struct particle_sim_l3
{
    float Mass;

    v2 Pos;
    v2 Vel;
};

