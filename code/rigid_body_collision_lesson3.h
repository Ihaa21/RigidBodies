#pragma once

/*

  NOTE: Tons of references for this one:

   - https://www.toptal.com/game/video-game-physics-part-iii-constrained-rigid-body-simulation
   - https://ubm-twvideo01.s3.amazonaws.com/o1/vault/gdc09/slides/04-GDC09_Catto_Erin_Solver.pdf
   - https://github.com/granj2020/Cirobb-Engine

   Restitution:

   - https://en.wikipedia.org/wiki/Coefficient_of_restitution
  
 */

struct collision_result_l3
{
    v2 ContactPoint;
    v2 Normal;
    b32 Collides;
    f32 Depth;
};

struct constraint_penetration_l3
{
    u32 FirstBodyId;
    u32 SecondBodyId;

    v2 ContactPoint;
    v2 Normal;
    f32 Depth;

    // NOTE: Precalculated values
    f32 InvJMJT;
    f32 Restitution;
    f32 Bias;

    // NOTE: Accumulated values
    f32 AccumImpulse;
};

struct rigid_body_l3
{
    circle_2d Circle;
    
    f32 InvMass;
    f32 InvInertia;
    f32 RestitutionCoeff;

    v2 Pos;
    v2 Vel;
    
    f32 Angle;
    f32 AngleVel;
};

struct rigid_body_sim_l3
{
    b32 ApplyGravity;

    // TODO: Might make sesne to have fixed arrays
    u32 MaxNumPenetrationConstraints;
    u32 NumPenetrationConstraints;
    constraint_penetration_l3* PenetrationConstraintArray;
    
    u32 NumRigidBodies;
    u32 MaxNumRigidBodies;
    rigid_body_l3* RigidBodyArray;
};
