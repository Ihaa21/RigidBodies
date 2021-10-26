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

    v2 ContactPoint1;
    v2 ContactPoint2;
    v2 Normal;
    f32 Depth;

    // NOTE: Precalculated values
    f32 InvJMJT;
    f32 Restitution;
    f32 Bias;

    // NOTE: Accumulated values
    f32 AccumImpulse;
};

enum rigid_body_type_l3
{
    RigidBodyTypeL3_None,
    
    RigidBodyTypeL3_Circle,
    RigidBodyTypeL3_Polygon,
};

struct rigid_body_l3
{
    u32 Type;
    union
    {
        circle_2d Circle;
        polygon_2d Polygon;
    };
    
    f32 InvMass;
    f32 InvInertia;
    f32 RestitutionCoeff;

    v2 Pos;
    v2 Vel;
    
    f32 Angle;
    f32 AngleVel;
};

struct constraint_hashtable
{
    u32 ProbeLength;
    u32 MaxNumElements;
    u32 NumElements;
    u32* Keys;
    constraint_penetration_l3* Values;
};

#define LESSON3_USE_DERIVED_MATH 1
#define LESSON3_USE_GJK 1
#define LESSON3_USE_HASHTABLE 0
#define LESSON3_USE_WARMSTART 0

#define LESSON3_VISUALIZE_VELOCITY 0

struct rigid_body_sim_l3
{
    b32 ApplyGravity;

#if LESSON3_USE_HASHTABLE

    constraint_hashtable PrevPenetrationConstraintTable;
    constraint_hashtable PenetrationConstraintTable;
    
#else
    // TODO: Might make sesne to have fixed arrays data structure
    u32 MaxNumPenetrationConstraints;

    u32 PrevNumPenetrationConstraints;
    constraint_penetration_l3* PrevPenetrationConstraintArray;
    
    u32 NumPenetrationConstraints;
    constraint_penetration_l3* PenetrationConstraintArray;
#endif
    
    u32 NumRigidBodies;
    u32 MaxNumRigidBodies;
    rigid_body_l3* RigidBodyArray;
};
