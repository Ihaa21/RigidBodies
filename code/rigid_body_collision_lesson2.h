#pragma once

/*

  NOTE: https://www.toptal.com/game/video-game-physics-part-ii-collision-detection-for-solid-objects

    - 
  
 */

struct circle_2d
{
    f32 Radius;
};

struct polygon_2d
{
    u32 NumVertices;
    u32 MeshId;
    v2* Vertices;
};

struct transform_2d
{
    v2 Pos;
    f32 Rotation;
};

enum rigid_body_type_l2
{
    RigidBodyTypeL2_None,
    
    RigidBodyTypeL2_Circle,
    RigidBodyTypeL2_Polygon,
};

struct rigid_body_l2
{
    u32 Type;
    f32 Angle;
    f32 AngleVel;
    f32 DistFromCenter;

    f32 Rotation;
    f32 RotationVel;
    
    union
    {
        circle_2d Circle;
        polygon_2d Polygon;
    };
};

struct rigid_body_sim_l2
{
    u32 NumRigidBodies;
    rigid_body_l2* RigidBodyArray;
};
