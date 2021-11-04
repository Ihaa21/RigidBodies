#pragma once

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

struct gjk_result_2d
{
    v2 ContactPoint1;
    v2 ContactPoint2;
    v2 Normal;
    f32 Distance;
    b32 Intersects;
};

#define GJK_VISUALIZE 0
#define EPA_VISUALIZE 0
#define EPA_ITERATION_ID -1
#define EPA_VISUALIZE_FINAL 1
#define EPA_MAX_ITERATIONS 16

#define GJK_SUPPORT_L2(name) v2 name(v2 Dir, void* Geometry, v2 Pos, f32 Rotation)
typedef GJK_SUPPORT_L2(gjk_support_l2);

