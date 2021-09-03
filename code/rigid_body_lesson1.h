#pragma once

/*

  NOTE: https://www.toptal.com/game/video-game-physics-part-i-an-introduction-to-rigid-body-dynamics

    - Bodies have a center of mass which for uniform density shapes is the geometric center.
    - center of mass = 1/M * Integral(every point in volume, p(r) * r * dv)
        - M is total mass, r is the point, p(r) is the density/mass at that point
        
    - Since we are 2d, we have a scalar orientation stored in radians, with a scalar angular velocity
    - The analog of force in angles is torque
    - F = ma is Torque = angular_accel * moment_of_inertia <- acts like mass
    - moment of inertia = Integral(every point in volume, p(r) * dot(r, r) * dv)

    - Applied forces can add torque using the following formula: torque = |f| * |r| * sin(theta)
        - r is the offset from center of mass, theta is smallest angle between f and r
    - In 3D, its torque = cross(r, f)
  
 */

struct rigid_body_l1
{
    f32 Width;
    f32 Height;
    
    float Mass;
    float MomentOfInertia;

    v2 Pos;
    v2 Vel;
    
    f32 Angle;
    f32 AngleVel;
};

struct rigid_body_sim_l1
{
    u32 NumRigidBodies;
    rigid_body_l1* RigidBodyArray;
};
