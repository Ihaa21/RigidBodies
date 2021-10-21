
//
// NOTE: Circle Collision
//

inline collision_result_l3 CircleCollision(v2 PosA, f32 RadiusA, v2 PosB, f32 RadiusB)
{
    collision_result_l3 Result = {};

    // TODO: Handle when PosA == PosB
    
    v2 DistVec = PosB - PosA;
    f32 Dist = Length(DistVec);
    Result.Collides = Dist < (RadiusA + RadiusB);

    if (Result.Collides)
    {
        Result.Depth = (RadiusA + RadiusB) - Dist;
        Result.Normal = DistVec / Dist;
        // NOTE: https://github.com/granj2020/Cirobb-Engine/blob/master/cirobb/Collision.cpp
        // NOTE: Instead of 2 contact points, we generate a single center one
        Result.ContactPoint = PosA + Result.Normal * (RadiusA - RadiusB + Dist) * 0.5f;
    }

    return Result;
}

//
// NOTE: Rigid Body Functions
//

inline void RigidBodyL3Create(rigid_body_sim_l3* Sim, v2 Pos, v2 Vel, f32 Angle, f32 AngleVel, f32 InvMass, f32 Radius)
{
    Assert(Sim->NumRigidBodies < Sim->MaxNumRigidBodies);
    rigid_body_l3* Result = Sim->RigidBodyArray + Sim->NumRigidBodies++;

    *Result = {};
    Result->InvMass = InvMass;
    Result->InvInertia = 0.5f * InvMass / (0.25f * Square(Radius)); // NOTE: 1 / (1/4 * m * r^2)
    Result->RestitutionCoeff = 0.95f;
    
    Result->Pos = Pos;
    Result->Vel = Vel;
    Result->Angle = Angle;
    Result->AngleVel = AngleVel;
    Result->Circle.Radius = Radius;
}

//
// NOTE: Penetration Constraint Functions
//

inline void PenetrationConstrantL3Create(rigid_body_sim_l3* Sim, u32 FirstBodyId, u32 SecondBodyId, v2 ContactPoint,
                                         v2 Normal, f32 Depth)
{
    Assert(Sim->NumPenetrationConstraints < Sim->MaxNumPenetrationConstraints);
    constraint_penetration_l3* Result = Sim->PenetrationConstraintArray + Sim->NumPenetrationConstraints++;

    // TODO: We want a hashtable here
    *Result = {};
    Result->FirstBodyId = FirstBodyId;
    Result->SecondBodyId = SecondBodyId;
    Result->ContactPoint = ContactPoint;
    Result->Normal = Normal;
    Result->Depth = Depth;
}

//
// NOTE: Sim Functions
//

inline rigid_body_sim_l3 RigidBodySimL3Init(linear_arena* Arena, render_scene* Scene)
{
    rigid_body_sim_l3 Result = {};
    Result.MaxNumRigidBodies = 100;
    Result.RigidBodyArray = PushArray(Arena, rigid_body_l3, Result.MaxNumRigidBodies);

    Result.MaxNumPenetrationConstraints = 1000;
    Result.PenetrationConstraintArray = PushArray(Arena, constraint_penetration_l3, Result.MaxNumPenetrationConstraints);
    
    // NOTE: Listing various scenes below
    {
#if 0
        // NOTE: n circles shot at each other
        {
            Result.ApplyGravity = false;

            u32 NumCircles = 3;
            for (u32 CircleId = 0; CircleId < NumCircles; ++CircleId)
            {
                f32 Angle = 2.0f * Pi32 * f32(CircleId) / f32(NumCircles);
                v2 Pos = V2(Cos(Angle), Sin(Angle));
                
                RigidBodyL3Create(&Result, Pos, V2(0) - Pos, 0.0f, 0.0f, 1.0f / 25.0f, 0.25f);
            }
        }
#endif

#if 0
        // NOTE: Stack of circles
        {
            Result.ApplyGravity = true;

            RigidBodyL3Create(&Result, V2(0.0f, -2.0f), V2(0, 0), 0.0f, 0.0f, 0.0f, 0.25f);
            RigidBodyL3Create(&Result, V2(0.0f, -1.5f), V2(0, 0), 0.0f, 0.0f, 1.0f / 5.0f, 0.25f);
            RigidBodyL3Create(&Result, V2(0.0f, -1.0f), V2(0, 0), 0.0f, 0.0f, 1.0f / 5.0f, 0.25f);
            RigidBodyL3Create(&Result, V2(0.0f, -0.5f), V2(0, 0), 0.0f, 0.0f, 1.0f / 5.0f, 0.25f);
            
        }
#endif
        
#if 0
        // NOTE: Falling circles on a ground
        {
            Result.ApplyGravity = true;

            // TODO: Add rectangles and polygons to this, right now im faking a floor with circles
            for (f32 StartX = -2.0f; StartX <= 2.0f; StartX += 0.15f)
            {
                RigidBodyL3Create(&Result, V2(StartX, -2.0f), V2(0, 0), 0.0f, 0.0f, 0.0f, 0.25f);
            }

            /*
        m   0.196349546 float
        invm    5.09295797  float
        I   0.00613592332   float
        invI    162.974655  float

             */
            
            // NOTE: Falling bodies
            RigidBodyL3Create(&Result, V2(-0.6f, 0.0f), V2(0, 0), 0.0f, 0.0f, 5.09295797f, 0.25f);
            //RigidBodyL3Create(&Result, V2(0.0f, 0.0f), V2(0, 0), 0.0f, 0.0f, 1.0f / 25.0f, 0.25f);
            //RigidBodyL3Create(&Result, V2(0.6f, 0.0f), V2(0, 0), 0.0f, 0.0f, 1.0f / 25.0f, 0.25f);
            
        }
#endif
    }
    
    return Result;
}

inline f32 RigidBodyComputeTorqueL3(v2 Force, v2 OffsetPoint)
{
    f32 Result = OffsetPoint.x * Force.y - OffsetPoint.y * Force.x;;
    return Result;
}

inline void RigidBodySimUpdate(rigid_body_sim_l3* Sim, f32 FrameTime, render_scene* Scene)
{
    // TODO: Add warm starting
    Sim->NumPenetrationConstraints = 0;

    // NOTE: Integrate forces to get velocities
    for (u32 CurrBodyId = 0; CurrBodyId < Sim->NumRigidBodies; ++CurrBodyId)
    {
        rigid_body_l3* CurrBody = Sim->RigidBodyArray + CurrBodyId;

        f32 Torque = 0.0f;
        v2 Accel = V2(0);
        if (Sim->ApplyGravity && CurrBody->InvMass != 0.0f)
        {
            Accel = 9.81f * V2(0, -1);
        }

        CurrBody->Vel += Accel * FrameTime;
        CurrBody->AngleVel += Torque * CurrBody->InvInertia;
    }
    
    // NOTE: Find Collisions
    for (u32 CurrBodyId = 0; CurrBodyId < Sim->NumRigidBodies; ++CurrBodyId)
    {
        rigid_body_l3* CurrBody = Sim->RigidBodyArray + CurrBodyId;
        
        for (u32 OtherBodyId = CurrBodyId + 1; OtherBodyId < Sim->NumRigidBodies; ++OtherBodyId)
        {
            rigid_body_l3* OtherBody = Sim->RigidBodyArray + OtherBodyId;

            if (!(CurrBody->InvMass == 0.0f && OtherBody->InvMass == 0.0f))
            {
                collision_result_l3 CollisionResult = CircleCollision(CurrBody->Pos, CurrBody->Circle.Radius, OtherBody->Pos, OtherBody->Circle.Radius);

                if (CollisionResult.Collides)
                {
                    DebugPushPoint(CollisionResult.ContactPoint, V4(1, 0, 1, 1));
                    PenetrationConstrantL3Create(Sim, CurrBodyId, OtherBodyId, CollisionResult.ContactPoint, CollisionResult.Normal, CollisionResult.Depth);
                }
            }
        }        
    }
    
    // NOTE: Sequential-Impulse/Projected-Gauss-Seidel solver
    {
        // TODO: Add friction
        
        // NOTE: Prestep, calculate constants that don't depend on each iteration
        for (u32 CurrConstraintId = 0; CurrConstraintId < Sim->NumPenetrationConstraints; ++CurrConstraintId)
        {
            constraint_penetration_l3* Constraint = Sim->PenetrationConstraintArray + CurrConstraintId;

            /*
              NOTE: Penetration constraint derivation:

              [ -n, -RA*n, n, RB*n ] * Diag(1/ma, 1/ia, 1/mb, 1/ib) * Transpose([ -n, -RA*n, n, RB*n ])
              = [ -n/ma, -RA*n/ia, n/mb, RB*n/ib ] * Transpose([ -n, -RA*n, n, RB*n ])
              = Dot(n, n)/ma + (RA*n)*(RA*n)/ia + Dot(n, n)/mb + (RB*n)*(RB*n)/ib
              = (1/ma + 1/mb) + (Dot(RA, n)^2 / ia + Dot(RB, n)^2) / ib)

              ^ The above doesn't depend on velocity so we can precalculate
          
            */
            
            // NOTE: We apply the update to both bodies
            rigid_body_l3* BodyA = Sim->RigidBodyArray + Constraint->FirstBodyId;
            rigid_body_l3* BodyB = Sim->RigidBodyArray + Constraint->SecondBodyId;
            
            v2 RA = Constraint->ContactPoint - BodyA->Pos;
            v2 RB = Constraint->ContactPoint - BodyB->Pos;
            
            Constraint->InvJMJT = (BodyB->InvMass + BodyA->InvMass +
                                   Square(Cross(RA, Constraint->Normal)) * BodyA->InvInertia +
                                   Square(Cross(RB, Constraint->Normal)) * BodyB->InvInertia);
            Constraint->InvJMJT = 1.0f / Constraint->InvJMJT;

            // TODO: How do you derive restitution??
            v2 Jq2Vec = BodyB->Vel - BodyA->Vel + Cross(RB, BodyB->AngleVel) - Cross(RA, BodyA->AngleVel);
            f32 Jq2 = Dot(Jq2Vec, Constraint->Normal);
            f32 ResitutionCoeff = Max(BodyA->RestitutionCoeff, BodyB->RestitutionCoeff);
            Constraint->Restitution = Jq2 < -1.0f ? Jq2 * ResitutionCoeff : 0.0f;
            
            f32 BiasSlop = 0.004f;
            f32 BiasFactor = 0.1f;
            Constraint->Bias = Min(0.0f, Constraint->Depth + BiasSlop) * BiasFactor / FrameTime;
        }
        
        for (u32 IterationId = 0; IterationId < 1200; ++IterationId)
        {
            for (u32 CurrConstraintId = 0; CurrConstraintId < Sim->NumPenetrationConstraints; ++CurrConstraintId)
            {
                constraint_penetration_l3* Constraint = Sim->PenetrationConstraintArray + CurrConstraintId;

                /*
                  NOTE: Penetration Constraitn Derivation:

                  Lambda = -Jq2* - b / JMJT <- This value is precalculated

                    -Jq2* = the derivative of dC/dt with our current velocity

                    [ -n, -RA*n, n, RB*n ] * Transpose(VelA, AngleVelA, VelB, AngleVelB)
                    = Dot(VelB - VelA + Rot(RB, AngleVelB) - Rot(RA, AngleVelA), n)

                    TODO: Why is Jt just the normal?

                    TODO: How does the Cross work in this case for rotation as well as apply impulse?
                    
                 */
                
                // NOTE: We apply the update to both bodies
                rigid_body_l3* BodyA = Sim->RigidBodyArray + Constraint->FirstBodyId;
                rigid_body_l3* BodyB = Sim->RigidBodyArray + Constraint->SecondBodyId;
            
                v2 RA = Constraint->ContactPoint - BodyA->Pos;
                v2 RB = Constraint->ContactPoint - BodyB->Pos;

                // TODO: Why was this failing?
                //m2 TransformA = M2Rotation(BodyA->AngleVel);
                //m2 TransformB = M2Rotation(BodyB->AngleVel);
                //f32 Jq2 = Dot(BodyB->Vel - BodyA->Vel + TransformB * RB - TransformA * RA, Constraint->Normal);
                f32 Jq2 = Dot(BodyB->Vel - BodyA->Vel + Cross(RB, BodyB->Angle) - Cross(RA, BodyA->Angle), Constraint->Normal);

                f32 Lambda = -(Jq2 + Constraint->Restitution + Constraint->Bias) * Constraint->InvJMJT;
                
                // NOTE: Negative means we are inside the object, we don't want to over shoot so we clamp
                f32 SavedAccumImpulse = Constraint->AccumImpulse;
                Constraint->AccumImpulse += Lambda;
                Constraint->AccumImpulse = Max(Constraint->AccumImpulse, 0.0f);
                v2 ApplyImpulse = Constraint->Normal * (Constraint->AccumImpulse - SavedAccumImpulse);

                // NOTE: Apply impulse to our objects
                BodyA->Vel -= ApplyImpulse * BodyA->InvMass;
                BodyB->Vel += ApplyImpulse * BodyB->InvMass;
                BodyA->AngleVel -= Cross(ApplyImpulse, RA) * BodyA->InvInertia;
                BodyB->AngleVel += Cross(ApplyImpulse, RB) * BodyB->InvInertia;
            }
        }
    }
    
    // NOTE: Integrate velocities to get positions
    for (u32 CurrBodyId = 0; CurrBodyId < Sim->NumRigidBodies; ++CurrBodyId)
    {
        rigid_body_l3* CurrBody = Sim->RigidBodyArray + CurrBodyId;

        CurrBody->Pos += CurrBody->Vel * FrameTime;
        CurrBody->Angle += CurrBody->AngleVel * FrameTime;
    }

    // NOTE: Generate render data
    for (u32 CurrBodyId = 0; CurrBodyId < Sim->NumRigidBodies; ++CurrBodyId)
    {
        rigid_body_l3* CurrBody = Sim->RigidBodyArray + CurrBodyId;
        v4 Color = V4(0, 1, 0, 1);

        f32 Scale = 2.0f * CurrBody->Circle.Radius;
        u32 MeshId = DemoState->CircleId;
        
        m4 Transform = (M4Pos(V3(CurrBody->Pos, 0)) *
                        M4Rotation(V3(0.0f, 0.0f, CurrBody->Angle))*
                        M4Scale(V3(Scale)));

        SceneOpaqueInstanceAdd(Scene, MeshId, Transform, Color);
    }
}

