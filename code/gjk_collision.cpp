
GJK_SUPPORT_L2(SupportCircle2d)
{
    circle_2d* Circle = (circle_2d*)Geometry;
    
    v2 NormalizedDir = Normalize(Dir);
    v2 Result = Pos + Circle->Radius * NormalizedDir;
    
    return Result;
}

GJK_SUPPORT_L2(SupportConvexPolygon2d)
{
    v2 Result = {};

    m2 RotationMat = M2Rotation(Rotation);
    m2 RotationTransposedMat = Transpose(RotationMat);
    v2 TransformedDir = RotationTransposedMat * Dir;

    polygon_2d* Polygon = (polygon_2d*)Geometry;

    f32 LargestDot = 0.0f;
    for (u32 VertexId = 0; VertexId < Polygon->NumVertices; ++VertexId)
    {
        f32 VertexDotProduct = Dot(TransformedDir, Polygon->Vertices[VertexId]);
        if (VertexDotProduct > LargestDot)
        {
            LargestDot = VertexDotProduct;
            Result = Polygon->Vertices[VertexId];
        }
    }

    Result = RotationMat * Result + Pos;
    
    return Result;
}

inline v2 Gjk2dGetPerp(v2 LineDir, v2 DirToOrigin)
{
    // TODO: I think this can be setup differently, debug a bit but this is a bit of a hack to workaround not knowing the correct order
    v2 Result = GetPerp(LineDir);
    if (Dot(Result, DirToOrigin) < 0)
    {
        Result = GetPerp(-LineDir);
    }

    return Result;
}

inline gjk_result_2d Gjk2d(void* GeometryA, v2 PosA, f32 AngleA, gjk_support_l2* SupportA,
                           void* GeometryB, v2 PosB, f32 AngleB, gjk_support_l2* SupportB)
{
    // NOTE: https://www.youtube.com/watch?v=Qupqu1xe7Io&ab_channel=CaseyMuratori
    // NOTE: https://www.youtube.com/watch?v=ajv46BSqcK4&ab_channel=Reducible (secondary, first one is better for code IMO)
    
    gjk_result_2d Result = {};

    u32 NumGjkPoints = 0;
    v2 GjkPoints[3] = {};
    v2 GjkGeoAPoints[3] = {};

#if GJK_VISUALIZE || EPA_VISUALIZE
    // NOTE: Visualize the Minkowski sum
    for (f32 Angle = 0.0f; Angle < 2.0f * Pi32; Angle += 0.01f)
    {
        v2 Dir0 = V2(Cos(Angle), Sin(Angle));
        v2 Dir1 = V2(Cos(Angle + 0.01f), Sin(Angle + 0.01f));
        v2 LineStart = (SupportA(Dir0, GeometryA, PosA, AngleA) - SupportB(-Dir0, GeometryB, PosB, AngleB));
        v2 LineEnd = (SupportA(Dir1, GeometryA, PosA, AngleA) - SupportB(-Dir1, GeometryB, PosB, AngleB));
        DebugPushLine(LineStart, LineEnd, V4(0, 0, 0, 1));
    }

    // NOTE: Origin
    DebugPushPoint(V2(0), V4(0, 0, 0, 1));
#endif

    // NOTE: GJK Algorithm
    {
        v2 Direction = PosA - PosB;
        if (Direction.x == 0.0f && Direction.y == 0.0f)
        {
            Direction = V2(1, 0);
        }

        GjkGeoAPoints[NumGjkPoints] = SupportA(Direction, GeometryA, PosA, AngleA);
        GjkPoints[NumGjkPoints] = (GjkGeoAPoints[NumGjkPoints] - SupportB(-Direction, GeometryB, PosB, AngleB));
        NumGjkPoints += 1;

        // NOTE: We didn't make it past the origin so no intersection
        // NOTE: Needed to prevent cases where we have two bodies that just touch
        if (Dot(Direction, GjkPoints[0]) <= 0)
        {
            return Result;
        }
        
        Direction = -GjkPoints[0];
        
        while (!Result.Intersects)
        {
            v2 NewPointA = SupportA(Direction, GeometryA, PosA, AngleA);
            v2 NewPoint = (NewPointA - SupportB(-Direction, GeometryB, PosB, AngleB));

            if (Dot(Direction, NewPoint) <= 0)
            {
                // NOTE: We didn't make it past the origin, so no intersection
                break;
            }

            GjkGeoAPoints[NumGjkPoints] = NewPointA;
            GjkPoints[NumGjkPoints] = NewPoint;
            NumGjkPoints += 1;

#if 0
            // NOTE: Debug code to make sure we don't mess up the order of our geo points
            for (u32 PointId = 0; PointId < NumGjkPoints; ++PointId)
            {
                v2 Val1 = GjkPoints[PointId];
                v2 Val2 = GjkGeoAPoints[PointId] - GjkGeoBPoints[PointId];
                Assert(Val1 == Val2);
            }
#endif
            
            // NOTE: Notation wise, we say A is the newest point, B is second, C is third, etc
            switch (NumGjkPoints)
            {
                case 2:
                {
                    v2 A = GjkPoints[1];
                    v2 B = GjkPoints[0];

                    v2 AO = V2(0) - A;
                    v2 AB = B - A;

                    if (Dot(AO, AB) > 0)
                    {
                        Direction = Gjk2dGetPerp(AB, AO);
                    }
                    else
                    {
                        NumGjkPoints = 1;
                        GjkGeoAPoints[0] = GjkGeoAPoints[1];
                        GjkPoints[0] = A;
                        Direction = AO;
                    }
                } break;

                case 3:
                {
                    /* NOTE: We have 4 regions:
                       1) Above triangle edge AC
                       2) Inside triangle
                       3) Outside triangle vert A
                       4) Below triangle edge AB
                    */
                
                    v2 A = GjkPoints[2];
                    v2 B = GjkPoints[1];
                    v2 C = GjkPoints[0];

                    v2 AO = V2(0) - A;
                    v2 AC = C - A;
                    v2 ACPerp = GetPerp(-AC);
                    v2 AB = B - A;
                    v2 ABPerp = GetPerp(AB);

                    if (Dot(AO, ACPerp) > 0)
                    {
                        if (Dot(AO, AC) > 0)
                        {
                            // NOTE: We are in region 1
                            NumGjkPoints = 2;
                            GjkPoints[1] = A;
                            GjkPoints[0] = C;
                            GjkGeoAPoints[1] = GjkGeoAPoints[2];
                            GjkGeoAPoints[0] = GjkGeoAPoints[0];
                            Direction = Gjk2dGetPerp(AC, AO);
                        }
                        else
                        {
                            if (Dot(AO, AB) > 0)
                            {
                                // NOTE: We are in region 4
                                NumGjkPoints = 2;
                                GjkPoints[1] = A;
                                GjkPoints[0] = B;
                                GjkGeoAPoints[0] = GjkGeoAPoints[1];
                                GjkGeoAPoints[1] = GjkGeoAPoints[2];
                                Direction = Gjk2dGetPerp(AB, AO);
                            }
                            else
                            {
                                // NOTE: We are in region 3
                                NumGjkPoints = 1;
                                GjkPoints[0] = A;
                                GjkGeoAPoints[0] = GjkGeoAPoints[2];
                                Direction = AO;
                            }
                        }
                    }
                    else
                    {
                        if (Dot(AO, ABPerp) > 0)
                        {
                            if (Dot(AO, AB) > 0)
                            {
                                // NOTE: We are in region 4
                                NumGjkPoints = 2;
                                GjkPoints[1] = A;
                                GjkPoints[0] = B;
                                GjkGeoAPoints[0] = GjkGeoAPoints[1];
                                GjkGeoAPoints[1] = GjkGeoAPoints[2];
                                Direction = Gjk2dGetPerp(AB, AO);
                            }
                            else
                            {
                                // NOTE: We are in region 3
                                NumGjkPoints = 1;
                                GjkPoints[0] = A;
                                GjkGeoAPoints[0] = GjkGeoAPoints[2];
                                Direction = AO;
                            }
                        }
                        else
                        {
                            // NOTE: We are in region 2 so we can exit
                            Result.Intersects = true;
                        }
                    }
                } break;

                default:
                {
                    InvalidCodePath;
                } break;
            }
        }
    }

    // NOTE: https://dyn4j.org/2010/05/epa-expanding-polytope-algorithm/
    // NOTE: http://www.dtecta.com/papers/gdc2001depth.pdf
    // NOTE: http://allenchou.net/2013/12/game-physics-contact-generation-epa/
    // NOTE: Begin EPA algorithm, expand our simplex until we found penetration depth
    if (Result.Intersects)
    {
        Assert(NumGjkPoints == 3);
        u32 EpaNumPoints = 4;
        
        v2 EpaGeoAPoints[EPA_MAX_ITERATIONS] = {};
        EpaGeoAPoints[0] = GjkGeoAPoints[0];
        EpaGeoAPoints[1] = GjkGeoAPoints[1];
        EpaGeoAPoints[2] = GjkGeoAPoints[2];
        EpaGeoAPoints[3] = GjkGeoAPoints[0];
        
        v2 EpaPoints[EPA_MAX_ITERATIONS] = {};
        EpaPoints[0] = GjkPoints[0];
        EpaPoints[1] = GjkPoints[1];
        EpaPoints[2] = GjkPoints[2];
        EpaPoints[3] = GjkPoints[0];

#if GJK_VISUALIZE
        // NOTE: Visualize our triangle
        for (u32 GjkPointId = 0; GjkPointId < EpaNumPoints - 1; ++GjkPointId)
        {
            DebugPushLine(EpaPoints[GjkPointId + 0], EpaPoints[GjkPointId + 1], V4(1, 0, 0, 1));
        }
#endif
    
        u32 IterationId = 0;
        v2 PenetrationDist = V2(0);
        while (true)
        {
#if 0
            // NOTE: Debug code to make sure we don't mess up the order of our geo poitns
            for (u32 PointId = 0; PointId < EpaNumPoints; ++PointId)
            {
                v2 Val1 = EpaPoints[PointId];
                v2 Val2 = EpaGeoAPoints[PointId] - EpaGeoBPoints[PointId];
                Assert(Val1 == Val2);
            }
#endif
            
            // NOTE: Find closest edge
            u32 ClosestEdgeId = 0xFFFFFFFF;
            f32 DistanceToOrigin = F32_MAX;
            f32 ClosestEdgeT = 0.0f;
            v2 DirectionToEdge = V2(0);
            for (u32 EdgeId = 0; EdgeId < EpaNumPoints - 1; ++EdgeId)
            {
                v2 EdgeStart = EpaPoints[EdgeId + 0];
                v2 EdgeEnd = EpaPoints[EdgeId + 1];

                v2 EdgeVec = EdgeEnd - EdgeStart;
                v2 EdgeOrigin = V2(0) - EdgeStart;

                f32 LineLength = LengthSquared(EdgeVec);
                f32 T = Clamp(Dot(EdgeOrigin, EdgeVec) / LineLength, 0.0f, 1.0f);
                v2 NearestPoint = Lerp(EdgeStart, EdgeEnd, T);

                f32 NewDistanceToOrigin = LengthSquared(NearestPoint);
                if (NewDistanceToOrigin < DistanceToOrigin)
                {
                    DistanceToOrigin = NewDistanceToOrigin;
                    ClosestEdgeId = EdgeId;
                    ClosestEdgeT = T;
                    DirectionToEdge = NearestPoint;
                }
            }

            // NOTE: We were using squared distance before
            DistanceToOrigin = SquareRoot(DistanceToOrigin);

            v2 EdgeNormal = Normalize(Gjk2dGetPerp(EpaPoints[ClosestEdgeId + 0] - EpaPoints[ClosestEdgeId + 1], DirectionToEdge));
            v2 NewSupportPointA = SupportA(EdgeNormal, GeometryA, PosA, AngleA);
            v2 NewSupportPoint = (NewSupportPointA - SupportB(-EdgeNormal, GeometryB, PosB, AngleB));
            f32 SupportPointDistance = Dot(NewSupportPoint, EdgeNormal);

            // TODO: Why does this fail?
            //Assert(SupportPointDistance - DistanceToOrigin >= 0.0f);

#if EPA_VISUALIZE
            if (IterationId == EPA_ITERATION_ID)
            {
                // NOTE: Visualize the simplex
                for (u32 EpaPointId = 0; EpaPointId < EpaNumPoints - 1; ++EpaPointId)
                {
                    DebugPushLine(EpaPoints[EpaPointId + 0], EpaPoints[EpaPointId + 1], V4(0, 0, 1, 1));
                }
            }
#endif
            
            if (SupportPointDistance - DistanceToOrigin < 0.000000001f || EpaNumPoints >= ArrayCount(EpaPoints))
            {
                // NOTE: http://allenchou.net/2013/12/game-physics-contact-generation-epa/
                // NOTE: Our collision points lie on the nearest edge so we interpole the value of that point based on the
                // contribution from each support function to find where on the edge of our shapes they are

#if EPA_VISUALIZE && EPA_VISUALIZE_FINAL
                // NOTE: Visualize the simplex
                for (u32 EpaPointId = 0; EpaPointId < EpaNumPoints - 1; ++EpaPointId)
                {
                    DebugPushLine(EpaPoints[EpaPointId + 0], EpaPoints[EpaPointId + 1], V4(0, 0, 1, 1));
                }

                DebugPushPoint(EpaPoints[ClosestEdgeId + 0], V4(0.5f, 0.5f, 0.5f, 1.0f));
                DebugPushPoint(EpaPoints[ClosestEdgeId + 1], V4(0.5f, 0.5f, 0.5f, 1.0f));

                DebugPushPoint(EpaGeoAPoints[ClosestEdgeId + 0], V4(0.5f, 0.5f, 0.5f, 1.0f));
                DebugPushPoint(EpaGeoAPoints[ClosestEdgeId + 1], V4(0.5f, 0.5f, 0.5f, 1.0f));

                DebugPushPoint(-(EpaPoints[ClosestEdgeId + 0] - EpaGeoAPoints[ClosestEdgeId + 0]), V4(0.5f, 0.5f, 0.5f, 1.0f));
                DebugPushPoint(-(EpaPoints[ClosestEdgeId + 1] - EpaGeoAPoints[ClosestEdgeId + 1]), V4(0.5f, 0.5f, 0.5f, 1.0f));

                for (u32 EpaPointId = 0; EpaPointId < EpaNumPoints - 1; ++EpaPointId)
                {
                    //DebugPushPoint(EpaGeoAPoints[EpaPointId], V4(0, 0, 1, 1));
                    //DebugPushPoint(EpaGeoBPoints[EpaPointId], V4(0, 0, 1, 1));
                }
                
                DebugPushLine(V2(0), DirectionToEdge, V4(1.0f, 0.5f, 0.2f, 1.0f));
#endif
                
                Result.WorldPoint1 = Lerp(EpaGeoAPoints[ClosestEdgeId + 0], EpaGeoAPoints[ClosestEdgeId + 1], ClosestEdgeT);
                Result.WorldPoint2 = Lerp(-(EpaPoints[ClosestEdgeId + 0] - EpaGeoAPoints[ClosestEdgeId + 0]),
                                          -(EpaPoints[ClosestEdgeId + 1] - EpaGeoAPoints[ClosestEdgeId + 1]), ClosestEdgeT);
                Result.Normal = EdgeNormal;
                Result.Distance = SupportPointDistance;
                break;
            }
            else
            {
                // NOTE: Insert our new support point
                for (u32 PointId = EpaNumPoints - 1; PointId > ClosestEdgeId; --PointId)
                {
                    EpaGeoAPoints[PointId + 1] = EpaGeoAPoints[PointId + 0];
                    EpaPoints[PointId + 1] = EpaPoints[PointId + 0];
                }

                EpaGeoAPoints[ClosestEdgeId + 1] = NewSupportPointA;
                EpaPoints[ClosestEdgeId + 1] = NewSupportPoint;
                EpaNumPoints += 1;
            }

#if EPA_VISUALIZE
            if (IterationId == EPA_ITERATION_ID)
            {
                // NOTE: Draw our found points
                // TODO: Add sizeable point rendering
                DebugPushLine(V2(0), DirectionToEdge, V4(0.0f, 0.5f, 1.0f, 0.1f));
                DebugPushPoint(NewSupportPoint, V4(0, 1, 0, 1));

                // NOTE: Visualize the simplex
                for (u32 EpaPointId = 0; EpaPointId < EpaNumPoints - 1; ++EpaPointId)
                {
                    DebugPushLine(EpaPoints[EpaPointId + 0], EpaPoints[EpaPointId + 1], V4(0, 0, 1, 1));
                }
                
                for (u32 EpaPointId = 0; EpaPointId < EpaNumPoints - 1; ++EpaPointId)
                {
                    DebugPushLine(EpaGeoAPoints[EpaPointId + 0], EpaGeoAPoints[EpaPointId + 1], V4(0, 0, 1, 1));
                    DebugPushLine(-(EpaPoints[EpaPointId + 0] - EpaGeoAPoints[EpaPointId + 0]),
                                   -(EpaPoints[EpaPointId + 1] - EpaGeoAPoints[EpaPointId + 1]), V4(0, 0, 1, 1));
                }
            }
#endif

            IterationId += 1;
        }
    }
    
    return Result;
}
