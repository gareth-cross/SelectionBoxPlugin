// Copyright 2021 Gareth Cross.
#include "SelectionBoxFunctionLibrary.h"

#include "Kismet/GameplayStatics.h"

FRegionPlanes FSelectionRegion::ComputePlanes() const
{
	FRegionPlanes Result;
	Result.LeftPlane = FPlane{
		CameraOrigin, FVector::CrossProduct(BottomLeftRay, TopLeftRay).GetSafeNormal()
	};
	Result.RightPlane = FPlane{
		CameraOrigin,
		FVector::CrossProduct(TopRightRay, BottomRightRay).GetSafeNormal()
	};
	Result.TopPlane = FPlane{
		CameraOrigin, FVector::CrossProduct(TopLeftRay, TopRightRay).GetSafeNormal()
	};
	Result.BottomPlane = FPlane{
		CameraOrigin,
		FVector::CrossProduct(BottomRightRay, BottomLeftRay).GetSafeNormal()
	};
	return Result;
}

bool USelectionBoxFunctionLibrary::InBoxXY(const FVector& Vec, const FBox& Box)
{
	return (Vec.X > Box.Min.X) && (Vec.Y > Box.Min.Y) && (Vec.X < Box.Max.X) && (Vec.Y < Box.Max.Y);
}

bool USelectionBoxFunctionLibrary::InBoxXZ(const FVector& Vec, const FBox& Box)
{
	return (Vec.X > Box.Min.X) && (Vec.Z > Box.Min.Z) && (Vec.X < Box.Max.X) && (Vec.Z < Box.Max.Z);
}

bool USelectionBoxFunctionLibrary::InBoxYZ(const FVector& Vec, const FBox& Box)
{
	return (Vec.Y > Box.Min.Y) && (Vec.Z > Box.Min.Z) && (Vec.Y < Box.Max.Y) && (Vec.Z < Box.Max.Z);
}

// Replacement for FMath::LineBoxIntersect, which does not appear to work.
bool USelectionBoxFunctionLibrary::RayIntersectsTransformedBox(const FVector& RayOrigin, const FVector& RayDirection,
                                                               const FTransform& BoxTransform, const FVector& Origin,
                                                               const FVector& Extent)
{
	// Convert origin and direction to box frame.
	const FVector OriginInBoxFrame = BoxTransform.InverseTransformPosition(RayOrigin);
	const FVector DirectionInBoxFrame = BoxTransform.InverseTransformVector(RayDirection.GetSafeNormal());

	// shorthand:
	const FVector& P = OriginInBoxFrame;
	const FVector& L = DirectionInBoxFrame;
	const FBox LocalBox{Origin - Extent, Origin + Extent};

	// check every face
	if (FMath::Abs(L.Z) > SMALL_NUMBER)
	{
		// +Z
		{
			// We intersect the top, check if it is in bounds:
			const float W = LocalBox.Max.Z;
			const float D = (-P.Z + W) / L.Z;
			const FVector IntersectPt = P + L * D;
			if (InBoxXY(IntersectPt, LocalBox))
			{
				return true;
			}
		}
		// -Z
		{
			const float W = -1 * LocalBox.Min.Z;
			const float D = -(P.Z + W) / L.Z;
			const FVector IntersectPt = P + L * D;
			if (InBoxXY(IntersectPt, LocalBox))
			{
				return true;
			}
		}
	}
	if (FMath::Abs(L.Y) > SMALL_NUMBER)
	{
		// +Y
		{
			const float W = LocalBox.Max.Y;
			const float D = (-P.Y + W) / L.Y;
			const FVector IntersectPt = P + L * D;
			if (InBoxXZ(IntersectPt, LocalBox))
			{
				return true;
			}
		}
		// -Y
		{
			const float W = -1 * LocalBox.Min.Y;
			const float D = -(P.Y + W) / L.Y;
			const FVector IntersectPt = P + L * D;
			if (InBoxXZ(IntersectPt, LocalBox))
			{
				return true;
			}
		}
	}
	if (FMath::Abs(L.X) > SMALL_NUMBER)
	{
		// +X
		{
			const float W = LocalBox.Max.X;
			const float D = (-P.X + W) / L.X;
			const FVector IntersectPt = P + L * D;
			if (InBoxYZ(IntersectPt, LocalBox))
			{
				return true;
			}
		}
		// -X
		{
			const float W = -1 * LocalBox.Min.X;
			const float D = -(P.X + W) / L.X;
			const FVector IntersectPt = P + L * D;
			if (InBoxYZ(IntersectPt, LocalBox))
			{
				return true;
			}
		}
	}
	return false;
}

// Determine which region a point falls in. This is the Cohen Sutherland Algorithm. The four planes
// we have created define a pyramid shape w/ flat sides. This breaks the space in front of the camera
// into 9 regions: the center, and 8 volumes around the pyramid.
static uint8 DetermineRegion(const FVector& Pt, const FRegionPlanes& Planes)
{
	if (Planes.TopPlane.PlaneDot(Pt) > 0)
	{
		if (Planes.LeftPlane.PlaneDot(Pt) > 0) //	top left
		{
			return 9; //	1001
		}
		if (Planes.RightPlane.PlaneDot(Pt) > 0) // top right
		{
			return 5; //	0101
		}
		// top middle
		return 1; //	0001
	}

	// Below bottom planes:
	if (Planes.BottomPlane.PlaneDot(Pt) > 0)
	{
		if (Planes.LeftPlane.PlaneDot(Pt) > 0) //	bottom left
		{
			return 10; //	1010
		}
		else if (Planes.RightPlane.PlaneDot(Pt) > 0) // bottom right
		{
			return 6; //	0110
		}
		// bottom middle
		return 2; //	0001
	}

	//	In between top and bottom
	if (Planes.LeftPlane.PlaneDot(Pt) > 0) //	middle left
	{
		return 8; //	1000
	}
	if (Planes.RightPlane.PlaneDot(Pt) > 0) // middle right
	{
		return 4; //	0100
	}
	return 0; //	middle
}

// Multipliers we apply to the box extent to get the corner points.
constexpr float PointMultipliers[8][3] = {
	{1, 1, 1},
	{1, 1, -1},
	{1, -1, -1},
	{1, -1, 1},
	{-1, 1, 1},
	{-1, 1, -1},
	{-1, -1, -1},
	{-1, -1, 1},
};

struct IntPair
{
	int i;
	int j;
};

// Defines the 12 unique edges of a box, given the offsets in PointMultipliers.
constexpr IntPair BoxEdges[12] = {
	{0, 1},
	{0, 3},
	{0, 4},
	{1, 2},
	{1, 5},
	{2, 3},
	{2, 6},
	{3, 7},
	{4, 5},
	{4, 7},
	{5, 6},
	{6, 7},
};

ETransformedBoxTestResult USelectionBoxFunctionLibrary::SelectionRegionOverlapsTransformedBox(
	const FSelectionRegion& Region, const FTransform& BoxTransform, const FVector& Origin, const FVector& Extent)
{
	// Convert box corner points to world coordinates:
	FVector WorldPts[8];
	for (int i = 0; i < 8; ++i)
	{
		const FVector Multiplier{PointMultipliers[i][0], PointMultipliers[i][1], PointMultipliers[i][2]};
		WorldPts[i] = BoxTransform.TransformPosition(Extent * Multiplier + Origin);
	}

	// Compute the planes that define our region.
	const FRegionPlanes Planes = Region.ComputePlanes();

	//	Assign regions to points
	uint8 Regions[8];
	for (int i = 0; i < 8; ++i)
	{
		Regions[i] = DetermineRegion(WorldPts[i], Planes);
		if (!Regions[i])
		{
			return ETransformedBoxTestResult::BoxCornerInsideRegion; //	early exit, one point is within the box
		}
	}

	// Check the edges for intersection
	for (const IntPair& Line : BoxEdges)
	{
		const uint8 RegionFirst = Regions[Line.i];
		const uint8 RegionSecond = Regions[Line.j];
		const bool bMightIntersect = (RegionFirst & RegionSecond) == 0;
		constexpr bool bUseEarlyExit = true;
		// If this test fails (cohen sutherland algorithm) then we can early exit this line, it can't possibly intersect
		if (!bMightIntersect && bUseEarlyExit)
		{
			// Cannot pass through, draw point...
			continue;
		}

		// Might still pass through, check it against the planes:
		FVector IntersectionPt;

		// Left plane
		const bool bIntersectsLeft = FMath::SegmentPlaneIntersection(WorldPts[Line.i],
		                                                             WorldPts[Line.j], Planes.LeftPlane,
		                                                             IntersectionPt);
		if (bIntersectsLeft)
		{
			// check that we are within the top/bottom plane
			const uint8 RegionBits = DetermineRegion(IntersectionPt, Planes);
			if (RegionBits == 0 || RegionBits == 4 || RegionBits == 8)
			{
				// Intersection point is in the middle (horizontal)
				return ETransformedBoxTestResult::BoxIntersectsPlane;
			}
		}

		// Right plane
		const bool bIntersectsRight = FMath::SegmentPlaneIntersection(WorldPts[Line.i],
		                                                              WorldPts[Line.j], Planes.RightPlane,
		                                                              IntersectionPt);
		if (bIntersectsRight)
		{
			// check that we are within the top/bottom plane
			const uint8 RegionBits = DetermineRegion(IntersectionPt, Planes);
			if (RegionBits == 0 || RegionBits == 4 || RegionBits == 8)
			{
				// intersection point is in the middle (horizontal)
				return ETransformedBoxTestResult::BoxIntersectsPlane;
			}
		}

		// Top plane
		const bool bIntersectsTop = FMath::SegmentPlaneIntersection(WorldPts[Line.i],
		                                                            WorldPts[Line.j], Planes.TopPlane,
		                                                            IntersectionPt);
		if (bIntersectsTop)
		{
			// check that we are within the left/right planes
			const uint8 RegionBits = DetermineRegion(IntersectionPt, Planes);
			if (RegionBits == 0 || RegionBits == 1 || RegionBits == 2)
			{
				// intersection point is in the middle (vertical)
				return ETransformedBoxTestResult::BoxIntersectsPlane;
			}
		}

		// Bottom plane:
		const bool bIntersectsBottom = FMath::SegmentPlaneIntersection(WorldPts[Line.i],
		                                                               WorldPts[Line.j], Planes.BottomPlane,
		                                                               IntersectionPt);
		if (bIntersectsBottom)
		{
			const uint8 RegionBits = DetermineRegion(IntersectionPt, Planes);
			if (RegionBits == 0 || RegionBits == 1 || RegionBits == 2)
			{
				// intersection point is in the middle (vertical)
				return ETransformedBoxTestResult::BoxIntersectsPlane;
			}
		}
	}

	// Finally check all the rays:
	if (RayIntersectsTransformedBox(Region.CameraOrigin, Region.TopLeftRay, BoxTransform, Origin, Extent) ||
		RayIntersectsTransformedBox(Region.CameraOrigin, Region.TopRightRay, BoxTransform, Origin, Extent) ||
		RayIntersectsTransformedBox(Region.CameraOrigin, Region.BottomLeftRay, BoxTransform, Origin, Extent) ||
		RayIntersectsTransformedBox(Region.CameraOrigin, Region.BottomRightRay, BoxTransform, Origin, Extent))
	{
		return ETransformedBoxTestResult::SelectionCornerIntersectsBox;
	}
	return ETransformedBoxTestResult::NoIntersection;
}

bool USelectionBoxFunctionLibrary::CreateSelectionRegionForBoxCorners(APlayerController* const Controller,
                                                                      const FVector2D& PixelCoordinates1,
                                                                      const FVector2D& PixelCoordinates2,
                                                                      FSelectionRegion& RegionOut)
{
	const FVector2D TopLeft{
		FMath::Min(PixelCoordinates1.X, PixelCoordinates2.X), FMath::Min(PixelCoordinates1.Y, PixelCoordinates2.Y)
	};
	const FVector2D TopRight{
		FMath::Max(PixelCoordinates1.X, PixelCoordinates2.X), FMath::Min(PixelCoordinates1.Y, PixelCoordinates2.Y)
	};
	const FVector2D BottomRight{
		FMath::Max(PixelCoordinates1.X, PixelCoordinates2.X), FMath::Max(PixelCoordinates1.Y, PixelCoordinates2.Y)
	};
	const FVector2D BottomLeft{
		FMath::Min(PixelCoordinates1.X, PixelCoordinates2.X), FMath::Max(PixelCoordinates1.Y, PixelCoordinates2.Y)
	};

	ULocalPlayer* const LocalPlayer = IsValid(Controller) ? Controller->GetLocalPlayer() : nullptr;
	if (!LocalPlayer || !LocalPlayer->ViewportClient)
	{
		return false;
	}

	// get the projection data
	FSceneViewProjectionData ProjectionData;
	if (!LocalPlayer->GetProjectionData(LocalPlayer->ViewportClient->Viewport, eSSP_FULL, /*out*/ ProjectionData))
	{
		return false;
	}
	const FMatrix InvViewProjMatrix = ProjectionData.ComputeViewProjectionMatrix().InverseFast();

	// De-project the rays to world unit vectors
	FSceneView::DeprojectScreenToWorld(TopLeft, ProjectionData.GetConstrainedViewRect(),
	                                   InvViewProjMatrix, RegionOut.CameraOrigin, RegionOut.TopLeftRay);
	FSceneView::DeprojectScreenToWorld(TopRight, ProjectionData.GetConstrainedViewRect(),
	                                   InvViewProjMatrix, RegionOut.CameraOrigin, RegionOut.TopRightRay);
	FSceneView::DeprojectScreenToWorld(BottomRight, ProjectionData.GetConstrainedViewRect(),
	                                   InvViewProjMatrix, RegionOut.CameraOrigin, RegionOut.BottomRightRay);
	FSceneView::DeprojectScreenToWorld(BottomLeft, ProjectionData.GetConstrainedViewRect(),
	                                   InvViewProjMatrix, RegionOut.CameraOrigin, RegionOut.BottomLeftRay);
	return true;
}

bool USelectionBoxFunctionLibrary::SelectionRegionOverlapsComponent(const FSelectionRegion& Region,
                                                                    USceneComponent* const Component)
{
	if (!IsValid(Component))
	{
		return false;
	}
	const FBoxSphereBounds LocalBounds = Component->CalcLocalBounds();
	const ETransformedBoxTestResult Result = SelectionRegionOverlapsTransformedBox(
		Region, Component->GetComponentTransform(), LocalBounds.Origin,
		LocalBounds.BoxExtent);
	return Result != ETransformedBoxTestResult::NoIntersection;
}

bool USelectionBoxFunctionLibrary::SelectionRegionOverlapsActor(const FSelectionRegion& Region, AActor* const Actor,
                                                                const bool bIncludeFromNonColliding,
                                                                const bool bIncludeChildActors)
{
	if (!IsValid(Actor))
	{
		return false;
	}
	const FBox Box = Actor->CalculateComponentsBoundingBoxInLocalSpace(bIncludeFromNonColliding, bIncludeChildActors);
	const ETransformedBoxTestResult Result = SelectionRegionOverlapsTransformedBox(
		Region, Actor->GetActorTransform(), Box.GetCenter(), Box.GetExtent());
	return Result != ETransformedBoxTestResult::NoIntersection;
}
