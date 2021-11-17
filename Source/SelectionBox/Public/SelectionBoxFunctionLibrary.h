// Copyright 2021 Gareth Cross.
#pragma once
#include "SelectionBoxFunctionLibrary.generated.h"

/**
 * Types of intersections that can result from a frustum-box test.
 */
UENUM(BlueprintType)
enum class ETransformedBoxTestResult : uint8 {
  // There was no intersection. The selection region does not overlap the bounding box.
  NoIntersection = 0 UMETA(DisplayName="No Intersection"),
  // One of the corners of our bounding box is inside the selection region.
  BoxCornerInsideRegion UMETA(DisplayName="Box Corner Inside Region"),
  // The selection corner intersects the box.
  SelectionCornerIntersectsBox UMETA(DisplayName="Selection Corner Intersects Box"),
  // Different types of intersects between box and region-bounding planes:
  BoxIntersectsPlane UMETA(DisplayName="Box Intersects Plane"),
};

/**
 * Planes computed from FSelectionRegion. These planes define the frustum (only in 4 dimensions, since we
 * omit the near/far planes) in which the selection must fall.
 */
USTRUCT(BlueprintType)
struct SELECTIONBOX_API FRegionPlanes {
  GENERATED_BODY()
public:
  // TODO(gareth): Pack these in FMatrix so we can test all w/ TransformPosition()
  FPlane LeftPlane;
  FPlane RightPlane;
  FPlane TopPlane;
  FPlane BottomPlane;
};

/**
 * Struct that defines a selection region in 3D. We specify the position of the camera, as well as the
 * rays directions (emanating from the camera origin) that define the 4 corners of the selection box.
 *
 * Together, these specify 4 planes which can be used to test whether bounding boxes fall inside the
 * selection region.
 */
USTRUCT(BlueprintType)
struct SELECTIONBOX_API FSelectionRegion {
  GENERATED_BODY()
public:
  // Camera position when selection box was defined.
  UPROPERTY(BlueprintReadWrite)
  FVector CameraOrigin{FVector::ZeroVector};

  // The unit-length ray (pointing into the world) at the top left corner of the selection box.
  UPROPERTY(BlueprintReadWrite)
  FVector TopLeftRay{FVector::ZeroVector};

  // The unit-length ray at the top right corner of the selection box.
  UPROPERTY(BlueprintReadWrite)
  FVector TopRightRay{FVector::ZeroVector};

  // The unit length ray at the bottom left corner of the selection box.
  UPROPERTY(BlueprintReadWrite)
  FVector BottomLeftRay{FVector::ZeroVector};

  // The unit length ray at the bottom right corner of the selection box.
  UPROPERTY(BlueprintReadWrite)
  FVector BottomRightRay{FVector::ZeroVector};

  // Compute the bounding planes from the rays.
  FRegionPlanes ComputePlanes() const;
};

/**
 * Functions for helping with drag-box style selection, like in RTS games.
 */
UCLASS(BlueprintType, meta=(DisplayName="Selection Box Function Library"))
class SELECTIONBOX_API USelectionBoxFunctionLibrary : public UBlueprintFunctionLibrary {
  GENERATED_BODY()
public:
  // Check if the point falls within the XY bounds of a box, ignoring Z.
  static bool InBoxXY(const FVector& Vec, const FBox& Box);

  // Check if the point falls within the XZ bounds of a box, ignoring Y.
  static bool InBoxXZ(const FVector& Vec, const FBox& Box);

  // Check if the point falls within the YZ bounds of a box, ignoring X.
  static bool InBoxYZ(const FVector& Vec, const FBox& Box);

  /**
   * Check if a ray intersects an arbitrarily oriented box. The RayOrigin and RayDirection must be
   * supplied in the same reference frame as the BoxTransform. So, for example, if they are specified
   * in World frame, the BoxTransform should be the rotation and translation of the box with respect
   * to the world.
   *
   * The `Origin` and `Extent` parameters are expressed in the local frame of the box.
   *
   * `RayDirection` will be normalized, but must have non-zero length.
   *
   * This function only checks for an intersection, and does not return the exact intersection point.
   */
  UFUNCTION(BlueprintCallable, BlueprintPure)
  static bool RayIntersectsTransformedBox(const FVector& RayOrigin,
                                          const FVector& RayDirection,
                                          const FTransform& BoxTransform,
                                          const FVector& Origin,
                                          const FVector& Extent);

  /**
   * Check if the provided region contains any part of the specified transformed box. The region is defined
   * by the camera origin, and 4 vectors that specify the bounds of a frustum-like shape. It is not a full frustum
   * because we omit the near and far planes.
   *
   * The `Origin` and `Extent` parameters are expressed in the local frame of the box.
   *
   * This function only checks for an intersection OR overlap. It does not find the specific intersection
   * point.
   */
  UFUNCTION(BlueprintCallable, BlueprintPure)
  static ETransformedBoxTestResult SelectionRegionOverlapsTransformedBox(const FSelectionRegion& Region,
                                                                         const FTransform& BoxTransform,
                                                                         const FVector& Origin,
                                                                         const FVector& Extent);

  // Version of the above that accepts the pre-computed planes as an argument.
  UFUNCTION(BlueprintCallable, BlueprintPure)
  static ETransformedBoxTestResult SelectionRegionOverlapsTransformedBox2(const FSelectionRegion& Region,
                                                                          const FRegionPlanes& Planes,
                                                                          const FTransform& BoxTransform,
                                                                          const FVector& Origin,
                                                                          const FVector& Extent);

  /**
   * Check if the provided region contains any part of the specified world-aligned sphere.
   */
  UFUNCTION(BlueprintCallable, BlueprintPure)
  static bool SelectionRegionOverlapsSphere(const FSelectionRegion& Region,
                                            const FVector& SphereOrigin,
                                            float Radius);

  // Version of the above that accepts the pre-computed planes as an argument.
  UFUNCTION(BlueprintCallable, BlueprintPure)
  static bool SelectionRegionOverlapsSphere2(const FRegionPlanes& Planes,
                                             const FVector& SphereOrigin,
                                             float Radius);

  /**
   * Create FSelectionRegion from a pair of pixel coordinates that define a selection box in screen space.
   *
   * De-projects the corners of the box into world-space unit vectors. The pixel coordinates just need to specify
   * two opposing corners of the bounding box, in any order.
   *
   * Should return true provided you pass a valid controller w/ a valid viewport.
   */
  UFUNCTION(BlueprintCallable, BlueprintPure)
  static bool CreateSelectionRegionForBoxCorners(APlayerController* Controller,
                                                 const FVector2D& PixelCoordinates1,
                                                 const FVector2D& PixelCoordinates2,
                                                 FSelectionRegion& RegionOut);

  /**
   * Check if the provided selection region overlaps the oriented bounding box of the given component.
   *
   * Computes the local-space bounding box of the component, and then checks if that box intersects or overlaps
   * the provided selection region. If it does, returns true.
   */
  UFUNCTION(BlueprintCallable, BlueprintPure)
  static bool SelectionRegionOverlapsComponent(const FSelectionRegion& Region, const USceneComponent* Component);

  /**
   * Check if the provided selection region overlaps the oriented bounding box of the given actor.
   *
   * Computes the local-space bounding box of the actor's components, and then checks if that box intersects
   * or overlaps the provided selection region. If it does, returns true.
   */
  UFUNCTION(BlueprintCallable, BlueprintPure)
  static bool SelectionRegionOverlapsActor(const FSelectionRegion& Region, const AActor* Actor,
                                           bool bIncludeFromNonColliding,
                                           bool bIncludeChildActors);

  // Pre-compute the planes for a given selection region.
  UFUNCTION(BlueprintCallable, BlueprintPure)
  static FRegionPlanes ComputePlanesForRegion(const FSelectionRegion& Region) {
    return Region.ComputePlanes();
  }
};
