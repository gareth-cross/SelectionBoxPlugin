
### SelectionBoxPlugin

---

Plugin for Unreal Engine for testing if actors in 3D space fall within a 2D selection region in the camera (RTS style selection box). Allows specification of a rectangular selection box in screen coordinates, against which you can subsequently test actors' 3D oriented-bounding boxes (OBB). Boxes that overlap or fall within the selection region are deemed as "intersecting".

This code emerged from a discussion in the Unreal Slackers Discord about how you might perform a frustum-OBB check in 3D. Since it is mostly for fun, I have not tested the speed of this algorithm against other methods yet. One would expect a screen-space version of this algorithm to be faster, at a minimum.

If you clone this repo into the `Plugins` folder of [this example project](https://github.com/gareth-cross/BoxSelection), you can try it out for yourself.

At a high-level, this method operates as follows:

1. Construct a pseudo-frustum from the screen-space selection box. It is not a full frustum since we ignore depth. There are only four bounding planes.
2. Eliminate obvious candidates using their world-space bounding spheres.
3. Test remaining candidates by handling the three possible "intersecting" cases:
    - One of the corner rays of the frustum intersects the OBB.
    - One of the corners of the OBB falls within the frustum.
    - One of the edges of OBB intersects the frustum. Cohen-Sutherland algorithm is used to speed this up a bit.

Example image: 

![Image of OBBs being selected.](example.png)
