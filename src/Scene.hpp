#ifndef SCENE_HPP
#define SCENE_HPP

#include <vector>
#include <algorithm>
#include "Object.hpp"
#include "Camera.hpp"
#include "Light.hpp"
#include "Renderer.hpp"
#include "JetConfig.hpp"
#include "PostFX.hpp"
#include "Sprite2D.hpp"

namespace Renderer {

/// @brief PATCHED FOR arcade-os jet60: OBJECT-LEVEL BSP FOR PAINTER ORDER.
///
/// WHY A TREE AND NOT A BETTER KEY. For two disjoint convex objects, "A must
/// be drawn before B" is decided by which side of their SEPARATING PLANE the
/// camera is on. That is a pairwise relation, and any scalar sort key -
/// centroid depth, nearest vertex, farthest vertex, grid rank - projects it
/// onto one axis and loses it. Five keys were tried on this branch and each
/// fixed one pair by breaking another; that is a property of the problem and
/// not of the keys.
///
/// A BSP stores the separating planes instead of guessing at them. Split the
/// object set with a plane that no object straddles; then along ANY ray from
/// a camera on the plus side, every plus-side hit comes before every
/// minus-side hit, because the ray crosses the plane at most once. So "minus
/// subtree, then plus subtree" is exactly correct, with no comparisons and no
/// cycles. Recurse and the whole set is ordered.
///
/// NO TRIANGLES ARE SPLIT. A textbook BSP splits polygons that straddle a
/// plane; this one splits nothing and simply refuses a plane that any object
/// straddles. The price is that a clean plane does not always exist: a pillar
/// standing under an overpass deck cannot be separated from it by a vertical
/// plane, nor from the taller building beside it by a horizontal one. When no
/// clean plane exists the remaining objects become a LEAF and the caller
/// falls back to the depth key inside it.
///
/// That fallback is what makes this safe to land. A leaf reproduces exactly
/// today's behaviour over a handful of neighbouring objects, so the tree can
/// only improve the ordering and never regress it, and it leaves zBias doing
/// the job it was designed for - the coplanar stack of road, slab and kerb
/// inside one block - while the tree handles everything BETWEEN blocks, which
/// is where every artifact this branch chased actually was.
///
/// The tree is built once from world bounding boxes and is owned by the
/// caller, so one tree can be shared by several Scenes over the same object
/// list (jet60 runs two, one per core, over one city).
class ObjectSortTree {
public:
    /// @brief Build over `objs`, which must be the Scene's object list: the
    ///        indices order() returns index into it, so the list may be
    ///        appended to afterwards but never reordered.
    ///        Objects flagged `sortDynamic` are kept out of the tree and
    ///        located per frame by their centre instead.
    /// @param leafMax Stop splitting at this many objects. Smaller is a more
    ///        exact order and a bigger tree.
    /// @return false when there was nothing static to build from.
    /// @param slack How far an object may poke through a plane and still be
    ///        counted as lying on one side of it, in world units. A hard
    ///        zero is defeated by geometry that merely GRAZES: a kerb strip
    ///        overlapping the overpass pillars beside it by a tenth of a unit
    ///        blocked every plane along that street and glued the pillars,
    ///        the buildings and the trees into one 152-object leaf. Slack
    ///        turns the tree's guarantee into "exact except within `slack` of
    ///        a plane", so pick it smaller than a pixel at playing distance
    ///        and the error has nowhere to show.
    bool build(const std::vector<Object*>& objs, int leafMax = 6, int32_t slack = 0);

    /// @brief Offer a splitting plane for build() to try before it searches.
    ///
    /// WHICH clean plane a node uses is free - every one of them is correct -
    /// and build() picks the most balanced, which is the right default and the
    /// wrong answer for one case. The city's ground (road, slab, kerb, helipad)
    /// all lies under the kerb line with everything else standing on top of it,
    /// so ONE horizontal plane there sorts every flat surface behind every
    /// object standing on it, for every camera above the kerb - which is every
    /// camera. Balance never picks it, because 352 flat pieces against 130
    /// standing ones is not a balanced cut.
    ///
    /// Without it, a kerb 30 m away can land in a leaf the traversal visits
    /// after the leaf holding the car, and paints over the car. That is not a
    /// flaw in the order - both leaves are correctly placed relative to each
    /// other - it is that a car overlapping the ground plane belongs on the
    /// near side of it, and only a plane there can say so.
    ///
    /// Tried in the order added, at every node, before the balance search.
    /// Call before build(); planes that are not clean at a node are skipped.
    /// @param axis 0=x, 1=y, 2=z.
    /// @param value Plane coordinate in world units.
    void preferPlane(uint8_t axis, int32_t value);

    /// @brief Forget every plane added with preferPlane().
    void clearPreferredPlanes();

    /// @brief True when no tree has been built (the caller then keeps the
    ///        plain depth-key sort).
    bool empty() const { return nodes.empty(); }

    /// @brief Far-to-near visit order for a camera at (camX, camY, camZ).
    /// @param outOrder Filled with object indices, farthest leaf group first.
    /// @param outGroups Filled with the offsets in `outOrder` at which each
    ///        leaf group starts, plus a terminator equal to outOrder.size(),
    ///        so group g spans [outGroups[g], outGroups[g+1]).
    /// @brief Called with a subtree's world bounding box. Return true when the
    ///        box is entirely out of view, and order() will skip the whole
    ///        subtree - no object in it is even looked at.
    ///
    /// MUST BE CONSERVATIVE: returning true for anything that might be visible
    /// deletes it from the frame. Returning false is always safe and just
    /// costs the per-object cull that would have run anyway.
    typedef bool (*BoxRejectFn)(void* ctx, const int32_t mn[3], const int32_t mx[3]);

    /// @param reject Optional whole-subtree cull; nullptr visits everything.
    ///        This is the tree's second job and it is worth as much as the
    ///        first: a per-object frustum cull has to read every object's
    ///        header, and on this console those live in PSRAM, so 488 objects
    ///        cost ~5.5 ms of pure memory latency before a single triangle is
    ///        transformed. A leaf that is behind the camera can be dismissed
    ///        by one sphere test against a box already in internal SRAM.
    void order(int32_t camX, int32_t camY, int32_t camZ,
               std::vector<int32_t>& outOrder,
               std::vector<int32_t>& outGroups,
               BoxRejectFn reject = nullptr, void* rejectCtx = nullptr);

    /// @brief How many objects the tree accounts for: every static item plus
    ///        every mover. order() can emit FEWER than this once culling is on,
    ///        so callers must not infer "objects added after build" from the
    ///        size of its output.
    int coveredCount() const { return (int)items.size() + (int)dynamics.size(); }

    /// @name Build statistics, for reporting what the geometry allowed.
    /// @{
    int    nodeCount()     const { return (int)nodes.size(); }
    int    leafCount()     const { return leaves_; }
    int    worstLeaf()     const { return worstLeaf_; }
    int    unsplitLeaves() const { return unsplit_; }
    int    staticCount()   const { return (int)items.size(); }
    int    dynamicCount()  const { return (int)dynamics.size(); }
    size_t byteSize()      const;
    /// @}

private:
    struct Node {
        int32_t value;   ///< Plane coordinate on `axis`, in world units.
        int32_t a, b;    ///< Internal: minus/plus child. Leaf: first/count into `items`.
        int32_t mn[3];   ///< World box of everything in this subtree, for the cull.
        int32_t mx[3];
        uint8_t axis;    ///< 0=x, 1=y, 2=z; 3 marks a leaf.
    };
    struct Box { int32_t idx; int32_t mn[3], mx[3]; };

    std::vector<Node>    nodes;
    std::vector<int32_t> items;      ///< Object indices, leaves owning contiguous runs.
    std::vector<int32_t> dynamics;   ///< Object indices located per frame.
    std::vector<int32_t> dynNode;    ///< Scratch: leaf node index per dynamic.
    std::vector<uint8_t> dynDone;    ///< Scratch: did this dynamic get emitted?
    struct Plane { int32_t value; uint8_t axis; };
    std::vector<Plane> preferred;    ///< Tried before the balance search; see preferPlane().
    const std::vector<Object*>* src = nullptr;

    int32_t buildRec(std::vector<Box>& v, int lo, int hi, int leafMax, int32_t slack, int depth);
    int32_t leafFor(int32_t x, int32_t y, int32_t z) const;

    int leaves_ = 0, worstLeaf_ = 0, unsplit_ = 0;
};

/// @brief Top-level container that owns the scene graph and drives rendering.
///
/// Holds the active camera, lights, object list and per-frame state, and
/// exposes a single `render()` entry point that runs the full
/// transform/cull/raster/post-FX pipeline.
class Scene {
public:
    /// @brief Construct a scene bound to caller-owned framebuffers.
    /// @param framebuffer RGB565 colour buffer of size screenWidth*screenHeight.
    /// @param zBuffer Depth buffer of size ZBUFFER_STRIDE(screenWidth)*screenHeight; pass nullptr when Z_BUFFERING is disabled.
    /// @param screenWidth Output width in pixels.
    /// @param screenHeight Output height in pixels.
    Scene(uint16_t* framebuffer, uint16_t* zBuffer, int screenWidth, int screenHeight);
    ~Scene();

    int   frameCounter = 0;       ///< Incremented once per render(); useful for animations and dither parity.
    float waterTime    = 0.0f;    ///< Accumulated wall-clock seconds; set each frame by the caller before render/prepareFrame.

    /// @name Per-frame counters populated by render()
    /// @{
    /// `lastFrameDrawnObjects` is the number of enabled objects that
    /// survived the AABB frustum cull. `lastFrameDrawnTriangles` is the
    /// number of triangles submitted to the rasteriser (renderQueue size
    /// after all culling). `lastFrameRasterizedTriangles` is the subset of
    /// those that produced rasterizer work (drawTriangle returned true).
    int lastFrameDrawnObjects        = 0;
    int lastFrameDrawnTriangles      = 0;
    int lastFrameRasterizedTriangles = 0;
    /// PATCHED FOR arcade-os jet60: vertices pushed through the transform,
    /// summed over every object admitted this frame. This is what prepareFrame
    /// costs - the whole of an object's mesh is transformed whether one of its
    /// triangles survives or all of them do.
    int lastFrameTransformedVertices = 0;

    /// PATCHED IN THE INSTALLED COPY (ion-drift tools/patch_jet_profile.py).
    ///
    /// Where prepareFrame()'s time goes, in microseconds, split by the three
    /// phases it runs in sequence: the buffer clear, the transform/cull loop
    /// that fills renderQueue, and the bucket sort that fills renderOrder.
    /// Written only when the library is built with -D JET_PROFILE_PREP=1; they
    /// stay 0 otherwise, so a host can read them without a matching #if.
    uint32_t lastFramePrepClearUs     = 0;
    uint32_t lastFramePrepTransformUs = 0;
    uint32_t lastFramePrepSortUs      = 0;
    /// PATCHED FOR arcade-os jet60: of lastFramePrepTransformUs, how much was
    /// spent INSIDE renderObject(). The difference is the frustum cull, the
    /// LOD pick and the fade ramps - everything the loop does to decide an
    /// object is worth drawing. Half of the transform phase is per-object
    /// rather than per-vertex and this says which half.
    uint32_t lastFramePrepObjectUs    = 0;
    /// @}

    /// @brief Add an object to the scene.
    /// @param obj Object to add. Pointer is borrowed; caller retains ownership.
    void addObject(Object* obj);

    /// @brief Add a point light to the scene.
    /// @param light Light to add. Pointer is borrowed; caller retains ownership.
    void addPointLight(PointLight* light);

    /// @brief Register a 2D screen-space overlay to be drawn after every render().
    /// @param sprite Sprite to add. Pointer is borrowed; caller retains ownership.
    void addSprite(Sprite2D* sprite);

    /// @brief Set the active camera.
    /// @param cam Camera pointer (borrowed).
    void setCamera(Camera* cam);
    /// @brief Get the active camera.
    /// @return Pointer to the current camera, or nullptr if none is set.
    Camera* getCamera() { return camera; }

    /// @brief Set the active directional light.
    /// @param light Directional light (borrowed). May be nullptr.
    void setDirectionalLight(DirectionalLight* light);
    /// @brief Get the active directional light.
    DirectionalLight* getDirectionalLight() { return directionalLight; }

    /// @brief Set the active ambient light.
    /// @param light Ambient light (borrowed). May be nullptr.
    void setAmbientLight(AmbientLight* light);
    /// @brief Get the active ambient light.
    AmbientLight* getAmbientLight() { return ambientLight; }

    /// @brief Run the full pipeline for one frame: cull, transform, rasterise, post-FX.
    void render();

    /// @brief Phase 1 of split rendering: clear [yBandMin, yBandMax) on the rasteriser,
    ///        transform and depth-sort all objects. Does NOT rasterise triangles.
    ///
    ///        Call this once per frame before any rasterizeBand() calls.
    ///        The rasteriser's yBandMin/yBandMax gate which rows clearBuffers() clears
    ///        so the framebuffer pointer can be a virtual base (adjusted for band offset).
    void prepareFrame();

    /// @brief Phase 2 of split rendering: rasterise the sorted render queue for
    ///        rows [yMin, yMax) only. Uses a thread-local copy of the rasteriser so
    ///        concurrent calls with non-overlapping y ranges are safe when Z_BUFFERING==0.
    ///
    ///        May be called from multiple threads simultaneously with disjoint bands.
    /// @param zBandBase PATCHED FOR arcade-os jet60. Optional per-call depth
    ///        buffer, given as a VIRTUAL BASE (band - yMin*stride) so absolute
    ///        y indexes into a band-sized allocation. Two cores rasterising
    ///        different bands of one Scene need one each; nullptr keeps the
    ///        Scene's own.
    void rasterizeBand(int yMin, int yMax, uint16_t* zBandBase = nullptr);

    /// @brief Clear only the rows [yMin, yMax) of the current framebuffer without
    ///        re-running the transform or sort pipeline. Use this for bands 1+ when the
    ///        render queue from the preceding prepareFrame() call is still valid.
    ///
    ///        Sets the rasteriser's yBandMin/yBandMax before clearing so the
    ///        band-aware clear writes only into the correct region.
    void clearBand(int yMin, int yMax);

    /// @brief Advance the internal frame counter by one. Normally called automatically
    ///        by render(); use this when driving the pipeline via prepareFrame()/rasterizeBand().
    void advanceFrameCounter() { frameCounter++; }

    /// @brief Get total scene statistics (independent of camera position).
    /// @param objectCount Out: number of enabled objects.
    /// @param triangleCount Out: total triangle count across enabled objects.
    /// @param vertexCount Out: total vertex count across enabled objects.
    void getStatistics(int& objectCount, int& triangleCount, int& vertexCount);

    /// @brief Set the colour used to clear the framebuffer.
    /// @param color RGB565 clear colour.
    void setBackcolor(uint16_t color) { backcolor = color; }

    /// @brief Replace the colour buffer pointer (without changing dimensions).
    /// @param framebuffer New caller-owned RGB565 buffer.
    void setFramebuffer(uint16_t *framebuffer);

    /// @brief Enable or disable per-frame framebuffer clearing.
    /// @param clear True to clear before rendering, false to preserve previous content.
    void setClearBuffer(bool clear) { clearRenderBuffer = clear; }

    /// @brief Hot-swap framebuffer, z-buffer and dimensions (e.g. on window resize).
    ///
    /// The caller owns both buffers and is responsible for freeing the old
    /// ones AFTER this call returns. PostFX is recreated internally to
    /// pick up the new dimensions.
    /// @param newFramebuffer New caller-owned colour buffer.
    /// @param newZBuffer New caller-owned depth buffer.
    /// @param newWidth New width in pixels.
    /// @param newHeight New height in pixels.
    void resize(uint16_t* newFramebuffer, uint16_t* newZBuffer,
                int newWidth, int newHeight);

    /// @brief Get the underlying rasteriser.
    Rasterizer* getRenderer() { return renderer; }
    /// @brief Get the mutable list of scene objects.
    std::vector<Object*>& getObjects() { return objects; }
    /// @brief Get the mutable list of point lights.
    std::vector<PointLight*>& getPointLights() { return pointLights; }
    /// @brief Get the mutable list of materials owned by the scene.
    std::vector<Material*>& getMaterials() { return materials; }
    /// @brief Get the list of registered 2D sprites.
    /// On HALF_WIDTH_BUFFERS builds the display layer composites sprites
    /// during scanout at full resolution; expose the list so it can do so.
    std::vector<Sprite2D*>& getSprites() { return sprites; }

#if MAX_PICK_QUERIES > 0
    /// @brief Submit screen-space pick points to be tested during the next render().
    ///
    /// Excess queries beyond MAX_PICK_QUERIES are silently dropped. Pass
    /// count == 0 (or just don't call this) to disable picking. The renderer
    /// reads queries during render() and writes the matching slots in the
    /// pick result array. Both arrays are owned by Scene; the host should
    /// copy queries in by value and read results back after render().
    /// @param queries Caller-owned array of pick queries.
    /// @param count Number of valid entries in @p queries.
    void setPickQueries(const PickQuery* queries, int count);

    /// @brief Get the pick results from the most recent render() call.
    /// @return Pointer to an internal array of MAX_PICK_QUERIES results.
    const PickResult* getPickResults() const { return pickResults; }

    /// @brief Get the number of active pick queries set for the next render().
    int getPickQueryCount() const { return pickQueryCount; }
#endif


    uint16_t* backgroundGradientColors = nullptr;   ///< Optional per-row background gradient (screenHeight entries) used during clear.

    /// @name Distance-based level of detail (LOD)
    /// @brief Global LOD selection driven by camera-to-object distance.
    ///
    /// `lodScale` is the world-units-per-LOD-step. Setting it to 0 (the
    /// default) disables global LOD and every Object renders its own
    /// mesh as before. With `lodScale = 4096`, an Object with two LOD
    /// meshes attached renders LOD 0 below 4096 units, LOD 1 from 4096
    /// to 8191, LOD 2 from 8192 onward (or fades out / persists past
    /// the last LOD depending on the Object's `lodPersist` flag).
    ///
    /// `lodBias` is added to the computed level globally — a bias of -1
    /// pushes everything one LOD step higher in detail, +1 cheaper. This
    /// composes with the per-Object `lodBias` and is intended for runtime
    /// quality knobs (e.g. perf-driven dynamic adjustment).
    /// @{
    /// PATCHED FOR arcade-os jet60: borrowed OBJECT-LEVEL BSP. Null keeps the
    /// plain depth-key painter sort; set it and prepareFrame() walks the
    /// objects in tree order and depth-sorts only inside a leaf. Owned by the
    /// caller and safe to share between Scenes. See ObjectSortTree above.
    ///
    /// This replaces sortCellSize, a GRID painter order that was built,
    /// measured and reverted: a Manhattan cell rank measures distance from
    /// the camera's cell RADIALLY while occlusion depends on depth along the
    /// VIEW DIRECTION, so two cells of equal rank sit at very different
    /// depths whenever the camera looks diagonally.
    ObjectSortTree* sortTree = nullptr;

    int32_t lodScale = 0;   ///< World units per LOD step; 0 disables global LOD.
    int8_t  lodBias  = 0;   ///< Scene-wide LOD level offset (added to each object's choice).
    /// @}

private:
    struct RenderTri {
        RenderVertex v1, v2, v3;
        Material* material;
        int32_t avgZ;
        // PATCHED FOR arcade-os jet60: the PAINTER SORT KEY, separate from
        // avgZ so the rasteriser keeps the true triangle depth as its FAST_Z
        // hint while the sort can use something better.
        //
        // avgZ is the triangle centroid, and a centroid cannot order a big
        // triangle against a small one: a box facade is two triangles spanning
        // the whole wall, so its centroid sits metres deep and sorts BEHIND a
        // cylinder facet it actually occludes. That is the static poke-through
        // between adjacent buildings - it is not bucket width, and no bucket
        // count fixes it.
        //
        // For a CONVEX object under backface culling, its visible faces cannot
        // overlap each other on screen - which is the same argument that lets
        // sortOwnTriangles be false - so every triangle in it may share ONE key
        // and the order within the object is free. Giving them the OBJECT centroid
        // orders whole objects against each other, which for convex solids that do
        // not interpenetrate is exactly right. Non-convex meshes keep the
        // per-triangle centroid.
        int32_t sortZ;
        bool ignoreZBuffer;
        bool noWriteZBuffer;
        int8_t zBias;
        // Per-object alpha multiplier (255 = no per-object fade); folded
        // into the per-pixel screen-door alpha at raster time.
        uint8_t objAlpha;
        // When true, v1/v2/v3.lambertBrightness has been precomputed in
        // object-local space by renderObject (see "objectLocalLight" path
        // in Scene.cpp). drawTriangle skips its own jetShadeBrightness
        // calls in that case and reads the cached values directly. Only
        // ever set for objects whose materials are all non-specular.
        bool brightnessPrecomputed = false;
#if MAX_PICK_QUERIES > 0
        // Source object + ORIGINAL triangle index (in obj->triangles) for
        // pick attribution. Carried through the painter sort.
        Object* sourceObject;
        int32_t sourceTriangleIndex;
#endif
    };
    std::vector<RenderTri> renderQueue;
    // Painter's-sort output as indices into renderQueue, rebuilt by
    // prepareFrame() each frame. Sorting (scattering) 4-byte indices
    // instead of whole RenderTri structs avoids a full second copy of the
    // queue per frame; rasterizeBand() walks this to draw in depth order.
    std::vector<int32_t> renderOrder;

    // PATCHED FOR arcade-os jet60: sortTree scratch, kept across frames so a
    // frame costs no allocation. sortOrder is the object visit order,
    // sortGroups the leaf-group offsets into it, and groupQueueStart the
    // renderQueue offset each leaf group starts at - which is what lets the
    // depth fallback run per leaf instead of over the whole queue.
    std::vector<int32_t> sortOrder;
    std::vector<int32_t> sortGroups;
    std::vector<int32_t> groupQueueStart;

    // PATCHED FOR arcade-os jet60 (BANDED RASTERISATION).
    //
    // rasterizeBand() walks the WHOLE render queue for every band and lets
    // drawTriangle() reject what does not touch it. drawTriangle has to load
    // three RenderVertex plus the material - about 100 bytes - to work that
    // out, so an 8-band frame moves ~100 bytes x triangles x bands of memory
    // to throw nearly all of it away. Measured on an ESP32-S3 at 680 drawn
    // triangles: ~1.45 ms per extra band, i.e. 8 bands spend 19.32 ms
    // rasterising the same 240 rows that 5 bands do in 14.96.
    //
    // So keep each queued triangle's screen y span alongside the queue, packed
    // into one int32 (minY in the low half, maxY in the high half). The band
    // reject then touches 4 bytes instead of 100, and is exact - it is built
    // from the same three vertex y values drawTriangle would have read.
    //
    // Parallel to renderQueue: triYSpan[i] describes renderQueue[i].
    std::vector<int32_t> triYSpan;

    Camera* camera;
    DirectionalLight* directionalLight;
    AmbientLight* ambientLight;
    Rasterizer* renderer = nullptr;
    PostFX* postFX = nullptr;

    uint16_t* framebuffer;
    uint16_t* zBuffer;
    int screenWidth;
    int screenHeight;
    bool* scanlinesUpdated;

    std::vector<Object*> objects;
    std::vector<PointLight*> pointLights;
    std::vector<Material*> materials;
    std::vector<Sprite2D*> sprites;

    uint16_t backcolor = 0;
    bool clearRenderBuffer = true;
    bool renderEvenLines = false;

    // Frustum side-plane normal lengths for the quick sphere cull in
    // cullObject(): |(fovFactor, ±screenW/2)| and |(fovFactor, ±screenH/2)|.
    // Recomputed once per frame in prepareFrame() because fovFactor
    // changes at runtime (boost FOV kick).
    float cullPlaneLh = 1.0f;
    float cullPlaneLv = 1.0f;

    // PATCHED FOR arcade-os jet60: the camera rotation matrix, in float,
    // built ONCE PER FRAME by prepareFrame().
    //
    // renderObject() used to build this itself, and Jet's comment there said
    // why: "computed per object render rather than once per scene because the
    // call cost is negligible ... hoisting would shave nine muls per OBJECT,
    // not per vertex". That is a fair trade when the per-object cost is
    // negligible, and on this console it is not. Measured by holding one scene
    // and walking the LOD dial, which changes the vertex count while leaving
    // the object count fixed:
    //
    //     lod   vtx    xf        ->  xf = 38.8 us per OBJECT + 3.06 us per vertex
    //      -1  3416  19.23           at obj 227 that is 8.8 ms of per-object
    //       0  2970  17.89           cost against 9.1 ms of vertex work - half
    //      +1  2428  16.23           of the transform ignores mesh detail.
    //
    // Nine muls and eighteen divides per object is not the whole of that 38.8,
    // but it is pure waste: every object in a frame computes the same numbers.
    float fCamM00 = 1, fCamM01 = 0, fCamM02 = 0;
    float fCamM10 = 0, fCamM11 = 1, fCamM12 = 0;
    float fCamM20 = 0, fCamM21 = 0, fCamM22 = 1;
    void updateCameraMatrix(int32_t camCosX, int32_t camSinX,
                            int32_t camCosY, int32_t camSinY,
                            int32_t camCosZ, int32_t camSinZ);

    bool cullObject(Object* obj,
                    int32_t camCosX, int32_t camSinX,
                    int32_t camCosY, int32_t camSinY,
                    int32_t camCosZ, int32_t camSinZ) const;

    void renderObject(Object* obj,
                      int32_t camCosX, int32_t camSinX,
                      int32_t camCosY, int32_t camSinY,
                      int32_t camCosZ, int32_t camSinZ,
                      uint8_t objAlpha,
                      Object* meshSource = nullptr);
    void reconstructCheckerboard();
    void clearBuffers();

public:
    /// @brief Composite all enabled sprites onto the current framebuffer.
    ///        Normally called automatically by render(); call explicitly when
    ///        driving the pipeline via prepareFrame()/rasterizeBand().
    void drawSprites();

private:

#if MAX_PICK_QUERIES > 0
    PickQuery  pickQueries[MAX_PICK_QUERIES];
    PickResult pickResults[MAX_PICK_QUERIES];
    int        pickQueryCount = 0;
#endif
};

} // namespace Renderer

#endif // SCENE_HPP
