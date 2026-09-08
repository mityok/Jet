#include "Scene.hpp"
#include "TrigLUT.hpp"
#include "Renderer.hpp"
#include "JetConfig.hpp"
#include <cstring> // For memset
#include <algorithm> // For std::min, std::max
#include <cmath> // For sqrtf (per-object distance fade / LOD pick)

// PATCHED IN THE INSTALLED COPY (ion-drift tools/patch_jet_profile.py).
//
// prepareFrame() phase timing. Compiles to nothing at all unless the build
// defines JET_PROFILE_PREP, so an un-instrumented build pays neither the calls
// nor the three stores.
//
// micros() on Arduino rather than <chrono>: it is what the ESP32 frame-budget
// harness measures whole frames with, and mixing two clocks in one report is
// how a rounding difference becomes an argument. Both are unsigned 32-bit
// microseconds, so a delta stays correct across the ~71 minute wrap.
#if JET_PROFILE_PREP
  #ifdef ARDUINO
    #include <Arduino.h>
    static inline uint32_t jetPrepNowUs() { return (uint32_t)micros(); }
  #else
    #include <chrono>
    static inline uint32_t jetPrepNowUs() {
        return (uint32_t)std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    }
  #endif
  #define JET_PREP_CLOCK(v)      uint32_t v = jetPrepNowUs()
  #define JET_PREP_SPLIT(f, v)   do { const uint32_t n_ = jetPrepNowUs(); \
                                      (f) = n_ - (v); (v) = n_; } while (0)
#else
  #define JET_PREP_CLOCK(v)      ((void)0)
  #define JET_PREP_SPLIT(f, v)   ((void)0)
#endif

namespace Renderer {

// Static emissive material used to render triangles whose lighting has been
// pre-baked into Triangle::bakedColor.  The colour field is overwritten
// per-triangle before emitTri() is called; all other fields are constant.
static Material s_bakedMat;
static bool     s_bakedMatInit = false;
static void initBakedMat() {
    if (s_bakedMatInit) return;
    s_bakedMat.diffuseMap  = nullptr;
    s_bakedMat.shader      = nullptr;
    s_bakedMat.emissive    = true;
    s_bakedMat.alpha       = 255;
    s_bakedMat.diffuse     = 0;
    s_bakedMat.specular    = 0;
    s_bakedMat.shadingMode = ShadingMode::FLAT;
    s_bakedMat.name        = nullptr;
    s_bakedMatInit = true;
}

#if LIGHTING
// Diffuse-only Lambert shading, evaluated in whatever frame N and L
// share (object-local for the precompute path below). Matches the
// diffuse branch of Renderer.cpp's jetShadeBrightness when
// specularCoef == 0: 12-bit reduce, clamp, square-falloff, scale by
// lightIntensity, scale by diffuseCoef, clamp.
//
// The specular branch is intentionally absent — this helper is only
// used for objects whose materials are ALL non-specular, which is the
// exact precondition for skipping the view-space normal transform.
// Inlining the specular branch here would mean its `N.z < 0` test
// would be reading an object-local component instead of view-space Z
// and give wrong results.
static inline uint16_t sceneLambertDiffuse(const Vector3& N, const Vector3& L,
                                           uint16_t lightIntensity,
                                           uint8_t diffuseCoef)
{
    if (lightIntensity > 255) lightIntensity = 255;
    int64_t lit = Vector3::dotProduct(N, L);
    if (lit <= 0) return 0;
    uint32_t lambert = (uint32_t)(lit >> 12);
    if (lambert > 255) lambert = 255;
    lambert = (lambert * lambert + 128) >> 8;       // squared falloff
    lambert = (lambert * lightIntensity) >> 8;
    uint32_t diffuseTerm = (lambert * diffuseCoef) >> 8;
    if (diffuseTerm > 255) diffuseTerm = 255;
    return (uint16_t)diffuseTerm;
}
#endif

// Returns true if the object's AABB is entirely outside the view frustum.
bool Scene::cullObject(Object* obj,
                       int32_t camCosX, int32_t camSinX,
                       int32_t camCosY, int32_t camSinY,
                       int32_t camCosZ, int32_t camSinZ) const {
    if (obj->isBillboard) return false; // keep billboards simple for now

    Vector3 camPos(camera->position);
    Vector3 objPos(obj->position);
    const Vector3& bMin = obj->boundingBoxMin;
    const Vector3& bMax = obj->boundingBoxMax;

    int outLeft = 0, outRight = 0, outTop = 0, outBottom = 0, outNear = 0, outFar = 0;
    float   fovFactor = camera->fovFactor;
    int32_t nearPlane = camera->nearPlane;
    int32_t farPlane  = camera->farPlane;

    // ---- Quick sphere-vs-frustum classification ----------------------------
    // One point transform (~20 ops) instead of the 8-corner AABB test
    // (~240 ops) for the vast majority of objects. Conservative bounding
    // sphere: centre at position+centreVolume (same rotation-ignoring
    // convention as prepareFrame's far pre-cull) with radius = longest
    // AABB dimension, which always covers the true half-diagonal
    // (halfDiag <= 0.866*maxExtent) plus slack for rotated meshes.
    //
    //   * sphere entirely outside ANY single frustum half-space → culled.
    //   * sphere entirely inside ALL half-spaces → visible.
    //   * straddling a boundary → fall through to the precise 8-corner test.
    //
    // The side-plane half-space checks use the plane equation directly
    // (signed distance × normal length), so they're valid regardless of
    // the sphere's z sign — no divides, no per-object sqrt (plane normal
    // lengths cullPlaneLh/Lv are cached per frame in prepareFrame()).
    {
        // PATCHED FOR arcade-os jet60. This was the longest FULL extent of the
        // AABB, where the true bounding sphere is HALF THE DIAGONAL - 1.9x the
        // radius and ~7x the volume for the 8 x 42 x 8 towers this city is made
        // of. The comment it replaced justified the slack as cover for rotated
        // meshes, but a sphere is rotation invariant: rotating a mesh inside its
        // AABB cannot move a vertex outside the sphere centred on that AABB with
        // half-diagonal radius. An over-large sphere costs admissions - objects
        // that take neither cheap exit get fully transformed and then thrown
        // away - and prep is what sets the frame here. See finding 4 in
        // taxi-3d/docs/jet-sort-report.md, where the rendered frame came out
        // byte-identical with 87 objects admitted instead of 80.
        const float ex = (float)(bMax.x - bMin.x);
        const float ey = (float)(bMax.y - bMin.y);
        const float ez = (float)(bMax.z - bMin.z);
        const float r  = 0.5f * sqrtf(ex * ex + ey * ey + ez * ez);
        constexpr float invFps = 1.0f / (float)FIXED_POINT_SCALE;
        const float cYc = (float)camCosY * invFps, cYs = (float)camSinY * invFps;
        const float cXc = (float)camCosX * invFps, cXs = (float)camSinX * invFps;
        const float cZc = (float)camCosZ * invFps, cZs = (float)camSinZ * invFps;
        const float px = (float)(objPos.x + obj->centreVolume.x - camPos.x);
        const float py = (float)(objPos.y + obj->centreVolume.y - camPos.y);
        const float pz = (float)(objPos.z + obj->centreVolume.z - camPos.z);
        // Camera rotation Y, X, Z — same order as the corner loop below.
        const float t1x =  px * cYc + pz * cYs;
        const float t1z = -px * cYs + pz * cYc;
        const float t2y =  py * cXc - t1z * cXs;
        const float t2z =  py * cXs + t1z * cXc;
        const float cxv =  t1x * cZc - t2y * cZs;
        const float cyv =  t1x * cZs + t2y * cZc;
        const float czv =  t2z;

        if (czv + r < (float)nearPlane) return true;   // fully behind
        if (czv - r > (float)farPlane)  return true;   // fully beyond

        const float hw  = (float)screenWidth  * 0.5f;
        const float hh  = (float)screenHeight * 0.5f;
        const float rLh = r * cullPlaneLh;
        const float rLv = r * cullPlaneLv;
        // Signed distance × plane length to each side plane: positive =
        // outside that plane's visible half-space.
        const float dR =  cxv * fovFactor - czv * hw;   // right of right plane
        const float dL = -cxv * fovFactor - czv * hw;   // left of left plane
        const float dT =  cyv * fovFactor - czv * hh;   // above top plane
        const float dB = -cyv * fovFactor - czv * hh;   // below bottom plane
        if (dR > rLh || dL > rLh || dT > rLv || dB > rLv) return true;

        if (czv - r > (float)nearPlane && czv + r < (float)farPlane &&
            dR < -rLh && dL < -rLh && dT < -rLv && dB < -rLv)
            return false;   // fully inside — skip the 8-corner test
    }

    int32_t objCosX = lookupCosI(obj->rotation.x), objSinX = lookupSinI(obj->rotation.x);
    int32_t objCosY = lookupCosI(obj->rotation.y), objSinY = lookupSinI(obj->rotation.y);
    int32_t objCosZ = lookupCosI(obj->rotation.z), objSinZ = lookupSinI(obj->rotation.z);

    for (int i = 0; i < 8; ++i) {
        Vector3 p(
            (i & 1) ? bMax.x : bMin.x,
            (i & 2) ? bMax.y : bMin.y,
            (i & 4) ? bMax.z : bMin.z);

        // Object rotation (X, Y, Z) — same order as renderObject
        p.assign(p.x,
                 (p.y * objCosX - p.z * objSinX) / FIXED_POINT_SCALE,
                 (p.y * objSinX + p.z * objCosX) / FIXED_POINT_SCALE);
        p.assign((p.x * objCosY + p.z * objSinY) / FIXED_POINT_SCALE,
                  p.y,
                 (-p.x * objSinY + p.z * objCosY) / FIXED_POINT_SCALE);
        p.assign((p.x * objCosZ - p.y * objSinZ) / FIXED_POINT_SCALE,
                 (p.x * objSinZ + p.y * objCosZ) / FIXED_POINT_SCALE,
                  p.z);

        // Translation
        p.add(objPos);
        p.add(camPos.inverse());

        // Camera rotation (Y, X, Z)
        Vector3 r;
        r.assign((p.x * camCosY + p.z * camSinY) / FIXED_POINT_SCALE,
                  p.y,
                 (-p.x * camSinY + p.z * camCosY) / FIXED_POINT_SCALE); p = r;
        r.assign(p.x,
                 (p.y * camCosX - p.z * camSinX) / FIXED_POINT_SCALE,
                 (p.y * camSinX + p.z * camCosX) / FIXED_POINT_SCALE); p = r;
        r.assign((p.x * camCosZ - p.y * camSinZ) / FIXED_POINT_SCALE,
                 (p.x * camSinZ + p.y * camCosZ) / FIXED_POINT_SCALE,
                  p.z); p = r;

        if (p.z < nearPlane) { outNear++; continue; }
        if (p.z > farPlane)  { outFar++;  continue; }
        if (p.z <= 0)        { outNear++; continue; }

        const float invZ = fovFactor / (float)p.z;
        int32_t sx = (int32_t)(p.x * invZ) + screenWidth / 2;
        int32_t sy = screenHeight / 2 - (int32_t)(p.y * invZ);
        if (sx < 0)            outLeft++;
        if (sx > screenWidth)  outRight++;
        if (sy < 0)            outTop++;
        if (sy > screenHeight) outBottom++;
    }

    return (outNear == 8 || outFar == 8 ||
            outLeft == 8 || outRight == 8 ||
            outTop  == 8 || outBottom == 8);
}

Scene::Scene(uint16_t* framebuffer, uint16_t* zBuffer, int screenWidth, int screenHeight)
    : camera(nullptr), directionalLight(nullptr), ambientLight(nullptr),
      framebuffer(framebuffer), zBuffer(zBuffer), screenWidth(screenWidth), screenHeight(screenHeight), scanlinesUpdated(nullptr) {
    initializeTrigTables();
    initBakedMat();
    renderer = new Rasterizer(framebuffer, screenWidth, screenHeight, zBuffer);
    postFX = new PostFX(screenWidth, screenHeight);
}
Scene::~Scene() {
    if (renderer) {
        delete renderer;
        renderer = nullptr;
    }
    if (postFX) {
        delete postFX;
        postFX = nullptr;
    }
}

void Scene::setFramebuffer(uint16_t *newBuffer) {
    framebuffer = newBuffer;
    renderer->setFramebuffer(newBuffer);
}

void Scene::resize(uint16_t* newFramebuffer, uint16_t* newZBuffer,
                   int newWidth, int newHeight) {
    framebuffer  = newFramebuffer;
    zBuffer      = newZBuffer;
    screenWidth  = newWidth;
    screenHeight = newHeight;
    if (renderer) {
        renderer->resize(newFramebuffer, newZBuffer, newWidth, newHeight);
    }
    // PostFX caches its own dimensions and (when enabled) owns scratch
    // buffers sized to them. Easiest correct thing is to rebuild it.
    if (postFX) {
        delete postFX;
        postFX = new PostFX(newWidth, newHeight);
    }
}

void Scene::addObject(Object* obj) {
    objects.push_back(obj);
}

void Scene::addPointLight(PointLight* light) {
    pointLights.push_back(light);
}

void Scene::setCamera(Camera* cam) {
    camera = cam;
    renderer->camera = cam;
}

void Scene::setDirectionalLight(DirectionalLight* light) {
    directionalLight = light;
}

void Scene::setAmbientLight(AmbientLight* light) {
    ambientLight = light;
}

#if MAX_PICK_QUERIES > 0
void Scene::setPickQueries(const PickQuery* queries, int count) {
    if (count < 0) count = 0;
    if (count > MAX_PICK_QUERIES) count = MAX_PICK_QUERIES;
    pickQueryCount = count;
    for (int i = 0; i < count; ++i) {
        pickQueries[i] = queries[i];
    }
    // Slots beyond `count` keep their previous contents but are ignored
    // by the rasterizer (it loops 0..pickQueryCount). We don't bother
    // zeroing them.
}
#endif

void Scene::reconstructCheckerboard() {
    // For each pixel that was NOT rendered this frame (opposite parity),
    // approximate it as a 3-tap average of its own previous-frame value and
    // its two freshly-rendered horizontal neighbours. The old value provides
    // temporal stability; the fresh neighbours provide spatial accuracy.
    // At the left/right edges the missing neighbour is omitted (2-tap).
    // This is a separate pass over the completed framebuffer so that painter's
    // algorithm overdraw doesn't cause repeated averaging of the gap pixel.
    const int cbParity = renderEvenLines ? 0 : 1;
    for (int y = 0; y < screenHeight; ++y) {
        uint16_t* row = framebuffer + y * screenWidth;
        for (int x = 0; x < screenWidth; ++x) {
            if (((x ^ y) & 1) == cbParity) continue; // freshly rendered — skip
            const uint16_t old = row[x];
            if (x == 0) {
                // 2-tap: old + right
                const uint16_t r = row[x + 1];
                row[x] = (uint16_t)(
                    ((((old >> 11) & 0x1F) + ((r >> 11) & 0x1F)) >> 1 << 11) |
                    ((((old >>  5) & 0x3F) + ((r >>  5) & 0x3F)) >> 1 <<  5) |
                     (((old        & 0x1F) + ( r        & 0x1F)) >> 1));
            } else if (x == screenWidth - 1) {
                // 2-tap: left + old
                const uint16_t l = row[x - 1];
                row[x] = (uint16_t)(
                    ((((l >> 11) & 0x1F) + ((old >> 11) & 0x1F)) >> 1 << 11) |
                    ((((l >>  5) & 0x3F) + ((old >>  5) & 0x3F)) >> 1 <<  5) |
                     ((( l       & 0x1F) + ( old        & 0x1F)) >> 1));
            } else {
                // 3-tap: left + old + right
                const uint16_t l = row[x - 1];
                const uint16_t r = row[x + 1];
                row[x] = (uint16_t)(
                    ((((l >> 11) & 0x1F) + ((old >> 11) & 0x1F) + ((r >> 11) & 0x1F)) / 3 << 11) |
                    ((((l >>  5) & 0x3F) + ((old >>  5) & 0x3F) + ((r >>  5) & 0x3F)) / 3 <<  5) |
                     ((( l       & 0x1F) + ( old        & 0x1F) + ( r        & 0x1F)) / 3));
            }
        }
    }
}

#if defined(CONFIG_IDF_TARGET_ESP32S3)
// clearBuffers() row fill using 128-bit EE.VST.128.XP stores (4 × uint32
// per instruction). dest must be 16-byte aligned; n32 need not be a
// multiple of 4 — trailing elements are handled with scalar stores.
static void jet_fill_u32x16(uint32_t* dest, uint32_t val, int n32) {
    // PATCHED IN THE VENDORED COPY (ion-drift tools/sync_to_arcade.ps1).
    // EE.VST.128.XP ignores the low 4 address bits. Without this, a framebuffer
    // that is merely 4-byte aligned - which arcade-os's is - has row 0's first
    // store land BEFORE the buffer and corrupt the neighbouring heap block.
    while (n32 > 0 && (((uintptr_t)dest) & 0xF)) { *dest++ = val; --n32; }
    const int n16 = n32 >> 2;
    if (n16 > 0) {
        __asm__ volatile (
            "ee.movi.32.q q0, %[v], 0\n\t"
            "ee.movi.32.q q0, %[v], 1\n\t"
            "ee.movi.32.q q0, %[v], 2\n\t"
            "ee.movi.32.q q0, %[v], 3\n\t"
            : : [v] "r"(val)
        );
        // EE.VST.128.XP takes the post-increment stride as a register,
        // not a bare immediate: as += stride_reg after each 128-bit store.
        // 4x-unrolled: the volatile asm + memory clobber stops GCC from
        // unrolling this itself, so retire 64 bytes per loop iteration by
        // hand and mop up the remainder one store at a time.
        const int stride = 16;
        for (int i = n16 >> 2; i > 0; --i) {
            __asm__ volatile (
                "ee.vst.128.xp q0, %[p], %[s]\n\t"
                "ee.vst.128.xp q0, %[p], %[s]\n\t"
                "ee.vst.128.xp q0, %[p], %[s]\n\t"
                "ee.vst.128.xp q0, %[p], %[s]\n\t"
                : [p] "+r"(dest) : [s] "r"(stride) : "memory"
            );
        }
        for (int i = n16 & 3; i > 0; --i) {
            __asm__ volatile (
                "ee.vst.128.xp q0, %[p], %[s]\n\t"
                : [p] "+r"(dest) : [s] "r"(stride) : "memory"
            );
        }
    }
    for (int r = n32 & 3; r-- > 0; ) *dest++ = val;
}
#endif

void PERF_CRITICAL Scene::clearBuffers() {
    //cast the framebuffer to a 32-bit pointer
    uint32_t* framebuffer32 = (uint32_t*)framebuffer;

    // Wireframe mode forces a solid black background regardless of the
    // configured backcolor / gradient so the outlines pop. We detour the
    // gradient and backcolor for the duration of this clear and restore
    // them on the way out so the host-visible state is unchanged.
    uint16_t* savedGradient = backgroundGradientColors;
    uint16_t  savedBack     = backcolor;
    const bool wireClear = (renderer && renderer->wireframeMode);
    if (wireClear) {
        backgroundGradientColors = nullptr;
        backcolor = 0;
    }

    if (clearRenderBuffer) {
        if (renderer->checkerboardMode) {
            // Checkerboard clear: only wipe current-parity pixels. Opposite-parity
            // pixels keep last frame's values; when CHECKERBOARD_RECONSTRUCTION
            // is enabled the rasterizer fills them in-situ as it renders each
            // current-parity pixel.
            const int cbParity = renderEvenLines ? 0 : 1;
            for (int y = 0; y < screenHeight; ++y) {
                #if DEBUG_OVERDRAW
                const uint16_t lineColor = 0;
                #else
                const uint16_t lineColor = backgroundGradientColors
                                           ? backgroundGradientColors[y] : backcolor;
                #endif
                uint16_t* row = framebuffer + y * screenWidth;
                for (int x = 0; x < screenWidth; ++x) {
                    if (((x ^ y) & 1) == cbParity) {
                        row[x] = lineColor;
                    }
                }
            }
            #if Z_BUFFERING
            memset(zBuffer, 0xFF, (size_t)screenWidth * screenHeight * sizeof(uint16_t));
            #endif
        } else if (renderer->interlacedMode) {
            for (int y = (int)renderEvenLines; y < screenHeight; y += 2) {
                #if DEBUG_OVERDRAW
                uint16_t lineColor = 0;
                uint32_t lineColor32 = 0;
                #else
                uint16_t lineColor = backgroundGradientColors ? backgroundGradientColors[y] : backcolor;
                uint32_t lineColor32 = (lineColor << 16) | lineColor;
                #endif
                #if HALF_WIDTH_BUFFERS
                const int divisor = 4;
                #else
                const int divisor = 2;
                #endif
                #if FIELD_BUFFERS
                // Field-buffer layout: packed half-height. Row index = y>>1.
                uint32_t* lineStart = framebuffer32 + (y >> 1) * (screenWidth / divisor);
                #else
                uint32_t* lineStart = framebuffer32 + y * (screenWidth / divisor);
                #endif
#if defined(CONFIG_IDF_TARGET_ESP32S3)
                jet_fill_u32x16(lineStart, lineColor32, screenWidth / divisor);
#else
                for (int x = 0; x < screenWidth / divisor; x++) {
                    lineStart[x] = lineColor32;
                }
#endif
                #if Z_BUFFERING
                #if HALF_WIDTH_BUFFERS
                memset(zBuffer + (y / 2) * (screenWidth / 2), 0xFF, (screenWidth / 2) * sizeof(uint16_t));
                #else
                memset(zBuffer + y * screenWidth, 0xFF, screenWidth * sizeof(uint16_t));
                #endif
                #endif
            }
        } else {
            // Non-interlaced clear: fill every row. Honour the per-row
            // background gradient if one is set; otherwise fall back to
            // a solid backcolor. Note that `memset(fb, backcolor, ...)` is
            // wrong for a 16-bit backcolor because memset writes bytes —
            // we'd get backcolor's low byte duplicated. Pack two pixels per
            // 32-bit store and walk rows so the gradient (if any) actually
            // shows up.
            // HALF_WIDTH_BUFFERS makes the buffer half-width. FIELD_BUFFERS
            // additionally halves the height (each field buffer covers every other
            // row). When only HALF_WIDTH_BUFFERS is set the buffer is half-width
            // but FULL-height, so rowCount must be screenHeight in that case.
            #if HALF_WIDTH_BUFFERS
            const int rowPixels = screenWidth / 2;
            #if FIELD_BUFFERS
            const int rowCount  = screenHeight / 2;
            #else
            const int rowCount  = screenHeight;
            #endif
            #else
            const int rowPixels = screenWidth;
            const int rowCount  = screenHeight;
            #endif
            const int row32     = rowPixels / 2; // pairs of pixels per row
            // Respect yBandMin/yBandMax so the clear only touches the rows
            // assigned to this band (critical for virtual-base-pointer band
            // rendering where the buffer only covers [yBandMin, yBandMax)).
            const int yClearStart = renderer ? renderer->yBandMin             : 0;
            const int yClearEnd   = renderer ? std::min(renderer->yBandMax, rowCount) : rowCount;
            for (int y = yClearStart; y < yClearEnd; ++y) {
                #if DEBUG_OVERDRAW
                const uint16_t lineColor = 0;
                #else
                // Source row in the gradient table maps 1:1 with the
                // logical screen row (`screenHeight`). When the back buffer
                // is half-height (HALF_WIDTH_BUFFERS implies a y/2-style
                // layout in some configs), index by `y` directly because
                // rowCount already accounts for the halving.
                const uint16_t lineColor = backgroundGradientColors
                                           ? backgroundGradientColors[y * (screenHeight / rowCount)]
                                           : backcolor;
                #endif
                const uint32_t lineColor32 = ((uint32_t)lineColor << 16) | lineColor;
                uint32_t* lineStart = framebuffer32 + y * row32;
#if defined(CONFIG_IDF_TARGET_ESP32S3)
                jet_fill_u32x16(lineStart, lineColor32, row32);
#else
                for (int x = 0; x < row32; ++x) lineStart[x] = lineColor32;
#endif
            }
            #if Z_BUFFERING
            // Z-buffer stride matches the rasterizer's depth-buffer layout
            // (see ZBUFFER_STRIDE in Renderer.hpp): half-width when
            // HALF_WIDTH_BUFFERS is on, per-pixel otherwise. Height is the
            // full screen height regardless.
            #if HALF_WIDTH_BUFFERS
            memset(zBuffer, 0xFF, (size_t)(screenWidth / 2) * screenHeight * sizeof(uint16_t));
            #else
            memset(zBuffer, 0xFF, (size_t)screenWidth * screenHeight * sizeof(uint16_t));
            #endif
            #endif
        }
    }
    //memset(scanlinesUpdated, 0, screenHeight * sizeof(bool));

    // Restore the host-visible background state regardless of whether we
    // detoured it for the wireframe-mode clear above.
    if (wireClear) {
        backgroundGradientColors = savedGradient;
        backcolor = savedBack;
    }
}

// ---------------------------------------------------------------------------
// ObjectSortTree - PATCHED FOR arcade-os jet60. See Scene.hpp for why this
// exists at all; what follows is only how it is built and how it is walked.
// ---------------------------------------------------------------------------

// World-space AABB of an object. Conservative is enough for correctness: a box
// that is too big can only cost a clean split, never produce a wrong one.
static void jetWorldBox(const Object* o, int32_t mn[3], int32_t mx[3]) {
    int32_t lo[3] = { o->boundingBoxMin.x, o->boundingBoxMin.y, o->boundingBoxMin.z };
    int32_t hi[3] = { o->boundingBoxMax.x, o->boundingBoxMax.y, o->boundingBoxMax.z };
    // A rotated object's local box is not its world box. Every static object in
    // the city is authored axis-aligned (rotations are baked at build time), so
    // rather than transform eight corners per object, fall back to the
    // rotation-invariant cube that contains the mesh whatever its angle.
    if (o->rotation.x != 0 || o->rotation.y != 0 || o->rotation.z != 0) {
        int32_t r = 0;
        for (int a = 0; a < 3; ++a) {
            const int32_t la = lo[a] < 0 ? -lo[a] : lo[a];
            const int32_t ha = hi[a] < 0 ? -hi[a] : hi[a];
            if (la > r) r = la;
            if (ha > r) r = ha;
        }
        lo[0] = lo[1] = lo[2] = -r;
        hi[0] = hi[1] = hi[2] =  r;
    }
    const int32_t pos[3] = { o->position.x, o->position.y, o->position.z };
    for (int a = 0; a < 3; ++a) { mn[a] = pos[a] + lo[a]; mx[a] = pos[a] + hi[a]; }
}

bool ObjectSortTree::build(const std::vector<Object*>& objs, int leafMax, int32_t slack) {
    nodes.clear(); items.clear(); dynamics.clear(); dynNode.clear();
    leaves_ = 0; worstLeaf_ = 0; unsplit_ = 0;
    src = &objs;
    if (leafMax < 1) leafMax = 1;

    std::vector<Box> boxes;
    boxes.reserve(objs.size());
    for (size_t i = 0; i < objs.size(); ++i) {
        Object* o = objs[i];
        if (!o) continue;
        if (o->sortDynamic) { dynamics.push_back((int32_t)i); continue; }
        Box b;
        b.idx = (int32_t)i;
        jetWorldBox(o, b.mn, b.mx);
        boxes.push_back(b);
    }
    if (boxes.empty()) return false;

    items.resize(boxes.size());
    dynNode.assign(dynamics.size(), -1);
    nodes.reserve(boxes.size() * 2);
    if (slack < 0) slack = 0;
    buildRec(boxes, 0, (int)boxes.size(), leafMax, slack, 0);
    return true;
}

void ObjectSortTree::preferPlane(uint8_t axis, int32_t value) {
    if (axis > 2) return;
    Plane p;
    p.axis  = axis;
    p.value = value;
    preferred.push_back(p);
}

void ObjectSortTree::clearPreferredPlanes() { preferred.clear(); }

int32_t ObjectSortTree::buildRec(std::vector<Box>& v, int lo, int hi,
                                 int leafMax, int32_t slack, int depth) {
    const int n = hi - lo;
    int     bestAxis  = -1;
    int32_t bestValue = 0;
    int     bestScore = INT32_MAX;

    // CANDIDATE PLANES ARE THE OBJECTS' OWN MAX FACES, and that is not a
    // heuristic. For any clean split the legal planes form the interval
    // [largest left-hand max, smallest right-hand min], and the low end of
    // that interval IS some object's max face - so every split a plane could
    // possibly make is already reachable from this candidate set. Testing min
    // faces as well would double the work and find nothing new.
    //
    // The straddle test bails on the first object that crosses the plane,
    // which is what keeps this affordable: most candidates die immediately and
    // only a genuinely clean one pays for the full scan.
    if (n > leafMax && depth < 62) {
        // A caller's plane first. It is not a heuristic and not a shortcut: any
        // clean plane is correct, so this only decides WHICH correct order the
        // tree produces, and the caller knows things about its own world that
        // a balance score cannot see. See preferPlane() in Scene.hpp.
        for (size_t pi = 0; pi < preferred.size() && bestAxis < 0; ++pi) {
            const int     a   = preferred[pi].axis;
            const int32_t val = preferred[pi].value;
            int  left = 0, right = 0;
            bool clean = true;
            for (int j = lo; j < hi; ++j) {
                if (v[j].mx[a] <= val + slack)      ++left;
                else if (v[j].mn[a] >= val - slack) ++right;
                else { clean = false; break; }
            }
            if (clean && left > 0 && right > 0) { bestAxis = a; bestValue = val; }
        }
        for (int a = 0; a < 3 && bestAxis < 0; ++a) {
            for (int i = lo; i < hi; ++i) {
                const int32_t val = v[i].mx[a];
                int  left = 0, right = 0;
                bool clean = true;
                for (int j = lo; j < hi; ++j) {
                    if (v[j].mx[a] <= val + slack)      ++left;
                    else if (v[j].mn[a] >= val - slack) ++right;
                    else { clean = false; break; }
                }
                if (!clean || left == 0 || right == 0) continue;
                const int score = (left > right) ? (left - right) : (right - left);
                if (score < bestScore) { bestScore = score; bestAxis = a; bestValue = val; }
            }
        }
    }

    const int32_t self = (int32_t)nodes.size();
    nodes.push_back(Node());

    if (bestAxis < 0) {
        // No clean plane, or small enough already: a leaf, ordered later by
        // the caller's depth key.
        for (int k = lo; k < hi; ++k) items[k] = v[k].idx;
        Node& nd = nodes[self];
        nd.axis = 3; nd.value = 0; nd.a = lo; nd.b = n;
        for (int ax = 0; ax < 3; ++ax) { nd.mn[ax] = INT32_MAX; nd.mx[ax] = INT32_MIN; }
        for (int k = lo; k < hi; ++k)
            for (int ax = 0; ax < 3; ++ax) {
                if (v[k].mn[ax] < nd.mn[ax]) nd.mn[ax] = v[k].mn[ax];
                if (v[k].mx[ax] > nd.mx[ax]) nd.mx[ax] = v[k].mx[ax];
            }
        ++leaves_;
        if (n > worstLeaf_) worstLeaf_ = n;
        if (n > leafMax)    ++unsplit_;
        return self;
    }

    int mid = lo;
    for (int i = lo; i < hi; ++i)
        if (v[i].mx[bestAxis] <= bestValue + slack) { std::swap(v[i], v[mid]); ++mid; }

    const int32_t minusChild = buildRec(v, lo,  mid, leafMax, slack, depth + 1);
    const int32_t plusChild  = buildRec(v, mid, hi,  leafMax, slack, depth + 1);
    Node& nd = nodes[self];   // taken after the recursion: push_back may have moved it
    nd.axis  = (uint8_t)bestAxis;
    nd.value = bestValue;
    nd.a     = minusChild;
    nd.b     = plusChild;
    for (int ax = 0; ax < 3; ++ax) {
        nd.mn[ax] = nodes[minusChild].mn[ax] < nodes[plusChild].mn[ax]
                  ? nodes[minusChild].mn[ax] : nodes[plusChild].mn[ax];
        nd.mx[ax] = nodes[minusChild].mx[ax] > nodes[plusChild].mx[ax]
                  ? nodes[minusChild].mx[ax] : nodes[plusChild].mx[ax];
    }
    return self;
}

int32_t ObjectSortTree::leafFor(int32_t x, int32_t y, int32_t z) const {
    if (nodes.empty()) return -1;
    const int32_t q[3] = { x, y, z };
    int32_t ni = 0;
    for (int guard = 0; guard < 64; ++guard) {
        const Node& nd = nodes[ni];
        if (nd.axis == 3) return ni;
        ni = (q[nd.axis] >= nd.value) ? nd.b : nd.a;
    }
    return ni;
}

void ObjectSortTree::order(int32_t camX, int32_t camY, int32_t camZ,
                           std::vector<int32_t>& outOrder,
                           std::vector<int32_t>& outGroups,
                           BoxRejectFn reject, void* rejectCtx) {
    outOrder.clear();
    outGroups.clear();
    if (nodes.empty()) return;

    // A MOVER IS PLACED BY ITS CENTRE, not by its box. A point descends to
    // exactly one leaf, so there is no straddle case to invent a rule for, and
    // the object is then ordered against that leaf's few neighbours by the
    // same depth fallback everything else in the leaf uses. Emitting it after
    // the leaf's static objects, in a group of its own, is what ACTOR_BIAS used to
    // buy: a car standing on a road chunk draws on top of the road it stands
    // on, while the tree still orders it against every other leaf.
    if (src) {
        dynNode.resize(dynamics.size());
        for (size_t d = 0; d < dynamics.size(); ++d) {
            const Object* o = (*src)[dynamics[d]];
            dynNode[d] = leafFor(o->position.x + o->centreVolume.x,
                                 o->position.y + o->centreVolume.y,
                                 o->position.z + o->centreVolume.z);
        }
    }

    // A mover whose leaf never got visited still has to be drawn: the leaf was
    // dismissed on the box of the STATIC things in it, which says nothing about
    // where the car is now. Track what got emitted and sweep up at the end.
    dynDone.assign(dynamics.size(), 0);

    const int32_t cam[3] = { camX, camY, camZ };
    // Explicit stack, not recursion: this runs on a worker task with a small
    // stack. buildRec caps depth at 62, so 72 slots cannot overflow.
    int32_t stack[72];
    int sp = 0;
    stack[sp++] = 0;
    while (sp > 0) {
        const int32_t ni = stack[--sp];
        const Node&   nd = nodes[ni];
        // WHOLE SUBTREE OUT OF VIEW. One test against a box in internal SRAM
        // stands in for a per-object cull that would have had to page every
        // one of those objects in from PSRAM.
        if (reject && reject(rejectCtx, nd.mn, nd.mx)) continue;
        if (nd.axis == 3) {
            outGroups.push_back((int32_t)outOrder.size());
            for (int k = 0; k < nd.b; ++k) outOrder.push_back(items[nd.a + k]);
            // Each mover gets a GROUP OF ITS OWN, immediately after the leaf
            // it is standing in. The caller depth-sorts inside a group, so
            // sharing the leaf's group would just hand the car back to the
            // key that kept losing it; its own group makes "after the scenery
            // it stands on" structural. The blast radius is one leaf - at most
            // a handful of neighbours - where ACTOR_BIAS applied city-wide.
            for (size_t d = 0; d < dynNode.size(); ++d)
                if (dynNode[d] == ni) {
                    outGroups.push_back((int32_t)outOrder.size());
                    outOrder.push_back(dynamics[d]);
                    dynDone[d] = 1;
                }
            continue;
        }
        // Camera on the plus side: every ray from it crosses the plane at most
        // once, so all plus-side hits precede all minus-side ones and the minus
        // subtree is drawn first. Push the near child first so the far one pops.
        const bool    camPlus    = cam[nd.axis] >= nd.value;
        const int32_t drawFirst  = camPlus ? nd.a : nd.b;
        const int32_t drawSecond = camPlus ? nd.b : nd.a;
        if (sp + 2 <= (int)(sizeof(stack) / sizeof(stack[0]))) {
            stack[sp++] = drawSecond;
            stack[sp++] = drawFirst;
        }
    }
    // The sweep-up. Last is the only safe place for an object whose
    // neighbourhood was culled - and there is nothing left there to fight.
    for (size_t d = 0; d < dynDone.size(); ++d)
        if (!dynDone[d]) {
            outGroups.push_back((int32_t)outOrder.size());
            outOrder.push_back(dynamics[d]);
        }
    outGroups.push_back((int32_t)outOrder.size());
}

size_t ObjectSortTree::byteSize() const {
    return nodes.capacity()    * sizeof(Node)
         + items.capacity()    * sizeof(int32_t)
         + dynamics.capacity() * sizeof(int32_t)
         + dynNode.capacity()  * sizeof(int32_t)
         + dynDone.capacity()  * sizeof(uint8_t);
}

// PATCHED FOR arcade-os jet60. Was inline in renderObject(); see Scene.hpp for
// the measurement that moved it. Composed camera rotation Rz * Rx * Ry, since
// the original per-vertex code applied Y then X then Z.
void Scene::updateCameraMatrix(int32_t camCosX, int32_t camSinX,
                               int32_t camCosY, int32_t camSinY,
                               int32_t camCosZ, int32_t camSinZ) {
    const int32_t cx = camCosX, sx = camSinX;
    const int32_t cy = camCosY, sy = camSinY;
    const int32_t cz = camCosZ, sz = camSinZ;
    // K = Rx * Ry
    const int32_t k00 = cy;
    const int32_t k01 = 0;
    const int32_t k02 = sy;
    const int32_t k10 = (int32_t)((int64_t)sx * sy / FIXED_POINT_SCALE);
    const int32_t k11 = cx;
    const int32_t k12 = (int32_t)(-(int64_t)sx * cy / FIXED_POINT_SCALE);
    const int32_t k20 = (int32_t)(-(int64_t)cx * sy / FIXED_POINT_SCALE);
    const int32_t k21 = sx;
    const int32_t k22 = (int32_t)((int64_t)cx * cy / FIXED_POINT_SCALE);
    // M = Rz * K
    const int32_t m00 = (int32_t)(((int64_t)cz * k00 - (int64_t)sz * k10) / FIXED_POINT_SCALE);
    const int32_t m01 = (int32_t)(((int64_t)cz * k01 - (int64_t)sz * k11) / FIXED_POINT_SCALE);
    const int32_t m02 = (int32_t)(((int64_t)cz * k02 - (int64_t)sz * k12) / FIXED_POINT_SCALE);
    const int32_t m10 = (int32_t)(((int64_t)sz * k00 + (int64_t)cz * k10) / FIXED_POINT_SCALE);
    const int32_t m11 = (int32_t)(((int64_t)sz * k01 + (int64_t)cz * k11) / FIXED_POINT_SCALE);
    const int32_t m12 = (int32_t)(((int64_t)sz * k02 + (int64_t)cz * k12) / FIXED_POINT_SCALE);
    constexpr float inv = 1.0f / (float)FIXED_POINT_SCALE;
    fCamM00 = (float)m00 * inv; fCamM01 = (float)m01 * inv; fCamM02 = (float)m02 * inv;
    fCamM10 = (float)m10 * inv; fCamM11 = (float)m11 * inv; fCamM12 = (float)m12 * inv;
    fCamM20 = (float)k20 * inv; fCamM21 = (float)k21 * inv; fCamM22 = (float)k22 * inv;
}

// Whole-subtree frustum reject for ObjectSortTree::order(). This is the same
// conservative sphere-vs-frustum classification cullObject() opens with, over
// the box around a subtree instead of around one mesh - so a node is dismissed
// on exactly the evidence that would have dismissed every object in it, and
// never on less.
//
// It reads nothing but this struct, which is a few dozen bytes of stack. That
// is the entire point: the per-object cull it replaces has to touch position,
// centreVolume and both bounding-box corners of every object, and on this
// console the objects live in PSRAM.
struct JetSubtreeCull {
    float   camX, camY, camZ;
    float   cYc, cYs, cXc, cXs, cZc, cZs;
    float   fovFactor, hw, hh, planeLh, planeLv;
    float   nearPlane, farPlane;
};

static bool jetSubtreeReject(void* ctx, const int32_t mn[3], const int32_t mx[3]) {
    const JetSubtreeCull& c = *(const JetSubtreeCull*)ctx;
    const float ex = (float)(mx[0] - mn[0]);
    const float ey = (float)(mx[1] - mn[1]);
    const float ez = (float)(mx[2] - mn[2]);
    const float r  = 0.5f * sqrtf(ex * ex + ey * ey + ez * ez);
    const float px = (float)(mx[0] + mn[0]) * 0.5f - c.camX;
    const float py = (float)(mx[1] + mn[1]) * 0.5f - c.camY;
    const float pz = (float)(mx[2] + mn[2]) * 0.5f - c.camZ;
    // Camera rotation Y, X, Z - the same order cullObject uses.
    const float t1x =  px * c.cYc + pz * c.cYs;
    const float t1z = -px * c.cYs + pz * c.cYc;
    const float t2y =  py * c.cXc - t1z * c.cXs;
    const float t2z =  py * c.cXs + t1z * c.cXc;
    const float cxv =  t1x * c.cZc - t2y * c.cZs;
    const float cyv =  t1x * c.cZs + t2y * c.cZc;
    const float czv =  t2z;
    if (czv + r < c.nearPlane) return true;
    if (czv - r > c.farPlane)  return true;
    const float rLh = r * c.planeLh, rLv = r * c.planeLv;
    const float dR =  cxv * c.fovFactor - czv * c.hw;
    const float dL = -cxv * c.fovFactor - czv * c.hw;
    const float dT =  cyv * c.fovFactor - czv * c.hh;
    const float dB = -cyv * c.fovFactor - czv * c.hh;
    return (dR > rLh || dL > rLh || dT > rLv || dB > rLv);
}

void Scene::prepareFrame() {
    if (!camera) return;

    JET_PREP_CLOCK(prepT);
    // renderEvenLines drives the frame-parity selection used by both interlaced
    // and checkerboard modes.  In interlaced mode it selects which rows to draw;
    // in checkerboard mode it selects which (x+y) pixel parity to draw.  When
    // neither mode is active we force it to false so drawTriangle's
    // `yStart = interlacedMode ? (minY + renderEvenLines) : minY` never shifts
    // the first scanline by a stray ±1.
    renderEvenLines = (renderer->interlacedMode || renderer->checkerboardMode)
                      ? (frameCounter % 2 == 0)
                      : false;
    clearBuffers();
    JET_PREP_SPLIT(lastFramePrepClearUs, prepT);

#if MAX_PICK_QUERIES > 0
    // Reset pick results for this frame and hand the arrays to the
    // rasterizer. With FIELD_BUFFERS the rasterizer only writes one
    // parity of rows per frame, so a query landing on the "off" parity
    // would never be tested by drawTriangle's per-row pick loop. Snap
    // the y to a row this frame's field actually covers so the host's
    // requested pixel still produces a hit on roughly the right
    // location (off by one row at most). The snapped y is what gets
    // stored in PickResult.y so the caller can render their cursor on
    // the same row the renderer inspected.
    for (int i = 0; i < pickQueryCount; ++i) {
        pickResults[i] = PickResult{};
    #if FIELD_BUFFERS
        if (renderer->interlacedMode && pickQueries[i].y >= 0) {
            const int desiredParity = renderEvenLines ? 0 : 1;
            int sy = pickQueries[i].y;
            if ((sy & 1) != desiredParity) {
                // Prefer nudging up; clamp to a valid row at the bottom.
                if (sy + 1 < screenHeight) sy += 1; else if (sy > 0) sy -= 1;
            }
            pickQueries[i].y = (int16_t)sy;
        }
    #endif
    }
    renderer->pickQueries     = pickQueries;
    renderer->pickResults     = pickResults;
    renderer->pickQueryCount  = pickQueryCount;
#endif

#if LIGHTING
    if (directionalLight) directionalLight->updateViewSpaceDirection(camera);
#endif

    // Thread sky gradient and frame counter into the rasterizer so
    // WATER_REFLECT shading can sample them. Intentionally outside
    // #if LIGHTING — works with LIGHTING=0 on firmware.
    renderer->gradientColors = backgroundGradientColors;
    renderer->gradientSize   = backgroundGradientColors ? screenHeight : 0;
    renderer->frameCounter   = frameCounter;
    renderer->waterTime      = waterTime;

    int32_t camCosX, camSinX, camCosY, camSinY, camCosZ, camSinZ;
    camera->getRotationMatrix(camCosX, camSinX, camCosY, camSinY, camCosZ, camSinZ);
    updateCameraMatrix(camCosX, camSinX, camCosY, camSinY, camCosZ, camSinZ);

    // Frustum side-plane normal lengths for cullObject's quick sphere
    // test. fovFactor changes at runtime (boost FOV kick) so refresh per
    // frame rather than caching at init.
    {
        const float f  = camera->fovFactor;
        const float hw = (float)screenWidth  * 0.5f;
        const float hh = (float)screenHeight * 0.5f;
        cullPlaneLh = sqrtf(f * f + hw * hw);
        cullPlaneLv = sqrtf(f * f + hh * hh);
    }

    // Compute the screen row of the water/sky horizon from the camera's pitch.
    // Derivation: a world point at the water horizon (infinite Z along forward)
    // projects to sy = screenH/2 + camSinX * fovFactor / FIXED_POINT_SCALE.
    // WATER_REFLECT uses mirrorY = 2*waterlineY - y so reflections align
    // correctly for both near (low on screen) and distant (high on screen) objects.
    renderer->waterlineY = screenHeight / 2
                         + (int)(camSinX * camera->fovFactor / 1024.0f);

    lastFrameTransformedVertices = 0;
    lastFramePrepObjectUs = 0;
    renderQueue.clear();
    triYSpan.clear();          // patched: parallel to renderQueue, see Scene.hpp
    int drawnObjs = 0;
    // PATCHED FOR arcade-os jet60: does anything in this frame use the two
    // out-of-band draw orders? The tree orders the NORMAL band and nothing
    // else, so a skybox (noWriteZBuffer) or an overlay (ignoreZBuffer) sends
    // the whole frame back to the bucket sort, which knows how to keep them
    // first and last. Taxi Rush uses neither; this is here so a scene that
    // does cannot silently lose them.
    bool sawBands = false;

    // PATCHED FOR arcade-os jet60: THE PER-OBJECT BODY IS A LAMBDA so it can
    // be driven two ways - straight down the object list as before, or in the
    // leaf order the sort tree produces. Nothing inside it changed except
    // `continue` becoming `return`, which is the same statement in a lambda.
    auto drawOne = [&](Object* obj) {
        if (!obj->enabled) return;
        // 1) Quick sphere far-cull before the expensive 8-corner AABB test.
        //    distSq to the object centre is computed unconditionally so it
        //    is also available for the fade ramps and LOD pick below,
        //    replacing the old lazy-compute block. The conservative sphere
        //    radius used is the object's longest bounding-box dimension
        //    (always >= the true bounding-sphere radius — never drops a
        //    visible object).
        uint8_t objAlpha = 255;
        const int32_t _ocx = (obj->position.x + obj->centreVolume.x) - camera->position.x;
        const int32_t _ocy = (obj->position.y + obj->centreVolume.y) - camera->position.y;
        const int32_t _ocz = (obj->position.z + obj->centreVolume.z) - camera->position.z;
        int64_t distSq = (int64_t)_ocx*_ocx + (int64_t)_ocy*_ocy + (int64_t)_ocz*_ocz;
        int32_t dist   = -1;
        {
            const int32_t maxExtent = std::max({
                obj->boundingBoxMax.x - obj->boundingBoxMin.x,
                obj->boundingBoxMax.y - obj->boundingBoxMin.y,
                obj->boundingBoxMax.z - obj->boundingBoxMin.z});
            const int64_t farCutoff = static_cast<int64_t>(camera->farPlane) + maxExtent;
            if (distSq > farCutoff * farCutoff) return;
        }
        // 2) Object-level AABB frustum cull (all 8 corners; full rotation).
        if (cullObject(obj, camCosX, camSinX, camCosY, camSinY, camCosZ, camSinZ))
            return;
        // 3) Per-object distance fade (two ramps, multiplied):
        //     - fadeFar > 0:   close=opaque, far=invisible (decor fade-out).
        //     - appearFar > 0: close=invisible, far=opaque (LOD impostor
        //                      that pops in at distance).
        //     Both can be combined on the same object if you want a
        //     visibility "band" — opaque only between two distances.
        //     fadeFar==0 / appearFar==0 disable the respective ramp.
        //     Distance is measured in world space from the camera to the
        //     object's centre (position + centreVolume). Beyond fadeFar
        //     OR closer than appearNear the object is skipped entirely
        //     (no transform, no per-tri work), so this is a real perf
        //     win not just a visual fade.
        // distSq is always valid here (computed above for the sphere pre-cull).
        if (obj->fadeFar > 0 || obj->appearFar > 0) {
            if (obj->fadeFar > 0) {
                const int64_t farSq = (int64_t)obj->fadeFar * obj->fadeFar;
                if (distSq >= farSq) return; // fully past fade-out — skip
                const int64_t nearSq = (int64_t)obj->fadeNear * obj->fadeNear;
                if (distSq > nearSq && obj->fadeFar > obj->fadeNear) {
                    if (dist < 0) dist = (int32_t)sqrtf((float)distSq);
                    const int32_t span = obj->fadeFar - obj->fadeNear;
                    const int32_t over = dist - obj->fadeNear;
                    int32_t a = 255 - (over * 255) / span;
                    if (a < 0) a = 0;
                    if (a > 255) a = 255;
                    objAlpha = (uint8_t)((objAlpha * a) / 255);
                }
            }

            // Appear-in ramp (LOD impostor).
            if (obj->appearFar > 0) {
                const int64_t nearSq = (int64_t)obj->appearNear * obj->appearNear;
                if (distSq <= nearSq) return; // still too close — skip
                const int64_t farSq = (int64_t)obj->appearFar * obj->appearFar;
                if (distSq < farSq && obj->appearFar > obj->appearNear) {
                    if (dist < 0) dist = (int32_t)sqrtf((float)distSq);
                    const int32_t span = obj->appearFar - obj->appearNear;
                    const int32_t over = dist - obj->appearNear;
                    int32_t a = (over * 255) / span;
                    if (a < 0) a = 0;
                    if (a > 255) a = 255;
                    objAlpha = (uint8_t)((objAlpha * a) / 255);
                }
                // distSq >= farSq: fully appeared, multiplier already 255.
            }

            if (objAlpha == 0) return;
        }

        // 1c) Global LOD pick. The head Object IS LOD 0; entries in
        //     obj->lodMeshes are LOD 1, 2, ... in order. Beyond the last
        //     available LOD: cull (default) or clamp (`lodPersist`).
        //     The picked Object* contributes ONLY mesh data; the head's
        //     transform / flags / AABB / fade ramps still drive the draw.
        Object* meshSource = obj;
        if (lodScale > 0) {
            if (dist < 0 && distSq >= 0) dist = (int32_t)sqrtf((float)distSq);
            int32_t level = (dist < 0 ? 0 : dist / lodScale);
            level += (int32_t)lodBias + (int32_t)obj->lodBias;
            if (level < 0) level = 0;

            const int availableLODs = (int)obj->lodMeshes.size();
            if (level == 0) {
                meshSource = obj;
            } else if (level <= availableLODs) {
                Object* candidate = obj->lodMeshes[level - 1];
                meshSource = candidate ? candidate : obj;
            } else if (obj->lodPersist) {
                if (availableLODs > 0) {
                    Object* candidate = obj->lodMeshes[availableLODs - 1];
                    meshSource = candidate ? candidate : obj;
                }
                // else: no LOD chain at all, draw the head as-is.
            } else {
                return; // ran out of LODs and not persisting → cull.
            }
        }

        // 2) Transform + project + per-triangle cull, push into renderQueue
        if (obj->ignoreZBuffer || obj->noWriteZBuffer) sawBands = true;
#if JET_PROFILE_PREP
        const uint32_t robT = jetPrepNowUs();
#endif
        renderObject(obj, camCosX, camSinX, camCosY, camSinY, camCosZ, camSinZ, objAlpha, meshSource);
#if JET_PROFILE_PREP
        lastFramePrepObjectUs += jetPrepNowUs() - robT;
#endif
        ++drawnObjs;
    };

    // TWO DRIVERS, ONE BODY. Without a tree this is registration order and
    // the depth bucket sort below does all of the work. With one, the objects
    // arrive already in a correct back-to-front order BETWEEN leaves, and
    // groupQueueStart records where each leaf's triangles begin - so the depth
    // key is only ever asked to settle the handful of objects inside one leaf,
    // which is the one job it was ever good at.
    groupQueueStart.clear();
    const bool useTree = (sortTree != nullptr) && !sortTree->empty();
    if (useTree) {
        JetSubtreeCull cull;
        constexpr float invFps = 1.0f / (float)FIXED_POINT_SCALE;
        cull.camX = (float)camera->position.x;
        cull.camY = (float)camera->position.y;
        cull.camZ = (float)camera->position.z;
        cull.cYc  = (float)camCosY * invFps; cull.cYs = (float)camSinY * invFps;
        cull.cXc  = (float)camCosX * invFps; cull.cXs = (float)camSinX * invFps;
        cull.cZc  = (float)camCosZ * invFps; cull.cZs = (float)camSinZ * invFps;
        cull.fovFactor = camera->fovFactor;
        cull.hw = (float)screenWidth * 0.5f;
        cull.hh = (float)screenHeight * 0.5f;
        cull.planeLh = cullPlaneLh;
        cull.planeLv = cullPlaneLv;
        cull.nearPlane = (float)camera->nearPlane;
        cull.farPlane  = (float)camera->farPlane;
        sortTree->order(camera->position.x, camera->position.y, camera->position.z,
                        sortOrder, sortGroups, &jetSubtreeReject, &cull);
        const int nObj = static_cast<int>(objects.size());
        size_t g = 0;
        for (size_t k = 0; k < sortOrder.size(); ++k) {
            while (g < sortGroups.size() && sortGroups[g] == static_cast<int32_t>(k)) {
                groupQueueStart.push_back(static_cast<int32_t>(renderQueue.size()));
                ++g;
            }
            const int32_t oi = sortOrder[k];
            if (oi >= 0 && oi < nObj) drawOne(objects[oi]);
        }
        // Anything registered AFTER the tree was built is not in it. Draw it
        // last, in its own group: last is the only safe place for an object
        // whose position the tree never saw.
        //
        // Against coveredCount() and NOT against sortOrder.size(), which is now
        // shorter than the object list whenever the subtree cull fires - taking
        // the difference for "new objects" would redraw whatever the cull just
        // rejected, at the front of the frame, every frame.
        const int covered = sortTree->coveredCount();
        if (covered < nObj) {
            groupQueueStart.push_back(static_cast<int32_t>(renderQueue.size()));
            for (int i = covered; i < nObj; ++i) drawOne(objects[i]);
        }
        groupQueueStart.push_back(static_cast<int32_t>(renderQueue.size()));
    } else {
        for (auto obj : objects) drawOne(obj);
    }
    lastFrameDrawnObjects   = drawnObjs;
    lastFrameDrawnTriangles = static_cast<int>(renderQueue.size());
    JET_PREP_SPLIT(lastFramePrepTransformUs, prepT);

    // 3) Global painter's sort. Three bands:
    //      0. noWriteZBuffer  — drawn first, so later geometry paints over
    //                           them (e.g. skyboxes).
    //      1. Normal          — main scene, back-to-front by effective Z.
    //      2. ignoreZBuffer   — drawn last, unconditionally on top.
    //
    // Within the normal band, the sort key is `avgZ - zBias * zBiasScale`.
    // Bigger zBias pulls a triangle toward the camera in the sort, so it
    // draws later than coplanar geometry without that bias. The scale has
    // to be large enough to beat within-triangle avgZ variation on
    // typical world-scale geometry (~hundreds of units) so decals reliably
    // win coplanar fights, but not so large that a biased decal draws
    // *over* geometry that's genuinely much closer (where avgZ differences
    // are thousands). Tune if the scale of the world changes significantly.
    constexpr int32_t zBiasScale = 256;
    // Bucket sort: O(N) vs O(N log N) for std::sort. Separates the three
    // draw bands with one linear pass, then counting-sorts the normal band
    // by depth in K=64 buckets (farther triangles first). Within a bucket
    // (~Z_RANGE/64 depth units) relative order is preserved (stable).
    //
    // The sort scatters 4-byte INDICES into renderOrder rather than moving
    // the RenderTri structs themselves — the queue entries stay where
    // push_back left them and rasterizeBand() draws via renderOrder. This
    // deletes what used to be a full second copy of the queue every frame.
    // PER-LEAF DEPTH SORT. The tree has already ordered the leaves against
    // each other and that order is exact, so all that is left is the old key,
    // run inside one leaf at a time. That is where zBias belongs and always
    // did: the coplanar stack of road, slab and kerb inside a single block.
    // Sorting a few dozen triangles at a time is CHEAPER than the 256-bucket
    // counting sort it replaces, because it never has to touch the whole queue
    // - no histogram, no prefix sums, no scatter.
    if (useTree && !sawBands) {
        const int N = static_cast<int>(renderQueue.size());
        renderOrder.resize(N);
        for (int i = 0; i < N; ++i) renderOrder[i] = i;
        for (size_t g = 0; g + 1 < groupQueueStart.size(); ++g) {
            const int32_t a = groupQueueStart[g], b = groupQueueStart[g + 1];
            if (b - a < 2) continue;
            // std::sort AND NOT std::stable_sort, with the queue index as the
            // tie-break so the result is stable anyway. libstdc++'s
            // stable_sort takes a temporary buffer from the heap on EVERY
            // call, and this calls it once per leaf - a hundred small mallocs
            // a frame on a console whose internal heap is the scarce resource.
            // Breaking ties on the index costs one compare and keeps the
            // emission order of equal-depth triangles, which is what the
            // stability was ever for.
            std::sort(renderOrder.begin() + a, renderOrder.begin() + b,
                      [this](int32_t l, int32_t r) {
                const RenderTri& tl = renderQueue[l];
                const RenderTri& tr = renderQueue[r];
                const int32_t kl = tl.sortZ - static_cast<int32_t>(tl.zBias) * 256;
                const int32_t kr = tr.sortZ - static_cast<int32_t>(tr.zBias) * 256;
                return (kl != kr) ? (kl > kr) : (l < r);
            });
        }
    } else
    {
        const int N = static_cast<int>(renderQueue.size());
        renderOrder.resize(N);
        if (N > 1) {
            const int32_t nearZ  = camera->nearPlane;
            const int32_t farZ   = camera->farPlane;
            const int32_t zRange = (farZ > nearZ) ? (farZ - nearZ) : 1;
            // PATCHED FOR arcade-os jet60: 64 -> 256.
            //
            // K buckets spread linearly across the whole near-to-far range, so at a
            // 183 m far plane K=64 is 2.9 m per bucket and everything inside one is
            // ordered by insertion, not depth. That is coarser than a building is
            // wide, which is why an overpass pillar standing against a facade could
            // not be ordered against it - the z-fighting that survived turning the
            // decals off.
            //
            // K=256 was TRIED AND REVERTED ONCE (see tools/patch_jet_sort.py): at 72 cm
            // a road chunk and the lane dash built into the same object separated, and
            // the road painted over the dash. That caller no longer exists here - the
            // lane markings and the helipad H are off on this branch, being the other
            // half of the same z-fighting - so the constraint that pinned K to 64 is
            // gone with them. The sort is O(N+K) and costs ~0.6 ms, so the extra
            // buckets are close to free.
            constexpr int K = 256;
            int counts[K] = {};
            int band0N = 0, band2N = 0;
            // Pass 1: classify bands; histogram the normal band by depth.
            for (const auto& t : renderQueue) {
                if (t.noWriteZBuffer) { band0N++; continue; }
                if (t.ignoreZBuffer)  { band2N++; continue; }
                const int32_t key = t.sortZ - static_cast<int32_t>(t.zBias) * zBiasScale;
                int b = static_cast<int>((static_cast<int64_t>(key - nearZ) * K) / zRange);
                if (b < 0) b = 0; else if (b >= K) b = K - 1;
                ++counts[b];
            }
            // Prefix sums — output bucket K-1 first (farthest first).
            int pos[K];
            pos[K - 1] = band0N;
            for (int i = K - 2; i >= 0; --i) pos[i] = pos[i + 1] + counts[i + 1];
            // Pass 2: scatter indices.
            int b0 = 0;
            int b2 = band0N + (N - band0N - band2N);
            for (int32_t i = 0; i < N; ++i) {
                const RenderTri& t = renderQueue[i];
                if (t.noWriteZBuffer) { renderOrder[b0++] = i; continue; }
                if (t.ignoreZBuffer)  { renderOrder[b2++] = i; continue; }
                const int32_t key = t.sortZ - static_cast<int32_t>(t.zBias) * zBiasScale;
                int b = static_cast<int>((static_cast<int64_t>(key - nearZ) * K) / zRange);
                if (b < 0) b = 0; else if (b >= K) b = K - 1;
                renderOrder[pos[b]++] = i;
            }
        } else if (N == 1) {
            renderOrder[0] = 0;
        }
    }
    JET_PREP_SPLIT(lastFramePrepSortUs, prepT);
}  // end prepareFrame()

void Scene::clearBand(int yMin, int yMax) {
    if (!renderer) return;
    renderer->yBandMin = yMin;
    renderer->yBandMax = yMax;
    clearBuffers();
}

void Scene::rasterizeBand(int yMin, int yMax, uint16_t* zBandBase) {
    // Create a thread-local copy of the rasteriser so each band worker has
    // its own yBandMin/yBandMax. Only framebuffer/zbuffer ptrs are shared;
    // writes go to non-overlapping y regions so there is no write race.
    Rasterizer bandRast = *renderer;
    bandRast.yBandMin = yMin;
    bandRast.yBandMax = yMax;
#if Z_BUFFERING
    if (zBandBase) bandRast.setZBuffer(zBandBase);
#endif

    // 4) Flush. Count triangles that actually entered the rasterizer
    // (drawTriangle returned true). Triangles dropped by per-tri checks
    // inside drawTriangle (alpha=0, zero-area, near/far Z, degenerate
    // denom) return false and don't count toward the rasterized total.
    int rasterized = 0;
    const bool haveSpans = (triYSpan.size() == renderQueue.size());
    for (const int32_t idx : renderOrder) {
        // PATCHED FOR arcade-os jet60: a 4-byte band reject in front of the
        // ~100-byte load below. Guarded on the span array being in step with
        // the queue, so a frame built without it still renders - just slower.
        if (haveSpans) {
            const int32_t sp = triYSpan[idx];
            const int32_t lo = (int32_t)(int16_t)(sp & 0xFFFF);
            const int32_t hi = (int32_t)(int16_t)((sp >> 16) & 0xFFFF);
            if (hi < yMin || lo >= yMax) continue;
        }
        const RenderTri& t = renderQueue[idx];
#if MAX_PICK_QUERIES > 0
        bandRast.currentPickObject        = t.sourceObject;
        bandRast.currentPickTriangleIndex = t.sourceTriangleIndex;
#endif
        // t.avgZ rides along as the FAST_Z depth hint: emitTri computed the
        // same three-vertex average and already culled it against near/far,
        // so drawTriangle skips both the recompute and the redundant test.
        if (bandRast.drawTriangle(t.v1, t.v2, t.v3, t.material,
                                   directionalLight, ambientLight,
                                   renderEvenLines,
                                   t.ignoreZBuffer, t.noWriteZBuffer,
                                   (int)t.zBias, t.objAlpha,
                                   t.brightnessPrecomputed, t.avgZ)) {
            ++rasterized;
        }
    }
    lastFrameRasterizedTriangles = rasterized;
}

void Scene::render() {
    if (!camera) return;
    prepareFrame();
    rasterizeBand(0, screenHeight);

    // Checkerboard reconstruction: fill in opposite-parity pixels from the
    // freshly-rendered current-parity neighbours so PostFX sees a fully-populated
    // buffer. Done as a separate pass after all triangles are drawn so that
    // painter's algorithm overdraw cannot cause repeated averaging.
    #if defined(CHECKERBOARD_RECONSTRUCTION) && CHECKERBOARD_RECONSTRUCTION
    if (renderer->checkerboardMode) {
        reconstructCheckerboard();
    }
    #endif

    #if POSTFX_ANTIALIASING
    postFX->applyFXAA(framebuffer);
    #endif

    #if POSTFX_BLOOM
    postFX->applyBloom(framebuffer);
    #endif

    #if POSTFX_CRT
    postFX->applyCRT(framebuffer);
    #endif

    #if POSTFX_PIXELATE
    postFX->applyPixelate(framebuffer);
    #endif

    #if POSTFX_CHROMATIC
    postFX->applyChromatic(framebuffer);
    #endif

    #if POSTFX_MOTION_BLUR
    postFX->applyMotionBlur(framebuffer);
    #endif

    drawSprites();

    frameCounter++;
}

void Scene::addSprite(Sprite2D* sprite) {
    if (sprite) sprites.push_back(sprite);
}

// ---------------------------------------------------------------------------
// drawSprites — called at end of render(), after PostFX, before frameCounter++
// ---------------------------------------------------------------------------
// Blends each registered Sprite2D onto the framebuffer in zOrder sequence.
// Two paths:
//   textured  — iterate source rows, colour-key skip, optional alpha blend
//   solid     — span fill with material->color, optional alpha blend
//
// Alpha compositing uses a fast 5-bit RGB565 lerp:
//   out = src + ((dst - src) * inv_alpha >> 5)
// which is exact at 0 and 255 and has ≤1 LSB error at mid values.
// ---------------------------------------------------------------------------
void Scene::drawSprites() {
    // On HALF_WIDTH_BUFFERS builds, sprites are composited at full output
    // resolution during Display::pushFrame() scanout instead — writing them
    // into the half-width framebuffer here would halve their horizontal
    // resolution and use the wrong stride.
#if HALF_WIDTH_BUFFERS
    return;
#else
    if (sprites.empty()) return;

    // Sort by zOrder each frame (list is typically tiny — insertion sort
    // territory, but std::stable_sort keeps it simple and correct).
    std::stable_sort(sprites.begin(), sprites.end(),
        [](const Sprite2D* a, const Sprite2D* b){ return a->zOrder < b->zOrder; });

    for (Sprite2D* sp : sprites) {
        if (!sp || !sp->enabled || !sp->material) continue;
        const uint8_t matAlpha = sp->material->alpha;
        // Combined alpha: 0 = invisible, 255 = opaque.
        const int combined = ((int)sp->alpha * (int)matAlpha) / 255;
        if (combined == 0) continue;

        const Texture* tex = sp->material->diffuseMap;
        const bool opaque  = (combined == 255);

        // Clip dest rect to framebuffer bounds.
        const int srcW = tex ? tex->width  : sp->width;
        const int srcH = tex ? tex->height : sp->height;
        if (srcW <= 0 || srcH <= 0) continue;
        const int dstW = srcW * sp->scale;
        const int dstH = srcH * sp->scale;

        const int x0 = sp->x, y0 = sp->y;
        const int x1 = x0 + dstW, y1 = y0 + dstH;
        // Clipped dest rect
        const int dx0 = (x0 < 0) ? 0 : x0;
        const int dy0 = (y0 < 0) ? 0 : y0;
        const int dx1 = (x1 > screenWidth)  ? screenWidth  : x1;
        const int dy1 = (y1 > screenHeight) ? screenHeight : y1;
        if (dx0 >= dx1 || dy0 >= dy1) continue;

        // Source start offsets for the unscaled (scale==1) path.
        const int sx0 = dx0 - x0;
        const int sy0 = dy0 - y0;

        if (tex && sp->scale > 1) {
            // ---- Nearest-neighbour upscale textured blit (scale > 1) ---------
            // fp8 step: advances source coord by (1/scale) per output pixel.
            const int xStep = (srcW << 8) / dstW;
            const int yStep = (srcH << 8) / dstH;
            const bool colorKey = tex->hasAlpha;
            const uint16_t keyColor = tex->alphaColor;
            const uint16_t* src = tex->data;
            int sf_y = (dy0 - y0) * yStep;
            for (int dy = dy0; dy < dy1; ++dy, sf_y += yStep) {
                const int sy  = sf_y >> 8;   // nearest-neighbour: integer texel
                uint16_t* dstRow = framebuffer + dy * screenWidth + dx0;
                int sf_x = (dx0 - x0) * xStep;
                const int w = dx1 - dx0;
                for (int i = 0; i < w; ++i, sf_x += xStep) {
                    const int sx  = sf_x >> 8;
                    const uint16_t s = src[sy * srcW + sx];
                    if (colorKey && s == keyColor) continue;
                    const int sr = (s >> 11) & 0x1F;
                    const int sg = (s >>  5) & 0x3F;
                    const int sb =  s        & 0x1F;
                    if (sp->blendMode == BlendMode::BLEND_ADD) {
                        const uint16_t d = dstRow[i];
                        int ar=((d>>11)&0x1F)+sr; if(ar>0x1F)ar=0x1F;
                        int ag=((d>>5) &0x3F)+sg; if(ag>0x3F)ag=0x3F;
                        int ab=(d      &0x1F)+sb; if(ab>0x1F)ab=0x1F;
                        dstRow[i]=(uint16_t)((ar<<11)|(ag<<5)|ab);
                    } else if (combined == 255) {
                        dstRow[i]=(uint16_t)((sr<<11)|(sg<<5)|sb);
                    } else {
                        const uint16_t d = dstRow[i];
                        const int inv=255-combined;
                        const int dr=(d>>11)&0x1F,dg=(d>>5)&0x3F,db=d&0x1F;
                        dstRow[i]=(uint16_t)(
                            (((sr*combined+dr*inv)/255)<<11)|
                            (((sg*combined+dg*inv)/255)<<5)|
                             ((sb*combined+db*inv)/255));
                    }
                }
            }
        } else if (tex) {
            // ---- Textured sprite blit ----------------------------------------
            const bool colorKey = tex->hasAlpha;
            const uint16_t keyColor = tex->alphaColor;
            const uint16_t* src = tex->data;

            if (sp->blendMode == BlendMode::BLEND_ADD) {
                // Saturating additive: dst = clamp(dst + src). No alpha scaling.
                for (int dy = dy0; dy < dy1; ++dy) {
                    const int sy = sy0 + (dy - dy0);
                    const uint16_t* srcRow = src + sy * tex->width + sx0;
                    uint16_t*       dstRow = framebuffer + dy * screenWidth + dx0;
                    const int w = dx1 - dx0;
                    for (int i = 0; i < w; ++i) {
                        const uint16_t s = srcRow[i];
                        if (colorKey && s == keyColor) continue;
                        const uint16_t d = dstRow[i];
                        const int sr = (s >> 11) & 0x1F;
                        const int sg = (s >>  5) & 0x3F;
                        const int sb =  s        & 0x1F;
                        int ar = ((d >> 11) & 0x1F) + sr; if (ar > 0x1F) ar = 0x1F;
                        int ag = ((d >>  5) & 0x3F) + sg; if (ag > 0x3F) ag = 0x3F;
                        int ab = ( d        & 0x1F) + sb; if (ab > 0x1F) ab = 0x1F;
                        dstRow[i] = (uint16_t)((ar << 11) | (ag << 5) | ab);
                    }
                }
            } else {
                for (int dy = dy0; dy < dy1; ++dy) {
                    const int sy = sy0 + (dy - dy0);
                    const uint16_t* srcRow = src + sy * tex->width + sx0;
                    uint16_t*       dstRow = framebuffer + dy * screenWidth + dx0;
                    const int w = dx1 - dx0;

                    if (opaque) {
                        if (colorKey) {
                            for (int i = 0; i < w; ++i) {
                                if (srcRow[i] != keyColor) dstRow[i] = srcRow[i];
                            }
                        } else {
                            for (int i = 0; i < w; ++i) dstRow[i] = srcRow[i];
                        }
                    } else {
                        // 5-bit lerp: decompose RGB565 into R5/G6/B5 channels,
                        // lerp each independently, repack.
                        const int inv = 255 - combined;
                        for (int i = 0; i < w; ++i) {
                            const uint16_t s = srcRow[i];
                            if (colorKey && s == keyColor) continue;
                            const uint16_t d = dstRow[i];
                            const int sr = (s >> 11) & 0x1F;
                            const int sg = (s >>  5) & 0x3F;
                            const int sb =  s        & 0x1F;
                            const int dr = (d >> 11) & 0x1F;
                            const int dg = (d >>  5) & 0x3F;
                            const int db =  d        & 0x1F;
                            const int or_ = (sr * combined + dr * inv) / 255;
                            const int og  = (sg * combined + dg * inv) / 255;
                            const int ob  = (sb * combined + db * inv) / 255;
                            dstRow[i] = (uint16_t)((or_ << 11) | (og << 5) | ob);
                        }
                    }
                }
            }
        } else {
            // ---- Solid rectangle fill ----------------------------------------
            const uint16_t col = sp->material->color;

            if (sp->blendMode == BlendMode::BLEND_ADD) {
                // Additive: add colour directly, no alpha scaling.
                const int acr = (col >> 11) & 0x1F;
                const int acg = (col >>  5) & 0x3F;
                const int acb =  col        & 0x1F;
                for (int dy = dy0; dy < dy1; ++dy) {
                    uint16_t* row = framebuffer + dy * screenWidth + dx0;
                    const int w = dx1 - dx0;
                    for (int i = 0; i < w; ++i) {
                        const uint16_t d = row[i];
                        int ar = ((d >> 11) & 0x1F) + acr; if (ar > 0x1F) ar = 0x1F;
                        int ag = ((d >>  5) & 0x3F) + acg; if (ag > 0x3F) ag = 0x3F;
                        int ab = ( d        & 0x1F) + acb; if (ab > 0x1F) ab = 0x1F;
                        row[i] = (uint16_t)((ar << 11) | (ag << 5) | ab);
                    }
                }
            } else {
                if (opaque) {
                    for (int dy = dy0; dy < dy1; ++dy) {
                        uint16_t* row = framebuffer + dy * screenWidth + dx0;
                        const int w = dx1 - dx0;
                        for (int i = 0; i < w; ++i) row[i] = col;
                    }
                } else {
                    const int inv = 255 - combined;
                    const int cr = (col >> 11) & 0x1F;
                    const int cg = (col >>  5) & 0x3F;
                    const int cb =  col        & 0x1F;
                    for (int dy = dy0; dy < dy1; ++dy) {
                        uint16_t* row = framebuffer + dy * screenWidth + dx0;
                        const int w = dx1 - dx0;
                        for (int i = 0; i < w; ++i) {
                            const uint16_t d = row[i];
                            const int dr = (d >> 11) & 0x1F;
                            const int dg = (d >>  5) & 0x3F;
                            const int db =  d        & 0x1F;
                            const int or_ = (cr * combined + dr * inv) / 255;
                            const int og  = (cg * combined + dg * inv) / 255;
                            const int ob  = (cb * combined + db * inv) / 255;
                            row[i] = (uint16_t)((or_ << 11) | (og << 5) | ob);
                        }
                    }
                }
            }
        }
    }
#endif // !HALF_WIDTH_BUFFERS
}

void Scene::getStatistics(int& objectCount, int& triangleCount, int& vertexCount) {
    objectCount = static_cast<int>(objects.size());
    triangleCount = 0;
    vertexCount = 0;

    for (const auto& obj : objects) {
        if (!obj->enabled) continue;
        triangleCount += static_cast<int>(obj->triangles.size());
        vertexCount += static_cast<int>(obj->vertices.size());
    }
}

void PERF_CRITICAL Scene::renderObject(Object* obj,
                                     int32_t camCosX, int32_t camSinX,
                                     int32_t camCosY, int32_t camSinY,
                                     int32_t camCosZ, int32_t camSinZ,
                                     uint8_t objAlpha,
                                     Object* meshSource) {
    // meshSource decouples "which mesh do we rasterise" from "where / how
    // does the object live in the world". Defaults to obj itself, so the
    // non-LOD path is unchanged. When the global LOD system picks a
    // reduced-detail mesh, that Object* is passed in here while obj keeps
    // ownership of the transform, flags, AABB and fade state.
    if (!meshSource) meshSource = obj;

    // Reusable scratch buffers — kept across calls so we don't pay for a
    // heap alloc per object per frame. Renderer is single-threaded
    // (one render task), so plain static is fine here. RenderVertex (not
    // Object::Vertex): only the fields the configured pipeline consumes
    // are carried, and the transform loop below writes every live field,
    // so no upfront copy of the source vertex array is needed at all.
    // PATCHED FOR arcade-os jet60: the object's own camera-space depth, used as
    // the painter key for convex meshes. Accumulated in the transform loop that
    // is walking every vertex anyway, so it costs an add per vertex.

    int64_t objZSum = 0;
    int32_t objZCount = 0;
    int32_t objXMin = INT32_MAX, objXMax = INT32_MIN;
    int32_t objYMin = INT32_MAX, objYMax = INT32_MIN;
    int32_t objZMin = INT32_MAX, objZMax = INT32_MIN;
    // PATCHED FOR arcade-os jet60: how many vertices this frame transformed.
    //
    // `xf` tracks neither drawn objects nor drawn triangles - obj 161/tri 557
    // cost 17.06 ms where obj 241/tri 763 cost 14.44 - because the loop below
    // runs over the WHOLE mesh of every admitted object, however few of its
    // triangles survive. So the quantity that sets prepareFrame is the one
    // number the log never had. Two adds a frame.
    lastFrameTransformedVertices += (int)meshSource->vertices.size();

    static std::vector<RenderVertex> transformedVertices;
    static std::vector<Vector3> camSpacePos;
    const size_t vertCount = meshSource->vertices.size();
    transformedVertices.resize(vertCount);
    // Parallel array of camera-space positions (pre-projection). Needed so
    // triangles straddling the near plane can be clipped geometrically —
    // otherwise a single vertex slipping behind the near plane would force
    // the whole triangle to be discarded, leaving a visible hole in the
    // world right under the camera.
    camSpacePos.resize(vertCount);

    Vector3 camPos(camera->position);
    #if FLOAT_CAMERA_ANGLES
    Vector3_f camRotF(camera->rotation);
    #endif
    Vector3 objPos(obj->position);

    float   fovFactor = camera->fovFactor;
    bool isBillboard = obj->isBillboard;
    CullingMode cullingMode = obj->cullingMode;
    bool ignoreZBuffer = obj->ignoreZBuffer;
    bool noWriteZBuffer = obj->noWriteZBuffer;

    // Hoist object rotation trig out of the per-vertex loop — these are
    // constant for every vertex on the object. Also short-circuit the entire
    // rotation block when the object has zero rotation (true for most static
    // scenery), saving 12 mul + 6 div + 9 add per vertex.
    const bool objHasRotation = !isBillboard &&
        (obj->rotation.x != 0 || obj->rotation.y != 0 || obj->rotation.z != 0);

    // Composed object rotation matrix (Rz * Ry * Rx, since the previous
    // per-vertex code applied X→Y→Z). Storing all 9 entries at
    // FIXED_POINT_SCALE scale lets the per-vertex transform collapse from
    // three cascaded rotations (12 muls + 12 div per vertex) into a single
    // 3×3 mat-vec (9 muls + 3 div), with the same again saved on the
    // normal when LIGHTING is on. The trig product entries are pre-divided
    // by FIXED_POINT_SCALE so the entire matrix lives at one consistent
    // scale.
    int32_t objM00=FIXED_POINT_SCALE, objM01=0, objM02=0;
    int32_t objM10=0, objM11=FIXED_POINT_SCALE, objM12=0;
    int32_t objM20=0, objM21=0, objM22=FIXED_POINT_SCALE;
    // Float counterparts pre-divided by FIXED_POINT_SCALE. Per-vertex mat-vecs
    // use hardware-FPU fmul-add instead of int64_t multiply+shift. Defaults
    // are identity; vertex loop guards on objHasRotation so they're never
    // consumed when false.
    float fObjM00=1.0f, fObjM01=0.0f, fObjM02=0.0f;
    float fObjM10=0.0f, fObjM11=1.0f, fObjM12=0.0f;
    float fObjM20=0.0f, fObjM21=0.0f, fObjM22=1.0f;
    if (objHasRotation) {
        const int32_t cx = lookupCosI(obj->rotation.x);
        const int32_t sx = lookupSinI(obj->rotation.x);
        const int32_t cy = lookupCosI(obj->rotation.y);
        const int32_t sy = lookupSinI(obj->rotation.y);
        const int32_t cz = lookupCosI(obj->rotation.z);
        const int32_t sz = lookupSinI(obj->rotation.z);
        // K = Ry * Rx (at FPS scale; trig-product entries divided once).
        const int32_t k00 = cy;
        const int32_t k01 = (int32_t)((int64_t)sy * sx / FIXED_POINT_SCALE);
        const int32_t k02 = (int32_t)((int64_t)sy * cx / FIXED_POINT_SCALE);
        const int32_t k10 = 0;
        const int32_t k11 = cx;
        const int32_t k12 = -sx;
        const int32_t k20 = -sy;
        const int32_t k21 = (int32_t)((int64_t)cy * sx / FIXED_POINT_SCALE);
        const int32_t k22 = (int32_t)((int64_t)cy * cx / FIXED_POINT_SCALE);
        // M = Rz * K (at FPS scale).
        objM00 = (int32_t)(((int64_t)cz * k00 - (int64_t)sz * k10) / FIXED_POINT_SCALE);
        objM01 = (int32_t)(((int64_t)cz * k01 - (int64_t)sz * k11) / FIXED_POINT_SCALE);
        objM02 = (int32_t)(((int64_t)cz * k02 - (int64_t)sz * k12) / FIXED_POINT_SCALE);
        objM10 = (int32_t)(((int64_t)sz * k00 + (int64_t)cz * k10) / FIXED_POINT_SCALE);
        objM11 = (int32_t)(((int64_t)sz * k01 + (int64_t)cz * k11) / FIXED_POINT_SCALE);
        objM12 = (int32_t)(((int64_t)sz * k02 + (int64_t)cz * k12) / FIXED_POINT_SCALE);
        objM20 = k20;
        objM21 = k21;
        objM22 = k22;
        fObjM00=(float)objM00/FIXED_POINT_SCALE; fObjM01=(float)objM01/FIXED_POINT_SCALE; fObjM02=(float)objM02/FIXED_POINT_SCALE;
        fObjM10=(float)objM10/FIXED_POINT_SCALE; fObjM11=(float)objM11/FIXED_POINT_SCALE; fObjM12=(float)objM12/FIXED_POINT_SCALE;
        fObjM20=(float)objM20/FIXED_POINT_SCALE; fObjM21=(float)objM21/FIXED_POINT_SCALE; fObjM22=(float)objM22/FIXED_POINT_SCALE;
    }


    // Combined object→camera transform, composed ONCE per object:
    //   p_cam = Cam · (Obj·p + objPos − camPos) = (Cam·Obj)·p + Cam·(objPos − camPos)
    // so the per-vertex work collapses from two 3×3 mat-vecs plus two
    // vector adds down to a single mat-vec with the translation folded
    // into the accumulate. fM is also the combined rotation for normals
    // (both factors are pure rotations; translation is excluded there).
    // For unrotated objects (most scenery) fM is just the camera matrix.
    float fM00, fM01, fM02, fM10, fM11, fM12, fM20, fM21, fM22;
    if (objHasRotation) {
        fM00 = fCamM00*fObjM00 + fCamM01*fObjM10 + fCamM02*fObjM20;
        fM01 = fCamM00*fObjM01 + fCamM01*fObjM11 + fCamM02*fObjM21;
        fM02 = fCamM00*fObjM02 + fCamM01*fObjM12 + fCamM02*fObjM22;
        fM10 = fCamM10*fObjM00 + fCamM11*fObjM10 + fCamM12*fObjM20;
        fM11 = fCamM10*fObjM01 + fCamM11*fObjM11 + fCamM12*fObjM21;
        fM12 = fCamM10*fObjM02 + fCamM11*fObjM12 + fCamM12*fObjM22;
        fM20 = fCamM20*fObjM00 + fCamM21*fObjM10 + fCamM22*fObjM20;
        fM21 = fCamM20*fObjM01 + fCamM21*fObjM11 + fCamM22*fObjM21;
        fM22 = fCamM20*fObjM02 + fCamM21*fObjM12 + fCamM22*fObjM22;
    } else {
        fM00 = fCamM00; fM01 = fCamM01; fM02 = fCamM02;
        fM10 = fCamM10; fM11 = fCamM11; fM12 = fCamM12;
        fM20 = fCamM20; fM21 = fCamM21; fM22 = fCamM22;
    }
    const float fDx = (float)(objPos.x - camPos.x);
    const float fDy = (float)(objPos.y - camPos.y);
    const float fDz = (float)(objPos.z - camPos.z);
    const float fTx = fCamM00*fDx + fCamM01*fDy + fCamM02*fDz;
    const float fTy = fCamM10*fDx + fCamM11*fDy + fCamM12*fDz;
    const float fTz = fCamM20*fDx + fCamM21*fDy + fCamM22*fDz;

#if LIGHTING
    // Object-local lighting precompute eligibility.
    //
    // The view-space normal transform exists only so that the rasterizer
    // can evaluate the view-facing specular term (`N.z < 0` in
    // jetShadeBrightness). For objects whose materials are ALL non-
    // specular (`material->specular == 0`), there is no consumer of the
    // view-space frame — diffuse Lambert is rotation-invariant — so we
    // can skip the per-vertex normal transform entirely and instead
    // transform the world-space light direction into the object's local
    // space ONCE per object, then dot it against the untransformed
    // vertex normal to get brightness. That brightness is cached on the
    // vertex and read straight back out by drawTriangle, which also
    // skips its own jetShadeBrightness call when the flag is set.
    //
    // On the ESP32 with FLAT shading + non-specular materials being the
    // common case (track, ground, parapets, hulls), this deletes 9 muls
    // + 3 shifts per vertex plus the dot/clamp/square cycle from the
    // renderer's per-triangle path — replaced by a single 9-mul + 3-shift
    // light-direction transform amortized over the whole object.
    bool objectLocalLight = false;
    Vector3 objLightDir = {0, 0, 0};
    uint16_t objLightIntensity = 0;
    uint8_t  objDiffuseCoef = 255;
    if (directionalLight && !meshSource->triangles.empty()) {
        bool allNonSpecular = true;
        // Per-triangle material walk. Cheap — a handful of byte loads
        // per face — and exits early on the first specular material we
        // hit so specular-heavy objects (vehicles with shiny paint)
        // skip the rest of the scan. If we ever start caching this on
        // the Object itself we can drop the walk entirely; for now the
        // overhead is well below the savings on qualifying objects.
        //
        // Also disqualifies PHONG materials: PHONG interpolates per-
        // pixel from the vertex normals against directionalLight->lightDir
        // (view-space) and there's no place to feed cached brightness
        // back in. FLAT and GOURAUD both consume a per-triangle/vertex
        // scalar brightness so they slot the cache in cleanly.
        for (const auto& tri : meshSource->triangles) {
            if (!tri.material) continue;
            if (tri.material->specular != 0 ||
                tri.material->shadingMode == ShadingMode::PHONG) {
                allNonSpecular = false;
                break;
            }
        }
        if (allNonSpecular) {
            objectLocalLight = true;
            objLightIntensity = directionalLight->intensity;
            // Diffuse coef varies per triangle; we use the first triangle's
            // material as a representative. If diffuse coefs were ever
            // mixed across an object's faces, GOURAUD-style midtones
            // would shift slightly compared to the renderer's path. In
            // practice diffuse is a per-shading-style constant on every
            // material in this project (255 default).
            const Material* m0 = meshSource->triangles[0].material;
            if (m0) objDiffuseCoef = m0->diffuse;

            // Transform worldLightDir into object-local space. With M
            // composed as Rz*Ry*Rx (the same axis order applied to
            // positions and normals above) and orthonormal, the inverse
            // is the transpose: rows of M^T are columns of M, so we
            // dot worldLightDir with the COLUMNS of objM. For identity
            // object rotation we just use worldLightDir directly.
            const Vector3& Lw = directionalLight->worldLightDir;
            if (objHasRotation) {
                objLightDir.assign(
                    (int32_t)(((int64_t)Lw.x * objM00 + (int64_t)Lw.y * objM10 + (int64_t)Lw.z * objM20) / FIXED_POINT_SCALE),
                    (int32_t)(((int64_t)Lw.x * objM01 + (int64_t)Lw.y * objM11 + (int64_t)Lw.z * objM21) / FIXED_POINT_SCALE),
                    (int32_t)(((int64_t)Lw.x * objM02 + (int64_t)Lw.y * objM12 + (int64_t)Lw.z * objM22) / FIXED_POINT_SCALE));
            } else {
                objLightDir = Lw;
            }
        }
    }
#endif

    // Transform vertices and normals. Reads from the authoring vertices
    // (meshSource->vertices), writes only the configured pipeline's live
    // fields into the slim RenderVertex scratch array.
    for (size_t vi = 0; vi < vertCount; ++vi) {
        const Object::Vertex& srcVert = meshSource->vertices[vi];
        RenderVertex& dst = transformedVertices[vi];
        Vector3 pos(srcVert.position);
#if LIGHTING
        Vector3 normal(srcVert.normal);
#endif

        // Y-axis (cylindrical) billboard: pre-rotate the vertex offset
        // by +camRotY around Y so the camera's Y rotation below cancels
        // it out exactly, leaving the offset's X axis mapped to
        // camera-right and Y axis to world-Y. The billboard's world-
        // space POSITION (objPos) goes through the normal camera
        // transform unchanged, so it occupies real 3D space and
        // distance-fades / sorts correctly; only its local orientation
        // is locked to face the camera in yaw. Pitch/roll still apply
        // through the camera transform so the billboard appears
        // tilted exactly as a vertical real object would.
        if (isBillboard) {
            pos.assign((pos.x * camCosY - pos.z * camSinY) / FIXED_POINT_SCALE,
                        pos.y,
                       (pos.x * camSinY + pos.z * camCosY) / FIXED_POINT_SCALE);
        }

        // Object rotation, world translation and camera rotation in ONE
        // 3×3 mat-vec with the translation folded into the accumulate:
        // p_cam = fM·p + fT (see the composition above). Billboards enter
        // here with their yaw pre-rotation already applied and fM equal to
        // the bare camera matrix, which reproduces the old pipeline order
        // exactly.
        {
            const float fpx = (float)pos.x, fpy = (float)pos.y, fpz = (float)pos.z;
            pos.assign(
                (int32_t)(fpx * fM00 + fpy * fM01 + fpz * fM02 + fTx),
                (int32_t)(fpx * fM10 + fpy * fM11 + fpz * fM12 + fTy),
                (int32_t)(fpx * fM20 + fpy * fM21 + fpz * fM22 + fTz));
        }
#if LIGHTING
        // Normals use the combined ROTATION only — no translation. The
        // object-local-light path skips the transform entirely and shades
        // from the untransformed mesh-local normal below.
        if (!objectLocalLight) {
            const float fnx = (float)normal.x, fny = (float)normal.y, fnz = (float)normal.z;
            normal.assign(
                (int32_t)(fnx * fM00 + fny * fM01 + fnz * fM02),
                (int32_t)(fnx * fM10 + fny * fM11 + fnz * fM12),
                (int32_t)(fnx * fM20 + fny * fM21 + fnz * fM22));
        }
#endif

        // Perspective projection — float fovFactor lets us use a reciprocal
        // multiply instead of 64-bit integer divide, leveraging the hardware
        // FPU on ESP32-S3/P4 (64-bit div is software-emulated on those cores).
        if (pos.z == 0) pos.z = 1; // avoid divide-by-zero
        // Record camera-space position (pre-projection) for near-plane clipping.
        camSpacePos[vi] = pos;
        objZSum += (int64_t)pos.z;
        ++objZCount;
        if (pos.x < objXMin) objXMin = pos.x;  if (pos.x > objXMax) objXMax = pos.x;
        if (pos.y < objYMin) objYMin = pos.y;  if (pos.y > objYMax) objYMax = pos.y;
        if (pos.z < objZMin) objZMin = pos.z;  if (pos.z > objZMax) objZMax = pos.z;
        const float invZ = fovFactor / (float)pos.z;
        dst.position.x = (int32_t)(pos.x * invZ) + screenWidth / 2;
        dst.position.y = screenHeight / 2 - (int32_t)(pos.y * invZ);
        dst.position.z = pos.z;

#if TEXTURE_MAPPING
        dst.uv = srcVert.uv;
#endif
#if LIGHTING
        // Store transformed normal (only consumed by the lit shading paths).
        dst.normal = normal;
        // Object-local-light path: precompute the per-vertex Lambert
        // brightness here using the mesh-local normal + object-local
        // light direction. drawTriangle reads this back and skips its
        // own jetShadeBrightness call. For FLAT triangles all three
        // verts of a face share the same normal (computeFlatNormals
        // stamps identical normals) so the per-triangle FLAT path can
        // pick any one — it uses v1 already.
        dst.lambertBrightness = objectLocalLight
            ? sceneLambertDiffuse(normal, objLightDir, objLightIntensity, objDiffuseCoef)
            : 0;
#endif
    }

    // ------------------------------------------------------------------
    // PATCHED FOR arcade-os: SORT A KEY, NOT THE MESH.
    //
    // This sort was 8.3 ms of a 27.4 ms transform, measured on the device by
    // toggling sortOwnTriangles live in Ion Drift (33.7 ms of prepareFrame
    // against 25.6 with it off, on a parked view of 814 drawn triangles). That
    // is not the sort being algorithmically wrong - it is where its comparator
    // reads from.
    //
    // The old comparator dereferenced transformedVertices SIX TIMES per
    // comparison, at three effectively random indices per triangle. On this
    // console that array is tens of kilobytes and lives in PSRAM, so every one
    // of those was a likely cache miss, and std::sort does n log n of them: for
    // one 360-triangle chunk that is ~3,000 comparisons and ~18,000 scattered
    // PSRAM reads, per object, per frame.
    //
    // So compute the depth key ONCE per triangle - one sequential pass, three
    // reads each, over vertices the transform loop above has only just
    // written - and sort eight-byte {key, index} records instead. The
    // comparator then touches nothing but the two records being compared, and
    // the sort moves 8 bytes where it used to move a 16-byte Triangle.
    //
    // AND IT NO LONGER PERMUTES THE MESH. The old sort reordered
    // meshSource->triangles in place, mutating shared geometry from inside
    // what reads as a read-only transform. The queue loop below walks
    // triOrder instead, so any index into a mesh's triangle list now means the
    // same thing from one frame to the next - which is also what makes
    // srcTriIdx a real triangle id for picking rather than this frame's slot.
    //
    // The ORDER IS THE SAME ORDER. Same key, same expression, same int32
    // arithmetic, same descending compare. The only difference is ties, which
    // std::sort left arbitrary and which now come out in mesh order - the
    // stable choice, and the one coplanar faces want, since the bucket sort
    // downstream is itself stable.
    // ------------------------------------------------------------------
    struct TriDepthKey { int32_t key; uint32_t idx; };
    // Reused across objects and frames, like transformedVertices above and
    // under the same assumption: one thread is inside prepareFrame() at a
    // time. Under the two-Scene split that holds - core 0 transforms, core 1
    // rasterises, and rasterizeBand() touches none of this.
    static std::vector<TriDepthKey> triOrder;
    triOrder.clear();
#if SORT_TRIANGLES
    // PATCHED IN THE VENDORED COPY (ion-drift tools/sync_to_arcade.ps1).
    if (meshSource->sortOwnTriangles) {
        const size_t triCount = meshSource->triangles.size();
        triOrder.resize(triCount);
        for (size_t i = 0; i < triCount; ++i) {
            const Object::Triangle& t = meshSource->triangles[i];
            triOrder[i].key = (transformedVertices[t.v1].position.z +
                               transformedVertices[t.v2].position.z +
                               transformedVertices[t.v3].position.z) / 3;
            triOrder[i].idx = (uint32_t)i;
        }
        std::sort(triOrder.begin(), triOrder.end(),
                  [](const TriDepthKey& a, const TriDepthKey& b) {
                      if (a.key != b.key) return a.key > b.key;
                      return a.idx < b.idx;
                  });
    }
#endif

    // ------------------------------------------------------------------
    // Near-plane clipping helpers
    // ------------------------------------------------------------------
    // Without these, a triangle with even one vertex behind the near plane
    // is discarded whole (projected x/y would be garbage for that vertex),
    // producing visible holes in geometry right under the camera. We clip
    // straddling triangles to z==nearPlane and project the resulting new
    // vertices fresh, with UVs/normals interpolated along each clipped edge.
    const int32_t nz = camera->nearPlane;

    auto lerpI32 = [](int32_t a, int32_t b, int32_t tFixed) -> int32_t {
        return a + (int32_t)(((int64_t)tFixed * (b - a)) / FIXED_POINT_SCALE);
    };

    // Given a camera-space position, produce a RenderVertex with screen-
    // space x/y, camera-space z kept in position.z, and the provided normal
    // and uv (both interpolated upstream in camera / texture space). The
    // normal/uv/brightness parameters are only stored when the configured
    // pipeline carries those fields.
    auto projectVertex = [&](const Vector3& cam,
                             const Vector3& normalCamSpace,
                             const Vector2& uv,
                             uint16_t lambertBrightness = 0) -> RenderVertex {
        RenderVertex v;
        const float invZ = (cam.z != 0) ? fovFactor / (float)cam.z : 0.0f;
        v.position.x = (int32_t)(cam.x * invZ) + screenWidth / 2;
        v.position.y = screenHeight / 2 - (int32_t)(cam.y * invZ);
        v.position.z = cam.z;
#if LIGHTING
        v.normal = normalCamSpace;
        v.lambertBrightness = lambertBrightness;
#else
        (void)normalCamSpace; (void)lambertBrightness;
#endif
#if TEXTURE_MAPPING
        v.uv = uv;
#else
        (void)uv;
#endif
        return v;
    };

    // Clip edge from A (behind near plane) → B (in front). Returns vertex on
    // the near plane with all attributes interpolated.
    auto clipEdge = [&](const RenderVertex& A, const RenderVertex& B,
                        const Vector3& camA, const Vector3& camB) -> RenderVertex {
        int32_t dz = camB.z - camA.z;
        if (dz == 0) dz = 1;
        int32_t t = (int32_t)(((int64_t)(nz - camA.z) * FIXED_POINT_SCALE) / dz);
        Vector3 camNew(lerpI32(camA.x, camB.x, t),
                       lerpI32(camA.y, camB.y, t),
                       nz);
#if LIGHTING
        Vector3 n(lerpI32(A.normal.x, B.normal.x, t),
                  lerpI32(A.normal.y, B.normal.y, t),
                  lerpI32(A.normal.z, B.normal.z, t));
        // Interpolate the precomputed Lambert brightness along the clipped
        // edge too. Without this, clipped vertices land in projectVertex
        // with the default lambertBrightness=0 and the brightnessPrecomputed
        // path downstream reads pure black — for FLAT that turns the whole
        // face black whenever v1 happens to be the clipped vertex; for
        // GOURAUD it gradient-blends toward black at the clip seam,
        // producing the dark wedge artefacts visible on triangles whose
        // vertices straddle the near plane.
        const uint16_t lb = (uint16_t)(A.lambertBrightness +
            (int32_t)(((int64_t)t * (int32_t)(B.lambertBrightness - A.lambertBrightness)) / FIXED_POINT_SCALE));
#else
        Vector3 n(0, 0, 0);
        const uint16_t lb = 0;
#endif
#if TEXTURE_MAPPING
        Vector2 uv(lerpI32(A.uv.x, B.uv.x, t),
                   lerpI32(A.uv.y, B.uv.y, t));
#else
        Vector2 uv(0, 0);
#endif
        return projectVertex(camNew, n, uv, lb);
    };

    // Emit a projected triangle into renderQueue (does screen-bounds + backface
    // cull). Reused by both the fast path and the clipped path.
    auto emitTri = [&](const RenderVertex& a,
                       const RenderVertex& b,
                       const RenderVertex& c,
                       Material* mat
#if MAX_PICK_QUERIES > 0
                       , int32_t srcTriIdx
#endif
                       ) {
        // Per-triangle near/far cull on the average camera-space Z.
        // Object cull rejects entirely-outside boxes; this catches the
        // remaining far-plane tris on objects that straddle it (large
        // ground tiles, big cliff faces). We do this BEFORE the off-
        // screen XY tests + shoelace + queue-push so a doomed tri pays
        // none of those costs (and never enters the painter's-sort).
        // depthFogFar == farPlane in the current build, so this also
        // subsumes the depth-fog alpha=0 early-out that drawTriangle
        // would have done after a full setup.
        const int32_t avgZ = (a.position.z + b.position.z + c.position.z) / 3;
        if (avgZ > camera->farPlane || avgZ < camera->nearPlane) return;

        if (a.position.x < 0 && b.position.x < 0 && c.position.x < 0) return;
        if (a.position.x > screenWidth && b.position.x > screenWidth && c.position.x > screenWidth) return;
        if (a.position.y < 0 && b.position.y < 0 && c.position.y < 0) return;
        if (a.position.y > screenHeight && b.position.y > screenHeight && c.position.y > screenHeight) return;

        // 64-bit shoelace: projected coords from a near-plane-clipped vertex
        // can be tens of thousands of units, which would overflow a 32-bit
        // signed product and accidentally flip the backface-cull sign. That
        // was causing large floor triangles near the camera to vanish
        // (typically in the bottom-left quadrant where cam.x/cam.y are most
        // negative).
        int64_t shoelaceArea = (int64_t)a.position.x * (b.position.y - c.position.y) +
                               (int64_t)b.position.x * (c.position.y - a.position.y) +
                               (int64_t)c.position.x * (a.position.y - b.position.y);

        bool shouldCull = false;
        switch (cullingMode) {
            case CullingMode::CULL_BACKFACES: shouldCull = (shoelaceArea <= 0); break;
            case CullingMode::CULL_FRONTFACES: shouldCull = (shoelaceArea >= 0); break;
            case CullingMode::NO_CULLING: break;
        }
        if (shouldCull) return;

        RenderTri rt;
        if (cullingMode == CullingMode::NO_CULLING && shoelaceArea < 0) {
            rt.v1 = c; rt.v2 = b; rt.v3 = a;
        } else {
            rt.v1 = a; rt.v2 = b; rt.v3 = c;
        }
        rt.material       = mat;
        rt.ignoreZBuffer  = ignoreZBuffer;
        rt.noWriteZBuffer = noWriteZBuffer;
        rt.zBias          = obj->zBias;
        rt.objAlpha       = objAlpha;
        rt.avgZ           = avgZ;
        // ONE KEY PER OBJECT, BUT ONLY WHEN THE OBJECT IS COMPACT IN DEPTH.
        //
        // A convex mesh under backface culling cannot have two visible faces
        // overlap, so its triangles may share a key - but a key describes a
        // point, and a LONG object is not near a point. A kerb running the
        // length of a block took its midpoint depth and drew over a taxi
        // standing at its far end; the overpass deck had the same shape of
        // problem before it was chunked.
        //
        // The test is the object's own camera-space extents: use the object
        // key only while it is no deeper than it is wide or tall. That is
        // scene-independent and it follows the CAMERA - a kerb seen across is
        // compact in depth and sorts as one object, the same kerb seen along
        // its length spans depth and falls back to per-triangle, which is
        // exactly when each is right.
        const int32_t objZExt = (objZCount > 0) ? (objZMax - objZMin) : 0;
        const int32_t objXExt = (objZCount > 0) ? (objXMax - objXMin) : 0;
        const int32_t objYExt = (objZCount > 0) ? (objYMax - objYMin) : 0;
        const bool    objCompact = objZCount > 0 &&
                                   objZExt <= (objXExt > objYExt ? objXExt : objYExt);
        rt.sortZ      = (!meshSource->sortOwnTriangles && objCompact)
                      ? (int32_t)(objZSum / objZCount)
                      : avgZ;
#if LIGHTING
        rt.brightnessPrecomputed = objectLocalLight;
#endif
#if MAX_PICK_QUERIES > 0
        rt.sourceObject        = obj;
        rt.sourceTriangleIndex = srcTriIdx;
#endif
        renderQueue.push_back(rt);
        // PATCHED FOR arcade-os jet60: record this triangle's screen y span so
        // rasterizeBand() can reject it per band without loading its vertices.
        {
            const int32_t ya = rt.v1.position.y, yb = rt.v2.position.y, yc = rt.v3.position.y;
            int32_t lo = ya < yb ? ya : yb; if (yc < lo) lo = yc;
            int32_t hi = ya > yb ? ya : yb; if (yc > hi) hi = yc;
            if (lo < -32768) lo = -32768; else if (lo > 32767) lo = 32767;
            if (hi < -32768) hi = -32768; else if (hi > 32767) hi = 32767;
            triYSpan.push_back((int32_t)((uint32_t)(uint16_t)(int16_t)lo
                                       | ((uint32_t)(uint16_t)(int16_t)hi << 16)));
        }
    };

    // Render triangles with backface culling and shading.
    // triOrder is the depth order built above, or empty when this object opted
    // out of sorting its own triangles - in which case mesh order stands, as
    // it always did.
    const size_t triTotal = meshSource->triangles.size();
    for (size_t k = 0; k < triTotal; ++k) {
        const size_t triIdx = triOrder.empty() ? k : (size_t)triOrder[k].idx;
        const auto& triangle = meshSource->triangles[triIdx];
        const auto& vA = transformedVertices[triangle.v1];
        const auto& vB = transformedVertices[triangle.v2];
        const auto& vC = transformedVertices[triangle.v3];
        const Vector3& cA = camSpacePos[triangle.v1];
        const Vector3& cB = camSpacePos[triangle.v2];
        const Vector3& cC = camSpacePos[triangle.v3];

        // Classify each vertex against the near plane.
        const int outMask = (cA.z < nz ? 1 : 0)
                          | (cB.z < nz ? 2 : 0)
                          | (cC.z < nz ? 4 : 0);

        if (outMask == 7) continue;               // fully behind near plane

#if MAX_PICK_QUERIES > 0
        const int32_t srcTriIdx = (int32_t)triIdx;
        #define JET_EMIT_TRI(A, B, C, M)  emitTri((A), (B), (C), (M), srcTriIdx)
#else
        #define JET_EMIT_TRI(A, B, C, M)  emitTri((A), (B), (C), (M))
#endif

        if (outMask == 0) {                       // fast path: fully in front
            Material* effectiveMat = triangle.material;
            if (triangle.colorBaked) {
                s_bakedMat.color = triangle.bakedColor;
                effectiveMat = &s_bakedMat;
            }
            JET_EMIT_TRI(vA, vB, vC, effectiveMat);
            continue;
        }

        // Straddling near plane — produce a clipped polygon (3 or 4 verts)
        // while preserving winding order of the original triangle.
        const RenderVertex* vs[3]  = { &vA, &vB, &vC };
        const Vector3*        cvs[3] = { &cA, &cB, &cC };
        const bool in[3] = { (outMask & 1) == 0,
                             (outMask & 2) == 0,
                             (outMask & 4) == 0 };

        RenderVertex poly[4];
        int polyN = 0;
        for (int i = 0; i < 3; ++i) {
            const int j = (i + 1) % 3;
            if (in[i]) poly[polyN++] = *vs[i];
            if (in[i] != in[j]) {
                // One endpoint in, one out — add the near-plane intersection.
                if (in[i])
                    poly[polyN++] = clipEdge(*vs[j], *vs[i], *cvs[j], *cvs[i]);
                else
                    poly[polyN++] = clipEdge(*vs[i], *vs[j], *cvs[i], *cvs[j]);
            }
        }

        if (polyN >= 3) {
            Material* effectiveMat = triangle.material;
            if (triangle.colorBaked) { s_bakedMat.color = triangle.bakedColor; effectiveMat = &s_bakedMat; }
            JET_EMIT_TRI(poly[0], poly[1], poly[2], effectiveMat);
        }
        if (polyN == 4) JET_EMIT_TRI(poly[0], poly[2], poly[3], triangle.colorBaked ? &s_bakedMat : triangle.material);
        #undef JET_EMIT_TRI
    }
}

} // namespace Renderer
