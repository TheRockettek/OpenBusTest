# Main-view motion data

OpenBus uses a custom OpenGL 3.3 core renderer. Its model shaders already receive the active projection and model-view matrices, so main-view motion is emitted as additional fragment outputs from the existing model/material geometry submissions. The main view is rendered into an MRT framebuffer and its color attachment is blitted to the window; reflections continue using their existing independent targets and are not motion-instrumented.

## Accessing the frame

After a main-world render has finished, rendering integrations can query the GPU textures and the frame metadata:

```cpp
const openbus::rendering::MotionDataResources data =
    openbus::rendering::motionDataResources();
if (data.available) {
    // Bind data.motionVectors, data.validity, and data.depth directly on the GPU.
    // Check data.frame.historyValid before using temporal history.
}
```

The texture names are OpenGL texture handles owned by the renderer. They remain valid until the target is resized or the core renderer shuts down. Do not delete them or read them back to the CPU. `data.frame` describes the rendered frame: frame ID, monotonic render timestamp, delta between rendered frames, dimensions, current/previous view and projection matrices, and camera-history validity.

## Buffer contract

| Resource | Format | Meaning |
| --- | --- | --- |
| `motionVectors` | `GL_RG16F` | Screen displacement from the current pixel to the corresponding previous-frame pixel, in render-target pixels per rendered frame. |
| `validity` | `GL_R8` | `1` means the model fragment has usable transform history; `0` means consumers must not reuse it. This is not a disocclusion mask. |
| `depth` | `GL_DEPTH_COMPONENT24` | Current-frame OpenGL window depth in the usual `[0, 1]` projected depth representation, not linear view-space distance. |

Motion X increases to the right and Y increases upward, matching the OpenGL lower-left raster origin. For a current fragment, the vector is `previousPixel - currentPixel`. It is a displacement, not a velocity divided by time. `frame.width` and `frame.height` identify the motion texture's render resolution; `frame.displayWidth` and `frame.displayHeight` identify the output viewport. At sub-native render scale, multiply vector X and Y by the corresponding display-to-render dimension ratios to express displacement in display pixels. Consumer adapters can convert units, sign, origin, or jitter convention as required by a particular SDK. OpenBus currently renders without temporal projection jitter.

At native scale, the current color image is copied to the output with one framebuffer blit. At lower render scales, color is linearly upscaled to the output viewport before the native-resolution interface is drawn. Motion, validity, and depth remain at render-target resolution. No GPU-to-CPU readback or additional scene geometry draw is used. On an incomplete motion framebuffer, OpenBus logs an error and continues rendering to the window; `available` remains false.

## History and coverage

History advances when the main rendered view completes, not on simulation updates. Object history is keyed independently from simulation state; vehicle and human actor identities are stable for their lifetimes, waiting passengers use their stop and arrival order, and seated passengers use their vehicle and seat. Map batches use stable render-object keys. Objects absent from a rendered frame are removed from the next history set, so a newly created or reappearing object starts invalid. Static map geometry retains its prior rendered transform so camera motion is represented as well.

History is invalid on the first frame, a render-target or display-size change, a non-consecutive frame, a camera translation over 100 m, or a camera direction change over 60 degrees. The following completed frame can establish fresh history. Per-pixel validity is also zero for unsupported animated/skinned vertex deformation, blended geometry, and fragment paths without motion support. This initial coverage does not include reflections, secondary views, UI, or a disocclusion test. Unsupported shader effects write invalid data rather than fabricating motion.

## Diagnostics

Set `OPENBUS_MOTION_DEBUG=vectors` to show signed direction and magnitude for valid pixels, or `OPENBUS_MOTION_DEBUG=magnitude` to show a magnitude heatmap. Invalid pixels are magenta in either view. With the mode unset, the normal rendered image is unchanged. The motion, validity, and depth textures can also be inspected directly in RenderDoc.

## Cost, benchmark, and known limits

The target allocates RGBA8 scene color, RG16F motion, R8 validity, and 24-bit depth: approximately 13 bytes per pixel before driver allocation overhead (about 25.7 MiB at 1920x1080). The color blit adds one color read and write per rendered pixel; geometry submissions are reused.

The opt-in microbenchmark can be run with `build-ninja/OpenBusMotionDataGpuProbe.exe --benchmark`. It compares direct rendering with the motion MRT path at 1280x720, using the same single-triangle draw, 10 warmup frames, and 120 timed frames. On an NVIDIA GeForce RTX 3080 Ti (OpenGL 3.3.0, NVIDIA 621.14), one run reported:

| Metric | Direct rendering | Motion MRT | Difference |
| --- | ---: | ---: | ---: |
| GPU time per frame | 0.0144 ms | 0.0536 ms | +0.0392 ms |
| CPU time per frame | 0.0155 ms | 0.0542 ms | +0.0387 ms |
| Motion render-target allocation | — | 11.43 MiB | — |

The CPU measurement includes submission and `glFinish`; the GPU query covers geometry, MRT clears, and the scene-color blit. The probe also reports the same motion-MRT workload at 50% render scale, including its color upscale and estimated target allocation. In one synthetic run, native scale measured 0.0587 ms/frame and half scale 0.0362 ms/frame on an NVIDIA GeForce RTX 3080 Ti; the MRT target estimate changed from 11.43 MiB to 2.86 MiB. These single-triangle figures are not representative full-scene performance data. Results vary by driver and workload. At native resolution the color blit reads and writes about 7.03 MiB per frame. The runtime adds no scene draw calls and performs no GPU-to-CPU readback.

For focused correctness checks, `OpenBusMotionDataProbe` covers camera/object transform history, frame timing, visibility loss, resolution changes, camera teleports, and history recovery. `OpenBusMotionDataGpuProbe` uses a live OpenGL context to check framebuffer outputs and pixel displacement for stationary geometry, object translation, and camera translation.

The current validity channel checks transform/deformation history only; temporal consumers that need disocclusion rejection must compare reprojected depth or add a dedicated disocclusion stage. Object IDs are internal history keys and are not currently exported as a texture.
