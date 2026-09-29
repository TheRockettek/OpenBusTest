#include "CoreRenderer.h"

#include "Logger.h"
#include "OpenGLFunctions.h"
#include "PerfTrace.h"

#include <algorithm>
#include <array>
#include <limits>
#include <string>

extern Logger gameLog;

namespace openbus::rendering {
namespace {

constexpr GLsizei MODEL_VERTEX_STRIDE = static_cast<GLsizei>(9 * sizeof(float));

struct Uniforms {
    GLint projection = -1;
    GLint modelView = -1;
    GLint texture = -1;
    GLint textureArray = -1;
    GLint useTexture = -1;
    GLint useTextureArray = -1;
    GLint color = -1;
    GLint alphaMode = -1;
    GLint lightmap = -1;
    GLint nightmap = -1;
    GLint transmap = -1;
    GLint freeTexture = -1;
    GLint useLightmap = -1;
    GLint useNightmap = -1;
    GLint useTransmap = -1;
    GLint useFreeTexture = -1;
    GLint lightmapStrength = -1;
    GLint nightmapStrength = -1;
    GLint texcoordOffset = -1;
    GLint flipTextureY = -1;
    GLint environment = -1;
    GLint environmentAlpha = -1;
};

struct ModelUniformState {
    bool valid = false;
    bool textured = false;
    bool textureArray = false;
    bool useLightmap = false;
    bool useNightmap = false;
    bool useTransmap = false;
    bool useFreeTexture = false;
    bool flipTextureY = false;
    int alphaMode = 0;
    float lightmapStrength = 0.0f;
    float nightmapStrength = 0.0f;
    float texcoordOffsetX = 0.0f;
    float texcoordOffsetY = 0.0f;
    std::array<float, 4> color = {};
};

GLuint modelProgram = 0;
GLuint environmentProgram = 0;
GLuint primitiveProgram = 0;
GLuint modelVertexArray = 0;
GLuint primitiveVertexArray = 0;
GLuint primitiveBuffer = 0;
Uniforms modelUniforms;
Uniforms environmentUniforms;
Uniforms primitiveUniforms;
GLuint currentProgram = 0;
GLuint currentVertexArray = 0;
GLuint currentArrayBuffer = 0;
GLuint boundTexture2D = 0;
GLuint boundTextureArray = 0;
GLuint matrixProgram = 0;
Matrix4 cachedProjection = {};
Matrix4 cachedModelView = {};
ModelUniformState modelUniformState;

const char* modelVertexShader = R"GLSL(
#version 330 core
layout(location = 0) in vec3 aPosition;
layout(location = 1) in vec3 aTexCoord;
layout(location = 2) in vec3 aNormal;
uniform mat4 uProjection;
uniform mat4 uModelView;
uniform vec2 uTexcoordOffset;
uniform bool uFlipTextureY;
out vec3 vTexCoord;
out vec3 vNormal;
out vec3 vViewPosition;
void main() {
    vec4 viewPosition = uModelView * vec4(aPosition, 1.0);
    float textureY = uFlipTextureY ? 1.0 - aTexCoord.y : aTexCoord.y;
    vTexCoord = vec3(aTexCoord.x + uTexcoordOffset.x, textureY + uTexcoordOffset.y,
                     aTexCoord.z);
    vNormal = mat3(uModelView) * aNormal;
    vViewPosition = viewPosition.xyz;
    gl_Position = uProjection * viewPosition;
}
)GLSL";

const char* modelFragmentShader = R"GLSL(
#version 330 core
in vec3 vTexCoord;
in vec3 vNormal;
in vec3 vViewPosition;
uniform sampler2D uTexture;
uniform sampler2DArray uTextureArray;
uniform sampler2D uLightmap;
uniform sampler2D uNightmap;
uniform sampler2D uTransmap;
uniform sampler2D uFreeTexture;
uniform bool uUseTexture;
uniform bool uUseTextureArray;
uniform bool uUseLightmap;
uniform bool uUseNightmap;
uniform bool uUseTransmap;
uniform bool uUseFreeTexture;
uniform float uLightmapStrength;
uniform float uNightmapStrength;
uniform vec4 uColor;
uniform int uAlphaMode;
out vec4 fragmentColor;
void main() {
    vec4 color = uColor;
    if (uUseFreeTexture) {
        color *= texture(uFreeTexture, vTexCoord.xy);
    } else if (uUseTextureArray) {
        color *= texture(uTextureArray, vTexCoord);
    } else if (uUseTexture) {
        color *= texture(uTexture, vTexCoord.xy);
    }
    if (uUseLightmap) {
        vec4 lightmap = texture(uLightmap, vTexCoord.xy);
        color.rgb = mix(color.rgb, color.rgb * lightmap.rgb, uLightmapStrength);
    }
    if (uUseNightmap) {
        vec4 nightmap = texture(uNightmap, vTexCoord.xy);
        color.rgb = mix(color.rgb, color.rgb * nightmap.rgb, uNightmapStrength);
    }
    if (uUseTransmap) {
        color.a *= texture(uTransmap, vTexCoord.xy).r;
    }
    if (uAlphaMode == 1 && color.a < 0.5) {
        discard;
    }
    fragmentColor = color;
}
)GLSL";

const char* environmentFragmentShader = R"GLSL(
#version 330 core
in vec3 vNormal;
in vec3 vViewPosition;
uniform sampler2D uEnvironment;
uniform float uEnvironmentAlpha;
out vec4 fragmentColor;
void main() {
    vec3 normal = normalize(vNormal);
    vec3 viewDirection = normalize(-vViewPosition);
    vec3 reflection = reflect(-viewDirection, normal);
    float denominator = 2.0 * sqrt(max(reflection.z + 1.0, 0.0001));
    vec2 coordinates = reflection.xy / denominator + vec2(0.5);
    fragmentColor = vec4(texture(uEnvironment, coordinates).rgb, uEnvironmentAlpha);
}
)GLSL";

const char* primitiveVertexShader = R"GLSL(
#version 330 core
layout(location = 0) in vec3 aPosition;
layout(location = 1) in vec3 aColor;
uniform mat4 uProjection;
uniform mat4 uModelView;
out vec3 vColor;
void main() {
    vColor = aColor;
    gl_Position = uProjection * uModelView * vec4(aPosition, 1.0);
}
)GLSL";

const char* primitiveFragmentShader = R"GLSL(
#version 330 core
in vec3 vColor;
out vec4 fragmentColor;
void main() {
    fragmentColor = vec4(vColor, 1.0);
}
)GLSL";

GLuint compileShader(GLenum type, const char* source) {
    const GLuint shader = pglCreateShader(type);
    pglShaderSource(shader, 1, &source, nullptr);
    pglCompileShader(shader);
    GLint compiled = GL_FALSE;
    pglGetShaderiv(shader, GL_COMPILE_STATUS, &compiled);
    if (compiled == GL_TRUE) {
        return shader;
    }
    GLint logLength = 0;
    pglGetShaderiv(shader, GL_INFO_LOG_LENGTH, &logLength);
    std::string log(static_cast<std::size_t>(std::max(logLength, 1)), '\0');
    GLsizei written = 0;
    pglGetShaderInfoLog(shader, static_cast<GLsizei>(log.size()), &written, log.data());
    gameLog.Log("OpenGL shader compilation failed: " + log);
    pglDeleteShader(shader);
    return 0;
}

GLuint createProgram(const char* vertexSource, const char* fragmentSource) {
    const GLuint vertexShader = compileShader(GL_VERTEX_SHADER, vertexSource);
    const GLuint fragmentShader = compileShader(GL_FRAGMENT_SHADER, fragmentSource);
    if (vertexShader == 0 || fragmentShader == 0) {
        if (vertexShader != 0) {
            pglDeleteShader(vertexShader);
        }
        if (fragmentShader != 0) {
            pglDeleteShader(fragmentShader);
        }
        return 0;
    }
    const GLuint program = pglCreateProgram();
    pglAttachShader(program, vertexShader);
    pglAttachShader(program, fragmentShader);
    pglLinkProgram(program);
    pglDeleteShader(vertexShader);
    pglDeleteShader(fragmentShader);
    GLint linked = GL_FALSE;
    pglGetProgramiv(program, GL_LINK_STATUS, &linked);
    if (linked == GL_TRUE) {
        return program;
    }
    GLint logLength = 0;
    pglGetProgramiv(program, GL_INFO_LOG_LENGTH, &logLength);
    std::string log(static_cast<std::size_t>(std::max(logLength, 1)), '\0');
    GLsizei written = 0;
    pglGetProgramInfoLog(program, static_cast<GLsizei>(log.size()), &written, log.data());
    gameLog.Log("OpenGL shader link failed: " + log);
    pglDeleteProgram(program);
    return 0;
}

Uniforms modelUniformsFor(GLuint program, bool environment) {
    Uniforms uniforms;
    uniforms.projection = pglGetUniformLocation(program, "uProjection");
    uniforms.modelView = pglGetUniformLocation(program, "uModelView");
    if (environment) {
        uniforms.environment = pglGetUniformLocation(program, "uEnvironment");
        uniforms.environmentAlpha = pglGetUniformLocation(program, "uEnvironmentAlpha");
    } else {
        uniforms.texture = pglGetUniformLocation(program, "uTexture");
        uniforms.textureArray = pglGetUniformLocation(program, "uTextureArray");
        uniforms.useTexture = pglGetUniformLocation(program, "uUseTexture");
        uniforms.useTextureArray = pglGetUniformLocation(program, "uUseTextureArray");
        uniforms.color = pglGetUniformLocation(program, "uColor");
        uniforms.alphaMode = pglGetUniformLocation(program, "uAlphaMode");
        uniforms.lightmap = pglGetUniformLocation(program, "uLightmap");
        uniforms.nightmap = pglGetUniformLocation(program, "uNightmap");
        uniforms.transmap = pglGetUniformLocation(program, "uTransmap");
        uniforms.freeTexture = pglGetUniformLocation(program, "uFreeTexture");
        uniforms.useLightmap = pglGetUniformLocation(program, "uUseLightmap");
        uniforms.useNightmap = pglGetUniformLocation(program, "uUseNightmap");
        uniforms.useTransmap = pglGetUniformLocation(program, "uUseTransmap");
        uniforms.useFreeTexture = pglGetUniformLocation(program, "uUseFreeTexture");
        uniforms.lightmapStrength = pglGetUniformLocation(program, "uLightmapStrength");
        uniforms.nightmapStrength = pglGetUniformLocation(program, "uNightmapStrength");
        uniforms.texcoordOffset = pglGetUniformLocation(program, "uTexcoordOffset");
        uniforms.flipTextureY = pglGetUniformLocation(program, "uFlipTextureY");
    }
    return uniforms;
}

void uploadMatrices(const Uniforms& uniforms) {
    const Matrix4& projection = projectionMatrix();
    const Matrix4& modelView = modelViewMatrix();
    const bool projectionChanged =
        matrixProgram != currentProgram || cachedProjection != projection;
    const bool modelViewChanged = matrixProgram != currentProgram || cachedModelView != modelView;
    if (!projectionChanged && !modelViewChanged) {
        return;
    }
    std::array<GLfloat, 16> projectionFloat = {};
    std::array<GLfloat, 16> modelViewFloat = {};
    for (std::size_t index = 0; index < projection.size(); ++index) {
        projectionFloat[index] = static_cast<GLfloat>(projection[index]);
        modelViewFloat[index] = static_cast<GLfloat>(modelView[index]);
    }
    if (projectionChanged) {
        pglUniformMatrix4fv(uniforms.projection, 1, GL_FALSE, projectionFloat.data());
        cachedProjection = projection;
    }
    if (modelViewChanged) {
        pglUniformMatrix4fv(uniforms.modelView, 1, GL_FALSE, modelViewFloat.data());
        cachedModelView = modelView;
    }
    matrixProgram = currentProgram;
}

void bindModelVertexBuffer(GLuint buffer, GLuint vertexArray) {
    bool vertexArrayChanged = false;
    if (currentVertexArray != vertexArray) {
        pglBindVertexArray(vertexArray);
        currentVertexArray = vertexArray;
        vertexArrayChanged = true;
    }
    if (!vertexArrayChanged && currentArrayBuffer == buffer) {
        return;
    }
    if (currentArrayBuffer != buffer) {
        pglBindBuffer(GL_ARRAY_BUFFER, buffer);
        currentArrayBuffer = buffer;
    }
    pglVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, MODEL_VERTEX_STRIDE, nullptr);
    pglVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, MODEL_VERTEX_STRIDE,
                           reinterpret_cast<const void*>(3 * sizeof(float)));
    pglVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, MODEL_VERTEX_STRIDE,
                           reinterpret_cast<const void*>(6 * sizeof(float)));
}

void useProgram(GLuint program) {
    if (currentProgram == program) {
        return;
    }
    pglUseProgram(program);
    currentProgram = program;
}

void bindTexture(GLenum target, GLuint texture) {
    GLuint& cachedTexture = target == GL_TEXTURE_2D_ARRAY ? boundTextureArray : boundTexture2D;
    if (cachedTexture == texture) {
        return;
    }
    glBindTexture(target, texture);
    cachedTexture = texture;
}

void uploadModelUniforms(const ModelMaterial& material, const std::array<double, 3>& color,
                         double alpha, int alphaMode) {
    const std::array<float, 4> colorValue = {static_cast<float>(color[0]),
                                             static_cast<float>(color[1]),
                                             static_cast<float>(color[2]),
                                             static_cast<float>(alpha)};
    const bool flagsChanged =
        !modelUniformState.valid || modelUniformState.textured != material.textured ||
        modelUniformState.textureArray != (material.textured && material.textureArray) ||
        modelUniformState.useLightmap != material.useLightmap ||
        modelUniformState.useNightmap != material.useNightmap ||
        modelUniformState.useTransmap != material.useTransmap ||
        modelUniformState.useFreeTexture != material.useFreeTexture;
    if (flagsChanged) {
        pglUniform1i(modelUniforms.useTexture, material.textured ? 1 : 0);
        pglUniform1i(modelUniforms.useTextureArray,
                     material.textured && material.textureArray ? 1 : 0);
        pglUniform1i(modelUniforms.useLightmap, material.useLightmap ? 1 : 0);
        pglUniform1i(modelUniforms.useNightmap, material.useNightmap ? 1 : 0);
        pglUniform1i(modelUniforms.useTransmap, material.useTransmap ? 1 : 0);
        pglUniform1i(modelUniforms.useFreeTexture, material.useFreeTexture ? 1 : 0);
    }
    if (!modelUniformState.valid || modelUniformState.alphaMode != alphaMode) {
        pglUniform1i(modelUniforms.alphaMode, alphaMode);
    }
    if (!modelUniformState.valid ||
        modelUniformState.lightmapStrength != material.lightmapStrength) {
        pglUniform1f(modelUniforms.lightmapStrength, material.lightmapStrength);
    }
    if (!modelUniformState.valid || modelUniformState.nightmapStrength != material.nightmapStrength) {
        pglUniform1f(modelUniforms.nightmapStrength, material.nightmapStrength);
    }
    if (!modelUniformState.valid ||
        modelUniformState.texcoordOffsetX != material.texcoordOffsetX ||
        modelUniformState.texcoordOffsetY != material.texcoordOffsetY) {
        pglUniform2f(modelUniforms.texcoordOffset, material.texcoordOffsetX,
                     material.texcoordOffsetY);
    }
    if (!modelUniformState.valid || modelUniformState.flipTextureY != material.flipTextureY) {
        pglUniform1i(modelUniforms.flipTextureY, material.flipTextureY ? 1 : 0);
    }
    if (!modelUniformState.valid || modelUniformState.color != colorValue) {
        pglUniform4f(modelUniforms.color, colorValue[0], colorValue[1], colorValue[2],
                     colorValue[3]);
    }
    modelUniformState.valid = true;
    modelUniformState.textured = material.textured;
    modelUniformState.textureArray = material.textured && material.textureArray;
    modelUniformState.useLightmap = material.useLightmap;
    modelUniformState.useNightmap = material.useNightmap;
    modelUniformState.useTransmap = material.useTransmap;
    modelUniformState.useFreeTexture = material.useFreeTexture;
    modelUniformState.flipTextureY = material.flipTextureY;
    modelUniformState.alphaMode = alphaMode;
    modelUniformState.lightmapStrength = material.lightmapStrength;
    modelUniformState.nightmapStrength = material.nightmapStrength;
    modelUniformState.texcoordOffsetX = material.texcoordOffsetX;
    modelUniformState.texcoordOffsetY = material.texcoordOffsetY;
    modelUniformState.color = colorValue;
}

} // namespace

bool initializeCoreRenderer() {
    modelProgram = createProgram(modelVertexShader, modelFragmentShader);
    environmentProgram = createProgram(modelVertexShader, environmentFragmentShader);
    primitiveProgram = createProgram(primitiveVertexShader, primitiveFragmentShader);
    if (modelProgram == 0 || environmentProgram == 0 || primitiveProgram == 0) {
        shutdownCoreRenderer();
        return false;
    }
    modelUniforms = modelUniformsFor(modelProgram, false);
    environmentUniforms = modelUniformsFor(environmentProgram, true);
    primitiveUniforms = modelUniformsFor(primitiveProgram, false);
    pglGenVertexArrays(1, &modelVertexArray);
    pglGenVertexArrays(1, &primitiveVertexArray);
    pglGenBuffers(1, &primitiveBuffer);
    if (modelVertexArray != 0 && primitiveVertexArray != 0 && primitiveBuffer != 0) {
        pglBindVertexArray(modelVertexArray);
        pglEnableVertexAttribArray(0);
        pglEnableVertexAttribArray(1);
        pglEnableVertexAttribArray(2);
        pglBindVertexArray(primitiveVertexArray);
        pglEnableVertexAttribArray(0);
        pglEnableVertexAttribArray(1);
        pglBindVertexArray(0);
        useProgram(modelProgram);
        pglUniform1i(modelUniforms.texture, 0);
        pglUniform1i(modelUniforms.textureArray, 0);
        pglUniform1i(modelUniforms.lightmap, 1);
        pglUniform1i(modelUniforms.nightmap, 2);
        pglUniform1i(modelUniforms.transmap, 3);
        pglUniform1i(modelUniforms.freeTexture, 4);
        useProgram(environmentProgram);
        pglUniform1i(environmentUniforms.environment, 0);
        useProgram(primitiveProgram);
        useProgram(0);
        currentVertexArray = 0;
        currentArrayBuffer = 0;
        currentProgram = 0;
        matrixProgram = 0;
        modelUniformState = {};
        boundTexture2D = 0;
        boundTextureArray = 0;
        return true;
    }
    shutdownCoreRenderer();
    return false;
}

void shutdownCoreRenderer() {
    currentProgram = 0;
    currentVertexArray = 0;
    currentArrayBuffer = 0;
    matrixProgram = 0;
    modelUniformState = {};
    boundTexture2D = 0;
    boundTextureArray = 0;
    if (primitiveBuffer != 0) {
        pglDeleteBuffers(1, &primitiveBuffer);
        primitiveBuffer = 0;
    }
    if (primitiveVertexArray != 0) {
        pglDeleteVertexArrays(1, &primitiveVertexArray);
        primitiveVertexArray = 0;
    }
    if (modelVertexArray != 0) {
        pglDeleteVertexArrays(1, &modelVertexArray);
        modelVertexArray = 0;
    }
    if (primitiveProgram != 0) {
        pglDeleteProgram(primitiveProgram);
        primitiveProgram = 0;
    }
    if (environmentProgram != 0) {
        pglDeleteProgram(environmentProgram);
        environmentProgram = 0;
    }
    if (modelProgram != 0) {
        pglDeleteProgram(modelProgram);
        modelProgram = 0;
    }
}

void invalidateTextureBindings() {
    boundTexture2D = std::numeric_limits<GLuint>::max();
    boundTextureArray = std::numeric_limits<GLuint>::max();
}

void drawModelBatch(GLuint buffer, std::size_t vertexCount, const ModelMaterial& material,
                    const std::array<double, 3>& color, double alpha, int alphaMode) {
    TraceScope trace("render", "CoreRenderer::drawModelBatch");
    if (modelProgram == 0 || buffer == 0 || vertexCount == 0) {
        return;
    }
    useProgram(modelProgram);
    uploadMatrices(modelUniforms);
    uploadModelUniforms(material, color, alpha, alphaMode);
    pglActiveTexture(GL_TEXTURE0);
    bindTexture(material.textureArray ? GL_TEXTURE_2D_ARRAY : GL_TEXTURE_2D, material.texture);
    pglActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, material.lightmap);
    pglActiveTexture(GL_TEXTURE2);
    glBindTexture(GL_TEXTURE_2D, material.nightmap);
    pglActiveTexture(GL_TEXTURE3);
    glBindTexture(GL_TEXTURE_2D, material.transmap);
    pglActiveTexture(GL_TEXTURE4);
    glBindTexture(GL_TEXTURE_2D, material.freeTexture);
    pglActiveTexture(GL_TEXTURE0);
    bindModelVertexBuffer(buffer, modelVertexArray);
    glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(vertexCount));
}

void drawEnvironmentBatch(GLuint buffer, std::size_t vertexCount, GLuint texture, double alpha) {
    TraceScope trace("render", "CoreRenderer::drawEnvironmentBatch");
    if (environmentProgram == 0 || buffer == 0 || texture == 0 || vertexCount == 0) {
        return;
    }
    useProgram(environmentProgram);
    uploadMatrices(environmentUniforms);
    pglUniform1f(environmentUniforms.environmentAlpha, static_cast<GLfloat>(alpha));
    bindTexture(GL_TEXTURE_2D, texture);
    bindModelVertexBuffer(buffer, modelVertexArray);
    glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(vertexCount));
}

void drawPrimitives(const std::vector<PrimitiveVertex>& vertices, GLenum primitive,
                    float lineWidth) {
    TraceScope trace("render", "CoreRenderer::drawPrimitives");
    if (primitiveProgram == 0 || vertices.empty()) {
        return;
    }
    useProgram(primitiveProgram);
    uploadMatrices(primitiveUniforms);
    if (currentVertexArray != primitiveVertexArray) {
        pglBindVertexArray(primitiveVertexArray);
        currentVertexArray = primitiveVertexArray;
    }
    if (currentArrayBuffer != primitiveBuffer) {
        pglBindBuffer(GL_ARRAY_BUFFER, primitiveBuffer);
        currentArrayBuffer = primitiveBuffer;
    }
    pglBufferData(GL_ARRAY_BUFFER,
                  static_cast<std::ptrdiff_t>(vertices.size() * sizeof(PrimitiveVertex)),
                  vertices.data(), GL_STREAM_DRAW);
    pglVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(PrimitiveVertex), nullptr);
    pglVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(PrimitiveVertex),
                           reinterpret_cast<const void*>(3 * sizeof(float)));
    glLineWidth(lineWidth);
    glDrawArrays(primitive, 0, static_cast<GLsizei>(vertices.size()));
}

} // namespace openbus::rendering
