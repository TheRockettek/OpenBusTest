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
constexpr std::size_t MAX_MATERIAL_TEXTURES = 8;
constexpr std::size_t MAX_TEXTURE_UNITS = 16;

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
    GLint bumpmap = -1;
    GLint freeTexture = -1;
    GLint textTexture = -1;
    GLint useLightmap = -1;
    GLint useNightmap = -1;
    GLint useTransmap = -1;
    GLint useBumpmap = -1;
    GLint useFreeTexture = -1;
    GLint useTextTexture = -1;
    GLint useInteriorLight = -1;
    GLint lightmapStrength = -1;
    GLint nightmapStrength = -1;
    GLint bumpmapStrength = -1;
    GLint interiorLightStrength = -1;
    GLint interiorLightColor = -1;
    GLint texcoordOffset = -1;
    GLint flipTextureY = -1;
    std::array<GLint, MAX_MATERIAL_TEXTURES> materialFlipTextureY = {};
    GLint useMaterialColors = -1;
    GLint materialColors = -1;
    GLint useMaterialTextures = -1;
    std::array<GLint, MAX_MATERIAL_TEXTURES> materialTextures = {};
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
    bool useBumpmap = false;
    bool useFreeTexture = false;
    bool useTextTexture = false;
    bool useInteriorLight = false;
    bool flipTextureY = false;
    bool useMaterialColors = false;
    const void* materialColorData = nullptr;
    GLsizei materialColorCount = 0;
    bool useMaterialTextures = false;
    const void* materialTextureData = nullptr;
    GLsizei materialTextureCount = 0;
    int alphaMode = 0;
    float lightmapStrength = 0.0f;
    float nightmapStrength = 0.0f;
    float bumpmapStrength = 0.0f;
    float interiorLightStrength = 0.0f;
    std::array<float, 3> interiorLightColor = {};
    float texcoordOffsetX = 0.0f;
    float texcoordOffsetY = 0.0f;
    std::array<float, 4> color = {};
};

GLuint modelProgram = 0;
GLuint materialProgram = 0;
GLuint environmentProgram = 0;
GLuint primitiveProgram = 0;
GLuint textureQuadProgram = 0;
GLuint modelVertexArray = 0;
GLuint primitiveVertexArray = 0;
GLuint primitiveBuffer = 0;
GLuint textureQuadVertexArray = 0;
GLuint textureQuadBuffer = 0;
Uniforms modelUniforms;
Uniforms materialUniforms;
Uniforms environmentUniforms;
Uniforms primitiveUniforms;
GLint textureQuadRect = -1;
GLint textureQuadTexture = -1;
GLint textureQuadFlipTextureY = -1;
GLuint currentProgram = 0;
GLuint currentVertexArray = 0;
GLuint currentArrayBuffer = 0;
GLenum currentTextureUnit = GL_TEXTURE0;
std::array<GLuint, MAX_TEXTURE_UNITS> boundTexture2D = {};
std::array<GLuint, MAX_TEXTURE_UNITS> boundTextureArray = {};
GLuint matrixProgram = 0;
Matrix4 cachedProjection = {};
Matrix4 cachedModelView = {};
ModelUniformState modelUniformState;
struct MaterialUniformState {
    bool valid = false;
    const void* colorData = nullptr;
    GLsizei colorCount = 0;
};
MaterialUniformState materialUniformState;

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
uniform sampler2D uBumpmap;
uniform sampler2D uFreeTexture;
uniform sampler2D uTextTexture;
uniform bool uUseTexture;
uniform bool uUseTextureArray;
uniform bool uUseLightmap;
uniform bool uUseNightmap;
uniform bool uUseTransmap;
uniform bool uUseBumpmap;
uniform bool uUseFreeTexture;
uniform bool uUseTextTexture;
uniform bool uUseInteriorLight;
uniform float uLightmapStrength;
uniform float uNightmapStrength;
uniform float uBumpmapStrength;
uniform float uInteriorLightStrength;
uniform vec3 uInteriorLightColor;
uniform vec4 uColor;
uniform int uAlphaMode;
out vec4 fragmentColor;
void main() {
    vec4 color = uColor;
    if (uUseTextTexture) {
        color *= texture(uTextTexture, vTexCoord.xy);
    } else if (uUseFreeTexture) {
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
        color.a = texture(uTransmap, vTexCoord.xy).a;
    }
    if (uUseBumpmap) {
        vec3 surfaceNormal = normalize(vNormal);
        vec3 positionDerivativeX = dFdx(vViewPosition);
        vec3 positionDerivativeY = dFdy(vViewPosition);
        vec2 coordinateDerivativeX = dFdx(vTexCoord.xy);
        vec2 coordinateDerivativeY = dFdy(vTexCoord.xy);
        float coordinateDeterminant = coordinateDerivativeX.x * coordinateDerivativeY.y -
                                      coordinateDerivativeX.y * coordinateDerivativeY.x;
        if (abs(coordinateDeterminant) > 0.00001) {
            vec3 tangent = (positionDerivativeX * coordinateDerivativeY.y -
                            positionDerivativeY * coordinateDerivativeX.y) /
                           coordinateDeterminant;
            vec3 bitangent = (-positionDerivativeX * coordinateDerivativeY.x +
                              positionDerivativeY * coordinateDerivativeX.x) /
                             coordinateDeterminant;
            if (length(tangent) > 0.00001 && length(bitangent) > 0.00001) {
                tangent = normalize(tangent);
                bitangent = normalize(bitangent);
                vec3 mapNormal = texture(uBumpmap, vTexCoord.xy).xyz * 2.0 - 1.0;
                float strength = clamp(uBumpmapStrength, 0.0, 1.0);
                mapNormal.xy *= strength;
                surfaceNormal = normalize(tangent * mapNormal.x + bitangent * mapNormal.y +
                                           surfaceNormal * mapNormal.z);
                vec3 lightDirection = normalize(vec3(0.35, 0.55, 0.75));
                float bumpLighting = 0.65 +
                                     0.35 * max(dot(surfaceNormal, lightDirection), 0.0);
                color.rgb *= mix(1.0, bumpLighting, strength);
            }
        }
    }
    if (uUseInteriorLight) {
        color.rgb = clamp(color.rgb + uInteriorLightColor * uInteriorLightStrength, 0.0, 1.0);
    }
    if (uAlphaMode == 1 && color.a < 0.5) {
        discard;
    }
    if (uAlphaMode == 3 && color.a < 0.99) {
        discard;
    }
    fragmentColor = color;
}
)GLSL";

const char* materialFragmentShader = R"GLSL(
#version 330 core
in vec3 vTexCoord;
uniform vec4 uMaterialColors[128];
uniform sampler2D uMaterialTextures[8];
uniform bool uFlipTextureY[8];
out vec4 fragmentColor;
void main() {
    int materialIndex = clamp(int(vTexCoord.z + 0.5), 0, 127);
    int textureIndex = clamp(materialIndex, 0, 7);
    vec2 textureCoord = uFlipTextureY[textureIndex]
                            ? vec2(vTexCoord.x, 1.0 - vTexCoord.y)
                            : vTexCoord.xy;
    vec4 color = uMaterialColors[materialIndex];
    if (textureIndex == 0) {
        color *= texture(uMaterialTextures[0], textureCoord);
    } else if (textureIndex == 1) {
        color *= texture(uMaterialTextures[1], textureCoord);
    } else if (textureIndex == 2) {
        color *= texture(uMaterialTextures[2], textureCoord);
    } else if (textureIndex == 3) {
        color *= texture(uMaterialTextures[3], textureCoord);
    } else if (textureIndex == 4) {
        color *= texture(uMaterialTextures[4], textureCoord);
    } else if (textureIndex == 5) {
        color *= texture(uMaterialTextures[5], textureCoord);
    } else if (textureIndex == 6) {
        color *= texture(uMaterialTextures[6], textureCoord);
    } else {
        color *= texture(uMaterialTextures[7], textureCoord);
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

const char* textureQuadVertexShader = R"GLSL(
#version 330 core
layout(location = 0) in vec2 aPosition;
layout(location = 1) in vec2 aTexCoord;
uniform vec4 uRect;
uniform bool uFlipTextureY;
out vec2 vTexCoord;
void main() {
    vec2 position = uRect.xy + aPosition * uRect.zw;
    gl_Position = vec4(position, 0.0, 1.0);
    vTexCoord = vec2(aTexCoord.x, uFlipTextureY ? 1.0 - aTexCoord.y : aTexCoord.y);
}
)GLSL";

const char* textureQuadFragmentShader = R"GLSL(
#version 330 core
in vec2 vTexCoord;
uniform sampler2D uTexture;
out vec4 fragmentColor;
void main() {
    fragmentColor = texture(uTexture, vTexCoord);
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
        uniforms.bumpmap = pglGetUniformLocation(program, "uBumpmap");
        uniforms.freeTexture = pglGetUniformLocation(program, "uFreeTexture");
        uniforms.textTexture = pglGetUniformLocation(program, "uTextTexture");
        uniforms.useLightmap = pglGetUniformLocation(program, "uUseLightmap");
        uniforms.useNightmap = pglGetUniformLocation(program, "uUseNightmap");
        uniforms.useTransmap = pglGetUniformLocation(program, "uUseTransmap");
        uniforms.useBumpmap = pglGetUniformLocation(program, "uUseBumpmap");
        uniforms.useFreeTexture = pglGetUniformLocation(program, "uUseFreeTexture");
        uniforms.useTextTexture = pglGetUniformLocation(program, "uUseTextTexture");
        uniforms.useInteriorLight = pglGetUniformLocation(program, "uUseInteriorLight");
        uniforms.lightmapStrength = pglGetUniformLocation(program, "uLightmapStrength");
        uniforms.nightmapStrength = pglGetUniformLocation(program, "uNightmapStrength");
        uniforms.bumpmapStrength = pglGetUniformLocation(program, "uBumpmapStrength");
        uniforms.interiorLightStrength = pglGetUniformLocation(program, "uInteriorLightStrength");
        uniforms.interiorLightColor = pglGetUniformLocation(program, "uInteriorLightColor");
        uniforms.texcoordOffset = pglGetUniformLocation(program, "uTexcoordOffset");
        uniforms.flipTextureY = pglGetUniformLocation(program, "uFlipTextureY");
        uniforms.useMaterialColors = pglGetUniformLocation(program, "uUseMaterialColors");
        uniforms.materialColors = pglGetUniformLocation(program, "uMaterialColors");
        uniforms.useMaterialTextures = pglGetUniformLocation(program, "uUseMaterialTextures");
        for (std::size_t index = 0; index < MAX_MATERIAL_TEXTURES; ++index) {
            uniforms.materialTextures[index] = pglGetUniformLocation(
                program, ("uMaterialTextures[" + std::to_string(index) + "]").c_str());
        }
    }
    return uniforms;
}

Uniforms materialUniformsFor(GLuint program) {
    Uniforms uniforms;
    uniforms.projection = pglGetUniformLocation(program, "uProjection");
    uniforms.modelView = pglGetUniformLocation(program, "uModelView");
    uniforms.materialColors = pglGetUniformLocation(program, "uMaterialColors");
    for (std::size_t index = 0; index < MAX_MATERIAL_TEXTURES; ++index) {
        uniforms.materialFlipTextureY[index] = pglGetUniformLocation(
            program, ("uFlipTextureY[" + std::to_string(index) + "]").c_str());
    }
    for (std::size_t index = 0; index < MAX_MATERIAL_TEXTURES; ++index) {
        uniforms.materialTextures[index] = pglGetUniformLocation(
            program, ("uMaterialTextures[" + std::to_string(index) + "]").c_str());
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
    const std::size_t unit = static_cast<std::size_t>(currentTextureUnit - GL_TEXTURE0);
    GLuint& cachedTexture =
        target == GL_TEXTURE_2D_ARRAY ? boundTextureArray[unit] : boundTexture2D[unit];
    if (cachedTexture == texture) {
        return;
    }
    glBindTexture(target, texture);
    cachedTexture = texture;
}

void selectTextureUnit(GLenum textureUnit) {
    if (currentTextureUnit == textureUnit) {
        return;
    }
    pglActiveTexture(textureUnit);
    currentTextureUnit = textureUnit;
}

void uploadModelUniforms(const ModelMaterial& material, const std::array<double, 3>& color,
                         double alpha, int alphaMode) {
    const std::array<float, 4> colorValue = {
        static_cast<float>(color[0]), static_cast<float>(color[1]), static_cast<float>(color[2]),
        static_cast<float>(alpha)};
    const bool flagsChanged =
        !modelUniformState.valid || modelUniformState.textured != material.textured ||
        modelUniformState.textureArray != (material.textured && material.textureArray) ||
        modelUniformState.useLightmap != material.useLightmap ||
        modelUniformState.useNightmap != material.useNightmap ||
        modelUniformState.useTransmap != material.useTransmap ||
        modelUniformState.useBumpmap != material.useBumpmap ||
        modelUniformState.useFreeTexture != material.useFreeTexture ||
        modelUniformState.useTextTexture != material.useTextTexture ||
        modelUniformState.useInteriorLight != material.useInteriorLight;
    if (flagsChanged) {
        pglUniform1i(modelUniforms.useTexture, material.textured ? 1 : 0);
        pglUniform1i(modelUniforms.useTextureArray,
                     material.textured && material.textureArray ? 1 : 0);
        pglUniform1i(modelUniforms.useLightmap, material.useLightmap ? 1 : 0);
        pglUniform1i(modelUniforms.useNightmap, material.useNightmap ? 1 : 0);
        pglUniform1i(modelUniforms.useTransmap, material.useTransmap ? 1 : 0);
        pglUniform1i(modelUniforms.useBumpmap, material.useBumpmap ? 1 : 0);
        pglUniform1i(modelUniforms.useFreeTexture, material.useFreeTexture ? 1 : 0);
        pglUniform1i(modelUniforms.useTextTexture, material.useTextTexture ? 1 : 0);
        pglUniform1i(modelUniforms.useInteriorLight, material.useInteriorLight ? 1 : 0);
    }
    if (!modelUniformState.valid || modelUniformState.alphaMode != alphaMode) {
        pglUniform1i(modelUniforms.alphaMode, alphaMode);
    }
    if (!modelUniformState.valid ||
        modelUniformState.lightmapStrength != material.lightmapStrength) {
        pglUniform1f(modelUniforms.lightmapStrength, material.lightmapStrength);
    }
    if (!modelUniformState.valid ||
        modelUniformState.nightmapStrength != material.nightmapStrength) {
        pglUniform1f(modelUniforms.nightmapStrength, material.nightmapStrength);
    }
    if (!modelUniformState.valid || modelUniformState.bumpmapStrength != material.bumpmapStrength) {
        pglUniform1f(modelUniforms.bumpmapStrength, material.bumpmapStrength);
    }
    if (!modelUniformState.valid ||
        modelUniformState.interiorLightStrength != material.interiorLightStrength) {
        pglUniform1f(modelUniforms.interiorLightStrength, material.interiorLightStrength);
    }
    if (!modelUniformState.valid ||
        modelUniformState.interiorLightColor != material.interiorLightColor) {
        pglUniform3f(modelUniforms.interiorLightColor, material.interiorLightColor[0],
                     material.interiorLightColor[1], material.interiorLightColor[2]);
    }
    if (!modelUniformState.valid || modelUniformState.texcoordOffsetX != material.texcoordOffsetX ||
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
    modelUniformState.useBumpmap = material.useBumpmap;
    modelUniformState.useFreeTexture = material.useFreeTexture;
    modelUniformState.useTextTexture = material.useTextTexture;
    modelUniformState.useInteriorLight = material.useInteriorLight;
    modelUniformState.flipTextureY = material.flipTextureY;
    modelUniformState.alphaMode = alphaMode;
    modelUniformState.lightmapStrength = material.lightmapStrength;
    modelUniformState.nightmapStrength = material.nightmapStrength;
    modelUniformState.bumpmapStrength = material.bumpmapStrength;
    modelUniformState.interiorLightStrength = material.interiorLightStrength;
    modelUniformState.interiorLightColor = material.interiorLightColor;
    modelUniformState.texcoordOffsetX = material.texcoordOffsetX;
    modelUniformState.texcoordOffsetY = material.texcoordOffsetY;
    modelUniformState.color = colorValue;
}

} // namespace

bool initializeCoreRenderer() {
    modelProgram = createProgram(modelVertexShader, modelFragmentShader);
    materialProgram = createProgram(modelVertexShader, materialFragmentShader);
    environmentProgram = createProgram(modelVertexShader, environmentFragmentShader);
    primitiveProgram = createProgram(primitiveVertexShader, primitiveFragmentShader);
    textureQuadProgram = createProgram(textureQuadVertexShader, textureQuadFragmentShader);
    if (modelProgram == 0 || materialProgram == 0 || environmentProgram == 0 ||
        primitiveProgram == 0 || textureQuadProgram == 0) {
        shutdownCoreRenderer();
        return false;
    }
    modelUniforms = modelUniformsFor(modelProgram, false);
    materialUniforms = materialUniformsFor(materialProgram);
    environmentUniforms = modelUniformsFor(environmentProgram, true);
    primitiveUniforms = modelUniformsFor(primitiveProgram, false);
    textureQuadRect = pglGetUniformLocation(textureQuadProgram, "uRect");
    textureQuadTexture = pglGetUniformLocation(textureQuadProgram, "uTexture");
    textureQuadFlipTextureY = pglGetUniformLocation(textureQuadProgram, "uFlipTextureY");
    pglGenVertexArrays(1, &modelVertexArray);
    pglGenVertexArrays(1, &primitiveVertexArray);
    pglGenBuffers(1, &primitiveBuffer);
    pglGenVertexArrays(1, &textureQuadVertexArray);
    pglGenBuffers(1, &textureQuadBuffer);
    if (modelVertexArray != 0 && primitiveVertexArray != 0 && primitiveBuffer != 0 &&
        textureQuadVertexArray != 0 && textureQuadBuffer != 0) {
        pglBindVertexArray(modelVertexArray);
        pglEnableVertexAttribArray(0);
        pglEnableVertexAttribArray(1);
        pglEnableVertexAttribArray(2);
        pglBindVertexArray(primitiveVertexArray);
        pglEnableVertexAttribArray(0);
        pglEnableVertexAttribArray(1);
        pglBindVertexArray(textureQuadVertexArray);
        pglEnableVertexAttribArray(0);
        pglEnableVertexAttribArray(1);
        pglBindVertexArray(0);
        useProgram(modelProgram);
        pglUniform1i(modelUniforms.texture, 0);
        pglUniform1i(modelUniforms.textureArray, 0);
        pglUniform1i(modelUniforms.lightmap, 1);
        pglUniform1i(modelUniforms.nightmap, 2);
        pglUniform1i(modelUniforms.transmap, 3);
        pglUniform1i(modelUniforms.bumpmap, 6);
        pglUniform1i(modelUniforms.freeTexture, 4);
        pglUniform1i(modelUniforms.textTexture, 5);
        useProgram(materialProgram);
        for (std::size_t index = 0; index < MAX_MATERIAL_TEXTURES; ++index) {
            pglUniform1i(materialUniforms.materialTextures[index], static_cast<GLint>(index));
        }
        useProgram(environmentProgram);
        pglUniform1i(environmentUniforms.environment, 0);
        useProgram(primitiveProgram);
        useProgram(textureQuadProgram);
        pglUniform1i(textureQuadTexture, 0);
        useProgram(0);
        currentVertexArray = 0;
        currentArrayBuffer = 0;
        currentProgram = 0;
        matrixProgram = 0;
        modelUniformState = {};
        materialUniformState = {};
        currentTextureUnit = GL_TEXTURE0;
        boundTexture2D.fill(0);
        boundTextureArray.fill(0);
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
    materialUniformState = {};
    currentTextureUnit = GL_TEXTURE0;
    boundTexture2D.fill(0);
    boundTextureArray.fill(0);
    if (primitiveBuffer != 0) {
        pglDeleteBuffers(1, &primitiveBuffer);
        primitiveBuffer = 0;
    }
    if (textureQuadBuffer != 0) {
        pglDeleteBuffers(1, &textureQuadBuffer);
        textureQuadBuffer = 0;
    }
    if (primitiveVertexArray != 0) {
        pglDeleteVertexArrays(1, &primitiveVertexArray);
        primitiveVertexArray = 0;
    }
    if (textureQuadVertexArray != 0) {
        pglDeleteVertexArrays(1, &textureQuadVertexArray);
        textureQuadVertexArray = 0;
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
    if (materialProgram != 0) {
        pglDeleteProgram(materialProgram);
        materialProgram = 0;
    }
    if (textureQuadProgram != 0) {
        pglDeleteProgram(textureQuadProgram);
        textureQuadProgram = 0;
    }
}

void invalidateTextureBindings() {
    boundTexture2D.fill(std::numeric_limits<GLuint>::max());
    boundTextureArray.fill(std::numeric_limits<GLuint>::max());
    pglActiveTexture(GL_TEXTURE0);
    currentTextureUnit = GL_TEXTURE0;
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
    selectTextureUnit(GL_TEXTURE0);
    const GLenum textureTarget = material.textureArray ? GL_TEXTURE_2D_ARRAY : GL_TEXTURE_2D;
    bindTexture(textureTarget, material.texture);
    if (material.texture != 0) {
        glTexParameteri(textureTarget, GL_TEXTURE_WRAP_S, material.textureWrapS);
        glTexParameteri(textureTarget, GL_TEXTURE_WRAP_T, material.textureWrapT);
    }
    selectTextureUnit(GL_TEXTURE1);
    bindTexture(GL_TEXTURE_2D, material.lightmap);
    selectTextureUnit(GL_TEXTURE2);
    bindTexture(GL_TEXTURE_2D, material.nightmap);
    selectTextureUnit(GL_TEXTURE3);
    bindTexture(GL_TEXTURE_2D, material.transmap);
    selectTextureUnit(GL_TEXTURE0 + 6);
    bindTexture(GL_TEXTURE_2D, material.bumpmap);
    selectTextureUnit(GL_TEXTURE4);
    bindTexture(GL_TEXTURE_2D, material.freeTexture);
    selectTextureUnit(GL_TEXTURE0 + 5);
    bindTexture(GL_TEXTURE_2D, material.textTexture);
    selectTextureUnit(GL_TEXTURE0);
    bindModelVertexBuffer(buffer, modelVertexArray);
    glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(vertexCount));
}

void drawMaterialBatch(GLuint buffer, std::size_t vertexCount,
                       const std::vector<std::array<float, 4>>& colors,
                       const std::vector<GLuint>& textures, const std::vector<bool>& flipTextureY) {
    TraceScope trace("render", "CoreRenderer::drawMaterialBatch");
    if (materialProgram == 0 || buffer == 0 || vertexCount == 0 || colors.empty() ||
        textures.empty()) {
        return;
    }
    useProgram(materialProgram);
    uploadMatrices(materialUniforms);
    for (std::size_t index = 0; index < MAX_MATERIAL_TEXTURES; ++index) {
        pglUniform1i(materialUniforms.materialFlipTextureY[index],
                     flipTextureY.size() > index && flipTextureY[index] ? 1 : 0);
    }
    const GLsizei colorCount = static_cast<GLsizei>(std::min<std::size_t>(colors.size(), 128));
    if (!materialUniformState.valid || materialUniformState.colorData != colors.data() ||
        materialUniformState.colorCount != colorCount) {
        pglUniform4fv(materialUniforms.materialColors, colorCount,
                      reinterpret_cast<const GLfloat*>(colors.data()));
        materialUniformState.valid = true;
        materialUniformState.colorData = colors.data();
        materialUniformState.colorCount = colorCount;
    }
    const std::size_t textureCount = std::min<std::size_t>(textures.size(), MAX_MATERIAL_TEXTURES);
    for (std::size_t index = 0; index < textureCount; ++index) {
        selectTextureUnit(GL_TEXTURE0 + static_cast<GLenum>(index));
        bindTexture(GL_TEXTURE_2D, textures[index]);
    }
    selectTextureUnit(GL_TEXTURE0);
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
    selectTextureUnit(GL_TEXTURE0);
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

void drawTextureQuad(GLuint texture, float x, float y, float width, float height,
                     bool flipTextureY) {
    if (textureQuadProgram == 0 || textureQuadVertexArray == 0 || textureQuadBuffer == 0 ||
        texture == 0) {
        return;
    }
    constexpr std::array<float, 16> vertices = {0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f,
                                                0.0f, 1.0f, 0.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};
    useProgram(textureQuadProgram);
    pglBindVertexArray(textureQuadVertexArray);
    pglBindBuffer(GL_ARRAY_BUFFER, textureQuadBuffer);
    pglBufferData(GL_ARRAY_BUFFER, static_cast<std::ptrdiff_t>(vertices.size() * sizeof(float)),
                  vertices.data(), GL_STREAM_DRAW);
    pglVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), nullptr);
    pglVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float),
                           reinterpret_cast<const void*>(2 * sizeof(float)));
    pglUniform4f(textureQuadRect, x, y, width, height);
    pglUniform1i(textureQuadFlipTextureY, flipTextureY ? 1 : 0);
    selectTextureUnit(GL_TEXTURE0);
    bindTexture(GL_TEXTURE_2D, texture);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
}

} // namespace openbus::rendering
