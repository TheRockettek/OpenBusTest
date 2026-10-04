$input v_normal

#include <bgfx_shader.sh>

uniform vec4 u_lightDirection;
uniform vec4 u_baseColor;
uniform vec4 u_lightParameters;

void main()
{
    float diffuse = max(dot(normalize(v_normal), normalize(u_lightDirection.xyz)), 0.0);
    float illumination = clamp(u_lightParameters.x + u_lightParameters.y * diffuse, 0.0, 1.0);
    gl_FragColor = vec4(u_baseColor.rgb * illumination, u_baseColor.a);
}
