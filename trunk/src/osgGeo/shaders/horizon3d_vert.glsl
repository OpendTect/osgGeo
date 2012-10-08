#version 130

uniform sampler2D heightMap;
uniform sampler2D normals;
uniform float depthMin;
uniform float depthDiff;

out float depthOut;
out int undef;

@if("hasGeomShader")
out vec2 texCoordPass;
out float diffuseValuePass;
@else
out vec2 texCoordOut;
out float diffuseValue;
@endif

void main(void)
{
    // Extract texture coordinate
    vec2 texCoord = gl_MultiTexCoord0.st;

    // fetch value from the height map. Luminance component
    // contains elevation value, alpha contains undefined flag
    vec4 depthMask = texture2D(heightMap, texCoord);
    float depthComp = depthMask.r;

    // denormalize depth value
    depthOut = depthMin + depthComp * depthDiff;
    vec4 pos = vec4(gl_Vertex.xy, depthOut, 1.0);
    gl_Position = gl_ModelViewProjectionMatrix * pos;

    // pass undefined value to geometry shader
    undef = (depthMask.a > 0.1) ? 1 : 0;

    // Normals and lighting

    //read normal from texture and denormalize it
    vec3 normal = texture2D(normals, texCoord).xyz * 2.0 - 1.0;

    // Transforming The Normal To ModelView-Space
    vec3 vertex_normal = normalize(gl_NormalMatrix * (-normal));

    vec3 vertex_light_position = gl_LightSource[0].position.xyz;

    float diffuse_value = dot(vertex_normal, vertex_light_position);
    if(diffuse_value < 0)
      diffuse_value *= -1;

    // trickery to pass values to either geometry or fragment shader.
    @if("hasGeomShader")
        texCoordPass = texCoord;
        diffuseValuePass = diffuse_value;
    @else
        texCoordOut = texCoord;
        diffuseValue = diffuse_value;
    @endif
}
