#version 330 compatibility

layout( triangles ) in;
layout( triangle_strip, max_vertices = 3 ) out;
in int undef[];
in vec2 texCoordPass[];
out vec2 texCoordOut;

in float diffuseValuePass[];
out float diffuseValue;

void main()
{
    if(undef[0] != 1 && undef[1] != 1 && undef[2] != 1)
    {
        gl_Position = gl_in[0].gl_Position;
        texCoordOut = texCoordPass[0];
        diffuseValue = diffuseValuePass[0];
        EmitVertex();

        gl_Position = gl_in[1].gl_Position;
        texCoordOut = texCoordPass[1];
        diffuseValue = diffuseValuePass[1];
        EmitVertex();

        gl_Position = gl_in[2].gl_Position;
        texCoordOut = texCoordPass[2];
        diffuseValue = diffuseValuePass[2];
        EmitVertex();
    }
    EndPrimitive();
}
