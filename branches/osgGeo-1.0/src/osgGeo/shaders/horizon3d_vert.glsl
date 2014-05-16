#version 130
# osgGeo - A collection of geoscientific extensions to OpenSceneGraph.
# Copyright 2011 dGB Beheer B.V. and others.
# 
# osgGeo is free software; you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation; either version 3 of the License, or
# (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
# 
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>
# 
# $Id$
#

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
