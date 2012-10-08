# version 330 compatibility
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
