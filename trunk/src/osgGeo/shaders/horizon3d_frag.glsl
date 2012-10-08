# version 130
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
in vec2 texCoordOut;

in float diffuseValue;

uniform vec4 colourPoints[20];
uniform float colourPositions[20];
uniform int paletteSize;

void main(void)
{
  vec4 pix = texture2D(heightMap, texCoordOut);
  float value = pix.g;
  int sz = paletteSize;
  vec4 col;

  if(value == 0.0)
    col = colourPoints[0];
  else if(value > 1.0)
    col = colourPoints[sz-1];
  else
  {
    for(int ii = 0; ii < sz - 1; ++ii)
    {
      if(value < colourPositions[ii+1])
      {
        float factor = (value - colourPositions[ii]) / (colourPositions[ii+1] - colourPositions[ii]);
        col = mix(colourPoints[ii], colourPoints[ii+1], factor);
        break;
      }
    }
  }

  gl_FragColor = col * diffuseValue;
}
