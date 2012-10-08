#version 130

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
