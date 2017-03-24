/* osgGeo - A collection of geoscientific extensions to OpenSceneGraph.
Copyright 2011 dGB Beheer B.V.

osgGeo is free software; you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation; either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>

*/

#include "Palette"
#include <algorithm>

namespace osgGeo
{

Palette::Palette(const ColorPointList &cp)
{
    _colorPoints = cp;
}

osg::Vec3 makeColor(int r, int g, int b)
{
    osg::Vec3 res;
    res.x() = float(r) / 255;
    res.y() = float(g) / 255;
    res.z() = float(b) / 255;
    return res;
}

Palette::Palette()
{
    _colorPoints.push_back(ColorPoint( (float) 0.0, makeColor(170, 0, 0)));
    _colorPoints.push_back(ColorPoint( (float) 0.25, makeColor(255, 200, 0)));
    _colorPoints.push_back(ColorPoint( (float) 0.5, makeColor(243, 243, 243)));
    _colorPoints.push_back(ColorPoint( (float) 0.883249, makeColor(56, 70, 127)));
    _colorPoints.push_back(ColorPoint( (float) 1.0, makeColor(0, 0, 0)));
}

static inline bool fuzzyCompare(float p1, float p2)
{
    return (fabs(p1 - p2) <= 0.00001f * std::min(fabs(p1), fabs(p2)));
}

/*!
  \fn Vec3f GVPalette::get(float value, float min, float max) const

  This function returns a colour for a supplied value given the
  minimum and maximum values. The returned colour is stored in
  Vector<float> class and is simply a triple of floats in range
  from 0 to 1. To access the components use its .r .g and .b fields
*/
osg::Vec3 Palette::get(float value, float min, float max) const
{
    if(value < min)
        return _colorPoints[0].color;
    else if(value > max)
        return _colorPoints[_colorPoints.size() - 1].color;
    else
    {
        const ColorPoint *cPoints = &_colorPoints[0];

        float relativeValue;
        if(fuzzyCompare(max, min))
        {
            relativeValue = (value - min) > 0 ? 1.0 : 0.0;
        }
        else
        {
            relativeValue = (value - min) / (max - min);
        }
        for( unsigned int ii = 0; ii < _colorPoints.size() - 1; ++ii)
        {
            const ColorPoint &cp1 = cPoints[ii];
            const ColorPoint &cp2 = cPoints[ii + 1];

            if(relativeValue < cp2.pos)
            {
                osg::Vec3 color1 = cp1.color;
                osg::Vec3 color2 = cp2.color;

                const float factor = (relativeValue - cp1.pos) / (cp2.pos - cp1.pos);
                return color1 + (color2 - color1) * factor;
            }
        }
    }
    return _colorPoints[_colorPoints.size() - 1].color;
}

void Palette::setColorPoints(const ColorPointList &cps)
{
    _colorPoints = cps;
}

}
