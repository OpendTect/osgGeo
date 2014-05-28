/* osgGeo - A collection of geoscientific extensions to OpenSceneGraph.
Copyright 2011- dGB Beheer B.V.

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

$Id: MarkerShape.cpp 242 2013-04-26 11:26:59Z ding.zheng@dgbes.com $

*/

#include "osgGeo/MarkerShape"
#include <osg/LineWidth>

using namespace osgGeo;

MarkerShape::MarkerShape()
    : _hints(new osg::TessellationHints)
    , _size(1.0f)
    , _heightRatio(1.0f)
    , _shapeType(Box)
{
    _center = osg::Vec3(0.0f,0.0f,0.0f);
    _color =  osg::Vec4f(1.0f,1.0f,0.0f,1.0f);
    _hints->setDetailRatio(0.5f);
}


MarkerShape::~MarkerShape()
{
}


void MarkerShape::setSize(float size)
{
    _size = size;
}


void MarkerShape::setHeightRatio(float heightRatio)
{
    _heightRatio = heightRatio;
}


void MarkerShape::setDetail(float ratio)
{
    _hints->setDetailRatio(ratio);
}


const float MarkerShape::getDetail()
{
    return _hints->getDetailRatio();
}


void MarkerShape::setColor(const osg::Vec4f color)
{
    _color = color;
}


void MarkerShape::setType(const ShapeType type)
{
    _shapeType = type;
}

void MarkerShape::setCenter(const osg::Vec3 center)
{
    _center = center;
}


void MarkerShape::setRotation(const osg::Quat& quat)
{
    _rotation = quat;
}


osg::ref_ptr<osg::Drawable> MarkerShape::createShape()
{
    osg::ref_ptr<osg::Drawable> drwB(0);
    osg::ref_ptr<osg::ShapeDrawable> shapeDrwB(0);

    const float radius = 0.5 * _size;
    const float height = _heightRatio * _size;

    switch (_shapeType)
    {
    case Box:
	{
	    osg::Box* box = new osg::Box( _center, _size, _size, height );
	    box->setRotation( _rotation );
	    shapeDrwB = new osg::ShapeDrawable( box, _hints );
	    break;
	}
    case Cone:
	{
	    osg::Cone* cone = new osg::Cone( _center, radius, height );
	    cone->setRotation( _rotation );
	    shapeDrwB = new osg::ShapeDrawable( cone,  _hints );
	    break;
	}
    case Sphere:
	{
	    osg::Sphere* sphere = new osg::Sphere( _center, radius );
	    shapeDrwB = new osg::ShapeDrawable( sphere, _hints );
	    break;
	}
    case Cylinder:
	{
	    osg::Cylinder* cyl = new osg::Cylinder( _center, radius, height );
	    cyl->setRotation( _rotation );
	    shapeDrwB = new osg::ShapeDrawable( cyl, _hints );
	    break;
	}
    case Cross:
	{
	    drwB = createCrossDrawable();
	    break;
	}
    default:
	return 0;
    }

    if ( _shapeType < Cross && shapeDrwB )
    {
	shapeDrwB->setColor(_color);
	drwB = shapeDrwB;
    }
    
    return drwB;
}


osg::ref_ptr<osg::Drawable>  MarkerShape::createCrossDrawable()
{
    osg::ref_ptr<osg::Geometry> crossGeometry = new osg::Geometry;
    osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array(6);

    const float radius = 0.5 * _size;
    const float x1 = _center[0] - radius;
    const float x2 = _center[0] + radius;
    const float y1 = _center[1] - radius;
    const float y2 = _center[1] + radius;
    const float z1 = _center[2] - radius;
    const float z2 = _center[2] + radius;

    (*vertices)[0] = osg::Vec3(0.0f,0.0f,z1);
    (*vertices)[1] = osg::Vec3(0.0f,0.0f,z2);
    (*vertices)[2] = osg::Vec3(x1,0.0f,0.0f);
    (*vertices)[3] = osg::Vec3(x2,0.0f,0.0f);
    (*vertices)[4] = osg::Vec3(0.0f,y1,0.0f);
    (*vertices)[5] = osg::Vec3(0.0f,y2,0.0f);

    crossGeometry->setVertexArray(vertices);
    crossGeometry->addPrimitiveSet(
	new osg::DrawArrays(osg::PrimitiveSet::LINES,0,6));
    osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
    colors->push_back(_color);
    crossGeometry->setColorArray(colors);
    crossGeometry->setColorBinding(osg::Geometry::BIND_OVERALL);

    osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array;
    normals->push_back(osg::Vec3(0.0f,-1.0f,0.0f));
    crossGeometry->setNormalArray(normals);
    crossGeometry->setNormalBinding(osg::Geometry::BIND_OVERALL);

    osg::ref_ptr<osg::LineWidth> lineWidth = new osg::LineWidth;
    lineWidth->setWidth(2.0);
    crossGeometry->getOrCreateStateSet()->setAttributeAndModes(lineWidth);

    return crossGeometry;
}