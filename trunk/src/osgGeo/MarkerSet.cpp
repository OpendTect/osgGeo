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

$Id: MarkerSet.cpp 108 2012-12-08 08:32:40Z kristofer.tingdahl@dgbes.com $

*/

#include "osgGeo/MarkerSet"
#include <osgGeo/AutoTransform>

using namespace osgGeo;


MarkerSet::MarkerSet()
    : _rotateMode(osgGeo::AutoTransform::ROTATE_TO_SCREEN)
    , _hints(new osg::TessellationHints)
    , _shapeType(osgGeo::MarkerSet::Box)
    , _colorArr(new osg::Vec4Array)
    , _vertexArr(new osg::Vec3Array)
    , _nonShadingGroup(new osg::Group)
    , _needsUpdate(true)
    , _arrayModCount(-1)
    , _markerHeightRatio(1.0f)
    , _minScale(0.0f)
    , _maxScale(25.5f)
    , _normalArr(new osg::Vec3Array)
{
    setNumChildrenRequiringUpdateTraversal(1);
    _hints->setDetailRatio(0.5f);
}


 MarkerSet::~MarkerSet()
{
}


void MarkerSet::traverse( osg::NodeVisitor& nv )
{
    if ( nv.getVisitorType()==osg::NodeVisitor::UPDATE_VISITOR )
    {
	if ( needsUpdate() )
	    updateShapes();
    }
    
    if ( _nonShadingGroup ) 
	_nonShadingGroup->accept( nv );
}


bool MarkerSet::updateShapes()
{
    if (!_vertexArr || !_needsUpdate)
	return false;

    _nonShadingGroup->removeChildren(0, _nonShadingGroup->getNumChildren());

    for (unsigned int idx=0;idx<_vertexArr->size();idx++)
    {
	osg::ref_ptr<osg::ShapeDrawable> shapeDrwB;

	switch (_shapeType)
	{
	case osgGeo::MarkerSet::Box:
	    shapeDrwB = new osg::ShapeDrawable(new osg::Box(
		 osg::Vec3f(0,0,0),_markerSize,_markerSize,_markerHeightRatio*_markerSize),_hints);
	    break;
	case osgGeo::MarkerSet::Cone:
	    shapeDrwB = new osg::ShapeDrawable(new osg::Cone(
		 osg::Vec3f(0,0,0),_markerSize,_markerHeightRatio*_markerSize),_hints);
	    break;
	case osgGeo::MarkerSet::Sphere:
	    shapeDrwB = new osg::ShapeDrawable(new osg::Sphere(
		 osg::Vec3f(0,0,0),_markerSize),_hints);
	    break;
	case osgGeo::MarkerSet::Cylinder:
	    shapeDrwB = new osg::ShapeDrawable(new osg::Cylinder(
		osg::Vec3f(0,0,0),_markerSize,_markerHeightRatio*_markerSize),_hints);
	    break;
	default:
	    return false;
	}

	if (idx<_colorArr->size())
	    shapeDrwB->setColor(_colorArr->at( idx ));
	else
	    shapeDrwB->setColor(*(_colorArr->end()-1));

	osg::ref_ptr<osg::Geode> geode = new osg::Geode;
	geode->addDrawable(shapeDrwB);
	geode->getOrCreateStateSet()->setMode(GL_LIGHTING,
	    osg::StateAttribute::OFF);

	osg::ref_ptr<osg::AutoTransform> autotrans = 
	    new osg::AutoTransform;
	autotrans->setPosition(_vertexArr->at(idx));
	autotrans->setAutoRotateMode(_rotateMode);
	autotrans->setAutoScaleToScreen(true);

	autotrans->setMinimumScale(0.0f);
	autotrans->setMaximumScale(DBL_MAX);

	if ( !_useScreenSize )
	    autotrans->setScale( (double)1.0 );
	else
	{
	    autotrans->setMinimumScale(_minScale);
	    autotrans->setMaximumScale(_maxScale);
	}

	autotrans->addChild(geode);
	_nonShadingGroup->addChild(autotrans);
    }
    _needsUpdate = false;
    _arrayModCount = _vertexArr->getModifiedCount();
    return true;
}


osg::BoundingSphere MarkerSet::computeBound() const
{
   return _nonShadingGroup->getBound();
}


bool MarkerSet::needsUpdate()
{
    if ((int)_vertexArr->getModifiedCount()>_arrayModCount)
	_needsUpdate = true;
    return _needsUpdate;
}


void MarkerSet::setVertexArray(osg::Vec3Array* arr)
{
    if (arr==_vertexArr)
	return;
    _vertexArr = arr;
    _needsUpdate = true;
}


void MarkerSet::setNormalArray(osg::Vec3Array* arr)
{
    if (arr==_normalArr)
	return;
    _normalArr = arr;
    _needsUpdate = true;
}


void MarkerSet::setShape(osgGeo::MarkerSet::MarkerType shape)
{
    _shapeType = shape;
    _needsUpdate = true;
}


float MarkerSet::getMarkerSize() const
{
    return _useScreenSize ? _markerSize : -1;
}


void MarkerSet::setMarkerHeightRatio(float markerHeightRatio)
{
    _markerHeightRatio = markerHeightRatio;
    _needsUpdate = true;
}


void MarkerSet::setDetail(float ratio)
{
    _hints->setDetailRatio(ratio);
    _needsUpdate = true;
}


void MarkerSet::setMinScale(float minScale)
{
    _minScale = minScale;
    _needsUpdate = true;
}


void MarkerSet::setMaxScale(float maxScale)
{
    _maxScale = maxScale;
    _needsUpdate = true;
}


void MarkerSet::setRotateMode( osg::AutoTransform::AutoRotateMode rtmode )
{
    _rotateMode = rtmode;
    _needsUpdate = true;
}


void MarkerSet::setColorArray(osg::Vec4Array* colorArr)
{
    _colorArr = colorArr;
    _needsUpdate = true;
}


void MarkerSet::setMarkerSize(float markerSize, bool useScreenSize)
{
    _markerSize = markerSize;
    _useScreenSize = useScreenSize;
    _needsUpdate = true;
}



