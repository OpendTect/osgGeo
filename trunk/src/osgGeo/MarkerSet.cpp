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

$Id$

*/

#include "osgGeo/MarkerSet"
#include <osg/Switch>
#include <osgGeo/ComputeBoundsVisitor>


using namespace osgGeo;


MarkerSet::MarkerSet()
    : _rotateMode(osg::AutoTransform::ROTATE_TO_SCREEN)
    , _colorArr(new osg::Vec4Array)
    , _vertexArr(new osg::Vec3Array)
    , _nonShadingSwitch(new osg::Switch)
    , _minScale(0.0f)
    , _maxScale(25.5f)
    , _normalArr(new osg::Vec3Array)
    , _applySingleColor( false )
    , _forceRedraw( false )
    , _waitForAutoTransformUpdate( false )
{
    setNumChildrenRequiringUpdateTraversal(0);
    _singleColor = osg::Vec4(0.1f, 0.1f, 0.1f, 1.0f);
    _bbox.init();
}


 MarkerSet::~MarkerSet()
{
}


void MarkerSet::traverse( osg::NodeVisitor& nv )
{
    if ( nv.getVisitorType()==osg::NodeVisitor::UPDATE_VISITOR )
    {
	if ( _forceRedraw )
	{
	    forceRedraw( false );
	    updateShapes();
	}
    }
    else if ( nv.getVisitorType()==osg::NodeVisitor::CULL_VISITOR )
    {
	if ( _waitForAutoTransformUpdate )
	{
	    _waitForAutoTransformUpdate = false;
	    setCullingActive(true);
	    dirtyBound();
	}
    }
    else
    {
	osgGeo::ComputeBoundsVisitor* cbv =
	    dynamic_cast<osgGeo::ComputeBoundsVisitor*>( &nv );
	if ( cbv )
	{
	    cbv->applyBBox(_bbox);
	    return;
	}
    }
    
    if ( _nonShadingSwitch ) 
	_nonShadingSwitch->accept( nv );
}


bool MarkerSet::updateShapes()
{
    if (!_vertexArr ) return false;

    osg::Switch::ValueList valuelist = _nonShadingSwitch->getValueList();
    _nonShadingSwitch->removeChildren(0, _nonShadingSwitch->getNumChildren());

    osg::BoundingBox bbox;
    bbox.init();
    for (unsigned int idx=0;idx<_vertexArr->size();idx++)
    {
	if( !_applySingleColor && _colorArr )
	{
	    if (idx<_colorArr->size())
		_markerShape.setColor(_colorArr->at( idx ));
	    else if ( _colorArr->size() >0 )
		_markerShape.setColor(*(_colorArr->end()-1));
	}
	else if ( _applySingleColor )
	    _markerShape.setColor( _singleColor );

	osg::ref_ptr<osg::Drawable> drwB = _markerShape.createShape();

	osg::ref_ptr<osg::Geode> geode = new osg::Geode;
	geode->addDrawable(drwB);
	geode->getOrCreateStateSet()->setMode(GL_LIGHTING,
	    osg::StateAttribute::OFF);

	osg::ref_ptr<osg::AutoTransform> autotrans = 
	    new osg::AutoTransform;
	autotrans->setPosition(_vertexArr->at(idx));
	autotrans->setAutoRotateMode(_rotateMode);
	autotrans->setAutoScaleToScreen(true);
	autotrans->setMinimumScale(0.0f);
	autotrans->setMaximumScale(DBL_MAX);
	bbox.expandBy(_vertexArr->at(idx));

	if ( !_useScreenSize )
	    autotrans->setScale( (double)1.0 );
	else
	{
	    autotrans->setMinimumScale(_minScale);
	    autotrans->setMaximumScale(_maxScale);
	}

	autotrans->addChild(geode);
	_nonShadingSwitch->addChild(autotrans);

	if ( idx >= valuelist.size() )
	    valuelist.push_back(true);
    }

    if ( _vertexArr->size() < valuelist.size() )
    {
	int diff = valuelist.size() - _vertexArr->size();
	for ( int i = 0; i < diff; i++ )
	    valuelist.pop_back();
    }

    // In case of autoscale to screen, new AutoTransforms cannot compute
    // their bounding spheres till after their first cull traversal. 
    _waitForAutoTransformUpdate = true;
    setCullingActive(false);
    
    _nonShadingSwitch->setValueList(valuelist);

    if ( _bbox._min != bbox._min || _bbox._max!= bbox._max )
    {
	_bbox = bbox;
	dirtyBound();
    }

    return true;
}


void MarkerSet::turnMarkerOn(unsigned int idx,bool yn)
{
    if ( idx >= _nonShadingSwitch->getNumChildren() )
	return;
    _nonShadingSwitch->setChildValue(_nonShadingSwitch->getChild(idx), yn);
    forceRedraw(true);
}


void MarkerSet::turnAllMarkersOn(bool yn)
{
    yn = true ? _nonShadingSwitch->setAllChildrenOn() : 
	       _nonShadingSwitch->setAllChildrenOff() ;
    forceRedraw(true);
}


osg::BoundingSphere MarkerSet::computeBound() const
{
    if ( _bbox.valid() )
	return _bbox;

    osg::BoundingBox bbox;

    for ( unsigned int idx=0; idx<_vertexArr->size(); idx++ )
	bbox.expandBy((*_vertexArr)[idx]);

    return bbox;
}


void MarkerSet::forceRedraw(bool yn)
{
    if ( yn == _forceRedraw )
	return;
    setNumChildrenRequiringUpdateTraversal(
    _nonShadingSwitch->getNumChildrenRequiringUpdateTraversal() + ((int) yn));
    _forceRedraw = yn;
}


void MarkerSet::setVertexArray(osg::Vec3Array* arr)
{
    if (arr==_vertexArr)
	return;
    _vertexArr = arr;
    forceRedraw(true);
}


void MarkerSet::setNormalArray(osg::Vec3Array* arr)
{
    if (arr==_normalArr)
	return;
    _normalArr = arr;
    forceRedraw(true);
}


void MarkerSet::setShape(osgGeo::MarkerShape::ShapeType shape)
{
    _markerShape.setType(shape);
    forceRedraw(true);
}


osgGeo::MarkerShape::ShapeType MarkerSet::getShape()
{
    return _markerShape.getType();
}


const float MarkerSet::getMarkerSize()
{
    return _useScreenSize ? _markerShape.getSize() : -1;
}


void MarkerSet::setMarkerHeightRatio(float markerHeightRatio)
{
    _markerShape.setHeightRatio(markerHeightRatio);
    forceRedraw(true);
}


const float MarkerSet::getMarkerHeightRatio()
{
    return _markerShape.getHeightRatio();
}


void MarkerSet::setDetail(float ratio)
{
    _markerShape.setDetail(ratio);
    forceRedraw(true);
}

const float MarkerSet::getDetail()
{
    return _markerShape.getDetail();
}



void MarkerSet::setMinScale(float minScale)
{
    _minScale = minScale;
    forceRedraw(true);
}


void MarkerSet::setMaxScale(float maxScale)
{
    _maxScale = maxScale;
    forceRedraw(true);
}


void MarkerSet::setRotateMode( osg::AutoTransform::AutoRotateMode rtmode )
{
    _rotateMode = rtmode;
    forceRedraw(true);
}


void MarkerSet::setColorArray(osg::Vec4Array* colorArr)
{
    _colorArr = colorArr;
    forceRedraw(true);
}


void MarkerSet::setMarkerSize(float markerSize, bool useScreenSize)
{
    _markerShape.setSize(markerSize);
    _useScreenSize = useScreenSize;
    forceRedraw(true);
}


void MarkerSet::setSingleColor(osg::Vec4& singleColor)
{
    _singleColor = singleColor;
}

void MarkerSet::useSingleColor(bool applySingleColor)
{
    _applySingleColor = applySingleColor;
}
