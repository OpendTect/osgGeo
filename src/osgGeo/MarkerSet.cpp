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
#include <osg/Material>
#include <osgGeo/ComputeBoundsVisitor>
#include <osgUtil/CullVisitor>
#include <OpenThreads/ScopedLock>


using namespace osgGeo;

static std::vector<std::pair<MarkerSet*,osg::ref_ptr<osg::ByteArray> > > _onoffByteArr;
static OpenThreads::Mutex _onoffLock; // only protect _onoffByteArr

MarkerSet::MarkerSet()
    : _rotateMode(osg::AutoTransform::ROTATE_TO_SCREEN)
    , _colorArr(new osg::Vec4Array)
    , _vertexArr(new osg::Vec3Array)
    , _nonShadingSwitch(new osg::Switch)
    , _minScale(0.0f)
    , _maxScale(FLT_MAX)
    , _normalArr(new osg::Vec3Array)
    , _applySingleColor(false)
    , _forceRedraw(false)
    , _waitForAutoTransformUpdate(false)
    , _applyRotationForAll(true)
{
    _onoffLock.lock();
    _onoffByteArr.push_back(std::pair<MarkerSet*,osg::ByteArray*>(this,0));
    _onoffLock.unlock();
    setNumChildrenRequiringUpdateTraversal(0);
    _singleColor = osg::Vec4(0.1f, 0.1f, 0.1f, 1.0f);
    _bbox.init();
}


MarkerSet::~MarkerSet()
{
    _onoffLock.lock();
    for ( int idx=_onoffByteArr.size()-1; idx>=0; idx-- )
    {
	if ( this == _onoffByteArr[idx].first )
	{
	    _onoffByteArr[idx].second->unref();
	    _onoffByteArr.erase(_onoffByteArr.begin()+idx);
	}
    }
    _onoffLock.unlock();
}


void MarkerSet::applyRotationToAllMarkers( bool yn )
{
    _applyRotationForAll = yn;
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
	const osg::Vec3 eyePoint = dynamic_cast<osgUtil::CullVisitor*>(&nv)->getEyePoint();

	if ( _waitForAutoTransformUpdate || _prevEyePoint!=eyePoint )
	{
	    _prevEyePoint = eyePoint;
	    _waitForAutoTransformUpdate = false;
	    dirtyBound();
	    if ( !_useScreenSize || _nonShadingSwitch->getNumChildren()!=1 )
	    {
		setCullingActive(true);
		_nonShadingSwitch->setCullingActive(true);
	    }
	}
    }
    else
    {
	osgGeo::ComputeBoundsVisitor* cbv =
	    dynamic_cast<osgGeo::ComputeBoundsVisitor*>( &nv );
	if ( cbv )
	{
	    cbv->applyBoundingBox(_bbox);
	    return;
	}
    }

    if ( _nonShadingSwitch )
	_nonShadingSwitch->accept( nv );
}


bool MarkerSet::updateShapes()
{
    if ( !_vertexArr ) return false;

    _nonShadingSwitch->removeChildren(0, _nonShadingSwitch->getNumChildren());

    osg::ref_ptr<osg::Material> material = new osg::Material;
    material->setColorMode(osg::Material::AMBIENT_AND_DIFFUSE);

    for (unsigned int idx=0;idx<_vertexArr->size();idx++)
    {
	const osg::Vec4 color = _applySingleColor || !_colorArr
	    ? _singleColor
	    : idx<_colorArr->size()
	        ? _colorArr->at( idx )
		: _colorArr->back();

	const osg::Quat& rot = _applyRotationForAll
	    ? _rotationForAllMarkers
	    : idx < _rotationSet.size()
		? _rotationSet.at(idx)
		: osg::Quat();

	osg::ref_ptr<osg::Drawable> drwB = _markerShape.createShape( color, rot );

	osg::ref_ptr<osg::Geode> geode = new osg::Geode;
	geode->addDrawable(drwB);
	osg::StateSet* state = geode->getOrCreateStateSet();

	state->setAttributeAndModes(material,osg::StateAttribute::PROTECTED|osg::StateAttribute::ON);
	state->setMode(GL_RESCALE_NORMAL, osg::StateAttribute::ON);
	state->setMode(GL_LIGHTING, osg::StateAttribute::ON);

	osg::ref_ptr<osg::AutoTransform> autotrans =
	    new osg::AutoTransform;
	autotrans->setPosition(_vertexArr->at(idx));
	autotrans->setAutoRotateMode(_rotateMode);
	autotrans->setAutoScaleToScreen(true);
	autotrans->setMinimumScale(1.0f);
	autotrans->setMaximumScale(DBL_MAX);
	autotrans->setAutoScaleTransitionWidthRatio(0.5);

	if ( !_useScreenSize )
	    autotrans->setScale( (double)1.0 );
	else
	{
	    autotrans->setMinimumScale(_minScale);
	    autotrans->setMaximumScale(_maxScale);
	}
	const osg::ByteArray* arr = getOnOffArray();
	if ( !arr )
	    return false;
	const bool ison = idx < arr->size() ? arr->at(idx) : true;
	autotrans->addChild(geode);
	_nonShadingSwitch->addChild( autotrans, ison );
    }

    // In case of autoscale to screen, new AutoTransforms cannot compute
    // their bounding spheres till after their first cull traversal.
    if ( _useScreenSize )
    {
	_waitForAutoTransformUpdate = true;
	setCullingActive(false);
	_nonShadingSwitch->setCullingActive(false);
    }

    return true;
}


void MarkerSet::turnMarkerOn(unsigned int idx,bool yn)
{
    osg::ByteArray* arr = getOnOffArray();
    if ( !arr )
	return;

    if( idx>=arr->size())
	arr->resize(idx+1);

    (*arr)[idx] = yn;

    if ( idx<_nonShadingSwitch->getNumChildren() )
	_nonShadingSwitch->setChildValue(_nonShadingSwitch->getChild(idx), yn);
    else
	forceRedraw( true );
}


bool MarkerSet::markerOn(unsigned int idx) const
{
    osg::ByteArray* arr = getOnOffArray();
    if (!arr)
	return false;

    if ( idx<arr->size() )
	return (*arr)[idx];

    return false;
}


void MarkerSet::removeMarkerOnOff(unsigned int idx)
{}


void MarkerSet::turnAllMarkersOn(bool yn)
{
    osg::ByteArray* arr = getOnOffArray();
    if (!arr || arr->size()==0 )
	return;
    memset( &(*arr)[0], yn, arr->size()*sizeof(bool) );

    if ( yn )
	_nonShadingSwitch->setAllChildrenOn();
    else
	_nonShadingSwitch->setAllChildrenOff() ;
}


osg::BoundingSphere MarkerSet::computeBound() const
{
    osg::BoundingBox bbox;
    bbox.init();

    for ( unsigned int idx=0; idx<_nonShadingSwitch->getNumChildren(); idx++ )
	bbox.expandBy( _nonShadingSwitch->getChild(idx)->getBound() );

    _bbox = bbox;
    return _bbox;
}


void MarkerSet::forceRedraw(bool yn)
{
    if ( yn == _forceRedraw )
	return;

    setNumChildrenRequiringUpdateTraversal(
        _nonShadingSwitch->getNumChildrenRequiringUpdateTraversal()+((int) yn));
    _forceRedraw = yn;
}


void MarkerSet::setVertexArray(osg::Vec3Array* arr)
{
    if (arr==_vertexArr)
	return;
    _vertexArr = arr;
    forceRedraw(true);
}


void MarkerSet::setOnOffArray( osg::ByteArray* arr )
{
    OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_onoffLock);
    for ( size_t idx = 0; idx<_onoffByteArr.size(); idx++)
    {
	if (_onoffByteArr[idx].first==this)
	{
	    arr->ref();
	    _onoffByteArr[idx].second = arr;
	    break;
	}
    }
    
    forceRedraw(true);
}


osg::ByteArray*	MarkerSet::getOnOffArray() const
{
    OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_onoffLock);
    for ( size_t idx=0; idx<_onoffByteArr.size(); idx++ )
    {
	if ( _onoffByteArr[idx].first==this )
	{
	    return _onoffByteArr[idx].second;
	}
    }
    return 0;
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


float MarkerSet::getMarkerSize()
{
    return _useScreenSize ? _markerShape.getSize() : -1;
}


void MarkerSet::setMarkerHeightRatio(float markerHeightRatio)
{
    _markerShape.setHeightRatio(markerHeightRatio);
    forceRedraw(true);
}


float MarkerSet::getMarkerHeightRatio()
{
    return _markerShape.getHeightRatio();
}


void MarkerSet::setRotationForAllMarkers(const osg::Quat& quat)
{
    _rotationForAllMarkers = quat;
}


const osg::Quat& MarkerSet::getRotationForAllMarkers() const
{
    return _rotationForAllMarkers;
}


void MarkerSet::setSingleMarkerRotation( const osg::Quat& rot, int idx )
{
    if ( idx >= (int)_rotationSet.size() )
	_rotationSet.resize( idx+1 );

    _rotationSet[idx] = rot;
}


void MarkerSet::setDetail(float ratio)
{
    _markerShape.setDetail(ratio);
    forceRedraw(true);
}


float MarkerSet::getDetail()
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

