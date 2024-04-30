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


#include <osgGeo/PolygonSelection>
#include <osgGeo/ComputeBoundsVisitor>
#include <osg/Camera>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Material>
#include <osg/Version>
#include <osgUtil/CullVisitor>
#include <osgUtil/LineSegmentIntersector>
#include <osgViewer/View>

namespace osgGeo
{

static OpenThreads::Mutex _cloneMutex;
class PolygonSelectionEventHandler : public osgGA::GUIEventHandler
{
public:
    PolygonSelectionEventHandler()		{}
    void setPolygonSelector(osgGeo::PolygonSelection* ps)
    { _polysel = ps; }

protected:
    bool handle(const osgGA::GUIEventAdapter &ea,
					osgGA::GUIActionAdapter&,
					osg::Object*,
					osg::NodeVisitor*) override
    {
	return _polysel->handleEvent(ea);
    }

    bool handle( const osgGA::GUIEventAdapter &ea,
	    	 osgGA::GUIActionAdapter& aa ) override
    {
	return osgGA::GUIEventHandler::handle(ea,aa);
    }

#if OSG_MIN_VERSION_REQUIRED(3,3,1)
    bool handle(osgGA::Event* ev,osg::Object* obj,osg::NodeVisitor* nv) override
    {
	return osgGA::GUIEventHandler::handle(ev,obj,nv);
    }
#endif

    osgGeo::PolygonSelection*		_polysel;
};


PolygonSelection::PolygonSelection()
    : _geode(new osg::Geode)
    , _ison(true)
    , _lineGeometry(new osg::Geometry)
    , _coords(new osg::Vec3Array)
    , _coordsList(new osg::DrawArrays(GL_LINE_LOOP))
    , _shapeType(Polygon)
    , _callback(0)
    , _material(0)
    , _zcoord(0)
    , _masterCamera(0)
    , _eventHandler(0)
    , _isDrawing(false)
    , _hudCamera(0)
{
    _lineGeometry->setVertexArray(_coords);
    _lineGeometry->addPrimitiveSet(_coordsList);
    _geode->addDrawable(_lineGeometry);
    setColor(osg::Vec4(1,0,0,1));
    _bbox.init();
}


PolygonSelection::PolygonSelection(const PolygonSelection& sel,const osg::CopyOp& op)
    : _geode(0)
    , _ison(true)
    , _lineGeometry(0)
    , _coords(0)
    , _coordsList(0)
    , _shapeType(Polygon)
    , _callback(0)
    , _material(0)
    , _zcoord(0)
    , _masterCamera(0)
    , _eventHandler(0)
    , _isDrawing(false)
    , _hudCamera(0)
{
    _cloneMutex.lock();
    _geode = (osg::Geode*)sel._geode->clone(op);
    _lineGeometry = (osg::Geometry*)sel._lineGeometry->clone(op);
    _coords = (osg::Vec3Array*)sel._coords->clone(op);
    _coordsList = (osg::DrawArrays*)sel._coordsList->clone(op);
    _masterCamera = (osg::Camera*)sel._masterCamera;
    _hudCamera = (osg::Camera*)sel._hudCamera->clone(op);

    if (_masterCamera)
	_masterCamera->ref();
    if (_hudCamera)
	_hudCamera->ref();

    _cameraPos = sel._cameraPos;
    _cloneMutex.unlock();
}


PolygonSelection::~PolygonSelection()
{
    setEventHandlerCamera( 0 );
    setHUDCamera( 0 );
    if ( _eventHandler )
    {
	_eventHandler->setPolygonSelector( 0 );
	_eventHandler->unref();
    }
}


bool PolygonSelection::setEventHandlerCamera( osg::Camera* camera, bool handleAfterScene )
{
    if ( _masterCamera )
    {
	_masterCamera->removeEventCallback( _eventHandler );

	osgViewer::View* view = dynamic_cast<osgViewer::View*>(_masterCamera->getView());
	if ( view && view->getSceneData() )
	    view->getSceneData()->removeEventCallback( _eventHandler );

	_masterCamera->unref();
    }

    _masterCamera = camera;

    if ( _masterCamera )
    {
	_masterCamera->ref();

	if ( !_eventHandler )
	{
	    _eventHandler = new PolygonSelectionEventHandler;
	    _eventHandler->ref();
	    _eventHandler->setPolygonSelector(this);
	}

	if ( handleAfterScene )
	    _masterCamera->addEventCallback( _eventHandler );
	else
	{
	    osgViewer::View* view = dynamic_cast<osgViewer::View*>(_masterCamera->getView());
	    if ( !view || !view->getSceneData() )
		return false;

	    view->getSceneData()->addEventCallback( _eventHandler );
	}
    }

    return true;
}


void PolygonSelection::setHUDCamera( osg::Camera* hudCamera )
{
    if ( _hudCamera )
	_hudCamera->unref();

    _hudCamera = hudCamera;

    if ( _hudCamera )
	_hudCamera->ref();
}


void PolygonSelection::accept(osg::NodeVisitor& nv)
{
    osg::ref_ptr<osg::StateSet> stateset = getStateSet();
    osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*>(&nv);
    if (stateset && cv)
	cv->pushStateSet(stateset);

    osgGeo::ComputeBoundsVisitor* cbv =
	dynamic_cast<osgGeo::ComputeBoundsVisitor*>( &nv );
    if ( cbv )
	cbv->applyBoundingBox(_bbox);
    else
	_geode->accept(nv);

    if (stateset && cv)
	cv->popStateSet();

}


bool PolygonSelection::checkInteractionObjectIntersection(
						    const osg::Vec3& pos ) const
{
    if ( !_hudCamera )
	return false;

    osg::ref_ptr<osgUtil::LineSegmentIntersector> intersector =
	new osgUtil::LineSegmentIntersector(
			       osgUtil::Intersector::WINDOW, pos.x(), pos.y() );
    osgUtil::IntersectionVisitor iv( intersector.get() );
    _hudCamera->accept( iv );
    return intersector->containsIntersections();
}


bool PolygonSelection::isCameraChanged()
{
    bool isChanged = false;
    const osg::Vec3 newPos = _masterCamera->getViewMatrix().getTrans();
    if ( newPos != _cameraPos )
    {
	_cameraPos = newPos;
	isChanged = true;
    }

    return isChanged;
}


bool PolygonSelection::handleEvent(const osgGA::GUIEventAdapter& ea)
{
    if ( !_ison || _shapeType==Off )
	return false;

    if ( isCameraChanged() )
    {
	clear();
	return false;
    }

    const osg::Vec3 mousepos(ea.getX(),ea.getY(),_zcoord);

    if (ea.getEventType()==osgGA::GUIEventAdapter::PUSH &&
	ea.getButton()==osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON)
    {
	clear();

	if ( ea.getHandled() )
	    return false;
	if ( checkInteractionObjectIntersection(mousepos) )
	    return false;

	_isDrawing = true;

	if ( _shapeType == Rectangle )
	{
	    _coords->resize(4);
	    (*_coords)[0] = mousepos;
	}
	setLatestMousePoints(mousepos);
	return true;
    }

    if ( !_isDrawing )
	return false;

    if (ea.getEventType()==osgGA::GUIEventAdapter::DRAG
	&& ea.getButtonMask()==osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON)
    {
	setLatestMousePoints (mousepos);
	return true;
    }

    if (ea.getEventType()==osgGA::GUIEventAdapter::RELEASE
	&& ea.getButton()==osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON)
    {
	_isDrawing = false;
	bool ispolygon = _coords->size()>2;
	osg::NodeCallback* nodecb = dynamic_cast<osg::NodeCallback*>(
							    _callback.get() );
	if ( nodecb )
	{
	    osg::NodeVisitor nv;
	    (*nodecb)( this, &nv );
	}
	return ispolygon;
    }

    return false;
}


void PolygonSelection::setLatestMousePoints(const osg::Vec3& pos)
{
    osg::BoundingBox bbox;
    bbox.init();

    if (_shapeType == Rectangle)
    {
	(*_coords)[1] = osg::Vec3(pos.x(),_coords->at(0).y(),pos.z());
	(*_coords)[2] = pos;
	(*_coords)[3] = osg::Vec3(_coords->at(0).x(),pos.y(),pos.z());
	_coordsList->setCount(_coords->size());
    }
    else
    {
	_coords->push_back(pos);
	_coordsList->setCount(_coords->size());
    }


    for ( unsigned int idx=0; idx<_coords->size(); idx++ )
	bbox.expandBy((*_coords)[idx]);

    if ( _bbox._min != bbox._min || _bbox._max!= bbox._max )
    {
	_bbox = bbox;
	dirtyBound();
    }

    _lineGeometry->dirtyGLObjects();

}


osg::Vec2 PolygonSelection::projectPointToScreen(osg::Vec3 pointin3d) const
{
    osg::Vec2 pointin2d( 1e30, 1e30 );
    PolygonSelection::projectPointToScreen( pointin3d, pointin2d );
    return pointin2d;
}


bool PolygonSelection::projectPointToScreen(const osg::Vec3& pointIn3d,osg::Vec2& pointIn2d) const
{
    if ( !_masterCamera )
	return false;

     const osg::Matrix vpwMatrix(_masterCamera->getViewMatrix() *
			   _masterCamera->getProjectionMatrix() *
			   _masterCamera->getViewport()->computeWindowMatrix());
     const osg::Vec3 projectedPoint = pointIn3d * vpwMatrix;

     if ( projectedPoint.z() > 1.0 )
	 return false;

     pointIn2d = osg::Vec2( projectedPoint.x(), projectedPoint.y() );
     return true;
}


osg::BoundingSphere PolygonSelection::computeBound() const
{
    if ( _bbox.valid() )
	return _bbox;

    osg::BoundingBox bbox;

    for ( unsigned int idx=0; idx<_coords->size(); idx++ )
	bbox.expandBy((*_coords)[idx]);

    return bbox;
}


void PolygonSelection::clear()
{
    if ( _coordsList->getCount()>0 )
    {
	_coords->clear();
	_coordsList->setCount(0);
	_lineGeometry->dirtyGLObjects();
	_isDrawing = false;
    }
}


void PolygonSelection::setColor(const osg::Vec4& color)
{
   osg::StateSet* stateset = getOrCreateStateSet();
   stateset->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
   if ( !_material )
       _material = new osg::Material;
   _material->setDiffuse(osg::Material::FRONT, color);
   stateset->setAttribute(_material);
}


void PolygonSelection::setShapeType(ShapeType st)
{
    _shapeType = st;
    dirtyBound();
}


void PolygonSelection::setZCoord(float zcoord)
{
    _zcoord = zcoord;
}


void PolygonSelection::addCallBack(osg::NodeCallback *nc)
{
    if (!_callback)
	_callback = nc;
    else
	_callback->addNestedCallback(nc);
}


void PolygonSelection::removeCallBack(osg::NodeCallback *nc)
{
    if ( nc == _callback )
	_callback = _callback->getNestedCallback();
    else
	_callback->removeNestedCallback(nc);
}

} // osgGeo
