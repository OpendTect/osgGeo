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

$Id$

*/


#include <osgGeo/PolygonSelection>

#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Material>
#include <osgUtil/CullVisitor>


namespace osgGeo
{

class PolygonSelectionEventHandler : public osgGA::GUIEventHandler
{
public:
    void setPolygonSelector(osgGeo::PolygonSelection* ps)
    { _polysel = ps; }
    
protected:
    bool handle ( const osgGA::GUIEventAdapter &ea,
						osgGA::GUIActionAdapter&,
						osg::Object*,
						osg::NodeVisitor * )
    {
	return _polysel->handleEvent( ea );
    }
    osg::ref_ptr<osgGeo::PolygonSelection>	    _polysel;
};


PolygonSelection::PolygonSelection()
    : _geode(new osg::Geode)
    , _ison(true)
    , _linegeometry(new osg::Geometry)
    , _coords(new osg::Vec3Array) 
    , _coordslist(new osg::DrawArrays(GL_LINE_LOOP))
    , _shapetype(Rectangle)
    , _callback(0)
    , _material(0)
    , _zcoord(0)
{
    _linegeometry->setVertexArray( _coords );
    _linegeometry->addPrimitiveSet( _coordslist );
    _geode->addDrawable( _linegeometry );
    setColor( osg::Vec4(1,0,0,1) );
}


PolygonSelection::~PolygonSelection()
{
}


osgGA::GUIEventHandler* PolygonSelection::createEventHandler()
{
    PolygonSelectionEventHandler* res = new PolygonSelectionEventHandler;
    res->setPolygonSelector( this );
    return res;
}


void PolygonSelection::setLatestMousePoints( const osg::Vec3& pos )
{   
    if ( _shapetype == Rectangle )
    {
	(*_coords)[1] = osg::Vec3( pos.x(), _coords->at(0).y(), pos.z() );
	(*_coords)[2] = pos; 
	(*_coords)[3] = osg::Vec3( _coords->at(0).x(), pos.y(), pos.z() );
	_coordslist->setCount( _coords->size() );
    }
    else
    {
	_coords->push_back( pos );
	_coordslist->setCount( _coords->size() );
    }

    _linegeometry->dirtyDisplayList();
    dirtyBound();
}


void PolygonSelection::accept( osg::NodeVisitor& nv )
{
    osg::ref_ptr<osg::StateSet> stateset = getStateSet();
    osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*>(&nv);
    if ( stateset && cv )
	cv->pushStateSet( stateset );

    _geode->accept( nv );

    if ( stateset && cv )
	cv->popStateSet();
}


osg::BoundingSphere PolygonSelection::computeBound() const
{
    return _geode->computeBound();
}


void PolygonSelection::setColor( const osg::Vec4& color )
{
   osg::StateSet* stateset = getOrCreateStateSet();
   stateset->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
   if ( !_material )
       _material = new osg::Material;
   _material->setDiffuse( osg::Material::FRONT, color );
   stateset->setAttribute( _material );
}


bool PolygonSelection::handleEvent( const osgGA::GUIEventAdapter& ea )
				                   
{
    if ( !_ison )
	return false;
    
    const osg::Vec3 mousepos( ea.getX(), ea.getY(), _zcoord );
    if ( ea.getEventType()==osgGA::GUIEventAdapter::PUSH 
	&& ea.getButton()==osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON  )
    {
	_coords->clear();
	if ( _shapetype == Rectangle )
	{
	    _coords->resize( 4 );
	    (*_coords)[0] = mousepos;
	}
	return true;
    }
    if ( ea.getEventType()==osgGA::GUIEventAdapter::DRAG
	&& ea.getButtonMask()==osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON )
    {
	setLatestMousePoints ( mousepos );
	return true;
    }
    if ( ea.getEventType()==osgGA::GUIEventAdapter::RELEASE 
	&& ea.getButton()==osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON )
    {
	if ( _callback )
	{
	    osg::NodeVisitor nv;
	    ( *_callback )( this, &nv );
	}
	return true;
    }
   
    return false;
}


void PolygonSelection::setShapeType( ShapeType st )
{
    _shapetype = st;
}


void PolygonSelection::setZCoord( float zcoord )
{
    _zcoord = zcoord;
}


void PolygonSelection::addCallBack( osg::NodeCallback *nc )
{
    if ( !_callback )
	_callback = nc;
    else
	_callback->addNestedCallback( nc );
}


void PolygonSelection::removeCallBack( osg::NodeCallback *nc )
{
    if ( nc == _callback )
	_callback = _callback->getNestedCallback();
    else
	_callback->removeNestedCallback( nc );
}

} // osgGeo


