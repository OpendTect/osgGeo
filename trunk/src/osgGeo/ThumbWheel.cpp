/* -*-c++-*- OpenSceneGraph - Copyright (C) 1998-2006 Robert Osfield
 *
 * This library is open source and may be redistributed and/or modified under
 * the terms of the OpenSceneGraph Public License (OSGPL) version 0.0 or
 * (at your option) any later version.  The full license is in LICENSE file
 * included with this distribution, and on the openscenegraph.org website.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * OpenSceneGraph Public License for more details.
*/

#include <osgGeo/ThumbWheel>



//#include <osg/ShapeDrawable>
#include <osg/Geometry>
#include <osg/Geode>
#include <osg/Vec4ub>
#include <osgGA/EventVisitor>
#include <osg/Texture2D>
//#include <osg/Material>

using namespace osgGeo;

#define TEXUNIT 0
#define RESOLUTION 10
#define TEXTURELENGTH (30/180*M_PI)

ThumbWheel::ThumbWheel()
    : _geode( new osg::Geode )
    , _istracking( false )
    , _currentangle( 0 )
{
    _geode->ref();
    
    osg::Geometry* geometry = new osg::Geometry;
    _geode->addDrawable( geometry );
    geometry->setVertexArray( new osg::Vec3Array );
    geometry->setTexCoordArray( TEXUNIT, new osg::Vec2Array );
    
    setShape( 0, osg::Vec2( 200, 200 ), osg::Vec2( 400, 400 ) );
    setAngle( 0 );
}



ThumbWheel::~ThumbWheel()
{
    _geode->unref();
}


void ThumbWheel::setShape( short dim, const osg::Vec2& min,const osg::Vec2& max)
{
    _istracking = false;
    _dim = dim;
    _min = min; _max = max;
    osg::Geometry* geom = (osg::Geometry*) _geode->getDrawable( 0 );
    osg::Vec3Array* arr = (osg::Vec3Array*) geom->getVertexArray();
    osg::Vec2Array* tcarr = (osg::Vec2Array*) geom->getTexCoordArray( TEXUNIT );
    
    const int resolution = 10;
    const float anglestep = M_PI/(resolution-1);
    const float wheelradius = (_max[_dim]-_min[_dim])/2;
    const float wheelcenter = (_max[_dim]+_min[_dim])/2;
    
    const short dim2 = _dim ? 0 : 1;
    
    for ( int idx=arr->size(); idx<RESOLUTION*2; idx++ )
    {
	arr->push_back( osg::Vec3() );
	tcarr->push_back( osg::Vec2() );
    }
    
    const float texturelength = TEXTURELENGTH;
    for ( int idx=0; idx<RESOLUTION; idx++ )
    {
	const float angle = anglestep * idx;
	osg::Vec3 v0, v1;
	v0[2] = v1[2] = 0;
	v0[_dim] = v1[_dim] = wheelcenter+wheelradius*cos(angle);
	v0[dim2] = _min[dim2];
	v1[dim2] = _max[dim2];
	(*arr)[idx*2] = v0;
	(*arr)[idx*2+1] = v1;
	const float tc = angle/texturelength;
	(*tcarr)[idx*2] = osg::Vec2( tc, 0 );
	(*tcarr)[idx*2+1] = osg::Vec2( tc, 1 );
    }
    
    if ( !geom->getNumPrimitiveSets() )
    {
	geom->addPrimitiveSet( new osg::DrawArrays( GL_TRIANGLE_STRIP,0, resolution*2) );
    }
}


void ThumbWheel::setAngle( float angle )
{
    float diff = _currentangle-angle;
    if ( diff==0 )
	return;
    
    _currentangle = angle;
    
    osg::Geometry* geom = (osg::Geometry*) _geode->getDrawable( 0 );
    osg::Vec2Array* tcarr = (osg::Vec2Array*) geom->getTexCoordArray( TEXUNIT );
    
    const float increment = diff/TEXTURELENGTH;
    for ( int idx=0; idx<RESOLUTION; idx++ )
    {
	(*tcarr)[idx*2][0] += increment;
	(*tcarr)[idx*2+1][0] += increment;
    }
}


void ThumbWheel::accept( osg::NodeVisitor& nv )
{
    return _geode->accept( nv );
}


osg::BoundingSphere ThumbWheel::computeBound() const
{
    return _geode->computeBound();
}

char ThumbWheel::getMousePosStatus(const osg::Vec2& mousepos ) const
{
    if ( mousepos[0]<_min[0] || mousepos[0]>_max[0] )
	return 0;
    
    if ( mousepos[1]<_min[1] || mousepos[1]>_max[1] )
	return 0;
    
    return 2;
}


bool ThumbWheel::handleEvent( const osgGA::GUIEventAdapter& ea )
{
    if ( !_istracking )
    {
	if ( ea.getEventType()==osgGA::GUIEventAdapter::PUSH &&
	     ea.getButton()==1 )
	{
	    const osg::Vec2 mousepos( ea.getX(), ea.getY() );
	    if ( getMousePosStatus( mousepos )==2 )
	    {
		_istracking = true;
		_startpos = mousepos[_dim];
		return true;
	    }
	}
	else if ( ea.getEventType()==osgGA::GUIEventAdapter::SCROLL )
	{
	    //TODO
	    return false;
	}
	
	return false;
    }
    
    if ( ea.getEventType()==osgGA::GUIEventAdapter::RELEASE && ea.getButton()==1)
    {
	_istracking = false;
	return true;
    }
    
    if ( ea.getEventType()==osgGA::GUIEventAdapter::RESIZE ||
	ea.getEventType()==osgGA::GUIEventAdapter::DOUBLECLICK )
    {
	//Just quit
	_istracking = false;
	return false;
    }
    
    if ( ea.getEventType()==osgGA::GUIEventAdapter::MOVE )
    {
	const osg::Vec2 mousepos( ea.getX(), ea.getY() );
	const float movement = mousepos[_dim] - _startpos;
	const float diffangle = movement * 2 / (_max[_dim]-_min[_dim]);
	setAngle( _currentangle + diffangle );
    }
        
    return true;
}


bool ThumbWheelEventHandler::handle (const osgGA::GUIEventAdapter &ea,
				     osgGA::GUIActionAdapter&,
				     osg::Object*,
				     osg::NodeVisitor *)
{
    bool handled = false;
    for ( std::vector<osg::ref_ptr<ThumbWheel> >::iterator iter=_thumbwheels.begin();
	 iter!=_thumbwheels.end(); ++iter )
    {
	if ( iter->get()->handleEvent( ea ) )
	    handled = true;
    }
    
    return handled;
}
