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


#include <osg/Geometry>
#include <osg/Geode>
#include <osg/Vec4ub>
#include <osgGA/EventVisitor>
#include <osg/Image>
#include <osg/Texture2D>


using namespace osgGeo;

#define DEGREESPERTICK 20.0f
#define TEXUNIT 0
#define RESOLUTION 10
#define IMAGEHEIGHT 8
#define IMAGEWIDTH  8

unsigned char imagedata[] =
    {   255, 255, 255, 255,
	255, 255, 255, 255,
	255, 255, 255, 255,
	255, 255, 255, 255,
	255, 255, 255, 255,
	255, 255, 255, 255,
	255, 255, 255, 255,
	255, 255, 255, 255,
	
	255, 255, 255, 255,
	0, 0, 0, 255,
	0, 0, 0, 255,
	0, 0, 0, 255,
	0, 0, 0, 255,
	0, 0, 0, 255,
	0, 0, 0, 255,
	255, 255, 255, 255,
	
	255, 255, 255, 255,
	0, 0, 0, 255,
	0, 0, 0, 255,
	0, 0, 0, 255,
	0, 0, 0, 255,
	0, 0, 0, 255,
	0, 0, 0, 255,
	255, 255, 255, 255,
	
	255, 255, 255, 255,
	0, 0, 0, 255,
	0, 0, 0, 255,
	0, 0, 0, 255,
	0, 0, 0, 255,
	0, 0, 0, 255,
	0, 0, 0, 255,
	255, 255, 255, 255,
	
	255, 255, 255, 255,
	0, 0, 0, 255,
	0, 0, 0, 255,
	0, 0, 0, 255,
	0, 0, 0, 255,
	0, 0, 0, 255,
	0, 0, 0, 255,
	255, 255, 255, 255,

	255, 255, 255, 255,
	0, 0, 0, 255,
	0, 0, 0, 255,
	0, 0, 0, 255,
	0, 0, 0, 255,
	0, 0, 0, 255,
	0, 0, 0, 255,
	255, 255, 255, 255,

	255, 255, 255, 255,
	0, 0, 0, 255,
	0, 0, 0, 255,
	0, 0, 0, 255,
	0, 0, 0, 255,
	0, 0, 0, 255,
	0, 0, 0, 255,
	255, 255, 255, 255,
	
	255, 255, 255, 255,
	0, 0, 0, 255,
	0, 0, 0, 255,
	0, 0, 0, 255,
	0, 0, 0, 255,
	0, 0, 0, 255,
	0, 0, 0, 255,
	255, 255, 255, 255
    };

ThumbWheel::ThumbWheel()
    : _geode( new osg::Geode )
    , _istracking( false )
    , _currentangle( 0 )
{
    _geode->ref();
    
    _wheelgeometry = new osg::Geometry;
    _wheelgeometry->ref();
    _wheelgeometry->setVertexArray( new osg::Vec3Array );
    _wheelgeometry->setNormalArray( new osg::Vec3Array );
    _wheelgeometry->setTexCoordArray( TEXUNIT, new osg::Vec2Array );
    _wheelgeometry->setNormalBinding( osg::Geometry::BIND_PER_VERTEX );
    
    osg::Image* image = new osg::Image();
    image->setImage( IMAGEHEIGHT, IMAGEWIDTH, 1, GL_RGBA, GL_RGBA,
		    GL_UNSIGNED_BYTE, imagedata, osg::Image::NO_DELETE );
    osg::Texture2D* texture = new osg::Texture2D;
    texture->setWrap( osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE );
    texture->setWrap( osg::Texture::WRAP_T, osg::Texture::REPEAT );

    texture->setImage( image );
    _wheelgeometry->getOrCreateStateSet()->setTextureAttributeAndModes( TEXUNIT, texture );
    
    _outlinegeometry = new osg::Geometry;
    _geode->addDrawable( _outlinegeometry );
    _outlinegeometry->setVertexArray( _wheelgeometry->getVertexArray() );

    osg::ref_ptr<osg::Vec3Array> outlineNormals = new osg::Vec3Array;
    outlineNormals->push_back( osg::Vec3(0.0f,0.0f,1.0f) );
    _outlinegeometry->setNormalArray( outlineNormals.get() );
    _outlinegeometry->setNormalBinding( osg::Geometry::BIND_OVERALL );
    
    setShape( 0, osg::Vec2( 200, 200 ), osg::Vec2( 400,  250 ), 0 );
    setAngle( 0 );
}



ThumbWheel::~ThumbWheel()
{
    _geode->unref();
    _wheelgeometry->unref();
}


void ThumbWheel::setShape( short dim, const osg::Vec2& min,const osg::Vec2& max,
			   float zval )
{
    _istracking = false;
    _dim = dim;
    _min = min; _max = max;
    osg::Vec3Array* varr = (osg::Vec3Array*) _wheelgeometry->getVertexArray();
    osg::Vec3Array* narr = (osg::Vec3Array*) _wheelgeometry->getNormalArray();
    osg::Vec2Array* tcarr = (osg::Vec2Array*) _wheelgeometry->getTexCoordArray( TEXUNIT );
    
    const int resolution = 10;
    const float anglestep = M_PI/(resolution-1);
    const float wheelradius = (_max[_dim]-_min[_dim])/2;
    const float wheelcenter = (_max[_dim]+_min[_dim])/2;
    
    const short dim2 = _dim ? 0 : 1;
    
    for ( int idx=varr->size(); idx<RESOLUTION*2; idx++ )
    {
	varr->push_back( osg::Vec3() );
	narr->push_back( osg::Vec3() );
	tcarr->push_back( osg::Vec2() );
    }
    
    const float degreespertick = DEGREESPERTICK;
    const float radspertick = (degreespertick/180*M_PI);
    for ( int idx=0; idx<RESOLUTION; idx++ )
    {
	const float angle = anglestep * idx;
	osg::Vec3 v0, v1, normal;
	v0[2] = v1[2] = zval;
	v0[_dim] = v1[_dim] = wheelcenter+wheelradius*cos(angle);
	v0[dim2] = _min[dim2];
	v1[dim2] = _max[dim2];
	normal[_dim] = cos(angle);
	normal[2] = sin(angle);
	normal[dim2] = 0;
	(*varr)[idx*2] = v0;
	(*varr)[idx*2+1] = v1;
	const float tc = angle/radspertick;
	(*tcarr)[idx*2] = osg::Vec2( 0, tc );
	(*tcarr)[idx*2+1] = osg::Vec2( 1, tc );
	(*narr)[idx*2] = (*narr)[idx*2+1] = normal;
    }
    
    if ( !_wheelgeometry->getNumPrimitiveSets() )
    {
	_wheelgeometry->addPrimitiveSet( new osg::DrawArrays( GL_TRIANGLE_STRIP,0, resolution*2) );
    }
    
    if ( !_outlinegeometry->getNumPrimitiveSets() )
    {
	osg::DrawElementsUByte* primitive = new osg::DrawElementsUByte( GL_LINE_STRIP );
	primitive->push_back( 0 );
	primitive->push_back( 1 );
	primitive->push_back( varr->size()-1 );
	primitive->push_back( varr->size()-2 );
	primitive->push_back( 0 );
	_outlinegeometry->addPrimitiveSet( primitive );
    }
    
    _wheelgeometry->dirtyDisplayList();
    _outlinegeometry->dirtyDisplayList();
    dirtyBound();
}


void ThumbWheel::setAngle( float angle )
{
    float diff = angle-_currentangle;
    if ( diff==0 )
	return;
    
    _currentangle = angle;
    
    osg::Vec2Array* tcarr = (osg::Vec2Array*) _wheelgeometry->getTexCoordArray( TEXUNIT );
    
    const float degreespertick = DEGREESPERTICK;
    const float radspertick = (degreespertick/180*M_PI);

    const float increment = diff/radspertick;
    for ( int idx=0; idx<RESOLUTION; idx++ )
    {
	(*tcarr)[idx*2][1] += increment;
	(*tcarr)[idx*2+1][1] += increment;
    }
    
    tcarr->dirty();
    _wheelgeometry->dirtyDisplayList();
}


void ThumbWheel::accept( osg::NodeVisitor& nv )
{
    return _geode->accept( nv );
}


osg::BoundingSphere ThumbWheel::computeBound() const
{
    return _geode->computeBound();
}

void ThumbWheel::addRotateCallback( osg::NodeCallback* nc )
{
    if ( !_cb )
	_cb = nc;
    else
	_cb->addNestedCallback( nc );
}


void ThumbWheel::removeRotateCallback( osg::NodeCallback* nc )
{
    if ( nc==_cb )
	_cb = _cb->getNestedCallback();
    else
	_cb->removeNestedCallback( nc );
}


char ThumbWheel::getMousePosStatus( const osg::Vec2& mousepos ) const
{
    if ( mousepos[0]<_min[0] || mousepos[0]>_max[0] )
	return 0;
    
    if ( mousepos[1]<_min[1] || mousepos[1]>_max[1] )
	return 0;
    
    return 2;
}


bool ThumbWheel::handleEvent( const osgGA::GUIEventAdapter& ea )
{
    const osg::Vec2 mousepos( ea.getX(), ea.getY() );
    const char mouseposstatus = getMousePosStatus( mousepos );
    
    if ( mouseposstatus )
	showWheel( true );
    else if ( !_istracking )
	showWheel( false );
	
    if ( !_istracking )
    {
	if ( ea.getEventType()==osgGA::GUIEventAdapter::PUSH &&
	     ea.getButton()==1 )
	{
	    if ( mouseposstatus==2 )
	    {
		_istracking = true;
		_startpos = mousepos[_dim];
		_startangle = _currentangle;
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
	ea.getEventType()==osgGA::GUIEventAdapter::MOVE ||
	ea.getEventType()==osgGA::GUIEventAdapter::DOUBLECLICK )
    {
	//Just quit
	_istracking = false;
	return false;
    }
    
    const float movement = mousepos[_dim] - _startpos;
    const float deltaAngleSinceStart = movement * 2 / (_max[_dim]-_min[_dim]);
    const float newAngle = _startangle + deltaAngleSinceStart;
    const float deltaangle = _currentangle-newAngle;
    setAngle( newAngle );
    
    if ( deltaangle && _cb )
    {
	ThumbWheelEventNodeVisitor nv( deltaangle );
	(*_cb)( this, &nv );
    }
        
    return true;
}


ThumbWheelEventNodeVisitor::ThumbWheelEventNodeVisitor( float deltaangle )
    : _deltaangle( deltaangle )
{}


void ThumbWheel::showWheel( bool yn )
{
    const bool wheelshown =
	_geode->getDrawableIndex( _wheelgeometry )!=_geode->getNumDrawables();
    
    if ( wheelshown==yn )
	return;
    
    if ( yn )
	_geode->addDrawable( _wheelgeometry );
    else
	_geode->removeDrawable( _wheelgeometry );

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

