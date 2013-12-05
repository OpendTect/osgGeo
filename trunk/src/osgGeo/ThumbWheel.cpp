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
#include <osg/Material>
#include <osg/Texture2D>


using namespace osgGeo;

#define DEGREESPERTICK 20.0f
#define TEXUNIT 0
#define RESOLUTION 10
#define IMAGEHEIGHT 16
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
        0, 0, 0, 255,
	0, 0, 0, 255,
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
        0, 0, 0, 255,
	0, 0, 0, 255,
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
        0, 0, 0, 255,
	0, 0, 0, 255,
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
        0, 0, 0, 255,
	0, 0, 0, 255,
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
        0, 0, 0, 255,
	0, 0, 0, 255,
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
        0, 0, 0, 255,
	0, 0, 0, 255,
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
        0, 0, 0, 255,
	0, 0, 0, 255,
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
    , _isTracking( false )
    , _currentAngle( 0 )
    , _animationStart( -1 )
    , _animationTime( 1 )
{
    _geode->ref();
    
    _wheelGeometry = new osg::Geometry;
    _wheelGeometry->ref();
    _wheelGeometry->setVertexArray( new osg::Vec3Array );
    _wheelGeometry->setNormalArray( new osg::Vec3Array );
    _wheelGeometry->setTexCoordArray( TEXUNIT, new osg::Vec2Array );
    _wheelGeometry->setNormalBinding( osg::Geometry::BIND_PER_VERTEX );

    _wheelMaterial = new osg::Material;
    _wheelGeometry->getOrCreateStateSet()->setAttribute( _wheelMaterial, osg::StateAttribute::ON );
    _wheelGeometry->getStateSet()->setMode( GL_BLEND, osg::StateAttribute::ON );
    _wheelGeometry->getStateSet()->setMode( GL_LIGHTING, osg::StateAttribute::ON );
    _wheelGeometry->getStateSet()->setRenderingHint( osg::StateSet::TRANSPARENT_BIN );

    osg::Image* image = new osg::Image();
    image->setImage( IMAGEHEIGHT, IMAGEWIDTH, 1, GL_RGBA, GL_RGBA,
		    GL_UNSIGNED_BYTE, imagedata, osg::Image::NO_DELETE );
    osg::Texture2D* texture = new osg::Texture2D;
    texture->setWrap( osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE );
    texture->setWrap( osg::Texture::WRAP_T, osg::Texture::REPEAT );

    texture->setImage( image );
    _wheelGeometry->getStateSet()->setTextureAttributeAndModes( TEXUNIT, texture );
    _geode->addDrawable( _wheelGeometry );
    
    _outlineGeometry = new osg::Geometry;
    _geode->addDrawable( _outlineGeometry );
    _outlineMaterial = new osg::Material;
    _outlineGeometry->getOrCreateStateSet()->setAttribute( _outlineMaterial, osg::StateAttribute::ON );
    _outlineGeometry->getStateSet()->setMode( GL_BLEND, osg::StateAttribute::ON );
    _outlineGeometry->getStateSet()->setMode( GL_LIGHTING, osg::StateAttribute::OFF );
    _outlineGeometry->setVertexArray( _wheelGeometry->getVertexArray() );

    osg::ref_ptr<osg::Vec3Array> outlineNormals = new osg::Vec3Array;
    outlineNormals->push_back( osg::Vec3(0.0f,0.0f,1.0f) );
    _outlineGeometry->setNormalArray( outlineNormals.get() );
    _outlineGeometry->setNormalBinding( osg::Geometry::BIND_OVERALL );
    
    setShape( 0, osg::Vec2( 200, 200 ), osg::Vec2( 400,  250 ), 0 );
    setAngle( 0 );

    updateAnimation( 4 );
}


ThumbWheel::~ThumbWheel()
{
    _geode->unref();
    _wheelGeometry->unref();
}


void ThumbWheel::setBorderColor( const osg::Vec4& col )
{
    _outlineMaterial->setDiffuse( osg::Material::FRONT,
                                  osg::Vec4(col[0],col[1],col[2], 1));
}


void ThumbWheel::setWheelColor( const osg::Vec4& col )
{
    _wheelMaterial->setDiffuse( osg::Material::FRONT,
                                 osg::Vec4(col[0],col[1],col[2], 1));
}



void ThumbWheel::setShape( short dim, const osg::Vec2& min,const osg::Vec2& max,
			   float zval )
{
    _isTracking = false;
    _dim = dim;
    _min = min; _max = max;
    osg::Vec3Array* varr = (osg::Vec3Array*) _wheelGeometry->getVertexArray();
    osg::Vec3Array* narr = (osg::Vec3Array*) _wheelGeometry->getNormalArray();
    osg::Vec2Array* tcarr = (osg::Vec2Array*) _wheelGeometry->getTexCoordArray( TEXUNIT );
    
    const int resolution = RESOLUTION;
    const float anglestep = M_PI/(resolution-1);
    const float wheelradius = (_max[_dim]-_min[_dim])/2;
    const float wheelcenter = (_max[_dim]+_min[_dim])/2;
    
    const short dim2 = _dim ? 0 : 1;
    
    for ( int idx=varr->size(); idx<resolution*2; idx++ )
    {
	varr->push_back( osg::Vec3() );
	narr->push_back( osg::Vec3() );
	tcarr->push_back( osg::Vec2() );
    }

    const float tc0 = 0.5/IMAGEHEIGHT;
    const float tc1 = 1-(0.5/IMAGEHEIGHT);
    
    const float degreespertick = DEGREESPERTICK;
    const float radspertick = (degreespertick/180*M_PI);
    for ( int idx=0; idx<resolution; idx++ )
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
	(*tcarr)[idx*2] = osg::Vec2( tc1, tc );
	(*tcarr)[idx*2+1] = osg::Vec2( tc0, tc );
	(*narr)[idx*2] = (*narr)[idx*2+1] = normal;
    }
    
    if ( !_wheelGeometry->getNumPrimitiveSets() )
    {
	_wheelGeometry->addPrimitiveSet( new osg::DrawArrays( GL_TRIANGLE_STRIP,0, resolution*2) );
    }
    
    if ( !_outlineGeometry->getNumPrimitiveSets() )
    {
	osg::DrawElementsUByte* primitive = new osg::DrawElementsUByte( GL_LINE_STRIP );
	primitive->push_back( 0 );
	primitive->push_back( 1 );
	primitive->push_back( varr->size()-1 );
	primitive->push_back( varr->size()-2 );
	primitive->push_back( 0 );
	_outlineGeometry->addPrimitiveSet( primitive );
    }
    
    _wheelGeometry->dirtyDisplayList();
    _outlineGeometry->dirtyDisplayList();
    dirtyBound();
}


void ThumbWheel::setAngle( float angle )
{
    float diff = angle-_currentAngle;
    if ( diff==0 )
	return;
    
    _currentAngle = angle;
    
    osg::Vec2Array* tcarr = (osg::Vec2Array*) _wheelGeometry->getTexCoordArray( TEXUNIT );
    
    const float degreespertick = DEGREESPERTICK;
    const float radspertick = (degreespertick/180*M_PI);

    const float increment = diff/radspertick;
    for ( int idx=0; idx<RESOLUTION; idx++ )
    {
	(*tcarr)[idx*2][1] += increment;
	(*tcarr)[idx*2+1][1] += increment;
    }
    
    tcarr->dirty();
    _wheelGeometry->dirtyDisplayList();
}


void ThumbWheel::accept( osg::NodeVisitor& nv )
{
    if ( nv.getVisitorType()==osg::NodeVisitor::CULL_VISITOR )
    {
        float progress = (osg::Timer::instance()->time_s() - _animationStart)/_animationTime;
        if ( isAnimating() )
        {
            if ( progress>=4 )
            {
                progress = 4;
                _animationStart = -1;
            }

            updateAnimation( progress );
        }
    }

    return _geode->accept( nv );
}


void ThumbWheel::updateAnimation( float progress )
{
    float opacity;
    if ( progress<1 )
    	opacity = progress;
    else if ( progress<3 )
        opacity = 1;
    else if ( progress<4 )
    	opacity = 4-progress;
    else
        opacity = 0;

    if ( _wheelMaterial->getDiffuse( osg::Material::FRONT).a()!=opacity )
    {
    	_wheelMaterial->setAlpha( osg::Material::FRONT, opacity );
        _outlineMaterial->setAlpha( osg::Material::FRONT, 1-opacity );
    }
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
    if ( mousepos[0]>_min[0] && mousepos[0]<_max[0] && mousepos[1]>_min[1] && mousepos[1]<_max[1] )
	return 2;

    const osg::Vec2 center = (_min+_max)/2;

    const float diffLen2 = (center-mousepos).length2();
    const float hotarea2 = ((_min-_max)*2).length2();

    if ( diffLen2<hotarea2 )
        return 1;
    
    return 0;
}


bool ThumbWheel::handleEvent( const osgGA::GUIEventAdapter& ea )
{
    if ( ea.getEventType()==osgGA::GUIEventAdapter::FRAME )
        return false;

    const osg::Vec2 mousepos( ea.getX(), ea.getY() );
    const char mouseposstatus = getMousePosStatus( mousepos );
    
    if ( mouseposstatus )
	showWheel( mouseposstatus==1 );

    if ( !_isTracking )
    {
	if ( ea.getEventType()==osgGA::GUIEventAdapter::PUSH &&
	     ea.getButton()==1 )
	{
	    if ( mouseposstatus==2 )
	    {
		_isTracking = true;
		_startPos = mousepos[_dim];
		_startAngle = _currentAngle;
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
	_isTracking = false;
	return true;
    }
    
    if ( ea.getEventType()==osgGA::GUIEventAdapter::RESIZE ||
	ea.getEventType()==osgGA::GUIEventAdapter::MOVE ||
	ea.getEventType()==osgGA::GUIEventAdapter::DOUBLECLICK )
    {
	//Just quit
	_isTracking = false;
	return false;
    }
    
    const float movement = mousepos[_dim] - _startPos;
    const float deltaAngleSinceStart = movement * 2 / (_max[_dim]-_min[_dim]);
    const float newAngle = _startAngle + deltaAngleSinceStart;
    const float deltaangle = _currentAngle-newAngle;
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


ThumbWheelEventNodeVisitor::~ThumbWheelEventNodeVisitor()
{}


void ThumbWheel::showWheel( bool animate )
{
    const double now = osg::Timer::instance()->time_s();

    if ( animate )
    {
        if ( isAnimating() )
        {
            const float progress = (now-_animationStart)/_animationTime;
            if ( progress<1 )
                return;
            if ( progress<3 )
                _animationStart = now - _animationTime;
            else if ( progress<4 )
                _animationStart = now - (4-progress) * _animationTime;
            else
                _animationStart = now;
        }
        else
            _animationStart = now;
    }
    else
    {
	_animationStart = now - _animationTime;
    }
}


ThumbWheelEventHandler::~ThumbWheelEventHandler()
{}

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

