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

#define MAXANIMATEDROTATIONSPEED  5	// ticks per second

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
    , _hasFadeInOut( true )
    , _animationStart( -1 )
    , _animationTime( 1 )
    , _mouseProximity( None )
    , _rotationToDo( 0 )
    , _rotationProgressToDo( 0 )
    , _maxRotationProgress( 1 )
{
    _geode->ref();
    _geode->setCullingActive( false );

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

    _wheelGeometry->getStateSet()->setTextureAttributeAndModes( TEXUNIT, getSharedTexture() );
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


osg::Texture2D* ThumbWheel::getSharedTexture()
{
    static osg::ref_ptr<osg::Texture2D> texture = 0;
    if ( !texture )
    {
        texture = new osg::Texture2D;
        texture->setWrap( osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE );
        texture->setWrap( osg::Texture::WRAP_T, osg::Texture::REPEAT );

        osg::Image* image = new osg::Image();
        image->setImage( IMAGEHEIGHT, IMAGEWIDTH, 1, GL_RGBA, GL_RGBA,
                        GL_UNSIGNED_BYTE, imagedata, osg::Image::NO_DELETE );

        texture->setImage( image );
    }

    return texture;
}


void ThumbWheel::setBorderColor( const osg::Vec4& col )
{
    _outlineMaterial->setDiffuse( osg::Material::FRONT,
                                  osg::Vec4(col[0],col[1],col[2],col[3]) );
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


void ThumbWheel::updateWheelTexture( float diffAngle )
{
    osg::Vec2Array* tcarr = (osg::Vec2Array*) _wheelGeometry->getTexCoordArray( TEXUNIT );

    const float degreespertick = DEGREESPERTICK;
    const float radspertick = (degreespertick/180*M_PI);

    const float increment = diffAngle/radspertick;

    for ( int idx=0; idx<RESOLUTION; idx++ )
    {
	(*tcarr)[idx*2][1] += increment;
	(*tcarr)[idx*2+1][1] += increment;
    }

    tcarr->dirty();
    _wheelGeometry->dirtyDisplayList();
}


void ThumbWheel::setAngle( float angle, float rotationTime )
{
    _rotationToDo += angle-_currentAngle;
    _currentAngle = angle;

    const bool animate = rotationTime>0.0;
    _maxRotationProgress = animate ? rotationTime/_animationTime : 1.0;
    _rotationProgressToDo = 0.0;

    if ( fabs(_rotationToDo) > 1e-5 )
    {
	_rotationProgressToDo = _maxRotationProgress;

	if ( animate )
	    restartAnimation();
	else
	    updateRotation( _rotationProgressToDo );
    }
}


void ThumbWheel::enableFadeInOut( bool yn )
{
    _hasFadeInOut = yn;
    restartAnimation();
}


void ThumbWheel::accept( osg::NodeVisitor& nv )
{
    if ( nv.getVisitorType()==osg::NodeVisitor::UPDATE_VISITOR )
    {
        float progress = (osg::Timer::instance()->time_s() - _animationStart)/_animationTime;
        if ( isAnimating() && !updateAnimation( progress ) )
        {
            _animationStart = -1;
            setNumChildrenRequiringUpdateTraversal( getNumChildrenRequiringUpdateTraversal()-1);
        }

    }
    return _geode->accept( nv );
}


bool ThumbWheel::updateRotation( float progress )
{
    if ( _rotationProgressToDo<=0.0 )
	return false;

    float newRotationProgressToDo = _maxRotationProgress-progress;
    if ( newRotationProgressToDo<=0.0 )
    {
	newRotationProgressToDo = 0.0;
	restartAnimation();		// start mouse-proximity animation
    }

    const float fracToDo = newRotationProgressToDo/_rotationProgressToDo;
    float diffAngle = (1.0-fracToDo)*_rotationToDo;

    const float progressDone = _maxRotationProgress-_rotationProgressToDo;
    const float diffTime = (progress-progressDone)*_animationTime;
    const float maxTicks = MAXANIMATEDROTATIONSPEED*diffTime;
    const float maxRad = maxTicks*M_PI*DEGREESPERTICK/180;
    // Slip to avoid optical illusion of thumbwheel rolling backwards on screen
    if ( diffAngle > maxRad )
	diffAngle = maxRad;
    if ( diffAngle < -maxRad )
	diffAngle = -maxRad;

    _wheelMaterial->setAlpha( osg::Material::FRONT, 1.0 );
    updateWheelTexture( diffAngle );

    _rotationToDo *= fracToDo;
    _rotationProgressToDo = newRotationProgressToDo;
    return true;
}



bool ThumbWheel::updateAnimation( float progress )
{
    if ( updateRotation(progress) )
	return true;

    if ( !_hasFadeInOut )
    {
	_wheelMaterial->setAlpha( osg::Material::FRONT, 1 );
	return false;
    }

    float opacity = _wheelMaterial->getDiffuse( osg::Material::FRONT).a();
    bool do_cont = true;

    if ( _mouseProximity==Above )
    {
        opacity = 1;
	do_cont = false;
    }
    else if ( _mouseProximity==Nearby )
    {
	if ( opacity==1 )
	    return false;

	opacity = progress;
	if ( opacity>1 ) opacity = 1;
    }
    else
    {
        if ( !opacity )
	    return false;

	opacity = 1-progress;
	if ( opacity<0 ) opacity = 0;
    }

    _wheelMaterial->setAlpha( osg::Material::FRONT, opacity );
    return do_cont;
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


ThumbWheel::MouseProximity ThumbWheel::getMouseProximity( const osg::Vec2& mousepos ) const
{
    if ( mousepos[0]>_min[0] && mousepos[0]<_max[0] && mousepos[1]>_min[1] && mousepos[1]<_max[1] )
	return Above;

    const osg::Vec2 center = (_min+_max)/2;

    const float diffLen2 = (center-mousepos).length2();
    const float hotarea2 = ((_min-_max)*2).length2();

    if ( diffLen2<hotarea2 )
        return Nearby;

    return None;
}

#define mTurnOffTracking \
if ( _isTracking ) \
{ \
    _isTracking = false; \
    setNumChildrenRequiringUpdateTraversal( getNumChildrenRequiringUpdateTraversal()-1 ); \
}

bool ThumbWheel::handleEvent( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& )
{
    if ( ea.getEventType()==osgGA::GUIEventAdapter::FRAME )
        return false;

    const osg::Vec2 mousePos( ea.getX(), ea.getY() );
    updateWheel( getMouseProximity( mousePos ) );

    if ( !_isTracking )
    {
	if ( ea.getEventType()==osgGA::GUIEventAdapter::PUSH &&
	     ea.getButton()==1 )
	{
	    if ( _mouseProximity==Above )
	    {
		_isTracking = true;
                setNumChildrenRequiringUpdateTraversal( getNumChildrenRequiringUpdateTraversal()+1 );
		_startPos = mousePos[_dim];
		_startAngle = _currentAngle;
		return true;
	    }
	}

	return false;
    }

    if ( ea.getEventType()==osgGA::GUIEventAdapter::RELEASE && ea.getButton()==1)
    {
        mTurnOffTracking;
        return true;
    }

    if ( ea.getEventType()==osgGA::GUIEventAdapter::RESIZE ||
	ea.getEventType()==osgGA::GUIEventAdapter::MOVE ||
	ea.getEventType()==osgGA::GUIEventAdapter::DOUBLECLICK )
    {
	//Just quit
        mTurnOffTracking;
        return false;
    }

    const float movement = mousePos[_dim] - _startPos;
    const float deltaAngleSinceStart = movement * 2 / (_max[_dim]-_min[_dim]);
    const float newAngle = _startAngle + deltaAngleSinceStart;
    const float deltaangle = _currentAngle-newAngle;
    setAngle( newAngle );

    osg::NodeCallback* nodecb = dynamic_cast<osg::NodeCallback*>( _cb.get() );
    if ( deltaangle && nodecb )
    {
	ThumbWheelEventNodeVisitor nv( deltaangle );
	(*nodecb)( this, &nv );
    }

    return true;
}


void ThumbWheel::updateWheel( ThumbWheel::MouseProximity nmp )
{
    if ( nmp!=_mouseProximity )
    {
	_mouseProximity = nmp;
	restartAnimation();
    }
}


void ThumbWheel::restartAnimation()
{
    if ( !isAnimating() )
    {
	setNumChildrenRequiringUpdateTraversal( getNumChildrenRequiringUpdateTraversal()+1 );
    }

    _animationStart = osg::Timer::instance()->time_s();
}


ThumbWheelEventNodeVisitor::ThumbWheelEventNodeVisitor( float deltaangle )
    : _deltaangle( deltaangle )
{}


ThumbWheelEventNodeVisitor::~ThumbWheelEventNodeVisitor()
{}


ThumbWheelEventHandler::~ThumbWheelEventHandler()
{}


void ThumbWheelEventHandler::addThumbWheel( ThumbWheel* tw )
{
    _thumbwheels.push_back( osg::ref_ptr<ThumbWheel>(tw) );
}


void ThumbWheelEventHandler::removeThumbWheel( ThumbWheel* tw )
{
    for ( int idx=_thumbwheels.size()-1; idx>=0; idx-- )
    {
	if ( _thumbwheels[idx].get()==tw )
	    _thumbwheels.erase( _thumbwheels.begin()+idx );
    }
}


bool ThumbWheelEventHandler::handle (const osgGA::GUIEventAdapter &ea,
				     osgGA::GUIActionAdapter &us,
				     osg::Object*,
				     osg::NodeVisitor *)
{
    bool handled = false;
    for ( std::vector<osg::ref_ptr<ThumbWheel> >::iterator iter=_thumbwheels.begin();
	 iter!=_thumbwheels.end(); ++iter )
    {
	if ( iter->get()->handleEvent( ea, us ) )
	    handled = true;
    }

    return handled;
}

