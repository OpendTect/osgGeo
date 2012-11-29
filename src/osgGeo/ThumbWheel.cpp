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

ThumbWheel::ThumbWheel()
    : _geode( new osg::Geode )
{
    setNumChildrenRequiringEventTraversal( 1 );
    _geode->ref();
    
    osg::Geometry* geometry = new osg::Geometry;
    _geode->addDrawable( geometry );
    osg::Vec3Array* coords = new osg::Vec3Array;
    geometry->setVertexArray( coords );
    coords->push_back( osg::Vec3( 200, 200, 0 ));
    coords->push_back( osg::Vec3( 400, 200, 0 ));
    coords->push_back( osg::Vec3( 200, 400, 0 ));
    coords->push_back( osg::Vec3( 400, 400, 0 ));
    
    geometry->addPrimitiveSet( new osg::DrawArrays( GL_TRIANGLE_STRIP,0, 4) );
}


ThumbWheel::ThumbWheel( const ThumbWheel& tw, const osg::CopyOp& op )
    : _geode( (osg::Geode*) tw._geode->clone( op ) )
{
    setNumChildrenRequiringEventTraversal( 1 );
}


ThumbWheel::~ThumbWheel()
{
    _geode->unref();
}


void ThumbWheel::accept( osg::NodeVisitor& nv )
{
    if ( nv.getVisitorType()==osg::NodeVisitor::EVENT_VISITOR)
    {
	osgGA::EventVisitor* evnv = dynamic_cast<osgGA::EventVisitor*>( &nv );
	if ( evnv && !evnv->getEventHandled() )
	{
	    if ( handleEvents( evnv->getEvents() ) )
		evnv->setEventHandled( true );
	}
    }
    return _geode->accept( nv );
}

osg::BoundingSphere ThumbWheel::computeBound() const
{
    return _geode->computeBound();
}

bool ThumbWheel::handleEvents( osgGA::EventQueue::Events& events )
{
    for ( osgGA::EventQueue::Events::iterator iter = events.begin();
	 iter!=events.end(); iter++ )
    {
	osg::ref_ptr<osgGA::GUIEventAdapter> ea = *iter;
	if ( ea->getEventType()==osgGA::GUIEventAdapter::PUSH )
	{
	    return false;
	}
    }
    
    return false;
}

/*
bool ThumbWheel::handle(const osgManipulator::PointerInfo& pointer, const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
    // Check if the dragger node is in the nodepath.
    if (_checkForNodeInNodePath)
    {
        if (!pointer.contains(this)) return false;
    }

    switch (ea.getEventType())
    {
        // Pick start.
        case (osgGA::GUIEventAdapter::PUSH):
	{
	    // Get the LocalToWorld matrix for this node and set it for the projector.
	    osg::NodePath nodePathToRoot;
	    computeNodePathToRoot(*this,nodePathToRoot);
	    osg::Matrix localToWorld = osg::computeLocalToWorld(nodePathToRoot);
	    _projector->setLocalToWorld(localToWorld);

	    if (_projector->project(pointer, _startProjectedPoint))
	    {
		// Generate the motion command.
		osg::ref_ptr<osgManipulator::TranslateInLineCommand> cmd = new osgManipulator::TranslateInLineCommand(_projector->getLineStart(),
										      _projector->getLineEnd());
		cmd->setStage(osgManipulator::MotionCommand::START);
		cmd->setLocalToWorldAndWorldToLocal(_projector->getLocalToWorld(),_projector->getWorldToLocal());

		// Dispatch command.
		dispatch(*cmd);

		aa.requestRedraw();
	    }
	    return true;
	}

        // Pick move.
        case (osgGA::GUIEventAdapter::DRAG):
	{
	    osg::Vec3d projectedPoint;
	    if (_projector->project(pointer, projectedPoint))
	    {
		// Generate the motion command.
		osg::ref_ptr<osgManipulator::TranslateInLineCommand> cmd = new osgManipulator::TranslateInLineCommand(_projector->getLineStart(),
										      _projector->getLineEnd());
		cmd->setStage(osgManipulator::MotionCommand::MOVE);
		cmd->setLocalToWorldAndWorldToLocal(_projector->getLocalToWorld(),_projector->getWorldToLocal());
		cmd->setTranslation(projectedPoint - _startProjectedPoint);

		// Dispatch command.
		dispatch(*cmd);

		aa.requestRedraw();
	    }
	    return true;
	}

        // Pick finish.
        case (osgGA::GUIEventAdapter::RELEASE):
	{
	    osg::Vec3d projectedPoint;
	    if (_projector->project(pointer, projectedPoint))
	    {
		osg::ref_ptr<osgManipulator::TranslateInLineCommand> cmd = new osgManipulator::TranslateInLineCommand(_projector->getLineStart(),
			_projector->getLineEnd());

		cmd->setStage(osgManipulator::MotionCommand::FINISH);
		cmd->setLocalToWorldAndWorldToLocal(_projector->getLocalToWorld(),_projector->getWorldToLocal());

		// Dispatch command.
		dispatch(*cmd);

		aa.requestRedraw();
	    }

	    return true;
	}
        default:
            return false;
    }
}

void ThumbWheel::setupDefaultGeometry()
{
    osg::ref_ptr<osg::Geode> geode = new osg::Geode;

    osg::Geometry* geom = new osg::Geometry;
    geode->addDrawable( geom );

    osg::Vec3Array* vertices = new osg::Vec3Array;
    geom->setVertexArray( vertices );
    vertices->push_back( osg::Vec3f(-100,-50,100) );
    vertices->push_back( osg::Vec3f(100,-50,100) );
    vertices->push_back( osg::Vec3f(-100,50,-100) );
    vertices->push_back( osg::Vec3f(100,50,-100) );

    geom->addPrimitiveSet( new osg::DrawArrays(GL_TRIANGLE_STRIP, 0, 4 ) );


    const osg::Vec4ub frontcol( 255, 255, 255, 255 );
    const osg::Vec4ub backcol( 0, 0, 0, 255 );
    osg::ref_ptr<osg::Image> image = new osg::Image;
    image->allocateImage( 4, 4, 1, GL_RGB, GL_UNSIGNED_BYTE );

#define mSetCol( col ) \
{ \
    *imageptr++ = col[0]; \
    *imageptr++ = col[1]; \
    *imageptr++ = col[2]; \
}

    unsigned char* imageptr = image->data();
    for ( int idx=0; idx<4; idx++ )
	mSetCol( frontcol );

    for ( int idx=0; idx<3; idx++ )
    {
	mSetCol( frontcol );
	mSetCol( backcol );
	mSetCol( backcol );
	mSetCol( frontcol );
    }

    geom->getOrCreateStateSet()->setTextureAttribute( 0, new osg::Texture2D( image ) );

    addChild(geode);
}
 */
