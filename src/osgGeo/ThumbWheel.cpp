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
#include <osgManipulator/Command>

//#include <osg/ShapeDrawable>
#include <osg/Geometry>
#include <osg/Vec4ub>
#include <osg/Texture2D>
//#include <osg/Material>

using namespace osgGeo;

ThumbWheel::ThumbWheel()
    : Dragger()
    , _checkForNodeInNodePath(true)
{
    _projector = new osgManipulator::LineProjector;
    setupDefaultGeometry();
}


ThumbWheel::ThumbWheel( const ThumbWheel&, const osg::CopyOp& )
    : Dragger()
    , _checkForNodeInNodePath(true)
{
    _projector = new osgManipulator::LineProjector;
}


ThumbWheel::~ThumbWheel()
{ }


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
