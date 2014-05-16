/*
osgGeo - A collection of geoscientific extensions to OpenSceneGraph.
Copyright 2011 dGB Beheer B.V. and others.

http://osggeo.googlecode.com

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

$Id: PolyLines.cpp 108 2012-10-08 08:32:40Z kristofer.tingdahl@dgbes.com $

*/

#include "osggeoviewer.h"

#include <osg/ShapeDrawable>
#include <osg/MatrixTransform>
#include <osgGeo/ThumbWheel>
#include <osgManipulator/TabBoxDragger>


int main( int argc, char** argv )
{
    osgGeo::Viewer viewer( argc, argv );

    osg::ref_ptr<osg::Camera> hudcamera = new osg::Camera;
    hudcamera->setGraphicsContext( viewer.getCamera()->getGraphicsContext() );
    hudcamera->setName("HUD Camera");
    hudcamera->setProjectionMatrix(osg::Matrix::ortho2D(0,viewer.width(),0,viewer.height()));
    hudcamera->setViewport( viewer.getCamera()->getViewport() );
    hudcamera->setClearMask(GL_DEPTH_BUFFER_BIT);
    hudcamera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
    hudcamera->setViewMatrix(osg::Matrix::identity());
    
    //draw subgraph after main camera view.
    hudcamera->setRenderOrder(osg::Camera::POST_RENDER, 1 );
    
    //we don't want the camera to grab event focus from the viewers main cam(s).
    hudcamera->setAllowEventFocus(false);
    
    osg::ref_ptr<osgViewer::View> hudview = new osgViewer::View;
    hudview->setCamera( hudcamera );
    viewer.addView( hudview );
    
    osg::Group* sceneroot = new osg::Group;
    
    osgGeo::ThumbWheel* wheel = new osgGeo::ThumbWheel;
    osgGeo::ThumbWheelEventHandler* handler= new osgGeo::ThumbWheelEventHandler;
    handler->addThumbWheel( wheel );
    viewer.getCamera()->addEventCallback( handler );

    osg::Geode* geode = new osg::Geode;
    osg::MatrixTransform* trans = new osg::MatrixTransform;
    trans->setMatrix( osg::Matrix::scale(osg::Vec3f(100,100,100)) );
    trans->getOrCreateStateSet()->setMode( GL_RESCALE_NORMAL, GL_TRUE);
    trans->addChild( geode );
    sceneroot->addChild( trans );
    geode->addDrawable( new osg::ShapeDrawable(new osg::Box) );
    hudview->setSceneData( wheel );
    
    viewer.setSceneData( sceneroot );

    return viewer.run();
}
