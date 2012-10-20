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
#include <osgManipulator/TabBoxDragger>


int main( int argc, char** argv )
{
    osgGeo::Viewer viewer( argc, argv );

    osg::Group* sceneroot = new osg::Group;
    osg::Camera* hudcamera = new osg::Camera;
    
    sceneroot->addChild( hudcamera );
    hudcamera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
    hudcamera->setViewMatrix(osg::Matrix::identity());
    hudcamera->setClearMask(GL_DEPTH_BUFFER_BIT);
    hudcamera->setRenderOrder(osg::Camera::POST_RENDER);
    hudcamera->setAllowEventFocus(false);
    hudcamera->setName("HUD Camera" );
    hudcamera->setProjectionMatrix(osg::Matrix::ortho2D(0,viewer.width(),0,viewer.height()));
    
    osg::Matrix mat;
    mat.makeTranslate( 200, 200, -200 );
    mat.preMult( osg::Matrix::scale( 200, 200, 200 ) );
    
    osgManipulator::TabBoxDragger* tbd = new osgManipulator::TabBoxDragger;
    hudcamera->addChild( tbd );
    tbd->setupDefaultGeometry();
    tbd->setMatrix( mat );
    tbd->setHandleEvents( true );
    
    osg::Geode* geode = new osg::Geode;
    geode->addDrawable( new osg::ShapeDrawable(new osg::Box) );
    sceneroot->addChild( geode );


    return viewer.run();
}
