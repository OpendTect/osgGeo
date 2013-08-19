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

$Id: PolygonSel.cpp 169 2013-01-18 11:47:07Z ranojay.sen@dgbes.com $

*/

#include <osg/Node> 
#include <osgViewer/Viewer> 
#include <osgGA/TrackballManipulator>
#include <osg/ShapeDrawable>
#include <osg/MatrixTransform>

#include <osgGeo/PolygonSelection>
#include <osgGeo/ThumbWheel>

#include "osggeoviewer.h"


int main( int argc, char** argv )
{
    osgGeo::Viewer viewer( argc, argv );
    viewer.setupWindow( 1500, 100, 800, 600 );
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
    
    osg::ref_ptr<osgGeo::PolygonSelection> polygonsel
					= new  osgGeo::PolygonSelection;
    polygonsel->addEventHandlerCamera( viewer.getCamera() );
    polygonsel->setShapeType( osgGeo::PolygonSelection::Polygon );
    polygonsel->setColor( osg::Vec4(0,0.7,0,1) );
   
    osg::ref_ptr<osgViewer::View> hudview = new osgViewer::View;
    hudview->setCamera( hudcamera );
    hudview->setSceneData( polygonsel );
    viewer.addView( hudview );
    
    osg::Geode* geode = new osg::Geode;
    geode->addDrawable( new osg::ShapeDrawable(new osg::Box) );
    viewer.setSceneData( geode );

    return viewer.run();
}

