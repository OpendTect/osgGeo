#ifndef osggeoviewer_h
#define osggeoviewer_h

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

#include <osgGeo/Common>

#ifdef USEQT
# include <QtGui/QApplication>
# include <osgQt/GraphicsWindowQt>
#endif

#include <osgViewer/CompositeViewer>

DISABLE_WARNINGS;
#include <osgViewer/ViewerEventHandlers>
ENABLE_WARNINGS;

#include <osgGA/TrackballManipulator>

namespace osgGeo
{

class Viewer
{
public:
    			Viewer( int argc, char** argv );
    void		setSceneData( osg::Node* node );

    int			run();

    int			width() const { return view_->getCamera()->getViewport()->width(); }
    int			height() const { return view_->getCamera()->getViewport()->height(); }
    
    void		addView(osgViewer::View* v) { compositeviewer_->addView(v); }
    osgViewer::View*	getView() const { return view_; }
    osg::Camera*	getCamera() { return view_->getCamera(); }
    void		setThreadingModel(osgViewer::ViewerBase::ThreadingModel);
    void		setupWindow(float x,float y, float w, float h);

protected:
#ifdef USEQT
    QApplication				app_;
    QWidget*					widget_;
#endif
    osg::ref_ptr<osgViewer::CompositeViewer>	compositeviewer_;
    osg::ref_ptr<osgViewer::View>		view_;
};


inline
#ifdef USEQT
Viewer::Viewer( int argc, char** argv )
    : app_( argc, argv )
    , widget_( 0 )
    , compositeviewer_( 0 )
    , view_( 0 )
#else
Viewer::Viewer( int, char** )
    : compositeviewer_( 0 )
    , view_( 0 )
#endif
{
    compositeviewer_ = new osgViewer::CompositeViewer;
    view_ = new osgViewer::View;
    view_->setCameraManipulator( new osgGA::TrackballManipulator );
    view_->addEventHandler( new osgViewer::StatsHandler );
    compositeviewer_->addView( view_ );
    
#ifdef USEQT
    osgQt::initQtWindowingSystem();
    osgQt::setViewer( compositeviewer_.get() );
    
    osgQt::GLWidget* glw = new osgQt::GLWidget;
    widget_ = glw;
    glw->setGeometry(0, 0, 800, 600 );
#endif
    
    osg::Viewport* viewport = new osg::Viewport(0, 0, 800, 600 );
    view_->getCamera()->setViewport( viewport );

#ifdef USEQT
    osgQt::GraphicsWindowQt* graphicswin = new osgQt::GraphicsWindowQt( glw );
    view_->getCamera()->setGraphicsContext( graphicswin );
#endif
}


inline
int Viewer::run()
{
#ifdef USEQT
    widget_->show();
    return app_.exec();
#else
    return compositeviewer_->run();
#endif
}

inline
void Viewer::setSceneData( osg::Node* n )
{
    view_->setSceneData( n );
}


void Viewer::setThreadingModel( osgViewer::ViewerBase::ThreadingModel tm )
{
    compositeviewer_->setThreadingModel( tm );
}


void Viewer::setupWindow( float x, float y, float w, float h )
{
    view_->setUpViewInWindow( x, y, w, h );
}


} //namespace

#endif
