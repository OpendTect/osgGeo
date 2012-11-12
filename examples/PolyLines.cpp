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

$Id$

*/

#include <osg/Node> 
#include <osgGeo/PolyLine>
#include <osgViewer/Viewer> 

osg::ref_ptr<osg::Node> drawDNA()
{
    srand( clock() );
    osg::ref_ptr<osg::Vec3Array> coords = new osg::Vec3Array;
  

    osg::DrawElementsUInt* primset1 =
	new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLE_STRIP, 0);
    osg::DrawElementsUInt* primset2 =
	new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLE_STRIP, 0);

    int ci = 0;
    for ( int idx=0; idx<150; idx++ )
    { 
	osg::Vec3 coord( 200*sin(idx/10.0), 200*cos(idx/10.0), idx*20 ); 
	coords->push_back( coord );
	primset1->push_back( ci++ );
    }
    
    for ( int idx=0; idx<150; idx++ )
    { 
	osg::Vec3 coord( -200*sin(idx/10.0), -200*cos(idx/10.0),  idx*20  ); 
	coords->push_back( coord );
	primset2->push_back( ci++ );
    }

    osg::ref_ptr<osgGeo::PolyLineNode> polyline = new osgGeo::PolyLineNode();
    polyline->setRadius( 50 );
    polyline->setColor( osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f) );
    polyline->setVertexArray( coords );
    polyline->addPrimitiveSet( primset1 );
    polyline->addPrimitiveSet( primset2 );
    return polyline.get();
}

int main()
{
    osgViewer::Viewer viewer;
    viewer.setSceneData ( drawDNA() );
    viewer.setUpViewInWindow( 150, 20, 640, 480 );
    viewer.run();
}

