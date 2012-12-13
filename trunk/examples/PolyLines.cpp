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
#include <osg/MatrixTransform>
#include <osgGA/TrackBallManipulator>
#include <osg/Material>


osg::ref_ptr<osg::Node> drawDNA(float xoffset, float zscale, const osg::Vec4& col )
{
    srand( clock() );
    osg::ref_ptr<osg::Vec3Array> coords = new osg::Vec3Array;
  

    osg::DrawElementsUInt* primset1 =
	new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLE_STRIP, 0);
    osg::DrawElementsUInt* primset2 =
	new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLE_STRIP, 0);

    int ci = 0;
    for ( int idx=0; idx<200; idx++ )
    { 
	osg::Vec3 coord( xoffset + 200*sin(idx/10.0), 200*cos(idx/10.0), idx*20*zscale ); 
	coords->push_back( coord );
	primset1->push_back( ci++ );
    }
    
    for ( int idx=0; idx<200; idx++ )
    { 
	osg::Vec3 coord( xoffset -200*sin(idx/10.0), -200*cos(idx/10.0),  idx*20*zscale  ); 
	coords->push_back( coord );
	primset2->push_back( ci++ );
    }

    osg::ref_ptr<osgGeo::PolyLineNode> polyline = new osgGeo::PolyLineNode();
    polyline->setRadius( 20 );
    osg::Material* mat = new osg::Material;
    mat->setDiffuse( osg::Material::FRONT_AND_BACK, col );
    polyline->getOrCreateStateSet()->setAttribute( mat );
    polyline->setVertexArray( coords );
    polyline->addPrimitiveSet( primset1 );
    polyline->addPrimitiveSet( primset2 );
    return polyline.get();
}

int main()
{
    osg::Group* group = new osg::Group;
    group->getOrCreateStateSet()->setMode(GL_NORMALIZE, osg::StateAttribute::ON); 
    group->addChild( drawDNA( 200, 2, osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f))  );
 
    osg::MatrixTransform* trans = new osg::MatrixTransform;
    group->addChild( trans );
    trans->setMatrix( osg::Matrix::scale( osg::Vec3d(1,1,2) ) );
    trans->addChild( drawDNA( 0, 1, osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f) ) );

    osgViewer::Viewer viewer;
    viewer.setSceneData ( group );
    viewer.setUpViewInWindow( 150, 20, 640, 480 );
    viewer.setCameraManipulator( new osgGA::TrackballManipulator );
    viewer.run();
}

