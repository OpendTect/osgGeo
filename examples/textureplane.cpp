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
*/


#include <osgGeo/LayeredTexture>
#include <osgGeo/TexturePlane>
#include <osgViewer/Viewer>
#include <osgDB/ReadFile>
#include <osgUtil/UpdateVisitor>


int main( int argc, char** argv )
{
    osg::ArgumentParser args( &argc, argv );

    osg::ref_ptr<osg::Image> img = osgDB::readImageFile( "Images/dog_left_eye.jpg" );

    char thinDim = 1;
    unsigned int brickSize = 64;
    bool disperseBricks = false;

    std::string str;
    while ( args.read("--dim", str) )
    {
	thinDim = std::atoi( str.c_str() );
	if ( thinDim<0 || thinDim>2 )
	{
	    std::cout << "Thin dimension not in {0,1,2]" << std::endl;
	    return 1;
	}
    }

    while ( args.read("--bricksize", str) )
    {
	brickSize = std::atoi( str.c_str() );
	if ( brickSize < 2 )
	    brickSize = 2;
    }

    while ( args.read("--image", str) )
	img = osgDB::readImageFile( str );
	
    if ( !img || !img->s() || !img->t() )
    {
	std::cout << "Invalid texture image" << std::endl;
	return 1;
    }

    while ( args.read("--disperse") )
	disperseBricks = true;

    osg::ref_ptr<osgGeo::LayeredTexture> laytex = new osgGeo::LayeredTexture();
    laytex->addDataLayer();
    laytex->setDataLayerImage( 0, img ); 

    osg::ref_ptr<osgGeo::TexturePlaneNode> root = new osgGeo::TexturePlaneNode();
    root->setLayeredTexture( laytex );
    
    osg::Vec3 width( 1.0, float(img->t()) / float(img->s()), 0.0f );
    if ( width.y() < 1.0f )
	width = osg::Vec3( float(img->s()) / float(img->t()), 1.0f, 0.0f );

    if ( thinDim==0 )
	width = osg::Vec3( 0.0f, width.x(), width.y() );
    else if ( thinDim==1 )
	width = osg::Vec3( width.x(), 0.0f, width.y() );

    root->setWidth( width );
    root->disperseBricks( disperseBricks );

    brickSize = osgGeo::LayeredTexture::getTextureSize( brickSize );
    root->setTextureBrickSize( brickSize );

    osgUtil::UpdateVisitor updateVisitor;
    root->traverse( updateVisitor ); 

    osgViewer::Viewer viewer;
    viewer.setSceneData( root.get() );
    return viewer.run();
}
