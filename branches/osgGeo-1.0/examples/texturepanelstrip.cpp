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

$Id: textureplane.cpp 222 2013-04-03 13:56:02Z jaap.glas@dgbes.com $

*/


#include <osgGeo/LayeredTexture>
#include <osgGeo/TexturePanelStrip>
#include <osgViewer/Viewer>
#include <osgDB/ReadFile>
#include <osg/ShapeDrawable>
#include <osgViewer/ViewerEventHandlers>


/* This example aims at testing the TexturePanelStripNode class only. 
   The underlying LayeredTexture class is merely used for that purpose.
   Look at examples/textureplane.cpp to experience its full potential. */


void setGeometry( osgGeo::TexturePanelStripNode& node, int incr, int start=-1 )
{
    static int geometryNr = 0;

    if ( start>=0 )
	geometryNr = start;

    geometryNr += incr;
    if ( geometryNr<0 )
	geometryNr = 0;

    float s0 = 0.0f;
    float s1 = 1.0f;
    if ( node.getTexture() && node.getTexture()->isEnvelopeDefined() )
    {
	osg::Vec2f vec = node.getTexture()->envelopeCenter();
	vec -= node.getTexture()->textureEnvelopeSize() * 0.5f;
	s0 = vec[node.areTextureAxesSwapped() ? 1 : 0];
	vec += node.getTexture()->textureEnvelopeSize();
	s1 = vec[node.areTextureAxesSwapped() ? 1 : 0];
    }

    node.setZRange2TextureMapping( false );
    const float t0 = node.getTopTextureMapping();
    const float t1 = node.getBottomTextureMapping();
    const float ratio = (t1-t0)/(s1-s0);

    osg::ref_ptr<osg::Vec2Array> path = new osg::Vec2Array;
    osg::ref_ptr<osg::FloatArray> map = new osg::FloatArray;

    if ( geometryNr==0 )	// Postcard
    {
	const float border = 0.05f * (s1-s0+t1-t0);
	path->push_back( osg::Vec2(-1.0f, 0.0f) );
	map->push_back( s0 - border );
	path->push_back( osg::Vec2(1.0f, 0.0f) );
	map->push_back( s1 + border );
	node.setZRange( ratio, -ratio );
	node.setZRange2TextureMapping( true, t1+border, t0-border );
    }
    else if ( geometryNr==1 )	//  Degenetated cases 
    {
	path->push_back( osg::Vec2(-1.0f, 0.0f) );
	map->push_back( s0 );
	path->push_back( osg::Vec2(-1.0f, 0.0f) );
	map->push_back( s0 );
	path->push_back( osg::Vec2(-0.8f, 0.2f) );
	map->push_back( s0 + (s1-s0)*0.1 );
	path->push_back( osg::Vec2(-0.6f, 0.4f) );
	map->push_back( s0 + (s1-s0)*0.1 );
	path->push_back( osg::Vec2(-0.2f, 0.8f) );
	map->push_back( s0 + (s1-s0)*0.3 );
	path->push_back( osg::Vec2(-0.2f, 0.8f) );
	map->push_back( s0 + (s1-s0)*0.4 );
	path->push_back( osg::Vec2(0.0f, 1.0f) );
	map->push_back( s0 + (s1-s0)*0.5 );
	path->push_back( osg::Vec2(0.0f, 1.0f) );
	map->push_back( s0 + (s1-s0)*0.5 );
	path->push_back( osg::Vec2(0.0f, 1.0f) );
	map->push_back( s0 + (s1-s0)*0.5 );
	path->push_back( osg::Vec2(0.6f, 0.4f) );
	map->push_back( s0 + (s1-s0)*0.8 );
	path->push_back( osg::Vec2(0.8f, 0.2f) );
	map->push_back( s0 + (s1-s0)*0.9 );
	path->push_back( osg::Vec2(0.6f, 0.4f) );
	map->push_back( s0 + (s1-s0)*1.0 );

	node.setZRange( -ratio*sqrt(2.0), ratio*sqrt(2.0) );
    }
    else if ( geometryNr==2 )	// Cylinder
    {
	const float pi = 3.1415927;
	for ( int idx=0; idx<=36; idx++ )
	{
	    path->push_back( osg::Vec2(cos(idx*pi/18.0f),sin(idx*pi/18.0f)) );
	    map->push_back( s0 + (s1-s0)*idx/36.0f );
	}
	node.setZRange( -ratio*pi, ratio*pi );
    }
    else			// Random (!) line
    {
	std::srand( geometryNr );
	const int nrKnots = 3 + std::rand()%3;
	std::vector<float> arcLength;
	arcLength.push_back( 0.0f );
	for ( int idx=0; idx<nrKnots; idx++ )
	{
	    path->push_back( osg::Vec2(-1.0f+(std::rand()%200)/100.0f,
				       -1.0f+(std::rand()%200)/100.0f) );
	    if ( idx )
	    {
		const osg::Vec2 dif = (*path)[idx] - (*path)[idx-1];
		arcLength.push_back( arcLength.back() + dif.length() );
	    }
	}

	for ( int idx=0; idx<nrKnots; idx++ )
	    map->push_back( s0 + (s1-s0)*arcLength[idx]/arcLength[nrKnots-1] );

	node.setZRange( -ratio*arcLength[nrKnots-1]*0.5f, ratio*arcLength[nrKnots-1]*0.5f );
    }

    node.setPath( *path );
    node.setPath2TextureMapping( *map );
}


class TexEventHandler : public osgGA::GUIEventHandler
{
    bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa, osg::Object*, osg::NodeVisitor* )
    {
	return handle( ea, aa );
    }

    bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
    {
	osgViewer::Viewer* viewer = dynamic_cast<osgViewer::Viewer*>(&aa);

	osgGeo::TexturePanelStripNode* root = dynamic_cast<osgGeo::TexturePanelStripNode*>( viewer->getSceneData() );
	if ( !root )
	{
	    osg::Group* group = dynamic_cast<osg::Group*>( viewer->getSceneData() );
	    if ( group )
		 root = dynamic_cast<osgGeo::TexturePanelStripNode*>( group->getChild(0) );
	    if ( !root )
		return false;
	}

	if ( ea.getEventType() != osgGA::GUIEventAdapter::KEYDOWN )
	    return false;

	if ( !root->getTexture() || !root->getTexture()->getProcess(0) )
	    return false;

	const int layerId = root->getTexture()->getProcess(0)->getDataLayerID();

	if ( ea.getKey()==osgGA::GUIEventAdapter::KEY_Right )
	{
	    root->toggleTilingMode();
	    return true;
	}
	if ( ea.getKey()==osgGA::GUIEventAdapter::KEY_Left )
	{
	    root->toggleTilingMode();
	    root->toggleTilingMode();
	    return true;
	}

	if ( ea.getKey()==osgGA::GUIEventAdapter::KEY_Up )
	{
	    setGeometry( *root, 1 );
	    return true;
	}
	if ( ea.getKey()==osgGA::GUIEventAdapter::KEY_Down )
	{
	    setGeometry( *root, -1 );
	    return true;
	}

	if ( ea.getKey()==osgGA::GUIEventAdapter::KEY_BackSpace )
	{
	    root->getTexture()->allowShaders( !root->getTexture()->areShadersAllowed() );
	    return true;
	}

	if ( ea.getKey()==osgGA::GUIEventAdapter::KEY_Return )
	{
	    osgGeo::FilterType filter = root->getTexture()->getDataLayerFilterType(layerId);
	    if ( filter==osgGeo::Nearest )
		filter = osgGeo::Linear;
	    else
		filter = osgGeo::Nearest;

	    root->getTexture()->setDataLayerFilterType( layerId, filter );
	    root->getTexture()->setDataLayerFilterType( root->getTexture()->compositeLayerId(), filter );
	    return true;
	}

	if ( ea.getKey()==osgGA::GUIEventAdapter::KEY_Tab )
	{
	    root->smoothNormals( !root->areNormalsSmoothed() );
	    return true;
	}

	return false;
    }
};


int main( int argc, char** argv )
{
    osg::ArgumentParser args( &argc, argv );

    osg::ApplicationUsage* usage = args.getApplicationUsage();
    usage->setCommandLineUsage( "texturepanelstrip [options]" );
    usage->setDescription( "3D view of panel strip with tiled texture" );
    usage->addCommandLineOption( "--bricksize <n>", "Brick size [1,->]" );
    usage->addCommandLineOption( "--geometry <n>", "Geometry number [0,->]" );
    usage->addCommandLineOption( "--sizepolicy <n>", "Texture size policy [0,2]" );
    usage->addCommandLineOption( "--help | --usage", "Command line info" );
    usage->addCommandLineOption( "--image <path>", "Texture layer" );
    usage->addCommandLineOption( "--border <R> <B> <G> <A>", "Image RGBA border color [-1=edge,255]" );
    usage->addCommandLineOption( "--scene", "Add scene elements" );
    usage->addCommandLineOption( "--swapaxes", "Swap texture axes" );
    usage->addKeyboardMouseBinding( "Left/Right arrow", "Toggle tiling" );
    usage->addKeyboardMouseBinding( "Up/Down key", "Toggle geometries" );
    usage->addKeyboardMouseBinding( "Return key", "Toggle filtering" );
    usage->addKeyboardMouseBinding( "Tab key", "Toggle normal smoothing" );
    usage->addKeyboardMouseBinding( "BackSpace key", "Toggle shaders" );

    if ( args.read("--help") || args.read("--usage") )
    {
	std::cout << std::endl << usage->getDescription() << std::endl << std::endl;
	usage->write( std::cout );
	usage->write( std::cout, usage->getKeyboardMouseBindings() );
	return 1;
    }

    osg::ref_ptr<osgGeo::LayeredTexture> laytex = new osgGeo::LayeredTexture();
    const int layerId = laytex->addDataLayer();
    osg::ref_ptr<osgGeo::IdentityLayerProcess> process = new osgGeo::IdentityLayerProcess( *laytex, layerId );
    laytex->addProcess( process );

    bool scene = false;
    while ( args.read("--scene") )
	scene = true;

    bool swapTextureAxes = false;
    while ( args.read("--swapaxes") )
	swapTextureAxes = true;

    std::string imagePath( "Images/dog_left_eye.jpg" );
    while ( args.read("--image", imagePath) );
    osg::ref_ptr<osg::Image> img = osgDB::readImageFile( imagePath );
    if ( !img || !img->s() || !img->t() )
	args.reportError( "Invalid texture image: " + imagePath );
    else
	laytex->setDataLayerImage( layerId, img.get() );

    int brickSize = 64;
    while ( args.read("--bricksize", brickSize) )
    {
	if ( brickSize<1 )
	{
	    args.reportError( "Brick size must be at least 1" );
	    brickSize = 1;
	}
    }

    int geometryIdx = 0;
    while ( args.read("--geometry", geometryIdx) )
    {
	if ( geometryIdx<0 )
	{
	    args.reportError( "Geometry number should not be negative" );
            geometryIdx = 0;
	}
    }

    int texSizePolicy = 0;
    while ( args.read("--sizepolicy", texSizePolicy) )
    {
	if ( texSizePolicy<0 || texSizePolicy>2 )
	{
	    args.reportError( "Texture size policy not in [0,2]" );
	    texSizePolicy = 0;
	}
	laytex->setTextureSizePolicy( (osgGeo::LayeredTexture::TextureSizePolicy)  (texSizePolicy/2) );
    }

    osg::Vec4f borderCol = laytex->getDataLayerBorderColor( layerId );
    int I, J, K, L;
    while ( args.read("--border", I, J, K, L) )
    {
	if ( I<-1 || I>255 || J<-1 || J>255 || K<-1 || K>255 || L<-1 || L>255 )
	    args.reportError( "Border color value not in [-1,255]" );

	borderCol = osg::Vec4f( I/255.0f, J/255.0f, K/255.0f, L/255.0f );
    }

    laytex->setDataLayerBorderColor( layerId, borderCol );
    laytex->setDataLayerBorderColor( laytex->compositeLayerId(), borderCol );

    args.reportRemainingOptionsAsUnrecognized(); 
    args.writeErrorMessages( std::cerr );

    osg::ref_ptr<osgGeo::TexturePanelStripNode> root = new osgGeo::TexturePanelStripNode();
    root->setTexture( laytex );

    root->setTextureBrickSize( brickSize, texSizePolicy%2 );

    if ( swapTextureAxes )
	root->swapTextureAxes( !root->areTextureAxesSwapped() );

    setGeometry( *root, 0, geometryIdx );

    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    osg::ref_ptr<osg::ShapeDrawable> sphere = new osg::ShapeDrawable;
    sphere->setShape( new osg::Sphere(osg::Vec3(0.0f,2.0f,0.0f),1.0f) );
    sphere->setColor( osg::Vec4(0.0f,1.0f,0.0f,1.0f) );
    geode->addDrawable( sphere.get() );

    osg::ref_ptr<osg::ShapeDrawable> cone = new osg::ShapeDrawable;
    cone->setShape( new osg::Cone(osg::Vec3(0.0f,-2.0f,-1.0f),1.0f,1.0f) );
    cone->setColor( osg::Vec4(1.0f,1.0f,0.0f,1.0f) );
    geode->addDrawable( cone.get() );

    osg::ref_ptr<osg::Group> group = new osg::Group;
    group->addChild( root.get() );
    group->addChild( geode.get() );

    osgViewer::Viewer viewer;

    if ( scene )
	viewer.setSceneData( group.get() );
    else
	viewer.setSceneData( root.get() );

    TexEventHandler* texEventHandler = new TexEventHandler;
    viewer.addEventHandler( texEventHandler );
    viewer.addEventHandler( new osgViewer::StatsHandler() );
    viewer.setThreadingModel(osgViewer::ViewerBase::SingleThreaded);

    return viewer.run();
}
