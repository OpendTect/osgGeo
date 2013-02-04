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

$Id$

*/


#include <osgGeo/LayeredTexture>
#include <osgGeo/TexturePlane>
#include <osgViewer/Viewer>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osg/ShapeDrawable>
#include <osgViewer/ViewerEventHandlers>


static std::string dumpPath;

class TexEventHandler : public osgGA::GUIEventHandler
{
    bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
    {
	osgViewer::Viewer* viewer = dynamic_cast<osgViewer::Viewer*>(&aa);

	osgGeo::TexturePlaneNode* root = dynamic_cast<osgGeo::TexturePlaneNode*>( viewer->getSceneData() );
	if ( !root )
	{
	    osg::Group* group = dynamic_cast<osg::Group*>( viewer->getSceneData() );
	    if ( group )
		 root = dynamic_cast<osgGeo::TexturePlaneNode*>( group->getChild(0) );
	    if ( !root )
		return false;
	}

	if ( ea.getEventType() != osgGA::GUIEventAdapter::KEYDOWN )
	    return false;

	if ( ea.getKey()==osgGA::GUIEventAdapter::KEY_Up ||
	     ea.getKey()==osgGA::GUIEventAdapter::KEY_Down )
	{
	    osgGeo::LayeredTexture* tex = root->getLayeredTexture();
	    if ( !tex )
		return true;

	    if ( ea.getKey()==osgGA::GUIEventAdapter::KEY_Up )
	    {
		for ( int idx=0; idx<tex->nrProcesses(); idx++ )
		    tex->moveProcessLater( tex->getProcess(idx) );
	    }
	    else
	    {
		for ( int idx=tex->nrProcesses(); idx>0; idx-- )
		    tex->moveProcessEarlier( tex->getProcess(idx) );
	    }

	    return true;
	}

	if ( ea.getKey()==osgGA::GUIEventAdapter::KEY_Right ||
	     ea.getKey()==osgGA::GUIEventAdapter::KEY_Left )
	{
	    int disperseFactor = root->getDisperseFactor();

	    if ( ea.getKey()==osgGA::GUIEventAdapter::KEY_Right )
		disperseFactor++;
	    else 
		disperseFactor--;

	    root->setDisperseFactor( disperseFactor );
	    return true;
	}

	if ( ea.getKey()==osgGA::GUIEventAdapter::KEY_BackSpace )
	{
	    root->toggleShaders();
	    return true;
	}

	if ( ea.getKey()==osgGA::GUIEventAdapter::KEY_Return )
	{
	    osgGeo::LayeredTexture* tex = root->getLayeredTexture();
	    osg::ref_ptr<const osg::Image> image = tex->getCompositeTextureImage();
	    if ( !tex || !image.get() )
		return true;

	    if ( !dumpPath.empty() )
		osgDB::writeImageFile( *image, dumpPath );
	    else
		std::cerr << "Use --dump to specify texture dump path" << std::endl;
	    return true;
	}

	return false;
    }
};


static unsigned char heatArray[1024];
static osgGeo::ColorSequence* heatColSeq = 0;

static osgGeo::ColorSequence* heatColorSequence()
{
    if ( !heatColSeq )
    {
	for ( int idx=0; idx<256; idx++ )
	{
	    heatArray[4*idx+0] = idx<128 ? 0 : 2*idx-255; 
	    heatArray[4*idx+1] = idx<128 ? 2*idx : 511-2*idx; 
	    heatArray[4*idx+2] = idx<128 ? 255-2*idx : 0;
	    heatArray[4*idx+3] = 255;
	}

	heatColSeq = new osgGeo::ColorSequence( heatArray );
    }
    return heatColSeq;
}


static unsigned char transparencyArray[1024];
static osgGeo::ColorSequence* transparencyColSeq = 0;

static osgGeo::ColorSequence* transparencyColorSequence()
{
    if ( !transparencyColSeq )
    {
	for ( int idx=0; idx<256; idx++ )
	{
	    transparencyArray[4*idx+0] = 255; 
	    transparencyArray[4*idx+1] = 255; 
	    transparencyArray[4*idx+2] = 255;
	    transparencyArray[4*idx+3] = idx<85 ? 0 : idx<170 ? 128 : 255;
	}

	transparencyColSeq = new osgGeo::ColorSequence( transparencyArray );
    }
    return transparencyColSeq;
}


void addColTabProcess( osgGeo::LayeredTexture& laytex, int id, float opac, const osg::Vec4& udfCol, int seqNr, int channel )
{
    osgGeo::ColorSequence* colSeq = heatColorSequence();
    if ( seqNr%2 )
	colSeq = transparencyColorSequence();

    osg::ref_ptr<osgGeo::ColTabLayerProcess> process = new osgGeo::ColTabLayerProcess(laytex);
    process->setDataLayerID( id, channel );
    process->setColorSequence( colSeq );
    process->setOpacity( opac );
    process->setNewUndefColor( udfCol );
    laytex.addProcess( process );
}


void addRGBAProcess( osgGeo::LayeredTexture& laytex, int id, float opac, const osg::Vec4& udfCol, int r, int g, int b, int a )
{
    osg::ref_ptr<osgGeo::RGBALayerProcess> process = new osgGeo::RGBALayerProcess(laytex);

    if ( r>=0 ) process->setDataLayerID( 0, id, r );
    if ( g>=0 ) process->setDataLayerID( 1, id, g );
    if ( b>=0 ) process->setDataLayerID( 2, id, b );
    if ( a>=0 ) process->setDataLayerID( 3, id, a );

    process->setOpacity( opac );
    process->setNewUndefColor( udfCol );
    laytex.addProcess( process );
}


void addUndefLayer( osgGeo::LayeredTexture& laytex, int id, int R, int G, int B, int A )
{
    osg::ref_ptr<osg::Image> image = const_cast<osg::Image*>(laytex.getDataLayerImage(id));
    if ( !image )
	return;

    const GLenum format = image->getPixelFormat();
    const GLenum dataType = image->getDataType();
    const bool isByte = dataType==GL_UNSIGNED_BYTE || dataType==GL_BYTE;
    int udfVal[4];

    for ( int ic=0; ic<4; ic++ )
    {
	const int tc = osgGeo::LayeredTexture::image2TextureChannel(ic,format);

	if ( !ic && (tc<0 || !isByte) && (R>-1 || G>-1 || B>-1 || A>-1) )
        {
	    std::cerr << "Writing undefs not supported for given image format" << std::endl;
	    R = G = B = A = -1;
	}
	udfVal[ic] = tc==0 ? R : tc==1 ? G : tc==2 ? B : tc==3 ? A : -1;
    }

    int auxId = laytex.getDataLayerUndefLayerID(id);
    if ( auxId<0 )
    {
	auxId = laytex.addDataLayer();
	laytex.setDataLayerUndefLayerID( id, auxId );
	osg::ref_ptr<osg::Image> newImage = new osg::Image;
	newImage->allocateImage( image->s(), image->t(), image->r(), GL_LUMINANCE, GL_UNSIGNED_BYTE );
	unsigned char* ptr = newImage->data();
	for ( int cnt=newImage->getImageSizeInBytes(); cnt>0; cnt-- )
	    *ptr++ = 0;

	laytex.setDataLayerImage( auxId, newImage.get() );
	laytex.setDataLayerBorderColor( auxId, osg::Vec4f(1.0f, 1.0f, 1.0f, 1.0f) );
    }

    osg::ref_ptr<osg::Image> udfImage = const_cast<osg::Image*>(laytex.getDataLayerImage(auxId));
    if ( !udfImage )
	return;

    laytex.setDataLayerOrigin( auxId, laytex.getDataLayerOrigin(id) );
    laytex.setDataLayerScale( auxId, laytex.getDataLayerScale(id) );
    laytex.setDataLayerFilterType( auxId, laytex.getDataLayerFilterType(id) );

    const osg::Vec4f rgba( R/255.0f, G/255.0f, B/255.0f, A/255.0f );
    laytex.setDataLayerImageUndefColor( id, rgba );

    for ( int t=image->t()/4-1; t>=0; t-- )
    {
	for( int s=image->s()/4-1; s>=0; s-- )
	{
	    *udfImage->data(s,t) = 255;

	    for ( int idx=0; idx<4; idx++ )
	    {
		if ( udfVal[idx]>=0 )
		    image->data(s,t)[idx] = (unsigned char) udfVal[idx];
	    }
	}
    }
}


int main( int argc, char** argv )
{
    osg::ArgumentParser args( &argc, argv );

    osg::ApplicationUsage* usage = args.getApplicationUsage();
    usage->setCommandLineUsage( "textureplane [options]" );
    usage->setDescription( "3D view of tiled plane with layered set of textures or one default texture" );
    usage->addCommandLineOption( "--bricksize <n>", "Brick size [power-of-2]" );
    usage->addCommandLineOption( "--brickborders", "Allow bordered texture tiles" );
    usage->addCommandLineOption( "--dim <n>", "Thin dimension [0,2]" );
    usage->addCommandLineOption( "--help | --usage", "Command line info" );
    usage->addCommandLineOption( "--image <path> [origin-opt] [scale-opt] [opacity-opt] [colormap-opt] [rgbamap-opt] [udfimage-opt] [border-opt] [udfcolor-opt] [filter-opt]", "Add texture layer" );
    usage->addCommandLineOption( "--origin <x0> <y0>", "Layer origin" );
    usage->addCommandLineOption( "--scale <dx> <dy>", "Layer scale" );
    usage->addCommandLineOption( "--opacity <frac> ", "Layer opacity [0.0,1.0]" );
    usage->addCommandLineOption( "--colormap <n> <channel>", "Color map <n>  from channel [0,3]" );
    usage->addCommandLineOption( "--rgbamap <r> <g> <b> <a>", "RGBA map from channels [-1=void,3]" );
    usage->addCommandLineOption( "--filter <n>", "Filter type [0,1]" );
    usage->addCommandLineOption( "--udfimage <R> <B> <G> <A>", "Image RGBA undef area [-1=void,255]" );
    usage->addCommandLineOption( "--udfcolor <R> <B> <G> <A>", "New RGBA undef color [0,255]" );
    usage->addCommandLineOption( "--udfstack <R> <B> <G> <A>", "Stack RGBA undef area [0,255]" );
    usage->addCommandLineOption( "--border <R> <B> <G> <A>", "Image RGBA border color [-1=edge,255]" );
    usage->addCommandLineOption( "--scene", "Add scene elements" );
    usage->addCommandLineOption( "--dump <path>", "Texture dump path (.rgba)" );
    usage->addKeyboardMouseBinding( "Left/Right arrow", "Disperse tiles" );
    usage->addKeyboardMouseBinding( "Up/Down arrow", "Rotate layers" );
    usage->addKeyboardMouseBinding( "BackSpace key", "Toggle shaders" );
    usage->addKeyboardMouseBinding( "Return key", "Dump to specified file" );

    if ( args.read("--help") || args.read("--usage") )
    {
	std::cout << std::endl << usage->getDescription() << std::endl << std::endl;
	usage->write( std::cout );
	usage->write( std::cout, usage->getKeyboardMouseBindings() );
	return 1;
    }

    int thinDim = 1;
    int brickSize = 64;

    while ( args.read("--dim", thinDim) )
    {
	if ( thinDim<0 || thinDim>2 )
	{
	    args.reportError( "Thin dimension not in [0,2]" );
	    thinDim = 1;
	}
    }

    while ( args.read("--bricksize", brickSize) )
    {
	if ( brickSize < 2 )
	{
	    args.reportError( "Brick size must be at least 2" );
	    brickSize = 2;
	}
    }

    bool scene = false;
    while ( args.read("--scene") )
	scene = true;

    while ( args.read("--dump", dumpPath) )
    {}

    int I, J, K, L;
    osg::Vec4f udfStack( -1.0f, -1.0f, -1.0f, -1.0f );
    while ( args.read("--udfstack", I, J, K, L) )
    {
	if ( I<0 || I>255 || J<0 || J>255 || K<0 || K>255 || L<0 || L>255 )
	    args.reportError( "Stack RGBA undef value not in [0,255]" );

	udfStack = osg::Vec4f( I/255.0f, J/255.0f, K/255.0f, L/255.0f );
    }


    osg::ref_ptr<osgGeo::LayeredTexture> laytex = new osgGeo::LayeredTexture();

    while ( args.read("--brickborders") )
	laytex->allowBorderedTextures();


    const int firstId = laytex->addDataLayer();
    int lastId = firstId;

    int pos = 0;
    float opacity = 1.0;
    int channel = -1;
    int seqNr = 0;
    int r, g, b, a; 
    r = g = b = a = -2;
    int R, G, B, A; 
    R = G = B = A = -2;
    osg::Vec4f udfCol( 0.6f, 0.8f, 0.6f, 1.0f );

    while ( pos <= args.argc() )
    {
	std::string imagePath;
	if ( pos>=args.argc() && !laytex->getDataLayerImage(lastId) )
	    imagePath = "Images/dog_left_eye.jpg";
	else
	    args.read( pos, "--image", imagePath );

	if ( !imagePath.empty() )
	{
	    osg::ref_ptr<osg::Image> img = osgDB::readImageFile( imagePath );
	    if ( !img || !img->s() || !img->t() )
	    {
		args.reportError( "Invalid texture image: " + imagePath );
		continue;
	    }

	    if ( laytex->getDataLayerImage(lastId) )
		lastId = laytex->addDataLayer();

	    laytex->setDataLayerImage( lastId, img.get() );

	    if ( R>-2 || G>-2 || B>-2 || A>-2 )
		addUndefLayer( *laytex, lastId, R, G, B, A );

	    if ( channel>=0 )
		addColTabProcess( *laytex, lastId, opacity, udfCol, seqNr, channel );
	    else if ( r>-2 || g>-2 || b>-2 || a>-2 )
		addRGBAProcess( *laytex, lastId, opacity, udfCol, r, g, b, a );
	    else
	    {
		osg::ref_ptr<osgGeo::IdentityLayerProcess> process = new osgGeo::IdentityLayerProcess( *laytex, lastId );
		process->setOpacity( opacity );
		process->setNewUndefColor( udfCol );
		laytex->addProcess( process );
	    }

	    continue;
	}

	osg::Vec2f origin;
	if ( args.read(pos, "--origin", origin.x(), origin.y()) )
	{
	    laytex->setDataLayerOrigin( lastId, origin );
	    const int auxId = laytex->getDataLayerUndefLayerID( lastId );
	    laytex->setDataLayerOrigin( auxId, origin );
	    continue;
	}

	osg::Vec2f scale;
	if ( args.read(pos, "--scale", scale.x(), scale.y()) )
	{
	    if ( scale.x()<=0.0f || scale.y()<=0.0f )
		args.reportError( "Scales have to be positive" );

	    laytex->setDataLayerScale( lastId, scale );
	    const int auxId = laytex->getDataLayerUndefLayerID( lastId );
	    laytex->setDataLayerScale( auxId, scale );
	    continue;
	}

	int filterNr;
	if ( args.read(pos, "--filter", filterNr) )
	{
	    if ( filterNr<0 || filterNr>1 )
		args.reportError( "Filter number not in [0,1]" );

	    const osgGeo::FilterType filter = (osgGeo::FilterType) filterNr;

	    laytex->setDataLayerFilterType( lastId, filter );
	    const int auxId = laytex->getDataLayerUndefLayerID( lastId );
	    laytex->setDataLayerFilterType( auxId, filter );
	    continue;
	}

	if ( args.read(pos, "--border", I, J, K, L) )
	{
	    if ( I<-1 || I>255 || J<-1 || J>255 || K<-1 || K>255 || L<-1 || L>255 )
		args.reportError( "Border color value not in [-1,255]" );


	    osg::Vec4f borderCol( I/255.0f, J/255.0f, K/255.0f, L/255.0f );
	    laytex->setDataLayerBorderColor( lastId, borderCol );
	    continue;
	}

	if ( args.read(pos, "--opacity", opacity) )
	{
	    if ( opacity<0.0f || opacity>1.0f )
		args.reportError( "Opacity not in [0.0,1.0]" );

	    const int nrProc = laytex->nrProcesses();
	    if ( nrProc )
	    {
		laytex->getProcess(nrProc-1)->setOpacity( opacity );
		opacity = 1.0;
	    }
	    continue;
	}

	if ( args.read(pos, "--colormap", seqNr, channel) )
	{
	    if ( channel<0 || channel>3 )
		args.reportError( "Channel not in [0,3]" );

	    r = g = b = a = -2;
	    const int nrProc = laytex->nrProcesses();
	    if ( nrProc )
	    {
		const float opac = laytex->getProcess(nrProc-1)->getOpacity();
		laytex->removeProcess( laytex->getProcess(nrProc-1) );
		addColTabProcess( *laytex, lastId, opac, udfCol, seqNr, channel );
		channel = -1;
	    }
	    continue;
	}


	if ( args.read(pos, "--rgbamap", r, g, b, a) )
	{
	    if ( r<-1 || r>3 || g<-1 || g>3 || b<-1 || b>3 || a<-1 || a>3 )
		args.reportError( "Channel not in [-1=void,3]" );

	    channel = -1;
	    const int nrProc = laytex->nrProcesses();
	    if ( nrProc )
	    {
		const float opac = laytex->getProcess(nrProc-1)->getOpacity();
		laytex->removeProcess( laytex->getProcess(nrProc-1) );
		addRGBAProcess( *laytex, lastId, opac, udfCol, r, g, b, a );
		r = g = b = a = -2;
	    }
	    continue;
	}

	if ( args.read(pos, "--udfimage", R, G, B, A) )
	{
	    if ( R<-1 || R>255 || G<-1 || G>255 || B<-1 || B>255 || A<-1 || A>255 )
		args.reportError( "Image RGBA undef value not in [-1,255]" );

	    if ( laytex->nrProcesses() )
	    {
		addUndefLayer( *laytex, lastId, R, G, B, A );
		R = G = B = A = -2;
	    }
	    continue;
	}

	if ( args.read(pos, "--udfcolor", I, J, K, L) )
	{
	    if ( I<0 || I>255 || J<0 || J>255 || K<0 || K>255 || L<0 || L>255 )
		args.reportError( "New RGBA undef value not in [0,255]" );

	    udfCol = osg::Vec4f( I/255.0f, J/255.0f, K/255.0f, L/255.0f );

	    const int nrProc = laytex->nrProcesses();
	    if ( nrProc )
	    {
		laytex->getProcess(nrProc-1)->setNewUndefColor( udfCol );
		udfCol = osg::Vec4f( 0.6f, 0.8f, 0.6f, 1.0f );
	    }
	    continue;
	}

	pos++;
    }

    if ( udfStack != osg::Vec4f(-1.0f,-1.0f,-1.0f,-1.0f) )
    {
	const int id = laytex->addDataLayer();
	osg::ref_ptr<const osg::Image> img = laytex->getDataLayerImage( firstId );
	osg::ref_ptr<osg::Image> newImage = new osg::Image;
	newImage->allocateImage( img->s()/2 ,img->t()/2, img->r(), GL_LUMINANCE, GL_UNSIGNED_BYTE );
	unsigned char* ptr = newImage->data();
	for ( int cnt=newImage->getImageSizeInBytes(); cnt>0; cnt-- )
	    *ptr++ = 0;

	for ( int s=newImage->s()/2-1; s>=newImage->s()/4; s-- )
	{
	    for( int t=newImage->t()-1; t>=0; t-- )
		*newImage->data(s,t) = 255;
	}

	laytex->setDataLayerImage( id, newImage.get() );
	laytex->setDataLayerBorderColor( id, osg::Vec4f(0.0f, 0.0f, 0.0f, 0.0f) );
	laytex->setDataLayerOrigin( id, osg::Vec2f(0.0f,0.0f) );
	laytex->setDataLayerFilterType( id, laytex->getDataLayerFilterType(firstId) );
	laytex->setDataLayerImageUndefColor( id, osg::Vec4f(0.0f,0.0f,0.0f,0.0f) );

	laytex->setStackUndefLayerID( id );
	laytex->setStackUndefChannel( 0 );
	laytex->setStackUndefColor( udfStack );
    }

    args.reportRemainingOptionsAsUnrecognized(); 
    args.writeErrorMessages( std::cerr );

    osg::ref_ptr<osgGeo::TexturePlaneNode> root = new osgGeo::TexturePlaneNode();
    root->setLayeredTexture( laytex );

    // Fit to screen
    const osg::Vec2f envelopeSize = laytex->textureEnvelopeSize();
    osg::Vec3 center( laytex->envelopeCenter()/envelopeSize.x(), 0.0f );
    osg::Vec3 width( envelopeSize/envelopeSize.x(), 0.0f );
    if ( width.y() < 1.0f )
    {
	center = osg::Vec3( laytex->envelopeCenter()/envelopeSize.y(), 0.0f );
	width = osg::Vec3( envelopeSize/envelopeSize.y(), 0.0f );
    }

    if ( thinDim==0 )
    {
	width = osg::Vec3( 0.0f, width.x(), width.y() );
	center = osg::Vec3( 0.0f, center.x(), center.y() );
    }
    else if ( thinDim==1 )
    {
	width = osg::Vec3( width.x(), 0.0f, width.y() );
	center = osg::Vec3( center.x(), 0.0f, center.y() );
    }

    width.x() *= -1;		// Mirror x-dimension
    center.x() *= -1;

    //root->setCenter( center );  // Move texture origin to center of screen
    root->setWidth( width );
    root->setTextureBrickSize( brickSize );

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
