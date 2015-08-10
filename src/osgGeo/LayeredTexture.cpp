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
#include <osg/BlendFunc>
#include <osg/GLExtensions>
#include <osg/FragmentProgram>
#include <osg/Geometry>
#include <osg/State>
#include <osg/Texture2D>
#include <osg/Texture3D>
#include <osg/Version>
#include <osg/VertexProgram>
#include <osgUtil/CullVisitor>
#include <osgGeo/Vec2i>

#include <string.h>
#include <iostream>
#include <cstdio>


#if OSG_MIN_VERSION_REQUIRED(3,1,0)
     #define USE_IMAGE_STRIDE
#endif

#if defined _MSC_VER && __cplusplus < 201103L
# define snprintf( a, n, ... ) _snprintf_s( a, n, _TRUNCATE, __VA_ARGS__ )
#endif


typedef uintptr_t pixel_int;


namespace osgGeo
{


int LayeredTexture::powerOf2Ceil( unsigned int nr )
{
    if ( nr<=65536 )
    {
	if ( nr<=256 )
	{
	    if ( nr<=16 )
	    {
		if ( nr<=4 )
		    return nr<=2 ? nr : 4;

		return nr<=8 ? 8 : 16;
	    }

	    if ( nr<=64 )
		return nr<=32 ? 32 : 64;

	    return nr<=128 ? 128 : 256;
	}

	if ( nr<=4096 )
	{
	    if ( nr<=1024 )
		return nr<=512 ? 512 : 1024;

	    return nr<=2048 ? 2048 : 4096;
	}

	if ( nr<=16384 )
	    return nr<=8192 ? 8192 : 16384;

	return nr<=32768 ? 32768 : 65536;
    }

    if ( nr<=16777216 )
    {
	if ( nr<=1048576 )
	{
	    if ( nr<=262144 )
		return nr<=131072 ? 131072 : 262144;

	    return nr<=524288 ? 524288 : 1048576;
	}

	if ( nr<=4194304 )
	    return nr<=2097152 ? 2097152 : 4194304;

	return nr<=8388608 ? 8388608 : 16777216;
    }

    if ( nr<=268435456 )
    {
	if ( nr<=67108864 )
	    return nr<=33554432 ? 33554432 : 67108864; 

	return nr<=134217728 ? 134217728 : 268435456;
    }

    if ( nr<=1073741824 )
	return nr<=536870912 ? 536870912 : 1073741824; 

    return -1;	// larger than 2^30 not supported
}


int LayeredTexture::image2TextureChannel( int channel, GLenum format )
{
    if ( channel<0 || channel>3 )
	return -1;

    if ( format==GL_RGBA )
	return channel;
    if ( format==GL_RGB )
	return channel==3 ? -1 : channel;
    if ( format==GL_LUMINANCE_ALPHA )
	return channel>1 ? -1 : 3*channel;
    if ( format==GL_LUMINANCE || format==GL_INTENSITY ||
	 format==GL_RED || format==GL_DEPTH_COMPONENT )
	return channel==0 ? 0 : -1;
    if ( format==GL_ALPHA )
	return channel==0 ? 3 : -1;
    if ( format==GL_GREEN )
	return channel==0 ? 1 : -1;
    if ( format==GL_BLUE )
	return channel==0 ? 2 : -1;
    if ( format==GL_BGRA )
	return channel==3 ? 3 : 2-channel;
    if ( format==GL_BGR )
	return channel==3 ? -1 : 2-channel;

    return -1;
}


// Beware that osg::Image::getColor(.,.,.) is used occasionally, but
// not (yet) supports format: GL_RED, GL_GREEN, GL_BLUE, GL_INTENSITY

#define ONE_CHANNEL  4
#define ZERO_CHANNEL 5

static int texture2ImageChannel( int channel, GLenum format )
{
    if ( channel<0 || channel>3 )
	return -1;

    if ( format==GL_RGBA )
	return channel;
    if ( format==GL_RGB )
	return channel==3 ? ONE_CHANNEL : channel;
    if ( format==GL_LUMINANCE_ALPHA )
	return channel==3 ? 1 : 0;
    if ( format==GL_LUMINANCE || format==GL_DEPTH_COMPONENT )
	return channel==3 ? ONE_CHANNEL : 0;
    if ( format==GL_ALPHA )
	return channel==3 ? 0 : ZERO_CHANNEL;
    if ( format==GL_INTENSITY )
	return 0;
    if ( format==GL_RED )
	return channel==0 ? 0 : (channel==3 ? ONE_CHANNEL : ZERO_CHANNEL);
    if ( format==GL_GREEN )
	return channel==1 ? 0 : (channel==3 ? ONE_CHANNEL : ZERO_CHANNEL); 
    if ( format==GL_BLUE )
	return channel==2 ? 0 : (channel==3 ? ONE_CHANNEL : ZERO_CHANNEL);
    if ( format==GL_BGRA )
	return channel==3 ? 3 : 2-channel;
    if ( format==GL_BGR )
	return channel==3 ? ONE_CHANNEL : 2-channel;

    return -1;
}


static TransparencyType getImageTransparencyType( const osg::Image* image, int textureChannel=3 )
{                                                                               
    if ( !image )
	return FullyTransparent;

    GLenum format = image->getPixelFormat();
    const int imageChannel = texture2ImageChannel( textureChannel, format );

    if ( imageChannel==ZERO_CHANNEL )
	return FullyTransparent;
    if ( imageChannel==ONE_CHANNEL )
	return Opaque;

    if ( image->getDataType()==GL_UNSIGNED_BYTE && imageChannel>=0 )
    {
	const int step = image->getPixelSizeInBits()/8;
	const unsigned char* start = image->data()+imageChannel;
	const unsigned char* stop = start+image->getTotalSizeInBytes()-step;
	return getTransparencyTypeBytewise( start, stop, step ); 
    }

    bool foundOpaquePixel = false;
    bool foundTransparentPixel = false;

    for ( int r=image->r()-1; r>=0; r-- )
    {
	for ( int t=image->t()-1; t>=0; t-- )
	{
	    for ( int s=image->s()-1; s>=0; s-- )
	    {
		const float val = image->getColor(s,t,r)[imageChannel];
		if ( val<=0.0f )
		    foundTransparentPixel = true;
		else if ( val>=1.0f )
		    foundOpaquePixel = true;
		else
		    return HasTransparencies;
	    }
	}
    }

    if ( foundTransparentPixel )
	return foundOpaquePixel ? OnlyFullTransparencies : FullyTransparent;

    return Opaque;
}


static int encodeBaseChannelPower( osg::Image& image, int nrPowerChannels )
{
    if ( nrPowerChannels<1 )
	return 0;

    const int pixelSizeInBytes  = image.getPixelSizeInBits()/8;

    int maxPowerChannels = pixelSizeInBytes>2 ? 2 : pixelSizeInBytes-1;
    if ( image.getDataType()!=GL_UNSIGNED_BYTE )
	maxPowerChannels = 0;

    if ( nrPowerChannels>maxPowerChannels )
	nrPowerChannels = maxPowerChannels;

    if ( nrPowerChannels<1 )
    {
	std::cerr << "Unsupported image format to encode base channel power" << std::endl;
	return 0;
    }

    unsigned char lut1[256];
    unsigned char lut2[256];

    for ( int idx=0; idx<256; idx++ )
    {
	const float val = idx*idx/255.0f;
	if ( nrPowerChannels==2 )
	{
	    lut1[idx] = (unsigned char) floor( val );
	    lut2[idx] = (unsigned char) floor( 0.5 + (val-lut1[idx])*255.0f );
	}
	else
	    lut1[idx] = (unsigned char) floor( 0.5 + val );
    }

    unsigned char* ptr = image.data();
    const unsigned char* endPtr = ptr + image.getTotalSizeInBytes();

    while ( ptr < endPtr )
    {
	*(ptr+1) = lut1[*ptr];
	if ( nrPowerChannels==2 )
	    *(ptr+2) = lut2[*ptr];

	ptr += pixelSizeInBytes;
    }

    return nrPowerChannels;
}


static void copyImageTile( const osg::Image& srcImage, osg::Image& tileImage, const Vec2i& tileOrigin, const Vec2i& tileSize, int sliceNr=0, ImageDataOrder dataOrder=osgGeo::STR )
{
    const int xSize = tileSize.x();
    const int ySize = tileSize.y();

    tileImage.allocateImage( xSize, ySize, 1, srcImage.getPixelFormat(), srcImage.getDataType(), srcImage.getPacking() );

    const int pixelSize = srcImage.getPixelSizeInBits()/8;
    int xStep = pixelSize; int yStep = pixelSize; int zStep = pixelSize;

    if ( dataOrder==osgGeo::STR )
    {
	yStep *= srcImage.s(); zStep *= srcImage.s()*srcImage.t();
    }
    else if ( dataOrder==osgGeo::SRT )
    {
	zStep *= srcImage.s(); yStep *= srcImage.s()*srcImage.r();
    }
    else if ( dataOrder==osgGeo::TRS )
    {
	xStep *= srcImage.r(); yStep *= srcImage.r()*srcImage.s();
    }
    else if ( dataOrder==osgGeo::TSR )
    {
	xStep *= srcImage.t(); zStep *= srcImage.t()*srcImage.s();
    }
    else if ( dataOrder==osgGeo::RST )
    {
	zStep *= srcImage.t(); xStep *= srcImage.t()*srcImage.r();
    }
    else if ( dataOrder==osgGeo::RTS )
    {
	yStep *= srcImage.r(); xStep *= srcImage.r()*srcImage.t();
    }

    unsigned char* tilePtr = tileImage.data();
    const unsigned char* imagePtr = srcImage.data();
    imagePtr += tileOrigin.x()*xStep + tileOrigin.y()*yStep + sliceNr*zStep;
    yStep -= xSize * xStep;
    xStep -= pixelSize;

    for ( int yCount=0; yCount<ySize; yCount++ )
    {
	for ( int xCount=0; xCount<xSize; xCount++ )
	{
	    for ( int pCount=0; pCount<pixelSize; pCount++ )
		*tilePtr++ = *imagePtr++;

	    imagePtr += xStep;
	}
	imagePtr += yStep;
    }
}


//============================================================================


struct LayeredTextureData : public osg::Referenced
{
			LayeredTextureData(int id)
			    : _id( id )
			    , _origin( 0.0f, 0.0f )
			    , _scale( 1.0f, 1.0f )
			    , _image( 0 )
			    , _imageSource( 0 )
			    , _imageScale( 1.0f, 1.0f )
			    , _imageDataOrder( STR )
			    , _sliceNr( 0 )
			    , _vertex2TextureTrans( 0 )
			    , _freezeDisplay( false )
			    , _nrPowerChannels( 0 )
			    , _textureUnit( -1 )
			    , _filterType(Linear)
			    , _borderColor( 1.0f, 1.0f, 1.0f, 1.0f )
			    , _borderColorSource( 1.0f, 1.0f, 1.0f, 1.0f )
			    , _undefLayerId( -1 )
			    , _undefChannel( 0 )
			    , _undefColor( -1.0f, -1.0f, -1.0f, -1.0f )
			    , _undefColorSource( -1.0f, -1.0f, -1.0f, -1.0f )
			    , _dirtyTileImages( false )
			{
			    for ( int idx=0; idx<4; idx++ )
				_undefChannelRefCount[idx] = 0;
			}

			~LayeredTextureData();

    LayeredTextureData*	clone() const;
    osg::Vec2f		getLayerCoord(const osg::Vec2f& global) const;
    osg::Vec4f		getTextureVec(const osg::Vec2f& global) const;
    void		clearTransparencyType();
    void		adaptColors();
    void		cleanUp();
    void		updateTileImagesIfNeeded() const;
    bool		hasRescaledImage() const;
    void		rescaleImage(int sNew,int tNew,bool inPlace=false);
    bool		do3D() const;

    const int					_id;
    osg::Vec2f					_origin;
    osg::Vec2f					_scale;
    osg::ref_ptr<osg::Image>			_image;
    osg::ref_ptr<osg::Image>			_imageSource;
    Vec2i					_imageSourceSize;
    const unsigned char*			_imageSourceData;
    osg::Vec2f					_imageScale;
    int						_imageModifiedCount;
    bool					_imageModifiedFlag;
    ImageDataOrder				_imageDataOrder;
    int						_sliceNr;
    osg::Matrixf*				_vertex2TextureTrans;
    bool					_freezeDisplay;
    bool					_nrPowerChannels;
    int						_textureUnit;
    FilterType					_filterType;

    osg::Vec4f					_borderColor;
    osg::Vec4f					_borderColorSource;
    int						_undefLayerId;
    int						_undefChannel;
    osg::Vec4f					_undefColor;
    osg::Vec4f					_undefColorSource;
    int						_undefChannelRefCount[4];
    TransparencyType				_transparency[4];

    mutable std::vector<osg::Image*>		_tileImages;
    mutable bool				_dirtyTileImages;
};


LayeredTextureData::~LayeredTextureData()
{
    cleanUp();

    if ( _vertex2TextureTrans )
	delete _vertex2TextureTrans;
}


LayeredTextureData* LayeredTextureData::clone() const
{
    LayeredTextureData* res = new LayeredTextureData( _id );
    res->_origin = _origin;
    res->_scale = _scale; 
    res->_textureUnit = _textureUnit;
    res->_filterType = _filterType;
    res->_freezeDisplay = _freezeDisplay;
    res->_imageModifiedCount = _imageModifiedCount;
    res->_imageScale = _imageScale; 
    res->_imageDataOrder = _imageDataOrder; 
    res->_sliceNr = _sliceNr; 
    res->_imageSource = _imageSource.get();
    res->_vertex2TextureTrans = _vertex2TextureTrans ? new osg::Matrixf(*_vertex2TextureTrans) : 0;

    if ( _image.get()==_imageSource.get() )
	res->_image = _image.get();
    else
	res->_image = (osg::Image*) _image->clone(osg::CopyOp::DEEP_COPY_ALL);

    res->_borderColor = _borderColor;
    res->_borderColor = _borderColorSource;
    res->_undefLayerId = _undefLayerId;
    res->_undefChannel = _undefChannel;
    res->_undefColorSource = _undefColorSource;
    res->_undefColor = _undefColor;
    res->_dirtyTileImages = _dirtyTileImages;

    for ( int idx=0; idx<4; idx++ )
    {
	res->_undefChannelRefCount[idx] = _undefChannelRefCount[idx];
	res->_transparency[idx] = _transparency[idx];
    }

    return res;
}


osg::Vec2f LayeredTextureData::getLayerCoord( const osg::Vec2f& global ) const
{
    osg::Vec2f res = global - _origin;
    res.x() /= _scale.x() * _imageScale.x();
    res.y() /= _scale.y() * _imageScale.y();
    
    return res;
}


void LayeredTextureData::clearTransparencyType()
{
    for ( int idx=0; idx<4; idx++ )
	_transparency[idx] = TransparencyUnknown;
}


void LayeredTextureData::adaptColors()
{
    _undefColor  = _undefColorSource;
    _borderColor = _borderColorSource;

    if ( !_image )
	return;

    const GLenum format = _image->getPixelFormat();

    for ( int idx=0; idx<4; idx++ )
    {
	const int ic = texture2ImageChannel( idx, format );

	if ( ic==ZERO_CHANNEL )
	{
	    _undefColor[idx]  = _undefColor[idx]<0.0f  ? -1.0f : 0.0f;
	    _borderColor[idx] = _borderColor[idx]<0.0f ? -1.0f : 0.0f;
	}
	else if ( ic==ONE_CHANNEL )
	{
	    _undefColor[idx]  = _undefColor[idx]<0.0f  ? -1.0f : 1.0f;
	    _borderColor[idx] = _borderColor[idx]<0.0f ? -1.0f : 1.0f;
	}
	else if ( ic>=0 )
	{
	    const int tc = osgGeo::LayeredTexture::image2TextureChannel( ic, format );
	    _undefColor[idx]  = _undefColorSource[tc];
	    _borderColor[idx] = _borderColorSource[tc];
	}
    }
}


#define GET_PIXEL_INDEX( idx, image, x, y, z, order ) \
    const pixel_int idx = order==STR ? x+(y+z*image->t())*image->s() : \
			  order==SRT ? x+(z+y*image->r())*image->s() : \
			  order==TRS ? z+(x+y*image->s())*image->r() : \
			  order==TSR ? y+(x+z*image->s())*image->t() : \
			  order==RST ? y+(z+x*image->r())*image->t() : \
			/*order==RTS*/ z+(y+x*image->t())*image->r();

#define GET_COLOR_WITHOUT_OVERFLOW( color, image, idx ) \
			  /* OSG multiplies idx by number of BITS per pixel */ \
    if ( idx < 33554432 ) /* (2^32)/128 (upper bound for GL_RGBA+GL_DOUBLE) */ \
	color = image->getColor( pixelIdx ); \
    else if ( image->t() > image->r() ) \
    { \
	const unsigned int size = image->getRowStepInBytes(); \
	const unsigned int blocks = (unsigned int) (idx/size); \
	color = image->getColor( (unsigned int) (idx-blocks*size), blocks ); \
    } \
    else \
    { \
	const unsigned int size = image->getImageSizeInBytes(); \
	const unsigned int blocks = (unsigned int) (idx/size); \
	color = image->getColor( (unsigned int)(idx-blocks*size), 0, blocks ); \
    }


#define GET_COLOR( color, image, x, y, z, order ) \
\
    osg::Vec4f color = _borderColor; \
    if ( x>=0 && x<image->s() && y>=0 && y<image->t() ) \
    { \
	GET_PIXEL_INDEX( pixelIdx, image, x, y, z, order ); \
	GET_COLOR_WITHOUT_OVERFLOW( color, image, pixelIdx ); \
    } \
    else if ( _borderColor[0]<0.0f ) \
    { \
	const int xClamp = x<=0 ? 0 : ( x>=image->s() ? image->s()-1 : x ); \
	const int yClamp = y<=0 ? 0 : ( y>=image->t() ? image->t()-1 : y ); \
	GET_PIXEL_INDEX( pixelIdx, image, xClamp, yClamp, z, order ); \
	GET_COLOR_WITHOUT_OVERFLOW( color, image, pixelIdx ); \
    }

osg::Vec4f LayeredTextureData::getTextureVec( const osg::Vec2f& globalCoord ) const
{
    if ( do3D() || !_image.get() || !_image->s() || !_image->t() || !_image->r() )
	return _borderColor;

    osg::Vec2f local = getLayerCoord( globalCoord );
    if ( _filterType!=Nearest )
	local -= osg::Vec2f( 0.5, 0.5 );

    const ImageDataOrder dataOrder = hasRescaledImage() ? STR : _imageDataOrder;
    const int r = _sliceNr>=_image->r() ? _image->r()-1 : _sliceNr;

    int s = (int) floor( local.x() );
    int t = (int) floor( local.y() );

    GET_COLOR( col00, _image, s, t, r, dataOrder );

    if ( _filterType==Nearest )
	return col00;

    const float sFrac = local.x()-s;
    const float tFrac = local.y()-t;

    if ( !tFrac )
    {
	if ( !sFrac )
	    return col00;

	s++;
	GET_COLOR( col10, _image, s, t, r, dataOrder );
	return col00*(1.0f-sFrac) + col10*sFrac;
    }

    t++;
    GET_COLOR( col01, _image, s, t, r, dataOrder );
    col00 = col00*(1.0f-tFrac) + col01*tFrac;

    if ( !sFrac )
	return col00;

    s++;
    GET_COLOR( col11, _image, s, t, r, dataOrder );
    t--;
    GET_COLOR( col10, _image, s, t, r, dataOrder );

    col10 = col10*(1.0f-tFrac) + col11*tFrac;
    return  col00*(1.0f-sFrac) + col10*sFrac;
}


void LayeredTextureData::cleanUp()
{
    std::vector<osg::Image*>::iterator it = _tileImages.begin();
    for ( ; it!=_tileImages.end(); it++ )
	(*it)->unref();

    _tileImages.clear();
}


void LayeredTextureData::updateTileImagesIfNeeded() const
{
    if ( _dirtyTileImages )
    {
	std::vector<osg::Image*>::iterator it = _tileImages.begin();
	for ( ; it!=_tileImages.end(); it++ )
	    (*it)->dirty();

	_dirtyTileImages = false;
    }
}


bool LayeredTextureData::hasRescaledImage() const 
{ return _image && _image!=_imageSource; }


void LayeredTextureData::rescaleImage( int sNew, int tNew, bool inPlace )
{
    if ( sNew<1 || tNew<1 || !_imageSource )
	return;

    const int sliceNr = _sliceNr>=_imageSource->r() ? _imageSource->r()-1 : _sliceNr;
    osg::ref_ptr<osg::Image> imageToScale = new osg::Image();

    if ( _imageDataOrder==STR  )
    {
	imageToScale->setImage( _imageSource->s(), _imageSource->t(), 1, _imageSource->getInternalTextureFormat(), _imageSource->getPixelFormat(), _imageSource->getDataType(), _imageSource->data(0,0,sliceNr), osg::Image::NO_DELETE, _imageSource->getPacking() );
    }
    else
    {
	copyImageTile( *_imageSource, *imageToScale, Vec2i(0,0), Vec2i(_imageSource->s(),_imageSource->t()), sliceNr, _imageDataOrder ); 
    }

    // scaleImage(.) can only deal with 2D images without stride
    imageToScale->scaleImage( sNew, tNew, 1 ); 

    if ( inPlace && _image && sNew==_image->s() && tNew==_image->t() )
	_image->copySubImage( 0, 0, 0, imageToScale ); 
    else
	_image = imageToScale;
}


bool LayeredTextureData::do3D() const
{
    const osg::Image* image = _vertex2TextureTrans ? _imageSource.get() : 0;
    return image && image->s()>1 && image->t()>1 && image->r()>1;
}


//============================================================================


struct TilingInfo
{
			TilingInfo()			{ reInit(); }

    void		reInit()
			{
			    _envelopeOrigin = osg::Vec2f( 0.0f, 0.0f );
			    _envelopeSize = osg::Vec2f( 0.0f, 0.0f );
			    _smallestScale = osg::Vec2f( 1.0f, 1.0f );
			    _maxTileSize = osg::Vec2f( 0.0f, 0.0f );
			    _needsUpdate = false;
			    _retilingNeeded = true;
			}

    osg::Vec2f		_envelopeOrigin;
    osg::Vec2f		_envelopeSize;
    osg::Vec2f		_smallestScale;
    osg::Vec2f  	_maxTileSize;
    bool		_needsUpdate;
    bool		_retilingNeeded;
};


struct TextureInfo
{
    				TextureInfo()
				    : _isValid( false )
				    , _contextId( -1 )
				    , _nrUnits( 256 )
				    , _maxSize( 65536 )
				    , _nonPowerOf2Support( false )
				    , _shadingSupport( false )
				    , _floatSupport( false )
				    , _nrVertexUnits( 256 )
				{}

    bool			_isValid;
    int				_contextId;
    int				_nrUnits;
    int				_maxSize;
    bool			_nonPowerOf2Support;
    bool			_shadingSupport;
    bool			_floatSupport;
    int				_nrVertexUnits;
};


//============================================================================

#define EPS			1e-5
#define START_RECYCLING_ID	100

LayeredTexture::LayeredTexture()
    : _updateSetupStateSet( false )
    , _maxTextureCopySize( 32*32 )
    , _textureSizePolicy( PowerOf2 )
    , _anisotropicPower( -1 )
    , _seamPower( 0, 0 )
    , _externalTexelSizeRatio( 1.0 )
    , _tilingInfo( new TilingInfo )
    , _texInfo( new TextureInfo )
    , _useNormalizedTexCoords( false )
    , _vertexOffsetLayerId( -1 )
    , _vertexOffsetChannel( 0 )
    , _vertexOffsetFactor( 1.0f )
    , _vertexOffsetBias( 0.0f )
    , _vertexOffsetSpanVec0( 1.0f, 0.0f, 0.0f )
    , _vertexOffsetSpanVec1( 0.0f, 1.0f, 0.0f )
    , _stackUndefLayerId( -1 )
    , _stackUndefChannel( 0 )
    , _stackUndefColor( 0.0f, 0.0f, 0.0f, 0.0f )
    , _invertUndefLayers( false )
    , _allowShaders( true )
    , _maySkipEarlyProcesses( false )
    , _useShaders( false )
    , _enableMipmapping( true )
    , _compositeLayerUpdate( true )
    , _retileCompositeLayer( false )
    , _reInitTiling( false )
    , _isOn( true )
    , _compositeSubsampleSteps( 1 )
{
    _id2idxTable.push_back( -1 );	// ID=0 used to represent ColSeqTexture

    _compositeLayerId = addDataLayer();
}


LayeredTexture::LayeredTexture( const LayeredTexture& lt,
				const osg::CopyOp& co )
    : osgGeo::CallbackObject( lt, co )
    , _updateSetupStateSet( false )
    , _setupStateSet( 0 )
    , _maxTextureCopySize( lt._maxTextureCopySize )
    , _textureSizePolicy( lt._textureSizePolicy )
    , _anisotropicPower( lt._anisotropicPower )
    , _seamPower( lt._seamPower )
    , _externalTexelSizeRatio( lt._externalTexelSizeRatio )
    , _tilingInfo( new TilingInfo(*lt._tilingInfo) )
    , _texInfo( new TextureInfo(*lt._texInfo) )
    , _useNormalizedTexCoords( lt._useNormalizedTexCoords )
    , _vertexOffsetLayerId( lt._vertexOffsetLayerId )
    , _vertexOffsetChannel( lt._vertexOffsetChannel )
    , _vertexOffsetFactor( lt._vertexOffsetFactor )
    , _vertexOffsetBias( lt._vertexOffsetBias )
    , _vertexOffsetSpanVec0( lt._vertexOffsetSpanVec0 )
    , _vertexOffsetSpanVec1( lt._vertexOffsetSpanVec1 )
    , _stackUndefLayerId( lt._stackUndefLayerId )
    , _stackUndefChannel( lt._stackUndefChannel )
    , _stackUndefColor( lt._stackUndefColor )
    , _allowShaders( lt._allowShaders )
    , _maySkipEarlyProcesses( lt._maySkipEarlyProcesses )
    , _useShaders( lt._useShaders )
    , _enableMipmapping( lt._enableMipmapping )
    , _compositeLayerId( lt._compositeLayerId )
    , _compositeLayerUpdate( lt._compositeLayerUpdate )
    , _retileCompositeLayer( false )
    , _reInitTiling( false )
    , _isOn( lt._isOn )
    , _compositeSubsampleSteps( lt._compositeSubsampleSteps )
{
    for ( unsigned int idx=0; idx<lt._dataLayers.size(); idx++ )
    {
	osg::ref_ptr<LayeredTextureData> layer =
		co.getCopyFlags()==osg::CopyOp::DEEP_COPY_ALL
	    ? lt._dataLayers[idx]->clone()
	    : lt._dataLayers[idx];

	layer->ref();
	_dataLayers.push_back( layer );
    }

    for ( unsigned int idx=0; idx<lt._id2idxTable.size(); idx++ )
	_id2idxTable.push_back( lt._id2idxTable[idx] );
    for ( unsigned int idx=0; idx<lt._releasedIds.size(); idx++ )
	_releasedIds.push_back( lt._releasedIds[idx] );
}


LayeredTexture::~LayeredTexture()
{
    std::for_each( _dataLayers.begin(), _dataLayers.end(),
	    	   osg::intrusive_ptr_release );

    std::for_each( _processes.begin(), _processes.end(),
	    	   osg::intrusive_ptr_release );

    delete _tilingInfo;
    delete _texInfo;
}


void LayeredTexture::setUpdateVar( bool& variable, bool yn )
{
/*  Very suitable spot for breakpoints when debugging update issues
    if ( &variable == &_tilingInfo->_needsUpdate )
	std::cout << "_tilingInfo->_needsUpdate = " << yn << std::endl;
    if ( &variable == &_tilingInfo->_retilingNeeded )
	std::cout << "_tilingInfo->_retilingNeeded = " << yn << std::endl;
    if ( &variable == & _updateSetupStateSet )
	std::cout << " _updateSetupStateSet = " << yn << std::endl;
*/

    if ( yn )
	triggerRedrawRequest();

    variable = yn;
}


void LayeredTexture::turnOn( bool yn )
{
    if ( _isOn!=yn )
    {
	setUpdateVar( _tilingInfo->_retilingNeeded, true );
	_isOn = yn;
    }
}


int LayeredTexture::addDataLayer()
{
    _lock.writeLock();

    unsigned int freeId = _id2idxTable.size();
    if ( freeId>=START_RECYCLING_ID && !_releasedIds.empty() )
    {
	freeId = *_releasedIds.begin();
	_releasedIds.erase( _releasedIds.begin() );
    }

    osg::ref_ptr<LayeredTextureData> ltd = new LayeredTextureData( freeId );
    if ( ltd )
    {
	if ( freeId<_id2idxTable.size() )
	    _id2idxTable[freeId] = _dataLayers.size();
	else
	    _id2idxTable.push_back( _dataLayers.size() );

	ltd->ref();
	_dataLayers.push_back( ltd );
    }

    _lock.writeUnlock();
    return ltd ? ltd->_id : -1;
}


void LayeredTexture::raiseUndefChannelRefCount( bool yn, int idx )
{
    int udfIdx = getDataLayerIndex( _stackUndefLayerId );
    int channel = _stackUndefChannel;

    if ( idx>=0 && idx<nrDataLayers() )
    {
	udfIdx = getDataLayerIndex( _dataLayers[idx]->_undefLayerId );
	channel = _dataLayers[idx]->_undefChannel;
    }

    if ( udfIdx!=-1 )
    {
	_dataLayers[udfIdx]->_undefChannelRefCount[channel] += (yn ? 1 : -1);

	if ( _dataLayers[udfIdx]->_textureUnit>=0 )
	{
	    const int cnt = _dataLayers[udfIdx]->_undefChannelRefCount[channel];
	    if ( !cnt || (yn && cnt==1) )
		 setUpdateVar( _tilingInfo->_retilingNeeded, true );
	 }
    }
}


void LayeredTexture::removeDataLayer( int id )
{
    if ( id==_compositeLayerId )
	return; 

    _lock.writeLock();
    int idx = getDataLayerIndex( id );
    if ( idx!=-1 )
    {
	raiseUndefChannelRefCount( false, idx );

	for ( int tc=0; tc<4; tc++ )
	{
	    if ( _dataLayers[idx]->_undefChannelRefCount[tc]>0 )
	    {
		std::cerr << "Broken link to undef layer" << std::endl;
		break;
	    }

	    if ( tc==3 )
		_releasedIds.push_back( id );
	}

	osg::ref_ptr<LayeredTextureData> ltd = _dataLayers[idx];
	_dataLayers.erase( _dataLayers.begin()+idx );
	setUpdateVar( _tilingInfo->_needsUpdate, true );
	ltd->unref();

	_id2idxTable[id] = -1;
	while ( idx < (int)_dataLayers.size() )
	    _id2idxTable[_dataLayers[idx++]->_id]--;

	if (  id==_vertexOffsetLayerId )
	    _vertexOffsetLayerId = -1;
    }

    _lock.writeUnlock();
}


int LayeredTexture::getDataLayerID( int idx ) const
{
    return idx>=0 && idx<(int)_dataLayers.size() ? _dataLayers[idx]->_id : -1;
}


int LayeredTexture::getDataLayerIndex( int id ) const
{
    return id>=0 && id<(int)_id2idxTable.size() ? _id2idxTable[id] : -1;
}


bool LayeredTexture::isDataLayerOK( int id ) const
{
    return getDataLayerImage( id );
}


int LayeredTexture::getTextureUnitNrDims( int unit ) const
{
    for ( int idx=0; unit>=0 && idx<_dataLayers.size(); idx++ )
    {
	if ( _dataLayers[idx]->_textureUnit==unit && _dataLayers[idx]->do3D() )
	    return 3;
    }

    return 2;
}


void LayeredTexture::addAssignTexCrdLine( std::string& code, int unit ) const
{
    char line[50];

    if ( getTextureUnitNrDims(unit)==3 )
	snprintf( line, 50, "texcrd = (vertextrans%d*vertexpos).stp;\n", unit );
    else
	snprintf( line, 50, "texcrd = gl_TexCoord[%d].stp;\n", unit );

    code += line;
}


void LayeredTexture::setDataLayerOrigin( int id, const osg::Vec2f& origin )
{
    const int idx = getDataLayerIndex( id );
    if ( idx!=-1 )
    {
	_dataLayers[idx]->_origin = origin; 
	setUpdateVar( _tilingInfo->_needsUpdate, true );
    }
}


void LayeredTexture::setDataLayerScale( int id, const osg::Vec2f& scale )
{
    const int idx = getDataLayerIndex( id );
    if ( idx!=-1 && scale.x()>=0.0f && scale.y()>0.0f )
    {
	_dataLayers[idx]->_scale = scale;
	setUpdateVar( _tilingInfo->_needsUpdate, true );
    }
}


static void permuteDimensions( osg::Image* image, ImageDataOrder dataOrder )
{
    if ( !image || dataOrder==STR )
	return;

    int sNew=0, tNew=0, rNew=0;

    if ( dataOrder==SRT )
    {
	sNew = image->s(); tNew = image->r(); rNew = image->t();
    }
    else if ( dataOrder==TRS )
    {
	sNew = image->t(); tNew = image->r(); rNew = image->s();
    }
    else if ( dataOrder==TSR )
    {
	sNew = image->t(); tNew = image->s(); rNew = image->r();
    }
    else if ( dataOrder==RST )
    {
	sNew = image->r(); tNew = image->s(); rNew = image->t();
    }
    else if ( dataOrder==RTS )
    {
	sNew = image->r(); tNew = image->t(); rNew = image->s();
    }

    image->setImage( sNew, tNew, rNew, image->getInternalTextureFormat(), image->getPixelFormat(), image->getDataType(), image->data(), image->getAllocationMode(), image->getPacking(), image->getRowLength() );
}


static void permuteDimensionsBack( osg::Image* image, ImageDataOrder dataOrder )
{
    if ( dataOrder==TRS )
	dataOrder = RST;
    else if ( dataOrder==RST )
	dataOrder = TRS;

    permuteDimensions( image, dataOrder );

}


void LayeredTexture::setDataLayerImage( int id, osg::Image* image, bool freezewhile0, int nrPowerChannels )
{
    const int idx = getDataLayerIndex( id );
    if ( idx==-1 )
	return;

    LayeredTextureData& layer = *_dataLayers[idx];

    if ( image || !freezewhile0 )
	setUpdateVar( layer._freezeDisplay, false );
    else if ( layer._imageSource.get() )
	setUpdateVar( layer._freezeDisplay, true );

    if ( image )
    {
	if ( !image->getTotalSizeInBytes() || !image->getPixelFormat() )
	{
	    std::cerr << "Data layer image cannot be set before allocation" << std::endl;
	    return;
	}

	if ( nrPowerChannels>=0 )	// -1 = no need to update power channels
	    layer._nrPowerChannels = encodeBaseChannelPower(*image,nrPowerChannels);

	permuteDimensions( image, layer._imageDataOrder );
	Vec2i newImageSize( image->s(), image->t() );

#ifdef USE_IMAGE_STRIDE
	const bool retile = layer._imageSource.get()!=image || layer._imageSourceData!=image->data() || layer._imageSourceSize!=newImageSize || !layer._tileImages.size();
#else
	const bool retile = true;
#endif

	layer._imageSource = image;
	layer._imageSourceData = image->data();
	layer._imageSourceSize = newImageSize;

	const int s = powerOf2Ceil( image->s() );
	const int t = powerOf2Ceil( image->t() );

	bool rescaleImage = s>image->s() || t>image->t();
	if ( image->s()>=8 && image->t()>=8 && s*t>int(_maxTextureCopySize) )
	    rescaleImage = false;

	if ( rescaleImage && !layer.do3D() && _textureSizePolicy!=AnySize && id!=_compositeLayerId )
	{
	    layer.rescaleImage( s, t, !retile );
	    layer._imageScale.x() = float(image->s()) / float(s);
	    layer._imageScale.y() = float(image->t()) / float(t);
	}
	else
	{
	    layer._image = image;
	    layer._imageScale = osg::Vec2f( 1.0f, 1.0f );
	}

	layer._imageModifiedCount = image->getModifiedCount();
	layer.clearTransparencyType();

	if ( retile || layer.do3D() )
	{
	    layer.adaptColors();
	    setUpdateVar( _tilingInfo->_needsUpdate, true );
	}
	else
	    setUpdateVar( layer._dirtyTileImages, true );
    }
    else if ( layer._image )
    {
	layer._image = 0; 
	layer._imageSource = 0;
	layer._nrPowerChannels = 0;
	layer.adaptColors();
	setUpdateVar( _tilingInfo->_needsUpdate, true );
    }
}


void LayeredTexture::setDataLayerUndefLayerID( int id, int undefId )
{
    const int idx = getDataLayerIndex( id );
    if ( idx!=-1 )
    {
	raiseUndefChannelRefCount( false, idx );

	setUpdateVar( _updateSetupStateSet, true );
	_dataLayers[idx]->_undefLayerId = undefId;

	raiseUndefChannelRefCount( true, idx );
    }
}


void LayeredTexture::setDataLayerUndefChannel( int id, int channel )
{
    const int idx = getDataLayerIndex( id );
    if ( idx!=-1 && channel>=0 && channel<4 )
    {
	raiseUndefChannelRefCount( false, idx );

	setUpdateVar( _updateSetupStateSet, true );

	_dataLayers[idx]->_undefChannel = channel;
	if ( _dataLayers[idx]->_undefLayerId<0 )
	    _dataLayers[idx]->_undefLayerId = id;

	raiseUndefChannelRefCount( true, idx );
    }
}
   

void LayeredTexture::setDataLayerImageUndefColor( int id, const osg::Vec4f& col )
{
    const int idx = getDataLayerIndex( id );
    if ( idx!=-1 )
    {
	for ( int tc=0; tc<4; tc++ )
	{
	    _dataLayers[idx]->_undefColorSource[tc] =
		col[tc]<0.0f ? -1.0f : ( col[tc]>=1.0f ? 1.0f : col[tc] );
	}

	_dataLayers[idx]->adaptColors();
	if ( _dataLayers[idx]->_textureUnit>=0 )
	    setUpdateVar( _updateSetupStateSet, true );
    }
}


void LayeredTexture::setDataLayerBorderColor( int id, const osg::Vec4f& col )
{
    const int idx = getDataLayerIndex( id );
    if ( idx!=-1 )
    {
	_dataLayers[idx]->_borderColorSource = osg::Vec4f(-1.0f,-1.0f,-1.0f,-1.0f);
	if ( col[0]>=0.0f && col[1]>=0.0f && col[2]>=0.0f && col[3]>=0.0f )
	{
	    for ( int tc=0; tc<4; tc++ )
	    {
		_dataLayers[idx]->_borderColorSource[tc] = col[tc]>=1.0f ? 1.0f : col[tc];
	    }
	}

	_dataLayers[idx]->adaptColors();
	if ( _dataLayers[idx]->_textureUnit>=0 )
	    setUpdateVar( _tilingInfo->_retilingNeeded, true );
    }
}


void LayeredTexture::setDataLayerFilterType( int id, FilterType filterType )
{
    const int idx = getDataLayerIndex( id );
    if ( idx!=-1 )
    {
	_dataLayers[idx]->_filterType = filterType;
	if ( _dataLayers[idx]->_textureUnit>=0 )
	    setUpdateVar( _tilingInfo->_retilingNeeded, true );
    }
}


void LayeredTexture::setDataLayerTextureUnit( int id, int unit )
{
    const int idx = getDataLayerIndex( id );
    if ( idx!=-1 )
	_dataLayers[idx]->_textureUnit = unit;
}


void LayeredTexture::setDataLayerImageOrder( int id, ImageDataOrder dataOrder )
{
    const int idx = getDataLayerIndex( id );
    if ( idx==-1 )
	return;

    LayeredTextureData* layer = _dataLayers[idx];

    if ( layer->_imageDataOrder!=dataOrder )
    {
	permuteDimensionsBack( layer->_imageSource, layer->_imageDataOrder );
	layer->_imageDataOrder = dataOrder;
	setDataLayerImage( id, layer->_imageSource, false, -1 );
    }
}


void LayeredTexture::setDataLayerSliceNr( int id, int nr )
{
    if ( nr<0 ) nr=0;

    const int idx = getDataLayerIndex( id );
    if ( idx!=-1 && _dataLayers[idx]->_sliceNr!=nr )
    {
	_dataLayers[idx]->_sliceNr = nr;
	if ( _dataLayers[idx]->hasRescaledImage() )
	{
	    osg::Image* image = _dataLayers[idx]->_image;
	    _dataLayers[idx]->rescaleImage( image->s(), image->t() );
	}
	if ( _dataLayers[idx]->_textureUnit>=0 )
	    setUpdateVar( _tilingInfo->_retilingNeeded, true );
    }
}


void LayeredTexture::setDataLayerVertex2TextureTransform( int id, const osg::Matrixf* trans )
{
    const int idx = getDataLayerIndex( id );
    if ( idx<0 )
	return;

    const bool did3D = _dataLayers[idx]->do3D();

     if ( _dataLayers[idx]->_vertex2TextureTrans )
	 delete _dataLayers[idx]->_vertex2TextureTrans;

     _dataLayers[idx]->_vertex2TextureTrans = trans ? new osg::Matrixf(*trans) : 0;

     if ( did3D != _dataLayers[idx]->do3D() )
	setDataLayerImage( id, _dataLayers[idx]->_imageSource, false, -1 );

     setUpdateVar( _tilingInfo->_needsUpdate, true );
}


bool LayeredTexture::canDoVertexOffsetNow( bool usingFloat ) const
{
    const bool isDataTypeOk = !usingFloat || _texInfo->_floatSupport;
    return _useShaders && _texInfo->_nrVertexUnits>1 && isDataTypeOk;
}


void LayeredTexture::useNormalizedTexCoords( bool yn )
{
    if ( _useNormalizedTexCoords != yn )
    {
	setUpdateVar( _tilingInfo->_retilingNeeded, true );
	_useNormalizedTexCoords = yn;
    }
}


void LayeredTexture::setVertexOffsetLayerID( int id )
{
    setUpdateVar( _updateSetupStateSet, true );
    _vertexOffsetLayerId = id;
}


void LayeredTexture::setVertexOffsetChannel( int channel )
{
    if ( channel>=0 && channel<4 )
    {
	setUpdateVar( _updateSetupStateSet, true );
	_vertexOffsetChannel = channel;
    }
}


void LayeredTexture::setVertexOffsetFactor( float factor )
{
    setUpdateVar( _updateSetupStateSet, true );
    _vertexOffsetFactor = factor;
}


void LayeredTexture::setVertexOffsetBias( float bias )
{
    setUpdateVar( _updateSetupStateSet, true );
    _vertexOffsetBias = bias;
}


void LayeredTexture::setVertexOffsetTexelSpanVectors( const osg::Vec3f& v0, const osg::Vec3f& v1 )
{
    if ( !(v0^v1).length2() )
	return;

    setUpdateVar( _updateSetupStateSet, true );
    _vertexOffsetSpanVec0 = v0;
    _vertexOffsetSpanVec1 = v1;
}


int LayeredTexture::getVertexOffsetLayerID() const
{ return _vertexOffsetLayerId; }


int LayeredTexture::getVertexOffsetChannel() const
{ return _vertexOffsetChannel; }


float LayeredTexture::getVertexOffsetFactor() const
{ return _vertexOffsetFactor; }


float LayeredTexture::getVertexOffsetBias() const
{ return _vertexOffsetBias; }


const osg::Vec3f& LayeredTexture::getVertexOffsetTexelSpanVector( int dim ) const
{
    return dim<1 ? _vertexOffsetSpanVec0 : _vertexOffsetSpanVec1;
}


void LayeredTexture::setVertexOffsetValue( float val, float undefVal, int s, int t, int r )
{
    const osg::Image* img = getDataLayerImage( _vertexOffsetLayerId );

    if ( img && img->getDataType()==GL_UNSIGNED_BYTE )
    {
	unsigned char byte = (unsigned char) (val*255.0f);
	setVertexOffsetValues( &byte, &byte, undefVal, s, t, r );
    }
    else
	setVertexOffsetValues( &val, &val, undefVal, s, t, r );
}


template<class T> void LayeredTexture::setVertexOffsetValues( T* start, T* stop, float undefVal, int s, int t, int r )
{
    if ( start>stop )
	return;

    const int idx = getDataLayerIndex( _vertexOffsetLayerId );
    if ( idx<0 || _dataLayers[idx]->hasRescaledImage() )
	return;

    osg::Image* img = _dataLayers[idx]->_imageSource;
    if ( !img )
	return;

    const bool imgIsFloat = img->getDataType()==GL_FLOAT;
    if ( !imgIsFloat && img->getDataType()!=GL_UNSIGNED_BYTE )
	return;

    const int imgChannel = texture2ImageChannel( _vertexOffsetChannel, img->getPixelFormat() );
    if ( imgChannel<0 || imgChannel>3 )
	return;

    if ( s<0 || t<0 || r<0 || s>=img->s() || t>=img->t() || r>=img->r() )
	return;

    const int imgDataTypeInBits = imgIsFloat ? 32 : 8;
    const int imgStep = img->getPixelSizeInBits() / imgDataTypeInBits; 
    if ( imgStep < 1 )
	return;

    T* imgStart = (T*) img->data(s,t,r);
    const T* imgStop = (const T*) img->data( img->s()-1, img->t()-1, img->r()-1 );

    if ( (imgStop-imgStart)/imgStep < stop-start )
	stop = start + (imgStop-imgStart)/imgStep;

    imgStart += imgChannel;


    const int udfIdx = getDataLayerIndex( _dataLayers[idx]->_undefLayerId  );
    osg::Image* udfImg = udfIdx<0 ? 0 : _dataLayers[udfIdx]->_imageSource;

    int udfImgChannel = -1;
    if ( udfImg )
    {
	if ( udfImg->getDataType()!=GL_UNSIGNED_BYTE )
	    udfImg = 0;
	else if ( udfImg->s()!=img->s() || udfImg->t()!=img->t() || udfImg->r()!=img->r() )
	    udfImg = 0;
	else
	{
	    udfImgChannel = texture2ImageChannel( _dataLayers[idx]->_undefChannel, udfImg->getPixelFormat() );
	    if ( udfImgChannel<0 || udfImgChannel>3 )
		udfImg = 0;
	}
    }

    if ( udfImg )
    {
	const int udfImgStep = udfImg->getPixelSizeInBits()/8; 
	unsigned char* udfImgStart = udfImg->data(s,t,r) + udfImgChannel;

	const float imgUdfVal = _dataLayers[idx]->_undefColor[_vertexOffsetChannel];

	const T undefValT = (T) (imgIsFloat ? undefVal : undefVal*255.0f);
	const T imgUdfValT = (T) (imgIsFloat ? imgUdfVal : imgUdfVal*255.0f);

	const unsigned char udf = _invertUndefLayers ? 0 : 255;
	const unsigned char notUdf = 255 - udf;

	while ( start<=stop )
	{
	    if ( *start==undefValT )
	    {
		*udfImgStart = udf;
		*imgStart = imgUdfVal<0.0f ? *start : imgUdfValT;
		start++;
	    }
	    else
	    {
		*udfImgStart = notUdf;
		*imgStart = *start++;
	    }

	    udfImgStart += udfImgStep;
	    imgStart += imgStep;
	}
    }
    else while ( start<=stop )
    {
	*imgStart = *start++; 
	imgStart += imgStep;
    }
}


float LayeredTexture::getVertexOffsetValue( float undefVal, int s, int t, int r ) const
{
    const osg::Image* img = getDataLayerImage( _vertexOffsetLayerId );
    if ( !img )
	return undefVal;

    const int udfId = getDataLayerUndefLayerID( _vertexOffsetLayerId );
    const osg::Image* udfImg = getDataLayerImage( udfId );
    if ( udfImg )
    {
	const int udfChannel = getDataLayerUndefChannel( _vertexOffsetLayerId );
	const osg::Vec4f color = udfImg->getColor( s, t, r );
	if ( _invertUndefLayers == (color[udfChannel]<0.5f) )
	    return undefVal;
    }

    return img->getColor(s,t,r)[_vertexOffsetChannel];
}


void LayeredTexture::setStackUndefLayerID( int id )
{
    raiseUndefChannelRefCount( false );
    setUpdateVar( _updateSetupStateSet, true );
    _stackUndefLayerId = id;
    raiseUndefChannelRefCount( true );
}


void LayeredTexture::setStackUndefChannel( int channel )
{
    if ( channel>=0 && channel<4 )
    {
	raiseUndefChannelRefCount( false );
	setUpdateVar( _updateSetupStateSet, true );
	_stackUndefChannel = channel;
	raiseUndefChannelRefCount( true );
    }
}


void LayeredTexture::setStackUndefColor( const osg::Vec4f& color )
{
    for ( int idx=0; idx<4; idx++ )
    {
	_stackUndefColor[idx] = color[idx]<=0.0f ? 0.0f :
				color[idx]>=1.0f ? 1.0f : color[idx];
    }
}


int LayeredTexture::getStackUndefLayerID() const
{ return _stackUndefLayerId; }


int LayeredTexture::getStackUndefChannel() const
{ return _stackUndefChannel; }


const osg::Vec4f& LayeredTexture::getStackUndefColor() const
{ return _stackUndefColor; }


#define GET_PROP( funcpostfix, type, variable, undefval ) \
type LayeredTexture::getDataLayer##funcpostfix( int id ) const \
{ \
    const int idx = getDataLayerIndex( id ); \
    static type undefvar = undefval; \
    return idx==-1 ? undefvar : _dataLayers[idx]->variable; \
}

GET_PROP( Image, const osg::Image*, _imageSource.get(), 0 )
GET_PROP( Origin, const osg::Vec2f&, _origin, osg::Vec2f(0.0f,0.0f) )
GET_PROP( TextureUnit, int, _textureUnit, -1 )
GET_PROP( Scale, const osg::Vec2f&, _scale, osg::Vec2f(1.0f,1.0f) )
GET_PROP( FilterType, FilterType, _filterType, Nearest )
GET_PROP( UndefChannel, int, _undefChannel, -1 )
GET_PROP( UndefLayerID, int, _undefLayerId, -1 )
GET_PROP( BorderColor, const osg::Vec4f&, _borderColor, osg::Vec4f(1.0f,1.0f,1.0f,1.0f) )
GET_PROP( ImageUndefColor, const osg::Vec4f&, _undefColor, osg::Vec4f(-1.0f,-1.0f,-1.0f,-1.0f) )
GET_PROP( SliceNr, int, _sliceNr, -1 )
GET_PROP( Vertex2TextureTransform, const osg::Matrixf*, _vertex2TextureTrans, 0 );
GET_PROP( ImageOrder, ImageDataOrder, _imageDataOrder, STR )


TransparencyType LayeredTexture::getDataLayerTransparencyType( int id, int channel ) const
{
    const int idx = getDataLayerIndex( id );
    if ( idx==-1 || channel<0 || channel>3 || !_dataLayers[idx]->_image )
	return FullyTransparent;

    TransparencyType& tt = _dataLayers[idx]->_transparency[channel];

    if ( tt==TransparencyUnknown )
	tt = getImageTransparencyType( _dataLayers[idx]->_image, channel );

    return addOpacity( tt, _dataLayers[idx]->_borderColor[channel] );
}


osg::Vec4f LayeredTexture::getDataLayerTextureVec( int id, const osg::Vec2f& globalCoord ) const
{
    const int idx = getDataLayerIndex( id );
    if ( idx==-1 )
	return osg::Vec4f( -1.0f, -1.0f, -1.0f, -1.0f );

    return _dataLayers[idx]->getTextureVec( globalCoord );
}


LayerProcess* LayeredTexture::getProcess( int idx )
{ return idx>=0 && idx<(int) _processes.size() ? _processes[idx] : 0;  }


const LayerProcess* LayeredTexture::getProcess( int idx ) const
{ return const_cast<LayeredTexture*>(this)->getProcess(idx); }


void LayeredTexture::addProcess( LayerProcess* process )
{
    if ( !process )
	return;

    process->ref();
    _lock.writeLock();
    _processes.push_back( process );
    setUpdateVar( _updateSetupStateSet, true );
    _lock.writeUnlock();
}


void LayeredTexture::removeProcess( const LayerProcess* process )
{
    _lock.writeLock();
    std::vector<LayerProcess*>::iterator it = std::find( _processes.begin(),
	    					    _processes.end(), process );
    if ( it!=_processes.end() )
    {
	process->unref();
	_processes.erase( it );
	setUpdateVar( _updateSetupStateSet, true );
    }

    _lock.writeUnlock();
}

#define MOVE_LAYER( func, cond, inc ) \
void LayeredTexture::func( const LayerProcess* process ) \
{ \
    _lock.writeLock(); \
    std::vector<LayerProcess*>::iterator it = std::find( _processes.begin(), \
	    					    _processes.end(), process);\
    if ( it!=_processes.end() ) \
    { \
	std::vector<LayerProcess*>::iterator neighbor = it inc; \
	if ( cond ) \
	{ \
	    std::swap( *it, *neighbor ); \
	    setUpdateVar( _updateSetupStateSet, true ); \
	} \
    } \
    _lock.writeUnlock(); \
}

MOVE_LAYER( moveProcessEarlier, it!=_processes.begin(), -1 )
MOVE_LAYER( moveProcessLater, neighbor!=_processes.end(), +1 )


void LayeredTexture::setGraphicsContextID( int id )
{
    _texInfo->_contextId = id;
    _texInfo->_isValid = false;
    _texInfo->_shadingSupport = false;
    updateTextureInfoIfNeeded();
}


int LayeredTexture::getGraphicsContextID() const
{ return _texInfo->_contextId; }


static int _maxTexSizeOverride = -1;
void LayeredTexture::overrideGraphicsContextMaxTextureSize( int maxTexSize )
{ _maxTexSizeOverride = maxTexSize; }


void LayeredTexture::updateTextureInfoIfNeeded() const
{
    if ( _texInfo->_isValid )
	return;

    // Shortcut to force continuous redraw until texture info is available
    const_cast<LayeredTexture*>(this)->triggerRedrawRequest();

    const int maxContextID = (int) osg::GraphicsContext::getMaxContextID();
    for( int contextID=0; contextID<=maxContextID; contextID++ )
    {
	if ( _texInfo->_contextId>=0 && _texInfo->_contextId!=contextID )
	    continue;

#if OSG_MIN_VERSION_REQUIRED(3,3,3)
	osg::GLExtensions* ext = osg::GLExtensions::Get(contextID,_texInfo->_contextId>=0);
	if ( !ext )
	    continue;
#else
	const osg::VertexProgram::Extensions* vertExt = osg::VertexProgram::getExtensions( contextID, _texInfo->_contextId>=0 );
	const osg::FragmentProgram::Extensions* fragExt = osg::FragmentProgram::getExtensions( contextID, _texInfo->_contextId>=0 );

	const osg::Texture::Extensions* texExt = osg::Texture::getExtensions( contextID, _texInfo->_contextId>=0 );

	if ( !vertExt || !fragExt || !texExt )
	    continue;
#endif

#if OSG_MIN_VERSION_REQUIRED(3,3,1)
	osg::GraphicsContext::GraphicsContexts contexts = osg::GraphicsContext::getRegisteredGraphicsContexts( contextID );
	int maxUnits = -1;
	for( int idx=0; idx<(int)contexts.size(); idx++ )
	{
	    if ( maxUnits<0 || maxUnits>contexts[idx]->getState()->getMaxTextureUnits() )
		maxUnits = contexts[idx]->getState()->getMaxTextureUnits();
	}

	if ( osg::getGLVersionNumber()>=2.0 || osg::isGLExtensionSupported(contextID,"GL_ARB_vertex_shader") || OSG_GLES2_FEATURES )
	{
	    if ( maxUnits%3==0 ) maxUnits/=3;	// Need max units per shader
	}

	if ( maxUnits>=0 )
	{
	    if ( !_texInfo->_isValid || _texInfo->_nrUnits>maxUnits )
		_texInfo->_nrUnits = maxUnits;
	}
#else
	if ( !_texInfo->_isValid || _texInfo->_nrUnits>texExt->numTextureUnits() )
	    _texInfo->_nrUnits = texExt->numTextureUnits();
#endif

#if OSG_MIN_VERSION_REQUIRED(3,3,3)
	if ( !_texInfo->_isValid || _texInfo->_maxSize>ext->maxTextureSize )
	{
	    _texInfo->_maxSize = ext->maxTextureSize;
	    while ( _maxTexSizeOverride>0 && _texInfo->_maxSize>_maxTexSizeOverride && _texInfo->_maxSize>64 )
	    {
		_texInfo->_maxSize /= 2;
	    }
	}

	if ( !_texInfo->_isValid || _texInfo->_nonPowerOf2Support )
	    _texInfo->_nonPowerOf2Support = ext->isNonPowerOfTwoTextureSupported( osg::Texture::LINEAR_MIPMAP_LINEAR );;

	if ( !_texInfo->_isValid || _texInfo->_shadingSupport )
	    _texInfo->_shadingSupport = ext->isVertexProgramSupported && ext->isFragmentProgramSupported && _texInfo->_nrUnits>0;
#else
	if ( !_texInfo->_isValid || _texInfo->_maxSize>texExt->maxTextureSize() )
	{
	    _texInfo->_maxSize = texExt->maxTextureSize();
	    while ( _maxTexSizeOverride>0 && _texInfo->_maxSize>_maxTexSizeOverride && _texInfo->_maxSize>64 )
	    {
		_texInfo->_maxSize /= 2;
	    }
	}

	if ( !_texInfo->_isValid || _texInfo->_nonPowerOf2Support )
	    _texInfo->_nonPowerOf2Support = texExt->isNonPowerOfTwoTextureSupported( osg::Texture::LINEAR_MIPMAP_LINEAR );

	if ( !_texInfo->_isValid || _texInfo->_shadingSupport )
	    _texInfo->_shadingSupport = vertExt->isVertexProgramSupported() && fragExt->isFragmentProgramSupported() && _texInfo->_nrUnits>0;
#endif

	if ( !_texInfo->_isValid || _texInfo->_floatSupport )
	    _texInfo->_floatSupport = osg::isGLExtensionOrVersionSupported(contextID,"GL_ARB_texture_float",3.0); 

	if ( !_texInfo->_isValid || _texInfo->_nrVertexUnits>_texInfo->_nrUnits )
	    // TODO: Should get GL_MAX_VERTEX_TEXTURE_IMAGE_UNITS from OSG
	    _texInfo->_nrVertexUnits = _texInfo->_nrUnits;

	_texInfo->_isValid = true;
	_tilingInfo->_retilingNeeded = true;
    }
}


void LayeredTexture::updateTilingInfoIfNeeded() const
{
    if ( !_tilingInfo->_needsUpdate )
	return;

    _tilingInfo->reInit();

    std::vector<LayeredTextureData*>::const_iterator it = _dataLayers.begin();

    osg::Vec2f minBound( 0.0f, 0.0f );
    osg::Vec2f maxBound( 0.0f, 0.0f );
    osg::Vec2f minScale( 0.0f, 0.0f );
    osg::Vec2f minNoPow2Size( 0.0f, 0.0f );

    bool validLayerFound = false;

    for ( ; it!=_dataLayers.end(); it++ )
    {
	if ( !(*it)->_image.get() || (*it)->do3D() || (*it)->_id==_compositeLayerId )
	    continue;

	const osg::Vec2f scale( (*it)->_scale.x() * (*it)->_imageScale.x(),
				(*it)->_scale.y() * (*it)->_imageScale.y() );

	const osg::Vec2f layerSize( (*it)->_image->s() * scale.x(),
				    (*it)->_image->t() * scale.y() );

	const osg::Vec2f bound = layerSize + (*it)->_origin;

	if ( !validLayerFound || bound.x()>maxBound.x() )
	    maxBound.x() = bound.x();
	if ( !validLayerFound || bound.y()>maxBound.y() )
	    maxBound.y() = bound.y();
	if ( !validLayerFound || (*it)->_origin.x()<minBound.x() )
	    minBound.x() = (*it)->_origin.x();
	if ( !validLayerFound || (*it)->_origin.y()<minBound.y() )
	    minBound.y() = (*it)->_origin.y();
	if ( !validLayerFound || scale.x()<minScale.x() )
	    minScale.x() = scale.x();
	if ( !validLayerFound || scale.y()<minScale.y() )
	    minScale.y() = scale.y();

	if ( (minNoPow2Size.x()<=0.0f || layerSize.x()<minNoPow2Size.x()) &&
	     (*it)->_image->s() != powerOf2Ceil((*it)->_image->s()) )
	    minNoPow2Size.x() = layerSize.x();

	if ( (minNoPow2Size.y()<=0.0f || layerSize.y()<minNoPow2Size.y()) &&
	     (*it)->_image->t() != powerOf2Ceil((*it)->_image->t()) )
	    minNoPow2Size.y() = layerSize.y();

	validLayerFound = true;
    }

    if ( !validLayerFound )
	return;

    _tilingInfo->_envelopeSize = maxBound - minBound;
    _tilingInfo->_envelopeOrigin = minBound;
    _tilingInfo->_smallestScale = minScale;

    for ( int dim=0; dim<=1; dim++ )
    {
	_tilingInfo->_maxTileSize[dim] = ceil( _tilingInfo->_envelopeSize[dim]/ minScale[dim] );
	if ( minNoPow2Size[dim]>0.0f )
	    _tilingInfo->_maxTileSize[dim] = minNoPow2Size[dim] / minScale[dim];
    }
}


bool LayeredTexture::isDisplayFrozen() const
{
    std::vector<LayeredTextureData*>::const_iterator lit = _dataLayers.begin();
    for ( ; lit!=_dataLayers.end(); lit++ )
    {
	if ( (*lit)->_freezeDisplay )
	    return true;
    }

    return false;
}


bool LayeredTexture::needsRetiling() const
{
    if ( isDisplayFrozen() )
	return false;

    std::vector<LayeredTextureData*>::const_iterator lit = _dataLayers.begin();
    for ( ; lit!=_dataLayers.end(); lit++ )
	(*lit)->updateTileImagesIfNeeded();

    updateTilingInfoIfNeeded();
    updateTextureInfoIfNeeded();

    return _tilingInfo->_retilingNeeded || _retileCompositeLayer;
}


bool LayeredTexture::isEnvelopeDefined() const
{
    updateTilingInfoIfNeeded();
    return _tilingInfo->_envelopeSize[0]>0.0f && _tilingInfo->_envelopeSize[1]>0.0f;
}


osg::Vec2f LayeredTexture::imageEnvelopeSize() const
{
    updateTilingInfoIfNeeded();
    return _tilingInfo->_envelopeSize;
}


osg::Vec2f LayeredTexture::textureEnvelopeSize() const
{
    updateTilingInfoIfNeeded();
    return _tilingInfo->_envelopeSize - _tilingInfo->_smallestScale;
}


osg::Vec2f LayeredTexture::envelopeCenter() const
{
    updateTilingInfoIfNeeded();
    return _tilingInfo->_envelopeOrigin + textureEnvelopeSize()*0.5;
}


int LayeredTexture::maxTextureSize() const
{
    updateTextureInfoIfNeeded();
    return _texInfo->_isValid ? _texInfo->_maxSize : -1;
}


int LayeredTexture::nrTextureUnits() const
{
    updateTextureInfoIfNeeded();
    return _texInfo->_isValid ? _texInfo->_nrUnits : -1;
}


void LayeredTexture::reInitTiling( float texelSizeRatio )
{
    _reInitTiling = true;
    updateTilingInfoIfNeeded();
    updateTextureInfoIfNeeded();
    assignTextureUnits();

    std::vector<LayeredTextureData*>::iterator lit = _dataLayers.begin();
    for ( ; lit!=_dataLayers.end(); lit++ )
	(*lit)->cleanUp();

    setUpdateVar( _tilingInfo->_retilingNeeded, false );
    _externalTexelSizeRatio = texelSizeRatio; 
    _reInitTiling = false;
}


void LayeredTexture::setAnisotropicPower( int power )
{
    if ( power<0 )
	power = -1;

    if ( power!=_anisotropicPower )
    {
	_anisotropicPower = power;
	setUpdateVar( _tilingInfo->_retilingNeeded, true );
    }
}


float LayeredTexture::getAnisotropicPower() const
{ return _anisotropicPower; }


float LayeredTexture::getMaxAnisotropy( int layerIdx ) const
{
    if ( _anisotropicPower<0 )
	return 1.0;

    float ratio = 1.0f;
    if ( _externalTexelSizeRatio  )
	ratio = fabs( _externalTexelSizeRatio );

    if ( layerIdx>=0 && layerIdx<nrDataLayers() )
    {
	LayeredTextureData* layer = _dataLayers[layerIdx];
	if ( layer->_scale[0] && layer->_scale[1] )
	    ratio *= layer->_scale[0] / layer->_scale[1];
    }

    if ( ratio<1.0f )
	ratio = 1.0f/ratio;
    
    float maxAnisotropy = powerOf2Ceil( (int) floor(ratio+0.5) );

    int power = (int)_anisotropicPower;
    while ( (--power)>=0 )
	maxAnisotropy *= 2.0;

    return maxAnisotropy;
}


void LayeredTexture::setSeamPower( int power, int dim )
{
    if ( dim!=1 )
	_seamPower.x() = power;
    if ( dim!=0 )
	_seamPower.y() = power;

    setUpdateVar( _tilingInfo->_retilingNeeded, true );
}


int LayeredTexture::getSeamPower( int dim ) const
{ return dim>0 ? _seamPower.y() : _seamPower.x(); }


int LayeredTexture::getSeamWidth( int layerIdx, int dim ) const
{
    if ( layerIdx<0 || layerIdx>=nrDataLayers() || dim<0 || dim>1 )
	return 1;

    float ratio = 1.0f;
    if ( _externalTexelSizeRatio )
	ratio = fabs( _externalTexelSizeRatio );
    if ( dim==1 )
	ratio = 1.0f / ratio;

    LayeredTextureData* layer = _dataLayers[layerIdx];
    ratio *= _tilingInfo->_smallestScale[dim] / layer->_scale[1-dim];

    int seamWidth = (int) floor( ratio+0.5 );

    if ( seamWidth>_texInfo->_maxSize/4 )
	seamWidth = _texInfo->_maxSize/4;
    if ( seamWidth<1 )
	seamWidth = 1;

    seamWidth = powerOf2Ceil( seamWidth );

    int seamPower = getSeamPower( dim );
    while ( (--seamPower)>=0 && seamWidth<_texInfo->_maxSize/4 )
	seamWidth *= 2;

    return seamWidth;
}


int LayeredTexture::getTileOverlapUpperBound( int dim ) const
{
     if ( dim<0 || dim>1 )
	 return 0;

    float maxScaledWidth = 1.0f;

    for ( int idx=0; idx<nrDataLayers(); idx++ )
    {
	LayeredTextureData* layer = _dataLayers[idx];
	if ( !layer->_image.get() || layer->do3D() || layer->_id==_compositeLayerId )
	    continue;

	const float scaledWidth = layer->_scale[dim] * getSeamWidth(idx,dim);
	if ( scaledWidth > maxScaledWidth )
	    maxScaledWidth = scaledWidth;
    }

    // Include one extra seam width in upper bound to cover seam alignment
    return 3 * (int) ceil(maxScaledWidth/_tilingInfo->_smallestScale[dim]);
}


bool LayeredTexture::planTiling( unsigned short brickSize, std::vector<float>& xTickMarks, std::vector<float>& yTickMarks, bool strict ) const
{
    const Vec2i requestedSize( brickSize, brickSize );
    Vec2i actualSize = requestedSize;

    if ( !strict && _textureSizePolicy!=AnySize )
    {
	const osg::Vec2f& maxTileSize = _tilingInfo->_maxTileSize;

	for ( int dim=0; dim<=1; dim++ )
	{
	    const int overlap = getTileOverlapUpperBound( dim ); 
	    actualSize[dim] = powerOf2Ceil( brickSize+overlap );

	    // To minimize absolute difference with requested brick size
	    if ( brickSize<0.75*actualSize[dim]-overlap )
	    {
		// But let's stay above half the requested brick size
		if ( brickSize<actualSize[dim]-2*overlap )
		    actualSize[dim] /= 2;
	    }

	    bool hadToReduceTileSize = false;
	    while ( actualSize[dim]>maxTileSize[dim] && maxTileSize[dim]>0.0f )
	    {
		hadToReduceTileSize = true;
		actualSize[dim] /= 2;
	    }

	    if ( hadToReduceTileSize && overlap>0.75*actualSize[dim] )
	    {
		// Cut down seam if it (almost) overgrows the tile
		actualSize[dim] /= 4;
		if ( actualSize[dim]>brickSize )
		    actualSize[dim] = brickSize;
	    }
	    else 
		actualSize[dim] -= overlap;

	    // std::cout << "Tile size: " << actualSize[dim] << ", overlap: " << overlap << std::endl;
	}
    }

    const osg::Vec2f& size = _tilingInfo->_envelopeSize;
    const osg::Vec2f& minScale = _tilingInfo->_smallestScale;

    bool xRes = divideAxis( size.x()/minScale.x(), actualSize.x(), xTickMarks );
    bool yRes = divideAxis( size.y()/minScale.y(), actualSize.y(), yTickMarks );

    return xRes && yRes && actualSize==requestedSize;
}


bool LayeredTexture::divideAxis( float totalSize, int brickSize,
				 std::vector<float>& tickMarks ) const
{
    tickMarks.push_back( 0.0f );

    if ( totalSize <= 1.0f ) 
    {
	// to display something if no layers or images defined yet
	tickMarks.push_back( 1.0f );
	return false;
    }

    int stepSize = _texInfo->_maxSize;
    if ( brickSize < stepSize )
	stepSize = brickSize<1 ? 1 : brickSize;

    for ( float fidx=stepSize; fidx+EPS<totalSize-1.0f; fidx += stepSize )
	tickMarks.push_back( fidx );

    tickMarks.push_back( totalSize-1.0f );

    return stepSize==brickSize;
}


osg::Vec2 LayeredTexture::tilingPlanResolution() const
{
    return osg::Vec2( 1.0f / _tilingInfo->_smallestScale[0],
		      1.0f / _tilingInfo->_smallestScale[1] );
}


osg::StateSet* LayeredTexture::createCutoutStateSet( const osg::Vec2f& origin, const osg::Vec2f& opposite, std::vector<LayeredTexture::TextureCoordData>& tcData, const VertexOffsetCutoutInfo* vertexOffsetInfo ) const
{
    tcData.clear();
    osg::ref_ptr<osg::StateSet> stateset = new osg::StateSet;

    const osg::Vec2f smallestScale = _tilingInfo->_smallestScale;
    osg::Vec2f globalOrigin( smallestScale.x() * (origin.x()+0.5),
			     smallestScale.y() * (origin.y()+0.5) );
    globalOrigin += _tilingInfo->_envelopeOrigin;

    osg::Vec2f globalOpposite( smallestScale.x() * (opposite.x()+0.5),
			       smallestScale.y() * (opposite.y()+0.5) );
    globalOpposite += _tilingInfo->_envelopeOrigin;

    for ( int idx=nrDataLayers()-1; idx>=0; idx-- )
    {
	LayeredTextureData* layer = _dataLayers[idx];
	if ( layer->_textureUnit<0 )
	    continue;

	if ( layer->do3D() )
	{
	    add3DTextureToStateSet( *layer, tcData, *stateset );
	    continue;
	}

	const osg::Vec2f localOrigin = layer->getLayerCoord( globalOrigin );
	const osg::Vec2f localOpposite = layer->getLayerCoord( globalOpposite );

	osg::Image* image = layer->_image;
	if ( !image || !image->s() || !image->t() )
	    continue;

	const Vec2i imageSize( image->s(), image->t() );
	Vec2i hasBorderArea, tileOrigin, tileSize;

	bool overflowErrorMsg = false;
	bool resizeHint = false;

	for ( int dim=0; dim<=1; dim++ )
	{
	    hasBorderArea[dim] = localOrigin[dim]<-EPS || localOpposite[dim]>imageSize[dim]+EPS;

	    if ( localOpposite[dim]<EPS || localOrigin[dim]>imageSize[dim]-EPS )
	    {
		tileOrigin[dim] = localOpposite[dim]<EPS ? 0 : imageSize[dim]-1;
		tileSize[dim] = 1;  // More needed only if mipmapping-induced
		continue;	    // artifacts in extended-edge-pixel borders
	    }			    // become an issue. 

	    const int orgSeamWidth = getSeamWidth( idx, dim );
	    for ( int width=orgSeamWidth; ; width/=2 )
	    {
		tileOrigin[dim] = (int) floor( localOrigin[dim]-0.5 );
		int tileOpposite = (int) ceil( localOpposite[dim]+0.5 );

		/* width==0 represents going back to original seam width
		   after anything smaller does not solve the puzzle either. */ 
		const int seamWidth = width ? width : orgSeamWidth;

		tileOrigin[dim] -= seamWidth/2;
		if ( tileOrigin[dim]<=0 )
		    tileOrigin[dim] = 0;
		else 
		    // Align seams of subsequent tiles to minimize artifacts
		    tileOrigin[dim] -= tileOrigin[dim]%seamWidth; 

		tileOpposite += ((3*seamWidth)/2) - 1;
		tileOpposite -= tileOpposite%seamWidth;

		if ( tileOpposite>imageSize[dim] )  // Cannot guarantee seam
		    tileOpposite = imageSize[dim];  // alignment at last tile

		tileSize[dim] = tileOpposite - tileOrigin[dim];
		if ( tileSize[dim]>_texInfo->_maxSize )
		{
		    if ( seamWidth>1 )
			continue;

		    if ( !overflowErrorMsg )
		    {
			overflowErrorMsg = true;
			std::cerr << "Cut-out exceeds maximum texture size: " << _texInfo->_maxSize << std::endl;
		    }

		    tileSize[dim] = _texInfo->_maxSize;
		    hasBorderArea[dim] = true;
		    break;
		}

		if ( seamWidth==orgSeamWidth && usedTextureSizePolicy()==AnySize )
		    /* Note that seam alignment will be broken if OpenGL
		       implementation of AnySize is going to resample. */
		    break;

		const int powerOf2Size = powerOf2Ceil( tileSize[dim] );

		if ( powerOf2Size>imageSize[dim] )
		{
		    if ( orgSeamWidth>1 && width>0 )
			continue;

		    if ( !resizeHint ) 
		    {
			resizeHint = true;
			if ( _textureSizePolicy!=AnySize )
			{
			    std::cerr << "Can't avoid texture resampling for this cut-out: increase MaxTextureCopySize" << std::endl ;
			}
		    }
		    break;
		}

		const int extraSeam = (powerOf2Size-tileSize[dim]) / 2;
		// Be careful not to break the current seam alignments
		tileOrigin[dim] -= seamWidth * (extraSeam/seamWidth);

		tileSize[dim] = powerOf2Size;

		if ( tileOrigin[dim]<0 )
		    tileOrigin[dim] = 0;

		if ( tileOrigin[dim]+tileSize[dim] > imageSize[dim] )
		    tileOrigin[dim] = imageSize[dim] - tileSize[dim];

		break;
	    }
	}

	int sliceNr = layer->_sliceNr;
	if ( sliceNr>=image->r() )
	    sliceNr = image->r()-1;

	ImageDataOrder dataOrder( STR );
	if ( !layer->hasRescaledImage() ) 
	    dataOrder = layer->_imageDataOrder;

	osg::ref_ptr<osg::Image> tileImage = new osg::Image;

#ifdef USE_IMAGE_STRIDE
	// OpenGL crashes when resizing image with stride
	if ( (dataOrder==STR || dataOrder==SRT) && !resizeHint )
	{
	    unsigned char* dataOrigin = image->data(tileOrigin.x(),tileOrigin.y(),sliceNr);
	    int rowLength = image->s();

	    if ( dataOrder==SRT )
	    {
		dataOrigin = image->data(tileOrigin.x(),sliceNr,0);
		rowLength *= image->r();
		dataOrigin += tileOrigin.y() * rowLength * image->getPixelSizeInBits()/8;
	    }

	    tileImage->setUserData( image );
	    tileImage->setImage( tileSize.x(), tileSize.y(), 1, image->getInternalTextureFormat(), image->getPixelFormat(), image->getDataType(), dataOrigin, osg::Image::NO_DELETE, image->getPacking(), rowLength ); 

	    tileImage->ref();
	    const_cast<LayeredTexture*>(this)->_lock.writeLock();
	    layer->_tileImages.push_back( tileImage );
	    const_cast<LayeredTexture*>(this)->_lock.writeUnlock();
	}
	else
#endif
	    copyImageTile( *image, *tileImage, tileOrigin, tileSize, sliceNr, dataOrder );

	osg::Texture::WrapMode xWrapMode = osg::Texture::CLAMP_TO_EDGE;
	if ( layer->_borderColor[0]>=0.0f && hasBorderArea.x() )
	    xWrapMode = osg::Texture::CLAMP_TO_BORDER;

	osg::Texture::WrapMode yWrapMode = osg::Texture::CLAMP_TO_EDGE;
	if ( layer->_borderColor[0]>=0.0f && hasBorderArea.y() )
	    yWrapMode = osg::Texture::CLAMP_TO_BORDER;

	osg::Vec2f tc00, tc01, tc10, tc11;
	tc00.x() = (localOrigin.x() - tileOrigin.x()) / tileSize.x();
	tc00.y() = (localOrigin.y() - tileOrigin.y()) / tileSize.y();
	tc11.x() = (localOpposite.x()-tileOrigin.x()) / tileSize.x();
	tc11.y() = (localOpposite.y()-tileOrigin.y()) / tileSize.y();

	char uniformName[20];
	if ( _useNormalizedTexCoords )
	{
	    const osg::Vec4f texCrdBias( tc00.x(), tc00.y(), 0.0f, 0.0f );
	    osg::Vec4f texCrdFactor( tc11.x(), tc11.y(), 1.0f, 1.0f );
	    texCrdFactor -= texCrdBias;

	    tc00.x() = 0.0f;
	    tc00.y() = 0.0f;
	    tc11.x() = (globalOpposite.x()-globalOrigin.x()) / _tilingInfo->_envelopeSize.x();
	    tc11.y() = (globalOpposite.y()-globalOrigin.y()) / _tilingInfo->_envelopeSize.y();

	    texCrdFactor.x() /= tc11.x();
	    texCrdFactor.y() /= tc11.y();

	    snprintf( uniformName, 20, "texcrdfactor%d", layer->_textureUnit );
	    stateset->addUniform( new osg::Uniform(uniformName,texCrdFactor) );
	    snprintf( uniformName, 20, "texcrdbias%d", layer->_textureUnit );
	    stateset->addUniform( new osg::Uniform(uniformName,texCrdBias) );
	}

	tc01 = osg::Vec2f( tc11.x(), tc00.y() );
	tc10 = osg::Vec2f( tc00.x(), tc11.y() );

	tcData.push_back( TextureCoordData( layer->_textureUnit, tc00, tc01, tc10, tc11 ) );

	osg::ref_ptr<osg::Texture2D> texture = new osg::Texture2D( tileImage.get() );
	texture->setResizeNonPowerOfTwoHint( resizeHint );
	texture->setWrap( osg::Texture::WRAP_S, xWrapMode );
	texture->setWrap( osg::Texture::WRAP_T, yWrapMode );

	texture->setMaxAnisotropy( osg::maximum(getMaxAnisotropy(idx),1.0f) );

	osg::Texture::FilterMode filterMode = layer->_filterType==Nearest ? osg::Texture::NEAREST : osg::Texture::LINEAR;
	texture->setFilter( osg::Texture::MAG_FILTER, filterMode );

	if ( _enableMipmapping )
	    filterMode = layer->_filterType==Nearest ? osg::Texture::NEAREST_MIPMAP_NEAREST : osg::Texture::LINEAR_MIPMAP_LINEAR;

	texture->setFilter( osg::Texture::MIN_FILTER, filterMode );

	texture->setBorderColor( layer->_borderColor );

	stateset->setTextureAttributeAndModes( layer->_textureUnit, texture.get() );
	snprintf( uniformName, 20, "texsize%d", layer->_textureUnit );
	const osg::Vec2 texSize( tileSize.x(), tileSize.y() );
	stateset->addUniform( new osg::Uniform(uniformName,texSize) );

	if ( isDataLayerOK(_vertexOffsetLayerId) )
	{
	    const int udfId = getDataLayerUndefLayerID( _vertexOffsetLayerId );
	    if ( layer->_id==_vertexOffsetLayerId || layer->_id==udfId )
	    {
		float lod = vertexOffsetInfo ? vertexOffsetInfo->_lod : 0.0f;
		if ( layer->_id!=_vertexOffsetLayerId )
		{
		    const osg::Vec2f refScale = getDataLayerScale(_vertexOffsetLayerId);
		    const float xRatio = refScale.x()/layer->_scale.x();
		    const float yRatio = refScale.y()/layer->_scale.y();
		    lod += xRatio>yRatio ? log(xRatio)/log(2.0f) : log(yRatio)/log(2.0f);
		}
		else 
		    stateset->addUniform( new osg::Uniform("offsetifudf",(vertexOffsetInfo ? vertexOffsetInfo->_offsetIfUndef : 0.0f)) );

		if ( lod<0.0f )
		    lod = 0.0f;

		snprintf( uniformName, 20, "lod%d", layer->_textureUnit );
		stateset->addUniform( new osg::Uniform(uniformName,lod) );
	    }
	}
    }

    return stateset.release();
}


void LayeredTexture::add3DTextureToStateSet( const LayeredTextureData& layer, std::vector<LayeredTexture::TextureCoordData>& tcData, osg::StateSet& stateset ) const
{
    osg::Image* image = layer._image;
    if ( !image || !image->s() || !image->t() || !image->r() )
	return;

    const osg::Vec2f dummy( 0.0f, 0.0f );
    tcData.push_back( TextureCoordData( layer._textureUnit, dummy, dummy, dummy, dummy ) );

    osg::ref_ptr<osg::Texture3D> texture = new osg::Texture3D( image );
    texture->setResizeNonPowerOfTwoHint( false );

    osg::Texture::WrapMode wrapMode = osg::Texture::CLAMP_TO_EDGE;
    if ( layer._borderColor[0]>=0.0f )
	wrapMode = osg::Texture::CLAMP_TO_BORDER;

    texture->setWrap( osg::Texture::WRAP_S, wrapMode );
    texture->setWrap( osg::Texture::WRAP_T, wrapMode );
    texture->setWrap( osg::Texture::WRAP_R, wrapMode );

    //const int idx = getDataLayerIndex( layer._id );
    //texture->setMaxAnisotropy( osg::maximum(getMaxAnisotropy(idx),1.0f) );

    osg::Texture::FilterMode filterMode = layer._filterType==Nearest ? osg::Texture::NEAREST : osg::Texture::LINEAR;
    texture->setFilter( osg::Texture::MAG_FILTER, filterMode );

    //if ( _enableMipmapping )
    //	filterMode = layer._filterType==Nearest ? osg::Texture::NEAREST_MIPMAP_NEAREST : osg::Texture::LINEAR_MIPMAP_LINEAR;

    texture->setFilter( osg::Texture::MIN_FILTER, filterMode );
    texture->setBorderColor( layer._borderColor );

    stateset.setTextureAttributeAndModes( layer._textureUnit, texture.get() );

    char uniformName[20];
    snprintf( uniformName, 20, "texsize%d", layer._textureUnit );
    const osg::Vec3f texSize( image->s(), image->t(), image->r() );
    stateset.addUniform( new osg::Uniform(uniformName,texSize) );

    snprintf( uniformName, 20, "vertextrans%d", layer._textureUnit );
    stateset.addUniform( new osg::Uniform(uniformName,*layer._vertex2TextureTrans) );
}


void LayeredTexture::updateSetupStateSet()
{
    setUpdateVar( _updateSetupStateSet, true );
}


osg::StateSet* LayeredTexture::getSetupStateSet()
{
    updateSetupStateSetIfNeeded();
    return _setupStateSet;
}


void LayeredTexture::updateSetupStateSetIfNeeded()
{
    if ( isDisplayFrozen() )
	return;

    _lock.readLock();

    if ( !_setupStateSet )
    {
	_setupStateSet = new osg::StateSet;
	setUpdateVar( _updateSetupStateSet, true );
	setRenderingHint( false );
    }

    checkForModifiedImages();

    std::vector<LayerProcess*>::iterator it = _processes.begin();
    for ( ; _isOn && it!=_processes.end(); it++ )
	(*it)->checkForModifiedColorSequence();

    if ( _updateSetupStateSet )
    {
	if ( !_retileCompositeLayer )
	    _compositeLayerUpdate = true;
	buildShaders();
	setUpdateVar( _updateSetupStateSet, false );
    }

    _lock.readUnlock();
}


void LayeredTexture::checkForModifiedImages()
{
    std::vector<LayeredTextureData*>::iterator it = _dataLayers.begin();
    for ( ; it!=_dataLayers.end(); it++ )
    {
	(*it)->_imageModifiedFlag = false;
	if ( (*it)->_imageSource.get() )
	{
	    const int modifiedCount = (*it)->_imageSource->getModifiedCount();
	    if ( modifiedCount!=(*it)->_imageModifiedCount )
	    {
		setDataLayerImage( (*it)->_id, (*it)->_imageSource, false, (*it)->_nrPowerChannels );
		(*it)->_imageModifiedFlag = true;
		setUpdateVar( _updateSetupStateSet, true );
	    }
	}
    }

    for ( it=_dataLayers.begin(); it!=_dataLayers.end(); it++ )
    {
	const int udfIdx = getDataLayerIndex( (*it)->_undefLayerId );
	if ( udfIdx!=-1 && _dataLayers[udfIdx]->_imageModifiedFlag )
	    (*it)->clearTransparencyType();
    }
}


void LayeredTexture::buildShaders()
{
    _useShaders = _allowShaders && _texInfo->_shadingSupport;

    int nrProc = 0;
    int nrUsedLayers = 0;

    std::vector<int> orderedLayerIDs;
    bool stackIsOpaque = false;

    if ( _useShaders )
    {
	nrProc = getProcessInfo( orderedLayerIDs, nrUsedLayers, _useShaders, &stackIsOpaque );
    }

    if ( !_useShaders )
    {
	const bool create = !_retileCompositeLayer;
	setUpdateVar( _retileCompositeLayer, false );

	if ( create )
	    createCompositeTexture( !_texInfo->_isValid, true );

	if ( !_retileCompositeLayer )
	{
	    _setupStateSet->clear();
	    setRenderingHint( getDataLayerTransparencyType(_compositeLayerId)==Opaque );
	}

	return;
    }

    bool needColSeqTexture = false;
    int minUnit = _texInfo->_nrUnits;
    std::vector<int> activeUnits;

    std::vector<int>::iterator it = orderedLayerIDs.begin();
    for ( ; nrUsedLayers>0; it++, nrUsedLayers-- )
    {
	if ( *it )
	{
	    const int unit = getDataLayerTextureUnit( *it );
	    activeUnits.push_back( unit );
	    if ( unit<minUnit )
		minUnit = unit;
	}
	else
	    needColSeqTexture = true;
    }

    if ( minUnit<0 || (minUnit==0 && needColSeqTexture) )
    {
	setUpdateVar( _tilingInfo->_retilingNeeded, true );
	return;
    }

    _setupStateSet->clear();

    std::string code;
    getVertexShaderCode( code, activeUnits );
    osg::ref_ptr<osg::Shader> vertexShader = new osg::Shader( osg::Shader::VERTEX, code );

    if ( needColSeqTexture )
    {
	createColSeqTexture();
	activeUnits.push_back( 0 );
    }

    getFragmentShaderCode( code, activeUnits, nrProc, stackIsOpaque );
    osg::ref_ptr<osg::Shader> fragmentShader = new osg::Shader( osg::Shader::FRAGMENT, code );

    osg::ref_ptr<osg::Program> program = new osg::Program;

    program->addShader( vertexShader.get() );
    program->addShader( fragmentShader.get() );
    _setupStateSet->setAttributeAndModes( program.get() );

    char samplerName[20];
    for ( it=activeUnits.begin(); it!=activeUnits.end(); it++ )
    {
	snprintf( samplerName, 20, "texture%d", *it );
	_setupStateSet->addUniform( new osg::Uniform(samplerName, *it) );
    }

    setRenderingHint( stackIsOpaque );
}


void LayeredTexture::setRenderingHint( bool stackIsOpaque )
{
    if ( isDataLayerOK(_stackUndefLayerId) && _stackUndefColor[3]<1.0f )
	stackIsOpaque = false;

    if ( !stackIsOpaque ) 
    {
	osg::ref_ptr<osg::BlendFunc> blendFunc = new osg::BlendFunc;
	blendFunc->setFunction( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
	_setupStateSet->setAttributeAndModes( blendFunc );
	_setupStateSet->setRenderingHint( osg::StateSet::TRANSPARENT_BIN );
    }
    else
	_setupStateSet->setRenderingHint( osg::StateSet::OPAQUE_BIN );
}


int LayeredTexture::getProcessInfo( std::vector<int>& layerIDs, int& nrUsedLayers, bool& useShaders, bool* stackIsOpaque ) const
{
    layerIDs.empty();
    std::vector<int> skippedIDs;
    int nrProc = 0;
    nrUsedLayers = -1;

    if ( stackIsOpaque )
	*stackIsOpaque = false;

    if ( isDataLayerOK(_stackUndefLayerId) )
	layerIDs.push_back( _stackUndefLayerId );
    else if ( _stackUndefLayerId>0 )
	skippedIDs.push_back( _stackUndefLayerId );

    if ( _vertexOffsetLayerId != _stackUndefLayerId )
    {
	if ( isDataLayerOK(_vertexOffsetLayerId) )
	    layerIDs.push_back( _vertexOffsetLayerId );
	else if ( _vertexOffsetLayerId>0 )
	    skippedIDs.push_back( _vertexOffsetLayerId ); 
    }

    std::vector<LayerProcess*>::const_reverse_iterator it = _processes.rbegin();
    for ( ; _isOn && it!=_processes.rend(); it++ )
    {
	const TransparencyType transparency = (*it)->getTransparencyType();
	int nrPushed = 0;

	for ( int idx=-1; ; idx++ )
	{
	    bool skip = transparency==FullyTransparent;
	    int id = -1;

	    if ( idx>=0 )
	    {
		id = (*it)->getDataLayerID( idx/2 );
		if ( !(*it)->isOn(idx/2) || !isDataLayerOK(id) )
		    skip = true;

		if ( idx%2 )
		{
		    id = getDataLayerUndefLayerID(id);
		    if ( !isDataLayerOK(id) )
			skip = true;
		}
		else if ( idx>=8 && id<0 )
		    break;
	    }
	    else if ( (*it)->needsColorSequence() )
		id = 0;		// ColSeqTexture represented by ID=0

	    if ( id<0 )
		continue;

	    const std::vector<int>::iterator it1 = std::find(layerIDs.begin(),layerIDs.end(),id);
	    const std::vector<int>::iterator it2 = std::find(skippedIDs.begin(),skippedIDs.end(),id);

	    if ( nrUsedLayers<0 )
	    {
		if ( !skip )
		{
		    if ( it2!=skippedIDs.end() )
			skippedIDs.erase( it2 );
		}
		else if ( it1==layerIDs.end() && it2==skippedIDs.end() )
		    skippedIDs.push_back( id );
	    }

	    if ( it1==layerIDs.end() )
	    {
		if ( nrUsedLayers<0 )
		{
		    if ( !skip )
		    {
			layerIDs.push_back( id );
			nrPushed++;
		    }
		}
		else if ( it2==skippedIDs.end() )
		    layerIDs.push_back( id );
	    }
	}

	if ( nrUsedLayers<0 )
	{
	    const int sz = layerIDs.size();
	    if ( sz > _texInfo->_nrUnits )
	    {
		nrUsedLayers = sz-nrPushed;
		if ( !nrProc || !_maySkipEarlyProcesses )
		    useShaders = false;
	    }
	    else
	    {
		nrProc++;
		if ( transparency==Opaque )
		{
		    nrUsedLayers = sz;
		    if ( stackIsOpaque )
			*stackIsOpaque = true;
		}
	    }
	}
    }

    if ( nrUsedLayers<0 )
	nrUsedLayers = layerIDs.size();

    layerIDs.insert( layerIDs.begin()+nrUsedLayers,
		     skippedIDs.begin(), skippedIDs.end() );
    return nrProc;
}


void LayeredTexture::createColSeqTexture()
{
    const int nrProc = nrProcesses();
    const int nrScales = 10;	// stddev = 0, 0.5, 1, 2, 4, 8, 16, 32, 64, 128
    const int nrRows = powerOf2Ceil( nrProc*nrScales );

    osg::ref_ptr<osg::Image> colSeqImage = new osg::Image();
    colSeqImage->allocateImage( 256, nrRows, 1, GL_RGBA, GL_UNSIGNED_BYTE );
    const int rowSize = colSeqImage->getRowSizeInBytes();

    std::vector<LayerProcess*>::const_iterator it = _processes.begin();
    for ( int proc=0; _isOn && proc<nrProc; proc++, it++ )
    {
	const int row = proc*nrScales;
	(*it)->setColorSequenceTextureSampling( (row+0.5)/nrRows, 1.0/nrRows );

	const unsigned char* colSeqPtr = (*it)->getColorSequencePtr();
	if ( !colSeqPtr )
	    continue;

	unsigned char* ptr = colSeqImage->data( 0, row );
	memcpy( ptr, colSeqPtr, rowSize );
	ptr += rowSize;

	// Exclude undef color from smoothing when at far end of color sequence
	const int udfIdx = (*it)->getColorSequenceUndefIdx();
	const int start = udfIdx==0 ? 1 : 0;
	const int stop = udfIdx==255 ? 254 : 255;

	// Use uniform smoothing kernel to create color scale space recursively
	int stepout = 0;
	for ( int scale=1; scale<nrScales; scale++ )
	{
	    stepout = scale>2 ? stepout*2 : 1;

	    for ( int pivot=0; pivot<256; pivot++ )
	    {
		const int idx1 = pivot-stepout<start ? start : pivot-stepout;
		const int offset1 = 4*(idx1-pivot) - rowSize;

		const int idx2 = pivot+stepout>stop ? stop : pivot+stepout;
		const int offset2 = 4*(idx2-pivot) - rowSize;
		
		for ( int channel=0; channel<4; channel++ )
		{
		    int val = *(ptr-rowSize);

		    if ( pivot>=start && pivot<=stop )
		    {
			const int sum = *(ptr+offset1) + *(ptr+offset2);
			val = scale>1 ? sum : val+sum/2;

			// Alternate truncation to preserve signal strength
			val = scale%2 ? (val+1)/2 : val/2;
		    }

		    (*ptr++) = (unsigned char) val;
		}
	    }
	}
    }

    osg::ref_ptr<osg::Texture2D> texture = new osg::Texture2D( colSeqImage );
    texture->setFilter( osg::Texture::MIN_FILTER, osg::Texture::LINEAR );
    texture->setFilter( osg::Texture::MAG_FILTER, osg::Texture::LINEAR );
    _setupStateSet->setTextureAttributeAndModes( 0, texture.get() ); 

    const osg::Vec2 texSize( 256, nrRows );
    _setupStateSet->addUniform( new osg::Uniform("texsize0",texSize) );

    if ( _useNormalizedTexCoords )
    {
	const osg::Vec4 texCrdFactor( 1.0f, 1.0f, 1.0f, 1.0f );
	_setupStateSet->addUniform( new osg::Uniform("texcrdfactor0",texCrdFactor) );
	const osg::Vec4 texCrdBias( 0.0f, 0.0f, 0.0f, 0.0f );
	_setupStateSet->addUniform( new osg::Uniform("texcrdbias0",texCrdBias) );
    }
}


void LayeredTexture::assignTextureUnits()
{
    _useShaders = _allowShaders && _texInfo->_shadingSupport;

    std::vector<int> orderedLayerIDs;
    int nrUsedLayers = 0;

    if ( _useShaders )
	getProcessInfo( orderedLayerIDs, nrUsedLayers, _useShaders );

    std::vector<LayeredTextureData*>::iterator lit = _dataLayers.begin();
    for ( ; lit!=_dataLayers.end(); lit++ )
	(*lit)->_textureUnit = -1;

    if ( _useShaders )
    {
	int unit = 0;	// Reserved for ColSeqTexture if needed

	// nrUsedLayers = _texInfo->_nrUnits;
	// preloading shows bad performance in case of many tiles!

	std::vector<int>::iterator iit = orderedLayerIDs.begin();
	for ( ; iit!=orderedLayerIDs.end() && nrUsedLayers>0; iit++ )
	{
	    if ( (*iit)>0 )
		setDataLayerTextureUnit( *iit, (++unit)%_texInfo->_nrUnits );

	    nrUsedLayers--;
	}
    }
    else
	setDataLayerTextureUnit( _compositeLayerId, 0 );

    if ( _tilingInfo->_retilingNeeded || _updateSetupStateSet )
	setUpdateVar( _retileCompositeLayer, false );

    setUpdateVar( _updateSetupStateSet, true );
    updateSetupStateSetIfNeeded();
}


void LayeredTexture::getVertexShaderCode( std::string& code, const std::vector<int>& activeUnits ) const
{
    char line[100];
    code = "varying vec4 vertexpos;\n"
	   "\n";

    if ( _useNormalizedTexCoords )
    {
	std::vector<int>::const_iterator iit = activeUnits.begin();
	for ( ; iit!=activeUnits.end(); iit++ )
	{
	    snprintf( line, 100, "uniform vec4 texcrdfactor%d;\n", *iit );
	    code += line;
	    snprintf( line, 100, "uniform vec4 texcrdbias%d;\n", *iit );
	    code += line;
	}
	code += "\n";
    }

    const int udfId = getDataLayerUndefLayerID(_vertexOffsetLayerId);
    const int offsetUnit = getDataLayerTextureUnit(_vertexOffsetLayerId);

    const bool includeVertexOffset = isDataLayerOK(_vertexOffsetLayerId) &&
				     getTextureUnitNrDims(offsetUnit)!=3;
    if ( includeVertexOffset )
    {
	const int udfUnit = getDataLayerTextureUnit(udfId);

	std::vector<int>::const_iterator iit = activeUnits.begin();
	for ( ; iit!=activeUnits.end(); iit++ )
	{
	    if ( *iit!=offsetUnit && *iit!=udfUnit )
		continue;

	    snprintf( line, 100, "uniform sampler2D texture%d;\n", *iit );
	    code += line;
	    snprintf( line, 100, "uniform vec2 texsize%d;\n", *iit );
	    code += line;

	    snprintf( line, 100, "uniform float lod%d;\n", *iit );
	    code += line;
	}

	code += "uniform float offsetifudf;\n"
		"\n"
		"float offset( vec2 delta )\n"
		"{\n"
		"    vec2 texcrd;\n"
		"\n";

	if ( isDataLayerOK(udfId) )
	{
	    snprintf( line, 100, "    texcrd = gl_TexCoord[%d].st + delta/texsize%d;\n", udfUnit, udfUnit );
	    code += line;
	    const int udfChannel = getDataLayerUndefChannel( _vertexOffsetLayerId );
	    snprintf( line, 100, "    float udf = texture2DLod( texture%d, texcrd, lod%d )[%d];\n", udfUnit, udfUnit, udfChannel );
	    code += line;
	    if ( _invertUndefLayers )
		code += "    udf = 1.0 - udf;\n";

	    code += "\n"
		    "    if ( udf >= 1.0 )\n"
		    "        return 1e30;\n"
		    "\n";

	    if ( udfId!=_vertexOffsetLayerId )
	    {
		snprintf( line, 100, "    texcrd = gl_TexCoord[%d].st + delta/texsize%d;\n", offsetUnit, offsetUnit );
		code += line;
	    }
	    snprintf( line, 100, "    float offset = texture2DLod( texture%d, texcrd, lod%d )[%d];\n", offsetUnit, offsetUnit, _vertexOffsetChannel );
	    code += line;

	    const osg::Vec4f& udfColor = getDataLayerImageUndefColor(_vertexOffsetLayerId);

	    if ( udfColor[_vertexOffsetChannel]>=0.0 )
	    {
		code += "    if ( udf > 0.0 )\n";
		snprintf( line, 100, "        offset = (offset - udf*%.6f) / (1.0-udf);\n", udfColor[_vertexOffsetChannel] );
		code += line;
	    }
	}
	else
	{
	    snprintf( line, 100, "    texcrd = gl_TexCoord[%d].st + delta/texsize%d;\n", offsetUnit, offsetUnit );
	    code += line;
	    snprintf( line, 100, "    float offset = texture2DLod( texture%d, texcrd, lod%d )[%d];\n", offsetUnit, offsetUnit, _vertexOffsetChannel );
	    code += line;
	}

	code += "\n"
		"    return offset;\n"
		"}\n"
		"\n";
    }

    code += "void main(void)\n"
	    "{\n"
	    "    vec3 normal;\n"
	    "\n";

    std::vector<int>::const_iterator it = activeUnits.begin();
    for ( ; it!=activeUnits.end(); it++ )
    {
	snprintf( line, 100, "    gl_TexCoord[%d] = gl_TextureMatrix[%d] * gl_MultiTexCoord%d;\n", *it, *it, *it );
	code += line;

	if ( _useNormalizedTexCoords )
	{
	    snprintf( line, 100, "    gl_TexCoord[%d] = texcrdbias%d + texcrdfactor%d * gl_TexCoord[%d];\n", *it, *it, *it, *it );
	    code += line;
	}
    }

    if ( activeUnits.size() )
	code += "\n";

    if ( includeVertexOffset )
    {
	code += "    float pivot = offset( vec2(0.0,0.0) );\n";
	snprintf( line, 100, "    float delta = exp2( lod%d );\n", offsetUnit );
	code += line;
	code += "    vec4 connect4;\n"
		"    connect4[0] = offset( vec2(-delta,0.0) );\n"
		"    connect4[1] = offset( vec2( delta,0.0) );\n"
		"    connect4[2] = offset( vec2(0.0,-delta) );\n"
		"    connect4[3] = offset( vec2(0.0, delta) );\n"
		"\n";

	if ( isDataLayerOK(udfId) )
	{
	    code += "    if ( pivot > 1e29 )\n"
		    "    {\n"
		    "        int count = 0;\n"
		    "        float sum = 0.0;\n"
		    "        for ( int idx=0; idx<=3; idx++ )\n"
		    "        {\n"
		    "            if ( connect4[idx] < 1e29 )\n"
		    "            {\n"
		    "                count++;\n"
		    "                sum += connect4[idx];\n"
		    "            }\n"
		    "        }\n"
		    "        if ( count == 0 )\n"
		    "        {\n"
		    "            vec4 connect8;\n"
		    "            connect8[0] = offset( vec2(-delta, delta) );\n"
		    "            connect8[1] = offset( vec2( delta,-delta) );\n"
		    "            connect8[2] = offset( vec2(-delta,-delta) );\n"
		    "            connect8[3] = offset( vec2( delta, delta) );\n"
		    "            for ( int idx=0; idx<=3; idx++ )\n"
		    "            {\n"
		    "                if ( connect8[idx] < 1e29 )\n"
		    "                {\n"
		    "                    count++;\n"
		    "                    sum += connect8[idx];\n"
		    "                }\n"
		    "            }\n"
		    "        }\n"
		    "        if ( count > 0 )\n"
		    "            pivot = sum / float(count);\n"
		    "    }\n"
		    "\n"
		    "    for ( int idx=0; idx<=3; idx++ )\n"
		    "    {\n"
		    "        if ( connect4[idx] > 1e29 )\n"
		    "            connect4[idx] = pivot;\n"
		    "    }\n"
		    "\n";
	}

	osg::Vec3f normal = _vertexOffsetSpanVec0 ^ _vertexOffsetSpanVec1;
	normal.normalize();

	snprintf( line, 100, "    normal = vec3(%.6f,%.6f,%.6f);\n", normal[0], normal[1], normal[2] );
	code += line;

	code += "    pivot = ";
	if ( isDataLayerOK(udfId) )
	    code += "pivot>1e29 ? offsetifudf : ";
	if ( _vertexOffsetBias != 0.0f )
	{
	    snprintf( line, 100, "%e + ", _vertexOffsetBias );
	    code += line;
	}
	if ( _vertexOffsetFactor != 1.0f )
	{
	    snprintf( line, 100, "%e * ", _vertexOffsetFactor );
	    code += line;
	}
	code += "pivot;\n";

	code += "    vertexpos = vec4(normal*pivot, 0.0) + gl_Vertex;\n"
		"    gl_Position = gl_ModelViewProjectionMatrix * vertexpos;\n"
		"\n";

	const osg::Vec2f scale = getDataLayerScale( _vertexOffsetLayerId );
	const osg::Vec3f v0 = _vertexOffsetSpanVec0 * scale[0];
	const osg::Vec3f v1 = _vertexOffsetSpanVec1 * scale[1];

	snprintf( line, 100, "    vec3 v0 = vec3(%.6f,%.6f,%.6f);\n", v0[0], v0[1], v0[2] );
	code += line;
	snprintf( line, 100, "    vec3 v1 = vec3(%.6f,%.6f,%.6f);\n", v1[0], v1[1], v1[2] );
	code += line;
	snprintf( line, 100, "    v0 += normal * (connect4[1]-connect4[0]) * %.6f/delta;\n",  _vertexOffsetFactor*0.5 );
	code += line;
	snprintf( line, 100, "    v1 += normal * (connect4[3]-connect4[2]) * %.6f/delta;\n",  _vertexOffsetFactor*0.5 );
	code += line;

	code += "    normal = normalize( cross(v0,v1) );\n";
    }
    else
	code += "    vertexpos = gl_Vertex;\n"
		"    gl_Position = ftransform();\n"
		"    normal = gl_Normal;\n";

    code +=

"\n"
"    vec3 fragNormal = normalize(gl_NormalMatrix * normal);\n"
"\n"
"    vec4 diffuse = vec4(0.0,0.0,0.0,0.0);\n"
"    vec4 ambient = vec4(0.0,0.0,0.0,0.0);\n"
"    vec4 specular = vec4(0.0,0.0,0.0,0.0);\n"
"\n"
"    for ( int light=0; light<2; light++ )\n"
"    {\n"
"        vec3 lightDir = normalize( vec3(gl_LightSource[light].position) );\n"
"        float NdotL = abs( dot(fragNormal, lightDir) );\n"
"\n"
"        diffuse += gl_LightSource[light].diffuse * NdotL;\n"
"        ambient += gl_LightSource[light].ambient;\n"
"        float pf = 0.0;\n"
"        if (NdotL != 0.0)\n"
"        {\n"
"            float NdotH = abs( \n"
"	          dot(fragNormal, vec3(gl_LightSource[light].halfVector)) );\n"
"            if ( NdotH!=0.0 || gl_FrontMaterial.shininess!=0.0 )\n"
"                pf = pow( NdotH, gl_FrontMaterial.shininess );\n"
"        }\n"
"        specular += gl_LightSource[light].specular * pf;\n"
"    }\n"
"\n"
"    gl_FrontColor =\n"
"        gl_FrontLightModelProduct.sceneColor +\n"
"        ambient  * gl_FrontMaterial.ambient +\n"
"        diffuse  * gl_FrontMaterial.diffuse +\n"
"        specular * gl_FrontMaterial.specular;\n";

    code += "}\n";

    //std::cout << code << std::endl;
}


void LayeredTexture::getFragmentShaderCode( std::string& code, const std::vector<int>& activeUnits, int nrProc, bool stackIsOpaque ) const
{
    char line[100];
    code = "varying vec4 vertexpos;\n"
	   "\n";

    const bool useLOD = isDataLayerOK(_vertexOffsetLayerId)
	&& _stackUndefLayerId==getDataLayerUndefLayerID(_vertexOffsetLayerId)
	&& _stackUndefChannel==getDataLayerUndefChannel(_vertexOffsetLayerId);

    const int udfUnit = getDataLayerTextureUnit( _stackUndefLayerId );

    std::vector<int>::const_iterator iit = activeUnits.begin();
    for ( ; iit!=activeUnits.end(); iit++ )
    {
	if ( _useNormalizedTexCoords )
	{
	    snprintf( line, 100, "uniform vec4 texcrdfactor%d;\n", *iit );
	    code += line;
	    snprintf( line, 100, "uniform vec4 texcrdbias%d;\n", *iit );
	    code += line;
	}

	const int nrDims = getTextureUnitNrDims( *iit );
	snprintf( line, 100, "uniform sampler%dD texture%d;\n", nrDims, *iit );
	code += line;
	snprintf( line, 100, "uniform vec%d texsize%d;\n", nrDims, *iit );
	code += line;

	if ( nrDims==3 )
	{
	    snprintf( line, 100, "uniform mat4 vertextrans%d;\n", *iit );
	    code += line;
	}

	if ( useLOD && *iit==udfUnit )
	{
	    snprintf( line, 100, "uniform float lod%d;\n", *iit );
	    code += line;
	}
    }

    code += "\n";
    const bool stackUdf = isDataLayerOK(_stackUndefLayerId);
    code += stackUdf ? "void process( float stackudf )\n" :
		       "void process( void )\n";
    code += "{\n"
	    "    vec4 col, udfcol;\n"
	    "    vec3 texcrd;\n"
	    "    float a, b, udf, oldudf, orgcol3, mip, var, stddev, scale;\n"
	    "\n";

    int stage = 0;
    float minOpacity = 1.0f;

    std::vector<LayerProcess*>::const_reverse_iterator it = _processes.rbegin();
    for ( ; _isOn && it!=_processes.rend() && nrProc--; it++ )
    {
	if ( (*it)->getOpacity() < minOpacity )
	    minOpacity = (*it)->getOpacity();

	if ( (*it)->getTransparencyType()==FullyTransparent )
	    continue;

	if ( stage )
	{
	    code += "\n"
		    "    if ( gl_FragColor.a >= 1.0 )\n"
		    "       return;\n"
		    "\n";
	}

	(*it)->getShaderCode( code, stage++ );
    }

    if ( !stage )
    {
	snprintf( line, 100, "    gl_FragColor = vec4(1.0,1.0,1.0,%.6f);\n", minOpacity );
	code += line;
    }

    code += "}\n"
	    "\n"
	    "void main( void )\n"
	    "{\n"
	    "    if ( gl_FrontMaterial.diffuse.a <= 0.0 )\n"
	    "        discard;\n"
	    "\n"
	    "    gl_FragColor = vec4(1.0,1.0,1.0,1.0);\n"
	    "\n";

    if ( stackUdf )
    {
	code += "    vec3 ";
	addAssignTexCrdLine( code, udfUnit );
	const int udfDims = getTextureUnitNrDims( udfUnit );

	if ( useLOD )
	{
	    snprintf( line, 100, "    float udf = texture%dDLod( texture%d, texcrd.%.*s, lod%d )[%d];\n", udfDims, udfUnit, udfDims, "stp", udfUnit, _stackUndefChannel );
	}
	else
	    snprintf( line, 100, "    float udf = texture%dD( texture%d, texcrd.%.*s )[%d];\n", udfDims, udfUnit, udfDims, "stp", _stackUndefChannel );

	code += line;
	if ( _invertUndefLayers )
	    code += "        udf = 1.0 - udf;\n";

	if ( _stackUndefColor[3]<=0.0f )
	{
	    code += "\n"
		    "    if ( udf >= 1.0 )\n"
    		    "        discard;\n"
		    "\n"
		    "    process( udf );\n"
		    "\n"
		    "    if ( udf > 0.0 )\n"
		    "        gl_FragColor.a *= 1.0-udf;\n";
	}
	else
	{
	    code += "\n"
		    "    if ( udf < 1.0 )\n"
		    "        process( udf );\n"
		    "\n";

	    snprintf( line, 100, "    vec4 udfcol = vec4(%.6f,%.6f,%.6f,%.6f);\n", _stackUndefColor[0], _stackUndefColor[1], _stackUndefColor[2], _stackUndefColor[3] );
	    code += line;

	    code += "\n"
		    "    if ( udf >= 1.0 )\n"
		    "        gl_FragColor = udfcol;\n"
		    "    else if ( udf > 0.0 )\n";

	    if ( _stackUndefColor[3]>=1.0f && stackIsOpaque )
		code += "        gl_FragColor = mix( gl_FragColor, udfcol, udf );\n";
	    else
		code += "    {\n"
			"        if ( gl_FragColor.a > 0.0 )\n"
			"        {\n"
			"            vec4 col = gl_FragColor;\n"
			"            gl_FragColor.a = mix( col.a, udfcol.a, udf );\n"
			"            col.rgb = mix( col.a*col.rgb, udfcol.a*udfcol.rgb, udf );\n"
			"            gl_FragColor.rgb = col.rgb / gl_FragColor.a;\n"
			"        }\n"
			"        else\n"
			"            gl_FragColor = vec4( udfcol.rgb, udf*udfcol.a );\n"
			"    }\n";
	}
    }
    else
	 code += "    process();\n";

    code += "\n"
	    "    gl_FragColor.a *= gl_FrontMaterial.diffuse.a;\n"
	    "    gl_FragColor.rgb *= gl_Color.rgb;\n"
	    "}\n";

    //std::cout << code << std::endl;
}


void LayeredTexture::allowShaders( bool yn, bool maySkipEarlyProcs )
{
    if ( _allowShaders!=yn || _maySkipEarlyProcesses!=maySkipEarlyProcs )
    {
	_allowShaders = yn;
	_maySkipEarlyProcesses = maySkipEarlyProcs;
	setUpdateVar( _tilingInfo->_retilingNeeded, true );
    }
}


void LayeredTexture::enableMipmapping( bool yn )
{
    if ( _enableMipmapping!=yn )
    {
	_enableMipmapping = yn;
	setUpdateVar( _tilingInfo->_retilingNeeded, true );
    }
}


bool LayeredTexture::isMipmappingEnabled() const
{ return _enableMipmapping; }


void LayeredTexture::setTextureSizePolicy( TextureSizePolicy policy )
{
    _textureSizePolicy = policy;
    setUpdateVar( _tilingInfo->_retilingNeeded, true );
}


LayeredTexture::TextureSizePolicy LayeredTexture::getTextureSizePolicy() const
{ return _textureSizePolicy; }


LayeredTexture::TextureSizePolicy LayeredTexture::usedTextureSizePolicy() const
{
    if ( _textureSizePolicy==AnySize && !_texInfo->_nonPowerOf2Support )
	return PowerOf2;

    return _textureSizePolicy;
}


void LayeredTexture::setMaxTextureCopySize( unsigned int width_x_height )
{
    _maxTextureCopySize = width_x_height;

    std::vector<LayeredTextureData*>::iterator it = _dataLayers.begin();
    for ( ; it!=_dataLayers.end(); it++ )
    {
	(*it)->_imageModifiedFlag = false;
	if ( (*it)->_image.get()!=(*it)->_imageSource.get() )
	    (*it)->_imageModifiedCount = -1;
    }
}


void LayeredTexture::invertUndefLayers( bool yn )
{ _invertUndefLayers = yn; }


bool LayeredTexture::areUndefLayersInverted() const
{ return _invertUndefLayers; }


//============================================================================


class CompositeTextureTask : public osg::Referenced, public OpenThreads::Thread
{
public:
			CompositeTextureTask(const LayeredTexture& lt,
				    osg::Image& image,osg::Vec4f& borderCol,
				    const std::vector<LayerProcess*>& procs,
				    float minOpacity,bool dummyTexture,
				    pixel_int startNr,pixel_int stopNr,
				    OpenThreads::BlockCount& ready)
			    : _lt( lt )
			    , _image( image )
			    , _borderColor( borderCol )
			    , _processList( procs )
			    , _minOpacity( minOpacity )
			    , _dummyTexture( dummyTexture )
			    , _start( startNr>=0 ? startNr : 0 )
			    , _stop( stopNr<=image.s()*image.t() ? stopNr : image.s()*image.t() )
			    , _readyCount( ready )
			{}

			~CompositeTextureTask()
			{
			    while( isRunning() )
				OpenThreads::Thread::YieldCurrentThread();
			}

    void		run();

protected:

    const LayeredTexture&		_lt;
    bool				_dummyTexture;
    osg::Image&				_image;
    osg::Vec4f&				_borderColor;
    const std::vector<LayerProcess*>&	_processList;
    float				_minOpacity;
    pixel_int				_start;
    pixel_int				_stop;
    OpenThreads::BlockCount&		_readyCount;
};


void CompositeTextureTask::run()
{
    const int idx = _lt.getDataLayerIndex( _lt._compositeLayerId );
    const osg::Vec2f& origin = _lt._dataLayers[idx]->_origin;
    const osg::Vec2f& scale = _lt._dataLayers[idx]->_scale;

    const LayeredTextureData* udfLayer = 0;
    const int udfIdx = _lt.getDataLayerIndex( _lt._stackUndefLayerId );
    if ( udfIdx>=0 )
	udfLayer = _lt._dataLayers[udfIdx];

    const osg::Vec4f& udfColor = _lt._stackUndefColor;
    const int udfChannel = _lt._stackUndefChannel;
    float udf = 0.0f;

    std::vector<LayerProcess*>::const_reverse_iterator it;
    unsigned char* imagePtr = _image.data() + _start*4;
    const int width = _image.s();
    const pixel_int nrImagePixels = width * _image.t();

    for ( pixel_int pixelNr=_start; pixelNr<=_stop; pixelNr++ ) 
    {
	osg::Vec2f globalCoord( origin.x()+scale.x()*(pixelNr%width+0.5),
				origin.y()+scale.y()*(pixelNr/width+0.5) );

	osg::Vec4f fragColor( -1.0f, -1.0f, -1.0f, -1.0f );

	if ( udfLayer && !_dummyTexture )
	{
	    udf = udfLayer->getTextureVec(globalCoord)[udfChannel];
	    if ( _lt._invertUndefLayers )
		udf = 1.0-udf;
	}

	if ( udf<1.0 )
	{
	    for ( it=_processList.rbegin(); it!=_processList.rend(); it++ )
	    {
		(*it)->doProcess( fragColor, udf, globalCoord );

		if ( fragColor[3]>=1.0f )
		    break;
	    }

	    if ( _dummyTexture )
		fragColor = osg::Vec4f( 0.0f, 0.0f, 0.0f, 0.0f );
	    else if ( fragColor[0]==-1.0f )
		fragColor = osg::Vec4f( 1.0f, 1.0f, 1.0f, _minOpacity );
	}

	if ( udf>=1.0f )
	    fragColor = udfColor;
	else if ( udf>0.0 )
	{
	    if ( udfColor[3]<=0.0f )
		fragColor[3] *= 1.0f-udf;
	    else if ( udfColor[3]>=1.0f && fragColor[3]>=1.0f )
		fragColor = fragColor*(1.0f-udf) + udfColor*udf;
	    else if ( fragColor[3]>0.0f )
	    {
		const float a = fragColor[3]*(1.0f-udf);
		const float b = udfColor[3]*udf;
		fragColor = (fragColor*a + udfColor*b) / (a+b);
		fragColor[3] = a+b;
	    }
	    else
	    {
		fragColor = udfColor;
		fragColor[3] *= udf;
	    }
	}

	if ( fragColor[3]<0.5f/255.0f )
	    fragColor = osg::Vec4f( 0.0f, 0.0f, 0.0f, 0.0f );

	if ( pixelNr<nrImagePixels )
	{
	    fragColor *= 255.0f;
	    for ( int tc=0; tc<4; tc++ )
	    {
		int val = (int) floor( fragColor[tc]+0.5 );
		val = val<=0 ? 0 : (val>=255 ? 255 : val);

		*imagePtr = (unsigned char) val;
		imagePtr++;
	    }
	}
	else
	    _borderColor = fragColor;
    }

    _readyCount.completed();
}


void LayeredTexture::createCompositeTexture( bool dummyTexture, bool triggerProgress )
{
    if ( !_compositeLayerUpdate )
	return;

    if ( triggerProgress )
	triggerStartWorkInProgress();

    _compositeLayerUpdate = false;
    updateTilingInfoIfNeeded();
    const osgGeo::TilingInfo& ti = *_tilingInfo;

    int width  = (int) ceil( ti._envelopeSize.x()/ti._smallestScale.x() );
    int height = (int) ceil( ti._envelopeSize.y()/ti._smallestScale.y() );
    width *= _compositeSubsampleSteps;
    height *= _compositeSubsampleSteps;

    if ( dummyTexture || width<1 )
	width = 1;
    if ( dummyTexture || height<1 )
	height = 1;

    const int idx = getDataLayerIndex( _compositeLayerId );

    _dataLayers[idx]->_origin = ti._envelopeOrigin;
    _dataLayers[idx]->_scale = osg::Vec2f( ti._envelopeSize.x()/float(width),
					   ti._envelopeSize.y()/float(height) );

    osg::Image* image = const_cast<osg::Image*>(_dataLayers[idx]->_image.get());

    if ( !image || width!=image->s() || height!=image->t() )
    {
	image = new osg::Image;
	image->allocateImage( width, height, 1, GL_RGBA, GL_UNSIGNED_BYTE );
    }

    std::vector<LayerProcess*> processList;
    float minOpacity = 1.0f;

    std::vector<LayerProcess*>::const_iterator it = _processes.begin();
    for ( ; _isOn && it!=_processes.end(); it++ )
    {
	if ( (*it)->getTransparencyType()!=FullyTransparent )
	    processList.push_back( *it );

	if ( (*it)->getOpacity() < minOpacity )
	    minOpacity = (*it)->getOpacity();
    }

    pixel_int nrPixels = height*width;

    /* Cannot cover mixed use of uniform and extended-edge-pixel borders
       without shaders (trick with extra one-pixel wide border is screwed
       by mipmapping) */
    osg::Vec4f borderColor = getDataLayerBorderColor( _compositeLayerId );
    if ( borderColor[0]>=0.0f )	
	nrPixels++; // One extra pixel to compute uniform composite borderColor		
    int nrTasks = OpenThreads::GetNumberOfProcessors();

    if ( nrTasks<1 )
	 nrTasks=1;
    if ( nrTasks>nrPixels )
	nrTasks = nrPixels;

    std::vector<osg::ref_ptr<CompositeTextureTask> > tasks;
    OpenThreads::BlockCount readyCount( nrTasks );
    readyCount.reset();

    pixel_int remainder = nrPixels%nrTasks;
    pixel_int start = 0;

    while ( start<nrPixels )
    {
	pixel_int stop = start + nrPixels/nrTasks;
	if ( remainder )
	    remainder--;
	else
	    stop--;

	osg::ref_ptr<CompositeTextureTask> task = new CompositeTextureTask( *this, *image, borderColor, processList, minOpacity, dummyTexture, start, stop, readyCount );

	tasks.push_back( task.get() );
	task->start();

	start = stop+1;
    }

    readyCount.block();

    const bool retilingNeededAlready = _tilingInfo->_retilingNeeded;

    setDataLayerImage( _compositeLayerId, image );

    setUpdateVar( _retileCompositeLayer, 
		  _tilingInfo->_needsUpdate ||
		  getDataLayerTextureUnit(_compositeLayerId)!=0 ||
		  borderColor!=getDataLayerBorderColor(_compositeLayerId) );

    if ( _reInitTiling || _useShaders )
	setUpdateVar( _retileCompositeLayer, false );

    setDataLayerBorderColor( _compositeLayerId, borderColor );

    setUpdateVar( _tilingInfo->_needsUpdate, false );
    setUpdateVar( _tilingInfo->_retilingNeeded, retilingNeededAlready );

    if ( triggerProgress )
	triggerStopWorkInProgress();
}


const osg::Image* LayeredTexture::getCompositeTextureImage()
{
    createCompositeTexture();
    return getDataLayerImage( _compositeLayerId );
}


void LayeredTexture::setCompositeSubsampleSteps( int steps )
{ 
    if ( steps>0 && steps!=_compositeSubsampleSteps )
    {
	_compositeSubsampleSteps = steps;
	if ( _useShaders )
	    _compositeLayerUpdate = true;
	else
	    setUpdateVar( _tilingInfo->_retilingNeeded, true );
    }
}


int LayeredTexture::getCompositeSubsampleSteps() const
{
    return _compositeSubsampleSteps;
}


} //namespace

#include <osgDB/ObjectWrapper>
#include <osgDB/Registry>
#include <osgDB/Serializer>

REGISTER_EMPTY_OBJECT_WRAPPER( LayeredTexture_Wrapper,
                        new osgGeo::LayeredTexture,
                        osgGeo::LayeredTexture,
                        "osg::Object osgGeo::LayeredTexture");

