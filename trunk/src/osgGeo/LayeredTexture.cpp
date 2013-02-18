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
#include <osg/Geometry>
#include <osg/State>
#include <osg/Texture2D>
#include <osg/Version>
#include <osgUtil/CullVisitor>
#include <osgGeo/Vec2i>

#include <string.h>
#include <iostream>
#include <cstdio>


#if OSG_MIN_VERSION_REQUIRED(3,1,0)
     #define USE_IMAGE_STRIDE
#endif

namespace osgGeo
{


int LayeredTexture::powerOf2Ceil( unsigned short nr )
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


static TransparencyType getTransparencyTypeBytewise( const unsigned char* start, const unsigned char* stop, int step )
{
    bool foundOpaquePixel = false;
    bool foundTransparentPixel = false;

    for ( const unsigned char* ptr=start; ptr<=stop; ptr+=step )
    {
	if ( *ptr==0 )
	    foundTransparentPixel = true;
	else if ( *ptr==255 )
	    foundOpaquePixel = true;
	else
	    return HasTransparencies;
    }

    if ( foundTransparentPixel )
	return foundOpaquePixel ? OnlyFullTransparencies : FullyTransparent;

    return Opaque;
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

    GLenum dataType = image->getDataType();                               
    bool isByte = dataType==GL_UNSIGNED_BYTE || dataType==GL_BYTE;

    if ( isByte && imageChannel>=0 )
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


static TransparencyType addOpacity( TransparencyType tt, float opacity )
{
    if ( tt==TransparencyUnknown )
	return tt;

    if ( opacity<=0.0f )
	return tt==Opaque ? OnlyFullTransparencies : tt;

    if ( opacity>=1.0f )
	return tt==FullyTransparent ? OnlyFullTransparencies : tt;

    return HasTransparencies;
}


static TransparencyType multiplyOpacity( TransparencyType tt, float opacity )
{
    if ( tt==TransparencyUnknown )
	return tt;

    if ( opacity<=0.0f )
	return FullyTransparent;

    if ( opacity>=1.0f )
	return tt;

    return tt==FullyTransparent ? tt : HasTransparencies;
}

//============================================================================


ColorSequence::ColorSequence( unsigned char* array )
    : _arr( array )
    , _dirtyCount( 0 )
    , _transparencyType( TransparencyUnknown )
{
    if ( array )
	setRGBAValues( array );
}


ColorSequence::~ColorSequence()
{}


void ColorSequence::setRGBAValues( unsigned char* array )
{
    _arr = array;
    touch();
}


void ColorSequence::touch()
{
    _dirtyCount++;
    _transparencyType = TransparencyUnknown;
}


TransparencyType ColorSequence::getTransparencyType() const
{
    if ( _transparencyType==TransparencyUnknown )
    {
	if ( !_arr )
	    _transparencyType = FullyTransparent;
	else 
	    _transparencyType = getTransparencyTypeBytewise( _arr+3, _arr+1023, 4 );
    }

    return _transparencyType;
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
			    , _textureUnit( -1 )
			    , _filterType(Linear)
			    , _borderColor( 1.0f, 1.0f, 1.0f, 1.0f )
			    , _borderColorSource( 1.0f, 1.0f, 1.0f, 1.0f )
			    , _undefLayerId( -1 )
			    , _undefChannel( 0 )
			    , _undefColor( -1.0f, -1.0f, -1.0f, -1.0f )
			    , _undefColorSource( -1.0f, -1.0f, -1.0f, -1.0f )
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

    const int					_id;
    osg::Vec2f					_origin;
    osg::Vec2f					_scale;
    osg::ref_ptr<const osg::Image>		_image;
    osg::ref_ptr<const osg::Image>		_imageSource;
    osgGeo::Vec2i				_imageSourceSize;
    osg::Vec2f					_imageScale;
    int						_imageModifiedCount;
    bool					_imageModifiedFlag;
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
    std::vector<osg::Image*>			_tileImages;
};


LayeredTextureData::~LayeredTextureData()
{
    cleanUp();
}


LayeredTextureData* LayeredTextureData::clone() const
{
    LayeredTextureData* res = new LayeredTextureData( _id );
    res->_origin = _origin;
    res->_scale = _scale; 
    res->_textureUnit = _textureUnit;
    res->_filterType = _filterType;
    res->_imageModifiedCount = _imageModifiedCount;
    res->_imageScale = _imageScale; 
    res->_imageSource = _imageSource.get();

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


#define GET_COLOR( color, image, s, t ) \
\
    osg::Vec4f color = _borderColor; \
    if ( s>=0 && s<image->s() && t>=0 && t<image->t() ) \
	color = image->getColor( s, t ); \
    else if ( _borderColor[0]>=0.0f ) \
	color = _borderColor; \
    else \
    { \
	const int sClamp = s<=0 ? 0 : ( s>=image->s() ? image->s()-1 : s ); \
	const int tClamp = t<=0 ? 0 : ( t>=image->t() ? image->t()-1 : t ); \
	color = image->getColor( sClamp, tClamp ); \
    }

osg::Vec4f LayeredTextureData::getTextureVec( const osg::Vec2f& globalCoord ) const
{
    if ( !_image.get() || !_image->s() || !_image->t() )
	return _borderColor;

    osg::Vec2f local = getLayerCoord( globalCoord );
    if ( _filterType!=Nearest )
	local -= osg::Vec2f( 0.5, 0.5 );

    int s = (int) floor( local.x() );
    int t = (int) floor( local.y() );

    GET_COLOR( col00, _image, s, t );

    if ( _filterType==Nearest )
	return col00;

    const float sFrac = local.x()-s;
    const float tFrac = local.y()-t;

    if ( !tFrac )
    {
	if ( !sFrac )
	    return col00;

	s++;
	GET_COLOR( col10, _image, s, t );
	return col00*(1.0f-sFrac) + col10*sFrac;
    }

    t++;
    GET_COLOR( col01, _image, s, t );
    col00 = col00*(1.0f-tFrac) + col01*tFrac;

    if ( !sFrac )
	return col00;

    s++;
    GET_COLOR( col11, _image, s, t );
    t--;
    GET_COLOR( col10, _image, s, t );

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
				{}

    bool			_isValid;
    int				_contextId;
    int				_nrUnits;
    int				_maxSize;
    bool			_nonPowerOf2Support;
};


//============================================================================

#define EPS			1e-5
#define MIN_ID			1	// ID=0 used to represent ColSeqTexture

LayeredTexture::LayeredTexture()
    : _freeId( MIN_ID )
    , _updateSetupStateSet( false )
    , _maxTextureCopySize( 32*32 )
    , _tilingInfo( new TilingInfo )
    , _texInfo( new TextureInfo )
    , _stackUndefLayerId( -1 )
    , _stackUndefChannel( 0 )
    , _stackUndefColor( 0.0f, 0.0f, 0.0f, 0.0f )
    , _invertUndefLayers( false )
    , _useShaders( true )
    , _compositeLayerUpdate( true )
{
    _compositeLayerId = addDataLayer();
}


LayeredTexture::LayeredTexture( const LayeredTexture& lt,
				const osg::CopyOp& co )
    : osg::Object( lt, co )
    , _freeId( lt._freeId )
    , _updateSetupStateSet( false )
    , _setupStateSet( 0 )
    , _maxTextureCopySize( lt._maxTextureCopySize )
    , _tilingInfo( new TilingInfo(*lt._tilingInfo) )
    , _texInfo( new TextureInfo(*lt._texInfo) )
    , _stackUndefLayerId( lt._stackUndefLayerId )
    , _stackUndefChannel( lt._stackUndefChannel )
    , _stackUndefColor( lt._stackUndefColor )
    , _useShaders( lt._useShaders )
    , _compositeLayerId( lt._compositeLayerId )
    , _compositeLayerUpdate( lt._compositeLayerUpdate )
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


int LayeredTexture::addDataLayer()
{
    _lock.writeLock();
    osg::ref_ptr<LayeredTextureData> ltd = new LayeredTextureData( _freeId++ );
    if ( ltd )
    {
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
		 _tilingInfo->_retilingNeeded = true;
	 }
    }
}


void LayeredTexture::removeDataLayer( int id )
{
    if ( id==_compositeLayerId )
	return; 

    _lock.writeLock();
    const int idx = getDataLayerIndex( id );
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
	}

	osg::ref_ptr<LayeredTextureData> ltd = _dataLayers[idx];
	_dataLayers.erase( _dataLayers.begin()+idx );
	_tilingInfo->_needsUpdate = true;
	ltd->unref();
    }

    _lock.writeUnlock();
}


int LayeredTexture::getDataLayerID( int idx ) const
{
    return idx>=0 && idx< (int) _dataLayers.size() 
	? _dataLayers[idx]->_id
	: -1;
}


int LayeredTexture::getDataLayerIndex( int id ) const
{
    // Optimization since layers are ID-sorted
    unsigned int maxIdx = id - MIN_ID;
    if ( maxIdx >= _dataLayers.size() )
	maxIdx = _dataLayers.size()-1;

    for ( int idx=maxIdx; idx>=0; idx-- )
    {
	if ( _dataLayers[idx]->_id==id )
	    return idx;
    }

    return -1;
}


void LayeredTexture::setDataLayerOrigin( int id, const osg::Vec2f& origin )
{
    const int idx = getDataLayerIndex( id );
    if ( idx!=-1 )
    {
	_dataLayers[idx]->_origin = origin; 
	_tilingInfo->_needsUpdate = true;
    }
}


void LayeredTexture::setDataLayerScale( int id, const osg::Vec2f& scale )
{
    const int idx = getDataLayerIndex( id );
    if ( idx!=-1 && scale.x()>=0.0f && scale.y()>0.0f )
    {
	_dataLayers[idx]->_scale = scale; 
	_tilingInfo->_needsUpdate = true;
    }
}


void LayeredTexture::setDataLayerImage( int id, const osg::Image* image )
{
    const int idx = getDataLayerIndex( id );
    if ( idx==-1 )
	return;

    LayeredTextureData& layer = *_dataLayers[idx];

    if ( image )
    {
	if ( !image->s() || !image->t() || !image->getPixelFormat() )
	{
	    std::cerr << "Data layer image cannot be set before allocation" << std::endl;
	    return;
	}

	osgGeo::Vec2i newImageSize( image->s(), image->t() );

#ifdef USE_IMAGE_STRIDE
	const bool retile = layer._imageSource.get()!=image || layer._imageSourceSize!=newImageSize || !layer._tileImages.size();
#endif

	const int s = powerOf2Ceil( image->s() );
	const int t = powerOf2Ceil( image->t() );

	bool scaleImage = s>image->s() || t>image->t();
	scaleImage = scaleImage && s*t<=(int) _maxTextureCopySize;

	if ( scaleImage && id!=_compositeLayerId )
	{
	    osg::Image* imageCopy = new osg::Image( *image );
	    imageCopy->scaleImage( s, t, image->r() );
#ifdef USE_IMAGE_STRIDE
	    if ( !retile )
		const_cast<osg::Image*>(layer._image.get())->copySubImage( 0, 0, 0, imageCopy ); 
	    else
#endif
		layer._image = imageCopy;

	    layer._imageScale.x() = float(image->s()) / float(s);
	    layer._imageScale.y() = float(image->t()) / float(t);
	}
	else
	{
	    layer._image = image;
	    layer._imageScale = osg::Vec2f( 1.0f, 1.0f );
	}

	layer._imageSource = image;
	layer._imageSourceSize = newImageSize;
	layer._imageModifiedCount = image->getModifiedCount();
	layer.clearTransparencyType();

	if ( id==_compositeLayerId && _useShaders )
	    return;

#ifdef USE_IMAGE_STRIDE
	if ( !retile )
	{
	    std::vector<osg::Image*>::iterator it = layer._tileImages.begin();
	    for ( ; it!=layer._tileImages.end(); it++ )
		(*it)->dirty();
	}
	else
#endif
	{
	    layer.adaptColors();
	    _tilingInfo->_needsUpdate = true;
	}
    }
    else
    {
	layer._image = 0; 
	layer._imageSource = 0;
	layer.adaptColors();
	_tilingInfo->_needsUpdate = true;
    }
}


void LayeredTexture::setDataLayerUndefLayerID( int id, int undefId )
{
    const int idx = getDataLayerIndex( id );
    if ( idx!=-1 )
    {
	raiseUndefChannelRefCount( false, idx );

	_updateSetupStateSet = true;
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

	_updateSetupStateSet = true;

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
	    _updateSetupStateSet = true;
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
	    _tilingInfo->_retilingNeeded = true;
    }
}


void LayeredTexture::setDataLayerFilterType( int id, FilterType filterType )
{
    const int idx = getDataLayerIndex( id );
    if ( idx!=-1 )
    {
	_dataLayers[idx]->_filterType = filterType;
	if ( _dataLayers[idx]->_textureUnit>=0 )
	    _tilingInfo->_retilingNeeded = true;
    }
}


void LayeredTexture::setDataLayerTextureUnit( int id, int unit )
{
    const int idx = getDataLayerIndex( id );
    if ( idx!=-1 )
	_dataLayers[idx]->_textureUnit = unit;
}


void LayeredTexture::setStackUndefLayerID( int id )
{
    raiseUndefChannelRefCount( false );
    _updateSetupStateSet = true;
    _stackUndefLayerId = id;
    raiseUndefChannelRefCount( true );
}


void LayeredTexture::setStackUndefChannel( int channel )
{
    if ( channel>=0 && channel<4 )
    {
	raiseUndefChannelRefCount( false );
	_updateSetupStateSet = true;
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
    _updateSetupStateSet = true;
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
	_updateSetupStateSet = true;
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
	    _updateSetupStateSet = true; \
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
    updateTextureInfoIfNeeded();
}


int LayeredTexture::getGraphicsContextID() const
{ return _texInfo->_contextId; }


void LayeredTexture::updateTextureInfoIfNeeded() const
{
    if ( _texInfo->_isValid )
	return;

    const int maxContextID = (int) osg::GraphicsContext::getMaxContextID();
    for( int contextID=0; contextID<=maxContextID; contextID++ )
    {
	if ( _texInfo->_contextId>=0 && _texInfo->_contextId!=contextID )
	    continue;

	const osg::Texture::Extensions* texExt = osg::Texture::getExtensions( contextID, _texInfo->_contextId>=0 );

	if ( !texExt )
	    continue;

	if ( !_texInfo->_isValid || _texInfo->_nrUnits>texExt->numTextureUnits() )
	    _texInfo->_nrUnits = texExt->numTextureUnits();

	if ( !_texInfo->_isValid || _texInfo->_maxSize>texExt->maxTextureSize() )
	    _texInfo->_maxSize = texExt->maxTextureSize();

	if ( !_texInfo->_isValid || _texInfo->_nonPowerOf2Support )
	    _texInfo->_nonPowerOf2Support = texExt->isNonPowerOfTwoTextureSupported( osg::Texture::LINEAR_MIPMAP_LINEAR );

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
    osg::Vec2f minBound = (*it)->_origin;
    osg::Vec2f maxBound = minBound;
    osg::Vec2f minScale( 0.0f, 0.0f );
    osg::Vec2f minNoPow2Size( 0.0f, 0.0f );

    for ( ; it!=_dataLayers.end(); it++ )
    {
	if ( !(*it)->_image.get() || (*it)->_id==_compositeLayerId )
	    continue;

	const osg::Vec2f scale( (*it)->_scale.x() * (*it)->_imageScale.x(),
				(*it)->_scale.y() * (*it)->_imageScale.y() );

	const osg::Vec2f layerSize( (*it)->_image->s() * scale.x(),
				    (*it)->_image->t() * scale.y() );

	const osg::Vec2f bound = layerSize + (*it)->_origin;

	if ( bound.x() > maxBound.x() )
	    maxBound.x() = bound.x();
	if ( bound.y() > maxBound.y() )
	    maxBound.y() = bound.y();
	if ( (*it)->_origin.x() < minBound.x() )
	    minBound.x() = (*it)->_origin.x();
	if ( (*it)->_origin.y() < minBound.y() )
	    minBound.y() = (*it)->_origin.y();
	if ( minScale.x()<=0.0f || scale.x()<minScale.x() )
	    minScale.x() = scale.x();
	if ( minScale.y()<=0.0f || scale.y()<minScale.y() )
	    minScale.y() = scale.y();

	if ( ( minNoPow2Size.x()<=0.0f || layerSize.x()<minNoPow2Size.x()) &&
	     (*it)->_image->s() != powerOf2Ceil((*it)->_image->s()) )
	    minNoPow2Size.x() = layerSize.x();

	if ( ( minNoPow2Size.y()<=0.0f || layerSize.y()<minNoPow2Size.y()) &&
	     (*it)->_image->t() != powerOf2Ceil((*it)->_image->t()) )
	    minNoPow2Size.y() = layerSize.y();
    }

    if ( minScale.x()<=0.0f ||  minScale.y()<=0.0f )
	return;

    _tilingInfo->_envelopeSize = maxBound - minBound;
    _tilingInfo->_envelopeOrigin = minBound;
    _tilingInfo->_smallestScale = minScale;
    _tilingInfo->_maxTileSize = osg::Vec2f( minNoPow2Size.x() / minScale.x(),
					    minNoPow2Size.y() / minScale.y() );
}


bool LayeredTexture::needsRetiling() const
{
    updateTilingInfoIfNeeded();
    updateTextureInfoIfNeeded();

    return _tilingInfo->_retilingNeeded;
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
    return _tilingInfo->_envelopeOrigin + _tilingInfo->_envelopeSize*0.5;
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


void LayeredTexture::reInitTiling()
{
    std::vector<LayeredTextureData*>::iterator lit = _dataLayers.begin();
    for ( ; lit!=_dataLayers.end(); lit++ )
	(*lit)->cleanUp();

    updateTilingInfoIfNeeded();
    updateTextureInfoIfNeeded();
    assignTextureUnits();

    _tilingInfo->_retilingNeeded = !_texInfo->_isValid;
}


bool LayeredTexture::planTiling( unsigned short brickSize, std::vector<float>& xTickMarks, std::vector<float>& yTickMarks, bool strict ) const
{
    const osgGeo::Vec2i requestedSize( brickSize, brickSize );
    osgGeo::Vec2i actualSize = requestedSize;

    if ( !strict && _textureSizePolicy!=AnySize )
    {
	const int overlap = _textureSizePolicy==PowerOf2 ? 2 : 0;
	/* One to avoid seam (lower LOD needs more),
	   one because layers may mutually disalign. */

	const int powerOf2Size = powerOf2Ceil( brickSize+overlap+1 ) / 2;
	actualSize = osgGeo::Vec2i( powerOf2Size, powerOf2Size );

	const osg::Vec2f& maxTileSize = _tilingInfo->_maxTileSize;
	while ( actualSize.x()>maxTileSize.x() && maxTileSize.x()>0.0f )
	    actualSize.x() /= 2;

	while ( actualSize.y()>maxTileSize.y() && maxTileSize.y()>0.0f )
	    actualSize.y() /= 2;

	actualSize -= osgGeo::Vec2i( overlap, overlap );
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


static void boundedCopy( unsigned char* dest, const unsigned char* src, int len, const unsigned char* lowPtr, const unsigned char* highPtr )
{
    if ( src>=highPtr || src+len<=lowPtr )
    {
	std::cerr << "Unsafe memcpy" << std::endl;
	return;
    }

    if ( src < lowPtr )
    {
	std::cerr << "Unsafe memcpy" << std::endl;
	len -= lowPtr - src;
	src = lowPtr;
    }
    if ( src+len > highPtr )
    {
	std::cerr << "Unsafe memcpy" << std::endl;
	len = highPtr - src;
    }

    memcpy( dest, src, len );
}


static void copyImageWithStride( const unsigned char* srcImage, unsigned char* tileImage, int nrRows, int rowSize, int offset, int stride, int pixelSize, const unsigned char* srcEnd )
{
    int rowLen = rowSize*pixelSize;
    const unsigned char* srcPtr = srcImage+offset;

    if ( !stride )
    {
	boundedCopy( tileImage, srcPtr, rowLen*nrRows, srcImage, srcEnd);
	return;
    }

    const int srcInc = (rowSize+stride)*pixelSize;
    for ( int idx=0; idx<nrRows; idx++, srcPtr+=srcInc, tileImage+=rowLen )
	boundedCopy( tileImage, srcPtr, rowLen, srcImage, srcEnd );
}


static void copyImageTile( const osg::Image& srcImage, osg::Image& tileImage, const osgGeo::Vec2i& tileOrigin, const osgGeo::Vec2i& tileSize )
{
    tileImage.allocateImage( tileSize.x(), tileSize.y(), srcImage.r(), srcImage.getPixelFormat(), srcImage.getDataType(), srcImage.getPacking() );

    const int pixelSize = srcImage.getPixelSizeInBits()/8;
    int offset = tileOrigin.y()*srcImage.s()+tileOrigin.x();
    offset *= pixelSize;
    const int stride = srcImage.s()-tileSize.x();
    const unsigned char* sourceEnd = srcImage.data() + srcImage.s()*srcImage.t()*srcImage.r()*pixelSize;  

    copyImageWithStride( srcImage.data(), tileImage.data(), tileSize.y(), tileSize.x(), offset, stride, pixelSize, sourceEnd );
}


osg::StateSet* LayeredTexture::createCutoutStateSet(const osg::Vec2f& origin, const osg::Vec2f& opposite, std::vector<LayeredTexture::TextureCoordData>& tcData ) const
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

    if ( !_texInfo->_isValid )
    {
	// Cheap step aside into border until texture info becomes valid
	globalOpposite.x() = _tilingInfo->_envelopeOrigin.x() - 10.0f;
	globalOrigin.x() = globalOpposite.x() - 10.0f;
    }

    for ( int idx=nrDataLayers()-1; idx>=0; idx-- )
    {
	LayeredTextureData* layer = _dataLayers[idx];
	if ( layer->_textureUnit < 0 )
	    continue;

	const osg::Vec2f localOrigin = layer->getLayerCoord( globalOrigin );
	const osg::Vec2f localOpposite = layer->getLayerCoord( globalOpposite );

	const osg::Image* srcImage = layer->_image;
	if ( !srcImage || !srcImage->s() || !srcImage->t() )
	    continue;

	bool xBorderCrossing = localOrigin.x()<-EPS || localOpposite.x()>srcImage->s()+EPS;
	bool yBorderCrossing = localOrigin.y()<-EPS || localOpposite.y()>srcImage->t()+EPS;

	osgGeo::Vec2i size( (int) ceil(localOpposite.x()+0.5),
			    (int) ceil(localOpposite.y()+0.5) );

	osgGeo::Vec2i overshoot( size.x()-srcImage->s(),
				 size.y()-srcImage->t() );
	if ( overshoot.x() > 0 )
	{
	    size.x() -= overshoot.x();
	    overshoot.x() = 0;
	}
	if ( overshoot.y() > 0 )
	{
	    size.y() -= overshoot.y();
	    overshoot.y() = 0;
	}

	osgGeo::Vec2i tileOrigin( (int) floor(localOrigin.x()-0.5),
				  (int) floor(localOrigin.y()-0.5) );
	if ( tileOrigin.x() < 0 )
	    tileOrigin.x() = 0;
	else
	    size.x() -= tileOrigin.x();

	if ( tileOrigin.y() < 0 )
	    tileOrigin.y() = 0;
	else
	    size.y() -= tileOrigin.y();

	if ( size.x()<1 || size.y()<1 )
	{
	    size = osgGeo::Vec2i( 1, 1 );
	    tileOrigin = osgGeo::Vec2i( 0, 0 );
	}

	if ( size.x()>_texInfo->_maxSize || size.y()>_texInfo->_maxSize )
	{
	    std::cerr << "Cut-out exceeds maximum texture size: " << _texInfo->_maxSize << std::endl;
	    if ( size.x() > _texInfo->_maxSize )
	    {
		size.x() = _texInfo->_maxSize;
		xBorderCrossing = true;
	    }
	    if ( size.y() > _texInfo->_maxSize )
	    {
		size.y() = _texInfo->_maxSize;
		yBorderCrossing = true;
	    }
	}

	osgGeo::Vec2i tileSize = size;
	if ( usedTextureSizePolicy() != AnySize )
	{
	    tileSize = osgGeo::Vec2i( powerOf2Ceil(size.x()),
				      powerOf2Ceil(size.y()) );
	}

	bool useTextureBorder = false;
	if ( usedTextureSizePolicy()==BorderedPowerOf2 &&
	     size.x()<srcImage->s()-1 && size.y()<srcImage->t()-1 &&
	     !xBorderCrossing && !yBorderCrossing )
	{
	    const bool xProfit = size.x() != tileSize.x() &&
				 size.x()-2 <= tileSize.x()/2;
	    const bool yProfit = size.y() != tileSize.y() &&
				 size.y()-2 <= tileSize.y()/2;

	    if ( xProfit || yProfit )
	    {
		if ( xProfit )
		    tileSize.x() /= 2;
		if ( yProfit )
		    tileSize.y() /= 2;

		tileSize += osgGeo::Vec2i( 2, 2 );
		useTextureBorder = true;
	    }
	}

	overshoot += tileSize - size;

	bool resizeHint = false;
	if ( tileOrigin.x()<overshoot.x() || tileOrigin.y()<overshoot.y() )
	{
	    std::cerr << "Can't avoid texture resampling for this cut-out!" << std::endl;
	    overshoot = osgGeo::Vec2i( 0, 0 );
	    tileSize = size;
	    resizeHint = true;
	}

	if ( overshoot.x() > 0 )
	    tileOrigin.x() -= overshoot.x();
	if ( overshoot.y() > 0 )
	    tileOrigin.y() -= overshoot.y();

	osg::ref_ptr<osg::Image> tileImage = new osg::Image;

#ifdef USE_IMAGE_STRIDE
	if ( !resizeHint ) // OpenGL crashes when resizing image with stride
	{
	    osg::ref_ptr<osg::Image> si = const_cast<osg::Image*>(srcImage);
	    tileImage->setUserData( si.get() );
	    tileImage->setImage( tileSize.x(), tileSize.y(), si->r(), si->getInternalTextureFormat(), si->getPixelFormat(), si->getDataType(), si->data(tileOrigin.x(),tileOrigin.y()), osg::Image::NO_DELETE, si->getPacking(), si->s() ); 

	    tileImage->ref();
	    layer->_tileImages.push_back( tileImage );
	}
	else
#endif
	    copyImageTile( *srcImage, *tileImage, tileOrigin, tileSize );

	osg::Texture::WrapMode xWrapMode = osg::Texture::CLAMP_TO_EDGE;
	if ( layer->_borderColor[0]>=0.0f && xBorderCrossing )
	    xWrapMode = osg::Texture::CLAMP_TO_BORDER;

	osg::Texture::WrapMode yWrapMode = osg::Texture::CLAMP_TO_EDGE;
	if ( layer->_borderColor[0]>=0.0f && yBorderCrossing )
	    yWrapMode = osg::Texture::CLAMP_TO_BORDER;

	if ( useTextureBorder )
	{
	    tileSize -= osgGeo::Vec2i( 2, 2 );
	    tileOrigin += osgGeo::Vec2i( 1, 1 );
	    xWrapMode = osg::Texture::CLAMP;
	    yWrapMode = osg::Texture::CLAMP;
	}

	osg::Vec2f tc00, tc01, tc10, tc11;
	tc00.x() = (localOrigin.x() - tileOrigin.x()) / tileSize.x();
	tc00.y() = (localOrigin.y() - tileOrigin.y()) / tileSize.y();
	tc11.x() = (localOpposite.x()-tileOrigin.x()) / tileSize.x();
	tc11.y() = (localOpposite.y()-tileOrigin.y()) / tileSize.y();
	tc01 = osg::Vec2f( tc11.x(), tc00.y() );
	tc10 = osg::Vec2f( tc00.x(), tc11.y() );

	tcData.push_back( TextureCoordData( layer->_textureUnit, tc00, tc01, tc10, tc11 ) );

	osg::ref_ptr<osg::Texture2D> texture = new osg::Texture2D( tileImage.get() );
	texture->setResizeNonPowerOfTwoHint( resizeHint );
	texture->setWrap( osg::Texture::WRAP_S, xWrapMode );
	texture->setWrap( osg::Texture::WRAP_T, yWrapMode );
	texture->setBorderWidth( useTextureBorder ? 1 : 0 );

	osg::Texture::FilterMode filterMode = layer->_filterType==Nearest ? osg::Texture::NEAREST : osg::Texture::LINEAR;
	texture->setFilter( osg::Texture::MAG_FILTER, filterMode );

	filterMode = layer->_filterType==Nearest ? osg::Texture::NEAREST_MIPMAP_NEAREST : osg::Texture::LINEAR_MIPMAP_LINEAR;
	texture->setFilter( osg::Texture::MIN_FILTER, filterMode );

	texture->setBorderColor( layer->_borderColor );

	stateset->setTextureAttributeAndModes( layer->_textureUnit, texture.get() );
    }

    return stateset.release();
}


osg::StateSet* LayeredTexture::getSetupStateSet()
{
    updateSetupStateSetIfNeeded();
    return _setupStateSet;
}


void LayeredTexture::updateSetupStateSetIfNeeded()
{
    _lock.readLock();

    if ( !_setupStateSet )
    {
	_setupStateSet = new osg::StateSet;
	_updateSetupStateSet = true;
    }

    checkForModifiedImages();

    std::vector<LayerProcess*>::iterator it = _processes.begin();
    for ( ; it!=_processes.end(); it++ )
	(*it)->checkForModifiedColorSequence();

    if ( _updateSetupStateSet )
    {
	_compositeLayerUpdate = true;

	if ( _useShaders )
	    buildShaders();
	else
	    createCompositeTexture();

	_updateSetupStateSet = false;
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
		setDataLayerImage( (*it)->_id, (*it)->_imageSource );
		(*it)->_imageModifiedFlag = true;
		_updateSetupStateSet = true;
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
    int nrUsedLayers;
    std::vector<int> orderedLayerIDs;
    bool stackIsOpaque;
    const int nrProc = getProcessInfo( orderedLayerIDs, nrUsedLayers, &stackIsOpaque );

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
	_tilingInfo->_retilingNeeded = true;
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
	sprintf( samplerName, "texture%d", *it );
	_setupStateSet->addUniform( new osg::Uniform(samplerName, *it) );
    }

    setRenderingHint( stackIsOpaque );
}


void LayeredTexture::setRenderingHint( bool stackIsOpaque )
{
    if ( getDataLayerIndex(_stackUndefLayerId)>=0 && _stackUndefColor[3]<1.0f )
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


int LayeredTexture::getProcessInfo( std::vector<int>& layerIDs, int& nrUsedLayers, bool* stackIsOpaque ) const
{
    layerIDs.empty();
    std::vector<int> skippedIDs;
    int nrProc = 0;
    nrUsedLayers = 0;

    if ( stackIsOpaque )
	*stackIsOpaque = false;

    std::vector<LayerProcess*>::const_reverse_iterator it = _processes.rbegin();
    for ( ; it!=_processes.rend(); it++ )
    {
	const TransparencyType transparency = (*it)->getTransparencyType();
	int nrPushed = 0;
	for ( int idx=-2; ; idx++ )
	{
	    int id = -1;

	    if ( idx>=0 )
	    {
		id = (*it)->getDataLayerID( idx/2 );

		if ( idx%2 )
		    id = getDataLayerUndefLayerID(id);
		else if ( id<0 && idx>=8 )
		    break;
	    }
	    else if ( idx==-1 && !nrUsedLayers )
		id = _stackUndefLayerId;
	    else if ( idx==-2 && (*it)->needsColorSequence() )
		id = 0;		// ColSeqTexture represented by ID=0

	    if ( id<0 )
		continue;

	    const std::vector<int>::iterator it1 = std::find(layerIDs.begin(),layerIDs.end(),id);
	    const std::vector<int>::iterator it2 = std::find(skippedIDs.begin(),skippedIDs.end(),id);

	    if ( !nrUsedLayers )
	    {
		if ( transparency!=FullyTransparent )
		{
		    if ( it2 != skippedIDs.end() )
			skippedIDs.erase( it2 );
		}
		else if ( it1==layerIDs.end() && it2==skippedIDs.end() )
		    skippedIDs.push_back( id );
	    }

	    if ( it1==layerIDs.end() )
	    {
		if ( !nrUsedLayers || transparency!=FullyTransparent )
		{
		    layerIDs.push_back( id );
		    nrPushed++;
		}
	    }
	}

	if ( !nrUsedLayers )
	{
	    const int sz = layerIDs.size();
	    if ( sz > _texInfo->_nrUnits )
	    {
		nrUsedLayers = sz-nrPushed;
		const bool assigningTextureUnits = !stackIsOpaque;  // Hacky
		if ( assigningTextureUnits )
		    std::cerr << "Earliest process(es) dropped for lack of texture units" << std::endl;
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

    if ( !nrUsedLayers )
	nrUsedLayers = layerIDs.size();

    layerIDs.insert( layerIDs.begin()+nrUsedLayers,
		     skippedIDs.begin(), skippedIDs.end() );
    return nrProc;
}


void LayeredTexture::createColSeqTexture()
{
    osg::ref_ptr<osg::Image> colSeqImage = new osg::Image();
    const int nrProc = nrProcesses();
    const int texSize = powerOf2Ceil( nrProc );
    colSeqImage->allocateImage( 256, texSize, 1, GL_RGBA, GL_UNSIGNED_BYTE );

    const int rowSize = colSeqImage->getRowSizeInBytes();
    std::vector<LayerProcess*>::const_iterator it = _processes.begin();
    for ( int idx=0; idx<nrProc; idx++, it++ )
    {
	const unsigned char* ptr = (*it)->getColorSequencePtr();
	if ( ptr )
	    memcpy( colSeqImage->data(0,idx), ptr, rowSize );

	(*it)->setColorSequenceTextureCoord( (idx+0.5)/texSize );
    }
    osg::ref_ptr<osg::Texture2D> texture = new osg::Texture2D( colSeqImage );
    texture->setFilter( osg::Texture::MIN_FILTER, osg::Texture::NEAREST );
    texture->setFilter( osg::Texture::MAG_FILTER, osg::Texture::NEAREST );
    _setupStateSet->setTextureAttributeAndModes( 0, texture.get() ); 
}


void LayeredTexture::assignTextureUnits()
{
    std::vector<LayeredTextureData*>::iterator lit = _dataLayers.begin();
    for ( ; lit!=_dataLayers.end(); lit++ )
	(*lit)->_textureUnit = -1;

    if ( _useShaders )
    {
	std::vector<int> orderedLayerIDs;
	int nrUsedLayers;
	getProcessInfo( orderedLayerIDs, nrUsedLayers );

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

    _updateSetupStateSet = true;
    updateSetupStateSetIfNeeded();
}


void LayeredTexture::getVertexShaderCode( std::string& code, const std::vector<int>& activeUnits ) const
{
    code =

"void main(void)\n"
"{\n"
"    vec3 fragNormal = normalize(gl_NormalMatrix * gl_Normal);\n"
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
"            float nDotHV = abs( \n"
"	          dot(fragNormal, vec3(gl_LightSource[light].halfVector)) );\n"
"            pf = pow( nDotHV, gl_FrontMaterial.shininess );\n"
"        }\n"
"        specular += gl_LightSource[light].specular * pf;\n"
"    }\n"
"\n"
"    gl_FrontColor =\n"
"        gl_FrontLightModelProduct.sceneColor +\n"
"        ambient  * gl_FrontMaterial.ambient +\n"
"        diffuse  * gl_FrontMaterial.diffuse +\n"
"        specular * gl_FrontMaterial.specular;\n"
"\n"
"    gl_Position = ftransform();\n"
"\n";

    char line[100];
    std::vector<int>::const_iterator it = activeUnits.begin();
    for ( ; it!=activeUnits.end(); it++ )
    {
	sprintf( line, "    gl_TexCoord[%d] = gl_TextureMatrix[%d] * gl_MultiTexCoord%d;\n", *it, *it, *it );
	code += line;
    }

    code += "}\n";

    //std::cout << code << std::endl;
}


void LayeredTexture::getFragmentShaderCode( std::string& code, const std::vector<int>& activeUnits, int nrProc, bool stackIsOpaque ) const
{
    code.clear();
    char line[100];
    std::vector<int>::const_iterator iit = activeUnits.begin();
    for ( ; iit!=activeUnits.end(); iit++ )
    {
	sprintf( line, "uniform sampler2D texture%d;\n", *iit );
	code += line;
    }

    code += "\n";
    const bool stackUdf = getDataLayerIndex(_stackUndefLayerId)>=0;
    code += stackUdf ? "void process( float stackudf )\n" :
		       "void process( void )\n";
    code += "{\n"
	    "    vec4 col, udfcol;\n"
	    "    vec2 texcrd;\n"
	    "    float a, b, udf, oldudf;\n"
	    "\n";

    int stage = 0;
    std::vector<LayerProcess*>::const_reverse_iterator it = _processes.rbegin();
    for ( ; it!=_processes.rend() && nrProc--; it++ )
    {
	if ( (*it)->getTransparencyType() == FullyTransparent )
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
	code += "    gl_FragColor = vec4(1.0,1.0,1.0,1.0);\n";

    code += "}\n"
	    "\n"
	    "void main( void )\n"
	    "{\n"
	    "    if ( gl_FrontMaterial.diffuse.a <= 0.0 )\n"
	    "        discard;\n"
	    "\n";

    if ( stackUdf )
    {
	const int udfUnit = getDataLayerTextureUnit( _stackUndefLayerId );
	sprintf( line, "    vec2 texcrd = gl_TexCoord[%d].st;\n", udfUnit );
	code += line;
	sprintf( line, "    float udf = texture2D( texture%d, texcrd )[%d];\n", udfUnit, _stackUndefChannel );
	code += line;
	code += "\n"
		"    if ( udf < 1.0 )\n"
		"        process( udf );\n"
		"\n";

	sprintf( line, "    vec4 udfcol = vec4(%.6f,%.6f,%.6f,%.6f);\n", _stackUndefColor[0], _stackUndefColor[1], _stackUndefColor[2], _stackUndefColor[3] );
	code += line;

	code += "\n"
		"    if ( udf >= 1.0 )\n"
	    	"        gl_FragColor = udfcol;\n"
		"    else if ( udf > 0.0 )\n";

	if ( _stackUndefColor[3]<=0.0f )
	    code += "        gl_FragColor.a *= 1.0-udf;\n";
	else if ( _stackUndefColor[3]>=1.0f && stackIsOpaque )
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
    else
	 code += "    process();\n";

    code += "\n"
	    "    gl_FragColor.a *= gl_FrontMaterial.diffuse.a;\n"
	    "    gl_FragColor.rgb *= gl_Color.rgb;\n"
	    "}\n";

    //std::cout << code << std::endl;
}


void LayeredTexture::useShaders( bool yn )
{
    _useShaders = yn;
    _tilingInfo->_retilingNeeded = true;
}


void LayeredTexture::setTextureSizePolicy( TextureSizePolicy policy )
{
    _textureSizePolicy = policy;
    _tilingInfo->_retilingNeeded = true;
}


LayeredTexture::TextureSizePolicy LayeredTexture::getTextureSizePolicy() const
{ return _textureSizePolicy; }


LayeredTexture::TextureSizePolicy LayeredTexture::usedTextureSizePolicy() const
{
    if ( _textureSizePolicy==BorderedPowerOf2 && _texInfo->_nonPowerOf2Support )
	return AnySize;

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


void LayeredTexture::createCompositeTexture()
{
    if ( !_compositeLayerUpdate )
	return;

    _compositeLayerUpdate = false;
    updateTilingInfoIfNeeded();
    const osgGeo::TilingInfo& ti = *_tilingInfo;

    const int width  = (int) ceil( ti._envelopeSize.x()/ti._smallestScale.x() );
    const int height = (int) ceil( ti._envelopeSize.y()/ti._smallestScale.y() );
    if ( width<1 || height<1 )
	return;

    const osg::Vec2f scale( ti._envelopeSize.x()/float(width),
			    ti._envelopeSize.y()/float(height) );

    const int idx = getDataLayerIndex( _compositeLayerId );
    osg::Image* image = const_cast<osg::Image*>( _dataLayers[idx]->_image.get() );

    if ( !image || width!=image->s() || height!=image->t() )
    {
	image = new osg::Image;
	image->allocateImage( width, height, 1, GL_RGBA, GL_UNSIGNED_BYTE );
    }

    _dataLayers[idx]->_origin = ti._envelopeOrigin;
    _dataLayers[idx]->_scale = scale;

    const int udfIdx = getDataLayerIndex( _stackUndefLayerId );
    float udf = 0.0f;

    const std::vector<LayerProcess*>& constProcs = _processes;
    std::vector<LayerProcess*>::const_reverse_iterator it;

    for ( int s=0; s<width; s++ )
    {
	for ( int t=0; t<height; t++ )
	{
	    osg::Vec2f globalCoord( (s+0.5)*scale.x(), (t+0.5)*scale.y() );
	    globalCoord += ti._envelopeOrigin;

	    osg::Vec4f fragColor( -1.0f, -1.0f, -1.0f, -1.0f );

	    if ( udfIdx>=0 )
		udf = _dataLayers[udfIdx]->getTextureVec(globalCoord)[_stackUndefChannel];

	    if ( udf<1.0 )
	    {
		for ( it=constProcs.rbegin(); it!=constProcs.rend(); it++ )
		{
		    if ( (*it)->getTransparencyType() == FullyTransparent )
			continue;

		    (*it)->doProcess( fragColor, udf, globalCoord );

		    if ( fragColor[3]>=1.0f )
			break;
		}

		if ( fragColor[0]==-1.0f )
		    fragColor = osg::Vec4f( 1.0f, 1.0f, 1.0f, 1.0f );
	    }

	    if ( udf>=1.0f )
		fragColor = _stackUndefColor;
	    else if ( udf>0.0 )
	    {
		if ( _stackUndefColor[3]<=0.0f )
		    fragColor[3] *= 1.0f-udf;
		else if ( _stackUndefColor[3]>=1.0f && fragColor[3]>=1.0f )
		    fragColor = fragColor*(1.0f-udf) + _stackUndefColor*udf;
		else if ( fragColor[3]>0.0f )
		{
		    const float a = fragColor[3]*(1.0f-udf);
		    const float b = _stackUndefColor[3]*udf;
		    fragColor = (fragColor*a + _stackUndefColor*b) / (a+b);
		    fragColor[3] = a+b;
		}
		else
		{
		    fragColor = _stackUndefColor;
		    fragColor[3] *= udf;
		}
	    }

	    fragColor *= 255.0f;
	    if ( fragColor[3]<0.5f )
		fragColor = osg::Vec4f( 0.0f, 0.0f, 0.0f, 0.0f );

	    for ( int tc=0; tc<4; tc++ )
	    {
		int val = (int) floor( fragColor[tc]+0.5 );
		val = val<=0 ? 0 : (val>=255 ? 255 : val);

		image->data(s,t)[tc] = (unsigned char) val;
	    }
	}
    }

    setDataLayerImage( _compositeLayerId, image );

    if ( !_useShaders )
    {
	_setupStateSet->clear();
	setRenderingHint( getDataLayerTransparencyType(_compositeLayerId)==Opaque );
    }
}


const osg::Image* LayeredTexture::getCompositeTextureImage()
{
    createCompositeTexture();
    return getDataLayerImage( _compositeLayerId );
}


//============================================================================


LayerProcess::LayerProcess( LayeredTexture& layTex )
    : _layTex( layTex )
    , _colSeqPtr( 0 )
    , _newUndefColor( 1.0f, 1.0f, 1.0f, 1.0f )
    , _opacity( 1.0f )
{}


const unsigned char* LayerProcess::getColorSequencePtr() const
{ return _colSeqPtr; }


void LayerProcess::setColorSequenceTextureCoord( float coord )
{
    _colSeqTexCoord = coord;
    _layTex.updateSetupStateSet();
}


float LayerProcess::getOpacity() const
{ return _opacity; }


void LayerProcess::setOpacity( float opac )
{
    _opacity = opac<=0.0f ? 0.0f : ( opac>=1.0f ? 1.0f : opac );
    _layTex.updateSetupStateSet();
}


void LayerProcess::setNewUndefColor( const osg::Vec4f& color )
{
    for ( int idx=0; idx<4; idx++ )
    {
	_newUndefColor[idx] = color[idx]<=0.0f ? 0.0f :
			      color[idx]>=1.0f ? 1.0f : color[idx];
    }

    _layTex.updateSetupStateSet();
}


const osg::Vec4f& LayerProcess::getNewUndefColor() const
{ return _newUndefColor; }


void LayerProcess::getHeaderCode( std::string& code, int& nrUdf, int id, int toIdx, int fromIdx ) const
{
    const int unit = _layTex.getDataLayerTextureUnit(id);
    const int udfId = _layTex.getDataLayerUndefLayerID(id);

    char line[100];

    char to[5] = ""; 
    char from[5] = ""; 
    if ( toIdx>=0 )
    {
	sprintf( to, "[%d]", toIdx );
	sprintf( from, "[%d]", fromIdx );
    }

    code += nrUdf ? "    if ( udf < 1.0 )\n"
		  : "    if ( true )\n";

    code += "    {\n";

    if ( _layTex.getDataLayerIndex(udfId)>=0 )
    {
	if ( nrUdf )
	    code += "        oldudf = udf;\n";

	const int udfUnit = _layTex.getDataLayerTextureUnit( udfId );
	sprintf( line, "        texcrd = gl_TexCoord[%d].st;\n", udfUnit );
	code += line;
	const int udfChannel = _layTex.getDataLayerUndefChannel(id);
	sprintf( line, "        udf = texture2D( texture%d, texcrd )[%d];\n", udfUnit, udfChannel );
	code += line;
	if ( _layTex.areUndefLayersInverted() )
	    code += "        udf = 1.0 - udf;\n";

	code += "\n"
		"        if ( udf < 1.0 )\n"
		"        {\n";

	if ( udfId!=id )
	{
	    sprintf( line, "            texcrd = gl_TexCoord[%d].st;\n", unit );
	    code += line;
	}
	sprintf( line, "            col%s = texture2D( texture%d, texcrd )%s;\n", to, unit, from );
	code += line;

	const osg::Vec4f& udfColor = _layTex.getDataLayerImageUndefColor(id);
	if ( toIdx<0 )
	{
	    std::string ext = ".";
	    for ( int idx=0; idx<4; idx++ )
	    {
		if ( udfColor[idx]>=0.0f )
		    ext += idx==0 ? 'r' : idx==1 ? 'g' : idx==2 ? 'b' : 'a';
	    }

	    if ( ext.size()>1 )
	    {
		if ( ext.size()>4 )
		    ext.clear();

		sprintf( line, "            udfcol = vec4(%.6f,%.6f,%.6f,%.6f);\n", udfColor[0], udfColor[1], udfColor[2], udfColor[3] );
		code += line;
		code += "            if ( udf > 0.0 )\n";
		sprintf( line, "                col%s = (col%s - udf*udfcol%s) / (1.0-udf);\n", ext.data(), ext.data(), ext.data() );
		code += line;
	    }
	}
	else if ( udfColor[fromIdx]>=0.0f )
	{
	    code += "            if ( udf > 0.0 )\n";
	    sprintf( line, "                col%s = (col%s - %.6f*udf) / (1.0-udf);\n", to, to, udfColor[fromIdx] );
	    code += line;
	}

	if ( nrUdf++ )
	{
	    code += "\n"
		    "        udf = max( udf, oldudf );\n";
	}

	code += "        }\n";
    }
    else
    {
	sprintf( line, "        texcrd = gl_TexCoord[%d].st;\n", unit );
	code += line;
	sprintf( line, "        col%s = texture2D( texture%d, texcrd )%s;\n", to, unit, from );
	code += line;
    }

    code += "    }\n";
}


void LayerProcess::getFooterCode( std::string& code, int& nrUdf, int stage ) const
{
    char line[100];

    if ( nrUdf )
    {
	sprintf( line, "    udfcol = vec4(%.6f,%.6f,%.6f,%.6f);\n", _newUndefColor[0], _newUndefColor[1], _newUndefColor[2], _newUndefColor[3] );
	code += line;

	code += "\n"
		"    if ( udf >= 1.0 )\n"
	    	"        col = udfcol;\n";

	const bool stackUdf = _layTex.getDataLayerIndex( _layTex.getStackUndefLayerID() )>=0;

	if ( !stackUdf )
	    code += "    else if ( udf > 0.0 )\n";
	else
	    code += "    else if ( udf > stackudf )\n"
		    "    {\n"
		    "        udf = (udf-stackudf) / (1.0-stackudf);\n";

	if ( _newUndefColor[3]<=0.0f )
	    code += "        col.a *= 1.0-udf;\n";
	else if ( _newUndefColor[3]>=1.0f && getTransparencyType(true)==Opaque )
	    code += "        col = mix( col, udfcol, udf );\n";
	else
	{
	    if ( !stackUdf )
		code += "    {\n";

	    code += "        if ( col.a > 0.0 )\n"
		    "        {\n"
		    "            a = col.a;\n"
		    "            col.a = mix( a, udfcol.a, udf );\n"
		    "            col.rgb = mix(a*col.rgb, udfcol.a*udfcol.rgb, udf) / col.a;\n"
		    "        }\n"
		    "        else\n"
		    "            col = vec4( udfcol.rgb, udf*udfcol.a );\n";

	    if ( !stackUdf )                                                    
		code += "    }\n";
	}

	if ( stackUdf )
	    code += "    }\n";

	code += "\n";
	nrUdf = 0;
    }

    if ( _opacity<1.0 )
    {
	sprintf( line, "    col.a *= %.6f;\n", _opacity );
	code += line;
    }

    if ( stage )
    {
	code += "    a = gl_FragColor.a;\n"
	    	"    b = col.a * (1.0-a);\n"
		"    gl_FragColor.a += b;\n"
		"    if ( gl_FragColor.a > 0.0 )\n"
		"        gl_FragColor.rgb = (a*gl_FragColor.rgb+b*col.rgb) / gl_FragColor.a;\n";
    }
    else
	code += "    gl_FragColor = col;\n";
}


void LayerProcess::processHeader( osg::Vec4f& col, float& udf, const osg::Vec2f& coord, int id, int toIdx, int fromIdx ) const
{
    if ( udf>=1.0f )
	return;

    const int udfId = _layTex.getDataLayerUndefLayerID(id);

    if ( _layTex.getDataLayerIndex(udfId)>=0 )
    {
	const float oldUdf = udf;
	const int udfChannel = _layTex.getDataLayerUndefChannel(id);
	udf = _layTex.getDataLayerTextureVec(udfId,coord)[udfChannel];
	if ( _layTex.areUndefLayersInverted() )
	    udf = 1.0-udf;

	if ( udf<1.0f )
	{
	    const osg::Vec4f& udfCol = _layTex.getDataLayerImageUndefColor(id);

	    if ( toIdx<0 )
	    {
		col = _layTex.getDataLayerTextureVec( id, coord );
		for ( int idx=0; idx<4; idx++ )
		{
		    if ( udf>0.0f && udfCol[idx]>=0.0f )
			col[idx] = (col[idx] - udfCol[idx]*udf) / (1.0f-udf);
		}
	    }
	    else
	    {
		col[toIdx] = _layTex.getDataLayerTextureVec(id,coord)[fromIdx];
		if ( udf>0.0f && udfCol[fromIdx]>=0.0f )
		    col[toIdx] = (col[toIdx]-udfCol[fromIdx]*udf) / (1.0f-udf);
	    }
	}

	if ( udf<oldUdf )
	    udf = oldUdf;
    }
    else if ( toIdx<0 )
	col = _layTex.getDataLayerTextureVec( id, coord );
    else
	col[toIdx] = _layTex.getDataLayerTextureVec(id,coord)[fromIdx];
}


void LayerProcess::processFooter( osg::Vec4f& fragColor, float stackUdf, const osg::Vec2f&, osg::Vec4f col, float udf ) const
{
    if ( udf>=1.0f )
	col = _newUndefColor;
    else if ( udf>stackUdf )
    {
	udf = (udf-stackUdf) / (1.0-stackUdf);

	if ( _newUndefColor[3]<=0.0f )
	    col[3] *= 1.0f-udf;
	else if ( _newUndefColor[3]>=1.0f && col[3]>=1.0f )
	    col = col*(1.0f-udf) + _newUndefColor*udf;
	else if ( col[3]>0 )
	{
	    const float a = col[3] * (1.0f-udf);
	    const float b = _newUndefColor[3] * udf;
	    const float c = a + b;
	    col = (col*a + _newUndefColor*b) / c;
	    col[3] = c;
	}
	else
	{
	    col = _newUndefColor;
	    col[3] *= udf;
	}
    }

    if ( _opacity<1.0 )
	col[3] *= _opacity;

    if ( fragColor[0]==-1.0f )
	fragColor = col;
    else
    {
	const float a = fragColor[3];
	const float b = col[3] * (1.0f-a);
	const float c = a+b;
	if ( c>0.0f )
	    fragColor = (fragColor*a + col*b) / c;

	fragColor[3] = c;
    }
}


void LayeredTexture::invertUndefLayers( bool yn )
{ _invertUndefLayers = yn; }


bool LayeredTexture::areUndefLayersInverted() const
{ return _invertUndefLayers; }


//============================================================================


ColTabLayerProcess::ColTabLayerProcess( LayeredTexture& layTex )
    : LayerProcess( layTex )
    , _id( -1 )
    , _textureChannel( 0 )
    , _colorSequence( 0 )
{}


void ColTabLayerProcess::setDataLayerID( int id, int channel )
{
    _id = id;
    _textureChannel = channel>=0 && channel<4 ? channel : 0;
    _layTex.updateSetupStateSet();
}


int ColTabLayerProcess::getDataLayerID( int idx ) const
{ return idx ? -1 : _id; } 


int ColTabLayerProcess::getDataLayerTextureChannel() const
{ return _textureChannel; }


void ColTabLayerProcess::checkForModifiedColorSequence()
{
    if ( _colorSequence )
    {
	const int modifiedCount = _colorSequence->getModifiedCount();
	if ( modifiedCount != _colSeqModifiedCount )
	{
	    _colSeqModifiedCount = modifiedCount;
	    _layTex.updateSetupStateSet();
	}
    }
}


void ColTabLayerProcess::setColorSequence( const ColorSequence* colSeq )
{
    _colorSequence = colSeq;
    _colSeqModifiedCount = -1;
    _colSeqPtr = colSeq->getRGBAValues();
    _layTex.updateSetupStateSet();
}


const ColorSequence* ColTabLayerProcess::getColorSequence() const
{ return _colorSequence; }


void ColTabLayerProcess::getShaderCode( std::string& code, int stage ) const
{
    if ( _layTex.getDataLayerIndex(_id)<0 )
	return;

    int nrUdf = 0;
    getHeaderCode( code, nrUdf, _id, 0, _textureChannel );
    
    char line[100];
    sprintf( line, "\n    texcrd = vec2( 0.996093*col[0]+0.001953, %.6f );\n", _colSeqTexCoord );
    code += line;
    code += "    col = texture2D( texture0, texcrd );\n"
	    "\n";

    getFooterCode( code, nrUdf, stage );
}


TransparencyType ColTabLayerProcess::getTransparencyType( bool imageOnly ) const
{
    if ( !_colorSequence || !_layTex.getDataLayerImage(_id) )
	return FullyTransparent;

    TransparencyType tt = _colorSequence->getTransparencyType();
    // Only optimal if all colors in sequence are actually used

    if ( imageOnly )
	return tt;

    const int udfId = _layTex.getDataLayerUndefLayerID(_id);
    if ( _layTex.getDataLayerImage(udfId) )
	tt = addOpacity( tt, _newUndefColor[3] );

    return multiplyOpacity( tt, _opacity );
}


void ColTabLayerProcess::doProcess( osg::Vec4f& fragColor, float stackUdf, const osg::Vec2f& globalCoord )
{
    if ( !_colorSequence || _layTex.getDataLayerIndex(_id)<0 )
	return;

    osg::Vec4f col;
    float udf = 0.0f;

    processHeader( col, udf, globalCoord, _id, 0, _textureChannel );

    const int val = (int) floor( 255.0f*col[0] + 0.5 );
    const int offset = val<=0 ? 0 : (val>=255 ? 1020 : 4*val);
    const unsigned char* ptr = _colorSequence->getRGBAValues()+offset;
    for ( int idx=0; idx<4; idx++ )
	col[idx] = float(*ptr++) / 255.0f;

    processFooter( fragColor, stackUdf, globalCoord, col, udf );
}


//============================================================================


RGBALayerProcess::RGBALayerProcess( LayeredTexture& layTex )
    : LayerProcess( layTex )
{
   for ( int idx=0; idx<4; idx++ )
   {
       _id[idx] = -1;
       _isOn[idx] = true;
   }
}


void RGBALayerProcess::setDataLayerID( int idx, int id, int channel )
{
    if ( idx>=0 && idx<4 )
    {
	_id[idx] = id;
	_textureChannel[idx] = channel>=0 && channel<4 ? channel : 0;
	_layTex.updateSetupStateSet();
    }
}


int RGBALayerProcess::getDataLayerID( int idx ) const
{ return idx>=0 && idx<4 ? _id[idx] : -1;  } 


void RGBALayerProcess::turnOn( int idx, bool yn )
{
    if ( idx>=0 && idx<4 )
    {
	_isOn[idx] = yn;
	_layTex.updateSetupStateSet();
    }
}

bool RGBALayerProcess::isOn(int idx) const
{ return idx>=0 && idx<4 ? _isOn[idx] : false; }


int RGBALayerProcess::getDataLayerTextureChannel( int idx ) const
{ return idx>=0 && idx<4 ? _textureChannel[idx] : -1;  } 


void RGBALayerProcess::getShaderCode( std::string& code, int stage ) const
{
    code += "    col = vec4( 0.0, 0.0, 0.0, 1.0 );\n"
	    "\n";

    int nrUdf = 0;
    for ( int idx=0; idx<4; idx++ )
    {
	if ( _isOn[idx] && _layTex.getDataLayerIndex(_id[idx])>=0 )
	{
	    getHeaderCode( code, nrUdf, _id[idx], idx, _textureChannel[idx] );
	    code += "\n";
	}
    }

    getFooterCode( code, nrUdf, stage );
}


TransparencyType RGBALayerProcess::getTransparencyType( bool imageOnly ) const
{
    if ( !_isOn[3] || _layTex.getDataLayerIndex(_id[3])<0 )
	return imageOnly ? Opaque : multiplyOpacity( Opaque, _opacity );

    TransparencyType tt = _layTex.getDataLayerTransparencyType( _id[3], _textureChannel[3] );

    if ( imageOnly )
	return tt;

    for ( int idx=0; idx<4; idx++ )
    {
	const int udfId = _layTex.getDataLayerUndefLayerID( _id[idx] );

	if ( _isOn[idx] && _layTex.getDataLayerImage(udfId) )
	{
	    tt = addOpacity( tt, _newUndefColor[3] );
	    return multiplyOpacity( tt, _opacity );
	}
    }

    return multiplyOpacity( tt, _opacity );
}


void RGBALayerProcess::doProcess( osg::Vec4f& fragColor, float stackUdf, const osg::Vec2f& globalCoord )
{
    osg::Vec4f col( 0.0f, 0.0f, 0.0f, 1.0f );
    float udf = 0.0f;

    for ( int idx=0; idx<4; idx++ )
    {
	if ( _isOn[idx] && _layTex.getDataLayerIndex(_id[idx])>=0 )
	{
	    processHeader( col, udf, globalCoord, _id[idx], idx, _textureChannel[idx] );
	}
    }

    processFooter( fragColor, stackUdf, globalCoord, col, udf );
}	


//============================================================================


IdentityLayerProcess::IdentityLayerProcess( LayeredTexture& layTex, int id )
    : LayerProcess( layTex )
    , _id( id )
{}


int IdentityLayerProcess::getDataLayerID( int idx ) const
{ return idx ? -1 : _id; } 


void IdentityLayerProcess::getShaderCode( std::string& code, int stage ) const
{
    if ( _layTex.getDataLayerIndex(_id)<0 )
	return;

    int nrUdf = 0;
    getHeaderCode( code, nrUdf, _id );

    code += "\n";
    getFooterCode( code, nrUdf, stage );
}


TransparencyType IdentityLayerProcess::getTransparencyType( bool imageOnly ) const
{
    TransparencyType tt = _layTex.getDataLayerTransparencyType(_id);

    if ( imageOnly )
	return tt;

    const int udfId = _layTex.getDataLayerUndefLayerID(_id);

    if ( _layTex.getDataLayerImage(udfId) )
	tt = addOpacity( tt, _newUndefColor[3] );

    return multiplyOpacity( tt, _opacity );
}


void IdentityLayerProcess::doProcess( osg::Vec4f& fragColor, float stackUdf, const osg::Vec2f& globalCoord )
{
    if ( _layTex.getDataLayerIndex(_id)<0 )
	return;

    osg::Vec4f col;
    float udf = 0.0f;

    processHeader( col, udf, globalCoord, _id );
    processFooter( fragColor, stackUdf, globalCoord, col, udf );
}


} //namespace
