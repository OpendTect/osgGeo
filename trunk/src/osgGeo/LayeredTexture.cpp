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

#include <osg/Geometry>
#include <osg/Texture2D>
#include <osgUtil/CullVisitor>

namespace osgGeo
{

struct LayeredTextureData : public osg::Referenced
{
			LayeredTextureData(int id)
			    : _id( id ), _origin(0,0)
			    , _scale( 1, 1 )
			    , _updateSetupStateSet( true )
			    , _textureUnit( -1 )
			{}

    LayeredTextureData*	clone() const;
    osg::Vec2f		getLayerCoord(const osgGeo::Vec2i& global) const;

    TransparencyType	getTransparencyType();

    const int				_id;
    osg::Vec2f				_origin;
    osg::Vec2f				_scale;
    osg::ref_ptr<const osg::Image>	_image;
    bool				_updateSetupStateSet;
    int					_textureUnit;
};


LayeredTextureData* LayeredTextureData::clone() const
{
    LayeredTextureData* res = new LayeredTextureData( _id );
    res->_origin = _origin;
    if ( _image )
	res->_image = (osg::Image*) _image->clone(osg::CopyOp::DEEP_COPY_ALL);

    return res;
}


osg::Vec2f LayeredTextureData::getLayerCoord(
				const osgGeo::Vec2i& global ) const
{
    osg::Vec2f res = osg::Vec2f(global._v[0], global._v[1] )-_origin;
    res._v[0] /= _scale._v[0];
    res._v[1] /= _scale._v[1];
    
    return res;
}


LayeredTexture::LayeredTexture()
    : _freeId( 0 )
    , _updateSetupStateSet( false )
{}


LayeredTexture::LayeredTexture( const LayeredTexture& lt,
				const osg::CopyOp& co )
    : osg::Object( lt, co )
    , _freeId( lt._freeId )
    , _updateSetupStateSet( false )
    , _setupStateSet( 0 )
{
    for ( int idx=0; idx<lt._dataLayers.size(); idx++ )
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

    _updateSetupStateSet = true;

    _lock.writeUnlock();
    return ltd ? ltd->_id : -1;
}


void LayeredTexture::removeDataLayer( int id )
{
    _lock.writeLock();
    const int idx = getDataLayerIndex( id );
    if ( idx!=-1 )
    {
	osg::ref_ptr<LayeredTextureData> ltd = _dataLayers[idx];
	_dataLayers.erase( _dataLayers.begin()+idx );
	_updateSetupStateSet = true;
	ltd->unref();
    }

    _lock.writeUnlock();
}


int LayeredTexture::getDataLayerID( int idx ) const
{
    return idx>=0 && idx<_dataLayers.size() 
	? _dataLayers[idx]->_id
	: -1;
}


int LayeredTexture::getDataLayerIndex( int id ) const
{
    for ( int idx=_dataLayers.size()-1; idx>=0; idx-- )
    {
	if ( _dataLayers[idx]->_id==id )
	    return idx;
    }

    return -1;
}

#define SET_GET_PROP( funcpostfix, type, variable ) \
void LayeredTexture::setDataLayer##funcpostfix( int id, type localvar ) \
{ \
    const int idx = getDataLayerIndex( id ); \
    if ( idx!=-1 ) \
	_dataLayers[idx]->variable = localvar; \
} \
 \
 \
type LayeredTexture::getDataLayer##funcpostfix( int id ) const \
{ \
    const int idx = getDataLayerIndex( id ); \
    return _dataLayers[idx]->variable; \
}

SET_GET_PROP( Image, const osg::Image*, _image )
SET_GET_PROP( Origin, const osg::Vec2f&, _origin )
SET_GET_PROP( Scale, const osg::Vec2f&, _scale )

void LayeredTexture::addProcess( LayerProcess* process )
{
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
    std::vector<LayerProcess*>::iterator neighbor = it inc; \
    if ( it!=_processes.end() && ( cond ) ) \
    { \
	std::swap( it, neighbor ); \
	_updateSetupStateSet = true; \
    } \
 \
    _lock.writeUnlock(); \
}

MOVE_LAYER( moveProcessEarlier, neighbor!=_processes.begin(), -1 )
MOVE_LAYER( moveProcessLater, neighbor!=_processes.end(), +1 )

osg::StateSet* LayeredTexture::getSetupStateSet()
{
    updateSetupStateSet();
    return _setupStateSet;
}


osgGeo::Vec2i LayeredTexture::getEnvelope() const
{
    if ( !_dataLayers.size() )
	return osgGeo::Vec2i( 0, 0 );

    std::vector<LayeredTextureData*>::const_iterator it = _dataLayers.begin();
    osgGeo::Vec2i res((*it)->_image->s(), (*it)->_image->t() );
    for ( it++; it!=_dataLayers.end(); it++ )
    {
	const int s = (*it)->_image->s();
	const int t = (*it)->_image->t();
	if ( s>res._v[0] ) res._v[0] = s;
	if ( t>res._v[1] ) res._v[1] = t;
    }

    return res;
}


void LayeredTexture::divideAxis( int size, int bricksize,
				 std::vector<int>& origins,
				 std::vector<int>& sizes )
{
    int cur = 0;

    do
    {
	origins.push_back( cur );

	int cursize = bricksize;
	while ( cur+cursize/2>=size )
	    cursize /=2;

	int next = cur+cursize-1;
	sizes.push_back( cursize );
	cur = next;

    } while ( cur<size );

}


void LayeredTexture::updateSetupStateSet()
{
    _lock.readLock();

    bool needsupdate = _updateSetupStateSet;
    if ( !_setupStateSet )
    {
	_setupStateSet = new osg::StateSet;
	needsupdate = true;
    }

    if ( !needsupdate )
    {
	for ( int idx=nrDataLayers()-1; idx>=0; idx-- )
	{
	    if ( _dataLayers[idx]->_updateSetupStateSet )
	    {
		needsupdate = true;
		break;
	    }
	}
    }

    if ( needsupdate )
    {
	//construct shaders and put in stateset
	//TODO
	
	for ( int idx=nrDataLayers()-1; idx>=0; idx-- )
	    _dataLayers[idx]->_updateSetupStateSet = false;

	_updateSetupStateSet = false;
    }

    _lock.readUnlock();
}


unsigned int LayeredTexture::getTextureSize( unsigned short nr )
{
    if ( nr<=256 )
    {
	if ( nr<=16 )
	{
	    if ( nr<=4 )
	    {
		if ( nr<=2 )
		    return nr;

		return 4;
	    }

	    if ( nr<=8 )
		return 8;

	    return 16;
	}

	if ( nr<128 )
	{
	    if (nr<=32 )
		return 32;
	    return 64;
	}

	if ( nr<=128 )
	    return 128;
	return 256;
    }

    if ( nr<=4096 )
    {
	if ( nr<=1024 )
	{
	    if ( nr<=512 )
		return 512;

	    return 1024;
	}

	if ( nr<=2048 )
	    return 2048;

	return 4096;
    }

    if ( nr<=16384 )
    {
	if ( nr<=8192 )
	    return 8192;

	return 16384;
    }

    if ( nr<=32768 )
	return 32768;

    return 65526;
}

osg::StateSet*
LayeredTexture::createCutoutStateSet(const osgGeo::Vec2i& origin,
    const osgGeo::Vec2i& size,
    std::vector<LayeredTexture::TextureCoordData>& tcdata ) const
{
    tcdata.clear();
    osg::ref_ptr<osg::StateSet> stateset = new osg::StateSet;

    for ( int idx=nrDataLayers()-1; idx>=0; idx-- )
    {
	osg::Vec2f tc00, tc01, tc10, tc11;
	LayeredTextureData* layer = _dataLayers[idx];
	const osg::Vec2f layeroriginf = layer->getLayerCoord( origin );
	const osg::Vec2f layersizef =
	    layer->getLayerCoord( origin+size )-layeroriginf;
	const osgGeo::Vec2i layerorigin( (int) layeroriginf._v[0],
					 (int) layeroriginf._v[1] );
	osgGeo::Vec2i layersize( getTextureSize((int) layersize._v[0]+0.5),
			      getTextureSize((int) layersize._v[1]+0.5 ) );

	osg::ref_ptr<const osg::Image> sourceimage = layer->_image;
	osg::ref_ptr<osg::Image> imagetile = new osg::Image();
	if ( layerorigin._v[0]<0 ||
	     layerorigin._v[0]+layersize._v[0]>sourceimage->s() ||
	     layerorigin._v[1]<0 ||
	     layerorigin._v[1]+layersize._v[1]>sourceimage->t() )
	{
	    //copy image
	}
	else
	{
	    const int offset =
		(layerorigin._v[0] + layerorigin._v[1]*sourceimage->r())*sourceimage->getPixelSizeInBits()/8;
	    imagetile->setImage( size._v[0], size._v[1], 1, sourceimage->getInternalTextureFormat(), sourceimage->getPixelFormat(), sourceimage->getDataType(),
		    const_cast<unsigned char*>(sourceimage->data()+offset), osg::Image::NO_DELETE, 1);
	}

	osg::ref_ptr<osg::Texture2D> texture = new osg::Texture2D( imagetile );
	stateset->setTextureAttribute( layer->_textureUnit, texture.get() );
	tcdata.push_back( TextureCoordData( layer->_textureUnit, tc00, tc01, tc10, tc11 ) );
    }

    return stateset;
}

} //namespace
