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
#include <osgUtil/CullVisitor>

namespace osgGeo
{

struct LayeredTextureData : public osg::Referenced
{
				LayeredTextureData(int id)
				    : _id( id ), _origin(0,0)
				    , _scale( 1, 1 )
				    , _updateSetupStateSet( true )
				{}

    LayeredTextureData*		clone() const;

    TransparencyType		getTransparencyType();

    const int					_id;
    osg::Vec2f					_origin;
    osg::Vec2f					_scale;
    osg::ref_ptr<const osg::Image>		_image;
    bool					_updateSetupStateSet;
};


LayeredTextureData* LayeredTextureData::clone() const
{
    LayeredTextureData* res = new LayeredTextureData( _id );
    res->_origin = _origin;
    if ( _image )
	res->_image = (osg::Image*) _image->clone(osg::CopyOp::DEEP_COPY_ALL);

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
    for ( int idx=0; idx<lt._datalayers.size(); idx++ )
    {
	osg::ref_ptr<LayeredTextureData> layer =
		co.getCopyFlags()==osg::CopyOp::DEEP_COPY_ALL
	    ? lt._datalayers[idx]->clone()
	    : lt._datalayers[idx];

	layer->ref();
	_datalayers.push_back( layer );
    }
}


LayeredTexture::~LayeredTexture()
{
    std::for_each( _datalayers.begin(), _datalayers.end(),
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
	_datalayers.push_back( ltd );
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
	osg::ref_ptr<LayeredTextureData> ltd = _datalayers[idx];
	_datalayers.erase( _datalayers.begin()+idx );
	_updateSetupStateSet = true;
	ltd->unref();
    }

    _lock.writeUnlock();
}


int LayeredTexture::getDataLayerID( int idx ) const
{
    return idx>=0 && idx<_datalayers.size() 
	? _datalayers[idx]->_id
	: -1;
}


int LayeredTexture::getDataLayerIndex( int id ) const
{
    for ( int idx=_datalayers.size()-1; idx>=0; idx-- )
    {
	if ( _datalayers[idx]->_id==id )
	    return idx;
    }

    return -1;
}

#define SET_GET_PROP( funcpostfix, type, variable ) \
void LayeredTexture::setDataLayer##funcpostfix( int id, type localvar ) \
{ \
    const int idx = getDataLayerIndex( id ); \
    if ( idx!=-1 ) \
	_datalayers[idx]->variable = localvar; \
} \
 \
 \
type LayeredTexture::getDataLayer##funcpostfix( int id ) const \
{ \
    const int idx = getDataLayerIndex( id ); \
    return _datalayers[idx]->variable; \
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
	    if ( _datalayers[idx]->_updateSetupStateSet )
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
	    _datalayers[idx]->_updateSetupStateSet = false;

	_updateSetupStateSet = false;
    }

    _lock.readUnlock();
}

} //namespace
