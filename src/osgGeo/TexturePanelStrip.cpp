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

*/

#include <osgGeo/TexturePanelStrip>
#include <osgGeo/LayeredTexture>
#include <osgGeo/ComputeBoundsVisitor>
#include <osgGeo/Vec2i>

#include <osg/Geometry>
#include <osg/LightModel>
#include <osg/Version>
#include <osgUtil/CullVisitor>
#include <osgUtil/IntersectionVisitor>
#include <iostream>

namespace osgGeo
{

class TexturePanelStripNode::BoundingGeometry : public osg::Geometry
{
/* BoundingGeometry is an auxiliary class tuning the intersection visitor
   process in two ways: it saves the IntersectionVisitor from going along
   all tiles, and it provides a slightly bigger bounding box to circumvent
   unjustified clipping by osgUtil::LineSegmentIntersector which is caused
   by numerical instability. See osg-forum branch: "LineSegmentIntersector
   gives incorrect results (intersections missing)". */

public:
    			BoundingGeometry(TexturePanelStripNode& tpsn)
			    : _tpsn( tpsn )
			{}
#if OSG_MIN_VERSION_REQUIRED(3,3,2)	
    osg::BoundingBox	computeBoundingBox() const override
			{ return _boundingBox; }
#else
    osg::BoundingBox	computeBound() const		{ return _boundingBox; }
#endif
    void		update();

protected:
    TexturePanelStripNode&	_tpsn;
    osg::BoundingBox		_boundingBox;
};


void TexturePanelStripNode::BoundingGeometry::update()
{
    _boundingBox.init();

    while ( getNumPrimitiveSets() )
	removePrimitiveSet( 0 );

    const osg::Vec2Array& pathCoords = _tpsn.getPath();

    if ( pathCoords.size()<2 )
	return; 

    osg::ref_ptr<osg::Vec3Array> coords = new osg::Vec3Array( 2*pathCoords.size() );

    for ( unsigned int idx=0; idx<pathCoords.size()*2; idx++ )
    {
	(*coords)[idx] = osg::Vec3( pathCoords[idx/2][0], pathCoords[idx/2][1], (idx%2 ? _tpsn.getBottom() : _tpsn.getTop()) );

	_boundingBox.expandBy( (*coords)[idx] );
    }

    setVertexArray( coords.get() );
    addPrimitiveSet( new osg::DrawArrays(GL_TRIANGLE_STRIP,0,coords->size()) );

    const float eps = 1e-4 * _boundingBox.radius();
    const osg::Vec3 margin( eps, eps, eps );

    _boundingBox.expandBy( _boundingBox.corner(0) - margin );
    _boundingBox.expandBy( _boundingBox.corner(7) + margin );

    dirtyBound();
    _tpsn.dirtyBound();
}


//============================================================================


class TexturePanelStripNode::TextureCallbackHandler : public LayeredTexture::Callback
{
public:
    TextureCallbackHandler( TexturePanelStripNode& tpsn )
	: _tpsn( tpsn )
    {}

    void requestRedraw() const override		{ _tpsn.forceRedraw(); }

protected:
    TexturePanelStripNode&	_tpsn;
};


//============================================================================


TexturePanelStripNode::TexturePanelStripNode()
    : _texture( 0 )
    , _textureBrickSize( 64 )
    , _isBrickSizeStrict( false )
    , _pathCoords( new osg::Vec2Array )
    , _pathTexOffsets( new osg::FloatArray )
    , _pathTextureShift( 0.0f )
    , _pathTexShiftStartIdx( 0 )
    , _top( 0.0f )
    , _bottom( 1.0f )
    , _validZRangeOffsets( false )
    , _topTexOffset( 0.0f )
    , _bottomTexOffset( 0.0f )
    , _zTextureShift( 0.0f )
    , _swapTextureAxes( false )
    , _smoothNormals( false )
    , _compositeCutoutTexUnit( -1 )
    , _borderEnvelopeOffset( 0.0f, 0.0f )
    , _panelWidths( new osg::FloatArray )
    , _panelNormals( new osg::Vec3Array )
    , _knotNormals( new osg::Vec3Array )
    , _needsUpdate( false )
    , _frozen( false )
    , _isRedrawing( false )
    , _altTileMode( 0 )
{
    setUpdateVar( _needsUpdate, true );

    osg::ref_ptr<osg::LightModel> lightModel = new osg::LightModel;
    lightModel->setTwoSided( true );	// Needed in non-shader case.
    getOrCreateStateSet()->setAttributeAndModes( lightModel.get() );
    setDataVariance( DYNAMIC );

    _boundingGeometry = new BoundingGeometry( *this );
    _boundingGeometry->update();

     _textureCallbackHandler = new TextureCallbackHandler( *this );
}


TexturePanelStripNode::TexturePanelStripNode( const TexturePanelStripNode& node, const osg::CopyOp& op )
    : osg::Node( node, op )
    , _texture( 0 )
    , _textureBrickSize( node._textureBrickSize )
    , _isBrickSizeStrict( node._isBrickSizeStrict )
    , _pathCoords( new osg::Vec2Array )
    , _pathTexOffsets( new osg::FloatArray )
    , _pathTextureShift( node._pathTextureShift )
    , _pathTexShiftStartIdx( node._pathTexShiftStartIdx )
    , _top( node._top )
    , _bottom( node._bottom )
    , _validZRangeOffsets( node._validZRangeOffsets )
    , _topTexOffset( node._topTexOffset )
    , _bottomTexOffset( node._bottomTexOffset )
    , _zTextureShift( node._zTextureShift )
    , _smoothNormals( node._smoothNormals )
    , _swapTextureAxes( node._swapTextureAxes )
    , _compositeCutoutTexUnit( node._compositeCutoutTexUnit )
    , _borderEnvelopeOffset( node._borderEnvelopeOffset )
    , _panelWidths( new osg::FloatArray )
    , _panelNormals( new osg::Vec3Array )
    , _knotNormals( new osg::Vec3Array )
    , _needsUpdate( false )
    , _frozen( false )
    , _isRedrawing( false )
    , _altTileMode( 0 )
{
    setUpdateVar( _needsUpdate, true );
    setUpdateVar( _frozen, node._frozen );

    if ( node._texture )
    {
	if ( op.getCopyFlags()==osg::CopyOp::DEEP_COPY_ALL )
	    setTexture( (osgGeo::LayeredTexture*) node._texture->clone( op ));
	else
	    setTexture( node._texture );
    }

    _boundingGeometry = new BoundingGeometry( *this );
    _boundingGeometry->update();
    
    setPath( *node._pathCoords );
    setPath2TextureMapping( *node._pathTexOffsets );

    _textureCallbackHandler = new TextureCallbackHandler( *this );
}


TexturePanelStripNode::~TexturePanelStripNode()
{
    cleanUp();
    setTexture( 0 );
}


void TexturePanelStripNode::cleanUp()
{
    for ( std::vector<osg::Geometry*>::iterator it = _geometries.begin();
						it!=_geometries.end(); it++ )
	(*it)->unref();

    _geometries.clear();

    for ( std::vector<osg::StateSet*>::iterator it = _statesets.begin();
						it!=_statesets.end(); it++ )
	(*it)->unref();

    _statesets.clear();

    _compositeCutoutOrigins.clear();
    _compositeCutoutSizes.clear();
}


void TexturePanelStripNode::setTexture( osgGeo::LayeredTexture* lt )
{
    if ( lt==_texture )
	return;

    if ( _texture )
	_texture->removeCallback( _textureCallbackHandler );
    
    _texture = lt;

    if ( _texture )
	_texture->addCallback( _textureCallbackHandler );

    setUpdateVar( _needsUpdate, true );
}


void TexturePanelStripNode::setTextureBrickSize( int sz, bool strict )
{
    if ( sz > 0 )
	_textureBrickSize = sz;

    _isBrickSizeStrict = strict;
    setUpdateVar( _needsUpdate, true );
}


void TexturePanelStripNode::setPath( const osg::Vec2Array& coords )
{
    *_pathCoords = coords;
    _boundingGeometry->update();
    computeNormals();
    setUpdateVar( _needsUpdate, true );
}


void TexturePanelStripNode::setPath2TextureMapping( const osg::FloatArray& offsets )
{
    for ( unsigned int idx=1; idx<offsets.size(); idx++ )
    {
	if ( offsets[idx-1]>offsets[idx] )
	{
	    std::cerr << "Path-to-texture mapping should be monotonously non-decreasing!" << std::endl;
	    return;
	}
    }

    *_pathTexOffsets = offsets;
    setUpdateVar( _needsUpdate, true );
}


void TexturePanelStripNode::setPathTextureShift( float shift, int startIdx )
{
    if ( shift!=_pathTextureShift || startIdx!=_pathTexShiftStartIdx )
    {
	_pathTextureShift = shift;
	_pathTexShiftStartIdx = startIdx;
	setUpdateVar( _needsUpdate, true );
    }
}


void TexturePanelStripNode::setZRange( float top, float bottom )
{
    _top = top;
    _bottom = bottom;
    _boundingGeometry->update();
    setUpdateVar( _needsUpdate, true );
}


void TexturePanelStripNode::setZRange2TextureMapping( bool yn, float topOffset, float bottomOffset )
{
    _validZRangeOffsets = yn;
    _topTexOffset = topOffset;
    _bottomTexOffset = bottomOffset;
    setUpdateVar( _needsUpdate, true );
}


float TexturePanelStripNode::getTopTextureMapping() const
{
    if ( !_texture || !_texture->isEnvelopeDefined() || _validZRangeOffsets )
	return _topTexOffset;

    osg::Vec2f start = _texture->envelopeCenter();
    start -= _texture->textureEnvelopeSize() * 0.5f;
    return start[_swapTextureAxes ? 0 : 1];
}


float TexturePanelStripNode::getBottomTextureMapping() const
{
    if ( !_texture || !_texture->isEnvelopeDefined() || _validZRangeOffsets )
	return _bottomTexOffset;

    osg::Vec2f stop = _texture->envelopeCenter();
    stop += _texture->textureEnvelopeSize() * 0.5f;
    return stop[_swapTextureAxes ? 0 : 1];
}


void TexturePanelStripNode::setZTextureShift( float shift )
{
    if ( shift!=_zTextureShift )
    {
	_zTextureShift = shift;
	setUpdateVar( _needsUpdate, true );
    }
}


void TexturePanelStripNode::swapTextureAxes( bool yn )
{
    if ( _swapTextureAxes != yn )
    {
	_swapTextureAxes = yn;
	setUpdateVar( _needsUpdate, true );
    }
}


void TexturePanelStripNode::smoothNormals( bool yn )
{
    if ( _smoothNormals != yn )
    {
	_smoothNormals = yn;
	setUpdateVar( _needsUpdate, true );
    }
}


int TexturePanelStripNode::getValidNormalIdx( int idx, bool forward ) const
{
    if ( idx<0 || idx>=(int)_panelWidths->size() )
	return -1;

    if ( (*_panelWidths)[idx]<=0.0f )	// Normal undefined
	return getValidNormalIdx( (forward ? idx+1 : idx-1), forward );

    return idx;
}


#define WEIGHTED_AVERAGE true

osg::Vec3 TexturePanelStripNode::getAverageNormal( int idx ) const
{
    const int idx0 = getValidNormalIdx( idx-1, false );
    const int idx1 = getValidNormalIdx( idx, true );

    if ( idx0>=0 && idx1>=0 )
    {
	const float w0 = WEIGHTED_AVERAGE ? (*_panelWidths)[idx0] : 1.0f;
	const float w1 = WEIGHTED_AVERAGE ? (*_panelWidths)[idx1] : 1.0f;
	osg::Vec3 avg = (*_panelNormals)[idx0]*w0 + (*_panelNormals)[idx1]*w1;
	if ( avg.normalize() )
	    return avg;

	/* Sign of average normal might be undefined when the path makes
	   an exact u-turn. Returned normal is arbitrary but consistent */
	osg::Vec3 normal0 = (*_panelNormals)[idx0];
	return osg::Vec3( -normal0[1], normal0[0], 0.0f );
    }

    if ( idx0>=0 )
	return (*_panelNormals)[idx0];
    if ( idx1>=0 )
	return (*_panelNormals)[idx1];

    return osg::Vec3( 1.0f, 0.0f, 0.0f );
}


void TexturePanelStripNode::computeNormals()
{
    _panelWidths->clear();
    _panelNormals->clear();
    _knotNormals->clear();

    for ( unsigned int idx=1; idx<_pathCoords->size(); idx++ )
    {
	osg::Vec2 dif = (*_pathCoords)[idx] - (*_pathCoords)[idx-1];
	_panelWidths->push_back( dif.length() );
	dif.normalize();
	_panelNormals->push_back( osg::Vec3(-dif[1], dif[0], 0.0f) );
    }

    for ( unsigned int idx=0; idx<_pathCoords->size(); idx++ )
	_knotNormals->push_back( getAverageNormal(idx) );

    for ( unsigned int idx=0; idx<_panelNormals->size(); idx++ )
    {
	if ( (*_panelWidths)[idx]<=0.0f )
	    (*_panelNormals)[idx] = (*_knotNormals)[idx];
    }
}


void TexturePanelStripNode::forceRedraw( bool yn )
{
    _redrawLock.writeLock();
    if ( _isRedrawing != yn )
    {
	_isRedrawing = yn;
	int num = getNumChildrenRequiringUpdateTraversal();
	num += yn ? 1 : -1;
	setNumChildrenRequiringUpdateTraversal( num );
    }
    _redrawLock.writeUnlock();
}


void TexturePanelStripNode::setUpdateVar( bool& variable, bool yn )
{
    if ( yn )
	forceRedraw( true );

    variable = yn;
}


void TexturePanelStripNode::traverse( osg::NodeVisitor& nv )
{
    if ( nv.getVisitorType()==osg::NodeVisitor::UPDATE_VISITOR )
    {
	forceRedraw( false );

	if ( _texture && _texture->needsRetiling() )
	    setUpdateVar( _needsUpdate, true );

	if ( !_frozen && _needsUpdate && updateGeometry() )
	    setUpdateVar( _needsUpdate, false );
    }
    else if ( nv.getVisitorType()==osg::NodeVisitor::CULL_VISITOR )
    {
	osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*>(&nv);

	if ( getStateSet() )
	    cv->pushStateSet( getStateSet() );

	if ( _texture && _texture->getSetupStateSet() )
	    cv->pushStateSet( _texture->getSetupStateSet() );

	for ( unsigned int idx=0; idx<_geometries.size(); idx++ )
	{
	    cv->pushStateSet( _statesets[idx] );
#if OSG_MIN_VERSION_REQUIRED(3,3,2)
	    const osg::BoundingBox bb = _geometries[idx]->getBoundingBox();
#else
	    const osg::BoundingBox bb = _geometries[idx]->getBound();
#endif
	    const float depth = cv->getDistanceFromEyePoint(bb.center(),false);
	    cv->addDrawableAndDepth( _geometries[idx], cv->getModelViewMatrix(), depth );

	    cv->popStateSet();
	}

	if ( _texture && _texture->getSetupStateSet() )
	    cv->popStateSet();

	if ( getStateSet() )
	    cv->popStateSet();

#if OSG_MIN_VERSION_REQUIRED(3,3,2)
	if ( _boundingGeometry->getBoundingBox().valid() )
	    cv->updateCalculatedNearFar(*cv->getModelViewMatrix(), _boundingGeometry->getBoundingBox() );
#else
	if ( _boundingGeometry->getBound().valid() )
	    cv->updateCalculatedNearFar(*cv->getModelViewMatrix(), _boundingGeometry->getBound() );
#endif
    }
    else
    {
	osgUtil::IntersectionVisitor* iv =
	    dynamic_cast<osgUtil::IntersectionVisitor*>( &nv );

	if ( iv )
	{
	    osg::ref_ptr<osgUtil::Intersector> intersec = iv->getIntersector()->clone( *iv );

	    if ( intersec.valid() && _boundingGeometry )
		intersec->intersect( *iv, _boundingGeometry );
	}

	osgGeo::ComputeBoundsVisitor* cbv =
	    dynamic_cast<osgGeo::ComputeBoundsVisitor*>( &nv );
	if ( cbv )
	{
#if OSG_MIN_VERSION_REQUIRED(3,3,2)
	    cbv->applyBoundingBox(_boundingGeometry->getBoundingBox());
#else
	    cbv->applyBoundingBox(_boundingGeometry->getBound());
#endif
	}

    }
}


float TexturePanelStripNode::calcZTexOffset( int idx ) const
{
    float offset = idx>0 ? getBottomTextureMapping() : getTopTextureMapping();
    offset -= _zTextureShift;

    if ( _texture )
    {
	const osg::Vec2f origin = _texture->envelopeCenter() -
				  _texture->textureEnvelopeSize() * 0.5f;
	offset -= origin[_swapTextureAxes ? 0 : 1];

	const osg::Vec2 resolution = _texture->tilingPlanResolution();
	offset *= resolution[_swapTextureAxes ? 0 : 1];
    }

    return offset;
}


float TexturePanelStripNode::calcPathTexOffset( int idx ) const
{
    if ( idx<0 || idx>=(int)_pathTexOffsets->size() )
    {
	std::cerr << "_pathTexOffsets index out of bound" << std::endl;
	return -1.0f;
    }

    float offset = (*_pathTexOffsets)[idx];
    if ( idx>=_pathTexShiftStartIdx )
	offset -= _pathTextureShift;

    if ( _texture )
    {
	const osg::Vec2f origin = _texture->envelopeCenter() -
				  _texture->textureEnvelopeSize() * 0.5f;
	offset -= origin[_swapTextureAxes ? 1 : 0];

	const osg::Vec2 resolution = _texture->tilingPlanResolution();
	offset *= resolution[_swapTextureAxes ? 1 : 0];
    }

    return offset;
}


bool TexturePanelStripNode::getLocalGeomAtTexOffset( osg::Vec2& pathCoord, osg::Vec3& normal, float texOffset, int guessPanelIdx ) const
{
    int nrKnots = _pathTexOffsets->size();
    if ( nrKnots>(int)_pathCoords->size() )
	nrKnots = _pathCoords->size();

    if ( nrKnots<2 )
	return false;

    int knot = nrKnots/2;
    if ( guessPanelIdx>=0 && guessPanelIdx<nrKnots-1 )
	knot = guessPanelIdx;

    while ( knot>0 && texOffset<calcPathTexOffset(knot) )
	knot--;
    while ( knot<nrKnots-2 && texOffset>=calcPathTexOffset(knot+1) )
	knot++;

    const float num = texOffset - calcPathTexOffset(knot);
    const float denom = calcPathTexOffset(knot+1) - calcPathTexOffset(knot);
    const float frac = denom==0.0f ? 0.0f : num/denom;

    pathCoord = (*_pathCoords)[knot]*(1.0f-frac) + (*_pathCoords)[knot+1]*frac;

    normal = (*_knotNormals)[knot]*(1.0f-frac)+(*_knotNormals)[knot+1]*frac;
    if ( !normal.normalize() || frac<0.0f || frac>1.0f )
	normal = (*_panelNormals)[knot];

    return true;
}


#define EPS	1e-5


void TexturePanelStripNode::finalizeZTiling( const std::vector<float>& tOrigins, std::vector<float>& zCoords, std::vector<float>& zOffsets )
{
    zCoords.clear();
    zOffsets.clear();

    const int tLast = tOrigins.size()-1;

    const float start = calcZTexOffset( 0 );
    const float stop = calcZTexOffset( 1 );

    const float offset0 = start<=stop ? start : stop;
    const float offset1 = start<=stop ? stop : start;
    const float z0 = start<=stop ? _top : _bottom;
    const float z1 = start<=stop ? _bottom : _top;

    zCoords.push_back( z0 );
    zOffsets.push_back( offset0 );

    for ( int tIdx=1; tIdx<tLast; tIdx++ )
    {
	if ( offset0<tOrigins[tIdx]-EPS && offset1>tOrigins[tIdx]+EPS )
	{
	    const float num = tOrigins[tIdx] - offset0;
	    const float denom = offset1 - offset0;
	    const float frac = denom==0.0f ? 0.0f : num/denom;
	    zCoords.push_back( z0*(1.0f-frac) + z1*frac );
	    zOffsets.push_back( tOrigins[tIdx] );
	}
    }

    zCoords.push_back( z1 );
    zOffsets.push_back( offset1 );
}    


float TexturePanelStripNode::getTexelSizeRatio() const
{
    const float zTextureSize = getBottomTextureMapping() - getTopTextureMapping();
    const float zLength = getBottom() - getTop();

    const float pathTextureSize = _pathTexOffsets->back() - _pathTexOffsets->front();

    if ( zTextureSize==0.0f || zLength==0.0f || pathTextureSize==0.0f )
	return 0.0f;

    float pathLength = 0.0f;
    for ( unsigned int idx=1; idx<_pathCoords->size(); idx++ )
	pathLength += ((*_pathCoords)[idx]-(*_pathCoords)[idx-1]).length();

    if ( pathLength==0.0 )
	return 0.0f;

    const float ratio = zTextureSize*pathLength / (pathTextureSize*zLength);
    return _swapTextureAxes ? 1.0f/ratio : ratio;
}


bool TexturePanelStripNode::updateGeometry()
{
    cleanUp();

    int nrKnots = _pathTexOffsets->size();
    if ( nrKnots>(int)_pathCoords->size() )
	nrKnots = _pathCoords->size();

    if ( !_texture || nrKnots<2 )
	return false;

    _texture->reInitTiling( getTexelSizeRatio() );

    std::vector<float> xTicks, yTicks, zCoords, zOffsets;
    _texture->planTiling(_textureBrickSize, xTicks, yTicks, _isBrickSizeStrict);
    finalizeZTiling( (_swapTextureAxes ? xTicks : yTicks), zCoords, zOffsets );

    float sense = _smoothNormals ? -1.0f : 1.0f;
    if ( zCoords[zCoords.size()-1]-zCoords[0] > 0.0f )
	sense = -sense;

    const std::vector<float>& sOrigins = _swapTextureAxes ? yTicks : xTicks;

    osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
    colors->push_back( osg::Vec4(1.0f,1.0f,1.0f,1.0f) );

    osg::ref_ptr<osg::Vec2Array> tilePath = new osg::Vec2Array;
    osg::ref_ptr<osg::FloatArray> tileOffsets = new osg::FloatArray;
    osg::ref_ptr<osg::Vec3Array> tileNormals = new osg::Vec3Array;

    int knot = 0;

    const int sLast = sOrigins.size()-1;

    for ( int sIdx=1; sIdx<=sLast; sIdx++ )
    {
	if ( sIdx!=sLast && calcPathTexOffset(0)>sOrigins[sIdx]-EPS )
	    continue;

	while ( tileOffsets->size()>1 )
	{
	    tileOffsets->erase( tileOffsets->begin() );
	    tilePath->erase( tilePath->begin() );
	    tileNormals->erase( tileNormals->begin() );
	}

	while ( knot<nrKnots )
	{
	    if ( sIdx==sLast || calcPathTexOffset(knot)<sOrigins[sIdx]+EPS )
	    {
		tileOffsets->push_back( calcPathTexOffset(knot) );
		tilePath->push_back( (*_pathCoords)[knot] );
		if ( _smoothNormals )
		    tileNormals->push_back( (*_knotNormals)[knot] );
		else if ( knot!=0 )
		    tileNormals->push_back( (*_panelNormals)[knot-1] );

		knot++;
	    }
	    else
	    {
		if ( tileOffsets->back()<sOrigins[sIdx]-EPS )
		{
		    tileOffsets->push_back( sOrigins[sIdx] );

		    osg::Vec2 coord;
		    osg::Vec3 normal;
		    getLocalGeomAtTexOffset(coord,normal,sOrigins[sIdx],knot-1);
		    if ( !_smoothNormals )
			normal = (*_panelNormals)[knot-1];

		    tilePath->push_back( coord );
		    tileNormals->push_back( normal );
		}
		break;
	    }
	}

	const int last = tileOffsets->size()-1;
	if ( last<1 )
	    continue;

	const float firstOffset = (*tileOffsets)[0];
	const float lastOffset = (*tileOffsets)[last];

	for ( unsigned int zIdx=1; zIdx<zCoords.size(); zIdx++ )
	{
	    osg::ref_ptr<osg::Vec3Array> coords = new osg::Vec3Array;
	    osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array;
	    osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry;

	    for ( int idx=0; idx<=last; idx++ )
	    {
		for ( int cnt = (_smoothNormals || idx ? 0 : 2);
		      cnt < (_smoothNormals || idx==last ? 2 : 4);
		      cnt++ )
		{
		    const float z = zCoords[cnt==1 || cnt==2 ? zIdx : zIdx-1];
		    coords->push_back( osg::Vec3((*tilePath)[idx], z) );

		    const osg::Vec3 normal = (*tileNormals)[_smoothNormals || cnt>1 ? idx : idx-1];
		    normals->push_back( normal*sense );
		}
	    }

	    std::vector<LayeredTexture::TextureCoordData> tcData;
	    osg::Vec2f origin, opposite;
	    if ( _swapTextureAxes )
	    {
		origin = osg::Vec2f( zOffsets[zIdx-1], firstOffset );
		opposite = osg::Vec2f( zOffsets[zIdx], lastOffset );
	    }
	    else
	    {
		origin = osg::Vec2f( firstOffset, zOffsets[zIdx-1] );
		opposite = osg::Vec2f( lastOffset, zOffsets[zIdx] );
	    }

	    osg::ref_ptr<osg::StateSet> stateset = _texture->createCutoutStateSet( origin, opposite, tcData );

	    std::vector<LayeredTexture::TextureCoordData>::const_iterator it = tcData.begin();
	    for ( ; it!=tcData.end(); it++ )
	    {
		osg::ref_ptr<osg::Vec2Array> texCoords = new osg::Vec2Array;

		for ( int idx=0; idx<=last; idx++ )
		{
		    const float num = (*tileOffsets)[idx] - firstOffset; 
		    const float denom = lastOffset - firstOffset;
		    const float frac = denom==0.0f ? 0.0f : num/denom;

		    osg::Vec2 tc0 = it->_tc00 * (1.0f-frac);
		    tc0 += (_swapTextureAxes ? it->_tc10 : it->_tc01) * frac;
		    osg::Vec2 tc1 = it->_tc11 * frac;
		    tc1 += (_swapTextureAxes ? it->_tc01 : it->_tc10) * (1.0f-frac);

		    for ( int cnt = (_smoothNormals || idx ? 0 : 2);
			  cnt < (_smoothNormals || idx==last ? 2 : 4);
			  cnt++ )
		    {
			texCoords->push_back( cnt==0 || cnt==3 ? tc0 : tc1 );
		    }
		}
		geometry->setTexCoordArray( it->_textureUnit, texCoords.get() );
	    }

	    geometry->setVertexArray( coords.get() );
	    geometry->setNormalArray( normals.get() );
	    geometry->setNormalBinding( osg::Geometry::BIND_PER_VERTEX );
	    geometry->setColorArray( colors.get() );
	    geometry->setColorBinding( osg::Geometry::BIND_OVERALL );

	    GLenum primitive = _smoothNormals ? GL_TRIANGLE_STRIP : GL_QUADS;
	    geometry->addPrimitiveSet( new osg::DrawArrays(primitive,0,coords->size()) );

	    // Precalculate bounding sphere for (multi-threaded) cull traversal
	    geometry->getBound();

	    if ( !_altTileMode || (_altTileMode+sIdx+zIdx)%2 )
	    {
		geometry->ref();
		_geometries.push_back( geometry );
		stateset->ref();
		_statesets.push_back( stateset );

		_compositeCutoutTexUnit = tcData.size() ? tcData.begin()->_textureUnit : -1;
		_compositeCutoutOrigins.push_back( tcData.size() ? tcData.begin()->_cutoutOrigin : Vec2i(0,0) );
		_compositeCutoutSizes.push_back( tcData.size() ? tcData.begin()->_cutoutSize : Vec2i(0,0) );
	    }
	}
    }
   
    return true;
}


osg::BoundingSphere TexturePanelStripNode::computeBound() const
{ return _boundingGeometry->getBound(); }


void TexturePanelStripNode::freezeDisplay( bool yn )
{
    if ( !_frozen && yn )
    {
	osg::NodeVisitor nv( osg::NodeVisitor::UPDATE_VISITOR );
	traverse( nv );
    }

    setUpdateVar( _frozen, yn );
}


const osg::Image* TexturePanelStripNode::getCompositeTextureImage( bool addBorder )
{
    _borderEnvelopeOffset = osg::Vec2f( 0.0f, 0.0f );

    if ( !_texture || !_texture->isEnvelopeDefined() )
    {
	_compositeImageWithBorder = 0;
	return 0;
    }

    const int nrOffsets = _pathTexOffsets->size();
    const osg::Image* compositeImage = _texture->getCompositeTextureImage();

    if ( !nrOffsets || !compositeImage || !addBorder )
    {
	_compositeImageWithBorder = 0;
	return compositeImage;
    }

    const float sOffset0 = _swapTextureAxes ? calcZTexOffset(0)
					    : calcPathTexOffset(0);
    const float sOffset1 = _swapTextureAxes ? calcZTexOffset(1)
					    : calcPathTexOffset(nrOffsets-1);
    const float tOffset0 = _swapTextureAxes ? calcPathTexOffset(0)
					    : calcZTexOffset(0);
    const float tOffset1 = _swapTextureAxes ? calcPathTexOffset(nrOffsets-1)
					    : calcZTexOffset(1);

    int sBorder0 = (int) ceil( -sOffset0 - 0.5 );
    int tBorder0 = (int) ceil( -tOffset0 - 0.5 );
    int sBorder1 = (int) ceil(  sOffset1 - compositeImage->s() + 0.5 );
    int tBorder1 = (int) ceil(  tOffset1 - compositeImage->t() + 0.5 );

    if ( sBorder0 < 0 ) sBorder0 = 0;
    if ( tBorder0 < 0 ) tBorder0 = 0;
    if ( sBorder1 < 0 ) sBorder1 = 0;
    if ( tBorder1 < 0 ) tBorder1 = 0;

    if ( !sBorder0 && !tBorder0 && !sBorder1 && !tBorder1 )
    {
	_compositeImageWithBorder = 0;
	return compositeImage;
    }

    if ( !_compositeImageWithBorder )
	_compositeImageWithBorder = new osg::Image;

    const int sSize = sBorder0 + compositeImage->s() + sBorder1;
    const int tSize = tBorder0 + compositeImage->t() + tBorder1;

    if ( _compositeImageWithBorder->s()!=sSize || _compositeImageWithBorder->t()!=tSize )
    {
	_compositeImageWithBorder->allocateImage( sSize, tSize, 1, GL_RGBA, GL_UNSIGNED_BYTE );
    }

    const int id = _texture->compositeLayerId();
    const osg::Vec4f& borderColor = _texture->getDataLayerBorderColor( id );

#if OSG_MIN_VERSION_REQUIRED(3,3,0)
    for ( int s=0; s<sSize; s++ )
    {
	for ( int t=0; t<tSize; t++ )
	    _compositeImageWithBorder->setColor( borderColor, s, t, 0 );
    }
#else
    // Assuming contiguous data of type GL_RGBA
    unsigned char* imagePtr = _compositeImageWithBorder->data();
    for ( int count=0; count<sSize*tSize; count++ )
    {
	for ( int channel=0; channel<=3; channel++ )
	{
	    int val = (int) floor( 255.0f*borderColor[channel] + 0.5 );
	    val = val<=0 ? 0 : (val>=255 ? 255 : val);
	    *imagePtr = (unsigned char) val;
	    imagePtr++;
	}
    }
#endif

    _compositeImageWithBorder->copySubImage( sBorder0, tBorder0, 0, compositeImage );

    _borderEnvelopeOffset = osg::Vec2f( sBorder0, tBorder0 );

    return _compositeImageWithBorder;
}


const osg::Vec2Array* TexturePanelStripNode::getCompositeTextureCoords( int geomIdx ) const
{
    if ( geomIdx<0 || geomIdx>=_geometries.size() || !_texture )
	return 0;

    const osg::Image* compositeImage = _texture->getCompositeTextureImage();
    if ( _compositeImageWithBorder )
	compositeImage = _compositeImageWithBorder;

    if ( !compositeImage )
	return 0;

    const osg::Geometry* geom = _geometries[geomIdx];
    const osg::Array* arr = geom->getTexCoordArray( _compositeCutoutTexUnit );
    const osg::Vec2Array* texCoords = dynamic_cast<const osg::Vec2Array*>(arr);

    if ( !texCoords )
	return 0;

    const int toId = _texture->compositeLayerId();
    const int fromId = _texture->getTextureUnitLayerId(_compositeCutoutTexUnit);

    const osgGeo::Vec2i tileOrigin = _compositeCutoutOrigins[geomIdx];
    const osgGeo::Vec2i tileSize = _compositeCutoutSizes[geomIdx];

    osg::ref_ptr<osg::Vec2Array> compositeCoords = new osg::Vec2Array();

    for ( int tcIdx=0; tcIdx<texCoords->size(); tcIdx++ )
    {
	osg::Vec2f local( tileOrigin[0] + tileSize[0]*texCoords->at(tcIdx)[0],
			  tileOrigin[1] + tileSize[1]*texCoords->at(tcIdx)[1] );
	_texture->transformDataLayerCoord( local, fromId, toId );

	local += _borderEnvelopeOffset;

	const osg::Vec2 texCoord( local[0] / compositeImage->s(),
				  local[1] / compositeImage->t() );
	compositeCoords->push_back( texCoord );
    }

    return compositeCoords.release();
}

} //namespace osgGeo
