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


#include <osgGeo/TexturePlane>

#include <osgGeo/LayeredTexture>
#include <osgUtil/CullVisitor>
#include <osgUtil/IntersectionVisitor>
#include <osg/Geometry>
#include <osg/LightModel>
#include <osgGeo/Vec2i>
#include <osg/Version>


namespace osgGeo
{

class TexturePlaneNode::BoundingGeometry : public osg::Geometry
{
/* BoundingGeometry is an auxiliary class tuning the intersection visitor
   process in two ways: it saves the IntersectionVisitor from going along
   all tiles, and it provides a slightly bigger bounding box to circumvent
   unjustified clipping by osgUtil::LineSegmentIntersector which is caused
   by numerical instability. See osg-forum branch: "LineSegmentIntersector
   gives incorrect results (intersections missing)". */

public:
			BoundingGeometry(TexturePlaneNode& tpn)
			    : _tpn( tpn )
			{
			    addPrimitiveSet(new osg::DrawArrays(GL_QUADS,0,4));
			}

    osg::BoundingBox	computeBound() const	{ return _boundingBox; }
    void		update();

protected:
    TexturePlaneNode&	_tpn;
    osg::BoundingBox	_boundingBox;
};


void TexturePlaneNode::BoundingGeometry::update()
{
    _boundingBox.init();

    osg::ref_ptr<osg::Vec3Array> corners = new osg::Vec3Array( 4 );

    (*corners)[0] = (*corners)[1] = _tpn.getCenter() - _tpn.getWidth()*0.5;
    (*corners)[2] = (*corners)[3] = _tpn.getCenter() + _tpn.getWidth()*0.5;

    if ( _tpn.getThinDim() )
	(*corners)[1].x() = (*corners)[2].x();
    else
	(*corners)[1].y() = (*corners)[2].y();

    (*corners)[3] += (*corners)[0] - (*corners)[1];

    setVertexArray( corners.get() );

    const float eps = 1e-4 * _tpn.getWidth().length();
    const osg::Vec3 margin( eps, eps, eps );

    _boundingBox.expandBy( (*corners)[0] - margin );
    _boundingBox.expandBy( (*corners)[0] + margin );
    _boundingBox.expandBy( (*corners)[2] - margin );
    _boundingBox.expandBy( (*corners)[2] + margin );

    dirtyBound();
    _tpn.dirtyBound();
}


//============================================================================


TexturePlaneNode::TexturePlaneNode()
    : _center( 0, 0, 0 )
    , _width( 1, 1, 0 )
    , _textureBrickSize( 64 )
    , _isBrickSizeStrict( false )
    , _needsUpdate( true )
    , _swapTextureAxes( false )
    , _textureShift( 0.0f, 0.0f )
    , _textureGrowth( 0.0f, 0.0f )
    , _frozen( false )
    , _disperseFactor( 0 )
{
    osg::ref_ptr<osg::LightModel> lightModel = new osg::LightModel;
    lightModel->setTwoSided( true );
    getOrCreateStateSet()->setAttributeAndModes( lightModel.get() );
    setDataVariance( DYNAMIC );

    _boundingGeometry = new BoundingGeometry( *this );
    _boundingGeometry->update();

    setNumChildrenRequiringUpdateTraversal( 1 );
}


TexturePlaneNode::TexturePlaneNode( const TexturePlaneNode& node, const osg::CopyOp& co )
    : osg::Node( node, co )
    , _center( node._center )
    , _width( node._width )
    , _textureBrickSize( node._textureBrickSize )
    , _isBrickSizeStrict( node._isBrickSizeStrict )
    , _needsUpdate( true )
    , _swapTextureAxes( node._swapTextureAxes )
    , _textureShift( node._textureShift )
    , _textureGrowth( node._textureGrowth )
    , _frozen( node._frozen )
    , _disperseFactor( node._disperseFactor )
{
    if ( node._texture )
    {
        if ( co.getCopyFlags()==osg::CopyOp::DEEP_COPY_ALL )
	    _texture = static_cast<LayeredTexture*>(node._texture->clone( co ) );
	else
	    _texture = node._texture;
    }

    _boundingGeometry = new BoundingGeometry( *this );
    _boundingGeometry->update();

    setNumChildrenRequiringUpdateTraversal(getNumChildrenRequiringUpdateTraversal()+1);
}


TexturePlaneNode::~TexturePlaneNode()
{
    cleanUp();
}


void TexturePlaneNode::cleanUp()
{
    for ( std::vector<osg::Geometry*>::iterator it = _geometries.begin();
	  it!=_geometries.end();
	  it++ )
	(*it)->unref();

    _geometries.clear();

    for ( std::vector<osg::StateSet*>::iterator it = _statesets.begin();
	  it!=_statesets.end();
	  it++ )
	(*it)->unref();

    _statesets.clear();
}

void TexturePlaneNode::traverse( osg::NodeVisitor& nv )
{
    if ( nv.getVisitorType()==osg::NodeVisitor::UPDATE_VISITOR )
    {
	if ( !_frozen && needsUpdate() )
	    updateGeometry();
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
	    cv->addDrawable( _geometries[idx], cv->getModelViewMatrix() );
	    cv->popStateSet();
	}

	if ( _texture && _texture->getSetupStateSet() )
	    cv->popStateSet();

	if ( getStateSet() )
	    cv->popStateSet();

	cv->updateCalculatedNearFar(*cv->getModelViewMatrix(), _boundingGeometry->getBound() );
    }
    else
    {
	osgUtil::IntersectionVisitor* iv =
			    dynamic_cast<osgUtil::IntersectionVisitor*>( &nv );

#if OSG_MIN_VERSION_REQUIRED(3,1,3)
	if ( iv )
#else
        // Covers non-introduced osg::Dragger::setIntersectionMask(.) function
	if ( iv && iv->getTraversalMask()!=Node::NodeMask(~0) ) 
#endif
	{
	    osgUtil::Intersector* intersec = iv->getIntersector()->clone( *iv );
	    if ( intersec && _boundingGeometry )
		intersec->intersect( *iv, _boundingGeometry );
	}
    }
}


void TexturePlaneNode::finalizeTiling( std::vector<float>& origins, int dim ) const
{
    if ( origins.size()<2 || dim<0 || dim>1 || !_texture )
	return;

    const osg::Vec2 resolution = _texture->tilingPlanResolution();
    const float shift  = _textureShift[dim] * resolution[dim];
    const float growth = _textureGrowth[dim] * resolution[dim];

    if ( !shift && !growth )
	return;

    if ( origins.back()-origins.front()+growth <= 0.0f )
	return;

    origins.front() -= shift + 0.5*growth;
    origins.back()  -= shift - 0.5*growth;
    for ( int idx=origins.size()-2; idx>0; idx-- )
    {
	if ( origins.front()>=origins[idx] || origins.back()<=origins[idx] )
	    origins.erase( origins.begin()+idx );
    }
}


bool TexturePlaneNode::updateGeometry()
{
    if ( !_texture ) 
	return false;

    cleanUp();

    _texture->reInitTiling();

    std::vector<float> sOrigins, tOrigins;
    _texture->planTiling( _textureBrickSize, sOrigins, tOrigins, _isBrickSizeStrict );

    finalizeTiling( sOrigins, 0 );
    finalizeTiling( tOrigins, 1 );

    const int nrs = sOrigins.size()-1;
    const int nrt = tOrigins.size()-1;

    osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array;
    osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;

    const char thinDim = getThinDim();
    const osg::Vec3 normal = thinDim==2 ? osg::Vec3( 0.0f, 0.0f, getSense() ) :
			     thinDim==1 ? osg::Vec3( 0.0f,-getSense(), 0.0f ) :
					  osg::Vec3( getSense(), 0.0f, 0.0f ) ;
    normals->push_back( normal );
    colors->push_back( osg::Vec4(1.0f,1.0f,1.0f,1.0f) );

    for ( int ids=0; ids<nrs; ids++ )
    {
	for ( int idt=0; idt<nrt; idt++ )
	{
	    float ds = sOrigins[ids+1]-sOrigins[ids];
	    float dt = tOrigins[idt+1]-tOrigins[idt];

	    if ( _disperseFactor )
	    {
		if (_disperseFactor < 0 ) _disperseFactor = 0;
		if (_disperseFactor > 50 ) _disperseFactor = 50;
		ds *= 1.0f - _disperseFactor*0.01f;
		dt *= 1.0f - _disperseFactor*0.01f;
	    }

	    osg::ref_ptr<osg::Vec3Array> coords = new osg::Vec3Array( 4 );

	    (*coords)[0] = osg::Vec3( sOrigins[ids], tOrigins[idt], 0.0f );
	    (*coords)[1] = osg::Vec3( sOrigins[ids]+ds, tOrigins[idt], 0.0f );
	    (*coords)[2] = osg::Vec3( sOrigins[ids]+ds, tOrigins[idt]+dt, 0.0f);
	    (*coords)[3] = osg::Vec3( sOrigins[ids], tOrigins[idt]+dt, 0.0f );

	    for ( int idx=0; idx<4; idx++ )
	    {
		(*coords)[idx].x() -= sOrigins[0];
		(*coords)[idx].y() -= tOrigins[0];
		(*coords)[idx].x() /= sOrigins[nrs] - sOrigins[0];
		(*coords)[idx].y() /= tOrigins[nrt] - tOrigins[0];
		(*coords)[idx] -= osg::Vec3( 0.5f, 0.5f, 0.0f );

		if ( _swapTextureAxes )
		    (*coords)[idx] = osg::Vec3( (*coords)[idx].y(), (*coords)[idx].x(), 0.0f );

		if ( thinDim==0 )
		    (*coords)[idx] = osg::Vec3( 0.0f, (*coords)[idx].x(), (*coords)[idx].y() );
		else if ( thinDim==1 )
		    (*coords)[idx] = osg::Vec3( (*coords)[idx].x(), 0.0f, (*coords)[idx].y() );

		(*coords)[idx].x() *= _width.x();
		(*coords)[idx].y() *= _width.y();
		(*coords)[idx].z() *= _width.z();
		(*coords)[idx] += _center;
	    }

	    osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry;
	    geometry->ref();

	    geometry->setVertexArray( coords.get() );
	    geometry->setNormalArray( normals.get() );
	    geometry->setNormalBinding( osg::Geometry::BIND_OVERALL );
	    geometry->setColorArray( colors.get() );
	    geometry->setColorBinding( osg::Geometry::BIND_OVERALL );

	    std::vector<LayeredTexture::TextureCoordData> tcData;

	    osg::Vec2f origin( sOrigins[ids], tOrigins[idt] );
	    osg::Vec2f opposite( sOrigins[ids+1], tOrigins[idt+1] );
	    osg::ref_ptr<osg::StateSet> stateset = _texture->createCutoutStateSet( origin, opposite, tcData );
	    stateset->ref();

	    for ( std::vector<LayeredTexture::TextureCoordData>::iterator it = tcData.begin();
		  it!=tcData.end();
		  it++ )
	    {
		osg::ref_ptr<osg::Vec2Array> tCoords = new osg::Vec2Array( 4 );
		(*tCoords)[0] = it->_tc00;
		(*tCoords)[1] = it->_tc01;
		(*tCoords)[2] = it->_tc11;
		(*tCoords)[3] = it->_tc10;
		geometry->setTexCoordArray( it->_textureUnit, tCoords.get() );
	    }
		    
	    geometry->addPrimitiveSet( new osg::DrawArrays(GL_QUADS,0,4) );

	    _geometries.push_back( geometry );
	    _statesets.push_back( stateset );
	}
    }

    _needsUpdate = false;
    return true;
}


osg::BoundingSphere TexturePlaneNode::computeBound() const
{ return _boundingGeometry->getBound(); }


void TexturePlaneNode::setCenter( const osg::Vec3& center )
{
    _center = center;
    _boundingGeometry->update();
    _needsUpdate = true;
}


const osg::Vec3& TexturePlaneNode::getCenter() const
{ return _center; }


void TexturePlaneNode::setTextureBrickSize( short sz, bool strict )
{
    _textureBrickSize = sz;
    _isBrickSizeStrict = strict;
}

    
short TexturePlaneNode::getTextureBrickSize() const
{ return _textureBrickSize; }


void TexturePlaneNode::setWidth( const osg::Vec3& width )
{
    _width = width;
    _boundingGeometry->update();
    _needsUpdate = true;
}


const osg::Vec3& TexturePlaneNode::getWidth() const
{ return _width; }


float TexturePlaneNode::getSense() const 
{
    float sense = _width.x()<0 ? -1.0f : 1.0f;
    sense = _width.y()<0 ? -sense :  sense;
    return _width.z()<0 ? -sense : sense;
}


void TexturePlaneNode::setLayeredTexture( LayeredTexture* lt )
{
    _texture =  lt;
    _needsUpdate = true;
}


bool TexturePlaneNode::needsUpdate() const
{
    if ( _needsUpdate )
	return true;

    return _texture && _texture->needsRetiling();
}


LayeredTexture* TexturePlaneNode::getLayeredTexture()
{ return _texture; }


const LayeredTexture* TexturePlaneNode::getLayeredTexture() const
{ return _texture; }


char TexturePlaneNode::getThinDim() const
{
    if ( !_width.x() )
	return 0;

    if ( !_width.y() )
	return 1;

    return 2;
}


void TexturePlaneNode::swapTextureAxes( bool yn )
{ _swapTextureAxes = yn; }


bool TexturePlaneNode::areTextureAxesSwapped() const
{ return _swapTextureAxes; }


void TexturePlaneNode::freezeDisplay( bool yn )
{
    if ( !_frozen && yn )
    {
	osg::NodeVisitor nv( osg::NodeVisitor::UPDATE_VISITOR );
	traverse( nv );
    }

    _frozen = yn;
}


bool TexturePlaneNode::isDisplayFrozen() const
{ return _frozen; }


void TexturePlaneNode::setTextureShift( const osg::Vec2& shift )
{
    _textureShift = shift;
    _needsUpdate = true;
}


const osg::Vec2& TexturePlaneNode::getTextureShift() const
{ return _textureShift; }


void TexturePlaneNode::setTextureGrowth( const osg::Vec2& growth )
{
    _textureGrowth = growth;
    _needsUpdate = true;
}


const osg::Vec2& TexturePlaneNode::getTextureGrowth() const
{ return _textureGrowth; }


void TexturePlaneNode::toggleShaders()
{
    if ( _texture )
	_texture->allowShaders( !_texture->areShadersAllowed() );
}


} //namespace osgGeo
