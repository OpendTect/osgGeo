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


#include <osgGeo/TexturePlane>

#include <osgGeo/LayeredTexture>
#include <osgUtil/CullVisitor>
#include <osg/Geometry>

namespace osgGeo
{

TexturePlaneNode::TexturePlaneNode()
    : _center( 0, 0, 0 )
    , _width( 2, 2, 0 )
    , _needsUpdate( true )
    , _textureEnvelope( -1, -1 )
{}


TexturePlaneNode::TexturePlaneNode( const TexturePlaneNode& node, const osg::CopyOp& co )
    : osg::Node( node, co )
    , _center( node._center )
    , _width( node._width )
    , _needsUpdate( true )
    , _textureEnvelope( -1, -1 )
{
    if ( node._texture )
    {
        if ( co.getCopyFlags()==osg::CopyOp::DEEP_COPY_ALL )
	    _texture = static_cast<LayeredTexture*>(node._texture->clone( co ) );
	else
	    _texture = node._texture;
    }
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
	if ( _needsUpdate )
	    updateGeometry();
    }
    else if ( nv.getVisitorType()==osg::NodeVisitor::CULL_VISITOR )
    {
	osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*>(&nv);

	if ( getStateSet() )
	    cv->pushStateSet( getStateSet() );

	if ( _texture )
	    cv->pushStateSet( _texture->getSetupStateSet() );

	for ( int idx=0; idx<_geometries.size(); idx++ )
	{
	    cv->pushStateSet( _statesets[idx] );
	    cv->addDrawable( _geometries[idx], cv->getModelViewMatrix() );
	    cv->popStateSet();
	}

	if ( _texture )
	    cv->popStateSet();

	if ( getStateSet() )
	    cv->popStateSet();
    }
}


bool TexturePlaneNode::updateGeometry()
{
    cleanUp();

    const osgGeo::Vec2i texturesize = _texture->getEnvelope();

    std::vector<int> sorigins, ssizes;
    _texture->divideAxis( texturesize.x(), _textureBrickSize, sorigins, ssizes );
    const int nrs = sorigins.size();

    std::vector<int> torigins, tsizes;
    _texture->divideAxis( texturesize.y(), _textureBrickSize, torigins, tsizes );
    const int nrt = sorigins.size();

    const int nrcoords = (nrs+1)*(nrt+1)*4;

    osg::ref_ptr<osg::Vec3Array> coords = new osg::Vec3Array( nrcoords );

    for ( int ids=0; ids<nrs; ids++ )
    {
	for ( int idt=0; idt<nrt; idt++ )
	{
	    const short idx = _geometries.size();
	    const short coordidx = idx*4;

	    //Compute coords
	    (*coords)[coordidx]   = osg::Vec3(0,0,0);
	    (*coords)[coordidx+1] = osg::Vec3(0,0,0);
	    (*coords)[coordidx+2] = osg::Vec3(0,0,0);
	    (*coords)[coordidx+3] = osg::Vec3(0,0,0);

	    osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry;
	    geometry->ref();
	    if ( !geometry )
		return false;

	    std::vector<LayeredTexture::TextureCoordData> tcdata;
	    osg::ref_ptr<osg::StateSet> stateset = _texture->createCutoutStateSet( osgGeo::Vec2i(sorigins[ids], torigins[idt] ),
		                                                                      osgGeo::Vec2i(ssizes[ids], tsizes[idt] ), tcdata );

	    for ( std::vector<LayeredTexture::TextureCoordData>::iterator it = tcdata.begin();
		  it!=tcdata.end();
		  it++ )
	    {
		osg::ref_ptr<osg::Vec2Array> tcoords = new osg::Vec2Array( 4 );
		(*tcoords)[0] = it->_tc00;
		(*tcoords)[1] = it->_tc01;
		(*tcoords)[2] = it->_tc11;
		(*tcoords)[3] = it->_tc10;
		geometry->setTexCoordArray( it->_textureUnit, tcoords.get() );
	    }
		    
	    geometry->setVertexArray( coords );
	    geometry->addPrimitiveSet( new osg::DrawArrayLengths( GL_QUADS, coordidx, 4 ) );

	    _geometries.push_back( geometry );
	    _statesets.push_back( stateset );
	}
    }

    return true;
}


void TexturePlaneNode::setCenter( const osg::Vec3& center )
{
    _center = center;
    _needsUpdate = true;
}


const osg::Vec3& TexturePlaneNode::getCenter() const
{ return _center; }


void TexturePlaneNode::setTextureBrickSize( short nt )
{
    _textureBrickSize = nt;
}

    
short TexturePlaneNode::getTextureBrickSize() const
{ return _textureBrickSize; }


void TexturePlaneNode::setWidth( const osg::Vec3& width )
{
    _width = width;
    _needsUpdate = true;
}


const osg::Vec3& TexturePlaneNode::getWidth() const
{ return _width; }


void TexturePlaneNode::setLayeredTexture(LayeredTexture*)
{
}


bool TexturePlaneNode::needsUpdate() const
{
    if ( _needsUpdate )
	return true;
    
    return !_texture || _texture->getEnvelope()!=_textureEnvelope;
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



} //namespace osgGeo
