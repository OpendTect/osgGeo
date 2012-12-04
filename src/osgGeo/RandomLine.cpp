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

$Id$

*/

#include <osgGeo/RandomLine>

#include <osgGeo/LayeredTexture>

#include <osg/Geode>

using namespace osgGeo;


RandomLineNode::RandomLineNode()
    : _texture( 0 )
    , _pathindices( new osg::UIntArray )
    , _pathcoords( new osg::Vec2Array )
    , _top( 0.0f )
    , _bottom( 1.0f )
    , _geode( new osg::Geode )
    , _updatecount( 0 )
    , _lastgeomupdatecount( -1 )
{
}


RandomLineNode::RandomLineNode( const RandomLineNode& b,
	       const osg::CopyOp& op )
    : osg::Node( b, op )
    , _texture( 0 )
    , _pathindices( new osg::UIntArray )
    , _pathcoords( new osg::Vec2Array )
    , _top( b._top )
    , _bottom( b._bottom )
    , _geode( new osg::Geode )
    , _updatecount( 0 )
    , _lastgeomupdatecount( -1 )
{
    if ( b._texture )
    {
	if ( op.getCopyFlags()==osg::CopyOp::DEEP_COPY_ALL )
	    setTexture( (osgGeo::LayeredTexture*) b._texture->clone( op ));
	else
	    setTexture( b._texture );
    }
    
    setPath( *b._pathcoords );
    setTextureMapping( *b._pathindices );
}


RandomLineNode::~RandomLineNode()
{
}



void RandomLineNode::setTexture( osgGeo::LayeredTexture* t )
{
    if ( t==_texture )
	return;
    
    _texture = t;
    
    _updatecount++;
}


void RandomLineNode::setPath( const osg::Vec2Array& coords )
{
    *_pathcoords = coords;
    
    _updatecount++;
}


void RandomLineNode::setTextureMapping( const osg::UIntArray& indices )
{
    *_pathindices = indices;
    
    _updatecount++;
}


void RandomLineNode::setZRange( float top, float bottom )
{
    _top = top;
    _bottom = bottom;
    
    _updatecount++;
}


void RandomLineNode::traverse( osg::NodeVisitor& nv )
{
    if ( _lastgeomupdatecount<_updatecount )
    {
	if ( updateGeometry() )
	    _lastgeomupdatecount = _updatecount;
    }
    
    _geode->accept( nv );
}


bool RandomLineNode::updateGeometry()
{
    //int nrknots = osg::minimum( _pathindices->size(), _pathcoords->size() );
    
    //TODO port code from visBase and create tiles with own stateset and part
    //of the texture.
    
    return true;
}


