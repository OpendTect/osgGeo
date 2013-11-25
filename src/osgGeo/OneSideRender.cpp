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


#include <osgGeo/OneSideRender>
#include <osgUtil/CullVisitor>

osgGeo::OneSideRenderNode::OneSideRenderNode()
{}


osgGeo::OneSideRenderNode::OneSideRenderNode( const OneSideRenderNode& b,
					      const osg::CopyOp& op )
    : osg::Node( b, op )
{
    for ( DrawableList::const_iterator itr=b._drawables.begin();
	  itr!=b._drawables.end(); ++itr)
    {
	osg::Drawable* drawable = op( itr->get() );
	if ( drawable ) addDrawable( drawable );
    }

    _lines = b._lines;
}


bool osgGeo::OneSideRenderNode::addDrawable( osg::Drawable* gset )
{
    return addDrawable( gset, Line3(osg::Vec3(0,0,0), osg::Vec3(0,0,1)) );
}


bool osgGeo::OneSideRenderNode::addDrawable( osg::Drawable* gset,
					     const Line3& line )
{
    _drawables.push_back( gset );
    // Precalculate bounding sphere for (multi-threaded) cull traversal
    gset->getBound();

    _lines.push_back( line );
    return true;
}


bool osgGeo::OneSideRenderNode::removeDrawable( osg::Drawable* gset )
{
    DrawableList::iterator drawable = std::find( _drawables.begin(), _drawables.end(), gset);

    const int idx = drawable - _drawables.begin();
    if ( idx<0 )
	return false;

    _drawables.erase( drawable );
    _lines.erase( _lines.begin()+idx );

    return true;
}

void osgGeo::OneSideRenderNode::traverse( osg::NodeVisitor& nv )
{
    if ( nv.getVisitorType()==osg::NodeVisitor::CULL_VISITOR )
    {
	osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*>(&nv);

	if ( getStateSet() )
	    cv->pushStateSet( getStateSet() );

	const osg::Vec3 eye = cv->getEyePoint();
	for ( unsigned int idx=0; idx<_drawables.size(); idx++ )
	{
	    const osg::Vec3 viewline = eye-_lines[idx]._pos;
	    if ( viewline*_lines[idx]._dir>=0 )
	    {
		const osg::BoundingBox bb = _drawables[idx]->getBound();
		const float depth = cv->getDistanceFromEyePoint( bb.center(), false );
		cv->addDrawableAndDepth( _drawables[idx], cv->getModelViewMatrix(), depth );
	    }
	}

	if ( getStateSet() )
	    cv->popStateSet();
    }
    else
	osg::Node::traverse( nv );
}


void osgGeo::OneSideRenderNode::setLine(unsigned int idx,const Line3& line )
{
    if ( idx<_lines.size() )
	_lines[idx] = line;
}
