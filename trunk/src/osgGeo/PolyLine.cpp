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


#include <osgGeo/PolyLine>

#include <osg/Geometry>
#include <osgUtil/CullVisitor>

namespace osgGeo
{

PolylineNode::PolylineNode()
    : radius_( 5 )
    , maxRadius_( -1 )
    , screenSizeScaling_( false )
    , arrayModifiedCount_( 0 )
{}


PolylineNode::PolylineNode( const PolylineNode& node, const osg::CopyOp& co )
    : osg::Node( node, co )
    , radius_( node.radius_ )
    , maxRadius_( node.maxRadius_ )
    , screenSizeScaling_( node.screenSizeScaling_ )
    , array_( node.array_ )
    , geometry_( (osg::Geometry*) node.geometry_->clone(co) )
    , arrayModifiedCount_( 0 )
{}


PolylineNode::~PolylineNode()
{}


void PolylineNode::traverse( osg::NodeVisitor& nv )
{
    if ( nv.getVisitorType()==osg::NodeVisitor::UPDATE_VISITOR )
    {
	if ( array_.valid() )
	{
	    if ( !arrayModifiedCount_ ||
		 arrayModifiedCount_!=array_->getModifiedCount() )
	    {
		createGeometry();
	    }
	}
    }
    else if ( nv.getVisitorType()==osg::NodeVisitor::CULL_VISITOR )
    {
	if ( geometry_.valid() )
	{
	    osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*>(&nv);

	    if ( getStateSet() )
		cv->pushStateSet( getStateSet() );

	    cv->addDrawable( geometry_, cv->getModelViewMatrix() );

	    if ( getStateSet() )
		cv->popStateSet();
	}
    }
}


void PolylineNode::createGeometry()
{}


}
