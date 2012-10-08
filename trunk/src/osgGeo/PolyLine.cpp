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
    : _radius( 5 )
    , _maxRadius( -1 )
    , _screenSizeScaling( false )
    , _arrayModifiedCount( 0 )
{}


PolylineNode::PolylineNode( const PolylineNode& node, const osg::CopyOp& co )
    : osg::Node( node, co )
    , _radius( node._radius )
    , _maxRadius( node._maxRadius )
    , _screenSizeScaling( node._screenSizeScaling )
    , _array( node._array )
    , _geometry( (osg::Geometry*) node._geometry->clone(co) )
    , _arrayModifiedCount( 0 )
{}


PolylineNode::~PolylineNode()
{}


void PolylineNode::traverse( osg::NodeVisitor& nv )
{
    if ( nv.getVisitorType()==osg::NodeVisitor::UPDATE_VISITOR )
    {
	if ( _array.valid() )
	{
	    if ( !_arrayModifiedCount ||
		 _arrayModifiedCount!=_array->getModifiedCount() )
	    {
		createGeometry();
	    }
	}
    }
    else if ( nv.getVisitorType()==osg::NodeVisitor::CULL_VISITOR )
    {
	if ( _geometry.valid() )
	{
	    osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*>(&nv);

	    if ( getStateSet() )
		cv->pushStateSet( getStateSet() );

	    cv->addDrawable( _geometry, cv->getModelViewMatrix() );

	    if ( getStateSet() )
		cv->popStateSet();
	}
    }
}


void PolylineNode::createGeometry()
{}


}
