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


#include "PolyLine"

#include <osg/Geometry>
#include <osgUtil/CullVisitor>

namespace osgGeo
{

PolylineNode::PolylineNode()
    : _radius( 5 )
    , _maxRadius( -1 )
    , _screenSizeScaling( false )
    , _arrayModifiedCount( 0 )
    , _geometry(new osg::Geometry)
{}


PolylineNode::PolylineNode( const PolylineNode& node, const osg::CopyOp& co )
    : osg::Node( node, co )
    , _array( node._array )
    , _radius( node._radius )
    , _maxRadius( node._maxRadius )
    , _screenSizeScaling( node._screenSizeScaling )
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
	    osgUtil::CullVisitor* cv = static_cast<osgUtil::CullVisitor*>(&nv);

	    if ( getStateSet() )
		cv->pushStateSet( getStateSet() );

	    cv->addDrawable( _geometry, cv->getModelViewMatrix() );

	    if ( getStateSet() )
		cv->popStateSet();
	}
    }
    else
    {
	createGeometry();
    }
}


osg::BoundingSphere PolylineNode::computeBound() const
{
    osg::Vec3Array* newarr = dynamic_cast<osg::Vec3Array*>(_array.get());
    osg::BoundingSphere bb;
    if ( !newarr )
	return bb;
    const int sz = newarr->size();
    bb._center =  newarr->at( sz/2 );
    osg::Vec3 start = newarr->at( 0 );
    osg::Vec3 stop = newarr->at( sz-1 );
    const float xdiff = stop[0] - start[0];
    const float ydiff = stop[1] - start[1];
    const float zdiff = stop[2] - start[2];
    const float length = sqrt( xdiff*xdiff + ydiff*ydiff + zdiff*zdiff );
    bb._radius = length;
    return bb;
}


void PolylineNode::setVertexArray( osg::Array* arr )
{
    _array = arr;
}


void PolylineNode::setRadius( float rad )
{
    _radius = rad;
}


void PolylineNode::setColor( osg::Vec4 color )
{
    _color = color;
}

#define mAddVertex(vec,nrm,pos)\
    normals->push_back( nrm ); \
    coords->push_back( vec + pos ); \

void PolylineNode::createGeometry()
{
    osg::Vec3Array* newarr = dynamic_cast<osg::Vec3Array*>(_array.get());
    if ( !newarr )
	return;

    const int resolution = 10;
    osg::ref_ptr<osg::Vec3Array> coords = new osg::Vec3Array;
    osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array;
    osg::ref_ptr<osg::Vec3Array> dirvecs = new osg::Vec3Array;

    for ( int pidx=0; pidx<newarr->size()-1; pidx++ )
    {
	osg::Vec3 start = newarr->at( pidx );
	osg::Vec3 stop = newarr->at( pidx+1 );
	osg::Vec3 dirvec = stop - start;
	dirvec.normalize();
	dirvecs->push_back( dirvec );
	
	for ( int idx=0; idx<=resolution; idx++ )
	{
	    float angl = idx * 2 * M_PI / resolution;
	    osg::Vec3 dir( sin(angl), cos(angl), 0 ); 
	    osg::Vec3 vec1 = ( (pidx >= 1 ? dirvecs->at(pidx-1)
					  : dirvec ) ^ dir ) * _radius;
	    osg::Vec3 vec2 = ( dirvec ^ dir ) * _radius;
	    osg::Vec3 norm;
	    norm = vec1 / _radius;
	    norm.normalize();
	    mAddVertex( vec1, norm, start )
	    mAddVertex( vec2, norm, stop )
	}
    
    }
    
    _geometry->setVertexArray( coords );
    _geometry->addPrimitiveSet( new osg::DrawArrays( GL_TRIANGLE_STRIP, 0,
			       coords->size(), 0 ));
    _geometry->setNormalArray( normals.get() );
    _geometry->setNormalBinding( osg::Geometry::BIND_PER_VERTEX );
    osg::Vec4Array* colors = new osg::Vec4Array;
    colors->push_back( _color );
    _geometry->setColorArray( colors );
    _geometry->setColorBinding(osg::Geometry::BIND_OVERALL); 
     osg::StateSet* state = _geometry->getOrCreateStateSet();
     state->setMode( GL_CULL_FACE, osg::StateAttribute::ON );
}


}
