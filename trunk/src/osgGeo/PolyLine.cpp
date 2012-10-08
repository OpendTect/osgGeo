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

#include <osgGeo/Line3>

#include <osg/Geometry>
#include <osgUtil/CullVisitor>


namespace osgGeo
{

PolyLineNode::PolyLineNode()
    : _needsUpdate( true )
    , _radius(5)
    , _maxRadius(-1)
    , _screenSizeScaling(false)
    , _geometry( new osg::Geometry )
    , _arrayModifiedCount(0)
    , _resolution(4)
{
    setNumChildrenRequiringUpdateTraversal( 1 );
}


PolyLineNode::PolyLineNode( const PolyLineNode& node, const osg::CopyOp& co )
    : osg::Node(node,co)
    , _array(node._array)
    , _radius(node._radius)
    , _maxRadius(node._maxRadius)
    , _screenSizeScaling(node._screenSizeScaling)
    , _geometry((osg::Geometry*)node._geometry->clone(co))
    , _arrayModifiedCount(0)
    , _resolution(4)
{
    setNumChildrenRequiringUpdateTraversal( 1 );
}


PolyLineNode::~PolyLineNode()
{
}


void PolyLineNode::traverse( osg::NodeVisitor& nv )
{
    if ( nv.getVisitorType()==osg::NodeVisitor::UPDATE_VISITOR )
    {
	if ( needsUpdate() )
	    updateGeometry();
    }
    else if ( nv.getVisitorType()==osg::NodeVisitor::CULL_VISITOR )
    {
	osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*>(&nv);

	if ( getStateSet() )
	    cv->pushStateSet( getStateSet() );

	    cv->addDrawable( _geometry, cv->getModelViewMatrix() );

	if ( getStateSet() )
	    cv->popStateSet();
    }
}

#define mMid(t) \
    ( (max##t + min##t)/2 )

#define mMax(t) \
    max##t = t >= max##t ? t : max##t;

#define mMin(t) \
    min##t = t > min##t ? min##t : t;\

osg::BoundingSphere PolyLineNode::computeBound() const
{
    osg::Vec3Array* arr = dynamic_cast<osg::Vec3Array*>( _array.get());
    osg::BoundingSphere bs;
    if ( !arr )
	return bs;
    
    float maxx=0, maxy=0, maxz=0, minx=0, miny=0, minz=0;
    osg::Vec3 p = arr->at( 0 );
    maxx = minx = p.x();
    maxy = miny = p.y();
    maxz = minz = p.z();
	
    for ( unsigned int pidx=0; pidx<arr->size(); pidx++ )
    {
	osg::Vec3 p = arr->at( pidx );
	float x = p.x();
	float y = p.y();
	float z = p.z();
	mMax(x)
	mMax(y)
	mMax(z)
	
	mMin(x)
	mMin(y)
	mMin(z)
    }

    const osg::Vec3 maxvec( maxx, maxy, maxz );
    const osg::Vec3 minvec( minx, miny, minz );
    const osg::Vec3 resvec = maxvec - minvec;
    const float len = resvec.length();
    bs._radius = len / 2;
    bs._center = osg::Vec3( mMid(x), mMid(y), mMid(z) );
    return bs;
}


void PolyLineNode::setVertexArray( osg::Array* arr )
{
    _array = arr;
}


void PolyLineNode::setRadius( const float& rad )
{
    _radius = rad;
}


void PolyLineNode::setColor( const osg::Vec4& color )
{
    _color = color;
}


void PolyLineNode::getOrthoVecs( const osg::Vec3& w, osg::Vec3& u, osg::Vec3& v ) const
{
    float factor = 0;
    if ( w.x() > w.y() )
    {
	factor = 1/sqrt( w.x()*w.x() + w.y()*w.y() );
	float u0 = -w.z() * factor;
	float u1 = 0;
	float u2 = w.x()*factor;
	u.set( u0, u1, u2 );
    }
    else
    {
	factor = 1/sqrt( w.y()*w.y() + w.z()*w.z() );
	float u0 = 0;
	float u1 = w.z() * factor;
	float u2 = -w.y()* factor;
	u.set( u0, u1, u2 );
    }
    u.normalize();
    v = w ^ u;
    v.normalize();
}


#define mAddVertex(vec,pos)\
    coords->push_back( vec ); \
    norm = vec - pos; \
    norm.normalize();\
    normals->push_back( norm ); \

bool PolyLineNode::updateGeometry()
{
     osg::Vec3Array* arr = dynamic_cast<osg::Vec3Array*>(_array.get());
     if ( !arr || !_needsUpdate )
	 return false;

    osg::ref_ptr<osg::Vec3Array> coords = new osg::Vec3Array;
    osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array;
   
    const unsigned int arrsize = arr->size();
    osg::Vec3* corners1 = new osg::Vec3[_resolution];
    osg::Vec3* corners2 = new osg::Vec3[_resolution];
    for ( unsigned int pidx=0; pidx<arrsize; pidx++ )
    {
	osg::Vec3 p0;
	osg::Vec3 p1 = arr->at( pidx );
	osg::Vec3 p2;
	osg::Vec3 vec01;
	osg::Vec3 vec12;
	osg::Vec3 planenormal;
	bool doreverse = false;
	
	if ( !pidx )
	{
	    p0 = arr->at( pidx );
	    p1 = arr->at( pidx+1 );
	    p2 = arr->at( arrsize >= 3 ? pidx+2 : pidx+1 );
	    vec01 = p1 - p0; vec01.normalize();
	    vec12 = p2 - p1; vec12.normalize();
	    planenormal = vec01 + vec12;
	    osg::Vec3 curu,curv;
	    getOrthoVecs( vec01, curu, curv );
	    for ( int idx=0; idx<_resolution; idx++ )
	    {
		float angl = idx * 2 * M_PI / _resolution;
		const osg::Vec3 vec1 = ( curu * cos(angl) ) + ( curv * sin(angl) );
		corners1[idx] = vec1*_radius + p0;
	    }
	}
	else if ( pidx == arr->size()-1 )
	{
	    p0 = arr->at( pidx-1 );
	    planenormal = p1 - p0;
	    planenormal.normalize();
	    vec01 = planenormal;
	}
	else
	{
	    p0 = arr->at( pidx-1 );
	    p2 = arr->at( pidx+1 );
	    vec01 = p1-p0; vec01.normalize();
	    vec12 = p2-p1; vec12.normalize();
	    doreverse = vec01 * vec12 < -0.5f;
	    planenormal = !doreverse ? vec01 + vec12 : vec12 - vec01;
	}
	
	osg::Vec3 norm;
	const osg::Plane plane( planenormal, p1 );
	for ( int idx=0; idx<_resolution; idx++ )
	{
	    const osgGeo::Line3 lineproj( corners1[idx], vec01 );
	    corners2[idx] = lineproj.getInterSectionPoint( plane );
	    mAddVertex( corners1[idx], p0 )
	    mAddVertex( corners2[idx], p1 )
	}

	mAddVertex( corners1[0], p0 )
	mAddVertex( corners2[0], p1 )

	if ( doreverse )
	{
	    norm = -planenormal;
	    norm.normalize();
	    for ( int idx=0; idx<_resolution; idx++ )
	    {
		coords->push_back( corners2[idx] );
		normals->push_back( norm*0.5 );
		coords->push_back( p1 );
		normals->push_back( norm*0.5 );
	    }
	    
	   coords->push_back( corners2[0] );
	   normals->push_back( norm*0.5 );
	}
	
	for ( int idx=0; idx<_resolution; idx++ )
	    corners1[idx] = corners2[idx];
    }

    delete[] corners1;
    delete[] corners2;

    _geometry->setVertexArray( coords );
    _geometry->addPrimitiveSet( new osg::DrawArrays( GL_TRIANGLE_STRIP, 0,
			       coords->size(), 0 ));
    _geometry->setNormalArray( normals.get() );
    _geometry->setNormalBinding( osg::Geometry::BIND_PER_VERTEX );
    osg::Vec4Array* colors = new osg::Vec4Array;
    colors->push_back( _color );
    _geometry->setColorArray( colors );
    _geometry->setColorBinding(osg::Geometry::BIND_OVERALL); 
    _needsUpdate = false;
    return true;
}


bool PolyLineNode::needsUpdate() const
{
    return _needsUpdate;
}

} //namespace osgGeo
