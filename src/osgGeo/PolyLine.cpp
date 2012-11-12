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
	const float x = p.x();
	const float y = p.y();
	const float z = p.z();
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


unsigned int getMaxIndex( const osg::DrawElementsUInt* indices )
{
    unsigned int max = 0;
    for ( int idx=0; idx<indices->size(); idx++ )
    {
	const unsigned val = indices->at( idx );
    	max = max > val ? max : val;
    }
    
    return max;
}


#define mAddVertex(vec,pos)\
    coords->push_back( vec ); \
    norm = vec - pos; \
    norm.normalize();\
    normals->push_back( norm ); \
    triindices->push_back( ci++ );

bool PolyLineNode::updateGeometry()
{
     osg::Vec3Array* arr = dynamic_cast<osg::Vec3Array*>(_array.get());
     if ( !arr || !_needsUpdate )
	 return false;

    osg::ref_ptr<osg::Vec3Array> coords = new osg::Vec3Array;
    osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array;
   
    const unsigned int primsz = _primitivesets.size();
    int ci = 0;
    for ( unsigned int primidx=0; primidx<primsz; primidx++ )
    {
	osg::Vec3* corners1 = new osg::Vec3[_resolution];
	osg::Vec3* corners2 = new osg::Vec3[_resolution];
	const osg::PrimitiveSet* ps = _primitivesets.at( primidx );
	const osg::DrawElementsUInt* indices = 
	    dynamic_cast<const osg::DrawElementsUInt*>( ps );
	if ( !indices )
	    continue;
	const unsigned int maxprimsz = getMaxIndex( indices );
	bool doonce = true;
	osg::DrawElementsUInt* triindices =
		new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLE_STRIP, 0);
	for ( unsigned int cidx=0; cidx<indices->size(); cidx++ )
	{
	    const unsigned int pidx = indices->at( cidx );
	    const osg::Vec3  p0 = arr->at( pidx );
	    const osg::Vec3  p1 = arr->at( pidx < maxprimsz-1 ? pidx+1 : pidx );
	    const osg::Vec3  p2 = arr->at( pidx < maxprimsz-2 ? pidx+2 : pidx );
	    osg::Vec3 vec01 = p1 - p0; vec01.normalize();
	    osg::Vec3 vec12 = p2 - p1; vec12.normalize();
	    const bool doreverse = vec01 * vec12 < -0.5f;
	    const osg::Vec3 planenormal =
		doreverse ? vec12 - vec01 : vec01 + vec12;
	   
	    if ( doonce )
	    {
		osg::Vec3 curu,curv;
		getOrthoVecs( vec01, curu, curv );
		for ( int idx=0; idx<_resolution; idx++ )
		{
		    float angl = idx * 2 * M_PI / _resolution;
		    const osg::Vec3 vec1 = ( curu * cos(angl) ) + ( curv * sin(angl) );
		    corners1[idx] = vec1*_radius + p0;
		}

		doonce = false;
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
		    triindices->push_back( ci++ );
		    normals->push_back( norm*0.5 );
		    coords->push_back( p1 ); 
		    triindices->push_back( ci++ );
		    normals->push_back( norm*0.5 );
		}
	    
	       coords->push_back( corners2[0] ); 
	       triindices->push_back( ci++ );
	       normals->push_back( norm*0.5 );
	    }
	
	    for ( int idx=0; idx<_resolution; idx++ )
		corners1[idx] = corners2[idx];
	}

	delete[] corners1;
	delete[] corners2;
   
	_geometry->addPrimitiveSet( triindices );
    }
   
    _geometry->setVertexArray( coords );
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


void PolyLineNode::addPrimitiveSet( osg::PrimitiveSet* ps )
{
    _primitivesets.push_back( ps );
}


int PolyLineNode::getPrimitiveSetIndex( const osg::PrimitiveSet* ps ) const
{
    std::vector<osg::ref_ptr<osg::PrimitiveSet> >::const_iterator it =
	std::find( _primitivesets.begin(), _primitivesets.end(), ps );

    if ( it==_primitivesets.end() )
	return -1;

    return it-_primitivesets.begin();
}


void PolyLineNode::removePrimitiveSet( int idx )
{
    _primitivesets.erase( _primitivesets.begin()+idx );
}


} //namespace osgGeo
