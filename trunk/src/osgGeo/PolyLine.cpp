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
    : _needsUpdate(true)
    , _radius(5)
    , _maxRadius(-1)
    , _screenSizeScaling(false)
    , _geometry(new osg::Geometry)
    , _arrayModifiedCount(0)
    , _resolution(4)
    , _arraymodcount(0)
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
    , _arraymodcount(0)
    , _needsUpdate(true)
{
    setNumChildrenRequiringUpdateTraversal( 1 );
}


PolyLineNode::~PolyLineNode()
{
}


void PolyLineNode::traverse( osg::NodeVisitor& nv )
{
    if ( nv.getVisitorType()==osg::NodeVisitor::CULL_VISITOR )
    {
	osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*>(&nv);
	if ( needsUpdate(cv) )
	    updateGeometry( cv );

	if ( getStateSet() )
	    cv->pushStateSet( getStateSet() );

	cv->addDrawable( _geometry, cv->getModelViewMatrix() );

	if ( getStateSet() )
	    cv->popStateSet();
    }
}


osg::BoundingSphere PolyLineNode::computeBound() const
{
   return _bs;
}


void PolyLineNode::setVertexArray( osg::Array* arr )
{
    _array = arr;
    _needsUpdate = true;
}


void PolyLineNode::setRadius( const float& rad )
{
    _radius = rad;
    _needsUpdate = true;
}


void PolyLineNode::setResolution( int res )
{
    _resolution = res;
    _needsUpdate = true;
}


void PolyLineNode::getOrthoVecs( const osg::Vec3& w, osg::Vec3& u, osg::Vec3& v ) const
{
    float factor = 0;
    if ( w.x() > w.y() )
    {
	factor = 1/sqrt( w.x()*w.x() + w.y()*w.y() );
	float u0 = -w.z() * factor ;
	float u1 = 0;
	float u2 = w.x()*factor;
	u.set( u0, u1, u2 );
    }
    else
    {
	factor = 1/sqrt( w.y()*w.y() + w.z()*w.z() );
	float u0 = 0;
	float u1 = w.z() * factor ;
	float u2 = -w.y()* factor;
	u.set( u0, u1, u2 );
    }
    
    u.normalize();
    v = w ^ u;
    v.normalize();
}


int getMaxIndex( const osg::PrimitiveSet* ps )
{
    int max = 0;
    for ( unsigned int idx=0; idx<ps->getNumIndices(); idx++ )
    {
	const int val = (int)ps->index( idx );
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

#define mAddCap( croner, p ) \
	   	coords->push_back( croner[idx] ); \
		triindices->push_back( ci++ ); \
		normals->push_back( norm ); \
		coords->push_back( p ); triindices->push_back( ci++ ); \
		normals->push_back( norm ); \

bool PolyLineNode::updateGeometry( const osg::CullStack* )
{
    osg::Vec3Array* arr = dynamic_cast<osg::Vec3Array*>(_array.get());
    if ( !arr )
	 return false;
    _geometry->getPrimitiveSetList().clear();
    osg::BoundingBox bbox;
    int ci = 0;
    osg::ref_ptr<osg::Vec3Array> coords = new osg::Vec3Array;
    osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array;
    const unsigned int primsz = _primitivesets.size();
    for ( unsigned int primidx=0; primidx<primsz; primidx++ )
    {
	osg::Vec3* corners1 = new osg::Vec3[_resolution];
	osg::Vec3* corners2 = new osg::Vec3[_resolution];
	const osg::PrimitiveSet* ps = _primitivesets.at( primidx );
	const int maxprimsz = getMaxIndex( ps );
	bool doonce = true;
	osg::DrawElementsUInt* triindices =
		new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLE_STRIP, 0);
	for ( unsigned int cidx=0; cidx<ps->getNumIndices(); cidx++ )
	{
	    const int pidx = (int) ps->index( cidx );
	    const osg::Vec3  p0 = arr->at( pidx );
	    const osg::Vec3  p1 = arr->at( pidx <= maxprimsz-1 ? pidx+1 : pidx );
	    const osg::Vec3  p2 = arr->at( pidx <= maxprimsz-2 ? pidx+2 : pidx );
	    osg::Vec3 vec01 = p1 - p0; vec01.normalize();
	    osg::Vec3 vec12 = p2 - p1; vec12.normalize();
	    const bool doreverse = vec01 * vec12 < -0.5f;
	    const osg::Vec3 planenormal = 
			doreverse ? vec12 - vec01 : vec01 + vec12;
	    osg::Vec3 norm = -planenormal;
	    norm.normalize();
	    if ( doonce )
	    {
		osg::Vec3 curu,curv;
		getOrthoVecs( vec01, curu, curv );
		for ( int idx=0; idx<_resolution; idx++ )
		{
		    float angl = idx * 2 * M_PI / _resolution;
		    const osg::Vec3 vec1 = curu*cos(angl) + curv*sin(angl);
		    corners1[idx] = vec1*_radius + p0;
		    mAddCap( corners1, p0 );
		}

		coords->push_back( corners1[0] );
		triindices->push_back( ci++ );
		normals->push_back( norm );
		doonce = false;
	    }

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
		    mAddCap( corners2, p1 );
		}
	    
		coords->push_back( corners2[0] );
	        triindices->push_back( ci++ );
		normals->push_back( norm );
	    }
	
	    for ( int idx=0; idx<_resolution; idx++ )
		corners1[idx] = corners2[idx];
	}

	delete[] corners1;
	delete[] corners2;
   
	_geometry->addPrimitiveSet( triindices );
    }
   
    _geometry->setVertexArray( coords.get() );
    _geometry->setNormalArray( normals.get() );
    _geometry->setNormalBinding( osg::Geometry::BIND_PER_VERTEX );
    _geometry->setColorBinding(osg::Geometry::BIND_OVERALL); 
    _needsUpdate = false;
    _arraymodcount = _array->getModifiedCount();
    
    _bs = bbox;

    dirtyBound();
    return true;
}


bool PolyLineNode::needsUpdate( const osg::CullStack* ) const
{
    if ( _needsUpdate || _arraymodcount!=_array->getModifiedCount() )
	return true;

    for ( int idx=_primitivesets.size()-1; idx>=0; idx-- )
    {
	if ( _primitivesets[idx]->getModifiedCount()!=
	     _primitivesetmodcount[idx] )
	{
	    return true;
	}
    }

    return false;
}


void PolyLineNode::addPrimitiveSet( osg::PrimitiveSet* ps )
{
    _primitivesetmodcount.push_back( ps->getModifiedCount() );
    _primitivesets.push_back( ps );
    _needsUpdate = true;
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
    _primitivesetmodcount.erase( _primitivesetmodcount.begin()+idx );
    _primitivesets.erase( _primitivesets.begin()+idx );
    _needsUpdate = true;
}


} //namespace osgGeo
