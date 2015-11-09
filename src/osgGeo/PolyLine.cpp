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
#include <osgGeo/ComputeBoundsVisitor>
#include <osg/Geometry>
#include <osgUtil/CullVisitor>
#include <osgUtil/IntersectionVisitor>
#include <osg/Version>


namespace osgGeo
{

bool getAStripIndexes(const osg::PrimitiveSet* ps,osg::PrimitiveSet* stripidxarr,
			     unsigned int& endidx,unsigned int startidx)
{
    if (!ps) return 0;
    if (startidx >= ps->getNumIndices())
	return false;
    if (ps->getMode() != osg::PrimitiveSet::LINES)
    {
	endidx = ps->getNumIndices();
	return true;
    }

    endidx = startidx;
    stripidxarr->getDrawElements()->addElement(ps->index(startidx));
    for (unsigned int idx = startidx; idx < ps->getNumIndices()-2; idx++)
    {
	if (ps->index(idx+1) == ps->index(idx+2))
	{
	    stripidxarr->getDrawElements()->addElement(ps->index(idx+1));
	    endidx = startidx + idx + 2;
	    idx++;
	}
	else
	{
	    stripidxarr->getDrawElements()->addElement(ps->index(idx+1));
	    endidx = idx+1;
	    return true;
	}
    }
    endidx = ps->getNumIndices();
    stripidxarr->getDrawElements()->addElement(ps->index(ps->getNumIndices()-1));
    if (endidx == startidx)
	return false;
    return true;
}


PolyLineNode::PolyLineNode()
    : _needsUpdate(true)
    , _radius(5)
    , _screenSizeScaling(true)
    , _geom3DCoords(new osg::Vec3Array)
    , _geom3DNormals(new osg::Vec3Array)
    , _geometry(new osg::Geometry)
    , _arrayModifiedCount(0)
    , _resolution(4)
    , _polyLineCoords(0)
{
    setNumChildrenRequiringUpdateTraversal(1);
    _geometry->setVertexArray(_geom3DCoords);
    _geometry->setNormalArray(_geom3DNormals);
    _geometry->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);
}


PolyLineNode::PolyLineNode(const PolyLineNode& node, const osg::CopyOp& co)
    : osg::Node(node,co)
    , _polyLineCoords(node._polyLineCoords)
    , _radius(node._radius)
    , _screenSizeScaling(node._screenSizeScaling)
    , _geom3DCoords(node._geom3DCoords)
    , _geom3DNormals(node._geom3DNormals)
    , _geometry((osg::Geometry*)node._geometry->clone(co))
    , _arrayModifiedCount(0)
    , _resolution(4)
    , _needsUpdate(true)
{
   setNumChildrenRequiringUpdateTraversal(1);
   _geometry->setVertexArray(_geom3DCoords);
   _geometry->setNormalArray(_geom3DNormals);
   _geometry->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);
}


PolyLineNode::~PolyLineNode()
{
}


void PolyLineNode::traverse(osg::NodeVisitor& nv)
{
    if (nv.getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR)
    {
	if ( needsUpdate() )
	    updateGeometry();
    }
    else if (nv.getVisitorType() == osg::NodeVisitor::CULL_VISITOR)
    {
	osgUtil::CullVisitor* cv = static_cast<osgUtil::CullVisitor*>(&nv);
	if (_screenSizeScaling && (_isGeometryChanged || isCameraChanged(cv)))
	{
	     reScaleCoordinates(cv);
	    _isGeometryChanged = false;
	}
		
	if ( getStateSet() )
	    cv->pushStateSet(getStateSet());

	const float depth = cv->getDistanceFromEyePoint( _bbox.center(), false );
	cv->addDrawableAndDepth( _geometry, cv->getModelViewMatrix(), depth );

	if ( getStateSet() )
	    cv->popStateSet();

#if OSG_MIN_VERSION_REQUIRED(3,3,2)
	if ( _geometry->getBoundingBox().valid() )
	    cv->updateCalculatedNearFar(*cv->getModelViewMatrix(), _geometry->getBoundingBox() );
#else
	if ( _geometry->getBound().valid() )
	    cv->updateCalculatedNearFar(*cv->getModelViewMatrix(), _geometry->getBound() );
#endif
    }
    else
    {
	osgUtil::IntersectionVisitor* iv =
	    		dynamic_cast<osgUtil::IntersectionVisitor*>( &nv );
	if ( iv )
	{
	    osgUtil::Intersector* intersec = iv->getIntersector()->clone( *iv );
	    if ( intersec )
		intersec->intersect( *iv, _geometry );

	    return;
	}

	osgGeo::ComputeBoundsVisitor* cbv =
	    dynamic_cast<osgGeo::ComputeBoundsVisitor*>( &nv );
	if ( cbv )
	    cbv->applyBoundingBox(_bbox);

    }
}


void PolyLineNode::touch()
{
    setUpdateFlag(true);
}


void PolyLineNode::setUpdateFlag(bool updt)
{
    _needsUpdate = updt;
    if ( _needsUpdate )
	setNumChildrenRequiringUpdateTraversal(1);
    else
	setNumChildrenRequiringUpdateTraversal(0);
}


bool PolyLineNode::isCameraChanged(const osgUtil::CullVisitor* cv)
{
    if( !cv )
	return false;

    const double eps = 0.002f;
#define mIsNotEqual(v1,v2) \
	    (fabs(v1-v2) > eps)

    
    bool isChanged = false;
    const float pixsz = fabs( cv->pixelSize(_bbox.center(),1.0f) );

    if ( mIsNotEqual(_pixsz,pixsz) )
    {
	_pixsz = pixsz;
	isChanged = true;
    }
   
    return isChanged;
}


void PolyLineNode::reScaleCoordinates(const osgUtil::CullVisitor* cv)
{
    if ( !cv )
	return;

    for ( unsigned int idx=0; idx<_geom3DCoords->size(); idx++)
    {
	if ( _capflags[idx] )
	    continue;

	const osg::Vec3& coord = _unScaledGeomCoords->at(idx);
	const float factor = cv->pixelSize(coord,1.0f);
	    (*_geom3DCoords )[idx] = coord + _geom3DNormals->at(idx)*_radius/factor;
    }
    _geometry->dirtyDisplayList();
 }


osg::BoundingSphere PolyLineNode::computeBound() const
{
    if (_bbox.valid())
        return _bbox;
    
    osg::BoundingBox bbox;

    for (unsigned int idx=0; idx<_polyLineCoords->size(); idx++)
	bbox.expandBy((*_polyLineCoords)[idx]); 

    return bbox;

}


void PolyLineNode::setVertexArray(osg::Array* arr)
{
    _polyLineCoords = (osg::Vec3Array*) arr;
    touch();
}


void PolyLineNode::setRadius(float rad)
{
    _radius = rad;
    touch();
}


void PolyLineNode::setResolution(int res)
{
    _resolution = res;
    touch();
}


void PolyLineNode::useAutoScreenScaling(bool yn)
{
    _screenSizeScaling = yn;
}


void PolyLineNode::getOrthoVecs(const osg::Vec3& w,osg::Vec3& u,osg::Vec3& v) const
{
    float factor = 0;
    if (w.x() > w.y())
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


void PolyLineNode::clearAll()
{
    _geometry->getPrimitiveSetList().clear();
    _geom3DCoords ->clear();
    _geom3DNormals->clear();
    _capflags.clear();
    _bbox.init();
}


#define mAddVertex(vec,pos)\
    _geom3DCoords ->push_back(vec); \
    _capflags.push_back(false);\
    { \
        osg::Vec3 norm = vec - pos; \
        norm.normalize();\
        _geom3DNormals->push_back(norm); \
     } \
    _bbox.expandBy(pos)

bool PolyLineNode::updateGeometry()
{
    if (!_polyLineCoords)
	return false;
    
    clearAll();
    const unsigned int primsz = _primitiveSets.size();
    for (unsigned int primidx=0;primidx<primsz;primidx++)
    {
	const osg::PrimitiveSet* ps = _primitiveSets.at(primidx);
	const unsigned int pssz = ps->getNumIndices();
	unsigned int endidx = 0;
	for (unsigned int cidx=0; cidx<pssz; cidx++)
	{
	    osg::ref_ptr<osg::PrimitiveSet> stripidxarr = 0;
	    if (ps->getMode() == osg::PrimitiveSet::LINES)
		stripidxarr = new osg::DrawElementsUInt;
	    else
		stripidxarr = const_cast< osg::PrimitiveSet* >(ps);

	    if (cidx==endidx)
	    {
		const unsigned int startidx = cidx==0 ? cidx : cidx+1;
		if (!getAStripIndexes(ps,stripidxarr,endidx,startidx))
		    continue;
	    }

	    osg::ref_ptr<osg::Vec3Array> corners1 = new osg::Vec3Array(_resolution);
	    osg::ref_ptr<osg::Vec3Array> corners2 = new osg::Vec3Array(_resolution);

	    bool first = true;
	    const int originalsize = _geom3DCoords->size();
	    const unsigned int sz = stripidxarr->getNumIndices();
	    for (unsigned int stripidx=0; stripidx<sz-1; stripidx++)
	    {
		const unsigned int pidx0 = stripidxarr->index(stripidx);
		const unsigned int pidx1 = stripidxarr->index(stripidx+1);
		const unsigned int pidx2 = stripidxarr->index(stripidx<sz-2 ? stripidx+2 : stripidx+1);
		
		if (buildA3DLineStrip(*corners1,*corners2,pidx0,pidx1,pidx2,first,stripidx==sz-2))
		    first = false;
	    }

	    const int endsize = _geom3DCoords->size();
	    if (originalsize!=endsize)
		_geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::TRIANGLE_STRIP,originalsize,endsize-originalsize));
	    cidx = endidx-1;
	}
    }

    _unScaledGeomCoords = (osg::Vec3Array*) _geom3DCoords ->clone(osg::CopyOp::SHALLOW_COPY);
    _arrayModifiedCount = _polyLineCoords->getModifiedCount();

    dirtyBound();
    _isGeometryChanged = true;
    setUpdateFlag(false);
    return true;
}


bool PolyLineNode::buildA3DLineStrip(osg::Vec3Array& corners1,
	osg::Vec3Array& corners2,int pidx0,int pidx1, int pidx2, bool first,
	bool last )
{
    if (pidx0>=_polyLineCoords->size()-1)
	return false;

    const osg::Vec3  p0 = _polyLineCoords->at(pidx0);
    const osg::Vec3  p1 = _polyLineCoords->at(pidx1);
    const osg::Vec3  p2 = _polyLineCoords->at(pidx2);
   	    
    osg::Vec3 vec01 = p1 - p0; vec01.normalize();
    osg::Vec3 vec12 = p2 - p1; vec12.normalize();

    const bool vec12ok = vec12.length() != 0;

    if ( first && vec01.length()==0 )
	return false;

    const bool doreverse = vec01 * vec12 < -0.5f;
    const osg::Vec3 planenormal =
	vec12ok ? doreverse ? vec12 - vec01 : vec01 + vec12 : vec01;
    
    if (first)
    {
	//Create the cap and record the points in corner1
	osg::Vec3 curu,curv;
	getOrthoVecs(vec01,curu,curv);
	for (int idx=0; idx<_resolution; idx++)
	{
	    float angl=idx*2*M_PI/_resolution;
	    const osg::Vec3 vec1 = curu*cos(angl)+curv*sin(angl);
	    corners1[idx] = vec1*_radius+p0;
	}

	addCap(p0,corners1,-planenormal);
    }

    const osg::Plane plane(planenormal,p1);
    for (int idx=0; idx<_resolution; idx++)
    {
	const osgGeo::Line3 lineproj(corners1[idx],vec01);
	corners2[idx] = lineproj.getInterSectionPoint(plane);
	mAddVertex(corners1[idx],p0);
	mAddVertex(corners2[idx],p1);
    }

    mAddVertex(corners1[0],p0);
    mAddVertex(corners2[0],p1);

    if (doreverse||last)
    {
	//Make cap
	addCap(p1,corners2,-planenormal);
    }
    corners1 = corners2;
    return true;
}


void PolyLineNode::addCap(const osg::Vec3& p,const osg::Vec3Array& cornerpoints,const osg::Vec3& planenormal)
{
    for (int idx = 0; idx<cornerpoints.size(); idx++)
    {
	mAddVertex(cornerpoints[idx],p);
	_geom3DCoords ->push_back(p);
	_capflags.push_back(true);
	_geom3DNormals->push_back(planenormal);
	_bbox.expandBy(p);
    }

    mAddVertex(cornerpoints[0],p);
}



bool PolyLineNode::needsUpdate() const
{
    if (_needsUpdate || _arrayModifiedCount!=_polyLineCoords->getModifiedCount())
	return true;

    for (int idx=_primitiveSets.size()-1;idx>=0;idx--)
    {
	if (_primitiveSets[idx]->getModifiedCount()!=
	     _primitivesetModCount[idx])
	{
	    return true;
	}
    }

    return false;
}


void PolyLineNode::addPrimitiveSet(osg::PrimitiveSet* ps)
{
    _primitivesetModCount.push_back(ps->getModifiedCount());
    _primitiveSets.push_back( ps );
    touch();
}


int PolyLineNode::getPrimitiveSetIndex(const osg::PrimitiveSet* ps) const
{
    std::vector<osg::ref_ptr<osg::PrimitiveSet> >::const_iterator it =
	std::find(_primitiveSets.begin(), _primitiveSets.end(),ps);

    if (it==_primitiveSets.end())
	return -1;

    return it-_primitiveSets.begin();
}


void PolyLineNode::removePrimitiveSet(int idx)
{
    if (idx<0)
	return;

    if (idx<(int)_primitivesetModCount.size())
	_primitivesetModCount.erase(_primitivesetModCount.begin()+idx);

    if (idx<(int)_primitiveSets.size())
	_primitiveSets.erase(_primitiveSets.begin()+idx);
    
    touch();
}

} //namespace osgGeo
