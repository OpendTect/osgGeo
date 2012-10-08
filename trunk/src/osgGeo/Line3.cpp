#include <osgGeo/Line3>

#include <osg/Plane>

#define mDefEpsD		(1e-10)
#define mIsZero(x,eps)		( (x) < (eps) && (x) > (-eps) )

namespace osgGeo
{

bool Line3::intersectWith( const osg::Plane& plane, float& t ) const
{
    const double denominator = _dir.x()*plane[0] + _dir.y()*plane[1]
				+ _dir.z()*plane[2];
    const double dist0 =_pos.x()*plane[0] + _pos.y()*plane[1]
				+ _pos.z()*plane[2] + plane[3];
    if ( mIsZero(denominator,mDefEpsD) )
    {
	if ( mIsZero(dist0/sqrt(plane[0]*plane[0]+plane[1]*plane[1]+plane[2]*plane[2]),mDefEpsD) )
	{
	    t = 0;
	    return true;
	}

	return false;
    }

    t = -dist0 / denominator;
    return true;
}


osg::Vec3 Line3::getInterSectionPoint( const osg::Plane& plane ) const
{
    float intercept;
    intersectWith( plane, intercept );
    return getPositionOnLine( intercept );
}

} // osgGeo
