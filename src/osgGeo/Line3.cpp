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

#include <osgGeo/Line3>
#include <osg/Plane>
#include <osgDB/InputStream>
#include <osgDB/OutputStream>

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
    float intercept = 0;
    intersectWith( plane, intercept );
    return getPositionOnLine( intercept );
}


osgDB::InputStream& operator>>( osgDB::InputStream& is, Line3& line )
{
    return is >> line._pos >> line._dir;
}


osgDB::OutputStream& operator<<( osgDB::OutputStream& os, const Line3& line )
{
    return os << line._pos << line._dir;
}


} // osgGeo
