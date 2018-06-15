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

#include <osg/Version>
#include <osgGeo/Text>

#include <iostream>
#include <cstdio>


namespace osgGeo
{

#define MAX_ELEVATION_ANGLE	(75.0*M_PI/180.0)

Text::Text()
    : osgText::Text()
{}


Text::Text( const Text& text, const osg::CopyOp& copyop )
    : osgText::Text(text,copyop)
{}


Text::~Text()
{}

} // end namespace
