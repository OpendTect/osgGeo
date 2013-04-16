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

#include <osgGeo/TrackballManipulator>


namespace osgGeo
{


TrackballManipulator::TrackballManipulator( int flags )
    : osgGA::TrackballManipulator( flags )
    , _dragEnabled( true )
{
}


TrackballManipulator::TrackballManipulator( const TrackballManipulator& tm, const osg::CopyOp& copyOp )
    : osgGA::TrackballManipulator( tm, copyOp )
    , osg::Object( tm, copyOp )	// needs explicit init in copy constructor because of [-Wextra] warning
    , _dragEnabled( tm._dragEnabled )
{
}


bool TrackballManipulator::handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
{
    if ( !_dragEnabled && ea.getEventType()==osgGA::GUIEventAdapter::DRAG )
	return false;

    return osgGA::TrackballManipulator::handle( ea, aa );
}


} // end namespace

