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

$Id: TrackballManipulator.cpp 231 2013-04-16 12:35:57Z kristofer.tingdahl@dgbes.com $
*/

#include <osgGeo/Draggers>


namespace osgGeo
{

Translate1DDragger::Translate1DDragger()
    : osgManipulator::Translate1DDragger()
    , _inactivationModKeyMask( 0 )
{}


Translate1DDragger::Translate1DDragger(const osg::Vec3d& s, const osg::Vec3d& e)
    : osgManipulator::Translate1DDragger( s,e )
    , _inactivationModKeyMask( 0 )
{}


bool Translate1DDragger::handle(const osgManipulator::PointerInfo& pointer, const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
    if ( ea.getModKeyMask() & _inactivationModKeyMask )
	return false;

    return osgManipulator::Translate1DDragger::handle( pointer, ea, aa );
}


void Translate1DDragger::setInactivationModKeyMask(unsigned int mask)
{
    _inactivationModKeyMask = mask;
}


unsigned int Translate1DDragger::getInactivationModKeyMask() const
{
    return _inactivationModKeyMask;
}


//=============================================================================


Translate2DDragger::Translate2DDragger()
    : osgManipulator::Translate2DDragger()
    , _inactivationModKeyMask( 0 )
{}


Translate2DDragger::Translate2DDragger(const osg::Plane& plane)
    : osgManipulator::Translate2DDragger( plane )
    , _inactivationModKeyMask( 0 )
{}


bool Translate2DDragger::handle(const osgManipulator::PointerInfo& pointer, const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
    if ( ea.getModKeyMask() & _inactivationModKeyMask )
	return false;

    return osgManipulator::Translate2DDragger::handle( pointer, ea, aa );
}


void Translate2DDragger::setInactivationModKeyMask(unsigned int mask)
{
    _inactivationModKeyMask = mask;
}


unsigned int Translate2DDragger::getInactivationModKeyMask() const
{
    return _inactivationModKeyMask;
}




} // end namespace

