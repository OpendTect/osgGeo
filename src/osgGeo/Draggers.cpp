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
#include <osgViewer/View>
#include <osg/Version>


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
    if ( ea.getEventType()==osgGA::GUIEventAdapter::PUSH )
    {
	if ( ea.getModKeyMask() & _inactivationModKeyMask )
	    return false;
    }

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
    if ( ea.getEventType()==osgGA::GUIEventAdapter::PUSH )
    {
	if ( ea.getModKeyMask() & _inactivationModKeyMask )
	    return false;
    }

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


//===== TEMPORARY CLASSES TO PRESERVE BINARY COMPATIBILITY ====================


FastTranslate1DDragger::FastTranslate1DDragger()
    : osgManipulator::Translate1DDragger()
    , _inactivationModKeyMask( 0 )
{}


FastTranslate1DDragger::FastTranslate1DDragger(const osg::Vec3d& s, const osg::Vec3d& e)
    : osgManipulator::Translate1DDragger( s,e )
    , _inactivationModKeyMask( 0 )
{}


void FastTranslate1DDragger::traverse(osg::NodeVisitor& nv)
{
    if (_handleEvents && nv.getVisitorType()==osg::NodeVisitor::EVENT_VISITOR)
    {
	osgGA::EventVisitor* ev = dynamic_cast<osgGA::EventVisitor*>(&nv);
	if (ev)
	{
	    for(osgGA::EventQueue::Events::iterator itr = ev->getEvents().begin(); itr != ev->getEvents().end(); ++itr)
	    {
#if OSG_MIN_VERSION_REQUIRED(3,3,0)
		osgGA::GUIEventAdapter* ea = itr->asGUIEventAdapter();
#else
		osgGA::GUIEventAdapter* ea = itr->get();
#endif
		// begin modification
		if ( ea && ea->getEventType()==osgGA::GUIEventAdapter::PUSH )
		{
		    osgViewer::View* view = dynamic_cast<osgViewer::View*>(ev->getActionAdapter());
		    if ( view )
		    {
			osgUtil::LineSegmentIntersector::Intersections intersections;
			if ( !view->computeIntersections(*ea,nv.getNodePath(),intersections,_intersectionMask) ) continue;
		    }
		}
		// end modification
		if (Dragger::handle(*ea, *(ev->getActionAdapter()))) ea->setHandled(true);
	    }
	}
    }
    MatrixTransform::traverse(nv);
}


bool FastTranslate1DDragger::handle(const osgManipulator::PointerInfo& pointer, const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
    if ( ea.getEventType()==osgGA::GUIEventAdapter::PUSH )
    {
	if ( ea.getModKeyMask() & _inactivationModKeyMask )
	    return false;
    }

    return osgManipulator::Translate1DDragger::handle( pointer, ea, aa );
}


void FastTranslate1DDragger::setInactivationModKeyMask(unsigned int mask)
{
    _inactivationModKeyMask = mask;
}


unsigned int FastTranslate1DDragger::getInactivationModKeyMask() const
{
    return _inactivationModKeyMask;
}


//=============================================================================


FastTranslate2DDragger::FastTranslate2DDragger()
    : osgManipulator::Translate2DDragger()
    , _inactivationModKeyMask( 0 )
{}


FastTranslate2DDragger::FastTranslate2DDragger(const osg::Plane& plane)
    : osgManipulator::Translate2DDragger( plane )
    , _inactivationModKeyMask( 0 )
{}


void FastTranslate2DDragger::traverse(osg::NodeVisitor& nv)
{
    if (_handleEvents && nv.getVisitorType()==osg::NodeVisitor::EVENT_VISITOR)
    {
	osgGA::EventVisitor* ev = dynamic_cast<osgGA::EventVisitor*>(&nv);
	if (ev)
	{
	    for(osgGA::EventQueue::Events::iterator itr = ev->getEvents().begin(); itr != ev->getEvents().end(); ++itr)
	    {
#if OSG_MIN_VERSION_REQUIRED(3,3,0)
		osgGA::GUIEventAdapter* ea = itr->asGUIEventAdapter();
#else
		osgGA::GUIEventAdapter* ea = itr->get();
#endif
		// begin modification
		if ( ea && ea->getEventType()==osgGA::GUIEventAdapter::PUSH )
		{
		    osgViewer::View* view = dynamic_cast<osgViewer::View*>(ev->getActionAdapter());
		    if ( view )
		    {
			osgUtil::LineSegmentIntersector::Intersections intersections;
			if ( !view->computeIntersections(*ea,nv.getNodePath(),intersections,_intersectionMask) ) continue;
		    }
		}
		// end modification
		if (Dragger::handle(*ea, *(ev->getActionAdapter()))) ea->setHandled(true);
	    }
	}
    }
    MatrixTransform::traverse(nv);
}


bool FastTranslate2DDragger::handle(const osgManipulator::PointerInfo& pointer, const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
    if ( ea.getEventType()==osgGA::GUIEventAdapter::PUSH )
    {
	if ( ea.getModKeyMask() & _inactivationModKeyMask )
	    return false;
    }

    return osgManipulator::Translate2DDragger::handle( pointer, ea, aa );
}


void FastTranslate2DDragger::setInactivationModKeyMask(unsigned int mask)
{
    _inactivationModKeyMask = mask;
}


unsigned int FastTranslate2DDragger::getInactivationModKeyMask() const
{
    return _inactivationModKeyMask;
}


} // end namespace

