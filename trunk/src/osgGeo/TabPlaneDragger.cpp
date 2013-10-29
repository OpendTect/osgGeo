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

#include <osgGeo/TabPlaneDragger>


namespace osgGeo
{


TabPlaneDragger::TabPlaneDragger( float handleScaleFactor )
    : osgManipulator::TabPlaneDragger( handleScaleFactor )
    , _modKeyMask1D( osgGA::GUIEventAdapter::NONE )
    , _modKeyMask2D( osgGA::GUIEventAdapter::MODKEY_SHIFT )
{
}


bool TabPlaneDragger::handle(const osgManipulator::PointerInfo& pointer, const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
    if (ea.getButtonMask() & osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON) return false;

    // Check if the dragger node is in the nodepath.
    if (!pointer.contains(this)) return false;

    // Since the translate plane and the handleNode lie on the same plane the hit could've been on either one. But we
    // need to handle the scaling draggers before the translation. Check if the node path has the scaling nodes else
    // check for the scaling nodes in next hit.
    if (_cornerScaleDragger->handle(pointer, ea, aa))
        return true;
    if (_horzEdgeScaleDragger->handle(pointer, ea, aa))
        return true;
    if (_vertEdgeScaleDragger->handle(pointer, ea, aa))
        return true;

    osgManipulator::PointerInfo nextPointer(pointer);
    nextPointer.next();

    while (!nextPointer.completed())
    {
        if (_cornerScaleDragger->handle(nextPointer, ea, aa))
            return true;
        if (_horzEdgeScaleDragger->handle(nextPointer, ea, aa))
            return true;
        if (_vertEdgeScaleDragger->handle(nextPointer, ea, aa))
            return true;

        nextPointer.next();
    }

    // Start of customization. The rest of this function is supposed to
    // remain an exact copy of the handle(.) function in the base class.

    osg::ref_ptr<osgGA::GUIEventAdapter> ea1 = new osgGA::GUIEventAdapter(ea);
    if ( !convToTranslatePlaneDraggerEvent(*ea1) )
	return false;

    if (_translateDragger->handle(pointer, *ea1, aa))
        return true;

    // End of customization

    return false;
}


static bool isModKeyMaskMatching( osgGA::GUIEventAdapter& ea, int mask )
{
    const int ctrl  = osgGA::GUIEventAdapter::MODKEY_CTRL;
    const int shift = osgGA::GUIEventAdapter::MODKEY_SHIFT;
    const int alt   = osgGA::GUIEventAdapter::MODKEY_ALT;
    const int meta  = osgGA::GUIEventAdapter::MODKEY_META;
    const int super = osgGA::GUIEventAdapter::MODKEY_SUPER;
    const int hyper = osgGA::GUIEventAdapter::MODKEY_HYPER;

    // Let's be insensitive to left and/or right mod key issues as long as
    // distinguishing between them not fully supported by OSG (actually Qt).

    if ( (ea.getModKeyMask()&ctrl)  != (mask&ctrl) )  return false;
    if ( (ea.getModKeyMask()&shift) != (mask&shift) ) return false;
    if ( (ea.getModKeyMask()&alt)   != (mask&alt) )   return false;
    if ( (ea.getModKeyMask()&meta)  != (mask&meta) )  return false;
    if ( (ea.getModKeyMask()&super) != (mask&super) ) return false;
    if ( (ea.getModKeyMask()&hyper) != (mask&hyper) ) return false;

    return true;
}


bool TabPlaneDragger::convToTranslatePlaneDraggerEvent( osgGA::GUIEventAdapter& ea ) const
{
    if ( ea.getButtonMask() == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON )
    {
	if ( isModKeyMaskMatching(ea,_modKeyMask1D) )
	{
	    // Perpendicular-to-plane translation
	    ea.setButtonMask( osgGA::GUIEventAdapter::MIDDLE_MOUSE_BUTTON );
	    ea.setModKeyMask( osgGA::GUIEventAdapter::NONE );
	    return true;
	}

	if ( isModKeyMaskMatching(ea,_modKeyMask2D) )
	{
	    // In-plane translation
	    ea.setModKeyMask( osgGA::GUIEventAdapter::NONE );
	    return true;
	}
    }

    // Don't steal RELEASE. Otherwise Translate1DDragger is not deactivated!
    return ea.getEventType()==osgGA::GUIEventAdapter::RELEASE;
}


} // end namespace

