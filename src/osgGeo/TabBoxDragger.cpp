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

#include <osgGeo/TabBoxDragger>

#include <osg/ShapeDrawable>
#include <osg/Geometry>
#include <osg/LineWidth>
#include <osg/Quat>


namespace osgGeo
{


TabBoxDragger::TabBoxDragger( float handleScaleFactor )
{
    for (int i=0; i<6; ++i)
    {
        _planeDraggers.push_back(new osgGeo::TabPlaneDragger(handleScaleFactor));
        addChild(_planeDraggers[i].get());
        addDragger(_planeDraggers[i].get());
    }

    {
        _planeDraggers[0]->setMatrix(osg::Matrix::translate(osg::Vec3(0.0,0.5,0.0)));
    }
    {
        osg::Quat rotation; rotation.makeRotate(osg::Vec3(0.0f, -1.0f, 0.0f), osg::Vec3(0.0f, 1.0f, 0.0f));
        _planeDraggers[1]->setMatrix(osg::Matrix(rotation)*osg::Matrix::translate(osg::Vec3(0.0,-0.5,0.0)));
    }

    {
        osg::Quat rotation; rotation.makeRotate(osg::Vec3(0.0f, 1.0f, 0.0f), osg::Vec3(0.0f, 0.0f, 1.0f));
        _planeDraggers[2]->setMatrix(osg::Matrix(rotation)*osg::Matrix::translate(osg::Vec3(0.0,0.0,0.5)));
    }
    {
        osg::Quat rotation; rotation.makeRotate(osg::Vec3(0.0f, 0.0f, 1.0f), osg::Vec3(0.0f, 1.0f, 0.0f));
        _planeDraggers[3]->setMatrix(osg::Matrix(rotation)*osg::Matrix::translate(osg::Vec3(0.0,0.0,-0.5)));
    }

    {
        osg::Quat rotation; rotation.makeRotate(osg::Vec3(0.0f, 1.0f, 0.0f), osg::Vec3(1.0f, 0.0f, 0.0f));
        _planeDraggers[4]->setMatrix(osg::Matrix(rotation)*osg::Matrix::translate(osg::Vec3(0.5,0.0,0.0)));
    }
    {
        osg::Quat rotation; rotation.makeRotate(osg::Vec3(1.0f, 0.0f, 0.0f), osg::Vec3(0.0f, 1.0f, 0.0f));
        _planeDraggers[5]->setMatrix(osg::Matrix(rotation)*osg::Matrix::translate(osg::Vec3(-0.5,0.0,0.0)));
    }

    setParentDragger(getParentDragger());
}


TabBoxDragger::~TabBoxDragger()
{
}


void TabBoxDragger::setupDefaultGeometry()
{
    for (unsigned int i=0; i<_planeDraggers.size(); ++i)
        _planeDraggers[i]->setupDefaultGeometry(false);
}


void TabBoxDragger::setPlaneColor(const osg::Vec4& color)
{
    for (unsigned int i=0; i<_planeDraggers.size(); ++i)
        _planeDraggers[i]->setPlaneColor(color);
}


void TabBoxDragger::set1DTranslateMouseButtonMaskOfPlanes(int mask,int idx)
{
    for (unsigned int i=0; i<_planeDraggers.size(); ++i)
        _planeDraggers[i]->set1DTranslateMouseButtonMask( mask, idx );
}


int TabBoxDragger::get1DTranslateMouseButtonMaskOfPlanes(int idx) const
{
    return _planeDraggers[0]->get1DTranslateMouseButtonMask( idx ); 
}


void TabBoxDragger::set2DTranslateMouseButtonMaskOfPlanes(int mask,int idx)
{
    for (unsigned int i=0; i<_planeDraggers.size(); ++i)
        _planeDraggers[i]->set2DTranslateMouseButtonMask( mask, idx );
}


int TabBoxDragger::get2DTranslateMouseButtonMaskOfPlanes(int idx) const
{
    return _planeDraggers[0]->get2DTranslateMouseButtonMask( idx );
}


void TabBoxDragger::set1DTranslateModKeyMaskOfPlanes(int mask,int idx)
{
    for (unsigned int i=0; i<_planeDraggers.size(); ++i)
        _planeDraggers[i]->set1DTranslateModKeyMask( mask, idx );
}


int TabBoxDragger::get1DTranslateModKeyMaskOfPlanes(int idx) const
{
    return _planeDraggers[0]->get1DTranslateModKeyMask( idx ); 
}


void TabBoxDragger::set2DTranslateModKeyMaskOfPlanes(int mask,int idx)
{
    for (unsigned int i=0; i<_planeDraggers.size(); ++i)
        _planeDraggers[i]->set2DTranslateModKeyMask( mask, idx );
}


int TabBoxDragger::get2DTranslateModKeyMaskOfPlanes(int idx) const
{
    return _planeDraggers[0]->get2DTranslateModKeyMask( idx );
}


const TabPlaneDragger* TabBoxDragger::getEventHandlingTabPlane() const
{
    for (unsigned int i=0; i<_planeDraggers.size(); ++i)
    {
	if ( _planeDraggers[i]->isCurrentEventHandler() )
	    return _planeDraggers[i];
    }
    return 0;
}


int TabBoxDragger::getEventHandlingTabPlaneIdx() const
{
    for (unsigned int i=0; i<_planeDraggers.size(); ++i)
    {
	if ( _planeDraggers[i]->isCurrentEventHandler() )
	    return i;
    }
    return -1;
}


int TabBoxDragger::getCurMouseButtonModKeyIdx() const
{
    const int i = getEventHandlingTabPlaneIdx();
    return i>=0 ? _planeDraggers[i]->getCurMouseButtonModKeyIdx() : -1;
}


} // end namespace

