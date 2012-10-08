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

#include <osg/Geode>
#include <osg/Geometry>

#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>

#include <osgGeo/Horizon3D2.h>

int main(int argc, char **argv)
{
    double undef = 999999.0;

    int sizes[5] = {511, 1021, 2041, 3061, 4081};
    int sz = sizes[0];
    osgGeo::Vec2i size(sz, sz);

    osg::ref_ptr<osg::DoubleArray> depthValsPtr =
            new osg::DoubleArray(size.x()*size.y());

    osg::DoubleArray &depthVals = *depthValsPtr;

    for(int i = 0; i < size.x(); ++i)
        for(int j = 0; j < size.y(); ++j)
        {
            float val = 0;//float(std::rand()) / RAND_MAX * 3;
            float zFactor = 25.0 / size.x();
            depthVals[i*size.y()+j] = sin(double(i+val)/50.0) * sin(double(j+val)/50.0) *zFactor;
        }

    std::vector<osg::Vec2d> coords;
    coords.push_back(osg::Vec2d(0, 0));
    coords.push_back(osg::Vec2d(0, 1));
    coords.push_back(osg::Vec2d(1, 0));

    osg::ref_ptr<osgGeo::Horizon3D2> hor3 = new osgGeo::Horizon3D2;

    hor3->setCornerCoords(coords);
    hor3->setSize(size);
    hor3->setDepthArray(depthValsPtr.get());

    osgViewer::Viewer viewer;
    // add the stats handler
    viewer.addEventHandler(new osgViewer::StatsHandler);
    viewer.setSceneData(hor3);
    return viewer.run();
}
