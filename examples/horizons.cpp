#include <iostream>
#include <stdlib.h>

#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgGeo/Horizon3D>
#include <osgGeo/Horizon3D2>
#include <osgGeo/ShaderUtility.h>

int main(int argc, char **argv)
{
    osgGeo::ShaderUtility::setRootPath("/ldata1/hg/osg/osgGeo/src/osgGeo/shaders");

    double undef = 999999.0;

    int sizes[7] = {255, 511, 1021, 2041, 3061, 4081, 8161};
    int sz = sizes[2];
    osgGeo::Vec2i size(sz, sz);

    osg::ref_ptr<osg::DoubleArray> depthValsPtr =
            new osg::DoubleArray(size.x()*size.y());

    osg::DoubleArray &depthVals = *depthValsPtr;

    for(int i = 0; i < size.x(); ++i)
        for(int j = 0; j < size.y(); ++j)
        {
            float val = 0;//float(std::rand()) / RAND_MAX * 3;
            float zFactor = 50.0 / size.x();
            depthVals[i*size.y()+j] = sin(double(i+val)/50.0) * sin(double(j+val)/50.0) *zFactor;
        }

// This snippet creates stripes of undefined values
// to test the lines display
//    for(int i = 100; i < 200; ++i)
//        for(int j = 0; j < size.y(); ++j)
//        {
//            if(i % 2 == 1)
//                depthVals[i*size.y()+j] = undef;
//        }

    std::vector<osg::Vec2d> coords;
    coords.push_back(osg::Vec2d(0, 0));
    coords.push_back(osg::Vec2d(0, 1));
    coords.push_back(osg::Vec2d(1, 0));

    bool shadersEnabled = true;
    osg::ref_ptr<osgGeo::Horizon3DBase> horizon3d;
    if(shadersEnabled)
        horizon3d = new osgGeo::Horizon3DNode2();
    else
        horizon3d = new osgGeo::Horizon3DNode();

    horizon3d->setCornerCoords(coords);
    horizon3d->setSize(size);
    horizon3d->setMaxDepth(undef);
    horizon3d->setDepthArray(depthValsPtr.get());

    osgViewer::Viewer viewer;
    // add the stats handler
    viewer.addEventHandler(new osgViewer::StatsHandler);
    viewer.setSceneData(horizon3d.get());
    return viewer.run();
}

