#include <osg/Geode>
#include <osg/Geometry>

#include <osgViewer/Viewer>

#include <osgGeo/Horizon3D2.h>

int main(int argc, char **argv)
{
    double undef = 999999.0;
    osgGeo::Vec2i size(512, 512);
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
    viewer.setSceneData(hor3);
    return viewer.run();
}
