#include <osg/Node> 
#include <osgGeo/PolyLine>
#include <osgViewer/Viewer> 

osg::ref_ptr<osg::Node> drawPolyLineSnake()
{
    srand( clock() );
    osg::ref_ptr<osg::Vec3Array> coords = new osg::Vec3Array;
    for ( int idx=0; idx<100; idx++ )
    { 
	osg::Vec3 coord( 200*sin(idx/10.0), 200*cos(idx/10.0), idx*20 ); 
	coords->push_back( coord );
    }
    osg::ref_ptr<osgGeo::PolylineNode> polyline = new osgGeo::PolylineNode();
    polyline->setRadius( 50 );
    polyline->setColor( osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f) );
    polyline->setVertexArray( coords );
    return polyline.get();
}

int main()
{
    osgViewer::Viewer viewer;
    viewer.setSceneData ( drawPolyLineSnake() );
    viewer.setUpViewInWindow( 20, 20, 640, 480 );
    viewer.run();
}

