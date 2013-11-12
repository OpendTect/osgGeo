
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

#include <osgGeo/AxesNode>

#include <osg/Geode>
#include <osg/Geometry>
#include <osgUtil/CullVisitor>
#include <osg/Material>
#include <osg/Matrix>
#include <osg/MatrixTransform>
#include <osg/ShapeDrawable>

DISABLE_WARNINGS;

#include <osgText/Text>

ENABLE_WARNINGS;

namespace osgGeo
{

AxesNode::AxesNode()
    : _needsUpdate(true)
    , _radius(1)
    , _length(10)
    , _annotColor(osg::Vec4(1.0f,1.0f,1.0f,1.0f))
    , _root(new osg::Group)
    , _transform(new osg::MatrixTransform)
    , _mastercamera(0)
{
    setNumChildrenRequiringUpdateTraversal( 1 );
}


AxesNode::AxesNode( const AxesNode& node, const osg::CopyOp& co )
    : osg::Node(node,co)
    , _needsUpdate(true)
    , _radius(node._radius)
    , _length(node._length)
    , _annotColor(node._annotColor)
    , _root(node._root)
    , _transform(node._transform)
    , _mastercamera(node._mastercamera)
{
    setNumChildrenRequiringUpdateTraversal( 1 );
}


AxesNode::~AxesNode()
{}


void AxesNode::traverse( osg::NodeVisitor& nv )
{
    if ( nv.getVisitorType() == osg::NodeVisitor::CULL_VISITOR )
    {
       if ( needsUpdate() )
	   updateGeometry();
    
       if ( _mastercamera )
       {
	    osg::Vec3 eye, center, up; 
	    _mastercamera->getViewMatrixAsLookAt( eye, center, up, 45 ); 
	    osg::Matrixd matrix = _transform->getMatrix();
	    matrix.makeLookAt( eye-center, osg::Vec3(0,0,0), up ); 
	    matrix *= osg::Matrix::translate( _pos.x(), _pos.y(), 0 );
	    _transform->setMatrix( matrix );
	}
    }

    _transform->accept( nv );
}


osg::ref_ptr<osg::Node> arrowNode( const float rad, const float len, 
				   const osg::Vec4& color, const osg::Vec3& dir,
				   const char* text, const osg::Vec4& annotclr )
{
    osg::ref_ptr<osg::Vec3Array> coords = new osg::Vec3Array;
    osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array;
    const int resolution = 10;
    osg::Vec3 curu,curv;
    
    osg::Vec3 w( dir ); w.normalize();
    osg::Vec3 anyvec(1,1,1); anyvec.normalize();
    curu = w ^ anyvec; curu.normalize();
    curv = w ^ curu;
    curv.normalize();

    const float conelen = 0.33 * len;
    const osg::Vec3 p1 = dir * len;
    const osg::Vec3 p2 = dir * ( len - conelen );
   
    osg::Vec3 normdir( dir );
    normdir.normalize();
    osg::DrawElementsUInt* cone =
	new osg::DrawElementsUInt( osg::PrimitiveSet::TRIANGLE_STRIP ); 
    cone->push_back( 0 );
    int ci = 0;
    for ( int idx=0; idx<=resolution; idx++ )
    {
	float angl = idx * 2 * M_PI / resolution;
	osg::Vec3 vec = ( curu * cos(angl) ) + ( curv * sin(angl) );
	osg::Vec3 norm = vec + normdir;
	norm.normalize();
	normals->push_back( norm );
	coords->push_back( p1 );
	cone->push_back( ci++ );
	coords->push_back( p2 + vec*rad   );
	normals->push_back( norm );
	cone->push_back( ci++ );
    }

    //cone base
    osg::DrawElementsUInt* conebase =
	new osg::DrawElementsUInt( osg::PrimitiveSet::TRIANGLE_FAN ); 
    float conesz = coords->size();
    for ( int idx=0; idx<=resolution; idx++ )
    {
	float angl = idx * 2 * M_PI / resolution;
	osg::Vec3 vec = ( curu * cos(angl) ) + ( curv * sin(angl) );
	coords->push_back( p2 + vec*rad   );
	normals->push_back( -normdir );
	conebase->push_back( idx+conesz );
    }
    
    // cylinder
    float platesz = coords->size();
    for ( int idx=0; idx<=resolution; idx++ )
    {
	float angl = idx * 2 * M_PI / resolution;
	osg::Vec3 vec = ( curu * cos(angl) ) + ( curv * sin(angl) );
	coords->push_back( p2 + vec*rad/2   );
	normals->push_back( vec );
	coords->push_back( vec*rad/2   );
	normals->push_back( vec );
    }

    osg::Geometry* arrowgeometry = new osg::Geometry();
    arrowgeometry->setVertexArray( coords );
    arrowgeometry->addPrimitiveSet( cone );
    arrowgeometry->addPrimitiveSet( conebase );
    const float cylsz = coords->size() - platesz;
    arrowgeometry->addPrimitiveSet( 
		   new osg::DrawArrays( GL_TRIANGLE_STRIP, platesz, cylsz, 0 ));
    arrowgeometry->setNormalArray( normals.get() );
    arrowgeometry->setNormalBinding( osg::Geometry::BIND_PER_VERTEX );

    osg::ref_ptr<osg::Geode> arrowgeode = new osg::Geode();
    osg::ref_ptr<osg::Material> mat = new osg::Material;
    const float fac = 0.3f;
    osg::Vec4 ambcolor( color.r()*fac, color.g()*fac, color.b()*fac, 1.0f );
    mat->setDiffuse( osg::Material::FRONT, color );
    mat->setAmbient( osg::Material::FRONT, ambcolor );
    arrowgeode->getOrCreateStateSet()->setAttribute( mat );
    arrowgeode->addDrawable( arrowgeometry );
   
    osg::ref_ptr<osg::Geode> textgeode = new osg::Geode();
    osg::ref_ptr<osgText::Text> annot = new  osgText::Text;
    annot->setPosition( p1 );
    annot->setCharacterSize( 18 );
    annot->setAxisAlignment( osgText::TextBase::SCREEN );
    annot->setCharacterSizeMode( osgText::TextBase::SCREEN_COORDS );
    annot->setAutoRotateToScreen( true );
    annot->setText( text );
    annot->setColor( annotclr );
    textgeode->addDrawable( annot );
    osg::ref_ptr<osg::Group> grp = new osg::Group;
    grp->addChild( arrowgeode );
    grp->addChild( textgeode );
    return grp.get();
}


bool AxesNode::updateGeometry()
{
    if ( !_needsUpdate )
	return false;

    osg::Vec4 red(1,0,0,0), green(0,1,0,0), blue(0,0.55,1,0), yellow(1,1,0,1);
    
    osg::ref_ptr<osg::Material> mat = new osg::Material;
    mat->setDiffuse( osg::Material::FRONT, yellow );
    osg::ref_ptr<osg::ShapeDrawable> sphere = 
	new osg::ShapeDrawable(new osg::Sphere( osg::Vec3f(0,0,0),_radius*0.75f) );
    osg::ref_ptr<osg::Geode> spheregeode = new osg::Geode();
    spheregeode->getOrCreateStateSet()->setAttribute( mat );
    spheregeode->addDrawable( sphere );
    const osg::Vec4& c = _annotColor;
    _root->removeChildren( 0, _root->getNumChildren() );
    _root->addChild( spheregeode );
    _root->addChild( arrowNode(_radius,_length,red,osg::Vec3(0,1,0),  "N", c) );
    _root->addChild( arrowNode(_radius,_length,green,osg::Vec3(1,0,0),"E", c) );
    _root->addChild( arrowNode(_radius,_length,blue,osg::Vec3(0,0,-1),"Z", c) );
    _transform->addChild( _root );
    _needsUpdate = false;
    return true;
}


osg::BoundingSphere AxesNode::computeBound() const
{
    osg::BoundingSphere sphere;
    return sphere;
}


bool AxesNode::needsUpdate() const
{
    return _needsUpdate;
}


void AxesNode::setMasterCamera( osg::Camera* camera )
{
    _mastercamera = camera;
}


void AxesNode::setRadius( const float& radius )
{
    _radius = radius;
    _needsUpdate = true;
}


void AxesNode::setLength( const float& len )
{
    _length = len;
    _needsUpdate = true;
}


void AxesNode::setPosition( osg::Vec2 pos )
{
    _pos = pos;
    _transform->setMatrix( osg::Matrix::translate(_pos.x(),_pos.y(),0.0f) );
}


void AxesNode::setSize( osg::Vec2 size )
{
    _radius = size.x();
    _length = size.y();
    _size = size;
    _needsUpdate = true;
}


void AxesNode::setAnnotationColor( osg::Vec4 col )
{
    _annotColor = col;
    _needsUpdate = true;
}

} //namespace osgGeo
