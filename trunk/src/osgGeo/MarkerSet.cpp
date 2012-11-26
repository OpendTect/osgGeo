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

$Id: OneSideRender.cpp 108 2012-10-08 08:32:40Z kristofer.tingdahl@dgbes.com $

*/

#include "osgGeo/MarkerSet"
#include <osgGeo/AutoTransform>

namespace osgGeo
{

MarkerSet::MarkerSet()
    :_rotateMode( osgGeo::AutoTransform::ROTATE_TO_SCREEN )
    ,_hints(new osg::TessellationHints )
    ,_shapeType( osgGeo::MarkerSet::Box )
    ,_vec4Array( new osg::Vec4Array )
    ,_vec3Array( new osg::Vec3Array )
    ,_nonshadinggroup( new osg::Group )
    ,_geode( new osg::Geode )
    ,_needsUpdate( true )
    ,_arraymodcount( -1 )
    ,_radius(0.0f)
    ,_height(0.0f)
{
    setNumChildrenRequiringUpdateTraversal( 1 );
    _hints->setDetailRatio(0.5f);
}

MarkerSet::MarkerSet( const MarkerSet& node, const osg::CopyOp& co )
    : osg::Node(node,co)
    ,_rotateMode( osgGeo::AutoTransform::ROTATE_TO_SCREEN )
    ,_hints(new osg::TessellationHints )
    ,_shapeType( osgGeo::MarkerSet::Box )
    ,_vec4Array( new osg::Vec4Array )
    ,_vec3Array( new osg::Vec3Array )
    ,_nonshadinggroup( new osg::Group )
    ,_geode( new osg::Geode )
    ,_needsUpdate( true )
    ,_arraymodcount( -1 )
    ,_radius(0.0f)
    ,_height(0.0f)
{
    setNumChildrenRequiringUpdateTraversal( 1 );
    _hints->setDetailRatio(0.5f);
}

MarkerSet::~MarkerSet()
{
}

void MarkerSet::traverse( osg::NodeVisitor& nv )
{
    if ( nv.getVisitorType()==osg::NodeVisitor::UPDATE_VISITOR )
    {
	if ( needsUpdate() )
	    updateShapes();
    }

    if ( _nonshadinggroup ) _nonshadinggroup->accept( nv );
}

bool MarkerSet::updateShapes()
{
    if ( !_vec3Array || !_needsUpdate ||!_geode)
	return false;

    _nonshadinggroup->removeChildren( 0, _nonshadinggroup->getNumChildren() );

    for ( unsigned int idx=0; idx<_vec3Array->size(); idx++ )
    {
	osg::ref_ptr<osg::ShapeDrawable> shapedrwb;
	osg::ref_ptr<osgGeo::AutoTransform> autotrans = 
	    new osgGeo::AutoTransform;
	switch(_shapeType)
	{
	case osgGeo::MarkerSet::Box:
	    shapedrwb = new osg::ShapeDrawable(new osg::Box(osg::Vec3f(0,0,0),
		_radius),_hints);
	    break;
	case osgGeo::MarkerSet::Cone:
	    shapedrwb = new osg::ShapeDrawable(new osg::Cone(osg::Vec3f(0,0,0),
		_radius,_height),_hints);
	    break;
	case osgGeo::MarkerSet::Sphere:
	    shapedrwb = new osg::ShapeDrawable(new osg::Sphere(osg::Vec3f(0,0,
		0),_radius),_hints);
	    break;
	case osgGeo::MarkerSet::Cylinder:
	    shapedrwb = new osg::ShapeDrawable(new osg::Cylinder(osg::Vec3f(0,
		0,0),_radius,_height),_hints);
	    break;
	default:
	    return false;
	}

	if ( idx<_vec4Array->size() )
	    shapedrwb->setColor(_vec4Array->at(idx) );
	else
	    shapedrwb->setColor( *(_vec4Array->end()-1) );

	autotrans->setPosition( _vec3Array->at(idx) );
	autotrans->setAutoRotateMode( _rotateMode );
	autotrans->setMinimumScale(_minScale);
	autotrans->setMaximumScale(_maxScale);
	autotrans->setAutoScaleToScreen(true);
	autotrans->setRestoreProportions(true);

	osg::ref_ptr<osg::Geode> geode = new osg::Geode;
	geode->addDrawable( shapedrwb );
	geode->getOrCreateStateSet()->setMode(GL_LIGHTING,
					osg::StateAttribute::ON);
	autotrans->addChild( geode );
	_nonshadinggroup->addChild( autotrans );
	
    }

    _needsUpdate = false;
    _arraymodcount = _vec3Array->getModifiedCount();
    return true;
}

osg::BoundingSphere MarkerSet::computeBound() const
{
    osg::BoundingSphere sphere;
    for ( std::vector<osg::Vec3>::const_iterator iter = _vec3Array->begin();
	  iter != _vec3Array->end(); iter++ )
    {
	sphere.expandBy( *iter );
    }

    return sphere;
}

bool MarkerSet::needsUpdate()
{
    if ( (int)_vec3Array->getModifiedCount() > _arraymodcount )
	_needsUpdate = true;
    return _needsUpdate;
}


void MarkerSet::setVertexArray(osg::Vec3Array* arr)
{
    _vec3Array = arr;
}

void MarkerSet::setShape(osgGeo::MarkerSet::MarkerType shape)
{
    _shapeType = shape;
    _needsUpdate = true;
}

void MarkerSet::setRadius(float radius)
{
    _radius = radius;
    _needsUpdate = true;

}

void MarkerSet::setHeight(float height)
{
    _height = height;
    _needsUpdate = true;
}

void MarkerSet::setDetail(float ratio)
{
    _hints->setDetailRatio(ratio);
    _needsUpdate = true;
}

void MarkerSet::setScale ( double scale )
{
    _scale = scale;
    _needsUpdate = true;
}

void MarkerSet::setScale ( const osg::Vec3d &scale )
{
    _vec3dScale  = scale;
    _needsUpdate = true;
}

void MarkerSet::setMinimumScale( double minScale )
{
    _minScale = minScale;
    _needsUpdate = true;
}

void MarkerSet::setMaximumScale( double maxScale )
{
    _maxScale = maxScale;
    _needsUpdate = true;
}

void MarkerSet::setAutoTransformRotateMode( osg::AutoTransform::
					    AutoRotateMode rtMode )
{
    _rotateMode = rtMode;
    _needsUpdate = true;
}

}