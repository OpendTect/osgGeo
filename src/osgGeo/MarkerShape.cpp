/* osgGeo - A collection of geoscientific extensions to OpenSceneGraph.
Copyright 2011- dGB Beheer B.V.

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

#include "osgGeo/MarkerShape"
#include <osg/LineWidth>
#include <osg/Point>
#include <osg/VertexProgram>
#include <osg/FragmentProgram>
#include <osg/Version>

#include "GLInfo"


using namespace osgGeo;

MarkerShape::MarkerShape()
    : _hints(new osg::TessellationHints)
    , _size(1.0f)
    , _heightRatio(1.0f)
    , _shapeType(Box)
{
    _center = osg::Vec3(0.0f,0.0f,0.0f);
    _hints->setDetailRatio(0.5f);
}


MarkerShape::~MarkerShape()
{
}


void MarkerShape::setSize(float size)
{
    _size = size;
}


void MarkerShape::setHeightRatio(float heightRatio)
{
    _heightRatio = heightRatio;
}


void MarkerShape::setDetail(float ratio) 
{
    _hints->setDetailRatio(ratio);
}


float MarkerShape::getDetail() const
{
    return _hints->getDetailRatio();
}


void MarkerShape::setType(ShapeType type)
{
    _shapeType = type;
}

void MarkerShape::setCenter(const osg::Vec3& center)
{
    _center = center;
}


osg::ref_ptr<osg::Drawable> MarkerShape::createShape( const osg::Vec4& color,
						      const osg::Quat& rot ) const
{
    osg::ref_ptr<osg::Drawable> drwB(0);
    osg::ref_ptr<osg::ShapeDrawable> shapeDrwB(0);

    const float radius = 0.5 * _size;
    const float height = _heightRatio * _size;

    switch (_shapeType)
    {
    case Box:
	{
	    osg::ref_ptr<osg::Box> box = new osg::Box( _center, _size, _size, height );
	    box->setRotation( rot );
	    shapeDrwB = new osg::ShapeDrawable( box, _hints );
	    break;
	}
    case Cone:
	{
	    osg::ref_ptr<osg::Cone> cone = new osg::Cone( _center, radius, height );
	    cone->setRotation( rot );
	    shapeDrwB = new osg::ShapeDrawable( cone,  _hints );
	    break;
	}
    case Sphere:
	{
	    osg::ref_ptr<osg::Sphere> sphere = new osg::Sphere( _center, radius );
	    shapeDrwB = new osg::ShapeDrawable( sphere, _hints );
	    break;
	}
    case Cylinder:
	{
	    osg::ref_ptr<osg::Cylinder> cyl = new osg::Cylinder( _center, radius, height );
	    cyl->setRotation( rot );
	    shapeDrwB = new osg::ShapeDrawable( cyl, _hints );
	    break;
	}
    case Plane:
	{
	    osg::ref_ptr<osg::Box> plane =
			    new osg::Box( _center, 3*_size, 2*_size, _size/2 );
	    plane->setRotation( rot );
	    shapeDrwB = new osg::ShapeDrawable( plane, _hints );
	    break;
	}
    case Cross:
	{
	    drwB = createCrossDrawable( color );
	    break;
	}
    case Arrow:
	{
	    drwB = createArrowDrawable( color, rot );
	    break;
	}
    default:
	return 0;
    }

    if ( _shapeType < Cross && shapeDrwB )
    {
	shapeDrwB->setColor( color );
	drwB = shapeDrwB;
    }
    
    return drwB;
}


osg::ref_ptr<osg::Drawable>  MarkerShape::createCrossDrawable(const osg::Vec4& col) const
{
    osg::ref_ptr<osg::Geometry> crossGeometry = new osg::Geometry;
    osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array(6);

    const float radius = 0.5 * _size;
    const float x1 = _center[0] - radius;
    const float x2 = _center[0] + radius;
    const float y1 = _center[1] - radius;
    const float y2 = _center[1] + radius;
    const float z1 = _center[2] - radius;
    const float z2 = _center[2] + radius;

    (*vertices)[0] = osg::Vec3(0.0f,0.0f,z1);
    (*vertices)[1] = osg::Vec3(0.0f,0.0f,z2);
    (*vertices)[2] = osg::Vec3(x1,0.0f,0.0f);
    (*vertices)[3] = osg::Vec3(x2,0.0f,0.0f);
    (*vertices)[4] = osg::Vec3(0.0f,y1,0.0f);
    (*vertices)[5] = osg::Vec3(0.0f,y2,0.0f);

    crossGeometry->setVertexArray(vertices);
    crossGeometry->addPrimitiveSet(
	new osg::DrawArrays(osg::PrimitiveSet::LINES,0,6));
    osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
    colors->push_back(col);
    crossGeometry->setColorArray(colors);
    crossGeometry->setColorBinding(osg::Geometry::BIND_OVERALL);

    osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array;
    normals->push_back(osg::Vec3(0.0f,-1.0f,0.0f));
    crossGeometry->setNormalArray(normals);
    crossGeometry->setNormalBinding(osg::Geometry::BIND_OVERALL);

    osg::ref_ptr<osg::LineWidth> lineWidth = new osg::LineWidth;
    lineWidth->setWidth(2.0);
    crossGeometry->getOrCreateStateSet()->setAttributeAndModes(lineWidth);

    return crossGeometry;
}


osg::ref_ptr<osg::Drawable>  MarkerShape::createArrowDrawable( const osg::Vec4& col,
							       const osg::Quat& rot ) const
{
    osg::ref_ptr<osg::Vec3Array> coords = new osg::Vec3Array;
    osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array;
    const int resolution = 4;
    osg::Vec3 curu,curv;
    const osg::Vec3 dir = rot * osg::Vec3(1,0,0);
    
    osg::Vec3 anyvec(1,1,1); 
    curu = dir ^ anyvec;
    curu.normalize();
    curv = dir ^ curu;
    curv.normalize();

    const float len = _size * 4;
    const float conelen = len/2;
    osg::Vec3 p1, p2, p3;
    p1 = dir * len;
    p2 = dir * ( len - conelen );
   
    osg::Vec3 normdir( dir );
    normdir.normalize();
    osg::ref_ptr<osg::DrawElementsUInt> cone =
	new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLE_STRIP, 0); 
    cone->push_back( 0 );
    int ci = 0;
    const float rad = len/8;
    for ( int idx=0; idx<=resolution; idx++ )
    {
	float angl = idx * 2 * M_PI / resolution;
	osg::Vec3 vec = ( curu * cos(angl) ) + ( curv * sin(angl) );
	osg::Vec3 norm = vec + normdir;
	norm.normalize();
	normals->push_back( norm*2.0 );
	coords->push_back( p1 );
	cone->push_back( ci++ );
	coords->push_back( p2 + vec*rad   );
	normals->push_back( norm*2.0 );
	cone->push_back( ci++ );
    }

    //cone base
    osg::ref_ptr<osg::DrawElementsUInt> conebase =
	new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLE_FAN, 0); 
    const unsigned int conesz = coords->size();
    for ( unsigned int idx=0; idx<=resolution; idx++ )
    {
	float angl = idx * 2 * M_PI / resolution;
	osg::Vec3 vec = ( curu * cos(angl) ) + ( curv * sin(angl) );
	coords->push_back( p2 + vec*rad   );
	normals->push_back( -normdir );
	conebase->push_back( idx+conesz );
    }
    
    // cylinder
    const int platesz = coords->size();
    for ( int idx=0; idx<=resolution; idx++ )
    {
	float angl = idx * 2 * M_PI / resolution;
	osg::Vec3 vec = ( curu * cos(angl) ) + ( curv * sin(angl) );
	coords->push_back( p2 + vec*rad/2   );
	normals->push_back( vec );
	coords->push_back( vec*rad/2   );
	normals->push_back( vec );
    }

    const int cylsz = coords->size() - platesz;
    osg::ref_ptr<osg::DrawElementsUInt> cylbase =
	new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLE_FAN, 0); 
    const unsigned sz = coords->size();
    for ( unsigned int idx=0; idx<=resolution; idx++ )
    {
	float angl = idx * 2 * M_PI / resolution;
	osg::Vec3 vec = ( curu * cos(angl) ) + ( curv * sin(angl) );
	coords->push_back( vec*rad/2   );
	normals->push_back( -normdir );
	cylbase->push_back( idx+sz );
    }

    osg::ref_ptr<osg::Geometry> arrowgeometry = new osg::Geometry();
    arrowgeometry->setVertexArray( coords );
     
    arrowgeometry->addPrimitiveSet( cone );
    arrowgeometry->addPrimitiveSet( conebase );

    arrowgeometry->addPrimitiveSet( 
	new osg::DrawArrays( GL_TRIANGLE_STRIP, platesz,
			       cylsz, 0 ));
    arrowgeometry->addPrimitiveSet( cylbase );
    
    arrowgeometry->setNormalArray( normals.get() );
    arrowgeometry->setNormalBinding( osg::Geometry::BIND_PER_VERTEX );
    osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array; 
    colors->push_back( col );
    arrowgeometry->setColorArray( colors );
    arrowgeometry->setColorBinding(osg::Geometry::BIND_OVERALL);
    return arrowgeometry;
}


osg::ref_ptr<osg::Geode> MarkerShape::createPoints( osg::Vec3Array* coords,
						    osg::Vec4Array* colors )
{
    osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry;
    geometry->addPrimitiveSet( new osg::DrawArrays(GL_POINTS,0,coords->size()) );
    geometry->setVertexArray( coords );
    geometry->setColorArray( colors, osg::Array::BIND_PER_VERTEX );
    osg::StateSet* state = geometry->getOrCreateStateSet();
    state->setAttribute( new osg::Point(_size) );
    state->setMode( GL_LIGHTING, osg::StateAttribute::OFF );
    if ( isShadingSupported() )
	addShader( state, _shapeType );

    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    geode->addDrawable( geometry.get() );
    geode->ref();
    return geode.get();
}


bool MarkerShape::isShadingSupported() const
{
    const GLInfo* glinfo = GLInfo::get();
    if ( !glinfo )
	return false;

    const bool issupported = glinfo->isVertexProgramSupported()
			     && glinfo->isShaderProgramSupported()
			     && glinfo->isGeometryShader4Supported();
    return issupported;
}


void MarkerShape::addShader( osg::StateSet* state, ShapeType shapetype )
{
    if ( shapetype != Box && shapetype != Sphere )
	return;

    	osg::ref_ptr<osg::Program> program = new osg::Program;
	osg::ref_ptr<osg::Shader> vertshader;
	osg::ref_ptr<osg::Shader> geomshader;

	if ( shapetype == Box )
	{
	    vertshader = new osg::Shader( osg::Shader::VERTEX, getVertexShader() );
	    geomshader = new osg::Shader( osg::Shader::GEOMETRY, getGeomShaderBox() );
	    program->setParameter( GL_GEOMETRY_VERTICES_OUT_EXT, 30 );
	    program->setParameter( GL_GEOMETRY_INPUT_TYPE_EXT, GL_POINTS );
	    program->setParameter( GL_GEOMETRY_OUTPUT_TYPE_EXT, GL_QUADS );

	    float ln = 20*_size; // 20 is just a good guess to make it visible
	    float wd = 20*_size;
	    float ht = 20*_size;

	    state->addUniform( new osg::Uniform("ln",ln) );
	    state->addUniform( new osg::Uniform("wd",wd) );
	    state->addUniform( new osg::Uniform("ht",ht) );
	}
	else if ( shapetype == Sphere )
	{
	    vertshader = new osg::Shader( osg::Shader::VERTEX, getVertexShader() );
	    geomshader = new osg::Shader( osg::Shader::GEOMETRY, getGeomShaderSphere() );
	    program->setParameter( GL_GEOMETRY_VERTICES_OUT_EXT, 128 );
	    program->setParameter( GL_GEOMETRY_INPUT_TYPE_EXT,	GL_POINTS );
	    program->setParameter( GL_GEOMETRY_OUTPUT_TYPE_EXT, GL_TRIANGLE_STRIP );
	    state->addUniform( new osg::Uniform("radius",20*_size) );
	}

	program->addShader( vertshader.get() );
	program->addShader( geomshader.get() );
	state->setAttributeAndModes( program.get() );
}


const char* MarkerShape::getVertexShader()
{
    static const char* vertexsrc = 
    {
	"#version 120\n"
	"#extension GL_EXT_geometry_shader4 : enable\n"
	"void main()\n"
	"{\n"
	" gl_Position = ftransform();\n"
	" gl_FrontColor = gl_Color;\n"
	"}\n"
    };

    return vertexsrc;
}


const char* MarkerShape::getGeomShaderBox()
{
    static const char* geomsrc =
    {
	"#version 120\n"
	"#extension GL_EXT_geometry_shader4 : enable\n"
	"uniform float ln;\n"
	"uniform float wd;\n"
	"uniform float ht;\n"
	"vec4 computeDiffuseColor( vec3 normal )\n"
	"{\n"
	"    vec3 norm = normalize( gl_NormalMatrix * normal );\n"
	"    return ( max( dot( norm, vec3( gl_LightSource[0].position ) ), 0.0 ) * gl_FrontColorIn[0] );\n"
	"}\n"
	"void main()\n"
	"{\n"
	" float l = ln/2; \n"
	" float w = wd/2; \n"
	" float h = ht/2; \n"
	" vec4 fcolor;\n"
	" for ( int idx=0; idx<gl_VerticesIn; idx++ )\n"
	" {\n"
	"   vec4[8] vertices;\n"
	"   vec3[6] normals;\n"
	"   vec4 center = gl_PositionIn[idx];\n"
	"   vertices[0] = gl_ModelViewProjectionMatrix * ( vec4( -l, -w, -h, 0 ));\n"
	"   vertices[1] = gl_ModelViewProjectionMatrix * ( vec4( -l,  w, -h, 0 ));\n"
	"   vertices[2] = gl_ModelViewProjectionMatrix * ( vec4(  l,  w, -h, 0 ));\n"
	"   vertices[3] = gl_ModelViewProjectionMatrix * ( vec4(  l, -w, -h, 0 ));\n"
	"   vertices[4] = gl_ModelViewProjectionMatrix * ( vec4( -l, -w,  h, 0 ));\n"
	"   vertices[5] = gl_ModelViewProjectionMatrix * ( vec4( -l,  w,  h, 0 ));\n"
	"   vertices[6] = gl_ModelViewProjectionMatrix * ( vec4(  l,  w,  h, 0 ));\n"
	"   vertices[7] = gl_ModelViewProjectionMatrix * ( vec4(  l, -w,  h, 0 ));\n"
	"   \n"
	"   normals[0] = vec3(  0,  0, -1 );\n"
	"   normals[1] = vec3( -1,  0,  0 );\n"
	"   normals[2] = vec3(  0,  0,  1 );\n"
	"   normals[3] = vec3(  1,  0,  0 );\n"
	"   normals[4] = vec3(  0,  1,  0 );\n"
	"   normals[5] = vec3(  0, -1,  0 );\n"
	"   int indices[30] = int[]( 3, 2, 1, 0, 3, \n"
	"			     0, 1, 5, 4, 0, \n"
	"			     4, 5, 6, 7, 4, \n"
	"			     7, 6, 2, 3, 7, \n"
	"			     1, 2, 6, 5, 1, \n"
	"			     0, 4, 7, 3, 0 );\n"
    	"   \n"
	"   int ci = 0;\n"
	"   for ( int i=0; i<6; i++ )\n"
	"   {\n"
	"	fcolor = computeDiffuseColor( normals[i] );\n"
	"	for ( int j=0; j<5; j++ )\n"
	"	{\n"
	"	    gl_Position = center + vertices[indices[ci++]];\n"
	"	    gl_FrontColor = fcolor;\n"
	"	    EmitVertex();\n"
	"	}\n"
	"	EndPrimitive();\n"
	"   }\n"
	" }\n"
	"}\n"
    };

    return geomsrc;
}


const char* MarkerShape::getGeomShaderSphere()
{
    static const char* geomsrc =
    {
	"#version 120\n"
	"#extension GL_EXT_geometry_shader4 : enable\n"
	"uniform float radius;\n"
	"vec4 computeDiffuseColor( vec3 normal )\n"
	"{\n"
	"    vec3 tNormal = normalize( gl_NormalMatrix * normal );\n"
	"    return ( max( dot( tNormal, vec3( gl_LightSource[0].position ) ), 0.0 ) * gl_FrontColorIn[0] );\n"
	"}\n" 
	"void main()\n"
	"{\n"
	" int res = 7; \n"
	" float angly = 0.0;\n"
	" float anglx = 0.0;\n"
	" float rs = 0.0;\n"
	" float hs = 0.0;\n"
	" float rf = 0.0;\n"
	" float hf = 0.0;\n"
	" for ( int iv=0; iv<gl_VerticesIn; iv++ )\n"
	" {\n"
	"   vec4 center = gl_PositionIn[iv];\n"
	"   for ( int j=0; j<=res; j++ ) \n"
	"   {\n"
	"       angly = float(j) * 3.14159 / float(res);\n"
	"	rs = radius * sin( angly );\n"
	"	hs = radius * cos( angly );\n"
	"	for ( int i=0; i<=res; i++ )\n"
	"	{\n"
	"	    float anglx = float(i) * 2.0 * 3.14159 / float(res); \n"
	"           if ( j!= 0 )\n"
	"	    {\n"
	"		vec3 v1 = vec3( sin(anglx)*rf, hf, cos(anglx)*rf );\n"
	"		vec4 vec1 = gl_ModelViewProjectionMatrix * vec4( v1, 0 );\n"
	"		gl_FrontColor = computeDiffuseColor( v1 ); \n"
	"		gl_Position = center + vec1;\n"
	"		EmitVertex();\n"
	"		vec3 v2 = vec3( sin(anglx)*rs, hs, cos(anglx)*rs );"		
	"		vec4 vec2 = gl_ModelViewProjectionMatrix * vec4( v2, 0 );\n"
	"		gl_FrontColor = computeDiffuseColor( v2 ); \n"
	"		gl_Position = center + vec2;\n"
	"		EmitVertex();\n"
        "	    }\n"
	"	}\n"
	"       rf=rs;\n"
	"	hf=hs;\n"
	"       EndPrimitive();\n"
	"   }\n"
	" }\n"
	"}\n"
    };

    return geomsrc;
}

