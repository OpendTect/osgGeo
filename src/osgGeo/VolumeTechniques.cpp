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

#include <osg/Texture1D>
#include <osg/Texture3D>
#include <osgVolume/VolumeTile>
#include <osgGeo/VolumeTechniques>
#include <osg/VertexProgram>
#include <osg/FragmentProgram>
#include <osg/TexGenNode>
#include <osgUtil/IntersectionVisitor>

#include <iostream>
#include <cstdio>

#if defined _MSC_VER && __cplusplus < 201103L
# define snprintf( a, n, ... ) _snprintf_s( a, n, _TRUNCATE, __VA_ARGS__ )
#endif


namespace osgGeo
{


BoundingGeometry::BoundingGeometry( osgVolume::VolumeTechnique& vt )
{
    _boundingBox.init();

    osgVolume::Locator* locator = vt.getVolumeTile()->getLocator();
    osg::ref_ptr<osg::Vec3Array> corners = new osg::Vec3Array();
    setVertexArray( corners );

    float eps = 1e-5;	// Solves interferences with boxdragger
    for ( unsigned int idx=0; idx<8; idx++ )
    {
	osg::Vec3 corner( idx&4 ? 1.0-eps : eps,
			  idx&2 ? 1.0-eps : eps,
			  idx&1 ? 1.0-eps : eps ); 

	corner = corner * locator->getTransform();
	corners->push_back( corner );
	_boundingBox.expandBy( corner );
    }

    GLubyte indices[] = {0,1,3,2, 0,2,6,5, 0,4,5,1, 1,5,7,3, 2,3,7,6, 4,6,7,5};
    addPrimitiveSet( new osg::DrawElementsUByte(GL_QUADS, 24, indices) );

    eps *= _boundingBox.radius();
    const osg::Vec3 margin( eps, eps, eps );

    _boundingBox.expandBy( _boundingBox.corner(0) - margin );
    _boundingBox.expandBy( _boundingBox.corner(7) + margin );

    dirtyBound();
}

// Binary compatibility hack, being appropriate for normal (OpendTect) usage
static osg::Vec4f _sharedBorderColor(1.0f,1.0f,1.0f,1.0f);


//=============================================================================


FixedFunctionTechnique::FixedFunctionTechnique():
    osgVolume::FixedFunctionTechnique()
{
}


FixedFunctionTechnique::FixedFunctionTechnique(const FixedFunctionTechnique& fft,const osg::CopyOp& copyop):
    osgVolume::FixedFunctionTechnique(fft,copyop)
{
}


void FixedFunctionTechnique::init()
{
    _boundingGeometry = new BoundingGeometry( *this );

    osgVolume::FixedFunctionTechnique::init();

    // Overwrite default filter settings of osgVolume::FixedFunctionTechnique

    osg::StateAttribute* attr0 = _node->getStateSet()->getTextureAttribute( 0, osg::StateAttribute::TEXTURE );
    osg::Texture3D* texture3D  = dynamic_cast<osg::Texture3D*>( attr0 );

    if ( texture3D )
    {
	osg::Texture::FilterMode minFilter = _volumeTile->getLayer()->getMinFilter();
	osg::Texture::FilterMode magFilter = _volumeTile->getLayer()->getMagFilter();
	texture3D->setFilter(osg::Texture3D::MIN_FILTER,minFilter);
	texture3D->setFilter(osg::Texture3D::MAG_FILTER,magFilter);

	texture3D->setWrap(osg::Texture3D::WRAP_R,osg::Texture3D::CLAMP_TO_BORDER);
	texture3D->setWrap(osg::Texture3D::WRAP_S,osg::Texture3D::CLAMP_TO_BORDER);
	texture3D->setWrap(osg::Texture3D::WRAP_T,osg::Texture3D::CLAMP_TO_BORDER);

	texture3D->setBorderColor( _sharedBorderColor );
    }

    /* Repair of OSG limitation: FixedFunctionTechnique does not distinguish
       between volume tile locator and image layer locator */
    osg::TexGenNode* tgn = dynamic_cast<osg::TexGenNode*>( _node.get() );
    if ( tgn && _volumeTile )
    {
	const osgVolume::Layer* layer = _volumeTile->getLayer();
	if ( layer )
	{
	    const osg::Matrix& mat = layer->getLocator()->getTransform();
	    tgn->getTexGen()->setPlanesFromMatrix(osg::Matrix::inverse(mat));
	}
    }
}


void FixedFunctionTechnique::traverse(osg::NodeVisitor& nv)
{
    osgUtil::IntersectionVisitor* iv = dynamic_cast<osgUtil::IntersectionVisitor*>( &nv );
    if ( iv )
    {
	osg::ref_ptr<osgUtil::Intersector> intersec = iv->getIntersector()->clone( *iv );
	if ( intersec.valid() && _boundingGeometry.valid() )
	    intersec->intersect( *iv, _boundingGeometry );
    }
    else
	osgVolume::FixedFunctionTechnique::traverse( nv );
}


void FixedFunctionTechnique::setBorderColor( const osg::Vec4f& borderColor )
{ _sharedBorderColor = borderColor; }


const osg::Vec4f& FixedFunctionTechnique::getBorderColor() const
{ return _sharedBorderColor; }


//=============================================================================


RayTracedTechnique::RayTracedTechnique( bool dynamicFragShading )
    : osgVolume::RayTracedTechnique()
    , _dynamicFragShader(0)
    , _fragShaderType(ColTab)
    , _colTabValueChannel(3)
    , _colTabUndefChannel(-1)
    , _colTabUndefValue(-1.0f)
    , _colTabUndefColor(1.0f,1.0f,1.0f,1.0f)
    , _invertColTabUndefChannel(false)
{
    for ( int idx=0; idx<4; idx++)
    {
	_dstEnabled[idx] = true;
	_srcChannel[idx] = idx;
    }

    if ( dynamicFragShading )
    {
	_dynamicFragShader = new osg::Shader( osg::Shader::FRAGMENT );
	updateFragShaderCode();
    }
}


RayTracedTechnique::RayTracedTechnique(const RayTracedTechnique& rtt,const osg::CopyOp& copyop)
    : osgVolume::RayTracedTechnique(rtt,copyop)
    , _boundingGeometry( rtt._boundingGeometry )
    , _dynamicFragShader(0)
    , _fragShaderType(rtt._fragShaderType)
    , _colTabValueChannel(rtt._colTabValueChannel)
    , _colTabUndefChannel(rtt._colTabUndefChannel)
    , _colTabUndefValue(rtt._colTabUndefValue)
    , _colTabUndefColor(rtt._colTabUndefColor)
    , _invertColTabUndefChannel(rtt._invertColTabUndefChannel)
{
    for ( unsigned int idx=0; idx<rtt._customShaders.size(); idx++ )
    {
	setCustomShader( _customShaders[idx]->getType(),
			 rtt._customShaders[idx]->getShaderSource().c_str() );
    }

    for ( int idx=0; idx<4; idx++ )
    {
	_dstEnabled[idx] = rtt._dstEnabled[idx];
	_srcChannel[idx] = rtt._srcChannel[idx];
    }

    if ( rtt._dynamicFragShader )
    {
	_dynamicFragShader = new osg::Shader( osg::Shader::FRAGMENT );
	updateFragShaderCode();
    }
}


void RayTracedTechnique::init()
{
    _boundingGeometry = new BoundingGeometry( *this );

    osgVolume::RayTracedTechnique::init();

    // Overwrite default filter settings of osgVolume::RayTracedTechnique

    osg::StateSet* stateSet = _transform->getChild(0)->getStateSet();
    osg::StateAttribute* attr0 = stateSet->getTextureAttribute( 0, osg::StateAttribute::TEXTURE );
    osg::Texture3D* texture3D  = dynamic_cast<osg::Texture3D*>( attr0 );

    if ( texture3D )
    {

	osg::Texture::FilterMode minFilter = _volumeTile->getLayer()->getMinFilter();
	osg::Texture::FilterMode magFilter = _volumeTile->getLayer()->getMagFilter();
	texture3D->setFilter(osg::Texture3D::MIN_FILTER,minFilter);
	texture3D->setFilter(osg::Texture3D::MAG_FILTER,magFilter);

	texture3D->setBorderColor( _sharedBorderColor );
    }

    osg::StateAttribute* attr1 = stateSet->getTextureAttribute( 1, osg::StateAttribute::TEXTURE );
    osg::Texture1D* tf_texture = dynamic_cast<osg::Texture1D*>( attr1 );

    if ( tf_texture )
    {
	tf_texture->setFilter(osg::Texture::MIN_FILTER, osg::Texture::NEAREST);
	tf_texture->setFilter(osg::Texture::MAG_FILTER, osg::Texture::NEAREST);
    }

    osg::StateAttribute* attr2 = stateSet->getAttribute( osg::StateAttribute::PROGRAM );
    osg::Program* program = dynamic_cast<osg::Program*>( attr2 );

    if ( _dynamicFragShader )
	_customShaders.push_back( _dynamicFragShader );

    for ( unsigned int idx=0; program && idx<_customShaders.size(); idx++ )
    {
	for ( int idy=program->getNumShaders()-1; idy>=0; idy-- )
	{
	    if ( _customShaders[idx]->getType()==program->getShader(idy)->getType() )
		program->removeShader( program->getShader(idy) );
	}

	program->addShader( _customShaders[idx] );
	stateSet->removeMode( GL_CULL_FACE );
    }

    if ( _dynamicFragShader )
	_customShaders.pop_back();
}


void RayTracedTechnique::traverse( osg::NodeVisitor& nv )
{
    osgUtil::IntersectionVisitor* iv = dynamic_cast<osgUtil::IntersectionVisitor*>( &nv );
    if ( iv )
    {
	osg::ref_ptr<osgUtil::Intersector> intersec = iv->getIntersector()->clone( *iv );
	if ( intersec.valid() && _boundingGeometry.valid() )
	    intersec->intersect( *iv, _boundingGeometry );
    }
    else
	osgVolume::RayTracedTechnique::traverse( nv );
}


void RayTracedTechnique::setBorderColor( const osg::Vec4f& borderColor )
{ _sharedBorderColor = borderColor; }


const osg::Vec4f& RayTracedTechnique::getBorderColor() const
{ return _sharedBorderColor; }


void RayTracedTechnique::setCustomShader( osg::Shader::Type type, const char* code )
{
    for ( unsigned int idx=0; idx<_customShaders.size(); idx++ )
    {
	if ( type==_customShaders[idx]->getType() )
	{
	    _customShaders[idx]->setShaderSource( code );
	    return;
	}
    }

    _customShaders.push_back( new osg::Shader(type,code) );
}


void RayTracedTechnique::setFragShaderType( FragShaderType fragShaderTyp )
{
    if ( _fragShaderType!=fragShaderTyp )
    {
	 _fragShaderType = fragShaderTyp;
	 updateFragShaderCode();
    }
}


int RayTracedTechnique::getFragShaderType() const
{
    return _fragShaderType;
}


void RayTracedTechnique::enableDestChannel( int dest, bool yn )
{
    if ( dest>=0 && dest<4 && _dstEnabled[dest]!=yn )
    {
	_dstEnabled[dest] = yn;
	updateFragShaderCode();
    }
}


bool RayTracedTechnique::isDestChannelEnabled( int dest ) const
{
    return dest>=0 && dest<4 ? _dstEnabled[dest] : false;
}


void RayTracedTechnique::setSourceChannel( int dest, int src )
{
    if ( dest>=0 && dest<4 && src>=0 && src<4 && _srcChannel[dest]!=src )
    {
	_srcChannel[dest] = src;
	updateFragShaderCode();
    }
}


int RayTracedTechnique::getSourceChannel( int dest ) const
{
    return dest>=0 && dest<4 ? _srcChannel[dest] : -1;
}


const osg::Vec4f& RayTracedTechnique::getChannelDefaults()
{
    static osg::Vec4f rgba( 0.0f, 0.0f, 0.0f, 1.0f );
    return rgba;
}


void RayTracedTechnique::setColTabValueChannel( int channel )
{
    if ( channel>=0 && channel<=3 )
    {
	_colTabValueChannel = channel;
	updateFragShaderCode();
    }
}


int RayTracedTechnique::getColTabValueChannel() const
{
    return _colTabValueChannel;
}


void RayTracedTechnique::setColTabUndefChannel( int channel )
{
    _colTabUndefChannel = channel>=0 && channel<=3 ? channel : -1;
    updateFragShaderCode();
}


int RayTracedTechnique::getColTabUndefChannel() const
{
    return _colTabUndefChannel;
}


void RayTracedTechnique::setColTabUndefValue( float undefVal )
{
    _colTabUndefValue = undefVal>=0.0f && undefVal<=1.0f ? undefVal : -1.0f;
    updateFragShaderCode();
}


float RayTracedTechnique::getColTabUndefValue() const
{
    return _colTabUndefValue;
}


void RayTracedTechnique::setColTabUndefColor( const osg::Vec4f& undefColor )
{
    for ( int idx=0; idx<4; idx++ )
    {
	_colTabUndefColor[idx] = undefColor[idx]<=0.0f ? 0.0f :
	    			 undefColor[idx]>=1.0f ? 1.0f : undefColor[idx];
    }
    updateFragShaderCode();
}


const osg::Vec4f& RayTracedTechnique::getColTabUndefColor() const
{
    return _colTabUndefColor;
}


void RayTracedTechnique::invertColTabUndefChannel( bool yn )
{
    _invertColTabUndefChannel = yn;
    updateFragShaderCode();
}


bool RayTracedTechnique::isColTabUndefChannelInverted() const
{
    return _invertColTabUndefChannel;
}


bool RayTracedTechnique::isShadingSupported()
{
    const int maxContextID = (int) osg::GraphicsContext::getMaxContextID();
    for( int contextID=0; contextID<=maxContextID; contextID++ )
    {
#if OSG_MIN_VERSION_REQUIRED(3,3,3)
	osg::GLExtensions* ext = osg::GLExtensions::Get( contextID, false );
	if ( ext && ext->isVertexProgramSupported && ext->isFragmentProgramSupported )
	    return true;
#else
	const osg::VertexProgram::Extensions* vertExt = osg::VertexProgram::getExtensions( contextID, false );
	 const osg::FragmentProgram::Extensions* fragExt = osg::FragmentProgram::getExtensions( contextID, false );

	 if ( vertExt && vertExt->isVertexProgramSupported() && fragExt && fragExt->isFragmentProgramSupported() )
	     return true;
#endif
    }

    return false;
}


//=============================================================================

static char volume_frag_depth_header[] =

"uniform sampler3D baseTexture;\n"
"\n"
"uniform sampler1D tfTexture;\n"
"uniform float tfScale;\n"
"uniform float tfOffset;\n"
"\n"
"uniform float SampleDensityValue;\n"
"uniform float TransparencyValue;\n"
"uniform float AlphaFuncValue;\n"
"\n"
"varying vec4 cameraPos;\n"
"varying vec4 vertexPos;\n"
"varying mat4 texgen;\n"
"varying vec4 baseColor;\n"
"\n"
"void main(void)\n"
"{ \n"
"    vec4 t0 = vertexPos;\n"
"    vec4 te = cameraPos;\n"
"\n"
"    if (te.x>=0.0 && te.x<=1.0 &&\n"
"        te.y>=0.0 && te.y<=1.0 &&\n"
"        te.z>=0.0 && te.z<=1.0)\n"
"    {\n"
"        // do nothing... te inside volume\n"
"    }\n"
"    else if ( !gl_FrontFacing )\n"				// modified
"    {\n"
"        te = t0 + 2.0 * normalize(vertexPos-cameraPos);\n"	// modified
"        if (te.x<0.0)\n"
"        {\n"
"            float r = -te.x / (t0.x-te.x);\n"
"            te = te + (t0-te)*r;\n"
"        }\n"
"\n"
"        if (te.x>1.0)\n"
"        {\n"
"            float r = (1.0-te.x) / (t0.x-te.x);\n"
"            te = te + (t0-te)*r;\n"
"        }\n"
"\n"
"        if (te.y<0.0)\n"
"        {\n"
"            float r = -te.y / (t0.y-te.y);\n"
"            te = te + (t0-te)*r;\n"
"        }\n"
"\n"
"        if (te.y>1.0)\n"
"        {\n"
"            float r = (1.0-te.y) / (t0.y-te.y);\n"
"            te = te + (t0-te)*r;\n"
"        }\n"
"\n"
"        if (te.z<0.0)\n"
"        {\n"
"            float r = -te.z / (t0.z-te.z);\n"
"            te = te + (t0-te)*r;\n"
"        }\n"
"\n"
"        if (te.z>1.0)\n"
"        {\n"
"            float r = (1.0-te.z) / (t0.z-te.z);\n"
"            te = te + (t0-te)*r;\n"
"        }\n"
"        vec4 temp = te; te = t0; t0 = temp;\n"			// modified
"    }\n"
"    else discard;\n"						// modified
"\n"
// begin modified
"    vec4 clip_pos = gl_ModelViewProjectionMatrix * t0;\n"
"    vec4 clip_eye = gl_ModelViewProjectionMatrix * te;\n"
// end modified
"\n"
"    t0 = t0 * texgen;\n"
"    te = te * texgen;\n"
"\n"
"    const float max_iteratrions = 2048.0;\n"
// begin modified (nVidia bug: length() & sqrt() may return NaN close to 0.0)
"    vec3 difvec = (te-t0).xyz/SampleDensityValue;\n"
"    float num_iterations = ceil( sqrt( max(4.0,dot(difvec,difvec)) ) );\n"
// end modified
"\n"
"    if (num_iterations>max_iteratrions) \n"
"    {\n"
"        num_iterations = max_iteratrions;\n"
"    }\n"
"\n"
"    vec3 deltaTexCoord=(te-t0).xyz/float(num_iterations-1.0);\n"
"    vec3 texcoord = t0.xyz;\n"
"\n"
// begin modified
"    vec4 delta_clip_pos = (clip_eye-clip_pos)/float(num_iterations-1.0);\n"
"    float sum_fn = gl_DepthRange.far+gl_DepthRange.near;\n"
"    float depth = gl_FragCoord.z;\n"
"    float max_r = 0.0;\n"
// end modified
"\n"
"    vec4 fragColor = vec4(0.0, 0.0, 0.0, 0.0); \n"
"    while(num_iterations>0.5)\n"	// bugfix to solve numerical instability
"    {\n";

//==============================================================================
// "     vec4 color = ... \n"	// INSERT DYNAMIC volume_frag_depth BODY HERE
//==============================================================================

static char volume_frag_depth_footer[] =

"\n"
"        float r = color[3]*TransparencyValue;\n"
"        if (r>AlphaFuncValue)\n"
"        {\n"
"            fragColor.xyz = fragColor.xyz*(1.0-r)+color.xyz*r;\n"
"            fragColor.w += r;\n"
"        }\n"
"\n"
"        if (fragColor.w<color.w)\n"
"        {\n"
"            fragColor = color;\n"
"        }\n"
"        texcoord += deltaTexCoord;\n"
"\n"
// begin modified
"        clip_pos += delta_clip_pos;\n"
	 // Impossible to be perfect, but any criterion is better than none
"        if ( r>AlphaFuncValue && r>=0.9*max_r )\n"
"        {\n"
"            depth = (gl_DepthRange.diff*clip_pos.z/clip_pos.w + sum_fn)/2.0;\n"
"        }\n"
"        max_r = max( max_r, r );\n"
// end modified
"\n"
"        --num_iterations;\n"
"    }\n"
"\n"
"    fragColor.w *= TransparencyValue;\n"
"    if (fragColor.w>1.0) fragColor.w = 1.0;\n"
"\n"
"    fragColor *= baseColor;\n"
"\n"
"    if (fragColor.w<AlphaFuncValue) discard;\n"
"    \n"
"    gl_FragColor = fragColor;\n"
// begin modified
"    if ( !gl_FrontFacing && depth<gl_FragCoord.z )\n"
"    {\n"				// Prevents being closer than dragger
"        depth = gl_FragCoord.z;\n"	// as result of numerical instability
"    }\n"
"    gl_FragDepth = depth;\n"
// end modified
"}\n"
"\n";


static char volume_coltab_frag_body [] =

"        float v = texture3D( baseTexture, texcoord).a * tfScale + tfOffset;\n"
"        vec4 color = texture1D( tfTexture, v);\n";


const char* RayTracedTechnique::volumeTfFragDepthCode()
{
    static std::string code;

    code = volume_frag_depth_header;
    code += volume_coltab_frag_body;
    code += volume_frag_depth_footer;

    return code.c_str();
}


void RayTracedTechnique::updateFragShaderCode()
{
    if ( !_dynamicFragShader )
	return;

    char line[100];

    std::string code = volume_frag_depth_header;

    if ( _fragShaderType==ColTab )
    {
	if ( _colTabUndefChannel<0 )
	{
	    snprintf( line, 100, "        float v = texture3D( baseTexture, texcoord)[%d] * tfScale + tfOffset;\n", _colTabValueChannel );
	    code += line;
	    code += "        vec4 color = texture1D( tfTexture, v );\n";
	}
	else
	{
	    code += "        vec4 src = texture3D( baseTexture, texcoord );\n";
	    snprintf( line, 100, "        float v = src[%d];\n", _colTabValueChannel );
	    code += line;
	    snprintf( line, 100, "        float udf = src[%d];\n", _colTabUndefChannel );
	    code += line;

	    if ( _invertColTabUndefChannel )
		code += "        udf = 1.0 - udf;\n";

	    if ( _colTabUndefValue>0.0f )
	    {
		code += "        if ( udf<1.0 )\n";
		snprintf( line, 100, "            v = (v - udf*%.6f) / (1.0-udf);\n", _colTabUndefValue );
		code += line;
	    }

	    code += "\n"
		    "        vec4 color = texture1D( tfTexture, v*tfScale+tfOffset );\n";
	    snprintf( line, 100, "        vec4 udfcol = vec4(%.6f,%.6f,%.6f,%.6f);\n", _colTabUndefColor[0], _colTabUndefColor[1], _colTabUndefColor[2], _colTabUndefColor[3] );
	    code += line;

	    code += "\n"
		    "        if ( udf >= 1.0 )\n"
		    "            color = udfcol;\n"
		    "        else if ( udf > 0.0 )\n";

	    if ( _colTabUndefColor[3]<=0.0f )
		code += "            color.a *= 1.0-udf;\n";
	    else
	    {
		code += "        {\n"
			"            if ( color.a > 0.0 )\n"
			"            {\n"
			"                float a = color.a;\n"
			"                color.a = mix( a, udfcol.a, udf );\n"
			"                color.rgb = mix(a*color.rgb, udfcol.a*udfcol.rgb, udf) / color.a;\n"
			"            }\n"
			"            else\n"
			"                color = vec4( udfcol.rgb, udf*udfcol.a );\n"
			"        }\n";
	    }
	}
    }
    else if ( getFragShaderType()==RGBA )
    {
	code += "        vec4 src = texture3D( baseTexture, texcoord );\n";

	const osg::Vec4f rgba = getChannelDefaults();
	snprintf( line, 100, "        vec4 color = vec4(%.6f,%.6f,%.6f,%.6f);\n", rgba[0], rgba[1], rgba[2], rgba[3] );
	code += line;

	for ( int idx=0; idx<4; idx++ )
	{
	    if ( !_dstEnabled[idx] )
		continue;

	    snprintf( line, 100, "        color[%d] = src[%d];\n", idx, _srcChannel[idx] );
	    code += line;
	}
    }
    else // _fragShaderType==Identity
    {
	code += "        vec4 color = texture3D( baseTexture, texcoord );\n";
    }

    code += volume_frag_depth_footer;

    _dynamicFragShader->setShaderSource( code );

    //std::cout << code << std::endl;
}


} // end namespace

