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

#include <osgGeo/LayerProcess>
#include <osgGeo/LayeredTexture>
#include <osg/Vec2f>
#include <cstdio>

#if defined _MSC_VER && __cplusplus < 201103L
# define snprintf( a, n, ... ) _snprintf_s( a, n, _TRUNCATE, __VA_ARGS__ )
#endif


namespace osgGeo
{


TransparencyType getTransparencyTypeBytewise( const unsigned char* start, const unsigned char* stop, int step )
{
    bool foundOpaquePixel = false;
    bool foundTransparentPixel = false;

    for ( const unsigned char* ptr=start; ptr<=stop; ptr+=step )
    {
	if ( *ptr==0 )
	    foundTransparentPixel = true;
	else if ( *ptr==255 )
	    foundOpaquePixel = true;
	else
	    return HasTransparencies;
    }

    if ( foundTransparentPixel )
	return foundOpaquePixel ? OnlyFullTransparencies : FullyTransparent;

    return Opaque;
}


TransparencyType addOpacity( TransparencyType tt, float opacity )
{
    if ( tt==TransparencyUnknown || opacity<0.0 )
	return tt;

    if ( opacity<=0.0f )
	return tt==Opaque ? OnlyFullTransparencies : tt;

    if ( opacity>=1.0f )
	return tt==FullyTransparent ? OnlyFullTransparencies : tt;

    return HasTransparencies;
}


TransparencyType multiplyOpacity( TransparencyType tt, float opacity )
{
    if ( tt==TransparencyUnknown )
	return tt;

    if ( opacity<=0.0f )
	return FullyTransparent;

    if ( opacity>=1.0f )
	return tt;

    return tt==FullyTransparent ? tt : HasTransparencies;
}


//============================================================================


ColorSequence::ColorSequence( unsigned char* array )
    : _arr( array )
    , _dirtyCount( 0 )
    , _transparencyType( TransparencyUnknown )
{
    if ( array )
	setRGBAValues( array );
}


ColorSequence::~ColorSequence()
{}


void ColorSequence::setRGBAValues( unsigned char* array )
{
    _arr = array;
    touch();
}


void ColorSequence::touch()
{
    _dirtyCount++;
    _transparencyType = TransparencyUnknown;
    triggerRedrawRequest();
}


TransparencyType ColorSequence::getTransparencyType() const
{
    if ( _transparencyType==TransparencyUnknown )
    {
	if ( !_arr )
	    _transparencyType = FullyTransparent;
	else 
	    _transparencyType = getTransparencyTypeBytewise( _arr+3, _arr+1023, 4 );
    }

    return _transparencyType;
}


//============================================================================


LayerProcess::LayerProcess( LayeredTexture& layTex )
    : _layTex( layTex )
    , _colSeqPtr( 0 )
    , _newUndefColor( 1.0f, 1.0f, 1.0f, 1.0f )
    , _opacity( 1.0f )
    , _colSeqTexSamplingStart( 0.0f )
    , _colSeqTexSamplingStep( 0.0f )
{}


const unsigned char* LayerProcess::getColorSequencePtr() const
{ return _colSeqPtr; }


void LayerProcess::setColorSequenceTextureSampling( float start, float step )
{
    _colSeqTexSamplingStart = start;
    _colSeqTexSamplingStep = step;
    _layTex.updateSetupStateSet();
}


float LayerProcess::getOpacity() const
{ return _opacity; }


void LayerProcess::setOpacity( float opac )
{
    _opacity = opac<=0.0f ? 0.0f : ( opac>=1.0f ? 1.0f : opac );
    _layTex.updateSetupStateSet();
}


void LayerProcess::setNewUndefColor( const osg::Vec4f& color )
{
    for ( int idx=0; idx<4; idx++ )
    {
	_newUndefColor[idx] = color[idx]<=0.0f ? 0.0f :
			      color[idx]>=1.0f ? 1.0f : color[idx];
    }

    _layTex.updateSetupStateSet();
}


const osg::Vec4f& LayerProcess::getNewUndefColor() const
{ return _newUndefColor; }


void LayerProcess::assignOrgCol3IfNeeded( std::string& code, int toIdx ) const
{
    if ( toIdx!=-1 && toIdx!=3 )
	return;
    if ( !isUndefPerChannel() || _newUndefColor[3]<=0.0f )
	return;
    if ( _newUndefColor[3]>=1.0f && getTransparencyType(true)==Opaque )
	return;

    if ( toIdx==3 )
	code += "    ";

    code += "    orgcol3 = col[3];\n";
} 


void LayerProcess::getHeaderCode( std::string& code, int& nrUdf, int id, int toIdx, int fromIdx ) const
{
    const int unit = _layTex.getDataLayerTextureUnit(id);
    const int udfId = _layTex.getDataLayerUndefLayerID(id);

    char line[100];

    char to[5] = ""; 
    char from[5] = ""; 
    if ( toIdx>=0 )
    {
	snprintf( to, 5, "[%d]", toIdx );
	snprintf( from, 5, "[%d]", fromIdx );
    }

    code += nrUdf ? "    if ( udf < 1.0 )\n"
		  : "    if ( true )\n";

    code += "    {\n";

    if ( _layTex.isDataLayerOK(udfId) )
    {
	if ( nrUdf )
	    code += "        oldudf = udf;\n";

	const int udfUnit = _layTex.getDataLayerTextureUnit( udfId );
	snprintf( line, 100, "        texcrd = gl_TexCoord[%d].st;\n", udfUnit );
	code += line;
	const int udfChannel = _layTex.getDataLayerUndefChannel(id);
	snprintf( line, 100, "        udf = texture2D( texture%d, texcrd )[%d];\n", udfUnit, udfChannel );
	code += line;
	if ( _layTex.areUndefLayersInverted() )
	    code += "        udf = 1.0 - udf;\n";

	code += "\n"
		"        if ( udf < 1.0 )\n"
		"        {\n";

	if ( udfId!=id )
	{
	    snprintf( line, 100, "            texcrd = gl_TexCoord[%d].st;\n", unit );
	    code += line;
	}
	snprintf( line, 100, "            col%s = texture2D( texture%d, texcrd )%s;\n", to, unit, from );
	code += line;

	const osg::Vec4f& udfColor = _layTex.getDataLayerImageUndefColor(id);
	if ( toIdx<0 )
	{
	    std::string ext = ".";
	    for ( int idx=0; idx<4; idx++ )
	    {
		if ( udfColor[idx]>=0.0f )
		    ext += idx==0 ? 'r' : idx==1 ? 'g' : idx==2 ? 'b' : 'a';
	    }

	    if ( ext.size()>1 )
	    {
		if ( ext.size()>4 )
		    ext.clear();

		snprintf( line, 100, "            udfcol = vec4(%.6f,%.6f,%.6f,%.6f);\n", udfColor[0], udfColor[1], udfColor[2], udfColor[3] );
		code += line;
		code += "            if ( udf > 0.0 )\n";
		snprintf( line, 100, "                col%s = (col%s - udf*udfcol%s) / (1.0-udf);\n", ext.data(), ext.data(), ext.data() );
		code += line;
	    }
	}
	else if ( udfColor[fromIdx]>=0.0f )
	{
	    code += "            if ( udf > 0.0 )\n";
	    snprintf( line, 100, "                col%s = (col%s - %.6f*udf) / (1.0-udf);\n", to, to, udfColor[fromIdx] );
	    code += line;
	}

	const bool stackUdf = _layTex.isDataLayerOK( _layTex.getStackUndefLayerID() );
	if ( stackUdf )
	{
	    code += "\n"
		    "            if ( udf > stackudf )\n"
		    "                udf = (udf-stackudf) / (1.0-stackudf);\n"
		    "            else\n"
		    "                udf = 0.0;\n";
	}

	code += "        }\n"
	    	"\n";

	assignOrgCol3IfNeeded( code, toIdx );

	if ( isUndefPerChannel() )
	{
	    code += "        if ( udf > 0.0 )\n";

	    if ( _newUndefColor[3]>0.0f )
	    {
		const bool resultIsOpaque = _newUndefColor[3]>=1.0f && getTransparencyType(true)==Opaque;
		if ( toIdx==3 || resultIsOpaque )
		{
		    snprintf( line, 100, "            col%s = mix( col%s, %.6f, udf );\n", to, to, _newUndefColor[toIdx] );
		}
		else
		{
		    snprintf( line, 100, "            col%s = mix(orgcol3*col%s, %.6f, udf) / col[3];\n", to, to, _newUndefColor[3]*_newUndefColor[toIdx] );
		}

		code += line;
	    }
	    else if ( toIdx==3 )
		code += "            col[3] *= 1.0-udf;\n";
	}
	else if ( nrUdf++ )
	    code += "        udf = max( udf, oldudf );\n";
    }
    else
    {
	snprintf( line, 100, "        texcrd = gl_TexCoord[%d].st;\n", unit );
	code += line;
	snprintf( line, 100, "        col%s = texture2D( texture%d, texcrd )%s;\n", to, unit, from );
	code += line;
	assignOrgCol3IfNeeded( code, toIdx );
    }

    code += "    }\n";
}


void LayerProcess::getFooterCode( std::string& code, int& nrUdf, int stage ) const
{
    char line[100];

    if ( nrUdf )
    {
	snprintf( line, 100, "    udfcol = vec4(%.6f,%.6f,%.6f,%.6f);\n", _newUndefColor[0], _newUndefColor[1], _newUndefColor[2], _newUndefColor[3] );
	code += line;

	code += "\n"
		"    if ( udf >= 1.0 )\n"
	    	"        col = udfcol;\n"
		"    else if ( udf > 0.0 )\n";

	if ( _newUndefColor[3]<=0.0f )
	    code += "        col.a *= 1.0-udf;\n";
	else if ( _newUndefColor[3]>=1.0f && getTransparencyType(true)==Opaque )
	    code += "        col = mix( col, udfcol, udf );\n";
	else
	{
	    code += "    {\n"
		    "        if ( col.a > 0.0 )\n"
		    "        {\n"
		    "            a = col.a;\n"
		    "            col.a = mix( a, udfcol.a, udf );\n"
		    "            col.rgb = mix(a*col.rgb, udfcol.a*udfcol.rgb, udf) / col.a;\n"
		    "        }\n"
		    "        else\n"
		    "            col = vec4( udfcol.rgb, udf*udfcol.a );\n"
		    "    }\n";
	}

	code += "\n";
	nrUdf = 0;
    }

    if ( _opacity<1.0 )
    {
	snprintf( line, 100, "    col.a *= %.6f;\n", _opacity );
	code += line;
    }

    if ( stage )
    {
	code += "    a = gl_FragColor.a;\n"
	    	"    b = col.a * (1.0-a);\n"
		"    gl_FragColor.a += b;\n"
		"    if ( gl_FragColor.a > 0.0 )\n"
		"        gl_FragColor.rgb = (a*gl_FragColor.rgb+b*col.rgb) / gl_FragColor.a;\n";
    }
    else
	code += "    gl_FragColor = col;\n";
}


void LayerProcess::processHeader( osg::Vec4f& col, float& udf, float stackUdf, const osg::Vec2f& coord, int id, int toIdx, int fromIdx, float* orgCol3 ) const
{
    if ( udf>=1.0f )
	return;

    const int udfId = _layTex.getDataLayerUndefLayerID(id);

    if ( _layTex.isDataLayerOK(udfId) )
    {
	const float oldUdf = udf;
	const int udfChannel = _layTex.getDataLayerUndefChannel(id);
	udf = _layTex.getDataLayerTextureVec(udfId,coord)[udfChannel];
	if ( _layTex.areUndefLayersInverted() )
	    udf = 1.0-udf;

	if ( udf<1.0f )
	{
	    const osg::Vec4f& udfCol = _layTex.getDataLayerImageUndefColor(id);

	    if ( toIdx<0 )
	    {
		col = _layTex.getDataLayerTextureVec( id, coord );
		for ( int idx=0; idx<4; idx++ )
		{
		    if ( udf>0.0f && udfCol[idx]>=0.0f )
			col[idx] = (col[idx] - udfCol[idx]*udf) / (1.0f-udf);
		}
	    }
	    else
	    {
		col[toIdx] = _layTex.getDataLayerTextureVec(id,coord)[fromIdx];
		if ( udf>0.0f && udfCol[fromIdx]>=0.0f )
		    col[toIdx] = (col[toIdx]-udfCol[fromIdx]*udf) / (1.0f-udf);
	    }

	    udf = udf>stackUdf ? (udf-stackUdf)/(1.0f-stackUdf) : 0.0f;
	}

	if ( isUndefPerChannel() && orgCol3 )
	{
	    if ( toIdx==3 )
		*orgCol3 = col[3];

	    if ( udf>0.0f )
	    {
		if ( _newUndefColor[3]>0.0f )
		{
		    const bool resultIsOpaque = _newUndefColor[3]>=1.0f && col[3]>=1.0f;
		    if ( toIdx==3 || resultIsOpaque )
		    {
			col[toIdx] = col[toIdx]*(1.0f-udf) + _newUndefColor[toIdx]*udf;
		    }
		    else
		    {
			const float a = (*orgCol3) * (1.0f-udf);
			const float b = _newUndefColor[3] * udf;
			col[toIdx] = (col[toIdx]*a + _newUndefColor[toIdx]*b) / col[3];
		    }
		}
		else if (  toIdx==3 )
		    col[3] *= 1.0f-udf;

		udf = 0.0f;
	    }
	}
	else if ( udf<oldUdf )
	    udf = oldUdf;
    }
    else if ( toIdx>=0 )
    {
	col[toIdx] = _layTex.getDataLayerTextureVec(id,coord)[fromIdx];
	if ( orgCol3 && toIdx==3 )
	    *orgCol3 = col[3];
    }
    else
	col = _layTex.getDataLayerTextureVec( id, coord );
}


void LayerProcess::processFooter( osg::Vec4f& fragColor, osg::Vec4f col, float udf ) const
{
    if ( udf>=1.0f )
	col = _newUndefColor;
    else if ( udf>0.0f )
    {
	if ( _newUndefColor[3]<=0.0f )
	    col[3] *= 1.0f-udf;
	else if ( _newUndefColor[3]>=1.0f && col[3]>=1.0f )
	    col = col*(1.0f-udf) + _newUndefColor*udf;
	else if ( col[3]>0 )
	{
	    const float a = col[3] * (1.0f-udf);
	    const float b = _newUndefColor[3] * udf;
	    const float c = a + b;
	    col = (col*a + _newUndefColor*b) / c;
	    col[3] = c;
	}
	else
	{
	    col = _newUndefColor;
	    col[3] *= udf;
	}
    }

    if ( _opacity<1.0 )
	col[3] *= _opacity;

    if ( fragColor[0]==-1.0f )
	fragColor = col;
    else
    {
	const float a = fragColor[3];
	const float b = col[3] * (1.0f-a);
	const float c = a+b;
	if ( c>0.0f )
	    fragColor = (fragColor*a + col*b) / c;

	fragColor[3] = c;
    }
}


void LayeredTexture::invertUndefLayers( bool yn )
{ _invertUndefLayers = yn; }


bool LayeredTexture::areUndefLayersInverted() const
{ return _invertUndefLayers; }


//============================================================================


class ColTabLayerProcess::ColSeqCallbackHandler : public ColorSequence::Callback
{
public:
    ColSeqCallbackHandler( LayeredTexture& layTex )
	: _layTex( layTex )
    {}

    virtual void requestRedraw() const	{ _layTex.triggerRedrawRequest(); }

protected: 
    LayeredTexture&	_layTex;
};


//============================================================================


ColTabLayerProcess::ColTabLayerProcess( LayeredTexture& layTex )
    : LayerProcess( layTex )
    , _colorSequence( 0 )
{
    _colSeqCallbackHandler = new ColSeqCallbackHandler( layTex );
    _colSeqCallbackHandler->ref();

    for ( int idx=0; idx<3; idx++ )
    {
	_id[idx] = -1;
	_textureChannel[idx] = 0;
    }
}


ColTabLayerProcess::~ColTabLayerProcess()
{
    setColorSequence( 0 );
    _colSeqCallbackHandler->unref();
}


void ColTabLayerProcess::setDataLayerID( int idx, int id, int channel )
{
    if ( idx>=0 && idx<3 )
    {
	_id[idx] = id;
	_textureChannel[idx] = channel>=0 && channel<4 ? channel : 0;
	_layTex.updateSetupStateSet();
    }
}


int ColTabLayerProcess::getDataLayerID( int idx ) const
{
    if ( idx==2 && _id[1]>=0 && _id[0]>=0 )
	return _id[2];
    if ( idx==1 && _id[0]>=0 )
	return _id[1];

    return idx==0 ? _id[0] : -1;
}


int ColTabLayerProcess::getDataLayerTextureChannel( int idx ) const
{ return idx>=0 && idx<3 ? _textureChannel[idx] : -1; }


void ColTabLayerProcess::checkForModifiedColorSequence()
{
    if ( _colorSequence )
    {
	const int modifiedCount = _colorSequence->getModifiedCount();
	if ( modifiedCount != _colSeqModifiedCount )
	{
	    _colSeqModifiedCount = modifiedCount;
	    _layTex.updateSetupStateSet();
	}
    }
}


void ColTabLayerProcess::setColorSequence( ColorSequence* colSeq )
{
    if ( _colorSequence )
    {
	_colorSequence->removeCallback( _colSeqCallbackHandler );
	_colorSequence->unref();
    }

    _colorSequence = colSeq;
    _colSeqModifiedCount = -1;
    _colSeqPtr = colSeq ? colSeq->getRGBAValues() : 0;
    _layTex.updateSetupStateSet();

     if ( _colorSequence )
     {
	 _colorSequence->ref();
	  _colorSequence->addCallback( _colSeqCallbackHandler );
     }
}


const ColorSequence* ColTabLayerProcess::getColorSequence() const
{ return _colorSequence; }


int ColTabLayerProcess::getColorSequenceUndefIdx() const
{
    const osg::Vec4f udfColor = _layTex.getDataLayerImageUndefColor( _id[0] );
    const int udfIdx = (int) floor( 0.5 + udfColor[_textureChannel[0]]*255.0 );
    return udfIdx>=0 && udfIdx<=255 ? udfIdx : -1;
}


void ColTabLayerProcess::getShaderCode( std::string& code, int stage ) const
{
    int nrUdf = 0;

    int nrChannels = 0;
    for ( int idx=0; idx<3; idx++ )
    {
	if ( !_layTex.isDataLayerOK(_id[idx]) )
	    break;

	getHeaderCode( code, nrUdf, _id[idx], idx, _textureChannel[idx] );
	code += "\n";
	nrChannels++;
    }

    if ( !nrChannels ) return;

    char line[100];
    if ( nrChannels>1 )
    {
	const int unit = _layTex.getDataLayerTextureUnit( _id[0] );
	snprintf( line, 100, "    texcrd *= texsize%d;\n", unit );
	code += line;

	code += "    float mip = log2( max(1.0,max(length(dFdx(texcrd)),length(dFdy(texcrd)))) );\n";

	code += "    float stddev = sqrt( max(0.0, col[1]";
	code += nrChannels>2 ? "+col[2]/255.0" : "";
	code += "-col[0]*col[0]) );\n";

	code += "    stddev *= 255.0 * clamp( mip, 0.0, 1.0 );\n";
	/* Upper bound of clamp interval tunes transition smoothness
	   between magnification and minification filtered areas */
	code += "    float scale = stddev>0.5 ? log2(stddev)+2.0 : stddev*2.0;\n";
    }

    code += "\n    texcrd = vec2( 0.996093*col[0]+0.001953, ";
    if ( nrChannels>1 )
    {
	 snprintf( line, 100, "%.6f*scale+", _colSeqTexSamplingStep );
	 code += line;
    }
    snprintf( line, 100, "%.6f );\n", _colSeqTexSamplingStart );
    code += line;

    code += "    col = texture2D( texture0, texcrd );\n"
	    "\n";

    getFooterCode( code, nrUdf, stage );
}


TransparencyType ColTabLayerProcess::getTransparencyType( bool imageOnly ) const
{
    if ( !_colorSequence || !_layTex.isDataLayerOK(_id[0]) )
	return FullyTransparent;

    TransparencyType tt = _colorSequence->getTransparencyType();
    // Only optimal if all colors in sequence are actually used

    if ( imageOnly )
	return tt;

    const int udfId = _layTex.getDataLayerUndefLayerID(_id[0]);
    if ( _layTex.isDataLayerOK(udfId) )
	tt = addOpacity( tt, _newUndefColor[3] );

    return multiplyOpacity( tt, _opacity );
}


void ColTabLayerProcess::doProcess( osg::Vec4f& fragColor, float stackUdf, const osg::Vec2f& globalCoord )
{
    if ( !_colorSequence || !_layTex.isDataLayerOK(_id[0]) )
	return;

    osg::Vec4f col;
    float udf = 0.0f;

    processHeader( col, udf, stackUdf, globalCoord, _id[0], 0, _textureChannel[0] );

    const int val = (int) floor( 255.0f*col[0] + 0.5 );
    const int offset = val<=0 ? 0 : (val>=255 ? 1020 : 4*val);
    const unsigned char* ptr = _colorSequence->getRGBAValues()+offset;
    for ( int idx=0; idx<4; idx++ )
	col[idx] = float(*ptr++) / 255.0f;

    processFooter( fragColor, col, udf );
}


//============================================================================


RGBALayerProcess::RGBALayerProcess( LayeredTexture& layTex )
    : LayerProcess( layTex )
    , _udfPerChannel( false )
{
    for ( int idx=0; idx<4; idx++ )
    {
	_id[idx] = -1;
	_textureChannel[idx] = 0;
	_isOn[idx] = true;
    }

    _newUndefColor = osg::Vec4f( 0.0f, 0.0f, 0.0f, 1.0f );
}


void RGBALayerProcess::setDataLayerID( int idx, int id, int channel )
{
    if ( idx>=0 && idx<4 )
    {
	_id[idx] = id;
	_textureChannel[idx] = channel>=0 && channel<4 ? channel : 0;
	_layTex.updateSetupStateSet();
    }
}


int RGBALayerProcess::getDataLayerID( int idx ) const
{ return idx>=0 && idx<4 ? _id[idx] : -1;  } 


void RGBALayerProcess::turnOn( int idx, bool yn )
{
    if ( idx>=0 && idx<4 )
    {
	_isOn[idx] = yn;
	_layTex.updateSetupStateSet();
    }
}

bool RGBALayerProcess::isOn(int idx) const
{ return idx>=0 && idx<4 ? _isOn[idx] : false; }


void RGBALayerProcess::applyUndefPerChannel( bool yn )
{
    if ( _udfPerChannel != yn )
    {
	_udfPerChannel = yn;
	_layTex.updateSetupStateSet();
    }
}

bool RGBALayerProcess::isUndefPerChannel() const
{ return _udfPerChannel; }


int RGBALayerProcess::getDataLayerTextureChannel( int idx ) const
{ return idx>=0 && idx<4 ? _textureChannel[idx] : -1;  } 


void RGBALayerProcess::getShaderCode( std::string& code, int stage ) const
{
    code += "    col = vec4( 0.0, 0.0, 0.0, 1.0 );\n";
    assignOrgCol3IfNeeded( code );

    code += "\n";

    int nrUdf = 0;
    for ( int idx=3; idx>=0; idx-- )
    {
	if ( _isOn[idx] && _layTex.isDataLayerOK(_id[idx]) )
	{
	    getHeaderCode( code, nrUdf, _id[idx], idx, _textureChannel[idx] );
	    code += "\n";
	}
    }

    getFooterCode( code, nrUdf, stage );
}


TransparencyType RGBALayerProcess::getTransparencyType( bool imageOnly ) const
{
    int nrActiveChannels = 0;
    for ( int idx=0; idx<4; idx++ )
    {
	if ( _isOn[idx] && _layTex.isDataLayerOK(_id[idx]) )
	    nrActiveChannels++;
    }
    if ( nrActiveChannels==0 )
	return FullyTransparent;

    if ( !_isOn[3] || !_layTex.isDataLayerOK(_id[3]) )
	return imageOnly ? Opaque : multiplyOpacity( Opaque, _opacity );

    TransparencyType tt = _layTex.getDataLayerTransparencyType( _id[3], _textureChannel[3] );

    if ( imageOnly )
	return tt;

    for ( int idx=0; idx<4; idx++ )
    {
	const int udfId = _layTex.getDataLayerUndefLayerID( _id[idx] );

	if ( _isOn[idx] && _layTex.isDataLayerOK(udfId) )
	{
	    tt = addOpacity( tt, _newUndefColor[3] );
	    return multiplyOpacity( tt, _opacity );
	}
    }

    return multiplyOpacity( tt, _opacity );
}


void RGBALayerProcess::doProcess( osg::Vec4f& fragColor, float stackUdf, const osg::Vec2f& globalCoord )
{
    osg::Vec4f col( 0.0f, 0.0f, 0.0f, 1.0f );
    float orgCol3 = col[3];
    float udf = 0.0f;

    for ( int idx=3; idx>=0; idx-- )
    {
	if ( _isOn[idx] && _layTex.isDataLayerOK(_id[idx]) )
	{
	    processHeader( col, udf, stackUdf, globalCoord, _id[idx], idx, _textureChannel[idx], &orgCol3 );
	}
    }

    processFooter( fragColor, col, udf );
}	


//============================================================================


IdentityLayerProcess::IdentityLayerProcess( LayeredTexture& layTex, int id )
    : LayerProcess( layTex )
    , _id( id )
{}


int IdentityLayerProcess::getDataLayerID( int idx ) const
{ return idx ? -1 : _id; } 


void IdentityLayerProcess::getShaderCode( std::string& code, int stage ) const
{
    if ( !_layTex.isDataLayerOK(_id) )
	return;

    int nrUdf = 0;
    getHeaderCode( code, nrUdf, _id );

    code += "\n";
    getFooterCode( code, nrUdf, stage );
}


TransparencyType IdentityLayerProcess::getTransparencyType( bool imageOnly ) const
{
    TransparencyType tt = _layTex.getDataLayerTransparencyType(_id);

    if ( imageOnly )
	return tt;

    const int udfId = _layTex.getDataLayerUndefLayerID(_id);

    if ( _layTex.isDataLayerOK(udfId) )
	tt = addOpacity( tt, _newUndefColor[3] );

    return multiplyOpacity( tt, _opacity );
}


void IdentityLayerProcess::doProcess( osg::Vec4f& fragColor, float stackUdf, const osg::Vec2f& globalCoord )
{
    if ( !_layTex.isDataLayerOK(_id) )
	return;

    osg::Vec4f col;
    float udf = 0.0f;

    processHeader( col, udf, stackUdf, globalCoord, _id );
    processFooter( fragColor, col, udf );
}


} //namespace
