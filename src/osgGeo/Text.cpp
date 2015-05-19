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

$Id: TrackballManipulator.cpp 499 2015-03-13 10:03:51Z jaap.glas@dgbes.com $
*/

#include <osgGeo/Text>

#include <iostream>
#include <cstdio>


namespace osgGeo
{

#define MAX_ELEVATION_ANGLE	(75.0*M_PI/180.0)

Text::Text()
    : osgText::Text()
    , _useRotateToScreenElevation(false)
    , _elevationAngle(M_PI/6.0)
    , _elevationPlane(0.0,0.0,1.0)
{}


Text::Text(const Text& text,const osg::CopyOp& copyop)
    : osgText::Text(text,copyop)
    , _useRotateToScreenElevation(text._useRotateToScreenElevation)
    , _elevationAngle(text._elevationAngle)
    , _elevationPlane(text._elevationPlane)
{}


Text::~Text()
{}


void Text::computePositions(unsigned int contextID) const
{
    switch(_alignment)
    {
    case LEFT_TOP:      _offset.set(_textBB.xMin(),_textBB.yMax(),_textBB.zMin()); break;
    case LEFT_CENTER:   _offset.set(_textBB.xMin(),(_textBB.yMax()+_textBB.yMin())*0.5f,_textBB.zMin()); break;
    case LEFT_BOTTOM:   _offset.set(_textBB.xMin(),_textBB.yMin(),_textBB.zMin()); break;

    case CENTER_TOP:    _offset.set((_textBB.xMax()+_textBB.xMin())*0.5f,_textBB.yMax(),_textBB.zMin()); break;
    case CENTER_CENTER: _offset.set((_textBB.xMax()+_textBB.xMin())*0.5f,(_textBB.yMax()+_textBB.yMin())*0.5f,_textBB.zMin()); break;
    case CENTER_BOTTOM: _offset.set((_textBB.xMax()+_textBB.xMin())*0.5f,_textBB.yMin(),_textBB.zMin()); break;

    case RIGHT_TOP:     _offset.set(_textBB.xMax(),_textBB.yMax(),_textBB.zMin()); break;
    case RIGHT_CENTER:  _offset.set(_textBB.xMax(),(_textBB.yMax()+_textBB.yMin())*0.5f,_textBB.zMin()); break;
    case RIGHT_BOTTOM:  _offset.set(_textBB.xMax(),_textBB.yMin(),_textBB.zMin()); break;

    case LEFT_BASE_LINE:  _offset.set(0.0f,0.0f,0.0f); break;
    case CENTER_BASE_LINE:  _offset.set((_textBB.xMax()+_textBB.xMin())*0.5f,0.0f,0.0f); break;
    case RIGHT_BASE_LINE:  _offset.set(_textBB.xMax(),0.0f,0.0f); break;

    case LEFT_BOTTOM_BASE_LINE:  _offset.set(0.0f,-_characterHeight*(1.0 + _lineSpacing)*(_lineCount-1),0.0f); break;
    case CENTER_BOTTOM_BASE_LINE:  _offset.set((_textBB.xMax()+_textBB.xMin())*0.5f,-_characterHeight*(1.0 + _lineSpacing)*(_lineCount-1),0.0f); break;
    case RIGHT_BOTTOM_BASE_LINE:  _offset.set(_textBB.xMax(),-_characterHeight*(1.0 + _lineSpacing)*(_lineCount-1),0.0f); break;
    }

    AutoTransformCache& atc = _autoTransformCache[contextID];
    osg::Matrix& matrix = atc._matrix;

    if (_characterSizeMode!=OBJECT_COORDS || _autoRotateToScreen)
    {

        matrix.makeTranslate(-_offset);

        osg::Matrix rotate_matrix;
        if (_autoRotateToScreen)
        {
            osg::Vec3d trans(atc._modelview.getTrans());
            atc._modelview.setTrans(0.0f,0.0f,0.0f);

            rotate_matrix.invert(atc._modelview);
            atc._modelview.setTrans(trans);
        }

        matrix.postMultRotate(_rotation);

        if (_characterSizeMode!=OBJECT_COORDS)
        {

            osg::Matrix M(rotate_matrix);
            M.postMultTranslate(_position);
            M.postMult(atc._modelview);
            osg::Matrix& P = atc._projection;

            // compute the pixel size vector.

            // pre adjust P00,P20,P23,P33 by multiplying them by the viewport window matrix.
            // here we do it in short hand with the knowledge of how the window matrix is formed
            // note P23,P33 are multiplied by an implicit 1 which would come from the window matrix.
            // Robert Osfield, June 2002.

            // scaling for horizontal pixels
            float P00 = P(0,0)*atc._width*0.5f;
            float P20_00 = P(2,0)*atc._width*0.5f + P(2,3)*atc._width*0.5f;
            osg::Vec3 scale_00(M(0,0)*P00 + M(0,2)*P20_00,
                               M(1,0)*P00 + M(1,2)*P20_00,
                               M(2,0)*P00 + M(2,2)*P20_00);

            // scaling for vertical pixels
            float P10 = P(1,1)*atc._height*0.5f;
            float P20_10 = P(2,1)*atc._height*0.5f + P(2,3)*atc._height*0.5f;
            osg::Vec3 scale_10(M(0,1)*P10 + M(0,2)*P20_10,
                               M(1,1)*P10 + M(1,2)*P20_10,
                               M(2,1)*P10 + M(2,2)*P20_10);

            float P23 = P(2,3);
            float P33 = P(3,3);

            float pixelSizeVector_w = M(3,2)*P23 + M(3,3)*P33;

            float pixelSizeVert=(_characterHeight*sqrtf(scale_10.length2()))/(pixelSizeVector_w*0.701f);
            float pixelSizeHori=(_characterHeight/getCharacterAspectRatio()*sqrtf(scale_00.length2()))/(pixelSizeVector_w*0.701f);

            // avoid nasty math by preventing a divide by zero
            if (pixelSizeVert == 0.0f)
               pixelSizeVert= 1.0f;
            if (pixelSizeHori == 0.0f)
               pixelSizeHori= 1.0f;

            if (_characterSizeMode==SCREEN_COORDS)
            {
                float scale_font_vert=_characterHeight/pixelSizeVert;
                float scale_font_hori=_characterHeight/getCharacterAspectRatio()/pixelSizeHori;

                if (P10<0)
                   scale_font_vert=-scale_font_vert;
                matrix.postMultScale(osg::Vec3f(scale_font_hori, scale_font_vert,1.0f));
            }
            else if (pixelSizeVert>getFontHeight())
            {
                float scale_font = getFontHeight()/pixelSizeVert;
                matrix.postMultScale(osg::Vec3f(scale_font, scale_font,1.0f));
            }

        }

        if (_autoRotateToScreen)
        {
	    // begin modified
	    if ( _useRotateToScreenElevation )
		matrix.postMult( getElevationMatrix(contextID) );
	    // end modified

            matrix.postMult(rotate_matrix);
        }

        matrix.postMultTranslate(_position);
    }
    else if (!_rotation.zeroRotation())
    {
        matrix.makeRotate(_rotation);
        matrix.preMultTranslate(-_offset);
        matrix.postMultTranslate(_position);
    }
    else
    {
        matrix.makeTranslate(_position-_offset);
    }

    // now apply matrix to the glyphs.
    for(TextureGlyphQuadMap::iterator titr=_textureGlyphQuadMap.begin();
        titr!=_textureGlyphQuadMap.end();
        ++titr)
    {
        GlyphQuads& glyphquad = titr->second;
        GlyphQuads::Coords2& coords2 = glyphquad._coords;
        GlyphQuads::Coords3& transformedCoords = glyphquad._transformedCoords[contextID];

        unsigned int numCoords = coords2.size();
        if (numCoords!=transformedCoords.size())
        {
            transformedCoords.resize(numCoords);
        }

        for(unsigned int i=0;i<numCoords;++i)
        {
            transformedCoords[i] = osg::Vec3(coords2[i].x(),coords2[i].y(),0.0f)*matrix;
        }
    }

    computeBackdropPositions(contextID);

    _normal = osg::Matrix::transform3x3(osg::Vec3(0.0f,0.0f,1.0f),matrix);
    _normal.normalize();

    const_cast<Text*>(this)->dirtyBound();
}


osg::Matrix Text::getElevationMatrix( unsigned int contextID ) const
{
    if ( !_autoRotateToScreen || !_useRotateToScreenElevation )
	return osg::Matrix::identity();

    AutoTransformCache& atc = _autoTransformCache[contextID];

    osg::Matrix MV( atc._modelview );
    MV.setTrans( 0.0, 0.0, 0.0 );
    const osg::Matrix rotate_matrix( osg::Matrix::inverse(MV) );

    osg::Vec3d projection = _elevationPlane * MV;

    double xAngle = atan2( projection[2], projection[1] );
    if ( xAngle < 0.0 )
	xAngle = xAngle + M_PI;

    xAngle += _elevationAngle - M_PI/2.0;

    if ( xAngle<0.0 || xAngle>MAX_ELEVATION_ANGLE )
	return osg::Matrix::identity();

    // Fade elevation towards angular singularity
    xAngle *= sqrt( fabs(1.0 - fabs(projection[0])) );

    float yAngle = 0.0;

    if ( _alignment==LEFT_TOP || _alignment==LEFT_CENTER || _alignment==LEFT_BOTTOM || _alignment==LEFT_BASE_LINE || _alignment==LEFT_BOTTOM_BASE_LINE )
	yAngle = -xAngle;

    if ( _alignment==RIGHT_TOP || _alignment==RIGHT_CENTER || _alignment==RIGHT_BOTTOM || _alignment==RIGHT_BASE_LINE || _alignment==RIGHT_BOTTOM_BASE_LINE )
	yAngle = xAngle;

    osg::Matrix elevation_matrix;
    elevation_matrix.makeRotate( xAngle, osg::Vec3(1.0,0.0,0.0) );
    const osg::Quat quat( yAngle, osg::Vec3(0.0,1.0,0.0) );
    elevation_matrix.postMultRotate( quat );

    osg::Matrix MVPW( rotate_matrix );
    MVPW.postMultTranslate( _position );
    MVPW.postMult( atc._modelview );
    MVPW.postMult( atc._projection );
    MVPW.postMultScale( osg::Vec3(atc._width,atc._height,1.0) );

    const osg::Vec3 xOrg = osg::Vec3(1,0,0)*MVPW - osg::Vec3(0,0,0)*MVPW;
    const osg::Vec3 yOrg = osg::Vec3(0,1,0)*MVPW - osg::Vec3(0,0,0)*MVPW;

    MVPW.preMult( elevation_matrix );

    const osg::Vec3 xNew = osg::Vec3(1,0,0)*MVPW - osg::Vec3(0,0,0)*MVPW;
    const osg::Vec3 yNew = osg::Vec3(0,1,0)*MVPW - osg::Vec3(0,0,0)*MVPW;

    osg::Matrix shear_matrix;
    shear_matrix(0,0) = xNew[0] / xOrg[0];
    shear_matrix(0,1) = xNew[1] / xOrg[0];
    shear_matrix(1,0) = yNew[0] / yOrg[1];
    shear_matrix(1,1) = yNew[1] / yOrg[1];

    osg::Matrix inv_shear( osg::Matrix::inverse(shear_matrix) );
    elevation_matrix.preMult( inv_shear );

    // Shear correction (position dependent) has reshaped original billboard
    const float maxBillboardHeightening = 1.0 / cos(MAX_ELEVATION_ANGLE);
    if ( inv_shear(0,0)<=0.0 || inv_shear(0,0)>maxBillboardHeightening )
	return osg::Matrix::identity();
    if ( inv_shear(1,1)<=0.0 || inv_shear(1,1)>maxBillboardHeightening )
	return osg::Matrix::identity();

    // Emperical rescaling to make billboard projection implode instead of
    // explode when elevating towards the camera eye
    const float implodeFactor = sqrt( fabs(cos(xAngle)) );
    elevation_matrix.preMultScale( osg::Vec3(implodeFactor,implodeFactor,1.0) );

    return elevation_matrix;
}


void Text::useRotateToScreenElevation( bool yn )
{ _useRotateToScreenElevation = yn; }


bool Text::isRotateToScreenElevationUsed() const
{ return _useRotateToScreenElevation; }


void Text::setRotateToScreenElevationAngle( float angle )
{ _elevationAngle = osg::minimum( (float) fabs(angle), (float) MAX_ELEVATION_ANGLE ); }


float Text::getRotateToScreenElevationAngle() const
{ return _elevationAngle; }


void Text::setRotateToScreenElevationPlane( const osg::Vec3& refPlaneNormal )
{
    _elevationPlane = refPlaneNormal;
    _elevationPlane.normalize();
}


const osg::Vec3& Text::getRotateToScreenElevationPlane() const
{ return _elevationPlane; }


} // end namespace
