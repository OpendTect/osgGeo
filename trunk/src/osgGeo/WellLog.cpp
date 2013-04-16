/* osgGeo - A collection of geoscientific extensions to OpenSceneGraph.
Copyright 2012 dGB Beheer B.V.

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

#include <osgGeo/WellLog>
#include <osg/MatrixTransform>
#include <osgViewer/Viewer>
#include <osg/PolygonOffset>
#include <osg/PolygonMode>
#include <osg/LineWidth>
#include <osg/Vec3>


#define mMAX 1e30
using namespace osgGeo;


WellLog::WellLog()
    :_logPath ( new osg::Vec3Array )	
    ,_nonShadingGroup( new osg::Group )
    ,_colorTable( new osg::Vec4Array )	
    ,_shapeLog ( new osg::FloatArray )
    ,_fillLog ( new osg::FloatArray )
    ,_fillLogDepths( new osg::FloatArray )
    ,_resizeWhenZooming( false )
    ,_screenSizeChanged( false )
    ,_colorTableChanged( false )
    ,_fillRevScale( false )
    ,_forceReBuild( false )
    ,_revScale( false)
    ,_constantSizeFactor( 1 )
    ,_minShapeValue( mMAX )
    ,_minFillValue( mMAX )
    ,_maxShapeValue( -mMAX )
    ,_maxFillValue( -mMAX )
    ,_geode( new osg::Geode )
    ,_forceCoordReCalculation ( true )
{
    setNumChildrenRequiringUpdateTraversal( 1 );
    _preProjDir.set( 0, 0, 0 );
    _nonShadingGroup->addChild( _geode );
}


WellLog::~WellLog()
{
    clearDraw();
}


void WellLog::setPath( osg::Vec3Array* vtxarraypath )
{
    _logPath = vtxarraypath;
    _forceReBuild = true;
}


void WellLog::setShapeLog( osg::FloatArray* shapelog )
{
    _shapeLog = shapelog;
    _forceReBuild = true;
}


void WellLog::setFillLogValues( osg::FloatArray* filllog )
{
    _fillLog = filllog;
    _forceReBuild = true;
}


void WellLog::setMaxShapeValue( float maxvalue )
{
    _maxShapeValue = maxvalue;
    _forceReBuild = true;
}


void WellLog::setMinShapeValue( float minvalue )
{
    _minShapeValue = minvalue;
    _forceReBuild = true;
}


void WellLog::setMaxFillValue( float maxfillvalue )
{
    _maxFillValue = maxfillvalue;
    _forceReBuild = true;
}


void WellLog::setMinFillValue( float minfillvalue )
{
    _minFillValue = minfillvalue;
    _forceReBuild = true;
}


void WellLog::setScreenWidth( float screenwidth )
{
    _screenWidth = screenwidth;
    _forceReBuild = true;
    _screenSizeChanged = true;
}


bool WellLog::eyeChanged( const osg::Vec3 projdir )
{
    return ( projdir != _preProjDir ? true : false );
}


void WellLog::setFillLogColorTab( osg::Vec4Array* logcolortable )
{
    _colorTable = logcolortable;
    _colorTableChanged = true;
}


void WellLog::setLogConstantSize( bool rsz )
{
    _resizeWhenZooming = !rsz;
    _forceReBuild = true;
}


void WellLog::setLogConstantSizeFactor( float fac )
{
    _constantSizeFactor =  fac;
    _forceReBuild = true;
}


bool WellLog::getDisplayStatus()
{
    return ( _nonShadingGroup->getNumChildren() > 0 ? true : 0 );
}


void WellLog::setShowLog( bool showlog )
{
    _showLog = showlog;
    if( _showLog )
	_forceCoordReCalculation = true;
    _forceReBuild = true;
}


void WellLog::setFillLogDepths( osg::FloatArray* depths )
{
    _fillLogDepths = depths; 
}


void WellLog::clearLog()
{
    _colorTable->clear();
    _logPath->clear();	
    _shapeLog->clear();
    _fillLog->clear();
    _fillLogDepths->clear();
    _minShapeValue =  mMAX ;
    _minFillValue  =  mMAX ;
    _maxShapeValue = -mMAX ;
    _maxFillValue  = -mMAX;

}


void WellLog::clearDraw()
{
    _nonShadingGroup->removeChildren( 0, _nonShadingGroup->getNumChildren() );
}


float WellLog::getShapeFactor(float val, float minval, float maxval )const
{
    float res = (val-minval)/(maxval-minval);
    if ( res<0 ) res = 0;
    else if ( res>1 ) res = 1;

    return res;
}


osg::Vec2 WellLog::worldToScreen( const osg::Vec3& worldposition,
    const osg::Camera* ca )
{
    osg::Vec2 screenposition(0,0);

    if ( ca )
    {
	osg::Matrixd MVPW = ca->getViewMatrix() * ca->getProjectionMatrix() * 
	    ca->getViewport()->computeWindowMatrix();

	osg::Vec4d screenposition4d = osg::Vec4d( worldposition, 1.0) * MVPW;
	screenposition4d = screenposition4d / screenposition4d.w();
	screenposition4d.y()=ca->getViewport()->height()-screenposition4d.y();
	screenposition.set( screenposition4d.x(), screenposition4d.y() );
    }
    return screenposition; 
}


osg::Vec3 WellLog::screenToWorld( const osg::Vec2d& screenposition, 
    const osg::Camera* ca )
{
    osg::Vec3 worldposition (0, 0, 0);

    if ( ca)
    {
	osg::Vec4 screenpositionnear( screenposition.x(), 
	    ca->getViewport()->height() - screenposition.y(), 0.0, 1.0 );
	osg::Vec4 screenpositionfar( screenposition.x(), 
	    ca->getViewport()->height() - screenposition.y(), 1.0, 1.0 );
	osg::Matrixd iMVPW = osg::Matrixd::inverse( ca->getViewMatrix() * 
	    ca->getProjectionMatrix() * 
	    ca->getViewport()->computeWindowMatrix() );
	osg::Vec4 worldpositionnear = screenpositionnear * iMVPW;
	osg::Vec4 worldpositionfar = screenpositionfar * iMVPW;
	worldpositionnear = worldpositionnear / worldpositionnear.w();
	worldpositionfar = worldpositionfar / worldpositionfar.w();

	worldposition.set( ( worldpositionnear.x() + worldpositionfar.x() )/2.0, 
	    ( worldpositionnear.y() + worldpositionfar.y() )/2.0,  
	    ( worldpositionnear.z() + worldpositionfar.z() )/2.0);
    }

    return worldposition; 
}


float WellLog::calcWorldWidth( const osgUtil::CullVisitor* cv )
{
    float worldwidth(.0);
    if( !cv ) 
	return 0;

    osg::Viewport* viewport = 
	const_cast<osgUtil::CullVisitor*>(cv)->getViewport();
    float szpixel = viewport->height();
    float nsize1 = _screenWidth / szpixel; 
    int hnum = (int)_logPath->size() / 2;

    const osg::Vec3& worldpnt =  _logPath->at(hnum);
    osg::Vec2d scrpnt = worldToScreen( worldpnt,
	const_cast<osgUtil::CullVisitor*>(cv)->getCurrentCamera() );
    scrpnt[0] += nsize1;
    osg::Vec3 newwldpnt = screenToWorld( scrpnt,
	const_cast<osgUtil::CullVisitor*>(cv)->getCurrentCamera() );

    worldwidth = fabs( newwldpnt[0] - worldpnt[0] );

    if ( !_resizeWhenZooming ) 
	worldwidth = nsize1*_constantSizeFactor;
    _screenSizeChanged = false;

    return worldwidth;
}


osg::Vec3 WellLog::getPrjDirection( const osgUtil::CullVisitor* cv ) const
{
    osg::Vec3 projdir( 0, 0, 0 );
    if( !cv ) return projdir;
    const osg::Camera* ca = 
	const_cast<osgUtil::CullVisitor*>(cv)->getCurrentCamera();
    osg::Vec3 up;
    osg::Vec3 eye;
    osg::Vec3 center;
    ca->getViewMatrixAsLookAt( eye,center,up );
    projdir = center - eye;
    projdir.normalize();
    return projdir;
}


void WellLog::setFillRevScale( bool yn )
{
    _fillRevScale = yn; 
}


void WellLog::setRevScale( bool yn ) 
{
    _revScale = yn; 
}
