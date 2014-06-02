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
namespace osgGeo
{


WellLog::WellLog()
    :_logPath ( new osg::Vec3Array )	
    ,_nonShadingGroup( new osg::Group )
    ,_colorTable( new osg::Vec4Array )	
    ,_shapeLog ( new osg::FloatArray )
    ,_fillLog ( new osg::FloatArray )
    ,_fillLogDepths( new osg::FloatArray )
    ,_logWidthChanged( false )
    ,_colorTableChanged( false )
    ,_fillRevScale( false )
    ,_forceReBuild( false )
    ,_revScale( false)
    ,_logWidth( 250 )
    ,_minShapeValue( mMAX )
    ,_minFillValue( mMAX )
    ,_maxShapeValue( -mMAX )
    ,_maxFillValue( -mMAX )
    ,_geode( new osg::Geode )
    ,_forceCoordReCalculation ( true )
    ,_lineColor( new osg::Vec4Array )
    ,_lineWidth( new osg::LineWidth )
{
    setNumChildrenRequiringUpdateTraversal( 1 );
    _preProjDir.set( 0, 0, 0 );
    _nonShadingGroup->addChild( _geode );
    _lineColor->push_back(osg::Vec4d(0, 0, 0, 0 ));
    _lineWidth->setWidth(1.0);

}

#define COPY_ARRAY( tp, var ) \
    var ( (cop.getCopyFlags() & osg::CopyOp::DEEP_COPY_ARRAYS) \
        ? new tp(*wl.var.get() ) \
        : const_cast<tp*>(wl.var.get()) )

WellLog::WellLog( const WellLog& wl, const osg::CopyOp& cop )
    : Node( wl, cop )
    , COPY_ARRAY( osg::Vec3Array, _logPath )
    , COPY_ARRAY( osg::Vec4Array, _colorTable )
    , COPY_ARRAY( osg::FloatArray, _shapeLog )
    , COPY_ARRAY( osg::FloatArray, _fillLog )
    , COPY_ARRAY( osg::FloatArray, _fillLogDepths )
    , _fillRevScale( wl._fillRevScale )
    , _revScale( wl._revScale )
    , _logWidth( wl._logWidth )
    , _minShapeValue( wl._minShapeValue )
    , _minFillValue( wl._minFillValue )
    , _maxShapeValue( wl._maxShapeValue )
    , _maxFillValue( wl._maxFillValue )
    , _geode( new osg::Geode )
    , _forceCoordReCalculation ( true )
    , _nonShadingGroup( new osg::Group )
    , _logWidthChanged( false )
    , _colorTableChanged( false )
    , _forceReBuild( false )
{}



WellLog::~WellLog()
{
    clearDraw();
}


void WellLog::traverse(osg::NodeVisitor& nv)
{
    if ( _logPath && _logPath->size() && _nonShadingGroup ) 
	_nonShadingGroup->accept(nv);
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


void WellLog::setMinFillValue( float minFillValue )
{
    _minFillValue = minFillValue;
    _forceReBuild = true;
}


void WellLog::setLogWidth( float logWidth )
{
    _logWidth = logWidth;
    _forceReBuild = true;
    _logWidthChanged = true;
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


void WellLog::setLineColor( const osg::Vec4& lineColor)
{
    (*_lineColor)[0] = lineColor;
    _lineColor->dirty();
}


const osg::Vec4& WellLog::getLineColor() const
{
    return _lineColor->at(0);
}


void WellLog::setLineWidth(float lineWidth)
{
    _lineWidth->setWidth( lineWidth );
}


float WellLog::getLineWidth() const
{
    return _lineWidth->getWidth();
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


struct ClosestCmp 
{
    bool operator()(const float &val1, const float& val2) 
    {
	return val1 > val2;
    }
};


int WellLog::getClosestIndex(const osg::FloatArray& arr, float val)
{
    osg::FloatArray::const_iterator it =
	std::lower_bound(arr.begin(), arr.end(), val, ClosestCmp());

    if (it != arr.end()) 
	return std::distance(arr.begin(),it);

    return -1;
}

}// Namespace

#include <osgDB/ObjectWrapper>
#include <osgDB/Registry>
#include <osgDB/Serializer>


REGISTER_OBJECT_WRAPPER( WellLog_Wrapper,
                        0,
                        osgGeo::WellLog,
                        "osg::Object osg::Node osgGeo::WellLog")
{
    ADD_OBJECT_SERIALIZER( Path, osg::Vec3Array, NULL );

    ADD_OBJECT_SERIALIZER( ShapeLog, osg::FloatArray, NULL );
    ADD_FLOAT_SERIALIZER( MaxShapeValue, -mMAX );
    ADD_FLOAT_SERIALIZER( MinShapeValue, mMAX );

    ADD_OBJECT_SERIALIZER( FillLogValues, osg::FloatArray, NULL );
    ADD_FLOAT_SERIALIZER( MaxFillValue, -mMAX );
    ADD_FLOAT_SERIALIZER( MinFillValue, mMAX );

    ADD_OBJECT_SERIALIZER(FillLogColorTab, osg::Vec4Array, NULL );

    ADD_BOOL_SERIALIZER(ShowLog, true );

    ADD_FLOAT_SERIALIZER( LogWidth, 250 );

    ADD_BOOL_SERIALIZER( RevScale, false );
    ADD_BOOL_SERIALIZER( FillRevScale, false );

    BEGIN_ENUM_SERIALIZER( DisplaySide, Right );
    	ADD_ENUM_VALUE( Left );
    	ADD_ENUM_VALUE( Right );
    END_ENUM_SERIALIZER();

    ADD_UINT_SERIALIZER(RepeatNumber, 0);
    ADD_FLOAT_SERIALIZER(RepeatGap, 1 );

    ADD_BOOL_SERIALIZER( FullFilled, true);
    ADD_BOOL_SERIALIZER( SeisLogStyle, false );

    ADD_BOOL_SERIALIZER( LogFill, true );
    ADD_FLOAT_SERIALIZER( LineWidth, 1 );

    ADD_VEC4_SERIALIZER( LineColor, osg::Vec4(1,0,0,0) );
    ADD_INT_SERIALIZER(Resolution, 1);
}

