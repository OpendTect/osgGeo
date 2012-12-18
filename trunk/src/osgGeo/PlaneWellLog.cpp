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

#include <osgGeo/PlaneWellLog>
#include <osg/MatrixTransform>
#include <osgViewer/Viewer>
#include <osg/LineWidth>
#include <osg/Vec3>


#define mMAX 1e30

using namespace osgGeo;


PlaneWellLog::PlaneWellLog()
    :_logLinedPoints( new osg::Vec3Array )
    ,_logLinedTriPoints ( new osg::Vec3Array )
    ,_logPath ( new osg::Vec3Array )	
    ,_nonshadinggroup( new osg::Group )
    ,_LogColors( new osg::Vec4Array )
    ,_colorTable( new osg::Vec4Array )	
    ,_shapeLog ( new osg::FloatArray )
    ,_fillLog ( new osg::FloatArray )
    ,_fillLogDepths( new osg::FloatArray )
    ,_coordLinedFactors( new osg::FloatArray )
    ,_coordLinedTriFactors( new osg::FloatArray )
    ,_resizewhenzooming( false )
    ,_screenSizeChanged( false )
    ,_clrTblChanged( false )
    ,_fillrevscale( false )
    ,_forceRebuild( false )
    ,_seisstyle( false )
    ,_revscale( false)
    ,_isFilled( false )
    ,_constantsizefactor( 1 )
    ,_minShapeValue(mMAX )
    ,_minFillValue( mMAX )
    ,_maxShapeValue( -mMAX )
    ,_maxFillValue(-mMAX )
    ,_shiftValue( 0 )
    ,_dispSide( Left )
    ,_repeatNumber( 1 )
    ,_repeatGap( .0 )
    ,_lineGeometryWidth( .0 )
    ,_geode( new osg::Geode )
{
    setNumChildrenRequiringUpdateTraversal( 1 );
    buildLineGeometry();
    buildTriangleGeometry();
    _preProjDir.set( 0, 0, 0 );
    _nonshadinggroup->addChild( _geode );
}


PlaneWellLog::~PlaneWellLog()
{
   clearDraw();
   clearCoords();
}


void PlaneWellLog::buildLineGeometry()
{
    _lineGeometry = new osg::Geometry();
    _geode->addDrawable( _lineGeometry );
    osg::ref_ptr<osg::Vec3Array> arr = new osg::Vec3Array;
    _lineGeometry->setVertexArray( _logLinedPoints.get() );
    _lineColor = new osg::Vec4Array;
    _lineColor->push_back( osg::Vec4d( 0, 0, 0, 0 ) );
    _lineGeometry->setColorArray( _lineColor.get() );
    _lineGeometry->setColorBinding( osg::Geometry::BIND_OVERALL );
    osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array;
    normals->push_back( osg::Vec3 (0.0f,-1.0f,0.0f ) );
    _lineGeometry->setNormalArray( normals.get());
    _lineGeometry->setNormalBinding( osg::Geometry::BIND_OVERALL );
    _linePrimitiveSet = new osg::DrawArrays( osg::PrimitiveSet::LINE_STRIP, 0, 0 );
    _lineGeometry->addPrimitiveSet( _linePrimitiveSet );
    _lineWidth = new osg::LineWidth();
    _lineWidth->setWidth(1.0);
    getOrCreateStateSet()->setAttributeAndModes( _lineWidth );
    getStateSet()->setMode( GL_LIGHTING, osg::StateAttribute::OFF );
}


void PlaneWellLog::buildTriangleGeometry()
{
    _triangleStrip  = new osg::Geometry();
    _geode->addDrawable( _triangleStrip );
    osg::ref_ptr<osg::Vec3Array> shared_normals = new osg::Vec3Array;
    shared_normals->push_back( osg::Vec3( 0.0f, -1.0f, 0.0f ) );

    _triangleStrip->setVertexArray( _logLinedTriPoints.get() );
    _triangleStrip->setNormalArray( shared_normals.get() );
    _triangleStrip->setNormalBinding( osg::Geometry::BIND_OVERALL );

    _LogColors = new osg::Vec4Array;
    _triangleStrip->setColorArray( _LogColors.get() );
    _triangleStrip->setColorBinding( osg::Geometry::BIND_PER_VERTEX );

    _trianglePrimitiveSet = new osg::DrawArrays( osg::PrimitiveSet::TRIANGLE_STRIP, 0, 0 );

    _triangleStrip->addPrimitiveSet( _trianglePrimitiveSet );
    _triangleStrip->setDataVariance( osg::Object::DYNAMIC );
}


void PlaneWellLog::traverse( osg::NodeVisitor& nv )
{
    if( !_logPath->size() )
	return;

    if ( nv.getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR )
    {
	if ( _forceRebuild )
	{
	    calcFactors();
	    _clrTblChanged = true;
	    _forceCoordRecalculation = true;
	}

	if ( _clrTblChanged )
	    updateFilledLogColor();
    }
    
    if ( nv.getVisitorType() == osg::NodeVisitor::CULL_VISITOR )
    {
	osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*>(&nv);
	const osg::Vec3 projDir = getPrjDirection( cv );
	
	if ( _forceCoordRecalculation || eyeChanged( projDir ) )
	{
	    calcCoordinates( updateNormal( projDir ), calcWorldWidth( cv ) ); 
	    _preProjDir = projDir;
	}

	osg::ref_ptr<osg::MatrixTransform> tr = new osg::MatrixTransform;
	osg::ref_ptr<osg::RefMatrix> modelviewmatrix = 
	    const_cast<osgUtil::CullVisitor*>(cv)->getModelViewMatrix();
	
	if ( getStateSet() ) cv->pushStateSet( getStateSet() );

	osg::Matrix repeatTransform;
	for ( int i = 0; i < (int)_repeatNumber; i++)
	{
	    if( _dispSide == Right )
		repeatTransform.setTrans( osg::Vec3(_repeatGap*i,0,0) );
	    else
		repeatTransform.setTrans( osg::Vec3(_repeatGap*(-i),0,0) );

	    osg::Matrix MVR = (*modelviewmatrix)*repeatTransform;
	    osg::ref_ptr<osg::RefMatrix> rfMx = new osg::RefMatrix( MVR );

	    cv->addDrawable( _lineGeometry, rfMx );
	    cv->addDrawable( _triangleStrip, rfMx );
	}

	if ( getStateSet() ) cv->popStateSet();
    }
}


osg::Vec3 PlaneWellLog::getPrjDirection( const osgUtil::CullVisitor* cv ) const
{
    osg::Vec3 projDir( 0, 0, 0 );
    if( !cv ) return projDir;
    const osg::Camera* ca = const_cast<osgUtil::CullVisitor*>(cv)->getCurrentCamera();
    osg::Vec3 up;
    osg::Vec3 eye;
    osg::Vec3 center;
    ca->getViewMatrixAsLookAt( eye,center,up );
    projDir = center - eye;
    projDir.normalize();
    return projDir;
}


osg::Vec3 PlaneWellLog::updateNormal( const osg::Vec3 projDir) 
{
    return calcNormal(projDir);
}


void PlaneWellLog::calcCoordinates( const osg::Vec3& normal, float screensize )
{
    int nrsample = _logPath->size();

    const bool dofill = ( getLogItem() != LOGLINE_ONLY );
    const osg::Vec3 appliedDir = normal * screensize;

    for ( int idx=0; idx<nrsample; idx++ )
    {
	const float shpFactor = _coordLinedFactors->at( idx ) ;
	const osg::Vec3 pathcrd = _logPath->at( idx );
	
	(*_logLinedPoints)[idx]= pathcrd + appliedDir * shpFactor;

	if ( dofill )
	{
	    const int idx1 = 2*idx;
	    const int idx2 = idx1+1;
	    const float shpFactor1 = _coordLinedTriFactors->at( 2*idx ) ;
	    const float shpFactor2 = _coordLinedTriFactors->at( 2*idx + 1 ) ;
	    if ( (int)_logLinedTriPoints->size() > nrsample && 
		std::find(_outFillIndex.begin(),_outFillIndex.end(), idx) 
			    != _outFillIndex.end() )
	    {
		if( idx < (int)_logLinedTriPoints->size() )
		{
		    (*_logLinedTriPoints)[idx1] = 
			pathcrd + appliedDir * shpFactor1;
		    (*_logLinedTriPoints)[idx2] = 
			pathcrd + appliedDir * shpFactor1; 
		    continue;
		}
	    }
	    (*_logLinedTriPoints)[idx1] = pathcrd + appliedDir * shpFactor1;
	    (*_logLinedTriPoints)[idx2] = pathcrd + appliedDir * shpFactor2; 
	}
    }

    _logLinedPoints->dirty();
    if ( dofill )
	_logLinedTriPoints->dirty();
    _forceCoordRecalculation = false;

    _trianglePrimitiveSet->setCount( _logLinedTriPoints->size()  );
    _lineGeometry->dirtyDisplayList();
    _triangleStrip->dirtyDisplayList();

    const osg::BoundingBox& geometryBox = _lineGeometry->getBound();
    _lineGeometryWidth = fabs( geometryBox.xMax() - geometryBox.xMin() );

}


void PlaneWellLog::calcFactors()
{
    clearFactors();
    if( !_logPath->size() )
       return;
 
    float meanlogval(.0);
    int nrsamp = _logPath->size();

    unsigned int item = getLogItem();

    if ( item != LOGLINE_ONLY )
    {
	for ( int idx=0; idx<nrsamp; idx++ )
	{
	    float logval = _shapeLog->at(idx);
	    if ( _dispSide == Left ) 
		logval = _maxShapeValue - logval;
	    meanlogval += logval/nrsamp;
	}
    }
    
    const float meanFactor = getShapeFactor( meanlogval,
	_minShapeValue, _maxShapeValue ); 

    for ( int idx=0; idx<nrsamp; idx++ )
    {
	float logval = _shapeLog->at( idx );

	if ( ( item == LOGLNFL_BOTH && _revscale ) || 
	     ( item == LOGLINE_ONLY && _revscale ) ||
	     ( item == SEISMIC_ONLY && _dispSide == Left ) )
	{
	    logval = _maxShapeValue - logval;
	}

	logval = ( logval<_minShapeValue ) ? _minShapeValue : logval;
	logval = ( logval>_maxShapeValue ) ? _maxShapeValue : logval;

	_coordLinedFactors->push_back( getShapeFactor( logval,
	    _minShapeValue, _maxShapeValue ) );

        if ( item == SEISMIC_ONLY)
	{
	    _coordLinedTriFactors->push_back( getShapeFactor( logval,
		_minShapeValue, _maxShapeValue ) );
	    if( _dispSide == Left )
	    {
		if ( logval < meanlogval )
		    _coordLinedTriFactors->push_back( meanFactor );
		else
		    _coordLinedTriFactors->push_back( getShapeFactor( logval,
		    _minShapeValue, _maxShapeValue ));
	    }
	    else
	    {
		if ( logval > meanlogval )
		    _coordLinedTriFactors->push_back( meanFactor  );
		else
		    _coordLinedTriFactors->push_back( getShapeFactor( logval,
		    _minShapeValue, _maxShapeValue ) );
	    }
	}
	else
	{
	    if ( _fillrevscale)
	    {
		_coordLinedTriFactors->push_back( getShapeFactor( _maxFillValue,
		    _minShapeValue, _maxShapeValue ) );
	    }
	    else
		_coordLinedTriFactors->push_back( .0 );
	    
	    _coordLinedTriFactors->push_back( getShapeFactor( logval,
		_minShapeValue, _maxShapeValue ) );

	}

    }

    _logLinedPoints->resize( _coordLinedFactors->size() );
    _logLinedTriPoints->resize( _coordLinedTriFactors->size() );
    _LogColors->resize( _coordLinedTriFactors->size() );

    _linePrimitiveSet->setCount( _logLinedPoints->size() );
    _trianglePrimitiveSet->setCount( _logLinedTriPoints->size()  ); 

    _forceRebuild = false;
}


unsigned int PlaneWellLog::getLogItem()
{
    if ( _seisstyle )
	return SEISMIC_ONLY;
    if ( !_isFilled)
	return LOGLINE_ONLY;
    return LOGLNFL_BOTH;
}


float PlaneWellLog::getShapeFactor( float val, float minval, float maxval ) const
{
    float res = (val-minval)/(maxval-minval);
    if ( res<0 ) res = 0;
    else if ( res>1 ) res = 1;

    return _dispSide==Left ? -res : res;
}


osg::Vec3 PlaneWellLog::calcNormal( const osg::Vec3& projdir ) const
{
    osg::Vec3 res( 0 ,0, -1);
    res = res^projdir;
    if ( res.length2()<1e-6 )
	res = osg::Vec3( 1, 0, 0 );
    else
	res.normalize();

    return res;
}


float expectDepth(.0);
#define DIFFLIMITATION 0.005

bool searchDepth( float val )
{
    return ( ( val == expectDepth ) || 
	fabs(val - expectDepth ) <= DIFFLIMITATION );
}


void PlaneWellLog::updateFilledLogColor()
{
    if ( !_shapeLog->size() || !_isFilled )
	return;

    float colstep = ( _maxFillValue - _minFillValue ) / 255;
    int   colindex = 0;

    const int nrsamp = _logPath->size();
    float step = 1;
    _outFillIndex.clear();

    for ( int idx=0; idx<nrsamp; idx++ )
    {
	int index = int( idx*step+.5 );
	osg::Vec3f pos = _logPath->at(idx);
	expectDepth = pos[2];
	osg::FloatArray::iterator it = find_if( 
	    _fillLogDepths->begin(), _fillLogDepths->end(), searchDepth ) ; 
	size_t findex = std::distance(_fillLogDepths->begin(), it);
	if(findex == _fillLogDepths->size())
	{
	    _outFillIndex.push_back( idx );
	    continue;
	}
	index = (int)findex;
        float filllogval = _fillLog->at( index );
	colindex = (int)( ( filllogval-_minFillValue ) / colstep );
	colindex = ( colindex > 255 ) ? 255 : colindex;
	colindex = ( colindex < 0   ) ? 0   : colindex;
	(*_LogColors)[2*idx] =  _colorTable->at( colindex );
	(*_LogColors)[2*idx+1] =  _colorTable->at( colindex );
    }

    _triangleStrip->setColorArray( _LogColors );
    _clrTblChanged = false;

}


void PlaneWellLog::setPath( osg::Vec3Array* vtxArrayPath )
{
    _logPath = vtxArrayPath;
    _forceRebuild = true;
}


void PlaneWellLog::setShapeLog( osg::FloatArray* shapeLog )
{
    _shapeLog = shapeLog;
    _forceRebuild = true;
}


void PlaneWellLog::setFillLogValues( osg::FloatArray* fillLog )
{
    _fillLog = fillLog;
    _forceRebuild = true;
}


void PlaneWellLog::setMaxShapeValue( float maxValue )
{
    _maxShapeValue = maxValue;
    _forceRebuild = true;
}


void PlaneWellLog::setMinShapeValue( float minValue )
{
    _minShapeValue = minValue;
    _forceRebuild = true;
}


void PlaneWellLog::setMaxFillValue( float maxFillValue )
{
    _maxFillValue = maxFillValue;
    _forceRebuild = true;
}

void PlaneWellLog::setMinFillValue( float minFillValue )
{
    _minFillValue = minFillValue;
    _forceRebuild = true;
}


void PlaneWellLog::setScreenWidth( float screenWidth )
{
    _screenWidth = screenWidth;
    _forceRebuild = true;
    _screenSizeChanged = true;
}

bool PlaneWellLog::eyeChanged( const osg::Vec3 projDir )
{
    return ( projDir != _preProjDir ? true : false );
}


void PlaneWellLog::setLineColor( osg::Vec4d lnColor )
{
    (*_lineColor)[0] = lnColor;
    _lineColor->dirty();
}

void PlaneWellLog::setFillLogColorTab( osg::Vec4Array* logClrTab )
{
    _colorTable = logClrTab;
    _clrTblChanged = true;
}


void PlaneWellLog::setLineWidth( float lineWidth )
{
    _lineWidth->setWidth( lineWidth );
}

void PlaneWellLog::setShift( float shift )
{
    _shiftValue = shift;
    _forceRebuild = true;
}


void PlaneWellLog::setDisplaySide( PlaneWellLog::DisplaySide side )
{
    _dispSide = side;
    _forceRebuild = true;
}

void PlaneWellLog::setSeisLogStyle( bool stl )
{
    _seisstyle = stl;
    _forceRebuild = true;
}


void PlaneWellLog::setLogFill( bool isfill )
{
    _isFilled = isfill;
    _forceRebuild = true;
}

void PlaneWellLog::setLogConstantSize( bool rsz )
{
    _resizewhenzooming = !rsz;
    _forceRebuild = true;
}


void PlaneWellLog::setLogConstantSizeFactor( float fac )
{
    _constantsizefactor =  fac;
    _forceRebuild = true;
}


osg::Vec4d PlaneWellLog::getLineColor() const 
{
     return _lineColor->at(0);
}


const float PlaneWellLog::getLineWidth() const
{
    return _lineWidth->getWidth();
}


bool PlaneWellLog::getDisplayStatus()
{
    return ( _nonshadinggroup->getNumChildren() > 0 ? true : 0 );
}


void PlaneWellLog::setRepeatNumber( unsigned int repeatNumber )
{
    _repeatNumber = repeatNumber;
}


void PlaneWellLog::setRepeatGap ( float repeatGap )
{
    if( _lineGeometryWidth != 0 )
	_repeatGap = repeatGap*_lineGeometryWidth / 100; 
}


void PlaneWellLog::setShowLog( bool showLog )
{
    _showLog = showLog;
    if( _showLog )
	_forceCoordRecalculation = true;
    _forceRebuild = true;
}


void PlaneWellLog::setFillLogDepths( osg::FloatArray* depths )
{
    _fillLogDepths = depths; 
}


void PlaneWellLog::clearCoords()
{
    _logLinedPoints->clear();
    _logLinedTriPoints->clear();
}


void PlaneWellLog::clearLog()
{
    _colorTable->clear();
    _logPath->clear();	
    _shapeLog->clear();
    _fillLog->clear();
    _fillLogDepths->clear();
    clearCoords();
    _minShapeValue =  mMAX ;
    _minFillValue  =  mMAX ;
    _maxShapeValue = -mMAX ;
    _maxFillValue  = -mMAX;

}


void PlaneWellLog::clearDraw()
{
    _nonshadinggroup->removeChildren( 0, _nonshadinggroup->getNumChildren() );
}


osg::Vec2 PlaneWellLog::worldToScreen( const osg::Vec3& worldPosition,
						const osg::Camera* ca )
{
    osg::Vec2 screenPosition(0,0);

    if ( ca )
    {
	osg::Matrixd MVPW = ca->getViewMatrix() * ca->getProjectionMatrix() * 
	    ca->getViewport()->computeWindowMatrix();
	
	osg::Vec4d screenPosition4d = osg::Vec4d( worldPosition, 1.0) * MVPW;
	screenPosition4d = screenPosition4d / screenPosition4d.w();
	screenPosition4d.y()=ca->getViewport()->height()-screenPosition4d.y();
	screenPosition.set( screenPosition4d.x(), screenPosition4d.y() );
    }
    return screenPosition; 
}


osg::Vec3 PlaneWellLog::screenToWorld( const osg::Vec2d& screenPosition, 
						    const osg::Camera* ca )
{
    osg::Vec3 worldPosition (0, 0, 0);

    if ( ca)
    {
	osg::Vec4 screenPositionNear( screenPosition.x(), 
		ca->getViewport()->height() - screenPosition.y(), 0.0, 1.0 );
	osg::Vec4 screenPositionFar( screenPosition.x(), 
		ca->getViewport()->height() - screenPosition.y(), 1.0, 1.0 );
	osg::Matrixd iMVPW = osg::Matrixd::inverse( ca->getViewMatrix() * 
					     ca->getProjectionMatrix() * 
				ca->getViewport()->computeWindowMatrix() );
	osg::Vec4 worldPositionNear = screenPositionNear * iMVPW;
	osg::Vec4 worldPositionFar = screenPositionFar * iMVPW;
	worldPositionNear = worldPositionNear / worldPositionNear.w();
	worldPositionFar = worldPositionFar / worldPositionFar.w();

	worldPosition.set( ( worldPositionNear.x() + worldPositionFar.x() )/2.0, 
			  ( worldPositionNear.y() + worldPositionFar.y() )/2.0,  
			  ( worldPositionNear.z() + worldPositionFar.z() )/2.0);
    }

    return worldPosition; 
}


float PlaneWellLog::calcWorldWidth( const osgUtil::CullVisitor* cv )
{
    float worldWidth(.0);
    if( !cv ) 
	return 0;

    osg::Viewport* viewport = const_cast<osgUtil::CullVisitor*>(cv)->getViewport();
    float szpixel = viewport->height();
    float nsize1 = _screenWidth / szpixel; 
    int hnum = (int)_logPath->size() / 2;

    const osg::Vec3& worldPnt =  _logPath->at(hnum);
    osg::Vec2d scrPnt = worldToScreen( worldPnt,
	const_cast<osgUtil::CullVisitor*>(cv)->getCurrentCamera() );
    scrPnt[0] += nsize1;
    osg::Vec3 newwldpnt = screenToWorld( scrPnt,
	const_cast<osgUtil::CullVisitor*>(cv)->getCurrentCamera() );
    
    worldWidth = fabs( newwldpnt[0] - worldPnt[0] );

    if ( !_resizewhenzooming ) 
	worldWidth = nsize1*_constantsizefactor;
    _screenSizeChanged = false;

    return worldWidth;
}


void PlaneWellLog::clearFactors()
{
  _coordLinedFactors->clear();
  _coordLinedTriFactors->clear();
}

