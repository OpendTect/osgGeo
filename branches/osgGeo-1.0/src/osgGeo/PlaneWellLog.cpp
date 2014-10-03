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
#include <osgGeo/ComputeBoundsVisitor>
#include <osg/MatrixTransform>
#include <osgViewer/Viewer>
#include <osg/PolygonOffset>
#include <osg/PolygonMode>
#include <osg/LineWidth>
#include <osg/Vec3>

#define mMAX 1e30

namespace osgGeo
{


PlaneWellLog::PlaneWellLog()
    : WellLog()
    ,_coordLinedFactors(new osg::FloatArray)
    ,_coordLinedTriFactors(new osg::FloatArray)
    ,_logLinedPoints(new osg::Vec3Array)
    ,_logLinedTriPoints (new osg::Vec3Array)
    ,_logColors(new osg::Vec4Array)
    ,_dispSide(Left)
    ,_repeatNumber(1)
    ,_repeatGap(100.0f)
    ,_seisStyle(false)
    ,_isFilled(false)
    ,_triGeometryWidth(.0)
    ,_isFullFilled(false)
{
    buildLineGeometry();
    buildTriangleGeometry();
}


PlaneWellLog::PlaneWellLog( const PlaneWellLog& pwl, const osg::CopyOp& cop )
    : WellLog( pwl, cop )
    ,_coordLinedFactors(new osg::FloatArray)
    ,_coordLinedTriFactors(new osg::FloatArray)
    ,_logLinedPoints(new osg::Vec3Array)
    ,_logLinedTriPoints (new osg::Vec3Array)
    ,_logColors(new osg::Vec4Array)
    ,_dispSide(Left)
    ,_repeatNumber(1)
    ,_repeatGap(100.0f)
    ,_seisStyle(false)
    ,_isFilled(false)
    ,_triGeometryWidth(.0)
    ,_isFullFilled(false)
{
    buildLineGeometry();
    buildTriangleGeometry();
}



PlaneWellLog::~PlaneWellLog()
{
   WellLog::clearDraw();
   clearCoords();
}


void PlaneWellLog::clearLog()
{
    clearCoords();
    WellLog::clearLog();

}


void PlaneWellLog::setSeisLogStyle(bool stl)
{
    _seisStyle = stl;
    _forceReBuild = true;
}


void PlaneWellLog::setDisplaySide(PlaneWellLog::DisplaySide side)
{
    _dispSide = side;
    _forceReBuild = true;
}


void PlaneWellLog::setRepeatNumber(unsigned int repeatnumber)
{
    if ( _repeatNumber != repeatnumber )
    {
	_repeatNumber = repeatnumber;
	dirtyBound();
    }
}


void PlaneWellLog::setRepeatGap (float repeatgap)
{
    if ( _repeatGap != repeatgap )
    {
	_repeatGap = repeatgap;
	if ( getRepeatNumber() > 1 )
	    dirtyBound();
    }
}


float PlaneWellLog::getRepeatStep() const
{
    float repeatStep = _repeatGap;
    if ( _triGeometryWidth != 0  )
	repeatStep *= _triGeometryWidth / 100.0f;

    return _dispSide==Right ? repeatStep : -repeatStep;
}


void PlaneWellLog::setFullFilled(bool isfullpanel)
{
    _isFullFilled = isfullpanel;
    _forceReBuild = true;
}


void PlaneWellLog::clearFactors()
{
    _coordLinedFactors->clear();
    _coordLinedTriFactors->clear();
}


void PlaneWellLog::clearCoords()
{
    _logLinedPoints->clear();
    _logLinedTriPoints->clear();
}


float PlaneWellLog::getShapeFactor(float val, float minval, float maxval )const
{
    float res = WellLog::getShapeFactor(val, minval, maxval);
    return _dispSide==Left ? -res : res;
}


void PlaneWellLog::buildLineGeometry()
{
    _lineGeometry = new osg::Geometry();
    _geode->addDrawable(_lineGeometry);
    _lineGeometry->setVertexArray(_logLinedPoints.get());
    _lineGeometry->setColorArray(_lineColor.get());
    _lineGeometry->setColorBinding(osg::Geometry::BIND_OVERALL);
    osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array;
    normals->push_back( osg::Vec3 (0.0f,-1.0f,0.0f ));
    _lineGeometry->setNormalArray(normals.get());
    _lineGeometry->setNormalBinding(osg::Geometry::BIND_OVERALL);
    _linePrimitiveSet = new osg::DrawArrays(
	osg::PrimitiveSet::LINE_STRIP, 0, 0);
    _lineGeometry->addPrimitiveSet(_linePrimitiveSet);

    osg::PolygonOffset* polyoffset = new osg::PolygonOffset;
    polyoffset->setFactor(1.0f);
    polyoffset->setUnits(1.0f);
    getOrCreateStateSet()->setAttributeAndModes(_lineWidth);
    getStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    getStateSet()->setAttributeAndModes( 
	polyoffset,osg::StateAttribute::OVERRIDE|osg::StateAttribute::ON);
}


void PlaneWellLog::buildTriangleGeometry()
{
    _triangleGeometry  = new osg::Geometry();
    _geode->addDrawable( _triangleGeometry );
    osg::ref_ptr<osg::Vec3Array> shared_normals = new osg::Vec3Array;
    shared_normals->push_back(osg::Vec3( 0.0f, -1.0f, 0.0f ));

    _triangleGeometry->setVertexArray(_logLinedTriPoints.get());
    _triangleGeometry->setNormalArray(shared_normals.get());
    _triangleGeometry->setNormalBinding(osg::Geometry::BIND_OVERALL);

    _logColors = new osg::Vec4Array;
    _triangleGeometry->setColorArray(_logColors.get());
    _triangleGeometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

    _trianglePrimitiveSet = new osg::DrawArrays(
	osg::PrimitiveSet::TRIANGLE_STRIP, 0, 0);

    _triangleGeometry->addPrimitiveSet(_trianglePrimitiveSet);
    _triangleGeometry->setDataVariance(osg::Object::DYNAMIC);
}


void PlaneWellLog::traverse(osg::NodeVisitor& nv)
{
    WellLog::traverse( nv );

    if( !_logPath->size() )
	return;

    if ( nv.getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR )
    {
	if ( _forceReBuild )
	{
	    calcFactors();
	    _colorTableChanged = true;
	    _forceCoordReCalculation = true;
	}

	if ( _colorTableChanged )
	    updateFilledLogColor();
    }
    else if ( nv.getVisitorType() == osg::NodeVisitor::CULL_VISITOR )
    {
	osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*>(&nv);
	const osg::Vec3 projdir = getPrjDirection(cv);
	const osg::Vec3 normal = calcNormal(projdir);
	
	if ( _forceCoordReCalculation || eyeChanged(projdir) || 
	    _logWidthChanged )
	{
	    calcCoordinates(normal); 
	    _preProjDir = projdir;
	    _logWidthChanged = false;

	    if ( _lineWidth->getWidth() == 0 )
		getStateSet()->removeAttribute(_lineWidth);
	    else
		getStateSet()->setAttributeAndModes(_lineWidth);
	    
	    dirtyBound();
	}

	osg::ref_ptr<osg::MatrixTransform> tr = new osg::MatrixTransform;
	osg::ref_ptr<osg::RefMatrix> modelViewMatrix = 
	    const_cast<osgUtil::CullVisitor*>(cv)->getModelViewMatrix();
	
	if ( getStateSet() ) cv->pushStateSet( getStateSet() );

	osg::Matrix repeatTransform;
	osg::BoundingBox bbox;
	for ( int idx=0; idx<(int)_repeatNumber; idx++ )
	{
	    repeatTransform.setTrans( normal*idx*getRepeatStep() );
	    osg::Matrix RMV = repeatTransform * (*modelViewMatrix);
	    osg::ref_ptr<osg::RefMatrix> rfMx = new osg::RefMatrix(RMV);

	    const osg::BoundingBox bbtri = _triangleGeometry->getBound() ;
	    const osg::BoundingBox bbline = _lineGeometry->getBound();
	    bbox.expandBy( bbtri );
	    bbox.expandBy( bbline );

	    if ( bbox.radius() == 0 )
		return;

	    const float depth = cv->getDistanceFromEyePoint(bbox.center(),false);
	    cv->addDrawableAndDepth( _lineGeometry, rfMx, depth );
	    cv->addDrawableAndDepth( _triangleGeometry, rfMx, depth );
	}

	if ( _bbox._min != bbox._min || _bbox._max != bbox._max )
	{
	    dirtyBound();
	    _bbox = bbox;
	}

	if ( getStateSet() ) cv->popStateSet();
    }
    else
    {
	osgUtil::IntersectionVisitor* iv =
			dynamic_cast<osgUtil::IntersectionVisitor*>(&nv);

	if ( iv && iv->getModelMatrix() )
	{
	    const osg::Vec3 normal = calcNormal( _preProjDir );

	    osg::Matrix repeatTransform;
	    for ( int idx=0; idx<(int)_repeatNumber; idx++ )
	    {
		repeatTransform.setTrans(normal*idx*getRepeatStep());

		osg::Matrix mat = repeatTransform * (*iv->getModelMatrix());
		osg::ref_ptr<osg::RefMatrix> rfMx = new osg::RefMatrix(mat);

		iv->pushModelMatrix( rfMx );

		osg::ref_ptr<osgUtil::Intersector> intersec = 
		    iv->getIntersector()->clone(*iv);
		if ( intersec.valid() )
		{
		    intersec->intersect(*iv, _lineGeometry);
		    intersec->intersect(*iv, _triangleGeometry);
		}

		iv->popModelMatrix();
	    }
	}

	osgGeo::ComputeBoundsVisitor* cbv =
	    dynamic_cast<osgGeo::ComputeBoundsVisitor*>(&nv);
	if ( cbv )
	    cbv->applyBoundingBox(_bbox);
    }

}


PlaneWellLog::RenderMode PlaneWellLog::getLogItem() const
{
    if ( _seisStyle )
	return SEISMIC_ONLY;
    if ( !_isFilled)
	return LOGLINE_ONLY;

    return LOGLNFL_BOTH;
}


void PlaneWellLog::setLogFill(bool isFill)
{
    _isFilled = isFill;
    _forceReBuild = true;
}


osg::BoundingSphere PlaneWellLog::computeBound() const
{
    if ( _bbox.valid() )
	return _bbox;

    osg::BoundingSphere logSphere;

    for( unsigned int idx=0; idx<_logPath->size(); idx++ )
    {
	float rad = 0.0f;
	if ( idx < _coordLinedFactors->size() )
	    rad = 
	    osg::maximum<float>( rad, fabs((*_coordLinedFactors)[idx]));
	if ( 2*idx < _coordLinedTriFactors->size() )
	    rad = 
	    osg::maximum<float>( rad, fabs((*_coordLinedTriFactors)[2*idx]));
	if ( 2*idx+1 < _coordLinedTriFactors->size() )
	    rad = 
	    osg::maximum<float>( rad, fabs((*_coordLinedTriFactors)[2*idx+1]));

	rad = rad*_logWidth + fabs(getRepeatStep())*(_repeatNumber-1);
	logSphere.expandBy(osg::BoundingSphere(_logPath->at(idx),rad));
    }

    return logSphere;
}


void PlaneWellLog::calcCoordinates(const osg::Vec3& normal)
{
    int nrSamples = _logPath->size();

    const bool doFill = (getLogItem() != LOGLINE_ONLY);
    const osg::Vec3 appliedDir = normal * _logWidth;
    const osg::Vec3 emptyPnt( 0, 0, 0 );

    for ( int idx=0; idx<nrSamples; idx++ )
    {
	const float shpFactor = _coordLinedFactors->at(idx) ;
	const osg::Vec3 pathCoord = _logPath->at(idx);
	
	(*_logLinedPoints)[idx] = (_lineWidth->getWidth() > 0)
	    ? ( pathCoord + appliedDir * shpFactor ) : emptyPnt;

	if ( doFill )
	{
	    const int idx1 = 2*idx;
	    const int idx2 = idx1+1;
	    const float shpfactor1 = _coordLinedTriFactors->at(2*idx) ;
	    const float shpfactor2 = _coordLinedTriFactors->at(2*idx + 1) ;
	    if ( (int)_logLinedTriPoints->size() > nrSamples && 
		std::find(_outFillIndex.begin(),_outFillIndex.end(), idx) 
			    != _outFillIndex.end() )
	    {
		if( idx < (int)_logLinedTriPoints->size() )
		{
		    (*_logLinedTriPoints)[idx1] = 
			pathCoord + appliedDir * shpfactor1;
		    (*_logLinedTriPoints)[idx2] = 
			pathCoord + appliedDir * shpfactor1; 
		    continue;
		}
	    }
	    (*_logLinedTriPoints)[idx1] = pathCoord + appliedDir * shpfactor1;
	    (*_logLinedTriPoints)[idx2] = pathCoord + appliedDir * shpfactor2; 
	}
    }

    _logLinedPoints->dirty();
    if ( doFill )
	_logLinedTriPoints->dirty();
    _forceCoordReCalculation = false;

    _trianglePrimitiveSet->setCount(_logLinedTriPoints->size());
    _lineGeometry->dirtyDisplayList();
    _triangleGeometry->dirtyDisplayList();
    _lineGeometry->dirtyBound();
    _triangleGeometry->dirtyBound();

    const osg::BoundingBox& bb = _triangleGeometry->getBound();
    _triGeometryWidth = 
	osg::Vec2(bb.xMax()-bb.xMin(), bb.yMax()-bb.yMin() ).length();

    if ( getRepeatNumber() > 1 )
	dirtyBound();
}


void PlaneWellLog::calcFactors()
{
    clearFactors();
    if( !_logPath->size() )
       return;
 
    float meanLogVal( .0 );
    int nrSamples = _logPath->size();

    unsigned int item = getLogItem();

    if ( item != LOGLINE_ONLY )
    {
	for ( int idx=0; idx<nrSamples; idx++ )
	{
	    float logval = _shapeLog->at(idx);
	    if ( _dispSide == Left && !_isFullFilled) 
		logval = _maxShapeValue - logval;
	    meanLogVal += logval/nrSamples;
	}
    }
    
    const float meanFactor = getShapeFactor(meanLogVal,
	_minShapeValue, _maxShapeValue); 

    for ( int idx=0; idx<nrSamples; idx++ )
    {
	float logVal = _shapeLog->at( idx );

	if ( !_isFullFilled && 
	   ((item == LOGLNFL_BOTH && _revScale) || 
	     (item == LOGLINE_ONLY && _revScale) ||
	     (item == SEISMIC_ONLY && _dispSide == Left)))
	{
	    logVal = _maxShapeValue - logVal;
	}

	logVal = (logVal<_minShapeValue) ? _minShapeValue : logVal;
	logVal = (logVal>_maxShapeValue) ? _maxShapeValue : logVal;

	_coordLinedFactors->push_back(getShapeFactor(logVal,
	    _minShapeValue, _maxShapeValue));

        if ( item == SEISMIC_ONLY)
	{
	    _coordLinedTriFactors->push_back( getShapeFactor( logVal,
		_minShapeValue, _maxShapeValue ) );

	    if( _dispSide == Left )
	    {
		if ( logVal < meanLogVal )
		    _coordLinedTriFactors->push_back(meanFactor);
		else
		    _coordLinedTriFactors->push_back(getShapeFactor(logVal,
		    _minShapeValue, _maxShapeValue ));
	    }
	    else
	    {
		if ( logVal > meanLogVal )
		    _coordLinedTriFactors->push_back(meanFactor);
		else
		    _coordLinedTriFactors->push_back(getShapeFactor(logVal,
		    _minShapeValue, _maxShapeValue ));
	    }
	}
	else
	{
	    if (_fillRevScale)
	    {
		_coordLinedTriFactors->push_back(getShapeFactor(_maxFillValue,
		    _minShapeValue, _maxShapeValue ));
	    }
	    else
		_coordLinedTriFactors->push_back(.0);

	    _coordLinedTriFactors->push_back(getShapeFactor(logVal,
		_minShapeValue, _maxShapeValue ));

	}
    }

    _logLinedPoints->resize(_coordLinedFactors->size());
    _logLinedTriPoints->resize(_coordLinedTriFactors->size());
    _logColors->resize(_coordLinedTriFactors->size() );

    _linePrimitiveSet->setCount(_logLinedPoints->size());
    _trianglePrimitiveSet->setCount(_logLinedTriPoints->size()); 

    _forceReBuild = false;
    dirtyBound();
}


osg::Vec3 PlaneWellLog::calcNormal(const osg::Vec3& projdir) const
{
    osg::Vec3 res(0 , 0, -1);
    res = res^projdir;
    if ( res.length2()<1e-6 )
	res = osg::Vec3(1, 0, 0);
    else
	res.normalize();

    return res;
}


void PlaneWellLog::updateFilledLogColor()
{
    if( getLogItem() == SEISMIC_ONLY )
    {
	for ( unsigned int idx=0; idx<_logPath->size(); idx++ )
	{
	    (*_logColors)[2*idx] =  _colorTable->at(1);
	    (*_logColors)[2*idx+1] =  _colorTable->at(1);
	}
	return ;
    }
    
    if ( !_shapeLog->size() || !_isFilled)
	return;

    const float clrStep = ( _maxFillValue - _minFillValue ) / 255;
    int   clrIndex = 0;

    const int nrSamples = _logPath->size();
    _outFillIndex.clear();

    const osg::FloatArray::iterator itmin = std::min_element(
	_fillLogDepths->begin(), _fillLogDepths->end());
    const float minfillz = *itmin;

    const osg::FloatArray::iterator itmax = std::max_element(
	_fillLogDepths->begin(), _fillLogDepths->end());
    const float maxfillz = *itmax;

    for ( int idx=0; idx<nrSamples; idx++ )
    {
	const osg::Vec3f pos = _logPath->at(idx);

	if( pos[2] < minfillz || pos[2] > maxfillz )
	{
	    _outFillIndex.push_back(idx);
	    continue;
	}

	const int fillIndex = getClosestIndex(*_fillLogDepths, pos[2]);
	if ( fillIndex < 0 )
	    continue;

	const float fillLogVal = _fillLog->at(fillIndex);
	clrIndex = (int)((fillLogVal-_minFillValue )/clrStep);
	clrIndex = (clrIndex > 255) ? 255 : clrIndex;
	clrIndex = (clrIndex < 0  ) ? 0   : clrIndex;
	(*_logColors)[2*idx] =  _colorTable->at(clrIndex);
	(*_logColors)[2*idx+1] =  _colorTable->at(clrIndex);
    }

    _colorTableChanged = false;

}

} //namespace

#include <osgDB/ObjectWrapper>
#include <osgDB/Registry>
#include <osgDB/Serializer>


REGISTER_EMPTY_OBJECT_WRAPPER( PlaneWellLog_Wrapper,
	new osgGeo::PlaneWellLog,
	osgGeo::PlaneWellLog,
	"osg::Object osg::Node osgGeo::WellLog osgGeo::PlaneWellLog");
