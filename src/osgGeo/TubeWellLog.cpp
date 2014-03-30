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

#include <osgGeo/TubeWellLog>
#include <osgGeo/ComputeBoundsVisitor>
#include <osg/MatrixTransform>
#include <osgViewer/Viewer>
#include <osg/PolygonOffset>
#include <osg/PolygonMode>
#include <osg/LineWidth>
#include <osg/Vec3>
#include <osgUtil/SmoothingVisitor>
#include <osg/LightModel>
#include <osg/CullFace>

#define mMAX 1e30
#define INIRESOLUTION 30

using namespace osgGeo;


TubeWellLog::TubeWellLog()
    : WellLog()
    , _logTubeShapePoints(new osg::Vec3Array)
    , _logTubeCircleNormals(new osg::Vec3Array)
    , _logTubeVerts (new osg::Vec3Array)
    , _logColors(new osg::Vec4Array)
    , _logPathVerts(new osg::Vec3Array)
    , _resolution(INIRESOLUTION)
{
    buildTubeGeometry();
    buildCenterLineGeometry();
    _bbox.init();
}


TubeWellLog::TubeWellLog( const TubeWellLog& twl, const osg::CopyOp& cop )
    : WellLog( twl, cop )
    ,_logTubeShapePoints(new osg::Vec3Array)
    ,_logTubeCircleNormals(new osg::Vec3Array)
    ,_logTubeVerts (new osg::Vec3Array)
    ,_logColors(new osg::Vec4Array)
    ,_logPathVerts(new osg::Vec3Array)
    , _resolution( twl._resolution )
{}


TubeWellLog::~TubeWellLog()
{
    clearVerts();
    WellLog::clearDraw();
}


void TubeWellLog::clearLog()
{
    clearVerts();
    WellLog::clearLog();

}


void TubeWellLog::buildCenterLineGeometry()
{
    _logPathGeometry = new osg::Geometry();
    _geode->addDrawable(_logPathGeometry);

    _logPathColor =  new osg::Vec4Array;
    _logPathColor->push_back(osg::Vec4d(0.2, 0, 0.6, 0));

    osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array;
    normals->push_back(osg::Vec3 (0.0f,-1.0f,0.0f));

    _logPathPrimitiveSet = new osg::DrawArrays(
	osg::PrimitiveSet::LINE_STRIP, 0, 0);

    _logPathGeometry->setVertexArray(_logPathVerts.get());
    _logPathGeometry->setColorArray(_logPathColor.get());
    _logPathGeometry->setColorBinding(osg::Geometry::BIND_OVERALL);
    _logPathGeometry->setNormalArray(normals.get());
    _logPathGeometry->setNormalBinding(osg::Geometry::BIND_OVERALL);
   
    _logPathGeometry->addPrimitiveSet( _logPathPrimitiveSet );

    _logPathWidth =  new osg::LineWidth;
    _logPathWidth->setWidth(3.0);
    osg::PolygonOffset* polyoffset = new osg::PolygonOffset;
    polyoffset->setFactor(1.0f);
    polyoffset->setUnits(1.0f);
    getOrCreateStateSet()->setAttributeAndModes(_logPathWidth);
    getStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    getStateSet()->setAttributeAndModes( 
	polyoffset,osg::StateAttribute::OVERRIDE|osg::StateAttribute::ON);
}


void TubeWellLog::buildTubeGeometry()
{
    osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array;
    normals->push_back(osg::Vec3( 0.0f, -1.0f, 0.0f ));

    _tubeGeometry  = new osg::Geometry();
    _geode->addDrawable(_tubeGeometry);
    _tubeGeometry->setNormalArray(normals.get());
//    _tubeGeometry->setNormalArray(_logTubeCircleNormals.get());
//  this is temporal solution, it will be figured out later on the normal issue

    _tubeGeometry->setNormalBinding(osg::Geometry::BIND_OVERALL);
    _tubeGeometry->setVertexArray(_logTubeVerts.get());
    _tubeLogColors = new osg::Vec4Array;
    _tubeGeometry->setColorArray(_tubeLogColors.get());
    _tubeGeometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
    _tubeGeometry->setDataVariance(osg::Object::DYNAMIC);

    setRenderMode( RenderFrontSide );
}



void TubeWellLog::setRenderMode( RenderMode mode )
{
    osg::StateSet* stateset = getOrCreateStateSet();
    if ( !stateset )
	return;

    osg::ref_ptr<osg::LightModel> lightmodel = new osg::LightModel;
    lightmodel->setTwoSided( true );
    stateset->setAttributeAndModes( lightmodel, osg::StateAttribute::ON );

    stateset->removeAttribute( osg::StateAttribute::CULLFACE );
    if ( mode == RenderBothSides )
	return;

    osg::ref_ptr<osg::CullFace> cullface = new osg::CullFace;
    cullface->setMode( mode>=RenderFrontSide
        ? osg::CullFace::FRONT
	: osg::CullFace::BACK );
    stateset->setAttributeAndModes( cullface, osg::StateAttribute::ON );
}


TubeWellLog::RenderMode TubeWellLog::getRenderMode() const
{
    const osg::StateSet* stateset = getStateSet();
    const osg::LightModel* lightmodel = stateset
        ? dynamic_cast<const osg::LightModel*>(stateset->getAttribute(osg::StateAttribute::LIGHTMODEL))
        : 0;

    const osg::CullFace* cullface =
    	dynamic_cast<const osg::CullFace*>(stateset->getAttribute(osg::StateAttribute::CULLFACE));

    if ( !stateset || !lightmodel || lightmodel->getTwoSided() || !cullface )
        return RenderBothSides;

    return cullface->getMode()==osg::CullFace::FRONT ? RenderFrontSide : RenderBackSide;
}


void TubeWellLog::traverse(osg::NodeVisitor& nv)
{
    WellLog::traverse( nv );
    if(!_logPath->size())
	return;

    if (nv.getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR)
    {
	if (_colorTableChanged)
	    updateTubeLogColor();
	dirtyBound();
    }

    if (nv.getVisitorType() == osg::NodeVisitor::CULL_VISITOR)
    {
	osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*>(&nv);
	if (_forceReBuild)
	{
	    calcTubeShape(calcWorldWidth(cv));
	    buildTube(true); 
	    _colorTableChanged = true;
	}

    }
    else
    {
	osgGeo::ComputeBoundsVisitor* cbv =
	    dynamic_cast<osgGeo::ComputeBoundsVisitor*>( &nv );
	if ( cbv )
	    cbv->applyBoundingBox(_bbox);
    }
	

}


void TubeWellLog::buildTube(bool dirtybound) 
{
    const int nrSamples = _logPath->size();

    if (!nrSamples)
	return;

    const double angle(osg::PI * 2./(double)_resolution);

    _tubeGeometry->removePrimitiveSet(0,_tubeGeometry->getNumPrimitiveSets());

    osg::BoundingBox bbox;
    bbox.init();

    int total(0);
    for (int res = 0; res <_resolution + 1; res++)
    {
	osg::ref_ptr<osg::DrawElementsUShort> drawElements = 
	    new osg::DrawElementsUShort(GL_TRIANGLE_STRIP,2*nrSamples);

	int count(0);

	for (int idx = 0; idx<nrSamples; idx++)
	{
	    const osg::Vec3 pathCoord = (*_logPath)[idx];
	    bbox.expandBy( pathCoord );
	    const osg::Vec3 edgePnt = pathCoord + (*_logTubeShapePoints)[idx];
	    osg::Vec3 pnt1 = edgePnt - pathCoord;
	    pnt1.normalize();
	    
	    osg::Vec3 ciclecNormal = (*_logTubeCircleNormals)[idx];
	    ciclecNormal.normalize();

	    const osg::Vec3 pnt2 = ciclecNormal^pnt1;
	    float radius = fabs((edgePnt - pathCoord ).length());

	    osg::Vec3 circlePnt = pathCoord +
		(pnt1*sin(res*angle) + pnt2*cos( res*angle )) * radius;

	    if (std::find(_outTubeIndex.begin(),_outTubeIndex.end(), idx) 
			    != _outTubeIndex.end() )
		 (*_logTubeVerts)[total] =  pathCoord;
	    else
		 (*_logTubeVerts)[total] = circlePnt;

	    bbox.expandBy((*_logTubeVerts)[total]);
		
	    (*drawElements)[2*count] = total;
	    (*drawElements)[2*count+1] = total + nrSamples;
	    count++;
	    total++;
	}

	if(res < _resolution)
	    _tubeGeometry->addPrimitiveSet(drawElements);
	
    }
    
    _logPathVerts->clear();
    _logPathVerts->insert(_logPathVerts->end(),_logPath->begin(),
			  _logPath->end() );

    if ( _bbox._min != bbox._min || _bbox._max != bbox._max )
    {
	dirtyBound(); 
	dirtybound = false;
	_bbox = bbox;
    }

    if ( dirtybound )
    {
	_bbox.init();
	dirtyBound();
    }

    _logTubeVerts->dirty();
    _tubeGeometry->dirtyDisplayList();
   // osgUtil::SmoothingVisitor::smooth(*_tubeGeometry);
    _logPathGeometry->dirtyDisplayList();


}


osg::BoundingSphere TubeWellLog::computeBound() const
{
    if ( _bbox.valid() )
       return _bbox;

    osg::BoundingBox bbox;

    for ( unsigned int idx=0; idx<_logPath->size(); idx++ )
	bbox.expandBy((*_logPath)[idx]);

    return bbox;

}


const osg::Vec4& TubeWellLog::getLineColor() const
{
    static osg::Vec4 res;
    return res;
}


void TubeWellLog::setResolution(int resolution)
{
    _resolution = resolution; 
    _forceReBuild = true;
}


float TubeWellLog::getShapeFactor(float val, float minval, float maxval) const
{
    return WellLog::getShapeFactor(val, minval, maxval);
}


void TubeWellLog::calcTubeShape(float screenSize)
{
    clearTubeShape();
 
    if(!_logPath->size())
	return;

    int nrSamples = _logPath->size();
    for (int idx=0; idx<nrSamples; idx++)
    {
	float logval = _shapeLog->at(idx);

	if(_revScale)
	    logval = _maxShapeValue - logval;

	logval = (logval<_minShapeValue) ? _minShapeValue : logval;
	logval = (logval>_maxShapeValue) ? _maxShapeValue : logval;

	const float shpFactor =  getShapeFactor( 
				logval, _minShapeValue, _maxShapeValue);

	osg::Vec3 circleNormal;
	if (idx<nrSamples-1)
	    circleNormal = _logPath->at(idx+1) - _logPath->at(idx);
	else
	    circleNormal = _logPath->at(idx-1) - _logPath->at(idx);

	osg::Vec3 appliedDir = osg::Vec3(circleNormal[2],0,-circleNormal[0]);		
	appliedDir.normalize();
	appliedDir *= screenSize;

	_logTubeShapePoints->push_back(appliedDir*shpFactor);
	_logTubeCircleNormals->push_back(circleNormal);

    }

     _logPathPrimitiveSet->setCount(_logPath->size());
     _logTubeVerts->resize((_resolution+1)*nrSamples);

    _forceReBuild = false;
}


void TubeWellLog::updateTubeLogColor()
{

    if (!_shapeLog->size())
	return;

    const float clrStep = (_maxFillValue - _minFillValue)/255;
    int   clrIndex = 0;

    const int nrSamples = _logPath->size();
    _outTubeIndex.clear();

    const osg::FloatArray::iterator itmin = std::min_element(
	_fillLogDepths->begin(), _fillLogDepths->end());
    const float minfillz = *itmin;

    const osg::FloatArray::iterator itmax = std::max_element(
	_fillLogDepths->begin(), _fillLogDepths->end());
    const float maxfillz = *itmax;

    _logColors->resize(nrSamples);

    for (int idx=0; idx<nrSamples; idx++)
    {
	const osg::Vec3f pos = _logPath->at(idx);

	if (pos[2]<minfillz || pos[2]>maxfillz)
	{
	    _outTubeIndex.push_back(idx);
	    (*_logColors)[idx] = _colorTable->at(1);
	    continue;
	}

	const int fillIndex = getClosestIndex(*_fillLogDepths, pos[2]);
	if ( fillIndex < 0 )
	    continue;

	const float fillLogVal = _fillLog->at((int)fillIndex);
	clrIndex = (int)((fillLogVal-_minFillValue )/clrStep);
	clrIndex = (clrIndex>255) ? 255 : clrIndex;
	clrIndex = (clrIndex<0) ? 0   : clrIndex;
	(*_logColors)[idx] = _colorTable->at(clrIndex) ;
    }

    _tubeLogColors->resize((_resolution+1 )*nrSamples);

    int total(0);
    for (int i=0; i<=_resolution; i++)
    {
	for (int idx=0; idx<nrSamples; idx++)
	{
	     (*_tubeLogColors)[total] = (*_logColors)[idx];
	     total++;
	}
	
    }

    _colorTableChanged = false;
}


void TubeWellLog::clearTubeShape()
{
    _logTubeShapePoints->clear();
    _logTubeCircleNormals->clear();
}


void TubeWellLog::clearVerts()
{
    _logTubeVerts->clear();
}

#include <osgDB/ObjectWrapper>
#include <osgDB/Registry>
#include <osgDB/Serializer>

REGISTER_OBJECT_WRAPPER( TubeWellLog_Wrapper,
                        new TubeWellLog,
                        osgGeo::TubeWellLog,
                        "osg::Object osg::Node osgGeo::WellLog osgGeo::TubeWellLog")
{
    BEGIN_ENUM_SERIALIZER( RenderMode, RenderBackSide );
        ADD_ENUM_VALUE( RenderBackSide );
        ADD_ENUM_VALUE( RenderBothSides );
        ADD_ENUM_VALUE( RenderFrontSide );
    END_ENUM_SERIALIZER();
}

