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

#include "osgGeo/AutoTransform"

#include <osg/CullStack>


namespace osgGeo
{

AutoTransform::AutoTransform()
    :_restoreProportions(false)
{
}

AutoTransform::AutoTransform(const AutoTransform& pat,const osg::CopyOp& copyop)
    :osg::AutoTransform(pat,copyop),
    _restoreProportions(pat._restoreProportions)
{
}

void AutoTransform::accept(osg::NodeVisitor& nv)
{
    if (nv.validNodeMask(*this))
    {
	// if app traversal update the frame count.
	if (nv.getVisitorType()==osg::NodeVisitor::UPDATE_VISITOR)
	{
	}
	else
	    if (nv.getVisitorType()==osg::NodeVisitor::CULL_VISITOR)
	    {

		osg::CullStack* cs = dynamic_cast<osg::CullStack*>(&nv);
		if (cs)
		{

		    osg::Viewport::value_type width = _previousWidth;
		    osg::Viewport::value_type height = _previousHeight;

		    osg::Viewport* viewport = cs->getViewport();
		    if (viewport)
		    {
			width = viewport->width();
			height = viewport->height();
		    }

		    osg::Vec3d eyePoint = cs->getEyeLocal();
		    osg::Vec3d localUp = cs->getUpLocal();
		    osg::Vec3d position = getPosition();

		    const osg::Matrix& projection = *(cs->getProjectionMatrix());

		    bool doUpdate = _firstTimeToInitEyePoint;
		    if (!_firstTimeToInitEyePoint)
		    {
			osg::Vec3d dv = _previousEyePoint-eyePoint;
			if (dv.length2()>getAutoUpdateEyeMovementTolerance()*(eyePoint-getPosition()).length2())
			{
			    doUpdate = true;
			}
			osg::Vec3d dupv = _previousLocalUp-localUp;
			// rotating the camera only affects ROTATE_TO_*
			if (_autoRotateMode &&
			    dupv.length2()>getAutoUpdateEyeMovementTolerance())
			{
			    doUpdate = true;
			}
			else if (width!=_previousWidth || height!=_previousHeight)
			{
			    doUpdate = true;
			}
			else if (projection != _previousProjection)
			{
			    doUpdate = true;
			}
			else if (position != _previousPosition)
			{
			    doUpdate = true;
			}
		    }
		    _firstTimeToInitEyePoint = false;

		    if (doUpdate)
		    {

			if (getAutoScaleToScreen())
			{
			    const double nativesize = 1.0/cs->pixelSize(getPosition(),0.48f);
			    osg::Vec3d newscale( nativesize, nativesize, nativesize );
			    if ( _restoreProportions )
			    {
				osg::RefMatrix* modelviewmatrix = cs->getModelViewMatrix();
				if ( modelviewmatrix )
				{
				    osg::Vec3d translation;
				    osg::Quat rotation;
				    osg::Vec3d modelscale;
				    osg::Quat so;

				    modelviewmatrix->decompose( translation, rotation, modelscale, so );

				    modelscale.normalize();

				    newscale[0] = nativesize/modelscale[0];
				    newscale[1] = nativesize/modelscale[1];
				    newscale[2] = nativesize/modelscale[2];
				}
			    }

			    double length = newscale.length();
			    newscale.normalize();

			    if (_autoScaleTransitionWidthRatio>0.0)
			    {
				if (_minimumScale>0.0)
				{
				    double j = _minimumScale;
				    double i = (_maximumScale<DBL_MAX) ?
					_minimumScale+(_maximumScale-_minimumScale)*_autoScaleTransitionWidthRatio :
				    _minimumScale*(1.0+_autoScaleTransitionWidthRatio);
				    double c = 1.0/(4.0*(i-j));
				    double b = 1.0 - 2.0*c*i;
				    double a = j + b*b / (4.0*c);
				    double k = -b / (2.0*c);

				    if (length<k) length = _minimumScale;
				    else if (length<i) length = a + b*length + c*(length*length);
				}

				if (_maximumScale<DBL_MAX)
				{
				    double n = _maximumScale;
				    double m = (_minimumScale>0.0) ?
					_maximumScale+(_minimumScale-_maximumScale)*_autoScaleTransitionWidthRatio :
				    _maximumScale*(1.0-_autoScaleTransitionWidthRatio);
				    double c = 1.0 / (4.0*(m-n));
				    double b = 1.0 - 2.0*c*m;
				    double a = n + b*b/(4.0*c);
				    double p = -b / (2.0*c);

				    if (length>p) length = _maximumScale;
				    else if (length>m) length = a + b*length + c*(length*length);
				}
			    }

			    newscale *= length;

			    setScale(newscale);
			}

			if (_autoRotateMode==ROTATE_TO_SCREEN)
			{
			    osg::Vec3d translation;
			    osg::Quat rotation;
			    osg::Vec3d scale;
			    osg::Quat so;

			    cs->getModelViewMatrix()->decompose( translation, rotation, scale, so );

			    setRotation(rotation.inverse());
			}
			else if (_autoRotateMode==ROTATE_TO_CAMERA)
			{
			    osg::Vec3d PosToEye = _position - eyePoint;
			    osg::Matrix lookto = osg::Matrix::lookAt(
				osg::Vec3d(0,0,0), PosToEye, localUp);
			    osg::Quat q;
			    q.set(osg::Matrix::inverse(lookto));
			    setRotation(q);
			}
			else if (_autoRotateMode==ROTATE_TO_AXIS)
			{
			    osg::Matrix matrix;
			    osg::Vec3 ev(eyePoint - _position);

			    switch(_cachedMode)
			    {
			    case(AXIAL_ROT_Z_AXIS):
				{
				    ev.z() = 0.0f;
				    float ev_length = ev.length();
				    if (ev_length>0.0f)
				    {
					//float rotation_zrotation_z = atan2f(ev.x(),ev.y());
					//mat.makeRotate(inRadians(rotation_z),0.0f,0.0f,1.0f);
					float inv = 1.0f/ev_length;
					float s = ev.x()*inv;
					float c = -ev.y()*inv;
					matrix(0,0) = c;
					matrix(1,0) = -s;
					matrix(0,1) = s;
					matrix(1,1) = c;
				    }
				    break;
				}
			    case(AXIAL_ROT_Y_AXIS):
				{
				    ev.y() = 0.0f;
				    float ev_length = ev.length();
				    if (ev_length>0.0f)
				    {
					//float rotation_zrotation_z = atan2f(ev.x(),ev.y());
					//mat.makeRotate(inRadians(rotation_z),0.0f,0.0f,1.0f);
					float inv = 1.0f/ev_length;
					float s = -ev.z()*inv;
					float c = ev.x()*inv;
					matrix(0,0) = c;
					matrix(2,0) = s;
					matrix(0,2) = -s;
					matrix(2,2) = c;
				    }
				    break;
				}
			    case(AXIAL_ROT_X_AXIS):
				{
				    ev.x() = 0.0f;
				    float ev_length = ev.length();
				    if (ev_length>0.0f)
				    {
					//float rotation_zrotation_z = atan2f(ev.x(),ev.y());
					//mat.makeRotate(inRadians(rotation_z),0.0f,0.0f,1.0f);
					float inv = 1.0f/ev_length;
					float s = -ev.z()*inv;
					float c = -ev.y()*inv;
					matrix(1,1) = c;
					matrix(2,1) = -s;
					matrix(1,2) = s;
					matrix(2,2) = c;
				    }
				    break;
				}
			    case(ROTATE_TO_AXIS): // need to implement
				{
				    float ev_side = ev*_side;
				    float ev_normal = ev*_normal;
				    float rotation = atan2f(ev_side,ev_normal);
				    matrix.makeRotate(rotation,_axis);
				    break;
				}
			    }
			    osg::Quat q;
			    q.set(matrix);
			    setRotation(q);
			}

			_previousEyePoint = eyePoint;
			_previousLocalUp = localUp;
			_previousWidth = width;
			_previousHeight = height;
			_previousProjection = projection;
			_previousPosition = position;

			_matrixDirty = true;
		    }

		}
	    }

	    // now do the proper accept
	    Transform::accept(nv);
    }
}

} // end namespace

