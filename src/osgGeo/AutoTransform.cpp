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

*/

#include "osgGeo/AutoTransform"

#include <osg/CullStack>
#include <cstdio>
#include <iostream>


namespace osgGeo
{

AutoTransform::AutoTransform()
    :_restoreProportions(false),
    _scaledBack(false),
    _invMatrixDirty(true)
{
}


AutoTransform::AutoTransform(const AutoTransform& pat,const osg::CopyOp& copyop)
    :osg::AutoTransform(pat,copyop),
    _restoreProportions(pat._restoreProportions),
    _scaledBack(false),
    _invMatrixDirty(true)
{
}


bool AutoTransform::computeLocalToWorldMatrix(osg::Matrix& matrix,osg::NodeVisitor*) const
{
    if ( _scaledBack && _matrixDirty )
    {
	computeMatrix();

	if ( !_restoreProportions )
	     _cachedMatrix.preMult( osg::Matrix::scale(_modelViewScale) );

	_invMatrixDirty = true;
    }

    if (_referenceFrame==RELATIVE_RF)
	matrix.preMult(_cachedMatrix);
    else // absolute
	matrix = _cachedMatrix;

    return true;
}


bool AutoTransform::computeWorldToLocalMatrix(osg::Matrix& matrix,osg::NodeVisitor* nv) const
{
    if ( _invMatrixDirty )
    {
	osg::Matrix mat;
	mat.makeIdentity();
	computeLocalToWorldMatrix( mat,  nv );
	_cachedInvMatrix = osg::Matrix::inverse(mat);
	_invMatrixDirty = false;
    }

    matrix.postMult( _cachedInvMatrix );
    return true;
}


void AutoTransform::accept(osg::NodeVisitor& nv)
{
    if ( !nv.validNodeMask(*this) )
	return;

    if ( nv.getVisitorType()!=osg::NodeVisitor::CULL_VISITOR )
	return;

    osg::CullStack* cs = dynamic_cast<osg::CullStack*>(&nv);
    osg::ref_ptr<osg::RefMatrix> mvm = new osg::RefMatrix(*cs->getModelViewMatrix());

    if ( _firstTimeToInitEyePoint || _lastModelViewMatrix!=*mvm )
    {
	osg::Vec3d trans;
	osg::Quat rot, so;
	mvm->decompose( trans, rot, _modelViewScale, so );
	_lastModelViewMatrix = *mvm;
    }

    osg::Vec3d invMVS( 1.0/_modelViewScale[0], 1.0/_modelViewScale[1], 1.0/_modelViewScale[2] );
    mvm->preMult( osg::Matrix::scale(invMVS) );

    cs->pushModelViewMatrix( mvm.get(), _referenceFrame );

    const osg::Vec3d usrPos = _position;
    const osg::Vec3d usrPivot = _pivotPoint;
    const double usrMinScale = _minimumScale;
    const double usrMaxScale = _maximumScale;

    _position = osg::Matrix::scale(_modelViewScale).preMult( usrPos );
    _pivotPoint = osg::Matrix::scale(_modelViewScale).preMult( usrPivot );
    _minimumScale = 0.0;
    _maximumScale = DBL_MAX;
    _scaledBack = true;

    osg::AutoTransform::accept( nv );

    _scaledBack = false;
    _position = usrPos;
    _pivotPoint = usrPivot;
    _minimumScale = usrMinScale;
    _maximumScale = usrMaxScale;                                               
                                    
    cs->popModelViewMatrix();
}


} // end namespace

