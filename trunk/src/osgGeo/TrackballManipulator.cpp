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

#include <osgGeo/TrackballManipulator>

#include <osgUtil/LineSegmentIntersector>
#include <osg/ComputeBoundsVisitor>
#include <osg/Timer>

#define mAllTraversals (0xFFFFFFFF)

namespace osgGeo
{


TrackballManipulator::TrackballManipulator( int flags )
    : osgGA::TrackballManipulator( flags )
    , _dragEnabled( true )
    , _boundTraversalMask( mAllTraversals )
    , _viewallMargin( 0.2f )
    , _viewAllInitalFactor( 3.5f )
{
}


TrackballManipulator::TrackballManipulator( const TrackballManipulator& tm, const osg::CopyOp& copyOp )
    : osgGA::TrackballManipulator( tm, copyOp )
    , osg::Object( tm, copyOp )	// needs explicit init in copy constructor because of [-Wextra] warning
    , _dragEnabled( tm._dragEnabled )
    , _boundTraversalMask( tm._boundTraversalMask )
    , _viewallMargin( tm._viewallMargin )
    , _viewAllInitalFactor( tm._viewAllInitalFactor )
{
}


TrackballManipulator::~TrackballManipulator()
{}


bool TrackballManipulator::computeViewAllParams(osg::View* view,
                                                osg::Vec3d& center, double& distance) const
{
    if ( !_node )
        return false;

    osg::ComputeBoundsVisitor visitor( osg::NodeVisitor::TRAVERSE_ACTIVE_CHILDREN );
    visitor.setNodeMaskOverride( _boundTraversalMask );

    _node->accept( visitor );

    osg::BoundingBox &bb = visitor.getBoundingBox();
    if ( bb.valid() )
    {
        osg::BoundingSphere boundingSphere;
        boundingSphere.expandBy(bb);
        if ( !boundingSphere.valid() )
            return false;

        distance = _viewAllInitalFactor * boundingSphere._radius;
        center = boundingSphere.center();

        if ( !view || !view->getCamera() || !view->getCamera()->getViewport() )
            return true;

        const osg::Viewport* vp = view->getCamera()->getViewport();
        const osg::Matrix projMatrix = view->getCamera()->getProjectionMatrix();
        const osg::Matrix windowMatrix = vp->computeWindowMatrix();

        //Results are better if this is iterated, and two times is a nice compromize between
        //speed and result.
        for ( int idx=0; idx<2; idx++ )
        {
            const osg::Matrix viewMatrix = getInverseMatrix( center, _rotation, distance );
            const osg::Matrix transform = viewMatrix * projMatrix * windowMatrix;

            osg::BoundingBox screenBBox;
            screenBBox.expandBy( osg::Vec3( bb.xMin(), bb.yMin(), bb.zMin() ) * transform );
            screenBBox.expandBy( osg::Vec3( bb.xMin(), bb.yMin(), bb.zMax() ) * transform );
            screenBBox.expandBy( osg::Vec3( bb.xMin(), bb.yMax(), bb.zMin() ) * transform );
            screenBBox.expandBy( osg::Vec3( bb.xMin(), bb.yMax(), bb.zMax() ) * transform );
            screenBBox.expandBy( osg::Vec3( bb.xMax(), bb.yMin(), bb.zMin() ) * transform );
            screenBBox.expandBy( osg::Vec3( bb.xMax(), bb.yMin(), bb.zMax() ) * transform );
            screenBBox.expandBy( osg::Vec3( bb.xMax(), bb.yMax(), bb.zMin() ) * transform );
            screenBBox.expandBy( osg::Vec3( bb.xMax(), bb.yMax(), bb.zMax() ) * transform );

            const float xWidth = (screenBBox.xMax()-screenBBox.xMin())/vp->width();
            const float yWidth = (screenBBox.yMax()-screenBBox.yMin())/vp->height();

            const float factor = (xWidth>yWidth ? xWidth : yWidth) * (1.0f+_viewallMargin);

            distance = factor * distance;
        }
    }
    else
    {
        osg::BoundingSphere boundingSphere = _node->getBound();

        if ( !boundingSphere.valid() )
            return false;

        distance = _viewAllInitalFactor * boundingSphere._radius;
        center = boundingSphere.center();
    }
    
    return true;
}


osg::Matrix TrackballManipulator::getInverseMatrix() const
{
    return getInverseMatrix( _center, _rotation, _distance );
}


osg::Matrix TrackballManipulator::getInverseMatrix(const osg::Vec3d& center,
                                                   const osg::Quat& rotation,
                                                   double distance )
{
    return osg::Matrixd::translate( -center ) *
        osg::Matrixd::rotate( rotation.inverse() ) *
        osg::Matrixd::translate( 0.0, 0.0, -distance );
}


bool TrackballManipulator::handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
{
    if ( ea.getEventType()==osgGA::GUIEventAdapter::FRAME )
        return false;

    if ( !_dragEnabled && ea.getEventType()==osgGA::GUIEventAdapter::DRAG )
	return false;

    osg::Vec3d oldEyePos, oldCenterPos, oldUpDir;
    getTransformation( oldEyePos, oldCenterPos, oldUpDir );
    const double oldDist = _distance;

    const bool res = osgGA::TrackballManipulator::handle( ea, aa );

    if ( _cb )
    {
        float horAngle = 0;
        float vertAngle = 0;
        if ( ea.getEventType()==osgGA::GUIEventAdapter::DRAG )
        {
            osg::Vec3 oldViewDir = oldEyePos-oldCenterPos;
            oldViewDir.normalize();

            osg::Vec3d newEyePos, newCenterPos, newUpDir;
            getTransformation( newEyePos, newCenterPos, newUpDir );

            osg::Vec3 newViewDir = newEyePos-newCenterPos;
            newViewDir.normalize();

            const float rotationAngle = newViewDir * oldViewDir;
            if ( rotationAngle )
            {
                const osg::Vec3 rotationAxis = newViewDir^oldViewDir;

                osg::Vec3 vertAxis = (oldViewDir)^oldUpDir;
                vertAxis.normalize();
                osg::Vec3 horAxis = oldUpDir;
                horAxis.normalize();

                horAngle = rotationAngle * (horAxis*rotationAxis);
                vertAngle = rotationAngle * (vertAxis*rotationAxis);
            }
	}

        TrackballEventNodeVisitor nv( horAngle, vertAngle,
                                     (_distance-oldDist)/oldDist );
        (*_cb)( 0, &nv );
    }

    return res;
}


TrackballEventNodeVisitor::TrackballEventNodeVisitor( float deltahorangle, float deltavertangle, float distfactor )
    : _deltahorangle( deltahorangle )
    , _deltavertangle( deltavertangle )
    , _distfactor( distfactor )
{}


TrackballEventNodeVisitor::~TrackballEventNodeVisitor()
{}

    

#define mDefaultHandling osgGA::TrackballManipulator::handleMouseWheel( ea, us )
    

bool TrackballManipulator::handleMouseWheel( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us )
{
    const osgGA::GUIEventAdapter::ScrollingMotion sm = ea.getScrollingMotion();

    if( ((sm == osgGA::GUIEventAdapter::SCROLL_DOWN && _wheelZoomFactor > 0.)) ||
       ((sm == osgGA::GUIEventAdapter::SCROLL_UP   && _wheelZoomFactor < 0.)) )
    {
        return handleMouseWheelZoomIn( ea, us );
    }

    if( ((sm == osgGA::GUIEventAdapter::SCROLL_UP && _wheelZoomFactor > 0.)) ||
       ((sm == osgGA::GUIEventAdapter::SCROLL_DOWN   && _wheelZoomFactor < 0.)) )
    {
        return handleMouseWheelZoomOut( ea, us );
    }

    return mDefaultHandling;
}
    

bool TrackballManipulator::handleMouseWheelZoomOut( const osgGA::GUIEventAdapter& ea,
                                                    osgGA::GUIActionAdapter& us )
{

    if ( !_node.get() )
        return mDefaultHandling;

    osg::Vec3d oldEyePos, oldCenterPos, oldUpDir;
    getTransformation( oldEyePos, oldCenterPos, oldUpDir );


    /*General idea is that if we are close, we move the eyepoint towards the
      viewAll eye-point, and move the center towards the viewAll-center.
     
      If we are not close, we simply move eyepoint backwards (standard interaction */

    osg::Vec3d viewAllCenter;
    double viewAllDistance;


    if ( !computeViewAllParams( us.asView(), viewAllCenter,viewAllDistance) )
        return mDefaultHandling;

    const double currentDistFromIdealCenter = (oldEyePos-viewAllCenter).length();
    if ( currentDistFromIdealCenter>viewAllDistance )
        return mDefaultHandling;

    osg::Vec3d viewDir = oldEyePos - oldCenterPos;
    const double viewDirLength = viewDir.length();
    if ( viewDirLength<1.0 )
        return mDefaultHandling;

    viewDir /= viewDirLength;

    osg::Vec3d viewAllPos = viewAllCenter + viewDir * viewAllDistance;

    const osg::Vec3d eyePath = viewAllPos - oldEyePos;
    const double eyePathLength = eyePath.length();
    if ( eyePathLength<1 )
        return mDefaultHandling;
    
    const osg::Vec3d eyePathDir = eyePath / eyePathLength;

    double eyeMovementLength = _distance * _wheelZoomFactor;
    if ( eyeMovementLength> eyePathLength )
        eyeMovementLength = eyePathLength;

    const double factor = eyeMovementLength / eyePathLength;

    const osg::Vec3d newEyePos = oldEyePos + eyePathDir * eyeMovementLength;

    const osg::Vec3d centerPath =  viewAllCenter - oldCenterPos;
    const double centerPathLength = centerPath.length();
    osg::Vec3d newCenterPos = oldCenterPos;
    if ( centerPathLength>1 )
    {
        const osg::Vec3d centerPathDir = centerPath / centerPathLength;
        double centerMovementLength = centerPathLength * factor;
        newCenterPos = oldCenterPos + centerPathDir * centerMovementLength;
    }

    setTransformation( newEyePos, newCenterPos, oldUpDir );

    return true;
}


bool TrackballManipulator::handleMouseWheelZoomIn( const osgGA::GUIEventAdapter& ea,
                                                   osgGA::GUIActionAdapter& us )
{
    osg::Vec3d intersectionPos;
    if ( !getIntersectionPoint(ea,us,intersectionPos) )
    {
        return mDefaultHandling;
    }

    //General idea is that we move centerpoint towards the eye, and the eye-point towards
    //the intersection point.
    //If we are close we will move the center of rotation further away though to avoid getting stuck

    osg::Vec3d oldEyePos, oldCenterPos, oldUpDir;
    getTransformation( oldEyePos, oldCenterPos, oldUpDir );

    osg::Vec3d movementDir = intersectionPos-oldEyePos;
    const double oldDistance = movementDir.length();
    if ( oldDistance<1.0 )
    {
        return mDefaultHandling;
    }

    movementDir /= oldDistance;

    double minDist = _minimumDistance;
    if( getRelativeFlag( _minimumDistanceFlagIndex ) )
        minDist *= _modelSize;

    double movementDist = _wheelZoomFactor * oldDistance;
    if ( oldDistance-movementDist<minDist )
    {
        movementDist = _wheelZoomFactor*minDist;
    }

    const osg::Vec3d movement = movementDir*movementDist;
    const osg::Vec3d newEyePos = oldEyePos + movement;
    osg::Vec3d newCenterDir = oldCenterPos - oldEyePos;
    const double centerDist = newCenterDir.length();

    newCenterDir /= centerDist;

    double centerMovementDist = centerDist * _wheelZoomFactor;
    if ( centerDist-centerMovementDist<minDist )
    {
        centerMovementDist = _wheelZoomFactor*minDist;
    }

    osg::Vec3d newCenterPos = newEyePos + newCenterDir * (centerDist-centerMovementDist);
    setTransformation( newEyePos, newCenterPos, oldUpDir );

    return true;
}


void TrackballManipulator::animateTo(const osg::Vec3d& newCenter,
                                     const osg::Quat& newRotation,
                                     double newDistance,
                                     bool animate )
{
    if ( animate && getAnimationTime() )
    {
        TrackballAnimationData* ad = dynamic_cast<TrackballAnimationData*>( _animationData.get() );
        if ( ad )
        {
            ad->start( newCenter-_center, newRotation-_rotation,
                      newDistance-_distance, osg::Timer::instance()->time_s() );
            return;
        }
    }

    _center = newCenter;
    _distance = newDistance;
    _rotation = newRotation;
}


void TrackballManipulator::viewAll( osg::View* view, bool animate )
{
    osg::Vec3d newCenter;
    double newDistance;

    computeViewAllParams( view, newCenter, newDistance );

    animateTo( newCenter, _rotation, newDistance, animate );
}


bool TrackballManipulator::getIntersectionPoint( const osgGA::GUIEventAdapter& ea,
                                                 osgGA::GUIActionAdapter& us,
                                                 osg::Vec3d& intersection ) const
{
    osg::View* view = us.asView();
    if( !view )
        return false;

    osg::Camera *camera = view->getCamera();
    if( !camera )
        return false;

    // perform intersection computation
    osg::ref_ptr< osgUtil::LineSegmentIntersector > picker =
    	new osgUtil::LineSegmentIntersector( osgUtil::Intersector::WINDOW, ea.getX(), ea.getY() );
    osgUtil::IntersectionVisitor iv( picker.get() );
    iv.setTraversalMask( _intersectTraversalMask );
    camera->accept( iv );

    // return on no intersections
    if( !picker->containsIntersections() )
        return false;

    // get all intersections
    osgUtil::LineSegmentIntersector::Intersections& intersections = picker->getIntersections();

    // new center
    intersection = (*intersections.begin()).getWorldIntersectPoint();


    return true;
}



void TrackballManipulator::TrackballAnimationData::start( const osg::Vec3d& centerMovement,
               const osg::Quat& rotationMovement,
               double distanceMovement,
               const double startTime )
{
    AnimationData::start( startTime );

    _centerMovement = centerMovement;
    _rotationMovement = rotationMovement;
    _distanceMovement = distanceMovement;
}


void  TrackballManipulator::applyAnimationStep( const double currentProgress, const double prevProgress )
{
    TrackballAnimationData* ad = dynamic_cast< TrackballAnimationData* >( _animationData.get() );
    if ( !ad )
        return;

    const double prevMovementProgress = 0.5 * (1 - cos(prevProgress*M_PI));
    const double currentMovementProgress = 0.5 * (1 - cos(currentProgress*M_PI));

    const double progress = (currentMovementProgress - prevMovementProgress);

    // compute new center
    _center += ad->_centerMovement * progress;
    _rotation += ad->_rotationMovement * progress;
    _distance += ad->_distanceMovement * progress;
}


void TrackballManipulator::addMovementCallback(osg::NodeCallback* nc)
{
    if ( !_cb )
	_cb = nc;
    else
	_cb->addNestedCallback( nc );
}


void TrackballManipulator::removeMovementCallback(osg::NodeCallback* nc)
{
    if ( nc==_cb )
        _cb = _cb->getNestedCallback();
    else
        _cb->removeNestedCallback( nc );


}




} // end namespace

