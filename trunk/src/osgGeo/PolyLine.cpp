namespace osgGeo
{

PolylineNode::PolylineNode()
    : radius_( 5 )
    , maxRadius( -1 )
    , screenSizeScaling_( false )
    , arrayModifiedCount_( 0 )
{}


PolylineNode::PolylineNode( const PolylineNode& node )
    : radius_( node.radius_ )
    , maxRadius_( node.maxRadius_ )
    , screenSizeScaling_( node.screenSizeScaling_ )
    , array_( node.array_ )
    , geometry_( node.geometry_->clone() )
    , arrayModifiedCount_( 0 )
{}


void PolylineNode::traverse( osg::NodeVisitor& nv )
{
    if ( nv.getVisitorType()==osg::NodeVisitor::UPDATE_VISITOR )
    {
	if ( array_.valid() )
	{
	    if ( !arrayModifiedCount_ ||
		 arrayModifiedCount_!=array_->getModifiedCount() )
	    {
		createGeometry();
	    }
	}
    }
    else if ( nv.getVisitorType()==osg::NodeVisitor::CULL_VISTOR )
    {
	if ( geometry_.valid() )
	{
	    osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*>(&nv);

	    if ( getStateSet() )
		cv->pushStateSet();

	    cv->addDrawable( geometry_, cv->getModelViewMatrix() );

	    if ( getStateSet() )
		cv->popStateSet();
	}
    }
}


}
