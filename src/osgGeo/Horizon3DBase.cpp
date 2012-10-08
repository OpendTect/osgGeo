//      =================================================================
//      |                                                               |
//      |                       COPYRIGHT (C) 2012                      |
//      |               ARK CLS Ltd, Bedford, Bedfordshire, UK          |
//      |                                                               |
//      |                       All Rights Reserved                     |
//      |                                                               |
//      | This software is confidential information which is proprietary|
//      | to and a trade secret of ARK-CLS Ltd. Use, duplication, or    |
//      | disclosure is subject to the terms of a separate source code  |
//      | licence agreement.                                            |
//      |                                                               |
//      =================================================================
//
//

#include "Horizon3DBase"


namespace osgGeo
{

Horizon3DBase::Horizon3DBase()
{
    setNumChildrenRequiringUpdateTraversal(getNumChildrenRequiringUpdateTraversal()+1);
    _needsUpdate = true;
}

Horizon3DBase::Horizon3DBase(const Horizon3DBase& other,
                             const osg::CopyOp& op) :
    osg::Node(other, op)
{
    // TODO Proper copy
}

void Horizon3DBase::setSize(const Vec2i& size)
{
    _size = size;
}

const Vec2i& Horizon3DBase::getSize() const
{
    return _size;
}

void Horizon3DBase::setDepthArray(osg::Array *arr)
{
    _array = arr;
    _needsUpdate = true;
    updateGeometry();
}

const osg::Array *Horizon3DBase::getDepthArray() const
{
    return _array;
}

osg::Array *Horizon3DBase::getDepthArray()
{
    return _array;
}

void Horizon3DBase::setMaxDepth(float val)
{
    _maxDepth = val;
}

float Horizon3DBase::getMaxDepth() const
{
    return _maxDepth;
}

bool Horizon3DBase::isUndef(double val)
{
    return val >= getMaxDepth();
}

void Horizon3DBase::setCornerCoords(const std::vector<osg::Vec2d> &coords)
{
    _cornerCoords = coords;
}

std::vector<osg::Vec2d> Horizon3DBase::getCornerCoords() const
{
    return _cornerCoords;
}

bool Horizon3DBase::needsUpdate() const
{
    return _needsUpdate;
}

void Horizon3DBase::traverse(osg::NodeVisitor &nv)
{
    if ( nv.getVisitorType()==osg::NodeVisitor::UPDATE_VISITOR )
    {
        if ( needsUpdate() )
            updateGeometry();
    }
    else if(nv.getVisitorType()==osg::NodeVisitor::CULL_VISITOR)
    {
        for(unsigned int i = 0; i <  _nodes.size(); ++i)
            _nodes.at(i)->accept(nv);
    }
}

Horizon3DTileNode::Horizon3DTileNode()
{
    setNumChildrenRequiringUpdateTraversal(getNumChildrenRequiringUpdateTraversal()+1);
    _nodes.resize(3);
    _pointLineNodes.resize(3);
}

Horizon3DTileNode::Horizon3DTileNode(const Horizon3DTileNode&, const osg::CopyOp& )
    : osg::MatrixTransform()
{
    setNumChildrenRequiringUpdateTraversal(getNumChildrenRequiringUpdateTraversal()+1);
}

void Horizon3DTileNode::setCornerCoords(const std::vector<osg::Vec2d> &coords)
{
    _cornerCoords = coords;
    _center = osg::Vec3((coords[1] + coords[2]) / 2, 0.0);
}

std::vector<osg::Vec2d> Horizon3DTileNode::getCornerCoords() const
{
    return _cornerCoords;
}

void Horizon3DTileNode::setSize(const Vec2i &size)
{
    _size = size;
}

Vec2i Horizon3DTileNode::getSize() const
{
    return _size;
}

void Horizon3DTileNode::traverseSubNode(int lod, osg::NodeVisitor &nv)
{
    _nodes[lod]->accept(nv);
    if(_pointLineNodes[lod].get())
        _pointLineNodes[lod]->accept(nv);
}

void Horizon3DTileNode::traverse(osg::NodeVisitor &nv)
{
    if(nv.getVisitorType()==osg::NodeVisitor::CULL_VISITOR)
    {
        const float distance = nv.getDistanceToViewPoint(getCenter(), true);

        const std::vector<osg::Vec2d> coords = getCornerCoords();
        const float iDen = ((coords[2] - coords[0]) / getSize().x()).length();
        const float jDen = ((coords[1] - coords[0]) / getSize().y()).length();

        const float k = std::min(iDen, jDen);
        const float threshold1 = k * 2000.0;
        const float threshold2 = k * 8000.0;

        if(distance < threshold1)
            traverseSubNode(0, nv);
        else if(distance < threshold2)
            traverseSubNode(1, nv);
        else
            traverseSubNode(2, nv);
    }
}

osg::Vec3 Horizon3DTileNode::getCenter() const
{
    return _center;
}

void Horizon3DTileNode::setNode(int resolution, osg::Node *node)
{
    _nodes[resolution] = node;
}

void Horizon3DTileNode::setPointLineNode(int resolution, osg::Node *node)
{
    _pointLineNodes[resolution] = node;
}

osg::BoundingSphere Horizon3DTileNode::computeBound() const
{
    return _bs;
}

void Horizon3DTileNode::setBoundingSphere(const osg::BoundingSphere &boundingSphere)
{
    _bs = boundingSphere;
    dirtyBound();
}


}
