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

#ifndef HORIZON3D2_H
#define HORIZON3D2_H

#include <osg/Group>
#include <osgGeo/Common>
#include <osgGeo/Vec2i>

namespace osgGeo
{

class OSGGEO_EXPORT Horizon3D2 : public osg::Group
{
public:
    Horizon3D2();

    // size of the grid
    void setSize(const Vec2i& size);
    const Vec2i& getSize() const;

    void setDepthArray(osg::Array*);
    const osg::Array* getDepthArray() const;
    osg::Array* getDepthArray();

    //! real world coordinate of the positions (0, 0), (0, height) and (width, 0)
    void setCornerCoords(const std::vector<osg::Vec2d> &coords);
    std::vector<osg::Vec2d> getCornerCoords() const;

    //! Everything larger than this will be treated as undef
    void setMaxDepth(float);
    float getMaxDepth() const;

private:
    void updateGeometry();

    Vec2i _size;
    std::vector<osg::Vec2d> _cornerCoords;
    osg::ref_ptr<osg::Array> _array;
    bool _needsUpdate;
    float _maxDepth;
};

}

#endif // HORIZON3D2_H
