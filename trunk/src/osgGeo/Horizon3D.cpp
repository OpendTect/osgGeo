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


#include <osg/Geometry>

#include <osgGeo/Horizon3D>

#include <iostream>

namespace osgGeo
{

Horizon3DNode::Horizon3DNode() :
    Geode()
{

}

Horizon3DNode::Horizon3DNode(const Horizon3DNode& other,
              const osg::CopyOp& op) :
    Geode(other, op)
{

}

void Horizon3DNode::setSize(const Size &size)
{
    _size = size;
}

Size Horizon3DNode::getSize() const
{
    return _size;
}

void Horizon3DNode::setDepthArray(osg::Array *arr)
{
    _array = arr;
    updateDrawables();
}

const osg::Array *Horizon3DNode::getDepthArray() const
{
    return _array;
}

osg::Array *Horizon3DNode::getDepthArray()
{
    return _array;
}

void Horizon3DNode::updateDrawables()
{
    if(getDepthArray()->getType() != osg::Array::DoubleArrayType)
        return;

    osg::DoubleArray &depthVals =
            *(dynamic_cast<osg::DoubleArray*>(getDepthArray()));

    const int allHSize = getSize().width;
    const int allVSize = getSize().height;

    const int hSize = allHSize;
    const int vSize = allVSize;

    osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array(hSize * vSize);

    // first we construct an array of vertices which is just a grid
    // of depth values.
    for(int i = 0; i < hSize; ++i)
        for(int j = 0; j < vSize; ++j)
        {
            float val = depthVals[i*allVSize+j];
            if(isUndef(val))
                val = 0.0;

            (*vertices)[i*vSize+j] = osg::Vec3(
                        float(i) / hSize,
                        val,
                        float(j) / vSize
                        );
        }

    // the following loop populates array of indices that make up
    // triangles out of vertices data, each grid cell has 2 triangles.
    // If a vertex is undefined then triangle that contains it is
    // discarded. Also normals are calculated for each triangle to be
    // later used for calculating normals per vertex
    osg::ref_ptr<osg::DrawElementsUInt> indices =
            new osg::DrawElementsUInt(GL_TRIANGLES);

    osg::ref_ptr<osg::Vec3Array> triangleNormals =
            new osg::Vec3Array((hSize - 1) * (vSize - 1) * 2);

    for(int i = 0; i < hSize - 1; ++i)
        for(int j = 0; j < vSize - 1; ++j)
        {
            float val1 = depthVals[i*allVSize+j];
            float val2 = depthVals[(i+1)*allVSize+j];
            float val3 = depthVals[i*allVSize+(j+1)];
            float val4 = depthVals[(i+1)*allVSize+(j+1)];

            if(isUndef(val2) || isUndef(val3))
                continue;

            int i1 = i*vSize+j;
            int i2 = (i+1)*vSize+j;
            int i3 = i*vSize+(j+1);
            int i4 = (i+1)*vSize+(j+1);

            // first triangle
            if(!isUndef(val1))
            {
                indices->push_back(i1);
                indices->push_back(i2);
                indices->push_back(i3);
            }

            // second triangle
            if(!isUndef(val4))
            {
                indices->push_back(i2);
                indices->push_back(i3);
                indices->push_back(i4);
            }

            // calculate triangle normals
            osg::Vec3 v1 = (*vertices)[i1];
            osg::Vec3 v2 = (*vertices)[i2];
            osg::Vec3 v3 = (*vertices)[i3];
            osg::Vec3 v4 = (*vertices)[i4];

            osg::Vec3 norm1 = (v3 - v1) ^ (v2 - v1);
            norm1.normalize();
            (*triangleNormals)[(i*(vSize-1)+j)*2] = norm1;

            osg::Vec3 norm2 = (v2 - v4) ^ (v3 - v4);
            norm2.normalize();
            (*triangleNormals)[(i*(vSize-1)+j)*2+1] = norm2;
        }

    // The following loop calculates normals per vertex. Because
    // each vertex might be shared between many triangles(up to 6)
    // we find out which triangles this particular vertex is shared
    // and then compute the average of normals per triangle.
    osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array(hSize * vSize);

    osg::Vec3 triNormCache[6];

    for(int i = 0; i < hSize; ++i)
        for(int j = 0; j < vSize; ++j)
        {
            int k = 0;
            for(int l = 0; l < k; ++l)
                triNormCache[l] = osg::Vec3();

            if((i < hSize - 1) && (j < vSize - 1))
            {
                triNormCache[k++] = (*triangleNormals)[(i*(vSize-1)+j)*2];
            }

            if(i > 0 && j < vSize - 1)
            {
                triNormCache[k++] = (*triangleNormals)[((i-1)*(vSize-1)+j)*2];
                triNormCache[k++] = (*triangleNormals)[((i-1)*(vSize-1)+j)*2+1];
            }

            if(j > 0 && i < hSize - 1)
            {
                triNormCache[k++] = (*triangleNormals)[(i*(vSize-1)+j-1)*2];
                triNormCache[k++] = (*triangleNormals)[(i*(vSize-1)+j-1)*2+1];
            }

            if(i > 0 && j > 0)
            {
                triNormCache[k++] = (*triangleNormals)[((i-1)*(vSize-1)+j-1)*2+1];
            }

            if(k > 0)
            {
                osg::Vec3 norm;
                for(int l = 0; l < k; ++l)
                    norm += triNormCache[l];

                norm /= k;
                norm.normalize();

                (*normals)[i*vSize+j] = -norm;
            }
        }

    osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
    colors->push_back(osg::Vec4(1.0f, 0.0f, 1.0f, 1.0f));

    osg::ref_ptr<osg::Geometry> quad = new osg::Geometry;
    quad->setVertexArray(vertices.get());

    quad->setNormalArray(normals.get());
    quad->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);
    quad->setColorArray(colors.get());
    quad->setColorBinding(osg::Geometry::BIND_OVERALL);

    quad->addPrimitiveSet(indices.get());

    addDrawable(quad.get());
}

bool Horizon3DNode::isUndef(double val)
{
    return val >= _maxdepth;
}

void Horizon3DNode::setMaxDepth(float val)
{
    _maxdepth = val;
}

float Horizon3DNode::getMaxDepth() const
{
    return _maxdepth;
}

}
