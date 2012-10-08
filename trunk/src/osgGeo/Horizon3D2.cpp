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
// $Id$
//
//
//

#include "Horizon3D2"

#include <iostream>
#include <cstdlib>

#include <osg/Geometry>
#include <osg/Geode>
#include <osg/Texture2D>
#include <osg/BoundingBox>

#include <osgGeo/Vec2i>
#include <osgGeo/Palette>
#include <osgGeo/ShaderUtility.h>

#include <climits>

namespace osgGeo
{

namespace
{

struct BoundCallback : public osg::Node::ComputeBoundingSphereCallback
{
   BoundCallback() {}

   virtual osg::BoundingSphere computeBound(const osg::Node&) const
   {
       osg::BoundingBox bb(osg::Vec3(0, -0.5, 0), osg::Vec3(2.25, 0.5, 1));
       return osg::BoundingSphere(bb);
   }

private:

};

}

Horizon3DTileNode2::Horizon3DTileNode2()
{
}

void Horizon3DTileNode2::traverse(osg::NodeVisitor &nv)
{
    if(nv.getVisitorType()==osg::NodeVisitor::CULL_VISITOR)
    {
        _nodes.at(0)->accept(nv);
    }
}

Horizon3DNode2::Horizon3DNode2()
{
}

void Horizon3DNode2::updateGeometry()
{
    osgGeo::Vec2i fullSize = getSize();

    osg::DoubleArray &depthVals = *dynamic_cast<osg::DoubleArray*>(getDepthArray());

    double min = +999999.0;
    double max = -999999.0;
    for(int j = 0; j < fullSize.x(); ++j)
    {
        for(int i = 0; i < fullSize.y(); ++i)
        {
            const double val = depthVals.at(i * fullSize.y() + j);
            if(isUndef(val))
                continue;
            min = std::min(val, min);
            max = std::max(val, max);
        }
    }
    const double diff = max - min;

    std::vector<osg::Vec2d> coords = getCornerCoords();

    osg::Vec2d iInc = (coords[2] - coords[0]) / (fullSize.x() - 1);
    osg::Vec2d jInc = (coords[1] - coords[0]) / (fullSize.y() - 1);

    const osgGeo::Vec2i tileSize(255, 255);

    const int compr = 1;

    const int hSize = tileSize.x() + 1;
    const int vSize = tileSize.y() + 1;

    osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array(hSize * vSize);
    osg::ref_ptr<osg::DrawElementsUInt> indices =
            new osg::DrawElementsUInt(GL_TRIANGLES);
    osg::ref_ptr<osg::Vec2Array> tCoords = new osg::Vec2Array(hSize * vSize);

    osg::Vec2 texStart(0.0, 0.0);
    osg::Vec2 textureTileStep(1.0, 1.0);

    for(int i = 0; i < hSize; ++i)
    {
        for(int j = 0; j < vSize; ++j)
        {
            osg::Vec2d hor = iInc * i + jInc * j;
            (*vertices)[i*vSize+j] = osg::Vec3(hor.x(), hor.y(), 0);
            (*tCoords)[i*vSize+j] = texStart + osg::Vec2(
                        float(i) / (hSize - 1) * textureTileStep.x(),
                        float(j) / (vSize - 1) * textureTileStep.y()
                        );
        }
    }

    for(int i = 0; i < hSize - 1; ++i)
    {
        for(int j = 0; j < vSize - 1; ++j)
        {
            const int i00 = i*vSize+j;
            const int i10 = (i+1)*vSize+j;
            const int i01 = i*vSize+(j+1);
            const int i11 = (i+1)*vSize+(j+1);

            // first triangle
            indices->push_back(i00);
            indices->push_back(i10);
            indices->push_back(i01);

            // second triangle
            indices->push_back(i10);
            indices->push_back(i01);
            indices->push_back(i11);
        }
    }

    osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
    geom->setVertexArray(vertices.get());
    geom->setTexCoordArray(0, tCoords.get());
    geom->addPrimitiveSet(indices.get());

    osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
    colors->push_back(osg::Vec4(1.0f, 0.0f, 1.0f, 1.0f)); // needs to be white!

    geom->setColorArray(colors.get());
    geom->setColorBinding(osg::Geometry::BIND_OVERALL);

    // ------------- geom finished ----------------

    ShaderUtility su;
    su.addDefinition("hasGeomShader");
    osg::Program* programGeom = su.createProgram("horizon3d_vert.glsl", "horizon3d_frag.glsl",
                                                 "horizon3d_geom.glsl");
    ShaderUtility su2;
    osg::Program* programNonGeom = su2.createProgram("horizon3d_vert.glsl", "horizon3d_frag.glsl");

    int numHTiles = (int) ceil(float(fullSize.x()) / tileSize.x());
    int numVTiles = (int) ceil(float(fullSize.y()) / tileSize.y());

    for(int hIdx = 0; hIdx < numHTiles; ++hIdx)
    {
        for(int vIdx = 0; vIdx < numVTiles; ++vIdx)
        {
            const int hSize2 = hIdx < (numHTiles - 1) ?
                        (tileSize.x() + 1) : (fullSize.x() - tileSize.x() * (numHTiles - 1)) / compr;
            const int vSize2 = vIdx < (numVTiles - 1) ?
                        (tileSize.y() + 1) : ((fullSize.y() - tileSize.y() * (numVTiles - 1))) / compr;

            if(hSize2 == 1 || vSize2 == 1)
                continue;

            osg::ref_ptr<osg::Image> image = new osg::Image;
            image->allocateImage(vSize, hSize, 1, GL_LUMINANCE_ALPHA, GL_UNSIGNED_BYTE);
            image->setInternalTextureFormat(GL_LUMINANCE8_ALPHA8);

            unsigned short *ptr = reinterpret_cast<unsigned short*>(image->data());
            bool hasUndefs = false;
            for(int j = 0; j < vSize; ++j)
            {
                for(int i = 0; i < hSize; ++i)
                {
                    bool defined = false;
                    if((i < hSize2) && (j < vSize2))
                    {
                        int iGlobal = hIdx * tileSize.x() + i;
                        int jGlobal = vIdx * tileSize.y() + j;
                        const double val = depthVals.at(iGlobal * fullSize.y() + jGlobal);
                        if(!isUndef(val))
                        {
                            *ptr = (unsigned short) ((val - min) / diff * UCHAR_MAX);
                            defined = true;
                        }
                    }
                    if(!defined)
                    {
                        *ptr = 0xFF00;
                        hasUndefs = true;
                    }
                    ptr++;
                }
            }

            const int i1 = hIdx * tileSize.x();
            const int j1 = vIdx * tileSize.y();
            const osg::Vec2d start = coords[0] + iInc * i1 + jInc * j1;
            const osg::Vec3 shift(start.x(), start.y(), 0);

            osg::ref_ptr<osg::Texture2D> heightMap = new osg::Texture2D;
            heightMap->setImage(image.get());

            osg::ref_ptr<osg::Vec3Array> triangleNormals =
                    new osg::Vec3Array((hSize - 1) * (vSize - 1) * 2);

            for(int i = 0; i < hSize - 1; ++i)
                for(int j = 0; j < vSize - 1; ++j)
                {
                    if((i < hSize2 - 1) && (j < vSize2 - 1))
                    {
                        int iGlobal = hIdx * tileSize.x() + i;
                        int jGlobal = vIdx * tileSize.y() + j;

                        const int i00 = i*vSize+j;
                        const int i10 = (i+1)*vSize+j;
                        const int i01 = i*vSize+(j+1);
                        const int i11 = (i+1)*vSize+(j+1);

                        osg::Vec3 v00 = (*vertices)[i00] + shift;
                        osg::Vec3 v10 = (*vertices)[i10] + shift;
                        osg::Vec3 v01 = (*vertices)[i01] + shift;
                        osg::Vec3 v11 = (*vertices)[i11] + shift;

                        const int i00_Global = iGlobal * fullSize.y() + jGlobal;
                        const int i10_Global = (iGlobal+1) * fullSize.y() + jGlobal;
                        const int i01_Global = iGlobal * fullSize.y() + (jGlobal+1);
                        const int i11_Global = (iGlobal+1) * fullSize.y() + (jGlobal+1);

                        v00.z() = depthVals.at(i00_Global);
                        v10.z() = depthVals.at(i10_Global);
                        v01.z() = depthVals.at(i01_Global);
                        v11.z() = depthVals.at(i11_Global);

                        if(isUndef(v10.z()) || isUndef(v01.z()))
                            continue;

                        // calculate triangle normals
                        osg::Vec3 norm1 = (v01 - v00) ^ (v10 - v00);
                        norm1.normalize();
                        (*triangleNormals)[(i*(vSize-1)+j)*2] = norm1;

                        osg::Vec3 norm2 = (v10 - v11) ^ (v01 - v11);
                        norm2.normalize();
                        (*triangleNormals)[(i*(vSize-1)+j)*2+1] = norm2;
                    }
                }

            osg::Image *normalsImage = new osg::Image();
            normalsImage->allocateImage(hSize, vSize, 1, GL_RGB, GL_UNSIGNED_BYTE);
            GLubyte *normPtr = (GLubyte*)normalsImage->data();

            // The following loop calculates normals per vertex. Because
            // each vertex might be shared between many triangles(up to 6)
            // we find out which triangles this particular vertex is shared
            // and then compute the average of normals per triangle.
            osg::Vec3 triNormCache[6];
            for(int j = 0; j < vSize; ++j)
            {
                for(int i = 0; i < hSize; ++i)
                {
                    if((i < hSize2) && (j < vSize2))
                    {
                        int k = 0;

                        const int vSizeT = vSize - 1;

                        // 3
                        if((i < hSize - 1) && (j < vSize - 1))
                        {
                            triNormCache[k++] = (*triangleNormals)[(i*vSizeT+j)*2];
                        }

                        // 4, 5
                        if(i > 0 && j < vSize - 1)
                        {
                            triNormCache[k++] = (*triangleNormals)[((i-1)*vSizeT+j)*2];
                            triNormCache[k++] = (*triangleNormals)[((i-1)*vSizeT+j)*2+1];
                        }

                        // 1, 2
                        if(j > 0 && i < hSize - 1)
                        {
                            triNormCache[k++] = (*triangleNormals)[(i*vSizeT+j-1)*2];
                            triNormCache[k++] = (*triangleNormals)[(i*vSizeT+j-1)*2+1];
                        }

                        // 6
                        if(i > 0 && j > 0)
                        {
                            triNormCache[k++] = (*triangleNormals)[((i-1)*vSizeT+j-1)*2+1];
                        }

                        if(k > 0)
                        {
                            osg::Vec3 norm;
                            for(int l = 0; l < k; ++l)
                                norm += triNormCache[l];

                            norm.normalize();

                            // scale [-1;1] to [0..255]
                            #define C_255_OVER_2 127.5
                            *(normPtr + 0) = GLubyte((norm.x() + 1.0) * C_255_OVER_2);
                            *(normPtr + 1) = GLubyte((norm.y() + 1.0) * C_255_OVER_2);
                            *(normPtr + 2) = GLubyte((norm.z() + 1.0) * C_255_OVER_2);
                        }
                    }
                    normPtr += 3;
                }
            }

            osg::ref_ptr<osg::Texture2D> normals = new osg::Texture2D;
            normals->setImage(normalsImage);

            osg::Geode* geode = new osg::Geode;
            geode->addDrawable(geom.get());

            // apply vertex shader to shift geometry
            osg::StateSet *ss = geode->getOrCreateStateSet();
            ss->addUniform(new osg::Uniform("colour", osg::Vec4(0.0f, 0.0f, 1.0f, 1.0f)));
            ss->addUniform(new osg::Uniform("depthMin", float(min)));
            ss->addUniform(new osg::Uniform("depthDiff", float(diff)));
            ss->setTextureAttributeAndModes(1, heightMap.get());
            ss->setTextureAttributeAndModes(2, normals.get());
            ss->addUniform(new osg::Uniform("heightMap", 1));
            ss->addUniform(new osg::Uniform("normals", 2));

            const Palette p;
            const int sz = p.colorPoints().size();

            osg::Uniform *colourPoints = new osg::Uniform(osg::Uniform::FLOAT_VEC4, "colourPoints[0]", 20);
            for(int i = 0; i < sz; ++i)
            {
                osg::Vec3 colour = p.colorPoints().at(i).color;
                colourPoints->setElement(i, osg::Vec4(colour, 1.0));
            }
            ss->addUniform(colourPoints);
            osg::Uniform *colourPositions = new osg::Uniform(osg::Uniform::FLOAT, "colourPositions[0]", 20);
            for(int i = 0; i < sz; ++i)
            {
                colourPositions->setElement(i, p.colorPoints().at(i).pos);
            }
            ss->addUniform(colourPositions);
            ss->addUniform(new osg::Uniform("paletteSize", sz));

            ss->setAttributeAndModes(hasUndefs ? programGeom : programNonGeom, osg::StateAttribute::ON);

            osg::ref_ptr<Horizon3DTileNode2> transform = new Horizon3DTileNode2;
            transform->setMatrix(osg::Matrix::translate(osg::Vec3(start, 0)));
            transform->setNode(0, geode);

            // compute bounding box as our nodes don't have proper vertex information
            // and OSG can't deduce bounding sphere for culling
            osg::BoundingBox bb(osg::Vec3(start, min),
                                osg::Vec3(start + iInc * hSize2 + jInc * vSize2, max));
            transform->setBoundingSphere(bb);

            _nodes.push_back(transform);
        }
    }

    _needsUpdate = false;
}

}
