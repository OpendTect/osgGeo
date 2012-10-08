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

#include "Horizon3D2.h"

#include <iostream>
#include <cstdlib>

#include <osg/Geometry>
#include <osg/Geode>
#include <osg/Texture2D>
#include <osg/MatrixTransform>

#include <osgGeo/Vec2i>

namespace osgGeo
{

namespace
{

static const char *shaderVertSource =
{
    "#version 330 compatibility\n"

    "attribute vec3 position;\n"
    "in vec2 texCoord;\n"
    "uniform sampler2D heightMap;\n"
    "varying vec2 texCoordOut;\n"

    "void main(void)\n"
    "{\n"
    "    vec4 pos = gl_Vertex;\n"
    "    texCoordOut = gl_MultiTexCoord0.st;\n"
    "    float depth = texture2D(heightMap, texCoordOut).r;\n"
    "    pos.z = depth;\n"
    "    gl_Position = gl_ModelViewProjectionMatrix * pos;\n"
    "}\n"
};

static const char *shaderFragSource =
{
    "#version 330 compatibility\n"

    "uniform vec4 colour;\n"
    "uniform sampler2D heightMap;\n"
    "varying vec2 texCoordOut;\n"

    "void main(void)\n"
    "{\n"
    "    float depth = texture2D(heightMap, texCoordOut).r * 5.0;\n"
    "    vec4 col = vec4(depth, 0.0, 0.0, 1.0);\n"
    "    gl_FragColor = col;\n"
    "}\n"
};

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

Horizon3D2::Horizon3D2()
{
}

void Horizon3D2::setSize(const Vec2i& size)
{
    _size = size;
}

const Vec2i& Horizon3D2::getSize() const
{
    return _size;
}

void Horizon3D2::setDepthArray(osg::Array *arr)
{
    _array = arr;
    _needsUpdate = true;
    updateGeometry();
}

const osg::Array *Horizon3D2::getDepthArray() const
{
    return _array;
}

osg::Array *Horizon3D2::getDepthArray()
{
    return _array;
}

void Horizon3D2::setMaxDepth(float val)
{
    _maxDepth = val;
}

float Horizon3D2::getMaxDepth() const
{
    return _maxDepth;
}

void Horizon3D2::setCornerCoords(const std::vector<osg::Vec2d> &coords)
{
    _cornerCoords = coords;
}

std::vector<osg::Vec2d> Horizon3D2::getCornerCoords() const
{
    return _cornerCoords;
}

void Horizon3D2::updateGeometry()
{
    osgGeo::Vec2i fullSize = getSize();

    osg::DoubleArray &depthVals = *dynamic_cast<osg::DoubleArray*>(getDepthArray());

    std::vector<osg::Vec2d> coords = getCornerCoords();

    osg::Vec2d iInc = (coords[2] - coords[0]) / (fullSize.x() - 1);
    osg::Vec2d jInc = (coords[1] - coords[0]) / (fullSize.y() - 1);

    const osgGeo::Vec2i tileSize(256, 256);
    const int hSize = tileSize.x();
    const int vSize = tileSize.y();

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

    osg::Program* program = new osg::Program;
    program->setName( "microshader" );
    program->addShader( new osg::Shader( osg::Shader::VERTEX, shaderVertSource ) );
    program->addShader( new osg::Shader( osg::Shader::FRAGMENT, shaderFragSource ) );

    int numHTiles = ceil(float(fullSize.x()) / tileSize.x());
    int numVTiles = ceil(float(fullSize.y()) / tileSize.y());

    for(int hIdx = 0; hIdx < numHTiles; ++hIdx)
    {
        for(int vIdx = 0; vIdx < numVTiles; ++vIdx)
        {
            osg::ref_ptr<osg::Image> image = new osg::Image;
            image->allocateImage(vSize, hSize, 1, GL_RED, GL_FLOAT);
            image->setInternalTextureFormat(GL_LUMINANCE32F_ARB);

            float *ptr = reinterpret_cast<float*>(image->data());
            for(int j = 0; j < vSize; ++j)
            {
                for(int i = 0; i < hSize; ++i)
                {
                    int iGlobal = hIdx * tileSize.x() + i;
                    int jGlobal = vIdx * tileSize.y() + j;
                    *ptr = depthVals.at(iGlobal * fullSize.y() + jGlobal);
                    ptr++;
                }
            }

            osg::ref_ptr<osg::Texture2D> texture = new osg::Texture2D;
            texture->setImage(image.get());

            osg::Geode* geode = new osg::Geode;
            geode->addDrawable(geom.get());

            // apply vertex shader to shift geometry
            osg::StateSet *ss = geode->getOrCreateStateSet();
            ss->addUniform(new osg::Uniform("colour", osg::Vec4(0.0f, 0.0f, 1.0f, 1.0f)));
            ss->setTextureAttributeAndModes(1, texture.get());
            ss->addUniform( new osg::Uniform("heightMap", 1));
            ss->setAttributeAndModes( program, osg::StateAttribute::ON );

            const int i1 = hIdx * tileSize.x();
            const int j1 = vIdx * tileSize.y();
            const osg::Vec2d start = coords[0] + iInc * i1 + jInc * j1;

            osg::ref_ptr<osg::MatrixTransform> transform = new osg::MatrixTransform;
            transform->setMatrix(osg::Matrix::translate(start.x(), start.y(), 0));
            transform->addChild(geode);

            addChild(transform);
        }
    }
}

}
