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

#include <climits>

namespace osgGeo
{

namespace
{

static const char *shaderVertSource =
{
    "#version 330 compatibility\n"

    "uniform sampler2D heightMap;\n"
    "uniform float depthMin;\n"
    "uniform float depthDiff;\n"

    "varying float depthOut;\n"
    "out vec2 texCoordOut;\n"
    "out int undef;\n"

    "void main(void)\n"
    "{\n"
    "    vec4 pos = gl_Vertex;\n"
    "    texCoordOut = gl_MultiTexCoord0.st;\n"
    "    vec4 depthMask = texture2D(heightMap, texCoordOut);\n"
    "    float depthComp = depthMask.r;\n"
    "    depthOut = depthMin + depthComp * depthDiff;\n"
    "    pos.z = depthOut;\n"
    "    undef = (depthMask.a > 0.1) ? 1 : 0;\n"
    "    gl_Position = gl_ModelViewProjectionMatrix * pos;\n"
    "}\n"
};

static const char *microshaderGeomSource =
    "#version 330 compatibility\n"

    "layout( triangles ) in;\n"
    "layout( triangle_strip, max_vertices = 3 ) out;"
    "in int undef[];\n"
    "in vec2 texCoordOut[];\n"
    "out vec2 texCoordOutOut;\n"

    "void main()\n"
    "{\n"
    "    if(undef[0] != 1 && undef[1] != 1 && undef[2] != 1)\n"
    "    {\n"
    "        gl_Position = gl_in[0].gl_Position;\n"
    "        texCoordOutOut = texCoordOut[0];\n"
    "        EmitVertex();\n"

    "        gl_Position = gl_in[1].gl_Position;\n"
    "        texCoordOutOut = texCoordOut[1];\n"
    "        EmitVertex();\n"

    "        gl_Position = gl_in[2].gl_Position;\n"
    "        texCoordOutOut = texCoordOut[2];\n"
    "        EmitVertex();\n"
    "    }\n"
    "    EndPrimitive();\n"
    "}\n";

static const char *shaderFragSource =
{
    "#version 330 compatibility\n"

    "uniform vec4 colour;\n"
    "uniform sampler2D heightMap;\n"
    "varying vec2 texCoordOutOut;\n"

    "void main(void)\n"
    "{\n"
    "    vec4 pix = texture2D(heightMap, texCoordOutOut);\n"
    "    vec4 col = vec4(pix.g, pix.g, pix.g, 1.0);\n"
    "    gl_FragColor = pix;\n"
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

    double min = +999999;
    double max = -999999;
    for(int j = 0; j < fullSize.x(); ++j)
    {
        for(int i = 0; i < fullSize.y(); ++i)
        {
            const double val = depthVals.at(i * fullSize.y() + j);
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

    osg::Program* program = new osg::Program;
    program->setName( "microshader" );
    program->addShader( new osg::Shader( osg::Shader::VERTEX, shaderVertSource ) );
    program->addShader( new osg::Shader( osg::Shader::FRAGMENT, shaderFragSource ) );
    program->addShader( new osg::Shader( osg::Shader::GEOMETRY, microshaderGeomSource ) );

    int numHTiles = ceil(float(fullSize.x()) / tileSize.x());
    int numVTiles = ceil(float(fullSize.y()) / tileSize.y());

    std::cerr << "numtiles " << numHTiles << " " << numVTiles << std::endl;
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
                    if((i < hSize2) && (j < vSize2))
                    {
                        int iGlobal = hIdx * tileSize.x() + i;
                        int jGlobal = vIdx * tileSize.y() + j;
                        double val = depthVals.at(iGlobal * fullSize.y() + jGlobal);
                        unsigned short depthNorm = (val - min) / diff * UCHAR_MAX;
                        *ptr = depthNorm;
                    }
                    else {
                        *ptr = 0xFF00;
                        hasUndefs = true;
                    }
                    ptr++;
                }
            }

//            std::cerr << "hasUndefs " << hasUndefs << std::endl;
            osg::ref_ptr<osg::Texture2D> texture = new osg::Texture2D;
            texture->setImage(image.get());

            osg::Geode* geode = new osg::Geode;
            geode->addDrawable(geom.get());

            // apply vertex shader to shift geometry
            osg::StateSet *ss = geode->getOrCreateStateSet();
            ss->addUniform(new osg::Uniform("colour", osg::Vec4(0.0f, 0.0f, 1.0f, 1.0f)));
            ss->addUniform(new osg::Uniform("depthMin", float(min)));
            ss->addUniform(new osg::Uniform("depthDiff", float(diff)));
            ss->setTextureAttributeAndModes(1, texture.get());
            ss->addUniform( new osg::Uniform("heightMap", 1));
            ss->setAttributeAndModes(program, osg::StateAttribute::ON);

//            ss->setMode( GL_DEPTH_TEST, osg::StateAttribute::OFF );

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
