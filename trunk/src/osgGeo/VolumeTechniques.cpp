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

$Id: TrackballManipulator.cpp 231 2013-04-16 12:35:57Z kristofer.tingdahl@dgbes.com $
*/

#include <osg/Texture1D>
#include <osg/Texture3D>
#include <osgVolume/VolumeTile>
#include <osgGeo/VolumeTechniques>


namespace osgGeo
{

FixedFunctionTechnique::FixedFunctionTechnique():
    osgVolume::FixedFunctionTechnique()
{
}


FixedFunctionTechnique::FixedFunctionTechnique(const FixedFunctionTechnique& fft,const osg::CopyOp& copyop):
    osgVolume::FixedFunctionTechnique(fft,copyop)
{
}


void FixedFunctionTechnique::init()
{
    osgVolume::FixedFunctionTechnique::init();

    // Overwrite default filter settings of osgVolume::FixedFunctionTechnique

    osg::StateAttribute* attr0 = _node->getStateSet()->getTextureAttribute( 0, osg::StateAttribute::TEXTURE );
    osg::Texture3D* texture3D  = dynamic_cast<osg::Texture3D*>( attr0 );

    if ( texture3D )
    {
	osg::Texture::FilterMode minFilter = _volumeTile->getLayer()->getMinFilter();
	osg::Texture::FilterMode magFilter = _volumeTile->getLayer()->getMagFilter();
	texture3D->setFilter(osg::Texture3D::MIN_FILTER,minFilter);
	texture3D->setFilter(osg::Texture3D::MAG_FILTER,magFilter);
    }
}


//=============================================================================


RayTracedTechnique::RayTracedTechnique():
    osgVolume::RayTracedTechnique()
{
}


RayTracedTechnique::RayTracedTechnique(const RayTracedTechnique& fft,const osg::CopyOp& copyop):
    osgVolume::RayTracedTechnique(fft,copyop)
{
}


void RayTracedTechnique::init()
{
    osgVolume::RayTracedTechnique::init();

    // Overwrite default filter settings of osgVolume::RayTracedTechnique

    osg::StateAttribute* attr0 = _transform->getChild(0)->getStateSet()->getTextureAttribute( 0, osg::StateAttribute::TEXTURE );
    osg::Texture3D* texture3D  = dynamic_cast<osg::Texture3D*>( attr0 );

    if ( texture3D )
    {

	osg::Texture::FilterMode minFilter = _volumeTile->getLayer()->getMinFilter();
	osg::Texture::FilterMode magFilter = _volumeTile->getLayer()->getMagFilter();
	texture3D->setFilter(osg::Texture3D::MIN_FILTER,minFilter);
	texture3D->setFilter(osg::Texture3D::MAG_FILTER,magFilter);

	texture3D->setWrap(osg::Texture3D::WRAP_R,osg::Texture3D::CLAMP_TO_EDGE);
	texture3D->setWrap(osg::Texture3D::WRAP_S,osg::Texture3D::CLAMP_TO_EDGE);
	texture3D->setWrap(osg::Texture3D::WRAP_T,osg::Texture3D::CLAMP_TO_EDGE);
    }

    osg::StateAttribute* attr1 = _transform->getChild(0)->getStateSet()->getTextureAttribute( 1, osg::StateAttribute::TEXTURE );
    osg::Texture1D* tf_texture = dynamic_cast<osg::Texture1D*>( attr1 );

    if ( tf_texture )
    {
	tf_texture->setFilter(osg::Texture::MIN_FILTER, osg::Texture::NEAREST);
	tf_texture->setFilter(osg::Texture::MAG_FILTER, osg::Texture::NEAREST);
    }
}


} // end namespace

