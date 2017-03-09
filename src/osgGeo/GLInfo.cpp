/* osgGeo - A collection of geoscientific extensions to OpenSceneGraph.
Copyright 2012 dGB Beheer B.V.

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


/* This implementation of class osgGeo::GLInfo is using code from: 
 *
 * [1] wglinfo.c by Nate Robins, 1997 (for Windows)
 *
 * [2] glxinfo by Brian Paul, 1999-2006 (for Linux)
 *
 * Copyright (C) 1999-2006  Brian Paul   All Rights Reserved.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 */


#if defined(__win64__) || defined(__win32__)
# include <windows.h>
#endif

#if defined( __APPLE__ )
# include <OpenGL/gl.h>
#else
# include <GL/gl.h>
# include <GL/glext.h>
#endif

#include <osgGeo/GLInfo>


using namespace osgGeo;


GLInfo::GLInfo()
    /*: _glvendor( (const char*) glGetString(GL_VENDOR) )
    , _glrenderer( (const char*) glGetString(GL_RENDERER) )
    , _glversion( (const char*) glGetString(GL_VERSION) )
    , _glextensions( (const char*) glGetString(GL_EXTENSIONS) )*/
{
    updateLimits();
}


static osg::ref_ptr<GLInfo> inst;


bool GLInfo::initGL()
{
    //TODO
#if defined(__win64__) || defined(__win32__)
     return false;
#else 
    return true;
#endif
}


const osg::ref_ptr<GLInfo> GLInfo::get()
{
    if ( !inst ) // TODO: && glGetString(GL_VENDOR) when context is available
    {
        GLInfo* res = new GLInfo;
        inst = res;
    }

    return inst;
}
    
void GLInfo::updateLimits()
{
    struct token_name {
        GLenum _token;
        const char* _name;
    };
#if defined(GL_ARB_vertex_shader)
    const struct token_name vertex_limits[] = {
        { GL_MAX_VERTEX_UNIFORM_COMPONENTS_ARB, "GL_MAX_VERTEX_UNIFORM_COMPONENTS_ARB" },
        { GL_MAX_VARYING_FLOATS_ARB, "GL_MAX_VARYING_FLOATS_ARB" },
        { GL_MAX_VERTEX_ATTRIBS_ARB, "GL_MAX_VERTEX_ATTRIBS_ARB" },
        { GL_MAX_TEXTURE_IMAGE_UNITS_ARB, "GL_MAX_TEXTURE_IMAGE_UNITS_ARB" },
        { GL_MAX_VERTEX_TEXTURE_IMAGE_UNITS_ARB, "GL_MAX_VERTEX_TEXTURE_IMAGE_UNITS_ARB" },
        { GL_MAX_COMBINED_TEXTURE_IMAGE_UNITS_ARB, "GL_MAX_COMBINED_TEXTURE_IMAGE_UNITS_ARB" },
        { GL_MAX_TEXTURE_COORDS_ARB, "GL_MAX_TEXTURE_COORDS_ARB" },
        { (GLenum) 0, NULL }
    };
    
    for ( int idx = 0; vertex_limits[idx]._token; idx++) {
        GLint max[1];
        glGetIntegerv( (GLenum) vertex_limits[idx]._token, max);
        if (glGetError() == GL_NO_ERROR) {
            _limits.push_back( Limit( vertex_limits[idx]._token, vertex_limits[idx]._name, max[0] ));
        }
    }

#endif
    
#if defined(GL_ARB_fragment_shader)
    const struct token_name fragment_limits[] = {
        { GL_MAX_FRAGMENT_UNIFORM_COMPONENTS_ARB, "GL_MAX_FRAGMENT_UNIFORM_COMPONENTS_ARB" },
        { GL_MAX_TEXTURE_COORDS_ARB, "GL_MAX_TEXTURE_COORDS_ARB" },
        { GL_MAX_TEXTURE_IMAGE_UNITS_ARB, "GL_MAX_TEXTURE_IMAGE_UNITS_ARB" },
        { (GLenum) 0, NULL }
    };
    
    for ( int idx = 0; fragment_limits[idx]._token; idx++) {
        GLint max[1];
        glGetIntegerv( (GLenum) fragment_limits[idx]._token, max);
        if (glGetError() == GL_NO_ERROR) {
            _limits.push_back( Limit( fragment_limits[idx]._token, fragment_limits[idx]._name, max[0] ));
        }
    }

#endif
}


const char* GLInfo::glVendor() const
{ return _glvendor.c_str(); }

const char* GLInfo::glRenderer() const
{ return _glrenderer.c_str(); }

const char* GLInfo::glVersion() const
{ return _glversion.c_str(); }


bool GLInfo::isOK() const
{
    return !_glvendor.empty() && !_glversion.empty() && !_glrenderer.empty();
}
    
    
int GLInfo::getLimit( int intenum ) const
{
    for ( int idx=0; idx<_limits.size(); idx++ )
        if ( _limits[idx]._token==intenum )
            return _limits[idx]._value;
    
    return -1;
}


bool GLInfo::getExtension( const char* extnsnnm ) const
{
    const bool exists = _glextensions.find( extnsnnm ) != std::string::npos;
    return exists;
}


bool GLInfo::isPlatformSupported() const
{
#if defined(__win64__) || defined(__win32__) || defined(__APPLE__)
    return false;
#else
    return false;
#endif
}


bool GLInfo::isVertexProgramSupported() const
{
    return getExtension( "GL_ARB_vertex_shader" );
}


bool GLInfo::isShaderProgramSupported() const
{
    return getExtension( "GL_ARB_fragment_shader" );
}


bool GLInfo::isGeometryShader4Supported() const
{
    return getExtension( "GL_EXT_geometry_shader4" );
}
