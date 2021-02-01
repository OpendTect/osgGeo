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


#if defined(WIN32) || defined(_WIN32)
# include <windows.h>
# include <GL/gl.h>
# include <GL/glext.h>
#else
#if defined( __APPLE__ )
# include <OpenGL/gl.h>
#else
# include <GL/glx.h>
#endif
#endif

#include <osgGeo/GLInfo>


using namespace osgGeo;
void initWinGL();
void initLuxGL();


GLInfo::GLInfo()
    : _glvendor( (const char*) glGetString(GL_VENDOR) )
    , _glrenderer( (const char*) glGetString(GL_RENDERER) )
    , _glversion( (const char*) glGetString(GL_VERSION) )
    , _glextensions( (const char*) glGetString(GL_EXTENSIONS) )
{
    updateLimits();
}


static osg::ref_ptr<GLInfo> inst;


void GLInfo::initGL()
{
#if defined(__APPLE__)
    return;
#elif defined(WIN32) || defined(_WIN32)
    initWinGL();
#elif defined(__linux__)
    initLuxGL();
#endif
}


const osg::ref_ptr<GLInfo> GLInfo::get()
{
#if defined(__APPLE__)  || defined(__mac__) //Hack to prepvent crash on MAC platform
    return 0;
#endif

    if ( !inst )
	initGL();

    if ( !inst && glGetString(GL_VENDOR) )
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


void GLInfo::getLineWidthRange( float& min, float& max ) const
{
    GLfloat rg[2];
    glGetFloatv( GL_ALIASED_LINE_WIDTH_RANGE, rg );
    min = rg[0];
    max = rg[1];
}


bool GLInfo::getExtension( const char* extnsnnm ) const
{
    const bool exists = _glextensions.find( extnsnnm ) != std::string::npos;
    return exists;
}


bool GLInfo::isPlatformSupported() const
{
#if defined(WIN32) || defined(_WIN32) || defined(__linux__)
    return true;
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

#if defined(WIN32) || defined(_WIN32)
LONG WINAPI WindowProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
    return (LONG)DefWindowProc(hWnd, uMsg, wParam, lParam);
}

void initWinGL()
{
    HDC	    hDC;
    HGLRC   hRC;
    HWND    hWnd;
    WNDCLASS    wc;
    PIXELFORMATDESCRIPTOR pfd;
    static HINSTANCE hInstance = 0;
    int         pf;

    if ( !hInstance )
    {
	hInstance = GetModuleHandle(NULL);
	wc.style         = CS_OWNDC;
	wc.lpfnWndProc   = (WNDPROC)WindowProc;
	wc.cbClsExtra    = 0;
	wc.cbWndExtra    = 0;
	wc.hInstance     = hInstance;
	wc.hIcon         = LoadIcon(NULL, IDI_WINLOGO);
	wc.hCursor       = LoadCursor(NULL, IDC_ARROW);
	wc.hbrBackground = NULL;
	wc.lpszMenuName  = NULL;
	wc.lpszClassName = "OpenGL";

	if ( !RegisterClass(&wc) )
	    return;
    }

    hWnd = CreateWindow("OpenGL", "OpenGL", WS_OVERLAPPEDWINDOW |
			WS_CLIPSIBLINGS | WS_CLIPCHILDREN,
			0, 0, 100, 100, NULL, NULL, hInstance, NULL);

    if ( hWnd == NULL )
	return ;

    hDC = GetDC( hWnd );
    memset( &pfd, 0, sizeof(pfd) );
    pfd.nSize        = sizeof( pfd );
    pfd.nVersion     = 1;
    pfd.dwFlags      = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL;
    pfd.iPixelType   = PFD_TYPE_RGBA;
    pfd.cColorBits   = 32;

    pf = ChoosePixelFormat( hDC, &pfd );

    if ( pf == 0 )
	return;

    if ( SetPixelFormat(hDC, pf, &pfd) == FALSE )
	return;

    DescribePixelFormat(hDC, pf, sizeof(PIXELFORMATDESCRIPTOR), &pfd);
    ReleaseDC(hWnd,hDC);
    hRC = wglCreateContext(hDC);
    wglMakeCurrent(hDC, hRC);
    DestroyWindow( hWnd );
}


#elif defined(__linux__)
void initLuxGL()
{
    Display* dpy = XOpenDisplay( NULL );
    int attribSingle[] = {
	GLX_RGBA,
	GLX_RED_SIZE, 1,
	GLX_GREEN_SIZE, 1,
	GLX_BLUE_SIZE, 1,
	None };
    int attribDouble[] = {
	GLX_RGBA,
	GLX_RED_SIZE, 1,
	GLX_GREEN_SIZE, 1,
	GLX_BLUE_SIZE, 1,
	GLX_DOUBLEBUFFER,
	None };

    int width = 100, height = 100;
    int scrnum = 0;
    Window root = RootWindow( dpy, scrnum );

    XVisualInfo* visinfo = glXChooseVisual( dpy, scrnum, attribSingle );
    if ( !visinfo )
    {
	visinfo = glXChooseVisual(dpy, scrnum, attribDouble);
	if ( !visinfo )
	    return;
    }

    XSetWindowAttributes attr;
    attr.background_pixel = 0;
    attr.border_pixel = 0;
    attr.colormap = XCreateColormap( dpy, root, visinfo->visual, AllocNone );
    attr.event_mask = StructureNotifyMask | ExposureMask;

    unsigned long mask = CWBackPixel | CWBorderPixel | CWColormap | CWEventMask;
    Window win = XCreateWindow( dpy, root, 0, 0, width, height,
				0, visinfo->depth, InputOutput,
				visinfo->visual, mask, &attr );

    Bool allowDirect = True;
    GLXContext ctx = glXCreateContext( dpy, visinfo, NULL, allowDirect );
    if ( !ctx )
    {
	XFree(visinfo);
	XDestroyWindow(dpy, win);
	return;
    }

    if ( !glXMakeCurrent(dpy,win,ctx) )
	return;

    glXDestroyContext(dpy, ctx);
    XFree(visinfo);
    XDestroyWindow(dpy, win);
}
#endif
