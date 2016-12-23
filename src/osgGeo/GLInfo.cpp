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
#elif defined( __APPLE__ )
#else
# include <GL/gl.h>
# include <GL/glx.h>
#endif

#include <osgGeo/GLInfo>

namespace osgGeo
{

GLInfo::GLInfo()
{}


bool GLInfo::isPlatformSupported() const
{
#if defined(__win64__) || defined(__win32__) || defined(__APPLE__)
    return false;
#else
    return true;
#endif
}


bool GLInfo::get()
{
#if defined(__win64__) || defined(__win32__) || defined(__APPLE__)
    // TODO
#else
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
	{
	    _errmsg = "Error: couldn't find RGB GLX visual";
	    return false;
	}
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
	_errmsg = "Error: glXCreateContext failed";
	XFree(visinfo);
	XDestroyWindow(dpy, win);
	return false;
    }

    if ( glXMakeCurrent(dpy,win,ctx) )
    {
	if ( (const char*) glGetString(GL_VENDOR) )
	    _glvendor = (const char*) glGetString(GL_VENDOR);
	if ( (const char*) glGetString(GL_RENDERER) )
	    _glrenderer = (const char*) glGetString(GL_RENDERER);
	if ( (const char*) (const char*) glGetString(GL_VERSION) )
	    _glversion = (const char*) glGetString(GL_VERSION);
    }

    glXDestroyContext(dpy, ctx);
    XFree(visinfo);
    XDestroyWindow(dpy, win);
    XCloseDisplay(dpy);
#endif
    return true;
}


const char* GLInfo::glVendor() const
{ return _glvendor.c_str(); }

const char* GLInfo::glRenderer() const
{ return _glrenderer.c_str(); }

const char* GLInfo::glVersion() const
{ return _glversion.c_str(); }

} // namespace osgGeo
