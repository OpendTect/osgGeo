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


#include <GL/gl.h>
#include <GL/glx.h>
#include <osgGeo/GLInfo>

namespace osgGeo
{

GLInfo::GLInfo()
{}


bool GLInfo::get()
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
	_glvendor = (const char*)glGetString(GL_VENDOR);
	_glrenderer = (const char*)glGetString(GL_RENDERER);
	_glversion = (const char*)glGetString(GL_VERSION);
    }

    glXDestroyContext(dpy, ctx);
    XFree(visinfo);
    XDestroyWindow(dpy, win);
    XCloseDisplay(dpy);
    return true;
}


const char* GLInfo::glVendor() const
{ return _glvendor.c_str(); }

const char* GLInfo::glRenderer() const
{ return _glrenderer.c_str(); }

const char* GLInfo::glVersion() const
{ return _glversion.c_str(); }

} // namespace osgGeo
