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

$Id: LayeredTexture.cpp 298 2013-11-26 08:03:31Z jaap.glas@dgbes.com $

*/

#include <osgGeo/Callback>
#include <algorithm>


namespace osgGeo
{


CallbackObject::CallbackObject(const CallbackObject& cbo,const osg::CopyOp& co)
    : osg::Object( cbo, co )
{
    for ( unsigned int idx=0; idx<cbo._callbacks.size(); idx++ )
	addCallback( cbo._callbacks[idx] );
}


CallbackObject::~CallbackObject()
{
    for ( unsigned int idx=0; idx<_callbacks.size(); idx++ )
	_callbacks[idx]->unref();
}

void CallbackObject::addCallback(Callback* cb)
{
    if ( !cb )
	return;

    std::vector<Callback*>::iterator it = std::find( _callbacks.begin(), _callbacks.end(), cb );

    if ( it==_callbacks.end() )
    {
	cb->ref();
	_callbacks.push_back( cb );
    }
}


void CallbackObject::removeCallback(Callback* cb)
{
    std::vector<Callback*>::iterator it = std::find( _callbacks.begin(), _callbacks.end(), cb );

    if ( it!=_callbacks.end() )
    {
	_callbacks.erase( it );
	(*it)->unref();
    }
}


void CallbackObject::triggerRedrawRequest()
{
    std::vector<Callback*>::iterator it = _callbacks.begin();
    for ( ; it!=_callbacks.end(); it++ )
	(*it)->requestRedraw();
}


}; // namespace osgGeo
