/* osgGeo - A collection of geoscientific extensions to OpenSceneGraph.
Copyright 2011- dGB Beheer B.V.

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
#include <osgGeo/TiledOffScreenRenderer>

#include <osg/Viewport>
#include <osg/Switch>
#include <osgDB/WriteFile>
#include <osgViewer/View>
#include <osgViewer/Viewer>
#include <osgViewer/CompositeViewer>

using namespace osgGeo;

static unsigned char NOTRANSPARENCY = 255;

TiledOffScreenRenderer::TiledOffScreenRenderer(osgViewer::View* view,
					       osgViewer::CompositeViewer* viewer)
    :_view(view)
    ,_viewer(viewer)
    ,_tileWidth(640)
    ,_tileHeight(480)
    ,_width(1)
    ,_height(1)
    ,_collectorCamera(0)
    ,_transparency(NOTRANSPARENCY)
    ,_foregroundTransparency(NOTRANSPARENCY)
    ,_orientationCamera(view->getCamera())
    ,_cameraSwitch(dynamic_cast<osg::Switch*>(view->getSceneData()))
{
    if (!_cameraSwitch)
    {
	_cameraSwitch = new osg::Switch;
	_cameraSwitch->addChild(view->getSceneData());
	view->setSceneData(_cameraSwitch);
    }
}


TiledOffScreenRenderer::~TiledOffScreenRenderer()
{
    if ( _cameraSwitch->getNumChildren()<= 0)
	return;
    _cameraSwitch->removeChild(_collectorCamera);
    _cameraSwitch->setValue(0, true);
}


void TiledOffScreenRenderer::setOutputSize(int width, int height)
{
    _width = width>0 ? width : 1;
    _height = height>0 ? height : 1;

    const osg::Viewport* viewport = _orientationCamera->getViewport();

    const int widthTimes = (int) ceil(_width/viewport->width());
    const int heighTimes = (int) ceil(_height/viewport->height());

    _tileWidth  = (int)(_width/widthTimes);
    _tileHeight = (int)(_height/heighTimes);
}


bool TiledOffScreenRenderer::createOutput()
{
    createImageCollectorCamera();
    setupImageCollector();

    return doRender();
}


void TiledOffScreenRenderer::createImageCollectorCamera()
{
    _collectorCamera = new osg::Camera;
    _collectorCamera->setClearColor(_orientationCamera->getClearColor());
    _collectorCamera->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    _collectorCamera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
    _collectorCamera->setRenderOrder(osg::Camera::PRE_RENDER);
    _collectorCamera->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER);
    osg::ref_ptr<osg::Viewport> viewport = new osg::Viewport(0,0,_tileWidth,_tileHeight);
    _collectorCamera->setViewport(viewport);

     osg::Node* scenedata = _cameraSwitch->getChild(0);
    _collectorCamera->addChild(scenedata);
    _cameraSwitch->setValue(0, false);
    _cameraSwitch->addChild(_collectorCamera);
    _cameraSwitch->setValue(_cameraSwitch->getChildIndex(_collectorCamera), true);
}


void TiledOffScreenRenderer::setupImageCollector()
{
    _imageCollector = new OffscreenTileImageCollector;
    _imageCollector->setTileSize(_tileWidth, _tileHeight);
    _imageCollector->setFinalImageSize(_width, _height);
    _imageCollector->setCaptureCamera(_collectorCamera);

    osg::ref_ptr<osg::Image> finalImage = new osg::Image;
    finalImage->allocateImage(_width, _height , 1, GL_RGBA, GL_UNSIGNED_BYTE);
    _imageCollector->setFinalImage(finalImage.get());
    _imageCollector->setBackgroundTransparency(_transparency);
    _imageCollector->setForegroundTransparency(getForegroundTransparency());
}



class SwapBuffersCallback : public osg::GraphicsContext::SwapCallback
{
public:
    SwapBuffersCallback(){};
    void swapBuffersImplementation( osg::GraphicsContext* ) override   {}
};



bool TiledOffScreenRenderer::doRender()
{
    _imageCollector->setCameraOrientation(_orientationCamera->getViewMatrix(),
					 _orientationCamera->getProjectionMatrix());

    osg::GraphicsContext* gc = _orientationCamera->getGraphicsContext();

    osg::GraphicsContext::SwapCallback* oldSwapCallBack = gc->getSwapCallback();

    osg::ref_ptr<SwapBuffersCallback> swapcallback = new SwapBuffersCallback;
    gc->setSwapCallback(swapcallback);

    const osgViewer::ViewerBase::FrameScheme oldscheme = _viewer->getRunFrameScheme();
    _viewer->setRunFrameScheme(osgViewer::ViewerBase::CONTINUOUS);
    while ( !_imageCollector->done() )
    {
	_viewer->advance();

	_imageCollector->frame(_viewer->getFrameStamp());
	_viewer->renderingTraversals();
    }

    _collectorCamera->setRenderingCache(NULL);
    _collectorCamera->detach(osg::Camera::COLOR_BUFFER);

    gc->setSwapCallback(oldSwapCallBack);

    _viewer->setRunFrameScheme(oldscheme);

    if ( !_imageCollector->getFinalImage() )
	return false;

    return true;
}


void TiledOffScreenRenderer::writeOutputFile(const std::string& fileName)
{
  if( !_imageCollector )
	return;

    osg::ref_ptr<osg::Image> outputImage = _imageCollector->getFinalImage();
    if ( outputImage )
	osgDB::writeImageFile(*outputImage,fileName);
}


const osg::Image* TiledOffScreenRenderer::getOutput() const
{
    if( !_imageCollector  )
	return 0;

    return _imageCollector->getFinalImage();
}


void TiledOffScreenRenderer::setOutputBackgroundTransparency(unsigned char transparency)
{
    _transparency = transparency;
}


unsigned char TiledOffScreenRenderer::getForegroundTransparency() const
{
     return _foregroundTransparency;
}


void TiledOffScreenRenderer::setForegroundTransparency(unsigned char transparency)
{
    _foregroundTransparency = transparency;
}


TiledOffScreenRenderer::OffscreenTileImageCollector::OffscreenTileImageCollector()
    :_isRunning(true)
    ,_isFinishing(false)
    ,_lastBindingFrame(0)
    ,_currentRow(0)
    ,_currentColumn(0)
    ,_camera(0)
    ,_finalImage(0)
    ,_transparency(NOTRANSPARENCY)
    ,_foregroundTransparency(NOTRANSPARENCY)
{
}


void TiledOffScreenRenderer::OffscreenTileImageCollector::setCameraOrientation(
    const osg::Matrixd& view, const osg::Matrixd& proj)
{
    _currentViewMatrix = view;
    _currentProjectionMatrix = proj;
}


void TiledOffScreenRenderer::OffscreenTileImageCollector::frame(const osg::FrameStamp* fs)
{
    if ( _isFinishing )
    {
	if ( (fs->getFrameNumber()-_lastBindingFrame)>2 )
	{
	    recordImages();
	    _isFinishing = false;
	}
    }

    const int tileRows = (int)(_finalImageSize.y() / _tileSize.y());
    const int tileColumns = (int)(_finalImageSize.x() / _tileSize.x());

    if ( _isRunning )
    {
	// Every "copy-to-image" process seems to be finished in 2 frames.
	// So record them and dispatch camera to next tiles.
	if ( (fs->getFrameNumber()-_lastBindingFrame)>2 )
	{
	    // Record images and unref them to free memory
	    recordImages();
	    if ( _camera.valid() )
	    {
		bindCameraToImage(_camera.get(), _currentRow, _currentColumn);
		if ( _currentColumn<tileColumns-1 )
		{
		    _currentColumn++;
		}
		else
		{
		    if ( _currentRow<tileRows-1 )
		    {
			_currentRow++;
			_currentColumn = 0;
		    }
		    else
		    {
			_isRunning = false;
			_isFinishing = true;
		    }
		}
	    }
	    _lastBindingFrame = fs->getFrameNumber();
	}
    }
}


void TiledOffScreenRenderer::OffscreenTileImageCollector::bindCameraToImage(
    osg::Camera* camera, int row, int col )
{
    osg::ref_ptr<osg::Image> image = new osg::Image;
    image->allocateImage((int)_tileSize.x(), (int)_tileSize.y(), 1, GL_RGBA, GL_UNSIGNED_BYTE);
    _images[TilePosition(row,col)] = image.get();

    const int tileRows = (int)(_finalImageSize.y() / _tileSize.y());
    const int tileColumns = (int)(_finalImageSize.x() / _tileSize.x());
    // Calculate projection matrix offset of each tile
    osg::Matrix offsetMatrix =
	osg::Matrix::scale(tileColumns, tileRows, 1.0) *
	osg::Matrix::translate(tileColumns-1-2*col, tileRows-1-2*row, 0.0);
    camera->setViewMatrix(_currentViewMatrix);
    camera->setProjectionMatrix(_currentProjectionMatrix * offsetMatrix);

    // Reattach cameras and new allocated images
    camera->setRenderingCache( NULL );
    camera->detach(osg::Camera::COLOR_BUFFER);
    camera->attach(osg::Camera::COLOR_BUFFER, image.get(), 0, 0);

    if ( _transparency != NOTRANSPARENCY )
    {
	osg::ref_ptr<osg::Image> depthImage = new osg::Image;
	depthImage->allocateImage((int)_tileSize.x(), (int)_tileSize.y(), 1, GL_DEPTH_COMPONENT, GL_UNSIGNED_BYTE);
	_depthImages[TilePosition(row,col)] = depthImage.get();
	camera->detach(osg::Camera::DEPTH_BUFFER);
	camera->attach(osg::Camera::DEPTH_BUFFER, depthImage.get(), 0, 0);
    }

}


void TiledOffScreenRenderer::OffscreenTileImageCollector::recordImages()
{
    TileImages::iterator depthItr = _depthImages.begin();

    const bool changeTransparency = _transparency != NOTRANSPARENCY;
    for ( TileImages::iterator itr=_images.begin(); itr!=_images.end(); ++itr )
    {
	osg::Image* image = (itr->second).get();

	if ( _finalImage.valid() )
	{
	    unsigned int row = itr->first.first, col = itr->first.second;
	    for ( int t=0; t<image->t(); ++t )
	    {
		if ( changeTransparency )
		{
		    const osg::Image* depthImage = (depthItr->second).get();
		    setImageRowBgTransparency(t,image,depthImage);
		}

		unsigned char* source = image->data(0, t);
		unsigned char* target = _finalImage->data(
		    col*(int)_tileSize.x(), t + row*(int)_tileSize.y());
		memcpy(target, source, image->s() * 4 * sizeof(unsigned char));
	    }
	}

	if ( changeTransparency )
	    ++depthItr;
    }

    _images.clear();

    if ( changeTransparency )
        _depthImages.clear();

}


void TiledOffScreenRenderer::OffscreenTileImageCollector
    ::setImageRowBgTransparency(int row,osg::Image* image, const osg::Image* depthImage)
{
    unsigned char BACKGROUNDDEPTH = 255;

    if ( !image || !depthImage )
	return;

    for ( int s=0; s<image->s();++s)
    {
	const int imageOffset = (row * image->s() + s) * 4;
	const int depthImageOffset = row * depthImage->s() + s;
	unsigned char * imageData = image->data() + imageOffset;
	unsigned char const* depthImageData = depthImage->data() + depthImageOffset;

	if ( imageData && depthImageData )
	{
	    if ( *depthImageData == BACKGROUNDDEPTH )
		imageData[3] = _transparency;
	    else if ( _foregroundTransparency != NOTRANSPARENCY )
		imageData[3] = _foregroundTransparency;
	}
    }
}

