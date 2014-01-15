#include <osgGeo/ScalarBar>
#include <osgText/Text>
#include <osg/Geometry>
#include <osg/Version>

#include <sstream>

using namespace osgGeo;

ScalarBar::~ScalarBar()
{
}

std::string ScalarBar::ScalarPrinter::printScalar(float scalar)
{
    std::stringstream ostr;
    ostr<<scalar;
    return ostr.str();
}

void ScalarBar::setNumColors(int numColors)
{
    _numColors = numColors;
    createDrawables();
}

int ScalarBar::getNumColors() const
{
    return _numColors;
}

void ScalarBar::setNumLabels(int numLabels)
{
    _numLabels = numLabels;
    createDrawables();
}

int ScalarBar::getNumLabels() const
{
    return _numLabels;
}

void ScalarBar::setScalarsToColors(osgSim::ScalarsToColors* stc)
{
    _stc = stc;
    createDrawables();
}

const osgSim::ScalarsToColors* ScalarBar::getScalarsToColors() const
{
    return _stc.get();
}

void ScalarBar::setTitle(const std::string& title)
{
    _title = title;
    createDrawables();
}

const std::string& ScalarBar::getTitle() const
{
    return _title;
}

void ScalarBar::setPosition(const osg::Vec3& pos)
{
    _position = pos;
    createDrawables();
}

void ScalarBar::setWidth(float width)
{
    _width = width;
    createDrawables();
}

void ScalarBar::setOrientation(ScalarBar::Orientation orientation)
{
    _orientation = orientation;
    createDrawables();
}

ScalarBar::Orientation ScalarBar::getOrientation() const
{
    return _orientation;
}

void ScalarBar::setAspectRatio(float aspectRatio)
{
    _aspectRatio = aspectRatio;
    createDrawables();
}

float ScalarBar::getAspectRatio() const
{
    return _aspectRatio;
}

void ScalarBar::setScalarPrinter(ScalarPrinter* sp)
{
    _sp = sp;
    createDrawables();
}

const ScalarBar::ScalarPrinter* ScalarBar::getScalarPrinter() const
{
    return _sp.get();
}


void ScalarBar::setTextProperties(const TextProperties& tp)
{
    _textProperties = tp;
    createDrawables();
}

const ScalarBar::TextProperties& ScalarBar::getTextProperties() const
{
    return _textProperties;
}


void ScalarBar::createDrawables()
{
    // Remove any existing Drawables
    _drawables.erase(_drawables.begin(), _drawables.end());

    if (_numColors==0) return;

    osg::Matrix matrix;
    if(_orientation==HORIZONTAL)
    {
        matrix = osg::Matrix::translate(_position);
    }
    else
    {
	// THIS IS WHERE I CHNAGED, THE ROTATION WAS AROUND Y AXIS WHICH WAS WRONG,
	// IT SHOULD BE AROUND Z-AXIS, AS Z-AXIS IS NORMAL TO THE SCREEN. - RANOJAY
        matrix = osg::Matrix::rotate(osg::DegreesToRadians(90.0f),0.0f,0.0f,1.0f) * osg::Matrix::translate(_position);
    }

    // 1. First the bar
    // =================
    osg::ref_ptr<osg::Geometry> bar = new osg::Geometry();

    // Create the bar - created in 'real' coordinate space the moment,
    // with xyz values reflecting those of the actual scalar values in play.
    // FIXME: Consider positioning at origin! Should be easy enough to do.

    // Vertices
    osg::ref_ptr<osg::Vec3Array> vs(new osg::Vec3Array);
    vs->reserve(4*_numColors);

    float incr = (_stc->getMax() - _stc->getMin()) / _numColors;
    float xincr = (_width) / _numColors;
    float arOffset = _width * _aspectRatio;

    int i;
    for(i=1; i<=_numColors; ++i)
    {
        vs->push_back(osg::Vec3((i-1) * xincr, 0.0f,     0.0f)*matrix);
        vs->push_back(osg::Vec3((i-1) * xincr, arOffset, 0.0f)*matrix);
        vs->push_back(osg::Vec3(i     * xincr, arOffset, 0.0f)*matrix);
        vs->push_back(osg::Vec3(i     * xincr, 0.0f,     0.0f)*matrix);
    }
    bar->setVertexArray(vs.get());

    // Colours
    osg::ref_ptr<osg::Vec4Array> cs(new osg::Vec4Array);
    cs->reserve(4*_numColors);
    const float halfIncr = incr*0.5;
    for(i=0; i<_numColors; ++i)
    {
        // We add half an increment to the color look-up to get the color
        // square in the middle of the 'block'.
        osg::Vec4 c = _stc->getColor(_stc->getMin() + (i*incr)  + halfIncr);
        cs->push_back(c);
        cs->push_back(c);
        cs->push_back(c);
        cs->push_back(c);
    }

#if OSG_VERSION_LESS_THAN(3,2,0)
    bar->setColorArray(cs.get());
    bar->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
#else
    bar->setColorArray(cs.get(), osg::Array::BIND_PER_VERTEX);
#endif

    // Normal
    osg::ref_ptr<osg::Vec3Array> ns(new osg::Vec3Array);
    ns->push_back(osg::Matrix::transform3x3(osg::Vec3(0.0f,0.0f,1.0f),matrix));

#if OSG_VERSION_LESS_THAN(3,2,0)
    bar->setNormalArray(ns.get());
    bar->setNormalBinding(osg::Geometry::BIND_OVERALL);
#else
    bar->setNormalArray(ns.get(), osg::Array::BIND_OVERALL);
#endif
    
    // The Quad strip that represents the bar
    bar->addPrimitiveSet(new osg::DrawArrays(GL_QUADS,0,vs->size()));

    addDrawable(bar.get());

    // 2. Then the text labels
    // =======================

    // Check the character size, if it's 0, estimate a good character size
    float characterSize = _textProperties._characterSize;
    if(characterSize == 0) characterSize = _width * 0.03f;

    //osgText::Font* font = osgText::readFontFile(_textProperties._fontFile.c_str());

    std::vector<osgText::Text*> texts(_numLabels);      // We'll need to collect pointers to these for later
    float labelIncr = (_numLabels>0) ? (_stc->getMax()-_stc->getMin())/(_numLabels-1) : 0.0f;
    float labelxIncr = (_numLabels>0) ? (_width)/(_numLabels-1) : 0.0f;
    float labely = arOffset + characterSize*0.3f;
    for(i=0; i<_numLabels; ++i)
    {
        osgText::Text* text = new osgText::Text;
        //text->setFont(font);
        text->setColor(_textProperties._color);
        text->setFontResolution(_textProperties._fontResolution.first,_textProperties._fontResolution.second);
        text->setCharacterSize(characterSize);
        text->setText(_sp->printScalar(_stc->getMin()+(i*labelIncr)));

	// I CHANGED HERE TOO, TO MAKE THE TEXT POSITIONS BETTER - RANOJAY
        text->setPosition(osg::Vec3((i*labelxIncr), (_orientation==HORIZONTAL) ? labely + labely/2 : -labely/2, 0.0f)*matrix);
        text->setAlignment(osgText::Text::CENTER_BASE_LINE);
	// THIS IS ALSO NOT NEEDED TO FIT IN MY REQUIREMENTS - RANOJAY
        //text->setAxisAlignment( (_orientation==HORIZONTAL) ? osgText::Text::XY_PLANE :  /*osgText::Text::XZ_PLANE*/ osgText::Text::XY_PLANE );
	text->setAlignment(osgText::Text::LEFT_BASE_LINE);
        addDrawable(text);

        texts[i] = text;
    }


    // 3. And finally the title
    // ========================

    if(_title != "")
    {
        osgText::Text* text = new osgText::Text;
        //text->setFont(font);
        text->setColor(_textProperties._color);
        text->setFontResolution(_textProperties._fontResolution.first,_textProperties._fontResolution.second);
        text->setCharacterSize(characterSize);
        text->setText(_title);

        float titleY = (_numLabels>0) ? labely + characterSize : labely;

        // Position the title at the middle of the bar above any labels.
        text->setPosition(osg::Vec3((_width/2.0f), titleY, 0.0f)*matrix);
        text->setAlignment(osgText::Text::CENTER_BASE_LINE);
        text->setAxisAlignment( (_orientation==HORIZONTAL) ? osgText::Text::XY_PLANE :  osgText::Text::XZ_PLANE );

        addDrawable(text);
    }


    // I ADDED THE REST OF THE CODE BELOW TO DRAW THE BOX AND THE TICKS - RANOJAY

    // 4. The rectangular border
    // ==========================

    osg::ref_ptr<osg::Vec3Array> rectvertices = new osg::Vec3Array;
   
    rectvertices->push_back( osg::Vec3(0.0f,0.0f,0.0f) * matrix  );
    rectvertices->push_back( osg::Vec3(0.0f,arOffset,0.0f) * matrix  );
    rectvertices->push_back( osg::Vec3(_width,arOffset,0.0f) * matrix  );
    rectvertices->push_back( osg::Vec3(_width,0.0f,0.0f) * matrix  );

    osg::ref_ptr<osg::Geometry> rectangle = new osg::Geometry();
    rectangle->addPrimitiveSet( new osg::DrawArrays(GL_LINE_LOOP,0,4) );
    rectangle->setVertexArray( rectvertices );
    osg::ref_ptr<osg::Vec4Array> rectcolor = new osg::Vec4Array;
    rectcolor->push_back( _textProperties._color );
        
#if OSG_VERSION_LESS_THAN(3,2,0)
    rectangle->setColorArray( rectcolor.get() );
    rectangle->setColorBinding(osg::Geometry::BIND_OVERALL );
#else
    rectangle->setColorArray( rectcolor.get(), osg::Array::BIND_OVERALL );
#endif

    setDrawable( 0, rectangle.get() ); //swap the drawables to avoid overlapping.
    addDrawable( bar );


    // 5. The sticks
    // ========================

    osg::ref_ptr<osg::Vec3Array> stickvertices = new osg::Vec3Array;
    osg::ref_ptr<osg::Vec4Array> stickcolor = new osg::Vec4Array;
    stickcolor->push_back( _textProperties._color );
    for(i=0; i<_numLabels; ++i)
    {
	osg::Vec3 p1(osg::Vec3((i*labelxIncr), 0.0f , 0.0f)*matrix);
	osg::Vec3 p2(osg::Vec3((i*labelxIncr), (_orientation==HORIZONTAL) ? labely + labely/2 : -labely/2, 0.0f)*matrix);
	stickvertices->push_back( p1 );
	stickvertices->push_back( p2 );
    }

    osg::ref_ptr<osg::Geometry> sticks = new osg::Geometry;
    sticks->setVertexArray( stickvertices );
        
#if OSG_VERSION_LESS_THAN(3,2,0)
    sticks->setColorArray( stickcolor.get() );
    sticks->setColorBinding( osg::Geometry::BIND_OVERALL );
#else
    sticks->setColorArray( stickcolor.get(), osg::Array::BIND_OVERALL );
#endif

    sticks->addPrimitiveSet( new osg::DrawArrays(GL_LINES,0,stickvertices->size()) );
    addDrawable( sticks );
}
