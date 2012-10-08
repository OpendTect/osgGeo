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
// $Id$
//

#include "ShaderUtility.h"

#include <osg/Program>

#include <osgDB/FileUtils>
#include <osgDB/fstream>


namespace osgGeo
{

std::string ShaderUtility::_root;

void ShaderUtility::setRootPath(const char *rootPath)
{
    _root = rootPath;
}

osg::Program *ShaderUtility::createProgram(std::string vs, std::string fs, std::string gs)
{
    osg::Program* program = new osg::Program;

    program->addShader( new osg::Shader( osg::Shader::VERTEX, readFile(vs) ) );
    program->addShader( new osg::Shader( osg::Shader::FRAGMENT, readFile(fs) ) );
    if(!gs.empty())
        program->addShader( new osg::Shader( osg::Shader::GEOMETRY, readFile(gs) ) );

    return program;
}

namespace
{

bool readRawFile(const char* fName, std::string& s)
{
    std::string foundFile = osgDB::findDataFile(fName);
    if (foundFile.empty()) return false;

    osgDB::ifstream is;//(fName);
    is.open(foundFile.c_str());
    if (is.fail())
    {
        std::cerr << "Could not open " << fName << " for reading.\n";
        return false;
    }
    char ch = is.get();
    while (!is.eof())
    {
        s += ch;
        ch = is.get();
    }
    is.close();
    return true;
}

struct Lexer
{
    Lexer() : pos(0) {}

    size_t findNext(const std::string &needle)
    {
        size_t res = src.find(needle, pos);
        pos = res + 1;
        return res;
    }

    void reset()
    {
        pos = 0;
    }

    std::string src;
    size_t pos;
};

const char *tok_lParen = "(";
const char *tok_rParen = ")";
const char *tok_quote = "\"";
//const char *tok_at = "@";
const char *tok_include = "@include";
const std::string tok_if = "@if";
const std::string tok_else = "@else";
const std::string tok_endIf = "@endif";
//const char *tok_semicolon = ";";
}

std::string ShaderUtility::readFile(std::string fileName)
{
    Lexer lx;
    bool res = readRawFile(getFullPath(fileName).c_str(), lx.src);
    if(!res)
        return std::string();

    // look for include directives

    while(true)
    {
        bool foundAnything = false;

        // @include statement
        lx.reset();
        while(true)
        {
            const size_t tokStart = lx.findNext(tok_include);
            if(tokStart == std::string::npos)
                break;

            foundAnything = true;
            lx.findNext(tok_lParen);

            size_t posStart = lx.findNext(tok_quote) + 1;
            size_t posEnd = lx.findNext(tok_quote);

            std::string newFileName = lx.src.substr(posStart, posEnd - posStart);
            lx.findNext(tok_rParen);

            lx.src = lx.src.substr(0, tokStart) +
                     readFile(newFileName) +
                     lx.src.substr(lx.pos);
        }

        // @if @else @endif statements
        while(true)
        {
            const size_t tokStart = lx.findNext(tok_if);
            if(tokStart == std::string::npos)
                break;

            foundAnything = true;

            size_t posStart = lx.findNext(tok_quote) + 1;
            size_t posEnd = lx.findNext(tok_quote);

            std::string clause = lx.src.substr(posStart, posEnd - posStart);
            bool isTrue = _definition.count(clause);

            size_t startBlock = lx.findNext(tok_rParen) + 1;

            std::string ifBlock;
            std::string elseBlock;
            bool foundElse = false;

            while(true)
            {
                if(lx.src[lx.pos] == '@')
                {
                    if(!lx.src.compare(lx.pos, tok_else.size(), tok_else))
                    {
                        ifBlock = lx.src.substr(startBlock, lx.pos - startBlock);
                        lx.pos += tok_else.size();
                        startBlock = lx.pos;
                        foundElse = true;
                        continue;
                    }
                    else if(!lx.src.compare(lx.pos, tok_endIf.size(), tok_endIf))
                    {
                        std::string tmp = lx.src.substr(startBlock, lx.pos - startBlock);
                        if(foundElse)
                            elseBlock = tmp;
                        else
                            ifBlock = tmp;

                        lx.pos += tok_endIf.size();
                        break;
                    }
                }
                lx.pos++;
            }

            lx.src = lx.src.substr(0, tokStart) +
                     (isTrue ? ifBlock : elseBlock) +
                     lx.src.substr(lx.pos);
        }

        if(!foundAnything)
            break;
    }

    return lx.src;
}

std::string ShaderUtility::getFullPath(std::string fileName)
{
    return _root + "/" + fileName;
}

void ShaderUtility::addDefinition(std::string def)
{
    _definition.insert(def);
}

}
