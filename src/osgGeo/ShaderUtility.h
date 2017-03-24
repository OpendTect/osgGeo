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
//

#ifndef SHADERUTILITY_H
#define SHADERUTILITY_H

#include <string>
#include <set>

#include <osgGeo/Common>

namespace osg { class Program; }

namespace osgGeo
{

class ShaderUtility
{
public:
    static void setRootPath(const char *rootPath);
    static std::string getFullPath(std::string fileName);

    void addDefinition(std::string def);

    osg::Program *createProgram(std::string vs, std::string fs, std::string gs = "");
    std::string readFile(std::string fileName);

private:
    static std::string _root;
    std::set<std::string> _definition;
};

}
#endif // SHADERUTILITY_H
