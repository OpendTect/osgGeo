## This file should be placed in the root directory of your project.
## Then modify the CMakeLists.txt file in the root directory of your
## project to incorporate the testing dashboard.
## 
## $Id$
##
## # The following are required to submit to the CDash dashboard:
##   ENABLE_TESTING()
##   INCLUDE(CTest)

set(CTEST_PROJECT_NAME "osgGeo")
set(CTEST_NIGHTLY_START_TIME "01:00:00 GMT")

set(CTEST_DROP_METHOD "http")
set(CTEST_DROP_SITE "intranet")
set(CTEST_DROP_LOCATION "/cdash/submit.php?project=osgGeo")
set(CTEST_DROP_SITE_CDASH TRUE)
