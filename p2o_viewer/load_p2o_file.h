#pragma once
#include "p2o.h"

bool loadP2OFile( const char *p2ofile, p2o::Pose3DVec &nodes, std::vector<p2o::ErrorFunc3D*> &errorfuncs, std::vector<std::string> &cloud_files);
