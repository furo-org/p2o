/****************************************************************************
 * p2o: Petite Portable Pose-graph Optimizer v2
 * Copyright (C) 2010-2024 Kiyoshi Irie
 * Copyright (C) 2017-2024 Future Robotics Technology Center (fuRo),
 *                         Chiba Institute of Technology.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at https://mozilla.org/MPL/2.0/.
 ****************************************************************************/
#pragma once
#include "p2o.h"

bool loadP2OFile( const char *p2ofile, p2o::Pose3DVec &nodes, std::vector<p2o::ErrorFunc3D*> &errorfuncs, std::vector<std::string> &cloud_files);
