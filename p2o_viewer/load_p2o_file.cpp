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
#include "load_p2o_file.h"

bool loadP2OFile( const char *p2ofile, p2o::Pose3DVec &nodes, std::vector<p2o::ErrorFunc3D*> &errorfuncs, std::vector<std::string> &cloud_files)
{
    using namespace p2o;
    std::ifstream is(p2ofile);
    if (!is) return false;

    nodes.clear();
    errorfuncs.clear();
    cloud_files.clear();

    int id;
    double x, y, z, qx, qy, qz, qw;
    while(is){
        char buf[1024];
        is.getline(buf,1024);
        if (buf[0] == '#') continue;
        std::istringstream sstrm(buf);
        std::string tag;
        sstrm >> tag;
        if (tag=="VERTEX_SE3:QUAT"){
            std::string cloud_fname;
            sstrm >> id >> x >> y >> z >> qx >> qy >> qz >> qw >> cloud_fname;
            //if (cloud_fname.size() > 0) std::cout << id << " " << cloud_fname << std::endl;
            nodes.push_back(Pose3D(x, y, z, Eigen::Quaterniond(qw, qx, qy, qz)));
            cloud_files.push_back(cloud_fname);
        } else if (tag=="EDGE_SE3:QUAT"){
            ErrorFunc3D_SE3 *err = new ErrorFunc3D_SE3();
            err->info = Mat6D::Identity();
            int id1, id2;
            sstrm >> id1 >> id2 >> x >> y >> z >> qx >> qy >> qz >> qw;
            err->relpose = Pose3D(x, y, z, Eigen::Quaterniond(qw, qx, qy, qz));
            for(int i=0; i<6; i++) {
                for(int j=i; j<6; j++) {
                    double val;
                    sstrm >> val;
                    err->info(i,j) = val;
                    err->info(j,i) = val;
                }
            }
            //std::cout << err->info << std::endl;
            if (id1 > id2) {
                err->relpose = Pose3D().ominus(err->relpose);
                std::swap(id1, id2);
            }
            err->ida = id1;
            err->idb = id2;
            errorfuncs.push_back(err);

            //Pose3D diff = nodes[con.id2].ominus(nodes[con.id1]);
            //std::cout << "diff: " << diff.x << " " << diff.y << " " << diff.z << std::endl;
            //std::cout << "con: " << con.t.x << " " << con.t.y << " " << con.t.z << std::endl;
        } else if (tag=="EDGE_LIN3D"){
            int id1, id2;
            ErrorFunc3D_Linear3D *err = new ErrorFunc3D_Linear3D();
            sstrm >> id1 >> id2 >> x >> y >> z;
            err->info = Mat6D::Identity();
            err->info(3,3) = 0;
            err->info(4,4) = 0;
            err->info(5,5) = 0;
            err->ida = id1;
            err->idb = id2;
            err->relpos << x , y , z;
            errorfuncs.push_back(err);
        }
    }
    return true;
}
