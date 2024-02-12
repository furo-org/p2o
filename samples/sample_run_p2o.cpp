/****************************************************************************
 * Copyright (C) 2010-2023 Kiyoshi Irie
 * Copyright (C) 2017-2023 Future Robotics Technology Center (fuRo),
 *                         Chiba Institute of Technology.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at https://mozilla.org/MPL/2.0/.
 ****************************************************************************/

#include <fstream>
#include <iomanip>
#include <chrono>
#include <getopt.h>
#include <p2o.h>

bool loadP2OFile( const char *p2ofile, p2o::Pose3DVec &nodes, std::vector<p2o::ErrorFunc3D*> &errorfuncs)
{
    using namespace p2o;
    std::ifstream is(p2ofile);
    if (!is) return false;

    nodes.clear();
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
            sstrm >> id >> x >> y >> z >> qx >> qy >> qz >> qw;
            nodes.push_back(Pose3D(x, y, z, Eigen::Quaterniond(qw, qx, qy, qz)));
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

void sample_g2o_3d(const std::string &filename, int max_iter, int min_iter, double robust_thre)
{
    std::string fname_in = filename + "_in.txt";
    std::string fname_out = filename + "_out.txt";
    std::ofstream ofs(fname_in);
    std::ofstream ofs2(fname_out);
    p2o::Pose3DVec nodes;
    p2o::Optimizer3D optimizer;
    optimizer.setVerbose(true);
    std::vector<p2o::ErrorFunc3D*> error_funcs;
    if (!loadP2OFile(filename.c_str(), nodes, error_funcs)) {
        std::cout << "can't open file: " << filename << std::endl;
        return;
    }
    optimizer.setRobustThreshold(robust_thre);
    auto t0 = std::chrono::high_resolution_clock::now();
    p2o::Pose3DVec result = optimizer.optimizePath(nodes, error_funcs, max_iter, min_iter);
    auto t1 = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration_cast< std::chrono::microseconds> (t1-t0);
    std::cout << filename << ": " << elapsed.count()*1e-6 << "s" << std::endl;
    ofs << std::fixed << std::setprecision(10);
    ofs2 << std::fixed << std::setprecision(10);
    for(int i=0; i<result.size(); i++) {
        Eigen::Quaterniond q1 = nodes[i].rv.toQuaternion();
        Eigen::Quaterniond q2 = result[i].rv.toQuaternion();
        ofs << nodes[i].x << " " << nodes[i].y << " " << nodes[i].z << " "
            << q1.x() << " " << q1.y() << " " << q1.z() << " " << q1.w() << std::endl;
        ofs2 << result[i].x << " " << result[i].y << " " << result[i].z << " "
            << q2.x() << " " << q2.y() << " " << q2.z() << " " << q2.w() << std::endl;
    }
    for (auto &err : error_funcs) {
        delete err;
    }
}

static void show_usage_and_exit()
{
    fprintf(stderr, "Usage: sample_run_p2o [-m max_iter] [-n min_iter] [-r robust_threshold] <p2ofile> \n");
    exit(EXIT_FAILURE);
}

int main(int argc, char *argv[])
{
    int max_iter = 300;
    int min_iter = 50;
    double robust_threshold = 0.01;

    int opt;
    while ((opt = getopt(argc, argv, "m:n:r:")) != -1) {
        switch (opt) {
            case 'm':
                max_iter = atoi(optarg);
                break;
            case 'n':
                min_iter = atoi(optarg);
                break;
            case 'r':
                robust_threshold = atof(optarg);
                break;
            default: /* '?' */
                show_usage_and_exit();
        }
    }

    if (optind >= argc) {
        show_usage_and_exit();
    }
    sample_g2o_3d(argv[optind], max_iter, min_iter, robust_threshold);

    return 0;
}

