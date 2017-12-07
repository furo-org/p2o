/****************************************************************************
 * Copyright (C) 2017 Future Robotics Technology Center (fuRo),
 *                    Chiba Institute of Technology.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at https://mozilla.org/MPL/2.0/.
 ****************************************************************************/

#include <fstream>
#include <chrono>
#include <p2o.h>

void sample_g2o(const std::string &filename, int max_iter, int min_iter, double robust_thre)
{
    std::string fname_in = filename + "_in.txt";
    std::string fname_out = filename + "_out.txt";
    std::ofstream ofs(fname_in);
    std::ofstream ofs2(fname_out);
    p2o::Pose2DVec nodes;
    p2o::Con2DVec con;
    p2o::Optimizer2D optimizer;
    optimizer.setLambda(1e-6);
    optimizer.setVerbose(true);
    if (!optimizer.loadFile(filename.c_str(), nodes, con)) {
        std::cout << "can't open file: " << filename << std::endl;
        return;
    }

    optimizer.setRobustThreshold(robust_thre);
    auto t0 = std::chrono::high_resolution_clock::now();
    p2o::Pose2DVec result = optimizer.optimizePath(nodes, con, max_iter, min_iter);
    auto t1 = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration_cast< std::chrono::microseconds> (t1-t0);
    std::cout << filename << ": " << elapsed.count()*1e-6 << "s" << std::endl;
    for(int i=1; i<result.size(); i++) {
        ofs << nodes[i].x << " " << nodes[i].y << " " << nodes[i].th << std::endl;
        ofs2 << result[i].x << " " << result[i].y << " " << result[i].th << std::endl;
    }
}

int main()
{
    sample_g2o("intel.g2o", 20, 3, 1);
    sample_g2o("manhattan3500.g2o", 20, 3, 1);

    // non-robust
    //sample_g2o("input_MITb_g2o.g2o", 100, 100, std::numeric_limits<double>::max());

    // robust
    sample_g2o("mit_killian.g2o", 20, 20, 1);
    return 0;
}

