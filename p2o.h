/****************************************************************************
 * p2o: Petite Portable Pose-graph Optimizer
 * Copyright (C) 2010-2017 Kiyoshi Irie
 * Copyright (C) 2017 Future Robotics Technology Center (fuRo),
 *                    Chiba Institute of Technology.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at https://mozilla.org/MPL/2.0/.
 ****************************************************************************/
#ifndef P2O_H_
#define P2O_H_

#include <vector>
#include <iostream>
#include <chrono>
#include <Eigen/Dense>
#include <Eigen/Sparse>

using p2o_float_t = double;

namespace p2o
{

using Vec3D = Eigen::Matrix<p2o_float_t,3,1>;
using Mat3D = Eigen::Matrix<p2o_float_t,3,3>;

static inline p2o_float_t normalize_rad_pi_mpi(double rad)
{
    #ifndef M_PI
    static const double M_PI = 3.141592653589793238;
    #endif
    p2o_float_t val = fmod(rad, 2*M_PI);
    if (val > M_PI) {
        val -= 2*M_PI;
    } else if ( val < -M_PI) {
        val += 2*M_PI;
    }
    return val;
}

static Mat3D rotMat2D(p2o_float_t th)
{
    Mat3D rot;
    rot << cos(th), -sin(th), 0,
           sin(th),  cos(th), 0,
           0,              0, 1;
    return rot;
}

static double robust_coeff(p2o_float_t squared_error, p2o_float_t delta)
{
    p2o_float_t sqre = sqrt(squared_error);
    if (sqre < delta) return 1; // no effect

    return delta / sqre; // linear
}

struct Pose2D
{
    p2o_float_t x, y, th;
    Pose2D() : x(0), y(0), th(0) {}
    Pose2D(p2o_float_t in_x, p2o_float_t in_y, p2o_float_t in_t) : x(in_x), y(in_y), th(in_t) {}
    Vec3D vec() const {
        Vec3D v;
        v << x, y, th;
        return v;
    }
    Pose2D oplus(const Pose2D r) const {
        Vec3D v = rotMat2D(th) * r.vec() + vec();
        return Pose2D(v[0], v[1], v[2]);
    }
    Pose2D ominus(const Pose2D r) const {
        Vec3D diff;
        diff << x - r.x, y - r.y, th - r.th;
        Vec3D v = rotMat2D(-r.th) * diff;
        v[2] = normalize_rad_pi_mpi(th - r.th);
        return Pose2D(v[0], v[1], v[2]);
    }
};

using Pose2DVec = std::vector<Pose2D>;

struct Con2D
{
    int id1, id2;
    Pose2D t;
    Mat3D info;
    void setCovarianceMatrix( const Mat3D &mat )
    {
        info = mat.inverse();
    }
EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

using Con2DVec = std::vector<Con2D,Eigen::aligned_allocator<Con2D> >;

struct ErrorFunc2D
{
    static Vec3D error_func(const Pose2D &pa, const Pose2D &pb, const Pose2D &con) {
        Pose2D ba = pb.ominus(pa);
        Vec3D ret;
        ret << ba.x - con.x, ba.y - con.y, normalize_rad_pi_mpi(ba.th - con.th);
        return ret;
    }
    static Vec3D calcError(const Pose2D &pa, const Pose2D &pb, const Pose2D &con, Mat3D &Ja, Mat3D &Jb) {
        p2o_float_t eps = 1e-5;
        Vec3D e0 = error_func(pa, pb, con);
        p2o_float_t dx = pb.x - pa.x;
        p2o_float_t dy = pb.y - pa.y;
        p2o_float_t dxdt = -sin(pa.th) * dx + cos(pa.th) * dy;
        p2o_float_t dydt = -cos(pa.th) * dx - sin(pa.th) * dy;
        Ja << -cos(pa.th), -sin(pa.th), dxdt,
               sin(pa.th), -cos(pa.th), dydt,
                        0,           0,   -1;
        Jb <<  cos(pa.th), sin(pa.th),     0,
              -sin(pa.th), cos(pa.th),     0,
                        0,          0,     1;
        #if 0 // numerical gradient
        for(size_t i=0; i<3; ++i) {
            double d[3] = {0, 0, 0};
            d[i] = eps;
            Pose2D pa1(pa.x + d[0], pa.y + d[1], pa.th + d[2]);
            Pose2D pb1(pb.x + d[0], pb.y + d[1], pb.th + d[2]);
            Ja.block<3,1>(0,i) = (error_func(pa1, pb, con) - e0)/eps;
            Jb.block<3,1>(0,i) = (error_func(pa, pb1, con) - e0)/eps;
        }
        #endif
        return e0;
    }
};

class Optimizer2D
{
    bool verbose;
    double lambda;
    double stop_thre;
    double robust_delta;
    typedef Eigen::Triplet<p2o_float_t> p2o_triplet;
    std::vector<p2o_triplet> tripletList;
public:
    Optimizer2D() : verbose(false), lambda(1e-6), stop_thre(1e-3), robust_delta(std::numeric_limits<double>::max()) {}
    ~Optimizer2D() {}
    void setLambda(double val) { lambda = val; }
    void setStopEpsilon(double val) { stop_thre = val; }
    void setRobustThreshold(double val) { robust_delta = val; }
    void setVerbose(bool val) { verbose = val; }
    Pose2DVec optimizePath(const Pose2DVec &in_graphnodes, const Con2DVec &constraints, int max_steps, int min_steps = 3);
    double optimizePathOneStep(Pose2DVec &out_nodes, const Pose2DVec &graphnodes, const Con2DVec &constraints, double prev_res);
    double globalCost(const Pose2DVec &poses, const Con2DVec &constraints);
    bool loadFile(const char *g2ofile, Pose2DVec &nodes, Con2DVec &constraints);
};

Pose2DVec Optimizer2D::optimizePath(const Pose2DVec &in_graphnodes, const Con2DVec &constraints, int max_steps, int min_steps)
{
    Pose2DVec graphnodes = in_graphnodes;
    Pose2DVec ret;
    p2o_float_t prevres = std::numeric_limits<p2o_float_t>::max();
    for(int i=1; i<=max_steps; i++) {
        auto t0 = std::chrono::high_resolution_clock::now();
        p2o_float_t res = optimizePathOneStep(ret, graphnodes, constraints, prevres);
        auto t1 = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast< std::chrono::microseconds> (t1-t0);
        if (verbose) std::cout << "step " << i << ": " << res << " time: " << elapsed.count()*1e-6 << "s" << std::endl;
        graphnodes = ret;

        if (i > min_steps && prevres - res < stop_thre) {
            if (verbose) std::cout << "converged: " << prevres - res << " < " << stop_thre << std::endl;
            break;
        }
        prevres = res;
    }
    return graphnodes;
}

double Optimizer2D::globalCost(const Pose2DVec &poses, const Con2DVec &constraints)
{
    double sum = 0;
    for (const Con2D &c : constraints) {
        Vec3D diff = ErrorFunc2D::error_func(poses[c.id1], poses[c.id2], c.t);
        Mat3D info = c.info * robust_coeff(diff.transpose() * c.info * diff, robust_delta);
        sum += diff.transpose() * info * diff;
    }
    return sum;
}

double Optimizer2D::optimizePathOneStep( Pose2DVec &out_nodes, const Pose2DVec &graphnodes, const Con2DVec &constraints, double prev_res )
{
    static const int dim = 3;
    size_t indlist[] = {0, 1, 2};
    size_t numnodes = graphnodes.size();
    Eigen::Matrix<p2o_float_t, Eigen::Dynamic, 1> bf = Eigen::Matrix<p2o_float_t, Eigen::Dynamic, 1>::Zero(numnodes * dim);

    tripletList.clear();
    tripletList.reserve(constraints.size() * dim * dim * 4);

    for(const Con2D &con: constraints) {
        int ida = con.id1;
        int idb = con.id2;
        const Pose2D &pa = graphnodes[ida];
        const Pose2D &pb = graphnodes[idb];

        Mat3D Ja, Jb;
        Vec3D r = ErrorFunc2D::calcError(pa, pb, con.t, Ja, Jb);
        Mat3D info = con.info * robust_coeff(r.transpose() * con.info * r, robust_delta);

        Mat3D trJaInfo = Ja.transpose() * info;
        Mat3D trJaInfoJa = trJaInfo * Ja;
        Mat3D trJbInfo = Jb.transpose() * info;
        Mat3D trJbInfoJb = trJbInfo * Jb;
        Mat3D trJaInfoJb = trJaInfo * Jb;

        for(size_t k: indlist) {
            for(size_t m: indlist) {
                tripletList.push_back(p2o_triplet(ida*dim+k, ida*dim+m, trJaInfoJa(k,m)));
                tripletList.push_back(p2o_triplet(idb*dim+k, idb*dim+m, trJbInfoJb(k,m)));
                tripletList.push_back(p2o_triplet(ida*dim+k, idb*dim+m, trJaInfoJb(k,m)));
                tripletList.push_back(p2o_triplet(idb*dim+k, ida*dim+m, trJaInfoJb(m,k)));
            }
        }
        bf.segment(ida*dim, 3) += trJaInfo * r;
        bf.segment(idb*dim, 3) += trJbInfo * r;
    }
    for(size_t k: indlist) {
        tripletList.push_back(p2o_triplet(k, k, 1e10));
    }
    for(size_t i=0; i<dim*numnodes; ++i) {
        tripletList.push_back(p2o_triplet(i, i, lambda));
    }

    Eigen::SparseMatrix<p2o_float_t> mat(numnodes*dim, numnodes*dim);
    mat.setFromTriplets(tripletList.begin(), tripletList.end());

    //Eigen::SimplicialLLT<Eigen::SparseMatrix<p2o_float_t> > solver;
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<p2o_float_t> > solver;
    //Eigen::SparseLU<Eigen::SparseMatrix<p2o_float_t> > solver;
    //Eigen::ConjugateGradient<Eigen::SparseMatrix<p2o_float_t> > solver;

    solver.compute(mat);
    Eigen::Matrix<p2o_float_t, Eigen::Dynamic, 1> x = solver.solve(-bf);

    out_nodes.clear();
    for(unsigned int i=0;i<graphnodes.size();i++) {
        int u_i = i * dim;
        Pose2D pos(graphnodes[i].x + x[u_i], graphnodes[i].y + x[u_i + 1], graphnodes[i].th + x[u_i + 2]);
        out_nodes.push_back(pos);
    }
    return globalCost(out_nodes, constraints);
}

bool Optimizer2D::loadFile( const char *g2ofile, Pose2DVec &nodes, Con2DVec &constraints )
{
    Pose2DVec ret;
    std::ifstream is(g2ofile);
    if (!is) return false;

    nodes.clear();
    constraints.clear();

    while(is) {
        p2o_float_t x, y, th;
        char buf[1024];
        is.getline(buf,1024);
        std::istringstream sstrm(buf);
        std::string tag;
        sstrm >> tag;
        if (tag=="VERTEX_SE2") {
            int id;
            sstrm >> id >> x >> y >> th;
            nodes.push_back(Pose2D(x, y, th));
        } else if (tag=="EDGE_SE2") {
            Con2D con;
            p2o_float_t c1, c2, c3, c4, c5, c6;
            sstrm >> con.id1 >> con.id2 >> x >> y >> th >> c1 >> c2 >> c3 >> c4 >> c5 >> c6;
            con.t = Pose2D(x, y, th);
            con.info << c1, c2, c3,
                        c2, c4, c5,
                        c3, c5, c6;
            constraints.push_back(con);
        }
    }

    return true;
}

} //namespace p2o

#endif /* P2O_H_ */
