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
    Optimizer2D() : verbose(false), lambda(0), stop_thre(1e-3), robust_delta(std::numeric_limits<double>::max()) {}
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
        assert(0 <= ida && ida < graphnodes.size());
        assert(0 <= idb && idb < graphnodes.size());
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

using Vec6D = Eigen::Matrix<p2o_float_t,6,1>;
using Mat6D = Eigen::Matrix<p2o_float_t,6,6>;

struct RotVec
{
    p2o_float_t ax, ay, az;
    RotVec() : ax(0), ay(0), az(0) {}
    RotVec(p2o_float_t x, p2o_float_t y, p2o_float_t z) : ax(x), ay(y), az(z) {}
    RotVec(const Eigen::Quaterniond &in_q) {
        Eigen::Quaterniond q = in_q.normalized();
        if (q.w() < 0) q = Eigen::Quaterniond(-q.w(), -q.x(), -q.y(), -q.z());
        p2o_float_t x = q.x();
        p2o_float_t y = q.y();
        p2o_float_t z = q.z();
        double norm_im = sqrt(x*x+y*y+z*z);
        if (norm_im < 1e-7) {
            ax = 2*x;
            ay = 2*y;
            az = 2*z;
        } else {
            double th = 2 * atan2(norm_im, q.w());
            ax = x / norm_im * th;
            ay = y / norm_im * th;
            az = z / norm_im * th;
        }
    }
    p2o_float_t norm() const { return sqrt(ax*ax + ay*ay + az*az); }
    Eigen::Quaterniond toQuaternion() const {
        p2o_float_t v = sqrt(ax*ax + ay*ay + az*az);
        if (v < 1e-6) {
            return Eigen::Quaterniond(1, 0, 0, 0);
        }
        return Eigen::Quaterniond(cos(v/2), sin(v/2)*ax/v, sin(v/2)*ay/v, sin(v/2)*az/v);
    }
    Mat3D toRotationMatrix() const {
        return toQuaternion().toRotationMatrix();
    }
};

inline std::ostream& operator<<(std::ostream& os, const RotVec &p)
{
    os << p.ax << ' ' << p.ay << ' ' << p.az << ' ';
    return os;
}

struct Pose3D
{
    p2o_float_t x, y, z;
    RotVec rv;
    Pose3D() : x(0), y(0), z(0) {}
    Pose3D(p2o_float_t in_x, p2o_float_t in_y, p2o_float_t in_z, const RotVec &in_rv) : x(in_x), y(in_y), z(in_z), rv(in_rv) {}
    Pose3D(p2o_float_t in_x,  p2o_float_t in_y,  p2o_float_t in_z,
           p2o_float_t ax, p2o_float_t ay, p2o_float_t az) : x(in_x), y(in_y), z(in_z), rv(ax, ay, az) {}
    Vec6D vec() const {
        Vec6D v;
        v << x, y, z, rv.ax, rv.ay, rv.az;
        return v;
    }
    Vec3D pos() const {
        Vec3D v;
        v << x, y, z;
        return v;
    }
    Pose3D oplus(const Pose3D &rel) const {
        Vec3D t = rv.toRotationMatrix()*rel.pos() + this->pos();
        RotVec rv2(rv.toQuaternion() * rel.rv.toQuaternion());
        Eigen::Quaterniond q = rv.toQuaternion() * rel.rv.toQuaternion();
        return Pose3D(t[0], t[1], t[2], rv2);
    }
    Pose3D ominus(const Pose3D &base) const {
        Vec3D t = base.rv.toRotationMatrix().transpose()*(this->pos() - base.pos());
        RotVec rv2(base.rv.toQuaternion().conjugate() * rv.toQuaternion());
        return Pose3D(t[0], t[1], t[2], rv2);
    }
};

using Pose3DVec = std::vector<Pose3D>;

struct Con3D
{
    int id1, id2;
    Pose3D t;
    Mat6D info;
    void setCovarianceMatrix( const Mat6D &mat )
    {
        info = mat.inverse();
    }
EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

using Con3DVec = std::vector<Con3D,Eigen::aligned_allocator<Con3D> >;

struct ErrorFunc3D
{
    static Vec6D error_func(const Pose3D &pa, const Pose3D &pb, const Pose3D &con) {
        Pose3D ba = pb.ominus(pa);
        RotVec drv(con.rv.toQuaternion().conjugate() * ba.rv.toQuaternion());
        Vec6D ret;
        ret << ba.x - con.x, ba.y - con.y, ba.z - con.z, drv.ax, drv.ay, drv.az;
        return ret;
    }
    static Vec6D calcError(const Pose3D &pa, const Pose3D &pb, const Pose3D &con, Mat6D &Ja, Mat6D &Jb) {
        p2o_float_t eps = 1e-5;
        Vec6D e0 = error_func(pa, pb, con);
        #if 1 // numerical jacobian
        for(size_t i=0; i<6; ++i) {
            double d[6] = {0, 0, 0};
            d[i] = eps;
            Pose3D pa1(pa.x + d[0], pa.y + d[1], pa.z + d[2], pa.rv.ax + d[3], pa.rv.ay + d[4], pa.rv.az + d[5]);
            Pose3D pb1(pb.x + d[0], pb.y + d[1], pb.z + d[2], pb.rv.ax + d[3], pb.rv.ay + d[4], pb.rv.az + d[5]);
            Ja.block<6,1>(0,i) = (error_func(pa1, pb, con) - e0)/eps;
            Jb.block<6,1>(0,i) = (error_func(pa, pb1, con) - e0)/eps;
        }
        #endif
        return e0;
    }
};

class Optimizer3D
{
    bool verbose;
    double lambda;
    double stop_thre;
    double robust_delta;
    typedef Eigen::Triplet<p2o_float_t> p2o_triplet;
    std::vector<p2o_triplet> tripletList;
public:
    Optimizer3D() : verbose(false), lambda(0), stop_thre(1e-3), robust_delta(std::numeric_limits<double>::max()) {}
    ~Optimizer3D() {}
    void setLambda(double val) { lambda = val; }
    void setStopEpsilon(double val) { stop_thre = val; }
    void setRobustThreshold(double val) { robust_delta = val; }
    void setVerbose(bool val) { verbose = val; }
    Pose3DVec optimizePath(const Pose3DVec &in_graphnodes, const Con3DVec &constraints, int max_steps, int min_steps = 3);
    double optimizePathOneStep(Pose3DVec &out_nodes, const Pose3DVec &graphnodes, const Con3DVec &constraints, double prev_res);
    double globalCost(const Pose3DVec &poses, const Con3DVec &constraints);
    bool loadFile(const char *g2ofile, Pose3DVec &nodes, Con3DVec &constraints);
};

Pose3DVec Optimizer3D::optimizePath(const Pose3DVec &in_graphnodes, const Con3DVec &constraints, int max_steps, int min_steps)
{
    Pose3DVec graphnodes = in_graphnodes;
    Pose3DVec ret;
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

double Optimizer3D::globalCost(const Pose3DVec &poses, const Con3DVec &constraints)
{
    double sum = 0;
    for (const Con3D &c : constraints) {
        Vec6D diff = ErrorFunc3D::error_func(poses[c.id1], poses[c.id2], c.t);
        Mat6D info = c.info * robust_coeff(diff.transpose() * c.info * diff, robust_delta);
        sum += diff.transpose() * info * diff;
    }
    return sum;
}

double Optimizer3D::optimizePathOneStep( Pose3DVec &out_nodes, const Pose3DVec &graphnodes, const Con3DVec &constraints, double prev_res )
{
    static const int dim = 6;
    size_t indlist[] = {0, 1, 2, 3, 4, 5};
    size_t numnodes = graphnodes.size();
    Eigen::Matrix<p2o_float_t, Eigen::Dynamic, 1> bf = Eigen::Matrix<p2o_float_t, Eigen::Dynamic, 1>::Zero(numnodes * dim);

    tripletList.clear();
    tripletList.reserve(constraints.size() * dim * dim * 4);

    for(const Con3D &con: constraints) {
        int ida = con.id1;
        int idb = con.id2;
        assert(0 <= ida && ida < graphnodes.size());
        assert(0 <= idb && idb < graphnodes.size());
        const Pose3D &pa = graphnodes[ida];
        const Pose3D &pb = graphnodes[idb];

        Mat6D Ja, Jb;
        Vec6D r = ErrorFunc3D::calcError(pa, pb, con.t, Ja, Jb);
        Mat6D info = con.info * robust_coeff(r.transpose() * con.info * r, robust_delta);

        Mat6D trJaInfo = Ja.transpose() * info;
        Mat6D trJaInfoJa = trJaInfo * Ja;
        Mat6D trJbInfo = Jb.transpose() * info;
        Mat6D trJbInfoJb = trJbInfo * Jb;
        Mat6D trJaInfoJb = trJaInfo * Jb;

        for(size_t k: indlist) {
            for(size_t m: indlist) {
                tripletList.push_back(p2o_triplet(ida*dim+k, ida*dim+m, trJaInfoJa(k,m)));
                tripletList.push_back(p2o_triplet(idb*dim+k, idb*dim+m, trJbInfoJb(k,m)));
                tripletList.push_back(p2o_triplet(ida*dim+k, idb*dim+m, trJaInfoJb(k,m)));
                tripletList.push_back(p2o_triplet(idb*dim+k, ida*dim+m, trJaInfoJb(m,k)));
            }
        }
        bf.segment(ida*dim, dim) += trJaInfo * r;
        bf.segment(idb*dim, dim) += trJbInfo * r;
    }
    for(size_t k: indlist) {
        tripletList.push_back(p2o_triplet(k, k, 1e10));
    }
    for(size_t i=0; i<dim*numnodes; ++i) {
        tripletList.push_back(p2o_triplet(i, i, lambda));
    }

    Eigen::SparseMatrix<p2o_float_t> mat(numnodes*dim, numnodes*dim);
    mat.setFromTriplets(tripletList.begin(), tripletList.end());

    Eigen::SimplicialLDLT<Eigen::SparseMatrix<p2o_float_t> > solver;
    solver.compute(mat);
    Eigen::Matrix<p2o_float_t, Eigen::Dynamic, 1> x = solver.solve(-bf);

    out_nodes.clear();
    for(unsigned int i=0;i<graphnodes.size();i++) {
        int u_i = i * dim;
        const Pose3D &p = graphnodes[i];
        RotVec rv(p.rv.ax + x[u_i+3], p.rv.ay + x[u_i+4], p.rv.az + x[u_i+5]);
        out_nodes.push_back(Pose3D(p.x + x[u_i], p.y + x[u_i + 1], p.z + x[u_i + 2], rv));
    }
    return globalCost(out_nodes, constraints);
}

bool Optimizer3D::loadFile( const char *g2ofile, Pose3DVec &nodes, Con3DVec &constraints )
{
    std::ifstream is(g2ofile);
    if (!is) return false;

    nodes.clear();
    constraints.clear();
    int id;
    double x, y, z, qx, qy, qz, qw;
    while(is){
        char buf[1024];
        is.getline(buf,1024);
        std::istringstream sstrm(buf);
        std::string tag;
        sstrm >> tag;
        if (tag=="VERTEX_SE3:QUAT"){
            sstrm >> id >> x >> y >> z >> qx >> qy >> qz >> qw; 
            nodes.push_back(Pose3D(x, y, z, Eigen::Quaterniond(qw, qx, qy, qz)));
        } else if (tag=="EDGE_SE3:QUAT"){
            Con3D con;
            sstrm >> con.id1 >> con.id2 >> x >> y >> z >> qx >> qy >> qz >> qw;
            con.t = Pose3D(x, y, z, Eigen::Quaterniond(qw, qx, qy, qz));
            for(int i=0; i<6; i++) {
                for(int j=i; j<6; j++) {
                    double val;
                    sstrm >> val;
                    con.info(i,j) = val;
                    con.info(j,i) = val;
                }
            }
            if (con.id1 > con.id2) {
                con.t = Pose3D().ominus(con.t);
                int id0 = con.id1;
                con.id1 = con.id2;
                con.id2 = id0;
            }
            constraints.push_back(con);
        }
    }
    return true;
}

} //namespace p2o

#endif /* P2O_H_ */
