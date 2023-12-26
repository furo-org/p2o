/****************************************************************************
 * p2o: Petite Portable Pose-graph Optimizer v2
 * Copyright (C) 2010-2023 Kiyoshi Irie
 * Copyright (C) 2017-2023 Future Robotics Technology Center (fuRo),
 *                         Chiba Institute of Technology.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at https://mozilla.org/MPL/2.0/.
 ****************************************************************************/

#include <vector>
#include <iostream>
#include <chrono>
#include "p2o.h"

namespace p2o
{

static inline Mat3D rightJacobianRV(const RotVec &rv)
{
    Mat3D S;
    S <<      0, -rv.az,   rv.ay,
            rv.az,      0,  -rv.ax,
            -rv.ay,  rv.ax,       0;
    double n0 = rv.norm();
    if (n0 < 1e-7) return Mat3D::Identity();
    return Mat3D::Identity() + S/2 + (1/(n0*n0)-(1+cos(n0))/(2*n0*sin(n0)))*S*S;
}

static inline Eigen::Matrix<p2o_float_t,3,9> jacobianR(const Mat3D &R)
{
    double cost = ((R(0,0) + R(1,1) + R(2,2))-1)/2.;
    double t = acos(cost);
    double sint = sqrt(1-cost*cost);
    Eigen::Matrix<p2o_float_t,3,9> J;
    J <<
      0,    0,    0,    0,    0,    1,    0,   -1,    0,
            0,    0,   -1,    0,    0,    0,    1,    0,    0,
            0,    1,    0,   -1,    0,    0,    0,    0,    0;
    if (fabs(1-cost) < 1e-8) {
        J = J * 0.5;
    } else {
        Vec3D a;
        a << R(2,1)-R(1,2), R(0,2)-R(2,0), R(1,0)-R(0,1);
        a = a * (t*cost-sint)/(4*sint*sint*sint);
        double b = t/(2*sint);

        J = J * b;
        J.block<3,1>(0,0) = a;
        J.block<3,1>(0,4) = a;
        J.block<3,1>(0,8) = a;
    }
    return J;
}

static inline double robust_coeff(p2o_float_t squared_error, p2o_float_t delta)
{
    if (squared_error < 0) return 0;
    p2o_float_t sqre = sqrt(squared_error);
    if (sqre < delta) return 1; // no effect

    return delta / sqre; // linear
}

Vec6D ErrorFunc3D_Linear3D::errorFunc(const Pose3D &pa, const Pose3D &pb, const Vec3D &con) const {
    Vec6D ret = Vec6D::Zero();
    ret[0] = pb.x - pa.x - con[0];
    ret[1] = pb.y - pa.y - con[1];
    ret[2] = pb.z - pa.z - con[2];
    return ret;
}

Vec6D ErrorFunc3D_Linear3D::calcError(const Pose3D &pa, const Pose3D &pb, Mat6D &Ja, Mat6D &Jb) const {
    // pa is origin
    const Vec3D &con = this->relpos;
    Vec6D e0 = errorFunc(pa, pb, con);
    Ja = Jb = Mat6D::Zero();

    Ja.block<3,3>(0, 0) = -pa.rv.toRotationMatrix();
    Jb.block<3,3>(0, 0) = pb.rv.toRotationMatrix();

    if (debug_numerical_grad) {
        p2o_float_t eps = 1e-5;
        Mat6D JaN = Mat6D::Identity();
        Mat6D JbN = Mat6D::Identity();
        for(size_t i=0; i<6; ++i) {
            double d[6] = {0, 0, 0};
            d[i] = eps;
            RotVec dr(d[3], d[4], d[5]);
            Pose3D pa1 = pa.oplus(Pose3D(d[0], d[1], d[2], dr));
            Pose3D pb1 = pb.oplus(Pose3D(d[0], d[1], d[2], dr));
            JaN.block<6,1>(0,i) = (errorFunc(pa1, pb, con) - e0)/eps;
            JbN.block<6,1>(0,i) = (errorFunc(pa, pb1, con) - e0)/eps;
        }
        std::cout << "----- Linear Ja, JaN" << std::endl;
        std::cout << Ja << std::endl;
        std::cout << JaN << std::endl;

        std::cout << "----- Linear Jb, JbN" << std::endl;
        std::cout << Jb << std::endl;
        std::cout << JbN << std::endl;
        Ja = JaN;
        Jb = JbN;
    }
    return e0;
}

static double calc_convergence(const Pose3DVec &p0, const Pose3DVec &p1)
{
    Eigen::VectorXd v0(p0.size()*6);
    Eigen::VectorXd diffv(p0.size()*6);
    for(size_t i=0; i < p0.size(); ++i) {
        diffv.segment(i*6, 6) = p0[i].vec() - p1[i].vec();
        v0.segment(i*6, 6) = p0[i].vec();
        if (p1[i].vec().array().hasNaN()) {
            std::cout << "NaN: " << i << " " << p1[i].vec() << std::endl;
            abort();
        }
    }
    return diffv.norm()/v0.norm();
}

Pose3DVec Optimizer3D::optimizePath(const Pose3DVec &in_graphnodes, const std::vector<ErrorFunc3D*> &error_funcs, int max_steps, int min_steps)
{
    Pose3DVec graphnodes = in_graphnodes;
    Pose3DVec ret;
    p2o_float_t prevres = std::numeric_limits<p2o_float_t>::max();
    for(int i=1; i<=max_steps; i++) {
        auto t0 = std::chrono::high_resolution_clock::now();
        p2o_float_t res = optimizePathOneStep(ret, graphnodes, error_funcs );
        auto t1 = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast< std::chrono::microseconds> (t1-t0);
        double convergence = calc_convergence(graphnodes, ret);
        if (verbose) std::cout << "step " << i << ": res=" << res << ", convergence_score=" << convergence << " time=" << elapsed.count()*1e-6 << "s" << std::endl;
        graphnodes = ret;

        if (i > min_steps && prevres - res < stop_thre && convergence < stop_thre) {
            if (verbose) std::cout << "converged: " << prevres - res << " < " << stop_thre << std::endl;
            break;
        }
        prevres = res;
    }
    return graphnodes;
}

double Optimizer3D::optimizePathOneStep( Pose3DVec &out_nodes, const Pose3DVec &graphnodes, const std::vector<ErrorFunc3D*> &error_funcs)
{
    static const int dim = 6;
    double global_cost = 0;
    size_t indlist[] = {0, 1, 2, 3, 4, 5};
    size_t numnodes = graphnodes.size();
    Eigen::Matrix<p2o_float_t, Eigen::Dynamic, 1> bf = Eigen::Matrix<p2o_float_t, Eigen::Dynamic, 1>::Zero(numnodes * dim);

    tripletList.clear();
    tripletList.reserve(error_funcs.size() * dim * dim * 4);

    for(const ErrorFunc3D *err: error_funcs) {
        int ida = err->ida;
        int idb = err->idb;
        assert(0 <= ida && ida < graphnodes.size());
        assert(0 <= idb && idb < graphnodes.size());
        const Pose3D &pa = graphnodes[ida];
        const Pose3D &pb = graphnodes[idb];

        Mat6D Ja, Jb;
        Vec6D r = err->calcError(pa, pb, Ja, Jb);
        Mat6D info = err->info * robust_coeff(r.transpose() * err->info * r, robust_delta);
        global_cost += r.transpose() * info * r;

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
        tripletList.push_back(p2o_triplet(k, k, 1e6));
    }
    for(size_t i=0; i<dim*numnodes; ++i) {
        tripletList.push_back(p2o_triplet(i, i, lambda));
    }
    Eigen::SparseMatrix<p2o_float_t> mat(numnodes*dim, numnodes*dim);
    mat.setFromTriplets(tripletList.begin(), tripletList.end());

    Eigen::SimplicialLDLT<Eigen::SparseMatrix<p2o_float_t> > solver;
    solver.compute(mat);
    Eigen::Matrix<p2o_float_t, Eigen::Dynamic, 1> x = solver.solve(-bf);
    if (x.array().hasNaN()) {
        std::cout << x << std::endl;
        abort();
    }
    out_nodes.clear();
    for(unsigned int i=0;i<graphnodes.size();i++) {
        int u_i = i * dim;
        const Pose3D &p = graphnodes[i];
        out_nodes.push_back(p.oplus(Pose3D(x[u_i], x[u_i+1], x[u_i+2], x[u_i+3], x[u_i+4], x[u_i+5])));
    }
    return global_cost;
}

ErrorFunc3D_SE3::ErrorFunc3D_SE3() {
    Rx << 0, 0, 0,
          0, 0,-1,
          0, 1, 0;
    Ry << 0, 0, 1,
          0, 0, 0,
         -1, 0, 0;
    Rz << 0,-1, 0,
          1, 0, 0,
          0, 0, 0;
}

Vec6D ErrorFunc3D_SE3::errorFunc(const Pose3D &pa, const Pose3D &pb, const Pose3D &con) const {
    Pose3D ba = pb.ominus(pa);
    Pose3D err = ba.ominus(con);
    return err.vec();
}

Vec6D ErrorFunc3D_SE3::calcError(const Pose3D &pa, const Pose3D &pb, Mat6D &Ja, Mat6D &Jb) const {
    const Pose3D &con = this->relpose;
    Vec6D e0 = errorFunc(pa, pb, con);
    Ja = Jb = Mat6D::Identity();

    Mat3D RaInv = pa.rv.toRotationMatrix().transpose();
    Mat3D Rb = pb.rv.toRotationMatrix();
    Mat3D RcInv = con.rv.inverted().toRotationMatrix();
    RotVec rve(e0.tail<3>());
    Mat3D Re = rve.toRotationMatrix();

    Ja.block<3,3>(0, 0) = -RcInv;
    Jb.block<3,3>(0, 0) = RotVec(e0.tail<3>()).toRotationMatrix();

    // Rc' * {Rx' Ry' Rz'} * R0' * d
    Vec3D d;
    d << pb.x - pa.x, pb.y - pa.y, pb.z - pa.z;
    Ja.block<3,1>(0,3) = -RcInv * Rx * RaInv * d;
    Ja.block<3,1>(0,4) = -RcInv * Ry * RaInv * d;
    Ja.block<3,1>(0,5) = -RcInv * Rz * RaInv * d;

    // rotation part: Re = Rc' * Ra' * Rb;
    p2o_float_t buf[9 * 3];
    Eigen::Map<Eigen::Matrix<p2o_float_t, 9, 3>> dRedRa(buf);
    Mat3D dx = -RcInv * Rx * RaInv * Rb;
    Mat3D dy = -RcInv * Ry * RaInv * Rb;
    Mat3D dz = -RcInv * Rz * RaInv * Rb;
    memcpy(buf   , dx.data(), sizeof(p2o_float_t)*9);
    memcpy(buf+9 , dy.data(), sizeof(p2o_float_t)*9);
    memcpy(buf+18, dz.data(), sizeof(p2o_float_t)*9);

    Ja.block<3,3>(3,3) = jacobianR(Re) * dRedRa;
    Jb.block<3,3>(3,3) = rightJacobianRV(rve);
    if (debug_numerical_grad) {
        p2o_float_t eps = 1e-5;
        Mat6D JaN = Mat6D::Identity();
        Mat6D JbN = Mat6D::Identity();
        for(size_t i=0; i<6; ++i) {
            double d[6] = {0, 0, 0};
            d[i] = eps;
            RotVec dr(d[3], d[4], d[5]);
            Pose3D pa1 = pa.oplus(Pose3D(d[0], d[1], d[2], dr));
            Pose3D pb1 = pb.oplus(Pose3D(d[0], d[1], d[2], dr));
            JaN.block<6,1>(0,i) = (errorFunc(pa1, pb, con) - e0)/eps;
            JbN.block<6,1>(0,i) = (errorFunc(pa, pb1, con) - e0)/eps;
        }
        std::cout << "----- Ja, JaN" << std::endl;
        std::cout << Ja << std::endl;
        std::cout << JaN << std::endl;

        std::cout << "----- Jb, JbN" << std::endl;
        std::cout << Jb << std::endl;
        std::cout << JbN << std::endl;
        Ja = JaN;
        Jb = JbN;
    }
    return e0;
}

} //namespace p2o
