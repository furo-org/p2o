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
#ifndef P2O2_H_
#define P2O2_H_

#include <vector>
#include <iostream>
#include <fstream>
#include <chrono>
#include <Eigen/Dense>
#include <Eigen/Sparse>

using p2o_float_t = double;

namespace p2o
{

using Vec3D = Eigen::Matrix<p2o_float_t,3,1>;
using Mat3D = Eigen::Matrix<p2o_float_t,3,3>;
using VecXD = Eigen::Matrix<p2o_float_t,Eigen::Dynamic,1>;
using MatXD = Eigen::Matrix<p2o_float_t,Eigen::Dynamic,Eigen::Dynamic>;

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

// === 3D utility classes and functions ===

using Vec6D = Eigen::Matrix<p2o_float_t,6,1>;
using Mat6D = Eigen::Matrix<p2o_float_t,6,6>;

struct RotVec
{
    p2o_float_t ax, ay, az;
    RotVec() : ax(0), ay(0), az(0) {}
    RotVec(const Vec3D &vec) : ax(vec(0)), ay(vec(1)), az(vec(2)) {}
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
    RotVec inverted() const { return RotVec(-ax, -ay, -az); }
    Eigen::Quaterniond toQuaternion() const {
        p2o_float_t v = sqrt(ax*ax + ay*ay + az*az);
        if (v < 1e-9) {
            return Eigen::Quaterniond(1, ax/2, ay/2, az/2);
        }
        return Eigen::Quaterniond(cos(v/2), sin(v/2)*ax/v, sin(v/2)*ay/v, sin(v/2)*az/v);
    }
    Mat3D toRotationMatrix() const {
        return toQuaternion().toRotationMatrix();
    }
    RotVec concat(const RotVec &rv) const {
        return RotVec(toQuaternion() * rv.toQuaternion());
    }
    void getEulerZYX(double *rotz, double *roty, double *rotx) const {
        Eigen::Matrix3d rot = toQuaternion().matrix();
        double sin_pitch = -rot(2,0);
        if (sin_pitch <= -1.0f) {
            *roty = -M_PI / 2;
        } else if (sin_pitch >= 1.0f) {
            *roty = M_PI / 2;
        } else {
            *roty = asin(sin_pitch);
        }
        if (sin_pitch > 0.99999f) {
            *rotz = 0.0f;
            *rotx = atan2(rot(0, 1), rot(0, 2));//need to verify
        } else {
            *rotz  = atan2(rot(1, 0), rot(0, 0));
            *rotx   = atan2(rot(2, 1), rot(2, 2));
        }
    }
};

inline std::ostream& operator<<(std::ostream& os, const RotVec &p)
{
    os << p.ax << ' ' << p.ay << ' ' << p.az << ' ';
    return os;
}

// === 3D pose-graph optimizer ===

struct Pose3D
{
    p2o_float_t bias_z = 0;
    p2o_float_t x, y, z;
    RotVec rv;
    Pose3D() : x(0), y(0), z(0) {}
    Pose3D(Vec3D in_t, const RotVec &in_rv) : x(in_t[0]), y(in_t[1]), z(in_t[2]), rv(in_rv) {}
    Pose3D(Eigen::Transform<p2o_float_t, 3, Eigen::Isometry> t) : x(t.translation()[0]), y(t.translation()[1]), z(t.translation()[2]), rv(Eigen::Quaterniond(t.rotation())) {}
    Pose3D(p2o_float_t in_x, p2o_float_t in_y, p2o_float_t in_z, const RotVec &in_rv) : x(in_x), y(in_y), z(in_z), rv(in_rv) {}
    Pose3D(p2o_float_t in_x, p2o_float_t in_y, p2o_float_t in_z, const RotVec &in_rv, p2o_float_t in_bias_z) : x(in_x), y(in_y), z(in_z), rv(in_rv), bias_z(in_bias_z){}
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
    Eigen::Transform<p2o_float_t, 3, Eigen::Isometry> toIsometry3() const {
        Eigen::Transform<p2o_float_t, 3, Eigen::Isometry> t = Eigen::Transform<p2o_float_t, 3, Eigen::Isometry>::Identity();
        t.translation() = pos();
        t.rotate(rv.toQuaternion());
        return t;
    }
    Pose3D oplus(const Pose3D &rel) const {
        Vec3D t = rv.toRotationMatrix()*rel.pos() + this->pos();
        RotVec rv2(rv.toQuaternion() * rel.rv.toQuaternion());
        return Pose3D(t[0], t[1], t[2], rv2, bias_z + rel.bias_z);
    }
    Pose3D ominus(const Pose3D &base) const {
        Vec3D t = base.rv.toRotationMatrix().transpose()*(this->pos() - base.pos());
        RotVec rv2(base.rv.toQuaternion().conjugate() * rv.toQuaternion());
        return Pose3D(t[0], t[1], t[2], rv2, bias_z - base.bias_z);
    }
};

using Pose3DVec = std::vector<Pose3D>;

struct ErrorFunc3D {
    bool debug_numerical_grad = false;
    Mat6D info;
    int ida = -1;
    int idb = -1;
    virtual Vec6D calcError(const Pose3D &pa, const Pose3D &pb, Mat6D &Ja, Mat6D &Jb) const = 0;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct ErrorFunc3D_SE3 : public ErrorFunc3D
{
    Mat3D Rx, Ry, Rz;
    Pose3D relpose;
    ErrorFunc3D_SE3();
    Vec6D errorFunc(const Pose3D &pa, const Pose3D &pb, const Pose3D &con) const;
    Vec6D calcError(const Pose3D &pa, const Pose3D &pb, Mat6D &Ja, Mat6D &Jb) const;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct ErrorFunc3D_Linear3D : public ErrorFunc3D
{
    Vec3D relpos;
    ErrorFunc3D_Linear3D() {}
    Vec6D errorFunc(const Pose3D &pa, const Pose3D &pb, const Vec3D &con) const;
    Vec6D calcError(const Pose3D &pa, const Pose3D &pb, Mat6D &Ja, Mat6D &Jb) const;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class Optimizer3D
{
    bool verbose = true;
    double lambda;
    double stop_thre;
    double robust_delta;
    typedef Eigen::Triplet<p2o_float_t> p2o_triplet;
    std::vector<p2o_triplet> tripletList;
public:
    Optimizer3D() : lambda(0), stop_thre(1e-3), robust_delta(std::numeric_limits<double>::max()) {
    }
    ~Optimizer3D() {}
    void setLambda(double val) { lambda = val; }
    void setStopEpsilon(double val) { stop_thre = val; }
    void setRobustThreshold(double val) { robust_delta = val; }
    void setVerbose(bool val) { verbose = val; }
    Pose3DVec optimizePath(const Pose3DVec &in_graphnodes, const std::vector<ErrorFunc3D*> &error_funcs, int max_steps, int min_steps = 3);
    double optimizePathOneStep(Pose3DVec &out_nodes, const Pose3DVec &graphnodes, const std::vector<ErrorFunc3D*> &error_funcs);
};

} //namespace p2o

#endif /* P2O2_H_ */
