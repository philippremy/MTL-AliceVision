// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// Copyright (c) 2010 libmv contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <AVNumeric/numeric.hpp>
#include <AVNumeric/projection.hpp>
#include <AVMultiview/essential.hpp>
#include <AVMultiview/triangulation/triangulationDLT.hpp>

namespace aliceVision {

// HZ 9.6 page 257 (formula 9.12)
void essentialFromFundamental(const Mat3& F, const Mat3& K1, const Mat3& K2, Mat3* E) { *E = K2.transpose() * F * K1; }

// HZ 9.6 page 257 (formula 9.12)
// Or http://ai.stanford.edu/~birch/projective/node20.html
void fundamentalFromEssential(const Mat3& E, const Mat3& K1, const Mat3& K2, Mat3* F) { *F = K2.inverse().transpose() * E * K1.inverse(); }

void relativeCameraMotion(const Mat3& R1, const Vec3& t1, const Mat3& R2, const Vec3& t2, Mat3* R, Vec3* t)
{
    *R = R2 * R1.transpose();
    *t = t2 - (*R) * t1;
}

// HZ 9.6 page 257
void essentialFromRt(const Mat3& R1, const Vec3& t1, const Mat3& R2, const Vec3& t2, Mat3* E)
{
    Mat3 R;
    Vec3 t;
    relativeCameraMotion(R1, t1, R2, t2, &R, &t);
    Mat3 Tx = CrossProductMatrix(t);
    *E = Tx * R;
}

// HZ 9.7 page 259 (Result 9.19)
void motionFromEssential(const Mat3& E, std::vector<Mat3>* Rs, std::vector<Vec3>* ts)
{
    Eigen::JacobiSVD<Mat3> USV(E, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Mat3 U = USV.matrixU();
    Mat3 Vt = USV.matrixV().transpose();

    // Last column of U is undetermined since d = (a a 0).
    if (U.determinant() < 0)
    {
        U.col(2) *= -1;
    }
    // Last row of Vt is undetermined since d = (a a 0).
    if (Vt.determinant() < 0)
    {
        Vt.row(2) *= -1;
    }

    Mat3 W;
    W << 0, -1, 0, 1, 0, 0, 0, 0, 1;

    Mat3 U_W_Vt = U * W * Vt;
    Mat3 U_Wt_Vt = U * W.transpose() * Vt;

    Rs->resize(4);
    ts->resize(4);
    (*Rs)[0] = U_W_Vt;
    (*ts)[0] = U.col(2);
    (*Rs)[1] = U_W_Vt;
    (*ts)[1] = -U.col(2);
    (*Rs)[2] = U_Wt_Vt;
    (*ts)[2] = U.col(2);
    (*Rs)[3] = U_Wt_Vt;
    (*ts)[3] = -U.col(2);
}

// HZ 9.7 page 259 (Result 9.19)
void motionFromEssential(const Mat3& E, std::vector<Mat4> & Ts)
{
    Eigen::JacobiSVD<Mat3> USV(E, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Mat3 U = USV.matrixU();
    Mat3 Vt = USV.matrixV().transpose();

    // Last column of U is undetermined since d = (a a 0).
    if (U.determinant() < 0)
    {
        U.col(2) *= -1;
    }
    // Last row of Vt is undetermined since d = (a a 0).
    if (Vt.determinant() < 0)
    {
        Vt.row(2) *= -1;
    }

    Mat3 W;
    W << 0, -1, 0, 1, 0, 0, 0, 0, 1;

    Mat3 U_W_Vt = U * W * Vt;
    Mat3 U_Wt_Vt = U * W.transpose() * Vt;

    Ts.resize(4);
    Ts[0].setIdentity();
    Ts[0].block<3, 3>(0, 0) = U_W_Vt;
    Ts[0].block<3, 1>(0, 3) = U.col(2);

    Ts[1].setIdentity();
    Ts[1].block<3, 3>(0, 0) = U_W_Vt;
    Ts[1].block<3, 1>(0, 3) = -U.col(2);

    Ts[2].setIdentity();
    Ts[2].block<3, 3>(0, 0) = U_Wt_Vt;
    Ts[2].block<3, 1>(0, 3) = U.col(2);

    Ts[3].setIdentity();
    Ts[3].block<3, 3>(0, 0) = U_Wt_Vt;
    Ts[3].block<3, 1>(0, 3) = -U.col(2);
}

// HZ 9.6 pag 259 (9.6.3 Geometrical interpretation of the 4 solutions)
int motionFromEssentialChooseSolution(const std::vector<Mat3>& Rs,
                                      const std::vector<Vec3>& ts,
                                      const Mat3& K1,
                                      const Vec2& x1,
                                      const Mat3& K2,
                                      const Vec2& x2)
{
    assert(Rs.size() == 4);
    assert(ts.size() == 4);

    Mat34 P1, P2;
    // Set P1 = K1 [Id|0]
    Mat3 R1 = Mat3::Identity();
    Vec3 t1 = Vec3::Zero();
    P_from_KRt(K1, R1, t1, P1);

    for (int i = 0; i < 4; ++i)
    {
        const Mat3& R2 = Rs[i];
        const Vec3& t2 = ts[i];
        P_from_KRt(K2, R2, t2, P2);
        Vec3 X;
        multiview::TriangulateDLT(P1, x1, P2, x2, X);
        // Test if point is front to the two cameras (positive depth)
        if (Depth(R1, t1, X) > 0 && Depth(R2, t2, X) > 0)
        {
            return i;
        }
    }
    return -1;
}

bool motionFromEssentialAndCorrespondence(const Mat3& E, const Mat3& K1, const Vec2& x1, const Mat3& K2, const Vec2& x2, Mat3* R, Vec3* t)
{
    std::vector<Mat3> Rs;
    std::vector<Vec3> ts;
    motionFromEssential(E, &Rs, &ts);
    int solution = motionFromEssentialChooseSolution(Rs, ts, K1, x1, K2, x2);
    if (solution >= 0)
    {
        *R = Rs[solution];
        *t = ts[solution];
        return true;
    }
    else
    {
        return false;
    }
}

bool estimateTransformStructureFromEssential(Mat4 & T,
                                std::vector<Vec3>& structure,
                                std::vector<size_t>& newVecInliers,
                                const Mat3& E,
                                const std::vector<size_t>& vecInliers,
                                const camera::IntrinsicBase & cam1,
                                const camera::IntrinsicBase & cam2,
                                const std::vector<Vec2> & x1,
                                const std::vector<Vec2> & x2)
{
    // Find set of analytical solutions
    std::vector<Mat4> Ts;
    motionFromEssential(E, Ts);

    // Check each possible solution and keep the best one
    size_t bestCoundValid = 0;
    for (int it = 0; it < Ts.size(); it++)
    {
        const Mat4 T1 = Eigen::Matrix4d::Identity();
        const Mat4 & T2 = Ts[it];

        std::vector<Vec3> points;
        std::vector<size_t> updatedInliers;

        // Triangulate each point
        size_t countValid = 0;
        for (size_t k = 0; k < vecInliers.size(); ++k)
        {
            const Vec2& pt1 = x1[vecInliers[k]];
            const Vec2& pt2 = x2[vecInliers[k]];

            const Vec3 pt3d1 = cam1.toUnitSphere(cam1.removeDistortion(cam1.ima2cam(pt1)));
            const Vec3 pt3d2 = cam2.toUnitSphere(cam2.removeDistortion(cam2.ima2cam(pt2)));

            Vec3 X;
            multiview::TriangulateSphericalDLT(T1, pt3d1, T2, pt3d2, X);

            Vec2 ptValid1 = cam1.transformProject(T1, X.homogeneous(), true);
            Vec2 ptValid2 = cam2.transformProject(T2, X.homogeneous(), true);

            Eigen::Vector3d dirX1 = (T1 * X.homogeneous()).head(3).normalized();
            Eigen::Vector3d dirX2 = (T2 * X.homogeneous()).head(3).normalized();

            // Check that the points are logically
            if (!(dirX1.dot(pt3d1) > 0.0 && dirX2.dot(pt3d2)  > 0.0))
            {
                continue;
            }

            updatedInliers.push_back(vecInliers[k]);
            points.push_back(X);
            countValid++;
        }

        if (countValid > bestCoundValid)
        {
            bestCoundValid = countValid;
            structure = points;
            newVecInliers = updatedInliers;
            T = Ts[it];
        }
    }

    // Check result validity
    if (newVecInliers.size() < 10)
    {
        return false;
    }

    return true;
}

}  // namespace aliceVision
