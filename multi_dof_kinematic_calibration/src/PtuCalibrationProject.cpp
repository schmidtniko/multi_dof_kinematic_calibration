#include "multi_dof_kinematic_calibration/PtuCalibrationProject.h"

#include <Eigen/Geometry>

#include "visual_marker_mapping/ReconstructionIO.h"
#include "visual_marker_mapping/DetectionIO.h"
#include "visual_marker_mapping/EigenCVConversions.h"

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <ceres/version.h>

#include <iostream>
#include <fstream>

template <typename T> inline Eigen::Map<const Eigen::Matrix<T, 2, 1> > eMap2(const T* v)
{
    return Eigen::Map<const Eigen::Matrix<T, 2, 1> >(v);
}
template <typename T> inline Eigen::Map<const Eigen::Matrix<T, 3, 1> > eMap3(const T* v)
{
    return Eigen::Map<const Eigen::Matrix<T, 3, 1> >(v);
}
template <typename T> inline Eigen::Map<const Eigen::Matrix<T, 4, 1> > eMap4(const T* v)
{
    return Eigen::Map<const Eigen::Matrix<T, 4, 1> >(v);
}
template <typename T> inline Eigen::Map<const Eigen::Matrix<T, 6, 1> > eMap6(const T* v)
{
    return Eigen::Map<const Eigen::Matrix<T, 6, 1> >(v);
}
template <typename T> inline Eigen::Map<const Eigen::Quaternion<T> > eMapq(const T* v)
{
    return Eigen::Map<const Eigen::Quaternion<T> >(v);
}

template <typename T> inline Eigen::Map<Eigen::Matrix<T, 2, 1> > eMap2(T* v)
{
    return Eigen::Map<Eigen::Matrix<T, 2, 1> >(v);
}
template <typename T> inline Eigen::Map<Eigen::Matrix<T, 3, 1> > eMap3(T* v)
{
    return Eigen::Map<Eigen::Matrix<T, 3, 1> >(v);
}
template <typename T> inline Eigen::Map<Eigen::Matrix<T, 4, 1> > eMap4(T* v)
{
    return Eigen::Map<Eigen::Matrix<T, 4, 1> >(v);
}
template <typename T> inline Eigen::Map<Eigen::Matrix<T, 6, 1> > eMap6(T* v)
{
    return Eigen::Map<Eigen::Matrix<T, 6, 1> >(v);
}
template <typename T> inline Eigen::Map<Eigen::Quaternion<T> > eMapq(T* v)
{
    return Eigen::Map<Eigen::Quaternion<T> >(v);
}

template <typename T>
void poseAccum(const Eigen::Matrix<T, 4, 1>& q1, const Eigen::Matrix<T, 3, 1>& t1,
    const Eigen::Matrix<T, 4, 1>& q2, const Eigen::Matrix<T, 3, 1>& t2, Eigen::Matrix<T, 4, 1>& q3,
    Eigen::Matrix<T, 3, 1>& t3)
{
    ceres::QuaternionProduct(&q1(0), &q2(0), &q3(0));

    ceres::QuaternionRotatePoint(&q1(0), &t2(0), &t3(0));
    t3 += t1;
}

template <typename T>
Eigen::Matrix<T, 7, 1> cmakePose(const Eigen::Matrix<T, 3, 1>& xyz, const Eigen::Matrix<T, 4, 1>& q)
{
    Eigen::Matrix<T, 7, 1> ret;
    ret << xyz(0), xyz(1), xyz(2), q(0), q(1), q(2), q(3);
    return ret;
}

template <typename T>
Eigen::Matrix<T, 7, 1> cposeAdd(const Eigen::Matrix<T, 7, 1>& a, const Eigen::Matrix<T, 7, 1>& b)
{
    Eigen::Matrix<T, 7, 1> ret;
    ceres::QuaternionProduct(&a(3), &b(3), &ret(3));
    ceres::QuaternionRotatePoint(&a(3), &b(0), &ret(0));
    ret.template segment<3>(0) += a.template segment<3>(0);
    return ret;
}

template <typename T> Eigen::Matrix<T, 4, 1> cquatConjugate(const Eigen::Matrix<T, 4, 1>& q)
{
    Eigen::Matrix<T, 4, 1> ret;
    ret << q(0), -q(1), -q(2), -q(3);
    return ret;
}

template <typename T> Eigen::Matrix<T, 7, 1> cposeInv(const Eigen::Matrix<T, 7, 1>& a)
{
    Eigen::Matrix<T, 7, 1> ret;
    ret.template segment<4>(3) = cquatConjugate<T>(a.template segment<4>(3));
    ceres::QuaternionRotatePoint(&ret(3), &a(0), &ret(0));
    ret.template segment<3>(0) *= T(-1);
    return ret;
}


template <typename T>
Eigen::Matrix<T, 6, 1> cposeManifoldMinus(
    const Eigen::Matrix<T, 7, 1>& a, const Eigen::Matrix<T, 7, 1>& b)
{
    Eigen::Matrix<T, 6, 1> ret;
    ret.template segment<3>(0) = a.template segment<3>(0) - b.template segment<3>(0);

    Eigen::Matrix<T, 4, 1> bqinv = cquatConjugate<T>(b.template segment<4>(3));

    Eigen::Matrix<T, 4, 1> delta_q;
    ceres::QuaternionProduct(&a(3), &bqinv(0), &delta_q(0));

    ceres::QuaternionToAngleAxis(&delta_q(0), &ret(3));
    return ret;
}

struct PTUPoseError
{
    PTUPoseError(const Eigen::Matrix<double, 7, 1>& camPose_origin)
        : camPose_origin(camPose_origin)
    {
    }

    template <typename T>
    bool operator()(const T* const panpose_xyz, const T* const panpose_q, const T* const alpha,
        const T* const campose_xyz, const T* const campose_q, T* residuals) const
    {

        Eigen::Matrix<T, 4, 1> alpha_q;
        alpha_q << cos((*alpha) / T(2)), T(0), T(0), sin((*alpha) / T(2));
        Eigen::Matrix<T, 3, 1> alpha_xyz;
        alpha_xyz << T(0), T(0), T(0);

        Eigen::Matrix<T, 4, 1> tmp_q;
        Eigen::Matrix<T, 3, 1> tmp_xyz;
        poseAccum<T>(alpha_q, alpha_xyz, eMap4(campose_q), eMap3(campose_xyz), tmp_q, tmp_xyz);
        Eigen::Matrix<T, 4, 1> tmp2_q;
        Eigen::Matrix<T, 3, 1> tmp2_xyz;
        poseAccum<T>(eMap4(panpose_q), eMap3(panpose_xyz), tmp_q, tmp_xyz, tmp2_q, tmp2_xyz);

        eMap3(&residuals[0]) = tmp2_xyz - camPose_origin.segment<3>(0).cast<T>();

        Eigen::Matrix<T, 4, 1> camPose_origin_q_conj = camPose_origin.segment<4>(3).cast<T>();
        camPose_origin_q_conj.template segment<3>(1) *= T(-1);

        Eigen::Matrix<T, 4, 1> delta_q;
        ceres::QuaternionProduct(&tmp2_q(0), &camPose_origin_q_conj(0), &delta_q(0));

        ceres::QuaternionToAngleAxis(&delta_q(0), &residuals[3]);
        for (int i = 0; i < 3; i++)
            residuals[3 + i] *= T(1000);

        return true;
    }

    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction* Create(const Eigen::Matrix<double, 7, 1>& camPose_origin)
    {
        return (new ceres::AutoDiffCostFunction<PTUPoseError, 6, 3, 4, 1, 3, 4>(
            new PTUPoseError(camPose_origin)));
    }

    Eigen::Matrix<double, 7, 1> camPose_origin;
};

struct PTUPoseErrorTilt
{
    PTUPoseErrorTilt(const Eigen::Matrix<double, 7, 1>& camPose_origin)
        : camPose_origin(camPose_origin)
    {
    }

    template <typename T>
    bool operator()(const T* const panposeinv_xyz, const T* const panposeinv_q,
        const T* const alpha, const T* const tiltposeinv_xyz, const T* const tiltposeinv_q,
        const T* const beta, const T* const campose_xyz, const T* const campose_q,
        T* residuals) const
    {

#if 1
        Eigen::Matrix<T, 3, 1> pz = eMap3(panposeinv_xyz);
        if (pz(0) < T(0)) pz(0) *= T(-1);
        auto panPose = cposeInv(cmakePose<T>(pz, eMap4(panposeinv_q)));
        Eigen::Matrix<T, 3, 1> tz = eMap3(tiltposeinv_xyz);
        if (tz(0) < T(0)) tz(0) *= T(-1);
        auto tiltPose = cposeInv(cmakePose<T>(tz, eMap4(tiltposeinv_q)));
        auto camPose = cmakePose<T>(eMap3(campose_xyz), eMap4(campose_q));

        Eigen::Matrix<T, 7, 1> betaPose;
        betaPose << T(0), T(0), T(0), cos((*beta) / T(2)), T(0), T(0), sin((*beta) / T(2));
        // betaPose << T(0), T(0), T(0), T(1), T(0), T(0), T(0);
        Eigen::Matrix<T, 7, 1> alphaPose;
        alphaPose << T(0), T(0), T(0), cos((*alpha) / T(2)), T(0), T(0), sin((*alpha) / T(2));
        // alphaPose << T(0), T(0), T(0), T(1), T(0), T(0), T(0);

        auto camWorldPose = cposeAdd(
            panPose, cposeAdd(alphaPose, cposeAdd(tiltPose, cposeAdd(betaPose, camPose))));

        // auto deltaPose = cposeAdd(camWorldPose,cposeInv<T>(camPose_origin.cast<T>()));
        // Eigen::Matrix<T, 7, 1> zeroPose;
        // zeroPose << T(0),T(0),T(0),T(1),T(0),T(0),T(0);
        // eMap6(&residuals[0])=cposeManifoldMinus<T>(deltaPose, zeroPose);

        eMap6(&residuals[0]) = cposeManifoldMinus<T>(camWorldPose, camPose_origin.cast<T>());

// for (int i=0;i<3;i++)
// residuals[3+i]*=T(1000);
#else
        Eigen::Matrix<T, 4, 1> cam2Tilt_q;
        Eigen::Matrix<T, 3, 1> cam2Tilt_xyz;

        // Tilt
        Eigen::Matrix<T, 4, 1> beta_q;
        beta_q << cos((*beta) / T(2)), T(0), T(0), sin((*beta) / T(2));
        Eigen::Matrix<T, 3, 1> beta_xyz;
        beta_xyz << T(0), T(0), T(0);

        poseAccum<T>(
            beta_q, beta_xyz, eMap4(campose_q), eMap3(campose_xyz), cam2Tilt_q, cam2Tilt_xyz);


        Eigen::Matrix<T, 4, 1> cam2TiltRoot_q;
        Eigen::Matrix<T, 3, 1> cam2TiltRoot_xyz;
        poseAccum<T>(eMap4(tiltpose_q), eMap3(tiltpose_xyz), cam2Tilt_q, cam2Tilt_xyz,
            cam2TiltRoot_q, cam2TiltRoot_xyz);


        // Pan
        Eigen::Matrix<T, 4, 1> tmp_q;
        Eigen::Matrix<T, 3, 1> tmp_xyz;

        Eigen::Matrix<T, 4, 1> alpha_q;
        alpha_q << cos((*alpha) / T(2)), T(0), T(0), sin((*alpha) / T(2));
        Eigen::Matrix<T, 3, 1> alpha_xyz;
        alpha_xyz << T(0), T(0), T(0);

        poseAccum<T>(alpha_q, alpha_xyz, cam2TiltRoot_q, cam2TiltRoot_xyz, tmp_q, tmp_xyz);
        Eigen::Matrix<T, 4, 1> tmp2_q;
        Eigen::Matrix<T, 3, 1> tmp2_xyz;
        poseAccum<T>(eMap4(panpose_q), eMap3(panpose_xyz), tmp_q, tmp_xyz, tmp2_q, tmp2_xyz);

        eMap3(&residuals[0]) = tmp2_xyz - camPose_origin.segment<3>(0).cast<T>();

        Eigen::Matrix<T, 4, 1> camPose_origin_q_conj = camPose_origin.segment<4>(3).cast<T>();
        camPose_origin_q_conj.template segment<3>(1) *= T(-1);

        Eigen::Matrix<T, 4, 1> delta_q;
        ceres::QuaternionProduct(&tmp2_q(0), &camPose_origin_q_conj(0), &delta_q(0));

        ceres::QuaternionToAngleAxis(&delta_q(0), &residuals[3]);
        for (int i = 0; i < 3; i++)
            residuals[3 + i] *= T(1000);
#endif

        return true;
    }

    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction* Create(const Eigen::Matrix<double, 7, 1>& camPose_origin)
    {
        return (new ceres::AutoDiffCostFunction<PTUPoseErrorTilt, 6, 3, 4, 1, 3, 4, 1, 3, 4>(
            new PTUPoseErrorTilt(camPose_origin)));
    }

    Eigen::Matrix<double, 7, 1> camPose_origin;
};

struct OpenCVReprojectionError
{
    OpenCVReprojectionError(const Eigen::Vector2d& observation,
        const Eigen::Matrix<double, 5, 1>& d, const Eigen::Matrix3d& K)
        : observation(observation)
        , d(d)
        , K(K)
    {
    }

    template <typename T>
    bool operator()(const T* const camera_xyz, const T* const camera_q, const T* const point_xyz,
        T* residuals) const
    {

        // transform into camera coordinate system
        T tagCorC[3];
        ceres::UnitQuaternionRotatePoint(camera_q, point_xyz, tagCorC);

        tagCorC[0] += camera_xyz[0];
        tagCorC[1] += camera_xyz[1];
        tagCorC[2] += camera_xyz[2];

        //
        tagCorC[0] = tagCorC[0] / tagCorC[2];
        tagCorC[1] = tagCorC[1] / tagCorC[2];

        // radius^2
        T r2 = tagCorC[0] * tagCorC[0] + tagCorC[1] * tagCorC[1];


        T k1 = T(d(0, 0));
        T k2 = T(d(1, 0));
        T p1 = T(d(2, 0));
        T p2 = T(d(3, 0));
        T k3 = T(d(4, 0));

        // distort
        T xp = tagCorC[0];
        T yp = tagCorC[1];
        T xd = xp * (T(1) + r2 * (k1 + r2 * (k2 + r2 * k3))) + T(2) * p1 * xp * yp
            + p2 * (r2 + T(2) * xp * xp);
        T yd = yp * (T(1) + r2 * (k1 + r2 * (k2 + r2 * k3))) + T(2) * p2 * xp * yp
            + p1 * (r2 + T(2) * yp * yp);

        T fx = T(K(0, 0));
        T fy = T(K(1, 1));
        T cx = T(K(0, 2));
        T cy = T(K(1, 2));

        T predicted_x = T(fx) * xd + T(cx);
        T predicted_y = T(fy) * yd + T(cy);

        // The error is the difference between the predicted and observed position.
        residuals[0] = predicted_x - T(observation.x());
        residuals[1] = predicted_y - T(observation.y());

        if (print)
        {
            std::cout << "Predicted: " << predicted_x << " " << predicted_y
                      << " Observed: " << observation.x() << " " << observation.y() << std::endl;
        }

        return true;
    }

    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction* Create(const Eigen::Vector2d& observation,
        const Eigen::Matrix<double, 5, 1>& d, const Eigen::Matrix3d& K)
    {
        return (new ceres::AutoDiffCostFunction<OpenCVReprojectionError, 2, 3, 4,
            3>( //  residuals, camPose, tagPose
            new OpenCVReprojectionError(observation, d, K)));
    }

    Eigen::Vector2d observation;

    // distortion coefficients
    Eigen::Matrix<double, 5, 1> d;
    Eigen::Matrix3d K;
    bool print = false;
};


struct PTUPoseErrorTiltRepError
{
    PTUPoseErrorTiltRepError(const Eigen::Vector2d& observation, const Eigen::Vector3d& point_3d,
        const Eigen::Matrix<double, 5, 1>& d, const Eigen::Matrix3d& K)
        : repError(observation, d, K)
        , point_3d(point_3d)
    {
    }

    template <typename T>
    bool operator()(const T* const panposeinv_xyz, const T* const panposeinv_q,
        const T* const alpha, const T* const tiltposeinv_xyz, const T* const tiltposeinv_q,
        const T* const beta, const T* const campose_xyz, const T* const campose_q,
        T* residuals) const
    {
#if 0
        Eigen::Matrix<T,4,1> cam2Tilt_q;
        Eigen::Matrix<T,3,1> cam2Tilt_xyz;

        //Tilt
        Eigen::Matrix<T, 4, 1> beta_q;
        beta_q << cos((*beta)/T(2)), T(0), T(0), sin((*beta)/T(2));
        Eigen::Matrix<T,3,1> beta_xyz;
        beta_xyz << T(0), T(0), T(0);

        poseAccum<T>(beta_q, beta_xyz, eMap4(campose_q), eMap3(campose_xyz), cam2Tilt_q, cam2Tilt_xyz);

        
        Eigen::Matrix<T,4,1> cam2TiltRoot_q;
        Eigen::Matrix<T,3,1> cam2TiltRoot_xyz;
        poseAccum<T>(eMap4(tiltpose_q), eMap3(tiltpose_xyz), cam2Tilt_q, cam2Tilt_xyz, cam2TiltRoot_q, cam2TiltRoot_xyz);


        // Pan
        Eigen::Matrix<T,4,1> tmp_q;
        Eigen::Matrix<T,3,1> tmp_xyz;
        
        Eigen::Matrix<T,4,1> alpha_q;
        alpha_q << cos((*alpha)/T(2)), T(0), T(0), sin((*alpha)/T(2));
        Eigen::Matrix<T,3,1> alpha_xyz;
        alpha_xyz << T(0), T(0), T(0);

        poseAccum<T>(alpha_q, alpha_xyz, cam2TiltRoot_q, cam2TiltRoot_xyz, tmp_q, tmp_xyz);
        Eigen::Matrix<T,4,1> tmp2_q;
        Eigen::Matrix<T,3,1> tmp2_xyz;
        poseAccum<T>(eMap4(panpose_q), eMap3(panpose_xyz), tmp_q, tmp_xyz, tmp2_q, tmp2_xyz);

        // tmp2 invertieren
        Eigen::Matrix<T,4,1> tmp2_q_inv = tmp2_q;
        tmp2_q_inv.template segment<3>(1)*=T(-1.0);

        Eigen::Matrix<T,3,1> tmp2_xyz_inv;
        ceres::QuaternionRotatePoint(&tmp2_q_inv(0), &tmp2_xyz(0), &tmp2_xyz_inv(0));
        tmp2_xyz_inv=-tmp2_xyz_inv;

        Eigen::Matrix<T,3,1> point_3d_T = point_3d.cast<T>();

        return repError(&tmp2_xyz_inv(0), &tmp2_q_inv(0), &point_3d_T(0), residuals);

#else
        Eigen::Matrix<T, 3, 1> pz = eMap3(panposeinv_xyz);
        if (pz(0) < T(0)) pz(0) *= T(-1);
        auto panPose = cposeInv(cmakePose<T>(pz, eMap4(panposeinv_q)));
        Eigen::Matrix<T, 3, 1> tz = eMap3(tiltposeinv_xyz);
        if (tz(0) < T(0)) tz(0) *= T(-1);
        auto tiltPose = cposeInv(cmakePose<T>(tz, eMap4(tiltposeinv_q)));
        auto camPose = cmakePose<T>(eMap3(campose_xyz), eMap4(campose_q));

        Eigen::Matrix<T, 7, 1> betaPose;
        betaPose << T(0), T(0), T(0), cos((*beta) / T(2)), T(0), T(0), sin((*beta) / T(2));
        // betaPose << T(0), T(0), T(0), T(1), T(0), T(0), T(0);
        Eigen::Matrix<T, 7, 1> alphaPose;
        alphaPose << T(0), T(0), T(0), cos((*alpha) / T(2)), T(0), T(0), sin((*alpha) / T(2));
        // alphaPose << T(0), T(0), T(0), T(1), T(0), T(0), T(0);

        auto camToWorld = cposeAdd(
            panPose, cposeAdd(alphaPose, cposeAdd(tiltPose, cposeAdd(betaPose, camPose))));

        auto worldToCam = cposeInv(camToWorld);

        Eigen::Matrix<T, 3, 1> point_3d_T = point_3d.cast<T>();
        return repError(&worldToCam(0), &worldToCam(3), &point_3d_T(0), residuals);

#endif
    }

    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction* Create(const Eigen::Vector2d& observation,
        const Eigen::Vector3d& point_3d, const Eigen::Matrix<double, 5, 1>& d,
        const Eigen::Matrix3d& K)
    {
        return (
            new ceres::AutoDiffCostFunction<PTUPoseErrorTiltRepError, 2, 3, 4, 1, 3, 4, 1, 3, 4>(
                new PTUPoseErrorTiltRepError(observation, point_3d, d, K)));
    }

    OpenCVReprojectionError repError;
    Eigen::Vector3d point_3d;
};

struct GaussianPrior1D
{
    GaussianPrior1D(double mu, double sigma)
        : mu(mu)
        , sigma(sigma)
    {
    }

    template <typename T> bool operator()(const T* const value, T* residuals) const
    {
        // Gauss
        residuals[0] = (*value - T(mu)) / T(sigma);
        return true;
    }

    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction* Create(const double mu, const double sigma)
    {
        return (
            new ceres::AutoDiffCostFunction<GaussianPrior1D, 1, 1>(new GaussianPrior1D(mu, sigma)));
    }

    double mu;
    double sigma;
};

//-----------------------------------------------------------------------------
void PtuCalibrationProject::processFolder(const std::string& folder)
{
    // Read Reconstructions
    {
        visual_marker_mapping::CameraModel camModel;
        std::map<int, visual_marker_mapping::Camera> reconstructedCameras;
        visual_marker_mapping::parseReconstructions(
            folder + "/reconstruction.json", reconstructedTags, reconstructedCameras, camModel);
        std::cout << "Read reconstructions!" << std::endl;
    }

    // Read Pan Tilt Data
    {
        std::string filePath = folder + "/pantiltData.json";
        ptuData.importPanTiltImages(folder + "/pantiltData.json");
        std::cout << "Read PTU Data!" << std::endl;
    }

    {
        ptuDetectionResult = visual_marker_mapping::readDetectionResult(folder + "/cam1/detectedMarkers.json");
        std::cout << "Read PTU Image Detections!" << std::endl;
    }

    ///////////////////////////////////////////
    // Process

    int numCam1Images = 0;
    std::map<int, int> distinctTiltTicks;

    int tickNum = 0;

    for (size_t i = 0; i < ptuData.ptuImagePoses.size(); i++)
    {
        if (ptuData.ptuImagePoses[i].cameraId != 1) continue;

        distinctTiltTicks.emplace(ptuData.ptuImagePoses[i].tiltTicks, distinctTiltTicks.size());

        const visual_marker_mapping::CameraModel& camModel
            = ptuData.cameraModelById[ptuData.ptuImagePoses[i].cameraId];

        size_t detectedImageId = -1;
        for (size_t j = 0; j < ptuDetectionResult.images.size(); j++)
        {
            // TODO check if works
            if (strstr(ptuData.ptuImagePoses[i].imagePath.c_str(),
                    ptuDetectionResult.images[j].filePath.c_str()))
            {
                detectedImageId = j;
                break;
            }
        }
        assert(
            detectedImageId >= 0); // if this fails, there is no detection for a certain ptu image

        Eigen::Quaterniond q;
        Eigen::Vector3d t;
        computeRelativeCameraPoseFromImg(
            detectedImageId, camModel.getK(), camModel.distortionCoefficients, q, t);

        DebugVis dbg;
        dbg.cam.setQuat(q);
        dbg.cam.t = t;
        // debugVis.push_back(dbg);

        reconstructedPoses[i] = dbg.cam;


        // std::cout << ptuData.ptuImagePoses[i].tiltTicks << std::endl;

        numCam1Images++;
    }

    // ceres problem aufbauen
    {
        auto poseInverse = [](const Eigen::Matrix<double, 7, 1>& pose)
        {
            Eigen::Matrix<double, 7, 1> ret;
            Eigen::Quaterniond q;
            q.w() = pose(3);
            q.x() = pose(4);
            q.y() = pose(5);
            q.z() = pose(6);
            ret.segment<3>(0) = -q.toRotationMatrix().transpose() * pose.segment<3>(0);
            ret(3) = pose(3);
            ret.segment<3>(4) = -pose.segment<3>(4);
            return ret;
        };
        auto poseAdd = [](const Eigen::Matrix<double, 7, 1>& x,
            const Eigen::Matrix<double, 7, 1>& d) -> Eigen::Matrix<double, 7, 1>
        {
            Eigen::Quaterniond qx(x(3), x(4), x(5), x(6));
            Eigen::Quaterniond qd(d(3), d(4), d(5), d(6));
            Eigen::Quaterniond qres = qx * qd;
            Eigen::Matrix<double, 7, 1> ret;
            ret.segment<3>(0) = x.segment<3>(0) + qx.toRotationMatrix() * d.segment<3>(0);
            ret(3) = qres.w();
            ret(4) = qres.x();
            ret(5) = qres.y();
            ret(6) = qres.z();
            return ret;
        };


#define FULLPROBLEM
#define ANGLEOFFSET

        ceres::Problem markerBAProblem;

        ceres::Problem markerBAProblem_RepError;

        Eigen::Matrix<double, 7, 1> panPoseInv;
        panPoseInv << 0, 0, 0, 1, 0, 0, 0;

        std::vector<double> alphas(numCam1Images);
#ifdef FULLPROBLEM
        Eigen::Matrix<double, 7, 1> tiltPoseInv;
        tiltPoseInv << 0, 0, 0, 1, 0, 0, 0;

        std::vector<double> betas(numCam1Images);
        Eigen::Matrix<double, 7, 1> camPose;
#else
        std::vector<Eigen::Matrix<double, 7, 1> > camPoses;
        camPoses.resize(distinctTiltTicks.size());
#endif


        std::vector<int> yzconstant_params = { 1, 2 };
        auto yzconstant_parametrization = new ceres::SubsetParameterization(3, yzconstant_params);
        auto yzconstant_parametrization2 = new ceres::SubsetParameterization(3, yzconstant_params);

        std::vector<int> constant_params = { 2 };
        auto zconstant_parametrization = new ceres::SubsetParameterization(3, constant_params);
        auto zconstant_parametrization2 = new ceres::SubsetParameterization(3, constant_params);

        std::vector<int> yconstant_params = { 1, 2 };
        auto yconstant_parametrization
            = nullptr; // new ceres::SubsetParameterization(3, yconstant_params);
        auto yconstant_parametrization2
            = nullptr; // new ceres::SubsetParameterization(3, yconstant_params);


        auto quaternion_parameterization = new ceres::QuaternionParameterization;
        auto quaternion_parameterization2 = new ceres::QuaternionParameterization;

        markerBAProblem.AddParameterBlock(&panPoseInv(0), 3, yzconstant_parametrization);
        markerBAProblem.AddParameterBlock(&panPoseInv(3), 4, quaternion_parameterization);

        markerBAProblem_RepError.AddParameterBlock(&panPoseInv(0), 3, yzconstant_parametrization2);
        markerBAProblem_RepError.AddParameterBlock(&panPoseInv(3), 4, quaternion_parameterization2);

#ifdef FULLPROBLEM
        markerBAProblem.AddParameterBlock(&tiltPoseInv(0), 3, yzconstant_parametrization);
        markerBAProblem.AddParameterBlock(&tiltPoseInv(3), 4, quaternion_parameterization);

        markerBAProblem_RepError.AddParameterBlock(&tiltPoseInv(0), 3, yzconstant_parametrization2);
        markerBAProblem_RepError.AddParameterBlock(
            &tiltPoseInv(3), 4, quaternion_parameterization2);
#endif


        int curImage = 0;
        for (size_t i = 0; i < ptuData.ptuImagePoses.size(); i++)
        {
            if (ptuData.ptuImagePoses[i].cameraId != 1) continue;

            alphas[curImage] = ptuData.ptuImagePoses[i].panAngle;
            // std::cout << ptuData.ptuImagePoses[i].panAngle << std::endl;
            markerBAProblem.AddParameterBlock(&alphas[curImage], 1);
            markerBAProblem.SetParameterBlockConstant(&alphas[curImage]);

            markerBAProblem_RepError.AddParameterBlock(&alphas[curImage], 1);
#ifndef ANGLEOFFSET
            markerBAProblem_RepError.SetParameterBlockConstant(&alphas[curImage]);
#else
            auto* alphaPrior = GaussianPrior1D::Create(alphas[curImage], 0.05 / 180.0 * M_PI);
            markerBAProblem_RepError.AddResidualBlock(alphaPrior, nullptr, &alphas[curImage]);
#endif
#ifdef FULLPROBLEM
            betas[curImage] = ptuData.ptuImagePoses[i].tiltAngle;
            markerBAProblem.AddParameterBlock(&betas[curImage], 1);
            markerBAProblem.SetParameterBlockConstant(&betas[curImage]);

            markerBAProblem_RepError.AddParameterBlock(&betas[curImage], 1);
#ifndef ANGLEOFFSET
            markerBAProblem_RepError.SetParameterBlockConstant(&betas[curImage]);
#else
            auto* betaPrior = GaussianPrior1D::Create(betas[curImage], 0.05 / 180.0 * M_PI);
            markerBAProblem_RepError.AddResidualBlock(betaPrior, nullptr, &betas[curImage]);
#endif
#endif

            curImage++;
        }


#ifdef FULLPROBLEM
        camPose << 0, 0, 0, 1, 0, 0, 0;
        markerBAProblem.AddParameterBlock(&camPose(0), 3, yconstant_parametrization);
        markerBAProblem.AddParameterBlock(&camPose(3), 4, quaternion_parameterization);

        markerBAProblem_RepError.AddParameterBlock(&camPose(0), 3, yconstant_parametrization2);
        markerBAProblem_RepError.AddParameterBlock(&camPose(3), 4, quaternion_parameterization2);
#else
        for (int i = 0; i < distinctTiltTicks.size(); i++)
        {
            camPoses[i] << 0, 0, 0, 1, 0, 0, 0;
            markerBAProblem.AddParameterBlock(&camPoses[i](0), 3);
            markerBAProblem.AddParameterBlock(&camPoses[i](3), 4, quaternion_parameterization);
        }
#endif

        curImage = 0;
        for (size_t i = 0; i < ptuData.ptuImagePoses.size(); i++)
        {
            if (ptuData.ptuImagePoses[i].cameraId != 1) continue;

            Eigen::Matrix<double, 7, 1> camPose_origin;
            camPose_origin.segment<3>(0) = reconstructedPoses[i].t;
            camPose_origin.segment<4>(3) = reconstructedPoses[i].q;
            camPose_origin = poseInverse(camPose_origin); // cam to world
            std::cout << "CamPose_Origin : " << camPose_origin.transpose() << std::endl;

#ifdef FULLPROBLEM
            constexpr bool robustify = true;
            auto* currentCostFunc = PTUPoseErrorTilt::Create(camPose_origin);
            markerBAProblem.AddResidualBlock(currentCostFunc,
                robustify ? new ceres::HuberLoss(1.0) : nullptr, // new ceres::CauchyLoss(3),
                &panPoseInv(0), &panPoseInv(3), &alphas[curImage], &tiltPoseInv(0), &tiltPoseInv(3),
                &betas[curImage], &camPose(0), &camPose(3));

            size_t detectedImageId = -1;
            for (size_t j = 0; j < ptuDetectionResult.images.size(); j++)
            {
                // TODO check if works
                if (strstr(ptuData.ptuImagePoses[i].imagePath.c_str(),
                        ptuDetectionResult.images[j].filePath.c_str()))
                {
                    detectedImageId = j;
                    break;
                }
            }

            const visual_marker_mapping::CameraModel& camModel
                = ptuData.cameraModelById[ptuData.ptuImagePoses[i].cameraId];

            for (const auto& tagObs : ptuDetectionResult.tagObservations)
            {
                if (tagObs.imageId != detectedImageId) continue;
                const auto tagIt = reconstructedTags.find(tagObs.tagId);
                if (tagIt == reconstructedTags.end()) continue;

                const std::vector<Eigen::Vector3d> tagCorners
                    = tagIt->second.computeMarkerCorners3D();
                for (int c = 0; c < 4; c++)
                {
                    // check origin pose
                    // {
                    //     OpenCVReprojectionError repErr(tagObs.corners[c],
                    //     camModel.distortionCoefficients,camModel.getK());
                    //     repErr.print=true;
                    //     double res[2];
                    //     repErr(&camPose_origin(0), &camPose_origin(3), &tagCorners[c](0), res);
                    // }
                    //

                    auto* currentCostFunc2 = PTUPoseErrorTiltRepError::Create(tagObs.corners[c],
                        tagCorners[c], camModel.distortionCoefficients, camModel.getK());
                    markerBAProblem_RepError.AddResidualBlock(currentCostFunc2,
                        robustify ? new ceres::HuberLoss(1.0)
                                  : nullptr, // new ceres::CauchyLoss(3),
                        &panPoseInv(0),
                        &panPoseInv(3), &alphas[curImage], &tiltPoseInv(0), &tiltPoseInv(3),
                        &betas[curImage], &camPose(0), &camPose(3));
                }
            }
#else
            auto* currentCostFunc = PTUPoseError::Create(camPose_origin);

            bool robustify = true;

            int curTickIndex = distinctTiltTicks[ptuData.ptuImagePoses[i].tiltTicks];

            markerBAProblem.AddResidualBlock(currentCostFunc,
                robustify ? new ceres::HuberLoss(1.0) : nullptr, // new ceres::CauchyLoss(3),
                &panPose(0), &panPose(3), &alphas[curImage], &camPoses[curTickIndex](0),
                &camPoses[curTickIndex](3));
#endif
            curImage++;
            // break;
        }
        std::cout << "done creating problem " << std::endl;

        ceres::Solver::Options options;
        // #if (CERES_VERSION_MAJOR==1)&&(CERES_VERSION_MINOR==11)
        //     options.linear_solver_ordering.reset(ordering);
        // #else
        //     options.linear_solver_ordering=ordering;
        // #endif
        options.minimizer_progress_to_stdout = false;
        options.max_num_iterations = 100; // maxNumIterations;
        options.num_threads = 4; // ceresThreads;
        options.num_linear_solver_threads = 4; // ceresThreads;
        // options.eta = 1e-2;

        ceres::Solver::Summary summary;
        Solve(options, &markerBAProblem, &summary);
        std::cout << "Solution " << summary.termination_type << std::endl;

        ceres::Solver::Summary summary2;
        Solve(options, &markerBAProblem_RepError, &summary2);
        std::cout << "Solution2 " << summary2.termination_type << std::endl;
        // if (printSummary)
        std::cout << summary2.FullReport() << std::endl;

        panPoseInv(0) = fabs(panPoseInv(0));
        tiltPoseInv(0) = fabs(tiltPoseInv(0));

        std::cout << "PanPose: " << poseInverse(panPoseInv).transpose() << std::endl;
        std::cout << "TiltPose: " << poseInverse(tiltPoseInv).transpose() << std::endl;

        std::cout << "Alphas: ";
        for (int i = 0; i < alphas.size(); i++)
            std::cout << alphas[i] << ", ";
        std::cout << std::endl;

#ifdef FULLPROBLEM
        std::cout << "CamPose: " << std::endl;
        std::cout << camPose.transpose() << std::endl;
#else
        std::cout << "CamPoses: " << std::endl;
        for (int i = 0; i < camPoses.size(); i++)
            std::cout << camPoses[i].transpose() << std::endl;
#endif

        // auto invPanPose=poseInverse(panPose);

        // Render Pan Pose
        {
            DebugVis dbg;
            dbg.cam.q = panPoseInv.segment<4>(3);
            dbg.cam.t = panPoseInv.segment<3>(0);
            debugVis.push_back(dbg);
        }
#ifdef FULLPROBLEM
        // Render Tilt Pose
        {
            auto invp = poseInverse(poseAdd(poseInverse(panPoseInv), poseInverse(tiltPoseInv)));

            DebugVis dbg;
            dbg.cam.q = invp.segment<4>(3);
            dbg.cam.t = invp.segment<3>(0);
            debugVis.push_back(dbg);
        }

        std::ofstream dbgAngleFile("dbgAngles.txt");
        std::vector<double> reprojectionErrors;
        curImage = 0;
        double meanRepErrorSqr = 0.0;
        int numRepErrors = 0;
        for (size_t i = 0; i < ptuData.ptuImagePoses.size(); i++)
        {
            if (ptuData.ptuImagePoses[i].cameraId != 1) continue;
            double p = ptuData.ptuImagePoses[i].panAngle;
            Eigen::Matrix<double, 7, 1> panRot;
            panRot << 0, 0, 0, cos(p / 2.0), 0, 0, sin(p / 2.0);

            double t = ptuData.ptuImagePoses[i].tiltAngle;
            Eigen::Matrix<double, 7, 1> tiltRot;
            tiltRot << 0, 0, 0, cos(t / 2.0), 0, 0, sin(t / 2.0);

            dbgAngleFile << ptuData.ptuImagePoses[i].panAngle << " "
                         << ptuData.ptuImagePoses[i].panAngle - alphas[curImage] << " "
                         << ptuData.ptuImagePoses[i].tiltAngle - betas[curImage] << std::endl;

            auto invp = poseInverse(poseAdd(
                poseAdd(poseAdd(poseAdd(poseInverse(panPoseInv), panRot), poseInverse(tiltPoseInv)),
                    tiltRot),
                camPose));

            DebugVis dbg;
            dbg.cam.q = invp.segment<4>(3);
            dbg.cam.t = invp.segment<3>(0);
            debugVis.push_back(dbg);


            // haesslich!

            size_t detectedImageId = -1;
            for (size_t j = 0; j < ptuDetectionResult.images.size(); j++)
            {
                // TODO check if works
                if (strstr(ptuData.ptuImagePoses[i].imagePath.c_str(),
                        ptuDetectionResult.images[j].filePath.c_str()))
                {
                    detectedImageId = j;
                    break;
                }
            }

            const visual_marker_mapping::CameraModel& camModel
                = ptuData.cameraModelById[ptuData.ptuImagePoses[i].cameraId];

            for (const auto& tagObs : ptuDetectionResult.tagObservations)
            {
                if (tagObs.imageId != detectedImageId) continue;
                const auto tagIt = reconstructedTags.find(tagObs.tagId);
                if (tagIt == reconstructedTags.end()) continue;

                const std::vector<Eigen::Vector3d> tagCorners
                    = tagIt->second.computeMarkerCorners3D();

                for (int c = 0; c < 4; c++)
                {
                    PTUPoseErrorTiltRepError currentCostFunc2(tagObs.corners[c], tagCorners[c],
                        camModel.distortionCoefficients, camModel.getK());
                    currentCostFunc2.repError.print = false;

                    Eigen::Vector2d repError;

                    currentCostFunc2(&panPoseInv(0), &panPoseInv(3), &alphas[curImage],
                        &tiltPoseInv(0), &tiltPoseInv(3), &betas[curImage], &camPose(0),
                        &camPose(3), &repError(0));

                    // std::cout <<  "Rep Error: " << repError.norm() << std::endl;
                    meanRepErrorSqr += repError.squaredNorm();
                    numRepErrors++;

                    reprojectionErrors.push_back(repError.norm());
                }
            }
            curImage++;
        }
        meanRepErrorSqr /= numRepErrors;
        meanRepErrorSqr = sqrt(meanRepErrorSqr);
        std::cout << "RMS Rep Error: " << meanRepErrorSqr << std::endl;

        // Dump Rep Errors
        {
            std::ofstream myfile(folder + "/reprojectionErrors.txt");
            for (const auto& repError : reprojectionErrors)
                myfile << repError << "\n";
        }

#else
        for (int i = 0; i < camPoses.size(); i++)
        {
            auto ret = poseAdd(poseInverse(panPoseInv), camPoses[i]);
            auto invRet = poseInverse(ret);
            DebugVis dbg;
            dbg.cam.q = invRet.segment<4>(3);
            dbg.cam.t = invRet.segment<3>(0);
            debugVis.push_back(dbg);
        }
#endif
    }
}
//-----------------------------------------------------------------------------
bool PtuCalibrationProject::computeRelativeCameraPoseFromImg(int imageId, const Eigen::Matrix3d& K,
    const Eigen::Matrix<double, 5, 1>& distCoefficients, Eigen::Quaterniond& q,
    Eigen::Vector3d& t) const
{
    std::vector<Eigen::Vector3d> markerCorners3D;
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > observations2D;
    // find all matches between this image and the reconstructions
    for (const auto& tagObs : ptuDetectionResult.tagObservations)
    {
        if (tagObs.imageId != imageId) continue;
        const auto tagIt = reconstructedTags.find(tagObs.tagId);
        if (tagIt == reconstructedTags.end()) continue;

        const std::vector<Eigen::Vector3d> tagCorners = tagIt->second.computeMarkerCorners3D();

        markerCorners3D.insert(markerCorners3D.begin(), tagCorners.begin(), tagCorners.end());
        observations2D.insert(observations2D.begin(), tagObs.corners.begin(), tagObs.corners.end());
    }
    std::cout << "   Reconstructing camera pose from " << observations2D.size()
              << " 2d/3d correspondences" << std::endl;

    if (observations2D.empty()) return false;

    Eigen::Matrix3d R;
    // solvePnPEigen(markerCorners3D, observations2D, K, distCoefficients, R, t);
    visual_marker_mapping::solvePnPRansacEigen(markerCorners3D, observations2D, K, distCoefficients, R, t);

    q = Eigen::Quaterniond(R);
    return true;
}
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
