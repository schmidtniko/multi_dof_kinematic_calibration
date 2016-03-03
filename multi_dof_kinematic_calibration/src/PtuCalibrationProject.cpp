#include "multi_dof_kinematic_calibration/PtuCalibrationProject.h"

#include <Eigen/Geometry>

#include "visual_marker_mapping/ReconstructionIO.h"
#include "visual_marker_mapping/DetectionIO.h"
#include "visual_marker_mapping/EigenCVConversions.h"
#include "visual_marker_mapping/PropertyTreeUtilities.h"

#include "multi_dof_kinematic_calibration/CeresUtil.h"

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>


#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <ceres/version.h>

#include <iostream>
#include <fstream>

namespace multi_dof_kinematic_calibration
{
#if 0
struct PTUPoseError {
 	PTUPoseError(const Eigen::Matrix<double,7,1>& camPose_origin) : camPose_origin(camPose_origin) {}

    template <typename T>
    bool operator()(const T* const panpose_xyz,
                    const T* const panpose_q,
                    const T* const alpha,
                    const T* const campose_xyz,
                    const T* const campose_q,
                    T* residuals) const {

        Eigen::Matrix<T,4,1> alpha_q;
        alpha_q << cos((*alpha)/T(2)), T(0), T(0), sin((*alpha)/T(2));
        Eigen::Matrix<T,3,1> alpha_xyz;
        alpha_xyz << T(0), T(0), T(0);

        Eigen::Matrix<T,4,1> tmp_q;
        Eigen::Matrix<T,3,1> tmp_xyz;
        poseAccum<T>(alpha_q, alpha_xyz, eMap4(campose_q), eMap3(campose_xyz), tmp_q, tmp_xyz);
        Eigen::Matrix<T,4,1> tmp2_q;
        Eigen::Matrix<T,3,1> tmp2_xyz;
        poseAccum<T>(eMap4(panpose_q), eMap3(panpose_xyz), tmp_q, tmp_xyz, tmp2_q, tmp2_xyz);

        eMap3(&residuals[0]) = tmp2_xyz - camPose_origin.segment<3>(0).cast<T>();

        Eigen::Matrix<T,4,1> camPose_origin_q_conj = camPose_origin.segment<4>(3).cast<T>();
        camPose_origin_q_conj.template segment<3>(1)*=T(-1);

        Eigen::Matrix<T,4,1> delta_q;
        ceres::QuaternionProduct(&tmp2_q(0), &camPose_origin_q_conj(0), &delta_q(0));

         ceres::QuaternionToAngleAxis(&delta_q(0), &residuals[3]);
         for (int i=0;i<3;i++)
         	residuals[3+i]*=T(1000);

        return true;
    }

    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction* Create(const Eigen::Matrix<double,7,1>& camPose_origin)
    {
        return (new ceres::AutoDiffCostFunction<PTUPoseError, 6, 3, 4, 1, 3, 4>(
                  new PTUPoseError(camPose_origin)));
    }

    Eigen::Matrix<double,7,1> camPose_origin;
};

struct PTUPoseErrorTilt {
 	PTUPoseErrorTilt(const Eigen::Matrix<double,7,1>& camPose_origin) : camPose_origin(camPose_origin) {}

    template <typename T>
    bool operator()(const T* const panposeinv_xyz,
                    const T* const panposeinv_q,
                    const T* const alpha,
                    const T* const tiltposeinv_xyz,
                    const T* const tiltposeinv_q,
                    const T* const beta,
                    const T* const campose_xyz,
                    const T* const campose_q,
                    T* residuals) const {

#if 1
        Eigen::Matrix<T,3,1> pz = eMap3(panposeinv_xyz);
        if (pz(0)<T(0)) pz(0)*=T(-1);
        auto panPose=cposeInv(cmakePose<T>(pz, eMap4(panposeinv_q)));
        Eigen::Matrix<T,3,1> tz = eMap3(tiltposeinv_xyz);
        if (tz(0)<T(0)) tz(0)*=T(-1);
        auto tiltPose=cposeInv(cmakePose<T>(tz, eMap4(tiltposeinv_q)));
        auto camPose=cmakePose<T>(eMap3(campose_xyz), eMap4(campose_q));

        Eigen::Matrix<T, 7, 1> betaPose;
        betaPose << T(0),T(0),T(0),cos((*beta)/T(2)), T(0), T(0), sin((*beta)/T(2));
        //betaPose << T(0), T(0), T(0), T(1), T(0), T(0), T(0);
        Eigen::Matrix<T, 7, 1> alphaPose;
        alphaPose << T(0),T(0),T(0),cos((*alpha)/T(2)), T(0), T(0), sin((*alpha)/T(2));
        //alphaPose << T(0), T(0), T(0), T(1), T(0), T(0), T(0);

        auto camWorldPose=cposeAdd(panPose, cposeAdd(alphaPose, cposeAdd(tiltPose, cposeAdd(betaPose, camPose))));

        // auto deltaPose = cposeAdd(camWorldPose,cposeInv<T>(camPose_origin.cast<T>()));
        // Eigen::Matrix<T, 7, 1> zeroPose;
        // zeroPose << T(0),T(0),T(0),T(1),T(0),T(0),T(0);
        // eMap6(&residuals[0])=cposeManifoldMinus<T>(deltaPose, zeroPose);

        eMap6(&residuals[0])=cposeManifoldMinus<T>(camWorldPose, camPose_origin.cast<T>());

        //for (int i=0;i<3;i++)
            //residuals[3+i]*=T(1000);
#else
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

        eMap3(&residuals[0]) = tmp2_xyz - camPose_origin.segment<3>(0).cast<T>();

        Eigen::Matrix<T,4,1> camPose_origin_q_conj = camPose_origin.segment<4>(3).cast<T>();
        camPose_origin_q_conj.template segment<3>(1)*=T(-1);

        Eigen::Matrix<T,4,1> delta_q;
        ceres::QuaternionProduct(&tmp2_q(0), &camPose_origin_q_conj(0), &delta_q(0));

        ceres::QuaternionToAngleAxis(&delta_q(0), &residuals[3]);
        for (int i=0;i<3;i++)
            residuals[3+i]*=T(1000);
#endif

        return true;
    }

    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction* Create(const Eigen::Matrix<double,7,1>& camPose_origin)
    {
        return (new ceres::AutoDiffCostFunction<PTUPoseErrorTilt, 6, 3, 4, 1, 3, 4, 1, 3, 4>(
                  new PTUPoseErrorTilt(camPose_origin)));
    }

    Eigen::Matrix<double,7,1> camPose_origin;
};


struct PTUPoseErrorTiltRepError {
    PTUPoseErrorTiltRepError(const Eigen::Vector2d& observation,
            const Eigen::Vector3d& point_3d,
            const Eigen::Matrix<double, 5, 1>& d,
            const Eigen::Matrix3d& K) : repError(observation,d,K), point_3d(point_3d) {}

    template <typename T>
    bool operator()(const T* const panposeinv_xyz,
                    const T* const panposeinv_q,
                    const T* const alpha,
                    const T* const tiltposeinv_xyz,
                    const T* const tiltposeinv_q,
                    const T* const beta,
                    const T* const campose_xyz,
                    const T* const campose_q,
                    T* residuals) const {
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
        Eigen::Matrix<T,3,1> pz = eMap3(panposeinv_xyz);
        if (pz(0)<T(0)) pz(0)*=T(-1);
        auto panPose=cposeInv(cmakePose<T>(pz, eMap4(panposeinv_q)));
        Eigen::Matrix<T,3,1> tz = eMap3(tiltposeinv_xyz);
        if (tz(0)<T(0)) tz(0)*=T(-1);
        auto tiltPose=cposeInv(cmakePose<T>(tz, eMap4(tiltposeinv_q)));
        auto camPose=cmakePose<T>(eMap3(campose_xyz), eMap4(campose_q));

        Eigen::Matrix<T, 7, 1> betaPose;
        betaPose << T(0),T(0),T(0),cos((*beta)/T(2)), T(0), T(0), sin((*beta)/T(2));
        //betaPose << T(0), T(0), T(0), T(1), T(0), T(0), T(0);
        Eigen::Matrix<T, 7, 1> alphaPose;
        alphaPose << T(0),T(0),T(0),cos((*alpha)/T(2)), T(0), T(0), sin((*alpha)/T(2));
        //alphaPose << T(0), T(0), T(0), T(1), T(0), T(0), T(0);

        auto camToWorld=cposeAdd(panPose, cposeAdd(alphaPose, cposeAdd(tiltPose, cposeAdd(betaPose, camPose))));

        auto worldToCam=cposeInv(camToWorld);

        Eigen::Matrix<T,3,1> point_3d_T = point_3d.cast<T>();
        return repError(&worldToCam(0), &worldToCam(3), &point_3d_T(0), residuals);

#endif
    }

    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction* Create(const Eigen::Vector2d& observation,
            const Eigen::Vector3d& point_3d,
            const Eigen::Matrix<double, 5, 1>& d,
            const Eigen::Matrix3d& K)
    {
        return (new ceres::AutoDiffCostFunction<PTUPoseErrorTiltRepError, 2, 3, 4, 1, 3, 4, 1, 3, 4>(
                  new PTUPoseErrorTiltRepError(observation, point_3d, d, K)));
    }

    OpenCVReprojectionError repError;
    Eigen::Vector3d point_3d;
};
#endif
struct DynPTUPoseErrorTiltRepError
{
    DynPTUPoseErrorTiltRepError(const Eigen::Vector2d& observation, const Eigen::Vector3d& point_3d,
        const Eigen::Matrix<double, 5, 1>& d, const Eigen::Matrix3d& K, int numJoints)
        : repError(observation, d, K)
        , point_3d(point_3d)
        , numJoints(numJoints)
    {
    }

    template <typename T> bool operator()(T const* const* parameters, T* residuals) const
    {

        const auto parent_to_cam = cmakePose<T>(
            eMap3(parameters[4 * numJoints + 0]), eMap4(parameters[4 * numJoints + 1]));

        auto world_to_cam = parent_to_cam;
        for (int i = numJoints - 1; i >= 0; i--)
        {
            const auto rotaxis_to_parent
                = cmakePose<T>(eMap3(parameters[i * 4 + 0]), eMap4(parameters[i * 4 + 1]));
            const T jointScale = *parameters[4 * i + 3];
            const T angle = *parameters[i * 4 + 2] * jointScale;
            Eigen::Matrix<T, 7, 1> invRot;
            invRot << T(0), T(0), T(0), cos(-angle / T(2)), T(0), T(0), sin(-angle / T(2));

            world_to_cam = cposeAdd(world_to_cam, cposeAdd(invRot, rotaxis_to_parent));
        }

        const Eigen::Matrix<T, 3, 1> point_3d_T = point_3d.cast<T>();
        return repError(&world_to_cam(0), &world_to_cam(3), &point_3d_T(0), residuals);
    }

    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction* Create(const Eigen::Vector2d& observation,
        const Eigen::Vector3d& point_3d, const Eigen::Matrix<double, 5, 1>& d,
        const Eigen::Matrix3d& K, int numJoints)
    {
        auto cost_function = new ceres::DynamicAutoDiffCostFunction<DynPTUPoseErrorTiltRepError, 4>(
            new DynPTUPoseErrorTiltRepError(observation, point_3d, d, K, numJoints));
        for (int i = 0; i < numJoints; i++)
        {
            cost_function->AddParameterBlock(3);
            cost_function->AddParameterBlock(4);
            cost_function->AddParameterBlock(1);
            cost_function->AddParameterBlock(1);
        }
        cost_function->AddParameterBlock(3);
        cost_function->AddParameterBlock(4);
        cost_function->SetNumResiduals(2);
        return cost_function;
    }

    OpenCVReprojectionError repError;
    Eigen::Vector3d point_3d;
    int numJoints;
};

struct DynPTUPoseErrorTilt
{
    DynPTUPoseErrorTilt(const Eigen::Matrix<double, 7, 1>& expected_world_to_cam, int numJoints)
        : expected_world_to_cam(expected_world_to_cam)
        , numJoints(numJoints)
    {
    }

    template <typename T> bool operator()(T const* const* parameters, T* residuals) const
    {

        const auto parent_to_cam = cmakePose<T>(
            eMap3(parameters[4 * numJoints + 0]), eMap4(parameters[4 * numJoints + 1]));

        auto world_to_cam = parent_to_cam;
        for (int i = numJoints - 1; i >= 0; i--)
        {
            const auto rotaxis_to_parent
                = cmakePose<T>(eMap3(parameters[i * 4 + 0]), eMap4(parameters[i * 4 + 1]));
            const T jointScale = *parameters[4 * i + 3];
            const T angle = *parameters[i * 4 + 2] * jointScale;
            Eigen::Matrix<T, 7, 1> invRot;
            invRot << T(0), T(0), T(0), cos(-angle / T(2)), T(0), T(0), sin(-angle / T(2));

            world_to_cam = cposeAdd(world_to_cam, cposeAdd(invRot, rotaxis_to_parent));
        }

        eMap6(&residuals[0]) = cposeManifoldMinus<T>(world_to_cam, expected_world_to_cam.cast<T>());
        // eMap3(&residuals[3])*=T(1000);
        return true;
    }

    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction* Create(
        const Eigen::Matrix<double, 7, 1>& expected_world_to_cam, int numJoints)
    {
        auto cost_function = new ceres::DynamicAutoDiffCostFunction<DynPTUPoseErrorTilt, 4>(
            new DynPTUPoseErrorTilt(expected_world_to_cam, numJoints));
        for (int i = 0; i < numJoints; i++)
        {
            cost_function->AddParameterBlock(3);
            cost_function->AddParameterBlock(4);
            cost_function->AddParameterBlock(1);
            cost_function->AddParameterBlock(1);
        }
        cost_function->AddParameterBlock(3);
        cost_function->AddParameterBlock(4);
        cost_function->SetNumResiduals(6);
        return cost_function;
    }

    Eigen::Matrix<double, 7, 1> expected_world_to_cam;
    int numJoints;
};


//-----------------------------------------------------------------------------
void PtuCalibrationProject::optimizeJoint(const std::string& jointName)
{
    const int onlyCamId = 1;

    int jointIndex = -1;
    for (int i = 0; i < (int)ptuData.jointNames.size(); i++)
    {
        if (ptuData.jointNames[i] == jointName) jointIndex = i;
    }

    auto poseInverse = [](const Eigen::Matrix<double, 7, 1>& pose)
    {
        return cposeInv<double>(pose);
    };
    auto poseAdd = [](const Eigen::Matrix<double, 7, 1>& x,
        const Eigen::Matrix<double, 7, 1>& d) -> Eigen::Matrix<double, 7, 1>
    {
        return cposeAdd<double>(x, d);
    };


    //#define ANGLEOFFSET

    ceres::Problem markerBAProblem;
    ceres::Problem markerBAProblem_RepError;

    std::vector<int> yzconstant_params = { 1, 2 };
    auto yzconstant_parametrization = new ceres::SubsetParameterization(3, yzconstant_params);
    auto yzconstant_parametrization2 = new ceres::SubsetParameterization(3, yzconstant_params);

    auto quaternion_parameterization = new ceres::QuaternionParameterization;
    auto quaternion_parameterization2 = new ceres::QuaternionParameterization;

    for (int j = 0; j < jointIndex + 1; j++)
    {
        auto& joint_to_parent_pose = jointData[j].joint_to_parent_pose;

// jointData[j].joint_to_parent_pose <<
// 1.5*sqrt(2),0,0,cos(135/180.0*M_PI/2.0),0,0,sin(135/180.0*M_PI/2.0);

#if 1
        // x always has to be positive; y,z have to be 0
        markerBAProblem.AddParameterBlock(&joint_to_parent_pose(0), 3, yzconstant_parametrization);
        markerBAProblem.AddParameterBlock(&joint_to_parent_pose(3), 4, quaternion_parameterization);

        // markerBAProblem.SetParameterLowerBound(&joint_to_parent_pose(0), 0, 0);

        markerBAProblem.AddParameterBlock(&jointData[j].ticks_to_rad, 1);
        // markerBAProblem.SetParameterBlockConstant(&jointData[j].ticks_to_rad);

        markerBAProblem_RepError.AddParameterBlock(
            &joint_to_parent_pose(0), 3, yzconstant_parametrization2);
        markerBAProblem_RepError.AddParameterBlock(
            &joint_to_parent_pose(3), 4, quaternion_parameterization2);

        markerBAProblem_RepError.AddParameterBlock(&jointData[j].ticks_to_rad, 1);
// markerBAProblem_RepError.SetParameterBlockConstant(&jointData[j].ticks_to_rad);


// markerBAProblem_RepError.SetParameterLowerBound(&joint_to_parent_pose(0), 0, 0);
#else
        // x always has to be positive; y,z have to be 0
        markerBAProblem.AddParameterBlock(&joint_to_parent_pose(0), 3, yzconstant_parametrization);
        markerBAProblem.AddParameterBlock(&joint_to_parent_pose(3), 4, quaternion_parameterization);

        markerBAProblem.SetParameterBlockConstant(&joint_to_parent_pose(0));
        markerBAProblem.SetParameterBlockConstant(&joint_to_parent_pose(3));

        markerBAProblem_RepError.AddParameterBlock(
            &joint_to_parent_pose(0), 3, yzconstant_parametrization2);
        markerBAProblem_RepError.AddParameterBlock(
            &joint_to_parent_pose(3), 4, quaternion_parameterization2);

        markerBAProblem_RepError.SetParameterBlockConstant(&joint_to_parent_pose(0));
        markerBAProblem_RepError.SetParameterBlockConstant(&joint_to_parent_pose(3));
#endif
    }


    using DistinctJointIndex = std::vector<int>;
    using DistinctLeverIndex = int;
    std::vector<DistinctJointIndex> indexToDistinctJoint(ptuData.ptuImagePoses.size());
    std::vector<DistinctLeverIndex> indexToDistinctLever(ptuData.ptuImagePoses.size());

    using JointState = int;
    using LeverState = std::vector<int>;
    std::vector<std::map<JointState, int> > distinctJointPositions(ptuData.jointNames.size());
    std::map<LeverState, DistinctLeverIndex> distinctLeverPositions;

    std::map<int, int> distinctFrequencies;


    std::map<DistinctLeverIndex, Eigen::Matrix<double, 7, 1> > camPoses;

    std::vector<std::map<int, double> > joint_positions(ptuData.jointNames.size());

    int numc = 0;
    for (size_t i = 0; i < ptuData.ptuImagePoses.size(); i++)
    {
        if (ptuData.ptuImagePoses[i].cameraId != onlyCamId) continue;

        const auto& jointConfig = ptuData.ptuImagePoses[i].jointConfiguration;


        for (int j = 0; j < jointIndex + 1; j++) // this loop could be swapped with the prev
        {
            if (0)
            { // one angle variable for each distinct(!) ptu pose
                auto it = distinctJointPositions[j].find(jointConfig[j]);
                if (it == distinctJointPositions[j].end())
                {
                    const int dj = distinctJointPositions[j].size();
                    indexToDistinctJoint[i].push_back(dj);
                    distinctJointPositions[j].emplace(jointConfig[j], dj);

                    //
                    joint_positions[j][dj]
                        = jointConfig[j] * 0.051429 / 180.0 * M_PI; //*0.00089779559;
                    markerBAProblem.AddParameterBlock(&joint_positions[j][dj], 1);
                    markerBAProblem.SetParameterBlockConstant(&joint_positions[j][dj]);

                    markerBAProblem_RepError.AddParameterBlock(&joint_positions[j][dj], 1);

                    // markerBAProblem_RepError.SetParameterBlockConstant(&joint_positions[j][dj]);
                    auto* alphaPrior
                        = GaussianPrior1D::Create(joint_positions[j][dj], 0.05 / 180.0 * M_PI);
                    markerBAProblem_RepError.AddResidualBlock(
                        alphaPrior, nullptr, &joint_positions[j][dj]);
                }
                else
                    indexToDistinctJoint[i].push_back(it->second);
            }
            else
            { // one angle variable per ptu pose
                indexToDistinctJoint[i].push_back(numc);
                distinctJointPositions[j].emplace(jointConfig[j], numc);

                //
                joint_positions[j][numc]
                    = jointConfig[j]; //*0.051429/180.0*M_PI*0.8;//*0.00089779559;
                markerBAProblem.AddParameterBlock(&joint_positions[j][numc], 1);
                markerBAProblem.SetParameterBlockConstant(&joint_positions[j][numc]);

                markerBAProblem_RepError.AddParameterBlock(&joint_positions[j][numc], 1);

                //                markerBAProblem_RepError.SetParameterBlockConstant(&joint_positions[j][numc]);
                // SET NOISE HERE
                auto* alphaPrior
                    = GaussianPrior1D::Create(joint_positions[j][numc], 1); // 0.05/180.0*M_PI);
                markerBAProblem_RepError.AddResidualBlock(
                    alphaPrior, nullptr, &joint_positions[j][numc]);
            }
        }
        {
            // one camera pose for each distinct lever config
            const std::vector<int> leverConfig(
                jointConfig.begin() + jointIndex + 1, jointConfig.end());

            auto it = distinctLeverPositions.find(leverConfig);
            if (it == distinctLeverPositions.end())
            {
                const int dl = distinctLeverPositions.size();
                indexToDistinctLever[i] = dl;
                distinctLeverPositions.emplace(leverConfig, dl);

                distinctFrequencies[dl] = 1;

                // std::cout << "New Config: ";
                // for (auto i : leverConfig)
                //     std::cout << i << " , ";
                // std::cout << std::endl;

                camPoses[dl] << 0, 0, 0, 1, 0, 0, 0;
                markerBAProblem.AddParameterBlock(&camPoses[dl](0), 3);
                markerBAProblem.AddParameterBlock(&camPoses[dl](3), 4, quaternion_parameterization);

                markerBAProblem_RepError.AddParameterBlock(&camPoses[dl](0), 3);
                markerBAProblem_RepError.AddParameterBlock(
                    &camPoses[dl](3), 4, quaternion_parameterization2);
            }
            else
            {
                indexToDistinctLever[i] = it->second;
                // std::cout << "Existing Config: ";
                // for (auto i : leverConfig)
                //     std::cout << i << " , ";
                // std::cout << std::endl;
                distinctFrequencies[it->second]++;
            }
        }
        numc++;
    }


    std::vector<std::function<double()> > repErrorFns;


    // curImage=0;
    for (size_t i = 0; i < ptuData.ptuImagePoses.size(); i++)
    {
        if (ptuData.ptuImagePoses[i].cameraId != onlyCamId) continue;

        if (distinctFrequencies[indexToDistinctLever[i]]
            < 2) // only accept lever groups with at least 2 calibration frames
        {
            // std::cout << "Skipping" << std::endl;
            continue;
        }
        // std::cout << "LeverGroupSize: " << distinctFrequencies[indexToDistinctLever[i]]  <<
        // std::endl;

        Eigen::Matrix<double, 7, 1> world_to_cam;
        world_to_cam.segment<3>(0) = reconstructedPoses[i].t;
        world_to_cam.segment<4>(3) = reconstructedPoses[i].q;
        //Eigen::Matrix<double, 7, 1> cam_to_world = poseInverse(world_to_cam);
        // std::cout << "CamToWorld : " << cam_to_world.transpose() << std::endl;

        constexpr bool robustify = false;
        auto* currentCostFunc = DynPTUPoseErrorTilt::Create(world_to_cam, jointIndex + 1);
        std::vector<double*> parameter_blocks;
        for (int j = 0; j < jointIndex + 1; j++)
        {
            const int dj = indexToDistinctJoint[i][j];
            parameter_blocks.push_back(&jointData[j].joint_to_parent_pose(0));
            parameter_blocks.push_back(&jointData[j].joint_to_parent_pose(3));
            parameter_blocks.push_back(&joint_positions[j][dj]);
            parameter_blocks.push_back(&jointData[j].ticks_to_rad);
        }
        const int dl = indexToDistinctLever[i];
        parameter_blocks.push_back(&camPoses[dl](0));
        parameter_blocks.push_back(&camPoses[dl](3));
        markerBAProblem.AddResidualBlock(currentCostFunc,
            robustify ? new ceres::HuberLoss(1.0) : nullptr, // new ceres::CauchyLoss(3),
            parameter_blocks);


        int detectedImageId = -1;
        for (int j = 0;
             j < (int)ptuDetectionResults[ptuData.ptuImagePoses[i].cameraId].images.size(); j++)
        {
            if (strstr(ptuData.ptuImagePoses[i].imagePath.c_str(),
                    ptuDetectionResults[ptuData.ptuImagePoses[i].cameraId]
                        .images[j]
                        .filename.c_str()))
            {
                detectedImageId = j;
                break;
            }
        }

        const visual_marker_mapping::CameraModel& camModel
            = ptuData.cameraModelById[ptuData.ptuImagePoses[i].cameraId];

        for (const auto& tagObs :
            ptuDetectionResults[ptuData.ptuImagePoses[i].cameraId].tagObservations)
        {
            if (tagObs.imageId != detectedImageId) continue;
            const auto tagIt = reconstructedTags.find(tagObs.tagId);
            if (tagIt == reconstructedTags.end()) continue;

            const std::vector<Eigen::Vector3d> tagCorners = tagIt->second.computeMarkerCorners3D();
            for (int c = 0; c < 4; c++)
            {
                // check origin pose
                // {
                //     OpenCVReprojectionError repErr(tagObs.corners[c],
                //     camModel.distortionCoefficients,camModel.getK());
                //     repErr.print=true;
                //     double res[2];
                //     repErr(&world_to_cam(0), &world_to_cam(3), &tagCorners[c](0), res);
                //     std::cout << "ERR: " << sqrt(res[0]*res[0]+res[1]*res[1]) << std::endl;
                // }


                auto* currentCostFunc2
                    = DynPTUPoseErrorTiltRepError::Create(tagObs.corners[c], tagCorners[c],
                        camModel.distortionCoefficients, camModel.getK(), jointIndex + 1);
                markerBAProblem_RepError.AddResidualBlock(currentCostFunc2,
                    robustify ? new ceres::HuberLoss(1.0) : nullptr, // new ceres::CauchyLoss(3),
                    parameter_blocks);

                repErrorFns.push_back([parameter_blocks, currentCostFunc2]() -> double
                    {
                        double err[2];
                        (*currentCostFunc2).Evaluate(&parameter_blocks[0], &err[0], nullptr);
                        return err[0] * err[0] + err[1] * err[1];
                    });
            }
        }
    }
    std::cout << "done creating problem " << std::endl;
    // exit(0);

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

#if 0
    for (int j=0;j<jointIndex;j++)
    {
        auto& joint_to_parent_pose = jointData[j].joint_to_parent_pose;
        markerBAProblem.SetParameterBlockConstant(&joint_to_parent_pose(0));
        markerBAProblem.SetParameterBlockConstant(&joint_to_parent_pose(3));
    }

    ceres::Solver::Summary summary_pre;
    Solve(options, &markerBAProblem, &summary_pre);
    std::cout << "Rough Pre Solution: " << summary_pre.termination_type << std::endl;

    for (int j=0;j<jointIndex;j++)
    {
        auto& joint_to_parent_pose = jointData[j].joint_to_parent_pose;
        markerBAProblem.SetParameterBlockVariable(&joint_to_parent_pose(0));
        markerBAProblem.SetParameterBlockVariable(&joint_to_parent_pose(3));
    }
#endif


    ceres::Solver::Summary summary;
    Solve(options, &markerBAProblem, &summary);
    std::cout << "Rough Solution: " << summary.termination_type << std::endl;
    // std::cout << summary.FullReport() << std::endl;

    {
        double rms = 0;
        for (int i = 0; i < (int)repErrorFns.size(); i++)
        {
            double sqrError = repErrorFns[i]();
            // std::cout << "RepError: " << sqrt(sqrError) << std::endl;
            rms += sqrError;
        }
        std::cout << "Reprojection Error RMS: " << sqrt(rms / repErrorFns.size()) << std::endl;
    }


    ceres::Solver::Summary summary2;
    Solve(options, &markerBAProblem_RepError, &summary2);
    std::cout << "Repr Error Solution: " << summary2.termination_type << std::endl;
    // if (printSummary)
    std::cout << summary2.FullReport() << std::endl;

    for (int j = 0; j < jointIndex + 1; j++) // this loop could be swapped with the prev
    {
        std::cout << "SCALE " << j << " : " << jointData[j].ticks_to_rad << std::endl;
    }

    // std::vector<int> angleOffsets(jointIndex+1);


    // x value is always positive for jointPoses
    // for (int j=0;j<jointIndex+1;j++)
    // {
    //     if (jointData[j].joint_to_parent_pose(0)<0)
    //     {
    //         Eigen::Matrix<double,7,1> preflip;
    //         preflip << 0,0,0,0,1,0,0;

    //         Eigen::Matrix<double,7,1> flip;
    //         flip << 0,0,0,0,0,0,1;
    //         std::cout << "AFTER FLIP1: " << jointData[j].joint_to_parent_pose.transpose() <<
    //         std::endl;
    //         jointData[j].joint_to_parent_pose=poseAdd(flip, jointData[j].joint_to_parent_pose);
    //         std::cout << "AFTER FLIP3: " << jointData[j].joint_to_parent_pose.transpose() <<
    //         std::endl;
    //     }
    // }

    // std::cout << "PanPose: " << poseInverse(panPoseInv).transpose() << std::endl;
    // std::cout << "TiltPose: " << poseInverse(tiltPoseInv).transpose() << std::endl;

    // std::cout << "Alphas: ";
    // for (int i=0;i<joint_positions[jointIndex].size();i++)
    //     std::cout << joint_positions[jointIndex][i] << ", ";
    // std::cout << std::endl;

    std::cout << "CamPoses: " << std::endl;
    for (int i = 0; i < (int)camPoses.size(); i++)
        std::cout << camPoses[i].transpose() << std::endl;

    {
        double rms = 0;
        for (int i = 0; i < (int)repErrorFns.size(); i++)
        {
            double sqrError = repErrorFns[i]();
            // std::cout << "RepError: " << sqrt(sqrError) << std::endl;
            rms += sqrError;
        }
        std::cout << "Reprojection Error RMS: " << sqrt(rms / repErrorFns.size()) << std::endl;
    }
#define exportJson
#ifdef exportJson
    std::ofstream jsonStream;
    jsonStream.open("results.json");
    jsonStream << "{" << std::endl;
    jsonStream << "\"camera_poses\":  [" << std::endl;
#endif

    if (jointIndex + 1 == (int)ptuData.jointNames.size())
    {
        for (size_t i = 0; i < ptuData.ptuImagePoses.size(); i++)
        {
            if (ptuData.ptuImagePoses[i].cameraId != onlyCamId) continue;

            const auto& jointConfig = ptuData.ptuImagePoses[i].jointConfiguration;

            Eigen::Matrix<double, 7, 1> root;
            root << 0, 0, 0, 1, 0, 0, 0;

            for (int j = 0; j < jointIndex + 1; j++)
            {
                root = poseAdd(root, poseInverse(jointData[j].joint_to_parent_pose));
                // std::cout << poseInverse(jointData[j].joint_to_parent_pose).transpose() << " !! "
                // << std::endl;

                double t = jointConfig[j] * 0.051429 / 180.0 * M_PI; //*0.00089779559;;
                // double t = joint_positions[j][indexToDistinctJoint[i][j]];
                Eigen::Matrix<double, 7, 1> tiltRot;
                tiltRot << 0, 0, 0, cos(t / 2.0), 0, 0, sin(t / 2.0);
                root = poseAdd(root, tiltRot);

                auto invRet = poseInverse(root);
                DebugVis dbg;
                dbg.cam.q = invRet.segment<4>(3);
                dbg.cam.t = invRet.segment<3>(0);
                dbg.type = 1;
                //        debugVis.push_back(dbg);
            }

#ifdef exportJson
            jsonStream << "{" << std::endl;
#endif
            // std::cout << "NumCamPoses" << camPoses.size() << std::endl;
            for (int c = 0; c < (int)camPoses.size(); c++)
            {
                auto ret = poseAdd(root, poseInverse(camPoses[c]));
                // std::cout << ret.transpose() << std::endl;
                auto invRet = poseInverse(ret);
                DebugVis dbg;
                dbg.cam.q = invRet.segment<4>(3);
                dbg.cam.t = invRet.segment<3>(0);

#ifdef exportJson
                jsonStream << "\"q\": [" << ret(3) << ", " << ret(4) << ", " << ret(5) << ", "
                           << ret(6) << "]," << std::endl;
                jsonStream << "\"t\": [" << ret(0) << ", " << ret(1) << ", " << ret(2) << "],"
                           << std::endl;
                jsonStream << "\"index\": " << i << std::endl;
#endif

                debugVis.push_back(dbg);
            }

#ifdef exportJson
            jsonStream << "}";
            if (i < ptuData.ptuImagePoses.size() - 1) jsonStream << ",";
            jsonStream << std::endl;
#endif
        }


        Eigen::Matrix<double, 7, 1> root;
        root << 0, 0, 0, 1, 0, 0, 0;

#ifdef exportJson
        jsonStream << "], " << std::endl;
        jsonStream << "\"axes\": [" << std::endl;
#endif

        for (int j = 0; j < jointIndex + 1; j++)
        {
            root = poseAdd(root, poseInverse(jointData[j].joint_to_parent_pose));

            jsonStream << "{" << std::endl;

            auto invRet = poseInverse(root);
            DebugVis dbg;
            dbg.cam.q = invRet.segment<4>(3);
            dbg.cam.t = invRet.segment<3>(0);
            dbg.type = 1;

// print for evaluation
#ifdef exportJson
            jsonStream << "\"joint\": " << j << "," << std::endl;
            jsonStream << "\"t\":[" << root(0) << ", " << root(1) << ", " << root(2) << "],"
                       << std::endl;
            jsonStream << "\"q\": [" << root(3) << ", " << root(4) << ", " << root(5) << ", "
                       << root(6) << "]" << std::endl;

            Eigen::Quaterniond tmpQ;
            //            tmpQ.x() = root(4);
            //            tmpQ.y() = root(5);
            //            tmpQ.z() = root(6);
            //            tmpQ.w() = root(3);
            //            std::cout << "tmpQ.R = " << tmpQ.toRotationMatrix() << std::endl;
            jsonStream << "}";
            if (j < jointIndex) jsonStream << ",";
            jsonStream << std::endl;
#endif

            debugVis.push_back(dbg);
        }

#ifdef exportJson
        jsonStream << "]" << std::endl;
        jsonStream << "}" << std::endl;
        jsonStream.close();
#endif

        if (1)
        {
            // compute rms for forward kinematics
            numc = 0;
            for (size_t i = 0; i < ptuData.ptuImagePoses.size(); i++)
            {
                if (ptuData.ptuImagePoses[i].cameraId != onlyCamId) continue;

                const auto& jointConfig = ptuData.ptuImagePoses[i].jointConfiguration;

                for (int j = 0; j < jointIndex + 1; j++) // this loop could be swapped with the prev
                {

                    joint_positions[j][numc] = jointConfig[j]; //*0.00089779559;
                }
                numc++;
            }
            {
                double rms = 0;
                for (int i = 0; i < (int)repErrorFns.size(); i++)
                {
                    double sqrError = repErrorFns[i]();
                    // std::cout << "RepError: " << sqrt(sqrError) << std::endl;
                    rms += sqrError;
                }
                std::cout << "Forward kinecmatics Reprojection Error RMS: "
                          << sqrt(rms / repErrorFns.size()) << std::endl;
            }
        }
    }
}
//-----------------------------------------------------------------------------
void PtuCalibrationProject::exportCalibrationResults(const std::string& filePath) const
{
    namespace pt = boost::property_tree;
    pt::ptree root;
   
    pt::ptree kinematicChainPt; 
    size_t counter = 1;
    for (const auto &joint : jointData)
    {
        const Eigen::Vector3d translation = joint.joint_to_parent_pose.head(3);
        const Eigen::Vector4d rotation = joint.joint_to_parent_pose.segment<4>(3);
        pt::ptree translationPt = visual_marker_mapping::matrix2PropertyTreeEigen(translation); 
        pt::ptree rotationPt = visual_marker_mapping::matrix2PropertyTreeEigen(rotation); 
        
        pt::ptree jointDataPt;
        jointDataPt.add_child("translation", translationPt);
        jointDataPt.add_child("rotation", rotationPt);
        jointDataPt.put("ticks_to_rad", joint.ticks_to_rad);

        if(counter == jointData.size())
            jointDataPt.put("name", "camera"); 
        else
            jointDataPt.put("name", "joint_" + std::to_string(counter));

        kinematicChainPt.push_back(std::make_pair("",jointDataPt));
        counter++;
    }
    
    root.add_child("kinematic_chain", kinematicChainPt);
    boost::property_tree::write_json(filePath, root);
}
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
        std::string filePath = folder + "/calibration_frames.json";
        ptuData=PtuImageCapture(folder + "/calibration_frames.json");
        std::cout << "Read PTU Data!" << std::endl;
    }

    {
        ptuDetectionResults[0]
            = visual_marker_mapping::readDetectionResult(folder + "/cam0/detectedMarkers.json");
        std::cout << "Read PTU Image Detections!" << std::endl;
        ptuDetectionResults[1]
            = visual_marker_mapping::readDetectionResult(folder + "/cam1/detectedMarkers.json");
        std::cout << "Read PTU Image Detections!" << std::endl;
    }


    for (size_t i = 0; i < ptuData.ptuImagePoses.size(); i++)
    {
        const visual_marker_mapping::CameraModel& camModel
            = ptuData.cameraModelById[ptuData.ptuImagePoses[i].cameraId];

        // std::cout << camModel.getK() << std::endl;
        // std::cout << camModel.distortionCoefficients.transpose() << std::endl;

        size_t detectedImageId = -1;
        for (size_t j = 0; j < ptuDetectionResults[ptuData.ptuImagePoses[i].cameraId].images.size();
             j++)
        {
            if (strstr(ptuData.ptuImagePoses[i].imagePath.c_str(),
                    ptuDetectionResults[ptuData.ptuImagePoses[i].cameraId]
                        .images[j]
                        .filename.c_str()))
            {
                detectedImageId = j;
                break;
            }
        }
        assert(
            detectedImageId >= 0); // if this fails, there is no detection for a certain ptu image

        Eigen::Quaterniond q;
        Eigen::Vector3d t;
        computeRelativeCameraPoseFromImg(ptuData.ptuImagePoses[i].cameraId, detectedImageId,
            camModel.getK(), camModel.distortionCoefficients, q, t);

        DebugVis dbg;
        dbg.cam.setQuat(q);
        dbg.cam.t = t;
        //      if (ptuData.ptuImagePoses[i].cameraId==0)
        //        debugVis.push_back(dbg);

        reconstructedPoses[i] = dbg.cam;
    }

    jointData.resize(ptuData.jointNames.size());
    for (int j = 0; j < (int)jointData.size(); j++)
    {
        jointData[j].joint_to_parent_pose << 0, 0, 0, 1, 0, 0, 0;
        jointData[j].ticks_to_rad = 0.051429 / 180.0 * M_PI;
    }
    for (const std::string& jointName : ptuData.jointNames)
    {
        std::cout << "Optimizing joint: " << jointName << std::endl;
        optimizeJoint(jointName);
        // continue;
        // return;
    }
// optimizeJoint(ptuData.jointNames[1]);
// optimizeJoint(ptuData.jointNames[2]);
#if 0
	///////////////////////////////////////////
	// Process

	int numCam1Images=0;
	std::map<int, int> distinctTiltTicks;

	int tickNum=0;

 	for (size_t i=0;i<ptuData.actuatorPoses.size();i++)
 	{
 		if (ptuData.actuatorPoses[i].cameraId!=1)
 			continue;

 		distinctTiltTicks.emplace(ptuData.actuatorPoses[i].tiltTicks, distinctTiltTicks.size());

 		const camSurv::CameraModel& camModel = ptuData.cameraModelById[ptuData.ptuImagePoses[i].cameraId];

        size_t detectedImageId=-1;
        for (size_t j=0;j<ptuDetectionResult.images.size();j++)
        {
        	if (strstr(ptuData.ptuImagePoses[i].imagePath.c_str(),ptuDetectionResult.images[j].filename.c_str()))
        	{
        		detectedImageId=j;
        		break;
        	}
        }
        assert(detectedImageId>=0); // if this fails, there is no detection for a certain ptu image
        
        Eigen::Quaterniond q;
        Eigen::Vector3d t;
        computeRelativeCameraPoseFromImg(detectedImageId, camModel.getK(), camModel.distortionCoefficients, q, t);

         DebugVis dbg;
         dbg.cam.setQuat(q);
         dbg.cam.t=t;
         //debugVis.push_back(dbg);

        reconstructedPoses[i]=dbg.cam;


        //std::cout << ptuData.ptuImagePoses[i].tiltTicks << std::endl;

        numCam1Images++;
 	}

 	// ceres problem aufbauen
 	{
 		auto poseInverse=[](const Eigen::Matrix<double,7,1>& pose)
	    {
	    	Eigen::Matrix<double,7,1> ret;
	    	Eigen::Quaterniond q;
	    	q.w()=pose(3);
	    	q.x()=pose(4);
	    	q.y()=pose(5);
	    	q.z()=pose(6);
	    	ret.segment<3>(0)=-q.toRotationMatrix().transpose()*pose.segment<3>(0);
	    	ret(3)=pose(3);
	    	ret.segment<3>(4)=-pose.segment<3>(4);
	    	return ret;
	    };
	    auto poseAdd=[](const Eigen::Matrix<double,7,1>& x, const Eigen::Matrix<double,7,1>& d) -> Eigen::Matrix<double,7,1>
	    {
	    	Eigen::Quaterniond qx(x(3),x(4),x(5),x(6));
	    	Eigen::Quaterniond qd(d(3),d(4),d(5),d(6));
	    	Eigen::Quaterniond qres=qx*qd;
	    	Eigen::Matrix<double,7,1> ret;
	    	ret.segment<3>(0)=x.segment<3>(0)+qx.toRotationMatrix()*d.segment<3>(0);
	    	ret(3)=qres.w();	ret(4)=qres.x();	ret(5)=qres.y();	ret(6)=qres.z();
	    	return ret;
	    };


#define FULLPROBLEM
#define ANGLEOFFSET

	    ceres::Problem markerBAProblem;

        ceres::Problem markerBAProblem_RepError;

 		Eigen::Matrix<double,7,1> panPoseInv;
 		panPoseInv << 0,0,0,1,0,0,0;

        std::vector<double> alphas(numCam1Images);
#ifdef FULLPROBLEM
        Eigen::Matrix<double, 7,1> tiltPoseInv;
        tiltPoseInv << 0,0,0,1,0,0,0;

        std::vector<double> betas(numCam1Images);
        Eigen::Matrix<double,7,1> camPose;
#else
 		std::vector<Eigen::Matrix<double,7,1> > camPoses;
 		camPoses.resize(distinctTiltTicks.size());
#endif

        
        std::vector<int> yzconstant_params={1,2};
        auto yzconstant_parametrization = new ceres::SubsetParameterization(3, yzconstant_params);
        auto yzconstant_parametrization2 = new ceres::SubsetParameterization(3, yzconstant_params);

        std::vector<int> constant_params={2};
        auto zconstant_parametrization = new ceres::SubsetParameterization(3, constant_params);
        auto zconstant_parametrization2 = new ceres::SubsetParameterization(3, constant_params);

        std::vector<int> yconstant_params={1,2};
        auto yconstant_parametrization = nullptr;//new ceres::SubsetParameterization(3, yconstant_params);
        auto yconstant_parametrization2 = nullptr;//new ceres::SubsetParameterization(3, yconstant_params);


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
        markerBAProblem_RepError.AddParameterBlock(&tiltPoseInv(3), 4, quaternion_parameterization2);
#endif


        int curImage=0;
        for (size_t i=0;i<ptuData.ptuImagePoses.size();i++)
 		{
 			if (ptuData.ptuImagePoses[i].cameraId!=1)
 				continue;

 			alphas[curImage]=ptuData.ptuImagePoses[i].panAngle;
            //std::cout << ptuData.ptuImagePoses[i].panAngle << std::endl;
 			markerBAProblem.AddParameterBlock(&alphas[curImage], 1);
            markerBAProblem.SetParameterBlockConstant(&alphas[curImage]);

            markerBAProblem_RepError.AddParameterBlock(&alphas[curImage], 1);
#ifndef ANGLEOFFSET
            markerBAProblem_RepError.SetParameterBlockConstant(&alphas[curImage]);
#else
            auto* alphaPrior = GaussianPrior1D::Create(alphas[curImage], 0.05/180.0*M_PI); 
            markerBAProblem_RepError.AddResidualBlock(alphaPrior, nullptr, &alphas[curImage]);
#endif
#ifdef FULLPROBLEM
            betas[curImage]=ptuData.ptuImagePoses[i].tiltAngle;
            markerBAProblem.AddParameterBlock(&betas[curImage], 1);
            markerBAProblem.SetParameterBlockConstant(&betas[curImage]);

            markerBAProblem_RepError.AddParameterBlock(&betas[curImage], 1);
#ifndef ANGLEOFFSET
            markerBAProblem_RepError.SetParameterBlockConstant(&betas[curImage]);
#else
            auto* betaPrior = GaussianPrior1D::Create(betas[curImage], 0.05/180.0*M_PI); 
            markerBAProblem_RepError.AddResidualBlock(betaPrior, nullptr, &betas[curImage]);
#endif
#endif

 			curImage++;
        }


#ifdef FULLPROBLEM
        camPose << 0,0,0,1,0,0,0;
        markerBAProblem.AddParameterBlock(&camPose(0), 3, yconstant_parametrization);
        markerBAProblem.AddParameterBlock(&camPose(3), 4, quaternion_parameterization);

        markerBAProblem_RepError.AddParameterBlock(&camPose(0), 3, yconstant_parametrization2);
        markerBAProblem_RepError.AddParameterBlock(&camPose(3), 4, quaternion_parameterization2);
#else
        for (int i=0;i<distinctTiltTicks.size();i++)
        {
        	camPoses[i] << 0,0,0,1,0,0,0;
        	markerBAProblem.AddParameterBlock(&camPoses[i](0), 3);
            markerBAProblem.AddParameterBlock(&camPoses[i](3), 4, quaternion_parameterization);
        }
#endif

        curImage=0;
       	for (size_t i=0;i<ptuData.ptuImagePoses.size();i++)
	 	{
 			if (ptuData.ptuImagePoses[i].cameraId!=1)
 				continue;

 			Eigen::Matrix<double,7,1> camPose_origin;
 			camPose_origin.segment<3>(0)=reconstructedPoses[i].t;
 			camPose_origin.segment<4>(3)=reconstructedPoses[i].q;
 			camPose_origin=poseInverse(camPose_origin); // cam to world
 			std::cout << "CamPose_Origin : " << camPose_origin.transpose() << std::endl;

#ifdef FULLPROBLEM
            constexpr bool robustify = true;
            auto* currentCostFunc = PTUPoseErrorTilt::Create(camPose_origin);
            markerBAProblem.AddResidualBlock(currentCostFunc,
                                             robustify?new ceres::HuberLoss(1.0):nullptr,//new ceres::CauchyLoss(3),
                                             &panPoseInv(0), &panPoseInv(3), &alphas[curImage],
                                             &tiltPoseInv(0), &tiltPoseInv(3), &betas[curImage],
                                             &camPose(0), &camPose(3));

            size_t detectedImageId=-1;
            for (size_t j=0;j<ptuDetectionResult.images.size();j++)
            {
                if (strstr(ptuData.ptuImagePoses[i].imagePath.c_str(),ptuDetectionResult.images[j].filename.c_str()))
                {
                    detectedImageId=j;
                    break;
                }
            }

            const camSurv::CameraModel& camModel = ptuData.cameraModelById[ptuData.ptuImagePoses[i].cameraId];

            for (const auto& tagObs : ptuDetectionResult.tagObservations)
            {
                if (tagObs.imageId!=detectedImageId)
                    continue;
                const auto tagIt = reconstructedTags.find(tagObs.tagId);
                if (tagIt == reconstructedTags.end())
                    continue;

                const std::vector<Eigen::Vector3d> tagCorners = tagIt->second.computeMarkerCorners3D();
                for (int c=0;c<4;c++)
                {
                    // check origin pose
                    // {
                    //     OpenCVReprojectionError repErr(tagObs.corners[c], camModel.distortionCoefficients,camModel.getK());
                    //     repErr.print=true;
                    //     double res[2];
                    //     repErr(&camPose_origin(0), &camPose_origin(3), &tagCorners[c](0), res);
                    // }
                    //

                    auto* currentCostFunc2 = PTUPoseErrorTiltRepError::Create(tagObs.corners[c],tagCorners[c],camModel.distortionCoefficients,camModel.getK());
                    markerBAProblem_RepError.AddResidualBlock(currentCostFunc2,
                                                 robustify?new ceres::HuberLoss(1.0):nullptr,//new ceres::CauchyLoss(3),
                                                 &panPoseInv(0), &panPoseInv(3), &alphas[curImage],
                                                 &tiltPoseInv(0), &tiltPoseInv(3), &betas[curImage],
                                                 &camPose(0), &camPose(3));
                }
            }
#else
 			auto* currentCostFunc = PTUPoseError::Create(camPose_origin);

 			bool robustify = true;

 			int curTickIndex = distinctTiltTicks[ptuData.ptuImagePoses[i].tiltTicks];

            markerBAProblem.AddResidualBlock(currentCostFunc,
                                             robustify?new ceres::HuberLoss(1.0):nullptr,//new ceres::CauchyLoss(3),
                                             &panPose(0), &panPose(3), &alphas[curImage],
                                             &camPoses[curTickIndex](0), &camPoses[curTickIndex](3));
#endif
            curImage++;
            //break;
		}
		std::cout << "done creating problem " << std::endl;

		ceres::Solver::Options options;
	// #if (CERES_VERSION_MAJOR==1)&&(CERES_VERSION_MINOR==11)
	//     options.linear_solver_ordering.reset(ordering);
	// #else
	//     options.linear_solver_ordering=ordering;
	// #endif
	    options.minimizer_progress_to_stdout = false;
	    options.max_num_iterations = 100;//maxNumIterations;
	    options.num_threads=4;//ceresThreads;
	    options.num_linear_solver_threads = 4;//ceresThreads;
	    //options.eta = 1e-2;

	    ceres::Solver::Summary summary;
	    Solve(options, &markerBAProblem, &summary);
        std::cout << "Solution " << summary.termination_type << std::endl;

        ceres::Solver::Summary summary2;
        Solve(options, &markerBAProblem_RepError, &summary2);
        std::cout << "Solution2 " << summary2.termination_type << std::endl;
	    //if (printSummary)
	    std::cout << summary2.FullReport() << std::endl;

        panPoseInv(0)=fabs(panPoseInv(0));
        tiltPoseInv(0)=fabs(tiltPoseInv(0));

	    std::cout << "PanPose: " << poseInverse(panPoseInv).transpose() << std::endl;
        std::cout << "TiltPose: " << poseInverse(tiltPoseInv).transpose() << std::endl;

		std::cout << "Alphas: ";
	    for (int i=0;i<alphas.size();i++)
	    	std::cout << alphas[i] << ", ";
	    std::cout << std::endl;

#ifdef FULLPROBLEM
        std::cout << "CamPose: " << std::endl;
        std::cout << camPose.transpose() << std::endl;
#else
	    std::cout << "CamPoses: " << std::endl;
	    for (int i=0;i<camPoses.size();i++)
	    	std::cout << camPoses[i].transpose() << std::endl;
#endif

	    //auto invPanPose=poseInverse(panPose);

        // Render Pan Pose
        {
            DebugVis dbg;
            dbg.cam.q=panPoseInv.segment<4>(3);
            dbg.cam.t=panPoseInv.segment<3>(0);
            debugVis.push_back(dbg);
        }
#ifdef FULLPROBLEM
        // Render Tilt Pose
        {
            auto invp=poseInverse(poseAdd(poseInverse(panPoseInv),poseInverse(tiltPoseInv)));

            DebugVis dbg;
            dbg.cam.q=invp.segment<4>(3);
            dbg.cam.t=invp.segment<3>(0);
            debugVis.push_back(dbg);
        }

        std::ofstream dbgAngleFile("dbgAngles.txt");
        std::vector<double> reprojectionErrors;
        curImage=0;
        double meanRepErrorSqr=0.0;
        int numRepErrors=0;
        for (size_t i=0;i<ptuData.ptuImagePoses.size();i++)
        {
            if (ptuData.ptuImagePoses[i].cameraId!=1)
                continue;
            double p=ptuData.ptuImagePoses[i].panAngle;
            Eigen::Matrix<double,7,1> panRot;
            panRot << 0,0,0,cos(p/2.0),0,0,sin(p/2.0);

            double t=ptuData.ptuImagePoses[i].tiltAngle;
            Eigen::Matrix<double,7,1> tiltRot;
            tiltRot << 0,0,0,cos(t/2.0),0,0,sin(t/2.0);

            dbgAngleFile << ptuData.ptuImagePoses[i].panAngle << " " << ptuData.ptuImagePoses[i].panAngle-alphas[curImage] << " " <<  ptuData.ptuImagePoses[i].tiltAngle-betas[curImage] << std::endl;

            auto invp=poseInverse(poseAdd(poseAdd(poseAdd(poseAdd(poseInverse(panPoseInv),panRot),poseInverse(tiltPoseInv)),tiltRot),camPose));

            DebugVis dbg;
            dbg.cam.q=invp.segment<4>(3);
            dbg.cam.t=invp.segment<3>(0);
            debugVis.push_back(dbg);



            // haesslich!

            size_t detectedImageId=-1;
            for (size_t j=0;j<ptuDetectionResult.images.size();j++)
            {
                if (strstr(ptuData.ptuImagePoses[i].imagePath.c_str(),ptuDetectionResult.images[j].filename.c_str()))
                {
                    detectedImageId=j;
                    break;
                }
            }

            const camSurv::CameraModel& camModel = ptuData.cameraModelById[ptuData.ptuImagePoses[i].cameraId];

            for (const auto& tagObs : ptuDetectionResult.tagObservations)
            {
                if (tagObs.imageId!=detectedImageId)
                    continue;
                const auto tagIt = reconstructedTags.find(tagObs.tagId);
                if (tagIt == reconstructedTags.end())
                    continue;

                const std::vector<Eigen::Vector3d> tagCorners = tagIt->second.computeMarkerCorners3D();

                for (int c=0;c<4;c++)
                {
                    PTUPoseErrorTiltRepError currentCostFunc2(tagObs.corners[c],tagCorners[c],camModel.distortionCoefficients,camModel.getK());
                    currentCostFunc2.repError.print=false;

                    Eigen::Vector2d repError;

                    currentCostFunc2(&panPoseInv(0), &panPoseInv(3), &alphas[curImage],
                                     &tiltPoseInv(0), &tiltPoseInv(3), &betas[curImage],
                                     &camPose(0), &camPose(3), &repError(0));

                    //std::cout <<  "Rep Error: " << repError.norm() << std::endl;
                    meanRepErrorSqr+=repError.squaredNorm();
                    numRepErrors++;

                    reprojectionErrors.push_back(repError.norm());
                }
            }
            curImage++;
        }
        meanRepErrorSqr/=numRepErrors;
        meanRepErrorSqr=sqrt(meanRepErrorSqr);
        std::cout << "RMS Rep Error: " << meanRepErrorSqr << std::endl;

        // Dump Rep Errors
        {
            std::ofstream myfile(folder + "/reprojectionErrors.txt");
            for (const auto& repError : reprojectionErrors)
                myfile << repError << "\n";
        }

#else
        for (int i=0;i<camPoses.size();i++)
        {
        	auto ret=poseAdd(poseInverse(panPoseInv),camPoses[i]);
        	auto invRet=poseInverse(ret);
        	DebugVis dbg;
	        dbg.cam.q=invRet.segment<4>(3);
	        dbg.cam.t=invRet.segment<3>(0);
	        debugVis.push_back(dbg);
        }
#endif
    }
#endif
}
//-----------------------------------------------------------------------------
bool PtuCalibrationProject::computeRelativeCameraPoseFromImg(int cameraId, int imageId,
    const Eigen::Matrix3d& K, const Eigen::Matrix<double, 5, 1>& distCoefficients,
    Eigen::Quaterniond& q, Eigen::Vector3d& t)
{
    std::vector<Eigen::Vector3d> markerCorners3D;
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > observations2D;
    // find all matches between this image and the reconstructions
    for (const auto& tagObs : ptuDetectionResults[cameraId].tagObservations)
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
    visual_marker_mapping::solvePnPRansacEigen(
        markerCorners3D, observations2D, K, distCoefficients, R, t);

    q = Eigen::Quaterniond(R);
    return true;
}
//-----------------------------------------------------------------------------
}
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
