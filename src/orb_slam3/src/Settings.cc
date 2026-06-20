/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include "Settings.h"

#include "CameraModels/Pinhole.h"
#include "CameraModels/KannalaBrandt8.h"

#include "System.h"

#include <opencv2/core/eigen.hpp>

#include <iostream>

using namespace std;

namespace ORB_SLAM3 {

Settings::Settings() :
    sensor_(0),
    cameraType_(PinHole),
    fx_(0), fy_(0), cx_(0), cy_(0),
    fx2_(0), fy2_(0), cx2_(0), cy2_(0),
    kb_k0_(0), kb_k1_(0), kb_k2_(0), kb_k3_(0),
    kb2_k0_(0), kb2_k1_(0), kb2_k2_(0), kb2_k3_(0),
    overlappingBegin_(0), overlappingEnd_(0),
    overlappingBegin2_(0), overlappingEnd2_(0),
    calibration1_(nullptr), calibration2_(nullptr),
    originalCalib1_(nullptr), originalCalib2_(nullptr),
    originalImSize_(0,0), newImSize_(0,0),
    fps_(30),
    bRGB_(true),
    bNeedToUndistort_(false), bNeedToRectify_(false),
    bNeedToResize1_(false), bNeedToResize2_(false),
    thDepth_(0),
    bf_(0), b_(0),
    noiseGyro_(0), noiseAcc_(0),
    gyroWalk_(0), accWalk_(0),
    imuFrequency_(200),
    insertKFsWhenLost_(true),
    depthMapFactor_(1.0f),
    nFeatures_(1000),
    scaleFactor_(1.2f),
    nLevels_(8),
    initThFAST_(20), minThFAST_(7),
    thFarPoints_(0),
    activeLoopClosing_(true)
{
}

// --- Setters ----------------------------------------------------------------

void Settings::setCamera1Intrinsics(float fx, float fy, float cx, float cy) {
    fx_ = fx; fy_ = fy; cx_ = cx; cy_ = cy;
}

void Settings::setCamera1Distortion(const std::vector<float>& d) {
    vPinHoleDistorsion1_ = d;
}

void Settings::setCamera1KB(float k0, float k1, float k2, float k3,
                            int overlapBegin, int overlapEnd) {
    kb_k0_ = k0; kb_k1_ = k1; kb_k2_ = k2; kb_k3_ = k3;
    overlappingBegin_ = overlapBegin;
    overlappingEnd_ = overlapEnd;
}

void Settings::setCamera2Intrinsics(float fx, float fy, float cx, float cy) {
    fx2_ = fx; fy2_ = fy; cx2_ = cx; cy2_ = cy;
}

void Settings::setCamera2Distortion(const std::vector<float>& d) {
    vPinHoleDistorsion2_ = d;
}

void Settings::setCamera2KB(float k0, float k1, float k2, float k3,
                            int overlapBegin, int overlapEnd) {
    kb2_k0_ = k0; kb2_k1_ = k1; kb2_k2_ = k2; kb2_k3_ = k3;
    overlappingBegin2_ = overlapBegin;
    overlappingEnd2_ = overlapEnd;
}

void Settings::setStereoParams(float baseline, float thDepth) {
    b_ = baseline;
    thDepth_ = thDepth;
}

void Settings::setImageInfo(int width, int height, int fps, bool rgb) {
    originalImSize_.width = width;
    originalImSize_.height = height;
    newImSize_ = originalImSize_;
    fps_ = fps;
    bRGB_ = rgb;
}

void Settings::setResize(int newWidth, int newHeight) {
    if (newHeight > 0) {
        bNeedToResize1_ = true;
        newImSize_.height = newHeight;
    }
    if (newWidth > 0) {
        bNeedToResize1_ = true;
        newImSize_.width = newWidth;
    }
}

void Settings::setIMUParams(float noiseGyro, float noiseAcc, float gyroWalk, float accWalk,
                            float frequency, const Sophus::SE3f& Tbc, bool insertKFsWhenLost) {
    noiseGyro_ = noiseGyro;
    noiseAcc_ = noiseAcc;
    gyroWalk_ = gyroWalk;
    accWalk_ = accWalk;
    imuFrequency_ = frequency;
    Tbc_ = Tbc;
    insertKFsWhenLost_ = insertKFsWhenLost;
}

void Settings::setRGBDParams(float depthMapFactor) {
    depthMapFactor_ = depthMapFactor;
}

void Settings::setORBParams(int nFeatures, float scaleFactor, int nLevels,
                            int iniThFAST, int minThFAST) {
    nFeatures_ = nFeatures;
    scaleFactor_ = scaleFactor;
    nLevels_ = nLevels;
    initThFAST_ = iniThFAST;
    minThFAST_ = minThFAST;
}

// --- initialize() -----------------------------------------------------------

void Settings::initialize() {
    vector<float> vCalibration;

    // --- Camera 1 ---
    if (cameraType_ == PinHole) {
        vCalibration = {fx_, fy_, cx_, cy_};
        calibration1_ = new Pinhole(vCalibration);
        originalCalib1_ = new Pinhole(vCalibration);

        if ((sensor_ == System::MONOCULAR || sensor_ == System::IMU_MONOCULAR)
            && !vPinHoleDistorsion1_.empty()) {
            bNeedToUndistort_ = true;
        }
    } else if (cameraType_ == Rectified) {
        vCalibration = {fx_, fy_, cx_, cy_};
        calibration1_ = new Pinhole(vCalibration);
        originalCalib1_ = new Pinhole(vCalibration);
    } else if (cameraType_ == KannalaBrandt) {
        vCalibration = {fx_, fy_, cx_, cy_, kb_k0_, kb_k1_, kb_k2_, kb_k3_};
        calibration1_ = new KannalaBrandt8(vCalibration);
        originalCalib1_ = new KannalaBrandt8(vCalibration);

        if (sensor_ == System::STEREO || sensor_ == System::IMU_STEREO) {
            static_cast<KannalaBrandt8*>(calibration1_)->mvLappingArea =
                {overlappingBegin_, overlappingEnd_};
        }
    } else {
        cerr << "Error: unknown camera type " << cameraType_ << endl;
        exit(-1);
    }

    // --- Camera 2 (stereo only) ---
    if (sensor_ == System::STEREO || sensor_ == System::IMU_STEREO) {
        if (cameraType_ == PinHole) {
            bNeedToRectify_ = true;
            vCalibration = {fx2_, fy2_, cx2_, cy2_};
            calibration2_ = new Pinhole(vCalibration);
            originalCalib2_ = new Pinhole(vCalibration);
        } else if (cameraType_ == KannalaBrandt) {
            vCalibration = {fx2_, fy2_, cx2_, cy2_, kb2_k0_, kb2_k1_, kb2_k2_, kb2_k3_};
            calibration2_ = new KannalaBrandt8(vCalibration);
            originalCalib2_ = new KannalaBrandt8(vCalibration);
            static_cast<KannalaBrandt8*>(calibration2_)->mvLappingArea =
                {overlappingBegin2_, overlappingEnd2_};
        } else if (cameraType_ == Rectified) {
            vCalibration = {fx2_, fy2_, cx2_, cy2_};
            calibration2_ = new Pinhole(vCalibration);
            originalCalib2_ = new Pinhole(vCalibration);
        }

        if (cameraType_ != Rectified) {
            b_ = Tlr_.translation().norm();
        }
        bf_ = b_ * calibration1_->getParameter(0);
    }

    // RGB-D (non-stereo): compute bf from baseline
    if ((sensor_ == System::RGBD || sensor_ == System::IMU_RGBD)
        && sensor_ != System::IMU_STEREO) {
        bf_ = b_ * calibration1_->getParameter(0);
    }

    // --- Handle resize (must happen after camera creation) ---
    if (bNeedToResize1_ && !bNeedToRectify_) {
        const bool isStereo = (sensor_ == System::STEREO || sensor_ == System::IMU_STEREO);

        if (newImSize_.height != originalImSize_.height) {
            float scaleRowFactor = (float)newImSize_.height / (float)originalImSize_.height;
            calibration1_->setParameter(calibration1_->getParameter(1) * scaleRowFactor, 1);
            calibration1_->setParameter(calibration1_->getParameter(3) * scaleRowFactor, 3);

            if (isStereo && cameraType_ != Rectified) {
                calibration2_->setParameter(calibration2_->getParameter(1) * scaleRowFactor, 1);
                calibration2_->setParameter(calibration2_->getParameter(3) * scaleRowFactor, 3);
            }
        }

        if (newImSize_.width != originalImSize_.width) {
            float scaleColFactor = (float)newImSize_.width / (float)originalImSize_.width;
            calibration1_->setParameter(calibration1_->getParameter(0) * scaleColFactor, 0);
            calibration1_->setParameter(calibration1_->getParameter(2) * scaleColFactor, 2);

            if (isStereo && cameraType_ != Rectified) {
                calibration2_->setParameter(calibration2_->getParameter(0) * scaleColFactor, 0);
                calibration2_->setParameter(calibration2_->getParameter(2) * scaleColFactor, 2);

                if (cameraType_ == KannalaBrandt) {
                    static_cast<KannalaBrandt8*>(calibration1_)->mvLappingArea[0] *= scaleColFactor;
                    static_cast<KannalaBrandt8*>(calibration1_)->mvLappingArea[1] *= scaleColFactor;
                    static_cast<KannalaBrandt8*>(calibration2_)->mvLappingArea[0] *= scaleColFactor;
                    static_cast<KannalaBrandt8*>(calibration2_)->mvLappingArea[1] *= scaleColFactor;
                }
            }
        }
    }

    // --- Precompute rectification maps ---
    if (bNeedToRectify_) {
        precomputeRectificationMaps();
    }

    cout << "Settings initialized from parameter setters." << endl;
    cout << "----------------------------------" << endl;
}

// --- precomputeRectificationMaps (unchanged logic) --------------------------

void Settings::precomputeRectificationMaps() {
    cv::Mat K1 = static_cast<Pinhole*>(calibration1_)->toK();
    K1.convertTo(K1,CV_64F);
    cv::Mat K2 = static_cast<Pinhole*>(calibration2_)->toK();
    K2.convertTo(K2,CV_64F);

    cv::Mat cvTlr;
    cv::eigen2cv(Tlr_.inverse().matrix3x4(),cvTlr);
    cv::Mat R12 = cvTlr.rowRange(0,3).colRange(0,3);
    R12.convertTo(R12,CV_64F);
    cv::Mat t12 = cvTlr.rowRange(0,3).col(3);
    t12.convertTo(t12,CV_64F);

    cv::Mat R_r1_u1, R_r2_u2;
    cv::Mat P1, P2, Q;

    cv::stereoRectify(K1,camera1DistortionCoef(),K2,camera2DistortionCoef(),newImSize_,
                      R12, t12,
                      R_r1_u1,R_r2_u2,P1,P2,Q,
                      cv::CALIB_ZERO_DISPARITY,-1,newImSize_);
    cv::initUndistortRectifyMap(K1, camera1DistortionCoef(), R_r1_u1, P1.rowRange(0, 3).colRange(0, 3),
                                newImSize_, CV_32F, M1l_, M2l_);
    cv::initUndistortRectifyMap(K2, camera2DistortionCoef(), R_r2_u2, P2.rowRange(0, 3).colRange(0, 3),
                                newImSize_, CV_32F, M1r_, M2r_);

    //Update calibration
    calibration1_->setParameter(P1.at<double>(0,0), 0);
    calibration1_->setParameter(P1.at<double>(1,1), 1);
    calibration1_->setParameter(P1.at<double>(0,2), 2);
    calibration1_->setParameter(P1.at<double>(1,2), 3);

    //Update bf
    bf_ = b_ * P1.at<double>(0,0);

    //Update relative pose between camera 1 and IMU if necessary
    if(sensor_ == System::IMU_STEREO){
        Eigen::Matrix3f eigenR_r1_u1;
        cv::cv2eigen(R_r1_u1,eigenR_r1_u1);
        Sophus::SE3f T_r1_u1(eigenR_r1_u1,Eigen::Vector3f::Zero());
        Tbc_ = Tbc_ * T_r1_u1.inverse();
    }
}

ostream &operator<<(std::ostream& output, const Settings& settings){
    output << "SLAM settings: " << endl;

    output << "\t-Camera 1 parameters (";
    if(settings.cameraType_ == Settings::PinHole || settings.cameraType_ ==  Settings::Rectified){
        output << "Pinhole";
    }
    else{
        output << "Kannala-Brandt";
    }
    output << ")" << ": [";
    for(size_t i = 0; i < settings.originalCalib1_->size(); i++){
        output << " " << settings.originalCalib1_->getParameter(i);
    }
    output << " ]" << endl;

    if(!settings.vPinHoleDistorsion1_.empty()){
        output << "\t-Camera 1 distortion parameters: [ ";
        for(float d : settings.vPinHoleDistorsion1_){
            output << " " << d;
        }
        output << " ]" << endl;
    }

    if(settings.sensor_ == System::STEREO || settings.sensor_ == System::IMU_STEREO){
        output << "\t-Camera 2 parameters (";
        if(settings.cameraType_ == Settings::PinHole || settings.cameraType_ ==  Settings::Rectified){
            output << "Pinhole";
        }
        else{
            output << "Kannala-Brandt";
        }
        output << "" << ": [";
        for(size_t i = 0; i < settings.originalCalib2_->size(); i++){
            output << " " << settings.originalCalib2_->getParameter(i);
        }
        output << " ]" << endl;

        if(!settings.vPinHoleDistorsion2_.empty()){
            output << "\t-Camera 1 distortion parameters: [ ";
            for(float d : settings.vPinHoleDistorsion2_){
                output << " " << d;
            }
            output << " ]" << endl;
        }
    }

    output << "\t-Original image size: [ " << settings.originalImSize_.width << " , " << settings.originalImSize_.height << " ]" << endl;
    output << "\t-Current image size: [ " << settings.newImSize_.width << " , " << settings.newImSize_.height << " ]" << endl;

    if(settings.bNeedToRectify_){
        output << "\t-Camera 1 parameters after rectification: [ ";
        for(size_t i = 0; i < settings.calibration1_->size(); i++){
            output << " " << settings.calibration1_->getParameter(i);
        }
        output << " ]" << endl;
    }
    else if(settings.bNeedToResize1_){
        output << "\t-Camera 1 parameters after resize: [ ";
        for(size_t i = 0; i < settings.calibration1_->size(); i++){
            output << " " << settings.calibration1_->getParameter(i);
        }
        output << " ]" << endl;

        if((settings.sensor_ == System::STEREO || settings.sensor_ == System::IMU_STEREO) &&
            settings.cameraType_ == Settings::KannalaBrandt){
            output << "\t-Camera 2 parameters after resize: [ ";
            for(size_t i = 0; i < settings.calibration2_->size(); i++){
                output << " " << settings.calibration2_->getParameter(i);
            }
            output << " ]" << endl;
        }
    }

    output << "\t-Sequence FPS: " << settings.fps_ << endl;

    //Stereo stuff
    if(settings.sensor_ == System::STEREO || settings.sensor_ == System::IMU_STEREO){
        output << "\t-Stereo baseline: " << settings.b_ << endl;
        output << "\t-Stereo depth threshold : " << settings.thDepth_ << endl;

        if(settings.cameraType_ == Settings::KannalaBrandt){
            auto vOverlapping1 = static_cast<KannalaBrandt8*>(settings.calibration1_)->mvLappingArea;
            auto vOverlapping2 = static_cast<KannalaBrandt8*>(settings.calibration2_)->mvLappingArea;
            output << "\t-Camera 1 overlapping area: [ " << vOverlapping1[0] << " , " << vOverlapping1[1] << " ]" << endl;
            output << "\t-Camera 2 overlapping area: [ " << vOverlapping2[0] << " , " << vOverlapping2[1] << " ]" << endl;
        }
    }

    if(settings.sensor_ == System::IMU_MONOCULAR || settings.sensor_ == System::IMU_STEREO || settings.sensor_ == System::IMU_RGBD) {
        output << "\t-Gyro noise: " << settings.noiseGyro_ << endl;
        output << "\t-Accelerometer noise: " << settings.noiseAcc_ << endl;
        output << "\t-Gyro walk: " << settings.gyroWalk_ << endl;
        output << "\t-Accelerometer walk: " << settings.accWalk_ << endl;
        output << "\t-IMU frequency: " << settings.imuFrequency_ << endl;
    }

    if(settings.sensor_ == System::RGBD || settings.sensor_ == System::IMU_RGBD){
        output << "\t-RGB-D depth map factor: " << settings.depthMapFactor_ << endl;
    }

    output << "\t-Features per image: " << settings.nFeatures_ << endl;
    output << "\t-ORB scale factor: " << settings.scaleFactor_ << endl;
    output << "\t-ORB number of scales: " << settings.nLevels_ << endl;
    output << "\t-Initial FAST threshold: " << settings.initThFAST_ << endl;
    output << "\t-Min FAST threshold: " << settings.minThFAST_ << endl;

    return output;
}
};
