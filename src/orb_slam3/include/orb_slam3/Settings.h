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

#ifndef ORB_SLAM3_SETTINGS_H
#define ORB_SLAM3_SETTINGS_H


// Flag to activate the measurement of time in each process (track,localmap, place recognition).
//#define REGISTER_TIMES

#include "CameraModels/GeometricCamera.h"

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>

namespace ORB_SLAM3 {

    class System;

    class Settings {
    public:
        /*
         * Enum for the different camera types implemented
         */
        enum CameraType {
            PinHole = 0,
            Rectified = 1,
            KannalaBrandt = 2
        };

        /*
         * Default constructor — initializes all members to sensible defaults.
         * Call setters then initialize() before use.
         */
        Settings();

        /*
         * Must be called after all setters, before passing to System.
         * Creates GeometricCamera objects and precomputes rectification maps.
         */
        void initialize();

        /*
         * Ostream operator overloading to dump settings to the terminal
         */
        friend std::ostream &operator<<(std::ostream &output, const Settings &s);

        /*
         * Getter methods
         */
        CameraType cameraType() {return cameraType_;}
        GeometricCamera* camera1() {return calibration1_;}
        GeometricCamera* camera2() {return calibration2_;}
        cv::Mat camera1DistortionCoef() {return cv::Mat(vPinHoleDistorsion1_.size(),1,CV_32F,vPinHoleDistorsion1_.data());}
        cv::Mat camera2DistortionCoef() {return cv::Mat(vPinHoleDistorsion2_.size(),1,CV_32F,vPinHoleDistorsion1_.data());}

        Sophus::SE3f Tlr() {return Tlr_;}
        float bf() {return bf_;}
        float b() {return b_;}
        float thDepth() {return thDepth_;}

        bool needToUndistort() {return bNeedToUndistort_;}

        cv::Size newImSize() {return newImSize_;}
        float fps() {return fps_;}
        bool rgb() {return bRGB_;}
        bool needToResize() {return bNeedToResize1_;}
        bool needToRectify() {return bNeedToRectify_;}

        float noiseGyro() {return noiseGyro_;}
        float noiseAcc() {return noiseAcc_;}
        float gyroWalk() {return gyroWalk_;}
        float accWalk() {return accWalk_;}
        float imuFrequency() {return imuFrequency_;}
        Sophus::SE3f Tbc() {return Tbc_;}
        bool insertKFsWhenLost() {return insertKFsWhenLost_;}

        float depthMapFactor() {return depthMapFactor_;}

        int nFeatures() {return nFeatures_;}
        int nLevels() {return nLevels_;}
        float initThFAST() {return initThFAST_;}
        float minThFAST() {return minThFAST_;}
        float scaleFactor() {return scaleFactor_;}

        std::string atlasLoadFile() {return sLoadFrom_;}
        std::string atlasSaveFile() {return sSaveto_;}

        float thFarPoints() {return thFarPoints_;}
        bool activeLoopClosing() {return activeLoopClosing_;}

        cv::Mat M1l() {return M1l_;}
        cv::Mat M2l() {return M2l_;}
        cv::Mat M1r() {return M1r_;}
        cv::Mat M2r() {return M2r_;}

        // --- Setters ----------------------------------------------------------

        void setSensor(int sensor)               { sensor_ = sensor; }
        void setCameraType(CameraType t)         { cameraType_ = t; }

        void setCamera1Intrinsics(float fx, float fy, float cx, float cy);
        void setCamera1Distortion(const std::vector<float>& d);
        void setCamera1KB(float k0, float k1, float k2, float k3, int overlapBegin, int overlapEnd);

        void setCamera2Intrinsics(float fx, float fy, float cx, float cy);
        void setCamera2Distortion(const std::vector<float>& d);
        void setCamera2KB(float k0, float k1, float k2, float k3, int overlapBegin, int overlapEnd);

        void setTlr(const Sophus::SE3f& Tlr)     { Tlr_ = Tlr; }
        void setStereoParams(float baseline, float thDepth);

        void setImageInfo(int width, int height, int fps, bool rgb);
        void setResize(int newWidth, int newHeight);

        void setIMUParams(float noiseGyro, float noiseAcc, float gyroWalk, float accWalk,
                          float frequency, const Sophus::SE3f& Tbc, bool insertKFsWhenLost);

        void setRGBDParams(float depthMapFactor);

        void setORBParams(int nFeatures, float scaleFactor, int nLevels, int iniThFAST, int minThFAST);

        void setAtlasFiles(const std::string& load, const std::string& save)
            { sLoadFrom_ = load; sSaveto_ = save; }

        void setThFarPoints(float v)           { thFarPoints_ = v; }
        void setActiveLoopClosing(bool v)      { activeLoopClosing_ = v; }

    private:
        void precomputeRectificationMaps();

        int sensor_;
        CameraType cameraType_;

        // Raw camera parameters (set by setters, consumed by initialize())
        float fx_, fy_, cx_, cy_;
        float fx2_, fy2_, cx2_, cy2_;
        float kb_k0_, kb_k1_, kb_k2_, kb_k3_;
        float kb2_k0_, kb2_k1_, kb2_k2_, kb2_k3_;
        int overlappingBegin_, overlappingEnd_;
        int overlappingBegin2_, overlappingEnd2_;

        /*
         * Visual stuff
         */
        GeometricCamera* calibration1_, *calibration2_;
        GeometricCamera* originalCalib1_, *originalCalib2_;
        std::vector<float> vPinHoleDistorsion1_, vPinHoleDistorsion2_;

        cv::Size originalImSize_, newImSize_;
        float fps_;
        bool bRGB_;

        bool bNeedToUndistort_;
        bool bNeedToRectify_;
        bool bNeedToResize1_, bNeedToResize2_;

        Sophus::SE3f Tlr_;
        float thDepth_;
        float bf_, b_;

        /*
         * Rectification stuff
         */
        cv::Mat M1l_, M2l_;
        cv::Mat M1r_, M2r_;

        /*
         * Inertial stuff
         */
        float noiseGyro_, noiseAcc_;
        float gyroWalk_, accWalk_;
        float imuFrequency_;
        Sophus::SE3f Tbc_;
        bool insertKFsWhenLost_;

        /*
         * RGBD stuff
         */
        float depthMapFactor_;

        /*
         * ORB stuff
         */
        int nFeatures_;
        float scaleFactor_;
        int nLevels_;
        int initThFAST_, minThFAST_;

        /*
         * Save & load maps
         */
        std::string sLoadFrom_, sSaveto_;

        /*
         * Other stuff
         */
        float thFarPoints_;
        bool activeLoopClosing_;
    };
};


#endif //ORB_SLAM3_SETTINGS_H
