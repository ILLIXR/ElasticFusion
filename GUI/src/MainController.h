/*
 * This file is part of ElasticFusion.
 *
 * Copyright (C) 2015 Imperial College London
 *
 * The use of the code within this file and all code within files that
 * make up the software that is ElasticFusion is permitted for
 * non-commercial purposes only.  The full terms and conditions that
 * apply to the code within this file are detailed within the LICENSE.txt
 * file and at <http://www.imperial.ac.uk/dyson-robotics-lab/downloads/elastic-fusion/elastic-fusion-license/>
 * unless explicitly stated.  By downloading this file you agree to
 * comply with these terms.
 *
 * If you wish to use any of this code for commercial purposes then
 * please email researchcontracts.engineering@imperial.ac.uk.
 *
 */

#include <ElasticFusion.h>
#include <Utils/Parse.h>

#include "Tools/GUI.h"
#include "Tools/GroundTruthOdometry.h"
#include "Tools/RawLogReader.h"
#include "Tools/LiveLogReader.h"

#include "common/switchboard.hpp"
#include "common/data_format.hpp"
#include "common/threadloop.hpp"

#ifndef MAINCONTROLLER_H_
#define MAINCONTROLLER_H_

class MainController : public ILLIXR::threadloop {

    public:
        MainController(const std::string& name_, phonebook* pb_);
        virtual ~MainController();

    protected:
        virtual void _p_one_iteration() override;

    private:
        std::shared_ptr<ILLIXR::switchboard> sb;
        std::unique_ptr<ILLIXR::reader_latest<rgb_depth_type>> _m_cam;


        void run();

        void loadCalibration(const std::string & filename);

        bool good;
        ElasticFusion * eFusion;
        GUI * gui;
        GroundTruthOdometry * groundTruthOdometry;
        LogReader * logReader;

        bool iclnuim;
        std::string logFile;
        std::string poseFile;

        float confidence,
              depth,
              icp,
              icpErrThresh,
              covThresh,
              photoThresh,
              fernThresh;

        int timeDelta,
            icpCountThresh,
            start,
            end;

        bool fillIn,
             openLoop,
             reloc,
             frameskip,
             quiet,
             fastOdom,
             so3,
             rewind,
             frameToFrameRGB;

        int framesToSkip;
        bool streaming;
        bool resetButton;

        Resize * resizeStream;

        std::chrono::time_point<std::chrono::system_clock> sync;
};

#endif /* MAINCONTROLLER_H_ */
