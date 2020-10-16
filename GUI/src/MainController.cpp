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
 
#include "MainController.h"

MainController::MainController(int argc, char * argv[])
 : good(true),
   eFusion(0),
   gui(0),
   groundTruthOdometry(0),
   logReader(0),
   framesToSkip(0),
   resetButton(false),
   resizeStream(0)
{
    std::string empty;
    iclnuim = Parse::get().arg(argc, argv, "-icl", empty) > -1;

    std::string calibrationFile;
    Parse::get().arg(argc, argv, "-cal", calibrationFile);

    Resolution::getInstance(640, 480);

    if(calibrationFile.length())
    {
        loadCalibration(calibrationFile);
    }
    else
    {
        Intrinsics::getInstance(528, 528, 320, 240);
    }

    Parse::get().arg(argc, argv, "-l", logFile);

    if(logFile.length())
    {
        logReader = new RawLogReader(logFile, Parse::get().arg(argc, argv, "-f", empty) > -1);
    }
    else
    {
        bool flipColors = Parse::get().arg(argc,argv,"-f",empty) > -1;
        logReader = new LiveLogReader(logFile, flipColors, LiveLogReader::CameraType::OpenNI2);

        good = ((LiveLogReader *)logReader)->cam->ok();

#ifdef WITH_REALSENSE
        if(!good)
        {
          delete logReader;
          logReader = new LiveLogReader(logFile, flipColors, LiveLogReader::CameraType::RealSense);

          good = ((LiveLogReader *)logReader)->cam->ok();
        }
#endif
    }

    if(Parse::get().arg(argc, argv, "-p", poseFile) > 0)
    {
        groundTruthOdometry = new GroundTruthOdometry(poseFile);
    }

    confidence = 10.0f;
    depth = 3.0f;
    icp = 10.0f;
    icpErrThresh = 5e-05;
    covThresh = 1e-05;
    photoThresh = 115;
    fernThresh = 0.3095f;

    timeDelta = 200;
    icpCountThresh = 40000;
    start = 1;
    so3 = !(Parse::get().arg(argc, argv, "-nso", empty) > -1);
    end = std::numeric_limits<unsigned short>::max(); //Funny bound, since we predict times in this format really!

    Parse::get().arg(argc, argv, "-c", confidence);
    Parse::get().arg(argc, argv, "-d", depth);
    Parse::get().arg(argc, argv, "-i", icp);
    Parse::get().arg(argc, argv, "-ie", icpErrThresh);
    Parse::get().arg(argc, argv, "-cv", covThresh);
    Parse::get().arg(argc, argv, "-pt", photoThresh);
    Parse::get().arg(argc, argv, "-ft", fernThresh);
    Parse::get().arg(argc, argv, "-t", timeDelta);
    Parse::get().arg(argc, argv, "-ic", icpCountThresh);
    Parse::get().arg(argc, argv, "-s", start);
    Parse::get().arg(argc, argv, "-e", end);

    logReader->flipColors = Parse::get().arg(argc, argv, "-f", empty) > -1;

    openLoop = !groundTruthOdometry && Parse::get().arg(argc, argv, "-o", empty) > -1;
    reloc = Parse::get().arg(argc, argv, "-rl", empty) > -1;
    frameskip = Parse::get().arg(argc, argv, "-fs", empty) > -1;
    quiet = Parse::get().arg(argc, argv, "-q", empty) > -1;
    fastOdom = Parse::get().arg(argc, argv, "-fo", empty) > -1;
    rewind = Parse::get().arg(argc, argv, "-r", empty) > -1;
    frameToFrameRGB = Parse::get().arg(argc, argv, "-ftf", empty) > -1;
    profile = Parse::get().arg(argc, argv, "-prof", empty) > - 1;

    gui = new GUI(logFile.length() == 0, Parse::get().arg(argc, argv, "-sc", empty) > -1);

    gui->flipColors->Ref().Set(logReader->flipColors);
    gui->rgbOnly->Ref().Set(false);
    gui->pyramid->Ref().Set(false);
    gui->fastOdom->Ref().Set(fastOdom);
    gui->confidenceThreshold->Ref().Set(confidence);
    gui->depthCutoff->Ref().Set(depth);
    gui->icpWeight->Ref().Set(icp);
    gui->so3->Ref().Set(so3);
    gui->frameToFrameRGB->Ref().Set(frameToFrameRGB);

    resizeStream = new Resize(Resolution::getInstance().width(),
                              Resolution::getInstance().height(),
                              Resolution::getInstance().width() / 2,
                              Resolution::getInstance().height() / 2);
}

MainController::~MainController()
{
    if(eFusion)
    {
        delete eFusion;
    }

    if(gui)
    {
        delete gui;
    }

    if(groundTruthOdometry)
    {
        delete groundTruthOdometry;
    }

    if(logReader)
    {
        delete logReader;
    }

    if(resizeStream)
    {
        delete resizeStream;
    }
}

void MainController::loadCalibration(const std::string & filename)
{
    std::ifstream file(filename);
    std::string line;

    assert(!file.eof());

    double fx, fy, cx, cy;

    std::getline(file, line);

    int n = sscanf(line.c_str(), "%lg %lg %lg %lg", &fx, &fy, &cx, &cy);

    assert(n == 4 && "Ooops, your calibration file should contain a single line with fx fy cx cy!");

    Intrinsics::getInstance(fx, fy, cx, cy);
}

void MainController::launch()
{
    while(good)
    {
        if(eFusion)
        {
            run();
        }

        if(eFusion == 0 || resetButton)
        {
            resetButton = false;

            if(eFusion)
            {
                delete eFusion;
            }

            logReader->rewind();
            eFusion = new ElasticFusion(openLoop ? std::numeric_limits<int>::max() / 2 : timeDelta,
                                        icpCountThresh,
                                        icpErrThresh,
                                        covThresh,
                                        !openLoop,
                                        iclnuim,
                                        reloc,
                                        photoThresh,
                                        confidence,
                                        depth,
                                        icp,
                                        fastOdom,
                                        fernThresh,
                                        so3,
                                        frameToFrameRGB,
                                        logReader->getFile());
        }
        else
        {
            break;
        }

    }
}

void MainController::run()
{
    while(!pangolin::ShouldQuit() && !((!logReader->hasMore()) && quiet) && !(eFusion->getTick() == end && quiet))
    {
        if(!gui->pause->Get() || pangolin::Pushed(*gui->step))
        {
            if((logReader->hasMore() || rewind) && eFusion->getTick() < end)
            {
                TICK("LogRead");
                if(rewind)
                {
                    if(!logReader->hasMore())
                    {
                        logReader->getBack();
                    }
                    else
                    {
                        logReader->getNext();
                    }

                    if(logReader->rewound())
                    {
                        logReader->currentFrame = 0;
                    }
                }
                else
                {
                    logReader->getNext();
                }
                TOCK("LogRead");

                if(eFusion->getTick() < start)
                {
                    eFusion->setTick(start);
                    logReader->fastForward(start);
                }

                float weightMultiplier = framesToSkip + 1;

                if(framesToSkip > 0)
                {
                    eFusion->setTick(eFusion->getTick() + framesToSkip);
                    logReader->fastForward(logReader->currentFrame + framesToSkip);
                    framesToSkip = 0;
                }

                Eigen::Matrix4f * currentPose = 0;

                if(groundTruthOdometry)
                {
                    currentPose = new Eigen::Matrix4f;
                    currentPose->setIdentity();
                    *currentPose = groundTruthOdometry->getTransformation(logReader->timestamp);
                }

                eFusion->processFrame(logReader->rgb, logReader->depth, logReader->timestamp, currentPose, weightMultiplier);

                // Log time and points. Drop the first two frames because they don't do full runs
                if (eFusion->getTick() > 2) {
                    const auto frame_time = Stopwatch::getInstance().getTimings().at("Run");
                    const auto preprocessing_time = Stopwatch::getInstance().getTimings().at("Preprocess");
                    const auto tracking_time = Stopwatch::getInstance().getTimings().at("Tracking1") + Stopwatch::getInstance().getTimings().at("Tracking2");
                    const auto fusion_time = Stopwatch::getInstance().getTimings().at("Fuse::Data") + Stopwatch::getInstance().getTimings().at("Fuse::Copy");
                    const auto mapping_time = frame_time - preprocessing_time - tracking_time - fusion_time;
                    std::cout << "Run: " << frame_time << "\n";
                    std::cout << "\tPreprocessing: " << preprocessing_time << "\n";
                    std::cout << "\tTracking: " << tracking_time << "\n";
                    std::cout << "\tMapping: " << mapping_time << "\n";
                    std::cout << "\tFusion: " << fusion_time << "\n";
                    std::cout << "\n";
                    times.push_back(Times{preprocessing_time, tracking_time, mapping_time, fusion_time, frame_time});

                    // Number of map points
                    points.push_back(eFusion->getGlobalModel().lastCount());
                }

                if(currentPose)
                {
                    delete currentPose;
                }

                if(frameskip && Stopwatch::getInstance().getTimings().at("Run") > 1000.f / 30.f)
                {
                    framesToSkip = int(Stopwatch::getInstance().getTimings().at("Run") / (1000.f / 30.f));
                }
            }
        }
        else
        {
            eFusion->predict();
        }

        TICK("GUI");

        if(!profile)
        {
            if(gui->followPose->Get())
            {
                pangolin::OpenGlMatrix mv;

                Eigen::Matrix4f currPose = eFusion->getCurrPose();
                Eigen::Matrix3f currRot = currPose.topLeftCorner(3, 3);

                Eigen::Quaternionf currQuat(currRot);
                Eigen::Vector3f forwardVector(0, 0, 1);
                Eigen::Vector3f upVector(0, iclnuim ? 1 : -1, 0);

                Eigen::Vector3f forward = (currQuat * forwardVector).normalized();
                Eigen::Vector3f up = (currQuat * upVector).normalized();

                Eigen::Vector3f eye(currPose(0, 3), currPose(1, 3), currPose(2, 3));

                eye -= forward;

                Eigen::Vector3f at = eye + forward;

                Eigen::Vector3f z = (eye - at).normalized();  // Forward
                Eigen::Vector3f x = up.cross(z).normalized(); // Right
                Eigen::Vector3f y = z.cross(x);

                Eigen::Matrix4d m;
                m << x(0),  x(1),  x(2),  -(x.dot(eye)),
                     y(0),  y(1),  y(2),  -(y.dot(eye)),
                     z(0),  z(1),  z(2),  -(z.dot(eye)),
                        0,     0,     0,              1;

                memcpy(&mv.m[0], m.data(), sizeof(Eigen::Matrix4d));

                gui->s_cam.SetModelViewMatrix(mv);
            }

            gui->preCall();

            std::stringstream stri;
            stri << eFusion->getModelToModel().lastICPCount;
            gui->trackInliers->Ref().Set(stri.str());

            std::stringstream stre;
            stre << (std::isnan(eFusion->getModelToModel().lastICPError) ? 0 : eFusion->getModelToModel().lastICPError);
            gui->trackRes->Ref().Set(stre.str());

            if(!gui->pause->Get())
            {
                gui->resLog.Log((std::isnan(eFusion->getModelToModel().lastICPError) ? std::numeric_limits<float>::max() : eFusion->getModelToModel().lastICPError), icpErrThresh);
                gui->inLog.Log(eFusion->getModelToModel().lastICPCount, icpCountThresh);
            }

            Eigen::Matrix4f pose = eFusion->getCurrPose();

            if(gui->drawRawCloud->Get() || gui->drawFilteredCloud->Get())
            {
                eFusion->computeFeedbackBuffers();
            }

            if(gui->drawRawCloud->Get())
            {
                eFusion->getFeedbackBuffers().at(FeedbackBuffer::RAW)->render(gui->s_cam.GetProjectionModelViewMatrix(), pose, gui->drawNormals->Get(), gui->drawColors->Get());
            }

            if(gui->drawFilteredCloud->Get())
            {
                eFusion->getFeedbackBuffers().at(FeedbackBuffer::FILTERED)->render(gui->s_cam.GetProjectionModelViewMatrix(), pose, gui->drawNormals->Get(), gui->drawColors->Get());
            }

            if(gui->drawGlobalModel->Get())
            {
                glFinish();
                TICK("Global");

                if(gui->drawFxaa->Get())
                {
                    gui->drawFXAA(gui->s_cam.GetProjectionModelViewMatrix(),
                                  gui->s_cam.GetModelViewMatrix(),
                                  eFusion->getGlobalModel().model(),
                                  eFusion->getConfidenceThreshold(),
                                  eFusion->getTick(),
                                  eFusion->getTimeDelta(),
                                  iclnuim);
                }
                else
                {
                    eFusion->getGlobalModel().renderPointCloud(gui->s_cam.GetProjectionModelViewMatrix(),
                                                               eFusion->getConfidenceThreshold(),
                                                               gui->drawUnstable->Get(),
                                                               gui->drawNormals->Get(),
                                                               gui->drawColors->Get(),
                                                               gui->drawPoints->Get(),
                                                               gui->drawWindow->Get(),
                                                               gui->drawTimes->Get(),
                                                               eFusion->getTick(),
                                                               eFusion->getTimeDelta());
                }
                glFinish();
                TOCK("Global");
            }

            if(eFusion->getLost())
            {
                glColor3f(1, 1, 0);
            }
            else
            {
                glColor3f(1, 0, 1);
            }
            gui->drawFrustum(pose);
            glColor3f(1, 1, 1);

            if(gui->drawFerns->Get())
            {
                glColor3f(0, 0, 0);
                for(size_t i = 0; i < eFusion->getFerns().frames.size(); i++)
                {
                    if((int)i == eFusion->getFerns().lastClosest)
                        continue;

                    gui->drawFrustum(eFusion->getFerns().frames.at(i)->pose);
                }
                glColor3f(1, 1, 1);
            }

            if(gui->drawDefGraph->Get())
            {
                const std::vector<GraphNode*> & graph = eFusion->getLocalDeformation().getGraph();

                for(size_t i = 0; i < graph.size(); i++)
                {
                    pangolin::glDrawCross(graph.at(i)->position(0),
                                          graph.at(i)->position(1),
                                          graph.at(i)->position(2),
                                          0.1);

                    for(size_t j = 0; j < graph.at(i)->neighbours.size(); j++)
                    {
                        pangolin::glDrawLine(graph.at(i)->position(0),
                                             graph.at(i)->position(1),
                                             graph.at(i)->position(2),
                                             graph.at(graph.at(i)->neighbours.at(j))->position(0),
                                             graph.at(graph.at(i)->neighbours.at(j))->position(1),
                                             graph.at(graph.at(i)->neighbours.at(j))->position(2));
                    }
                }
            }

            if(eFusion->getFerns().lastClosest != -1)
            {
                glColor3f(1, 0, 0);
                gui->drawFrustum(eFusion->getFerns().frames.at(eFusion->getFerns().lastClosest)->pose);
                glColor3f(1, 1, 1);
            }

            const std::vector<PoseMatch> & poseMatches = eFusion->getPoseMatches();

            int maxDiff = 0;
            for(size_t i = 0; i < poseMatches.size(); i++)
            {
                if(poseMatches.at(i).secondId - poseMatches.at(i).firstId > maxDiff)
                {
                    maxDiff = poseMatches.at(i).secondId - poseMatches.at(i).firstId;
                }
            }

            for(size_t i = 0; i < poseMatches.size(); i++)
            {
                if(gui->drawDeforms->Get())
                {
                    if(poseMatches.at(i).fern)
                    {
                        glColor3f(1, 0, 0);
                    }
                    else
                    {
                        glColor3f(0, 1, 0);
                    }
                    for(size_t j = 0; j < poseMatches.at(i).constraints.size(); j++)
                    {
                        pangolin::glDrawLine(poseMatches.at(i).constraints.at(j).sourcePoint(0), poseMatches.at(i).constraints.at(j).sourcePoint(1), poseMatches.at(i).constraints.at(j).sourcePoint(2),
                                             poseMatches.at(i).constraints.at(j).targetPoint(0), poseMatches.at(i).constraints.at(j).targetPoint(1), poseMatches.at(i).constraints.at(j).targetPoint(2));
                    }
                }
            }
            glColor3f(1, 1, 1);

            eFusion->normaliseDepth(0.3f, gui->depthCutoff->Get());

            for(std::map<std::string, GPUTexture*>::const_iterator it = eFusion->getTextures().begin(); it != eFusion->getTextures().end(); ++it)
            {
                if(it->second->draw)
                {
                    gui->displayImg(it->first, it->second);
                }
            }

            eFusion->getIndexMap().renderDepth(gui->depthCutoff->Get());

            gui->displayImg("ModelImg", eFusion->getIndexMap().imageTex());
            gui->displayImg("Model", eFusion->getIndexMap().drawTex());

            std::stringstream strs;
            strs << eFusion->getGlobalModel().lastCount();

            gui->totalPoints->operator=(strs.str());

            std::stringstream strs2;
            strs2 << eFusion->getLocalDeformation().getGraph().size();

            gui->totalNodes->operator=(strs2.str());

            std::stringstream strs3;
            strs3 << eFusion->getFerns().frames.size();

            gui->totalFerns->operator=(strs3.str());

            std::stringstream strs4;
            strs4 << eFusion->getDeforms();

            gui->totalDefs->operator=(strs4.str());

            std::stringstream strs5;
            strs5 << eFusion->getTick() << "/" << logReader->getNumFrames();

            gui->logProgress->operator=(strs5.str());

            std::stringstream strs6;
            strs6 << eFusion->getFernDeforms();

            gui->totalFernDefs->operator=(strs6.str());

            gui->postCall();

            logReader->flipColors = gui->flipColors->Get();
            eFusion->setRgbOnly(gui->rgbOnly->Get());
            eFusion->setPyramid(gui->pyramid->Get());
            eFusion->setFastOdom(gui->fastOdom->Get());
            eFusion->setConfidenceThreshold(gui->confidenceThreshold->Get());
            eFusion->setDepthCutoff(gui->depthCutoff->Get());
            eFusion->setIcpWeight(gui->icpWeight->Get());
            eFusion->setSo3(gui->so3->Get());
            eFusion->setFrameToFrameRGB(gui->frameToFrameRGB->Get());

            resetButton = pangolin::Pushed(*gui->reset);

            if(gui->autoSettings)
            {
                static bool last = gui->autoSettings->Get();

                if(gui->autoSettings->Get() != last)
                {
                    last = gui->autoSettings->Get();
                    static_cast<LiveLogReader *>(logReader)->setAuto(last);
                }
            }
        }

        Stopwatch::getInstance().sendAll();

        if(resetButton)
        {
            break;
        }

        if(pangolin::Pushed(*gui->save))
        {
            eFusion->savePly();
        }

        TOCK("GUI");
    }

    // Dump time per frame
    double average_preprocessing_time = 0.0;
    double average_tracking_time = 0.0;
    double average_mapping_time = 0.0;
    double average_fusion_time = 0.0;
    double average_frame_time = 0.0;

    std::ofstream output_times_file;
    output_times_file.open("ef_times.data", std::ios::out);
    output_times_file << "#Frame Preprocessing Tracking Mapping Fusion Total\n";

    for (unsigned i = 0; i < times.size(); i++) {
        const auto preprocessing_time = times[i].preprocessing;
        const auto tracking_time = times[i].tracking;
        const auto mapping_time = times[i].mapping;
        const auto fusion_time = times[i].fusion;
        const auto frame_time = times[i].frame;

        output_times_file << i
                          << " " << preprocessing_time
                          << " " << tracking_time
                          << " " << mapping_time
                          << " " << fusion_time
                          << " " << frame_time
                          << std::endl;

        average_preprocessing_time += preprocessing_time;
        average_tracking_time += tracking_time;
        average_mapping_time += mapping_time;
        average_fusion_time += fusion_time;
        average_frame_time += frame_time;
    }
    output_times_file.close();

    std::cout << "Average frame time = " << average_frame_time / ((double) times.size()) << std::endl;
    std::cout << "Average preprocessing time = " << average_preprocessing_time / ((double) times.size()) << std::endl;
    std::cout << "Average tracking time = " << average_tracking_time / ((double) times.size()) << std::endl;
    std::cout << "Average mapping time = " << average_mapping_time / ((double) times.size()) << std::endl;
    std::cout << "Average fusion time = " << average_fusion_time / ((double) times.size()) << std::endl;

    // Dump points per frame
    std::ofstream output_points_file;
    output_points_file.open("ef_points.data", std::ios::out);
    output_points_file << "#Frame Points\n";

    for (unsigned i = 0; i < points.size(); i++) {
        output_points_file << i << " " << points[i] << "\n";
    }
    output_points_file.close();
}
