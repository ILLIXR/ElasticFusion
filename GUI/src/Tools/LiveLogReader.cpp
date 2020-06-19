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

#include "LiveLogReader.h"

using namespace ILLIXR;

LiveLogReader::LiveLogReader(std::string file, bool flipColors, std::shared_ptr<switchboard> sb)
 : LogReader(file, flipColors),
   lastFrameTime(-1),
   lastGot(-1)
{
    std::cout << "Creating illxr capture... "; std::cout.flush();

	//decompressionBufferDepth = new Bytef[Resolution::getInstance().numPixels() * 2];
	//decompressionBufferImage = new Bytef[Resolution::getInstance().numPixels() * 3];

    _m_cam = sb->subscribe_latest<rgb_depth_type>("rgb_depth");

}

LiveLogReader::~LiveLogReader()
{
}

void LiveLogReader::getNext()
{
    auto most_recent_cam = _m_cam->get_latest_ro();

    lastFrameTime = most_recent_cam->time;

    timestamp = lastFrameTime;

    rgb = most_recent_cam->rgb;
    depth = most_recent_cam->depth;

    imageReadBuffer = 0;
    depthReadBuffer = 0;

    imageSize = Resolution::getInstance().numPixels() * 3;
    depthSize = Resolution::getInstance().numPixels() * 2;
}

const std::string LiveLogReader::getFile()
{
    return Parse::get().baseDir().append("live");
}

int LiveLogReader::getNumFrames()
{
    return std::numeric_limits<int>::max();
}

bool LiveLogReader::hasMore()
{
    return true;
}

void LiveLogReader::setAuto(bool value)
{
    //cam->setAutoExposure(value);
    //cam->setAutoWhiteBalance(value);
}
