#include <pangolin/pangolin.h>
#include <pangolin/gl/gl.h>
#include <pangolin/gl/gldraw.h>

#include "common/switchboard.hpp"
#include "common/data_format.hpp"
#include "common/threadloop.hpp"

#include "Core/src/ElasticFusion.h"



using namespace ILLIXR;

#define EF_EPOCH (1.0/10.0)

class elastic_fusion : public ILLIXR::threadloop {
public:
	elastic_fusion(const std::string& name_, phonebook* pb_)
		: threadloop(name_, pb_)
		, sb{pb->lookup_impl<switchboard>()}
		, _m_cam{sb->subscribe_latest<rgb_depth_type>("rgb_depth")}
	{
		// Initialize time
		sync = std::chrono::high_resolution_clock::now();

		Resolution::getInstance(1280, 720);
    Intrinsics::getInstance(681.11, 681.11, 611.625, 401.201);
		// Create GL context before initializing EF
		pangolin::Params windowParams;
    windowParams.Set("SAMPLE_BUFFERS", 0);
    windowParams.Set("SAMPLES", 0);
    pangolin::CreateWindowAndBind("Main", 1280, 800, windowParams);
    eFusion = new ElasticFusion();
	}

protected:
	virtual void _p_one_iteration() override {
		std::chrono::time_point<std::chrono::system_clock> blockStart = std::chrono::high_resolution_clock::now();
		if (blockStart < sync) {
			std::this_thread::yield(); // ←_←
			// continue;
			return;
		}
		// seconds in double
		double timespent = std::chrono::duration<double>(blockStart-sync).count();
		int num_epoch = ceil(timespent/EF_EPOCH);
		sync += std::chrono::microseconds(num_epoch*((int)(EF_EPOCH*1000000)));

		//TODO: add skipped frames to weight
		auto most_recent_cam = _m_cam->get_latest_ro();

		Eigen::Matrix4f* currentPose = 0;
		if (most_recent_cam == NULL) {
			return;
		}
		eFusion->processFrame(most_recent_cam->rgb, most_recent_cam->depth, most_recent_cam->time, currentPose, 1);
	}

private:
	std::shared_ptr<switchboard> sb;
	std::unique_ptr<reader_latest<rgb_depth_type>> _m_cam;
	std::chrono::time_point<std::chrono::system_clock> sync;

	ElasticFusion* eFusion;
};

PLUGIN_MAIN(elastic_fusion)
