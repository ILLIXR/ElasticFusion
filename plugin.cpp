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
	elastic_fusion(phonebook* pb)
		: sb{pb->lookup_impl<switchboard>()}
		, _m_cam{sb->subscribe_latest<ef_cam_type>("ef_cam")}
	{
		// Initialize time
		sync = std::chrono::high_resolution_clock::now();

		Resolution::getInstance(640, 480);
    	Intrinsics::getInstance(528, 528, 320, 240);
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
		eFusion->processFrame(most_recent_cam->rgb, most_recent_cam->depth, most_recent_cam->timestamp, currentPose, 1);
	}

private:
	switchboard * sb;
	std::unique_ptr<reader_latest<ef_cam_type>> _m_cam;
	std::chrono::time_point<std::chrono::system_clock> sync;

	ElasticFusion* eFusion;
};

PLUGIN_MAIN(elastic_fusion)
