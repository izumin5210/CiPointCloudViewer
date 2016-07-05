//
//  SequentialPcdGrabber.hpp
//  CiPointCloudViewer
//
//  Created by Masayuki IZUMI on 7/4/16.
//
//

#ifndef SequentialPcdGrabber_hpp
#define SequentialPcdGrabber_hpp

#include <pcl/io/pcd_io.h>

#include <map>

#include "PointCloudGrabber.hpp"

namespace bfs = boost::filesystem;
namespace bpt = boost::posix_time;

namespace grabber {

class SequentialPcdGrabber : public PointCloudGrabber {
public:
    typedef bfs::directory_iterator ditr;
    typedef bpt::ptime bptime;

    SequentialPcdGrabber(const bpath path)
        : PointCloudGrabber(path)
        , worker_canceled_(false)
    {
    }

    ~SequentialPcdGrabber() {
        worker_canceled_ = true;
        loader_worker_.join();
        player_worker_.join();
    }

    inline void start(std::function<void()> &callback) override {
        start(files_.begin()->first, callback);
    }                

    inline void start(bptime started_at, std::function<void()> &callback) override {
        start(time_in_nanos(started_at), callback);
    }

    inline void stop() override {
        player_worker_.join();
    }

    inline void initialize(std::function<void(bpath, int, int)> &callback) {
        loader_worker_ = std::thread([&]{
            for (auto file : boost::make_iterator_range(ditr(path_), ditr())) {
                if (file.path().extension().string() == ".pcd") {
                    const auto stamp = bpt::from_iso_string(file.path().stem().string());
                    files_[time_in_nanos(stamp)] = file.path();
                }
            }
            int i = 0;
            for (auto pair : files_) {
                if (worker_canceled_) { break; }
                const auto cloud = PointCloudPtr(new PointCloud);
                pcl::io::loadPCDFile(pair.second.string(), *cloud);
                clouds_[pair.first] = cloud;
                i++;
                callback(path_, i, files_.size());
            }
        });
    }

    inline bool isLoaded(const uint64_t started_at) {
        return clouds_.find(started_at) != clouds_.end();
    }

    inline uint64_t elapsedTime() {
        return current_time_in_nanos() - started_at_in_real_;
    }


private:
    std::map<uint64_t, PointCloudPtr> clouds_;
    std::map<uint64_t, bpath> files_;

    std::atomic<bool> worker_canceled_;
    std::atomic<bool> waiting_;

    std::thread loader_worker_;
    std::thread player_worker_;

    std::atomic<uint64_t> started_at_in_real_;
    std::atomic<uint64_t> waited_since_;

    inline void start(const uint64_t started_at, std::function<void()> &callback) {
        player_worker_ = std::thread([this, started_at, &callback] {
            auto started_at_in_real_ = time_in_nanos(current_time());
            auto itr = files_.begin();
            waiting_ = false;
            while (!worker_canceled_) {
                if (!isLoaded(itr->first)) {
                    if (waiting_) {
                        waited_since_ = current_time_in_nanos();
                    } else {
                        waiting_ = true;
                    }
                    continue;
                } else if (waiting_) {
                    started_at_in_real_ += current_time_in_nanos() - waited_since_;
                    waiting_ = false;
                }
                if ((itr->first - started_at) <= elapsedTime()) {
                    cloud_ = clouds_[itr->first];
                    callback();
                    itr++;
                }
                if (itr == files_.end()) {
                    itr = files_.begin();
                    started_at_in_real_ = time_in_nanos(current_time());
                }
            }
        });
    }
};

}

#endif /* SequentialPcdGrabber_hpp */
