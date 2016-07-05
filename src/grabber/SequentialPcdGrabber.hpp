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
        for (auto &worker : workers_) {
            worker.join();
        }
    }

    inline void start(std::function<void()> &callback) override {
        start(files_.begin()->first, callback);
    }                

    inline void start(bptime started_at, std::function<void()> &callback) override {
        start(time_in_nanos(started_at), callback);
    }

    inline void initialize(std::function<void(bpath, int, int)> &callback) {
        workers_.push_back(std::thread([&]{
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
        }));
    }

private:
    std::map<uint64_t, PointCloudPtr> clouds_;
    std::map<uint64_t, bpath> files_;
             
    std::vector<std::thread> workers_;
    std::atomic<bool> worker_canceled_;

    inline void start(const uint64_t started_at, std::function<void()> &callback) {
        workers_.push_back(std::thread([this, started_at, &callback] {
            const auto started_at_in_real = time_in_nanos(current_time());
            auto itr = files_.begin();
            while (!worker_canceled_) {
                if ((itr->first - started_at) <= (current_time_in_nanos() - started_at_in_real)) {
                    cloud_ = clouds_[itr->first];
                    callback();
                    itr++;
                }
                if (itr == files_.end()) {
                    itr = files_.begin();
                }
            }
        }));
    }
};

}

#endif /* SequentialPcdGrabber_hpp */
