#include <string>
#include <vector>
#include <chrono>
#include <unordered_map>

#include <MiscLib/Vector.h>
#include <MiscLib/RefCountPtr.h>

#include <RansacShapeDetector.h>
#include <PrimitiveShape.h>

namespace outputs
{
std::string get_timestamp();

void write_pointcloud(
    PointCloud pc,
    std::string file_path,
    MiscLib::Vector< std::pair< MiscLib::RefCountPtr< PrimitiveShape >, size_t > > shapes
    );

class TimeRecorder
{
public:
    TimeRecorder();
    void record(std::string label);

    std::vector<std::chrono::time_point<std::chrono::high_resolution_clock>> times_;
    std::vector<std::string> time_labels_;
};

void write_metadata(
    std::string file_path,
    int remaining,
    MiscLib::Vector< std::pair< MiscLib::RefCountPtr< PrimitiveShape >, size_t > > const& shapes,
    std::string timestamp,
    std::string object,
    std::unordered_map<std::string, float> misc_params,
    RansacShapeDetector::Options ransacOptions,
    TimeRecorder const& time_record
);

}  // end of namespace outputs
