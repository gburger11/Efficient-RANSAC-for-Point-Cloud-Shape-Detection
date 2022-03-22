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
// Get the current timestamp formatted as a string.
std::string get_timestamp();

// Write the pointcloud extracted by the RANSAC algorithm in PLY format.
void write_pointcloud(
    PointCloud pc,
    std::string file_path,
    MiscLib::Vector< std::pair< MiscLib::RefCountPtr< PrimitiveShape >, size_t > > shapes,
    bool keep_unassigned = true
    );

// Write the pointcloud extracted by the RANSAC projecting the points on the surfaces.
void write_pointcloud_projected(
    PointCloud pc,
    std::string file_path,
    MiscLib::Vector< std::pair< MiscLib::RefCountPtr< PrimitiveShape >, size_t > > shapes
    );

// Write a new pointcloud compressed and meshed with the projection of the points to the bitmap on the identified
// surfaces.
void write_pointcloud_shapes(
    PointCloud pc,
    std::string file_path,
    MiscLib::Vector< std::pair< MiscLib::RefCountPtr< PrimitiveShape >, size_t > > shapes,
    RansacShapeDetector::Options ransacOptions
    );

// Extract triangular faces from 4 points in the bitmap depending on their respective occupancy.
void extract_faces(MiscLib::Vector< char >  const& bitmap, MiscLib::Vector<size_t> const& bitmap_indices,
                   size_t i1, size_t i2, size_t i3, size_t i4,
                   std::vector<std::vector<int>>& facesOut);

// Class used to record the duration of different stages in an algorithm.
class TimeRecorder
{
public:
    TimeRecorder();
    void record(std::string label);
    float get_latest_duration();

    std::vector<std::chrono::time_point<std::chrono::high_resolution_clock>> times_;
    std::vector<std::string> time_labels_;
};

// Write metadata in a yaml format about the parameters and results of the RANSAC algorithm
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
