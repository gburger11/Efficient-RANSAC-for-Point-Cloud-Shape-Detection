// Taken from https://github.com/ihmcrobotics/ihmc-open-robotics-software/blob/5f5345ea78f681c1ca815bb1539041b5d0ab54d0/ihmc-sensor-processing/csrc/ransac_schnabel/main.cpp

#include <happly.h>
#include <PointCloud.h>
#include <Outputs.h>
#include <RansacShapeDetector.h>
#include <PlanePrimitiveShapeConstructor.h>
#include <CylinderPrimitiveShapeConstructor.h>
#include <SpherePrimitiveShapeConstructor.h>
#include <ConePrimitiveShapeConstructor.h>
#include <TorusPrimitiveShapeConstructor.h>

#include <iostream>
#include <fstream>
#include <time.h>
#include <chrono>



int main()
{
    //////////////////////////////////////////
    // Create Pointcloud
    //////////////////////////////////////////
    outputs::TimeRecorder time_record;

    // std::string const object = "Apple";
    // std::string const object = "Santa";
    // std::string const object = "Buddha";
    // std::string const object = "Donkey";
    // std::string const object = "Oil_pump";
    // std::string const object = "master_cylinder";
    std::string const object = "Lille_street_small";
    // std::string const object = "indoor_scan";

    PointCloud pc("../../data/" + object +  ".ply");
    pc.adjustBBoxToPoints();

    time_record.record("read");

    float normal_radius = 0.01;
    float bitmap_precision = pc.calcNormalsAndBEps(normal_radius * pc.getScale());

    time_record.record("normals");

	std::cout << "added " << pc.size() << " points" << std::endl;

    //////////////////////////////////////////
    // Actual RANSAC
    //////////////////////////////////////////

	RansacShapeDetector::Options ransacOptions;
	float eps = 0.0015f;
	ransacOptions.m_epsilon = eps * pc.getScale(); // set distance threshold to .01f of bounding box width
		// NOTE: Internally the distance threshold is taken as 3 * ransacOptions.m_epsilon!!!
    float b_eps = 0.005f;
    ransacOptions.m_bitmapEpsilon = b_eps * pc.getScale(); // set bitmap resolution to .02f of bounding box width
    // ransacOptions.m_bitmapEpsilon = bitmap_precision;
		// NOTE: This threshold is NOT multiplied internally!
    // ransacOptions.m_normalThresh = .9f; // this is the cos of the maximal normal deviation
    ransacOptions.m_normalThresh = .86f; // this is the cos of the maximal normal deviation
	// ransacOptions.m_normalThresh = .82f; // this is the cos of the maximal normal deviation
	ransacOptions.m_minSupport = 100; // this is the minimal number of points required for a primitive
	ransacOptions.m_probability = .001f; // this is the "probability" with which a primitive is overlooked

	RansacShapeDetector detector(ransacOptions); // the detector object

	// set which primitives are to be detected by adding the respective constructors
	detector.Add(new PlanePrimitiveShapeConstructor());
    detector.Add(new CylinderPrimitiveShapeConstructor());
    detector.Add(new SpherePrimitiveShapeConstructor());
    detector.Add(new ConePrimitiveShapeConstructor());
    detector.Add(new TorusPrimitiveShapeConstructor());

	std::cout << "Problem defined" << std::endl;

	MiscLib::Vector< std::pair< MiscLib::RefCountPtr< PrimitiveShape >, size_t > > shapes; // stores the detected shapes
	// size_t remaining = 0;
    size_t remaining = detector.Detect(pc, 0, pc.size(), &shapes); // run detection
		// returns number of unassigned points
		// the array shapes is filled with pointers to the detected shapes
		// the second element per shapes gives the number of points assigned to that primitive (the support)
		// the points belonging to the first shape (shapes[0]) have been sorted to the end of pc,
		// i.e. into the range [ pc.size() - shapes[0].second, pc.size() )
		// the points of shape i are found in the range
		// [ pc.size() - \sum_{j=0..i} shapes[j].second, pc.size() - \sum_{j=0..i-1} shapes[j].second )

    time_record.record("detection");

	std::cout << "remaining unassigned points " << remaining << std::endl;
	for(int i=0;i<shapes.size();i++)
	{
		std::string desc;
		shapes[i].first->Description(&desc);
		std::cout << "shape " << i << " consists of " << shapes[i].second << " points, it is a " << desc << std::endl;
	}

    //////////////////////////////////////////
    // Save results in PLY file
    //////////////////////////////////////////

    std::string timestamp = outputs::get_timestamp();

    outputs::write_pointcloud(
        pc,
        "out_" + object + "_" + timestamp + ".ply",
        shapes
    );

    outputs::write_pointcloud_projected(
        pc,
        "out_" + object + "_" + timestamp + "_proj.ply",
        shapes
    );

    outputs::write_pointcloud_shapes(
        pc,
        "out_" + object + "_" + timestamp + "_shapes.ply",
        shapes,
        ransacOptions
    );

    time_record.record("write");

    outputs::write_metadata(
        "out_" + object + "_" + timestamp + ".yaml",
        remaining,
        shapes,
        timestamp,
        object,
        {{"cloud_scale", pc.getScale()}, {"eps", eps}, {"b_eps", b_eps}, {"normal_radius", normal_radius}},
        ransacOptions,
        time_record
    );

    return 0;
}
