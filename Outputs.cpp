#include <time.h>

#include <Outputs.h>
#include <happly.h>
#include <PlanePrimitiveShape.h>
#include <CylinderPrimitiveShape.h>
#include <SpherePrimitiveShape.h>
#include <ConePrimitiveShape.h>
#include <TorusPrimitiveShape.h>

namespace outputs
{
std::vector<std::string> EXPECTED_SHAPES = {"Plane", "Sphere", "Cylinder", "Cone", "Torus"};

std::string get_timestamp()
{
    char timestamp[17];
    time_t rawtime;
    time(&rawtime);
    struct tm * timeinfo = localtime(&rawtime);
    strftime(timestamp, 17, "%Y%m%dT%H%M%SZ", timeinfo);
    return std::string(timestamp);
}

void write_pointcloud_shapes(
    PointCloud pc,
    std::string file_path,
    MiscLib::Vector< std::pair< MiscLib::RefCountPtr< PrimitiveShape >, size_t > > shapes,
	RansacShapeDetector::Options ransacOptions
    )
{
    std::array<std::vector<float>, 3> coord;
    for(int c = 0; c < 3; ++c)
        coord[c].reserve(pc.size());
    std::array<std::vector<int>, 2> labels;
    for(int c = 0; c < 2; ++c)
        labels[c].reserve(pc.size());
    std::vector<std::vector<int>> facesOut;

    int i_point = pc.size();
    int N_shapes_to_save = 1000;
	for(int i = 0; i < std::min(int(shapes.size()), N_shapes_to_save); ++i)
	{
		std::string shape_desc;
		shapes[i].first->Description(&shape_desc);
		int shape_ind = std::find(EXPECTED_SHAPES.begin(), EXPECTED_SHAPES.end(), shape_desc) - EXPECTED_SHAPES.begin();


        MiscLib::Vector< int > componentsImg;
        MiscLib::Vector< std::pair< int, size_t > > labels_b;
        MiscLib::Vector< size_t > indices;
        indices.reserve(shapes[i].second);
        for(int j = 0; j < shapes[i].second; ++j)
        {
            --i_point;
            indices.push_back(i_point);
        }

        BitmapInfo bitmapInfo;
        BitmapPrimitiveShape * bps = dynamic_cast<BitmapPrimitiveShape *>(shapes[i].first.Ptr());
        bps->AllConnectedComponents( pc, ransacOptions.m_bitmapEpsilon , bitmapInfo, &indices, componentsImg, labels_b, false );
        // bps->BuildBitmap(pc, &ransacOptions.m_bitmapEpsilon, indices.begin(), indices.end(), &bitmapInfo.params,
            // &bitmapInfo.bbox, &bitmapInfo.bitmap, &bitmapInfo.uextent, &bitmapInfo.vextent, &bitmapInfo.bmpIdx);
        // MiscLib::Vector< size_t > bitmap = bitmapInfo.bmpIdx;


        size_t idx_point = coord[0].size();
        MiscLib::Vector<size_t> bitmap_indices(componentsImg.size(), std::numeric_limits<size_t>::max());
        size_t rl = bitmapInfo.uextent;  // row length, used many times for offset in the 2D array

        for(size_t v = 0; v < bitmapInfo.vextent; ++v)
        {
            for (size_t u = 0; u < bitmapInfo.uextent; ++u)
            {
                if (componentsImg[u + v * rl] > 0)
                {
                    bitmap_indices[u + v * rl] = idx_point;
                    ++idx_point;

                    Vec3f p;
                    Vec3f n;
                    bps->InSpace(u, v, ransacOptions.m_bitmapEpsilon, bitmapInfo.bbox, bitmapInfo.uextent, bitmapInfo.vextent, &p, &n);
                    for (int c = 0; c < 3; ++c)
                    {
                        coord[c].push_back(p[c]);
                    }
                    labels[0].push_back(i);
                    labels[1].push_back(shape_ind);
                }
            }
        }
        for(size_t v = 0; v < bitmapInfo.vextent - 1; ++v)
        {
            for (size_t u = 0; u < bitmapInfo.uextent - 1; ++u)
            {
                if (componentsImg[u + v * rl] > 0)
                {
                    if (componentsImg[u + 1 + v * rl] > 0)
                    {
                        if (componentsImg[u + (v + 1) * rl] > 0)
                        {
                            std::vector<int> new_face = {
                                bitmap_indices[u + v * rl],
                                bitmap_indices[u + 1 + v * rl],
                                bitmap_indices[u + (v + 1) * rl]
                            };
                            facesOut.push_back(new_face);

                            if (componentsImg[u + 1 + (v + 1) * rl] > 0)
                            {
                                std::vector<int> new_face = {
                                    bitmap_indices[u + 1 + v * rl],
                                    bitmap_indices[u + 1 + (v + 1) * rl],
                                    bitmap_indices[u + (v + 1) * rl]
                                };
                                facesOut.push_back(new_face);
                            }
                        }
                        else if(componentsImg[u + 1 + (v + 1) * rl] > 0)
                        {
                            std::vector<int> new_face = {
                                bitmap_indices[u + v * rl],
                                bitmap_indices[u + 1 + v * rl],
                                bitmap_indices[u + 1 + (v + 1) * rl]
                            };
                            facesOut.push_back(new_face);
                        }
                    }
                    else if (componentsImg[u + (v + 1) * rl] > 0 and componentsImg[u + 1 + (v + 1) * rl] > 0)
                    {
                        std::vector<int> new_face = {
                            bitmap_indices[u + v * rl],
                            bitmap_indices[u + 1 + (v + 1) * rl],
                            bitmap_indices[u + (v + 1) * rl]
                        };
                        facesOut.push_back(new_face);
                    }
                }
                else if (componentsImg[u + 1 + v * rl] > 0 and
                         componentsImg[u + 1 + (v + 1) * rl] and
                         componentsImg[u + (v + 1) * rl])
                {
                    std::vector<int> new_face = {
                        bitmap_indices[u + 1 + v * rl],
                        bitmap_indices[u + 1 + (v + 1) * rl],
                        bitmap_indices[u + (v + 1) * rl]
                    };
                    facesOut.push_back(new_face);
                }
            }
        }

	}

    happly::PLYData plyOut;
    plyOut.addElement("vertex", coord[0].size());
    plyOut.getElement("vertex").addProperty<float>("x", coord[0]);
    plyOut.getElement("vertex").addProperty<float>("y", coord[1]);
    plyOut.getElement("vertex").addProperty<float>("z", coord[2]);
    plyOut.getElement("vertex").addProperty<int>("shape_ind", labels[0]);
    plyOut.getElement("vertex").addProperty<int>("shape_type", labels[1]);

    plyOut.addElement("face", facesOut.size());
    plyOut.getElement("face").addListProperty<int>("vertex_indices", facesOut);

    plyOut.write(file_path);
}

void write_pointcloud(
    PointCloud pc,
    std::string file_path,
    MiscLib::Vector< std::pair< MiscLib::RefCountPtr< PrimitiveShape >, size_t > > shapes
    )
{
    std::array<std::vector<float>, 3> coord;
    for(int c = 0; c < 3; ++c)
        coord[c].reserve(pc.size());
    std::array<std::vector<int>, 2> labels;
    for(int c = 0; c < 2; ++c)
        labels[c].reserve(pc.size());

    int i_point = pc.size();
    int N_shapes_to_save = 1000;
    for(int i = 0; i < std::min(int(shapes.size()), N_shapes_to_save); ++i)
    {
        std::string shape_desc;
        shapes[i].first->Description(&shape_desc);
        int shape_ind = std::find(EXPECTED_SHAPES.begin(), EXPECTED_SHAPES.end(), shape_desc) - EXPECTED_SHAPES.begin();

        for(int j = 0; j < shapes[i].second; ++j)
        {
            --i_point;
            for (int c = 0; c < 3; ++c)
            {
                coord[c].push_back(pc[i_point][c]);
            }
            labels[0].push_back(i);
            labels[1].push_back(shape_ind);
        }
    }
    --i_point;
    for(; i_point >= 0; --i_point)
    {
        for (int c = 0; c < 3; ++c)
        {
            coord[c].push_back(pc[i_point][c]);
        }
        labels[0].push_back(std::min(int(shapes.size()), N_shapes_to_save));
        labels[1].push_back(EXPECTED_SHAPES.size());
    }

    happly::PLYData plyOut;
    plyOut.addElement("vertex", pc.size());
    plyOut.getElement("vertex").addProperty<float>("x", coord[0]);
    plyOut.getElement("vertex").addProperty<float>("y", coord[1]);
    plyOut.getElement("vertex").addProperty<float>("z", coord[2]);
    plyOut.getElement("vertex").addProperty<int>("shape_ind", labels[0]);
    plyOut.getElement("vertex").addProperty<int>("shape_type", labels[1]);

    // if (plyIn.hasElement("face"))
    if (pc.getFaces().size() > 0)
    {
        // std::vector<std::vector<int>> facesIn = plyIn.getElement("face").getListProperty<int>("vertex_indices");
        std::vector<std::vector<int>> facesOut;
        // facesOut.reserve(facesIn.size());
        facesOut.reserve(pc.getFaces().size());

        std::vector<size_t> inversion_map(pc.size(), 0);
        for (size_t i =  0; i < pc.size(); ++i)
        {
            inversion_map[pc[i].index] = pc.size() - 1 - i;
        }


        // for (std::vector<int> const& face : facesIn)
        for (std::vector<int> const& face : pc.getFaces())
        {
            std::vector<int> new_face = {inversion_map[face[0]], inversion_map[face[1]], inversion_map[face[2]]};
            facesOut.push_back(new_face);
        }
        plyOut.addElement("face", facesOut.size());
        plyOut.getElement("face").addListProperty<int>("vertex_indices", facesOut);
    }

    plyOut.write(file_path);
}

TimeRecorder::TimeRecorder()
{
    times_.push_back(std::chrono::high_resolution_clock::now());
}

void TimeRecorder::record(std::string label)
{
    times_.push_back(std::chrono::high_resolution_clock::now());
    time_labels_.push_back(label);
}

void write_metadata(
    std::string file_path,
    int remaining,
    MiscLib::Vector< std::pair< MiscLib::RefCountPtr< PrimitiveShape >, size_t > > const& shapes,
    std::string timestamp,
    std::string object,
    std::unordered_map<std::string, float> misc_params,
    RansacShapeDetector::Options ransacOptions,
    TimeRecorder const& time_record
)
{
    std::ofstream yamlOut;
    yamlOut.open(file_path);
    // Global data
    yamlOut << "global:" << std::endl;
    yamlOut << "  time: " << timestamp << std::endl;
    yamlOut << "  object: " << object << std::endl;

    // Results
    yamlOut << "results:" << std::endl;
    yamlOut << "  remaining: " << remaining << std::endl;
    yamlOut << "  n_shapes: " << shapes.size() << std::endl;
    yamlOut << "  shapes: " << std::endl;
	for(int i = 0; i < shapes.size(); i++)
	{
		std::string desc;
		shapes[i].first->Description(&desc);
		yamlOut << "    - index: " << i << std::endl;
		yamlOut << "      type: " << desc << std::endl;
		yamlOut << "      num_points: " << shapes[i].second << std::endl;
		if (PlanePrimitiveShape const* pps = dynamic_cast<PlanePrimitiveShape const*>(shapes[i].first.Ptr()))
        {
            Plane const& plane = pps->Internal();
            yamlOut << "      parameters:" << std::endl;
            yamlOut << "        point:" << std::endl;
            for (size_t c = 0; c < 3; ++c)
            {
                yamlOut << "          - " << plane.getPosition()[c] << std::endl;
            }
            yamlOut << "        normal:" << std::endl;
            for (size_t c = 0; c < 3; ++c)
            {
                yamlOut << "          - " << plane.getNormal()[c] << std::endl;
            }
        }
        else if (CylinderPrimitiveShape const* cps = dynamic_cast<CylinderPrimitiveShape const*>(shapes[i].first.Ptr()))
        {
            Cylinder const& cylinder = cps->Internal();
            yamlOut << "      parameters:" << std::endl;
            yamlOut << "        axis_point:" << std::endl;
            for (size_t c = 0; c < 3; ++c)
            {
                yamlOut << "          - " << cylinder.AxisPosition()[c] << std::endl;
            }
            yamlOut << "        axis_direction:" << std::endl;
            for (size_t c = 0; c < 3; ++c)
            {
                yamlOut << "          - " << cylinder.AxisDirection()[c] << std::endl;
            }
            yamlOut << "        radius: " << cylinder.Radius() << std::endl;
        }
        else if (SpherePrimitiveShape const* sps = dynamic_cast<SpherePrimitiveShape const*>(shapes[i].first.Ptr()))
        {
            Sphere const& sphere = sps->Internal();
            yamlOut << "      parameters:" << std::endl;
            yamlOut << "        center:" << std::endl;
            for (size_t c = 0; c < 3; ++c)
            {
                yamlOut << "          - " << sphere.Center()[c] << std::endl;
            }
            yamlOut << "        radius: " << sphere.Radius() << std::endl;
        }
        else if (ConePrimitiveShape const* cps = dynamic_cast<ConePrimitiveShape const*>(shapes[i].first.Ptr()))
        {
            Cone const& cone = cps->Internal();
            yamlOut << "      parameters:" << std::endl;
            yamlOut << "        apex:" << std::endl;
            for (size_t c = 0; c < 3; ++c)
            {
                yamlOut << "          - " << cone.Center()[c] << std::endl;
            }
            yamlOut << "        axis_direction:" << std::endl;
            for (size_t c = 0; c < 3; ++c)
            {
                yamlOut << "          - " << cone.AxisDirection()[c] << std::endl;
            }
            yamlOut << "        angle: " << cone.Angle() << std::endl;
        }
        else if (TorusPrimitiveShape const* tps = dynamic_cast<TorusPrimitiveShape const*>(shapes[i].first.Ptr()))
        {
            Torus const& torus = tps->Internal();
            yamlOut << "      parameters:" << std::endl;
            yamlOut << "        center:" << std::endl;
            for (size_t c = 0; c < 3; ++c)
            {
                yamlOut << "          - " << torus.Center()[c] << std::endl;
            }
            yamlOut << "        normal:" << std::endl;
            for (size_t c = 0; c < 3; ++c)
            {
                yamlOut << "          - " << torus.AxisDirection()[c] << std::endl;
            }
            yamlOut << "        minor_radius: " << torus.MinorRadius() << std::endl;
            yamlOut << "        major_radius: " << torus.MajorRadius() << std::endl;
        }
        else
        {
            yamlOut << "      parameters: null" << std::endl;
        }
	}

    // Parameters
    yamlOut << "parameters:" << std::endl;
    for (auto const& mp: misc_params)
    {
        yamlOut << "  " << mp.first << ": " << mp.second << std::endl;
    }
    yamlOut << "  epsilon_abs: " << ransacOptions.m_epsilon << std::endl;
    yamlOut << "  normal_thrashold: " << ransacOptions.m_normalThresh << std::endl;
    yamlOut << "  bitmap_epsilon_abs: " << ransacOptions.m_bitmapEpsilon << std::endl;
    yamlOut << "  min_support: " << ransacOptions.m_minSupport << std::endl;
    yamlOut << "  probability: " << ransacOptions.m_probability << std::endl;

	// Times
    yamlOut << "timings:" << std::endl;
    yamlOut << "  total: " <<
        (std::chrono::duration<double>(time_record.times_.back() - time_record.times_.front())).count() << std::endl;
    for (int i = 0; i < time_record.times_.size() - 1; ++i)
    {
        yamlOut << "  " << time_record.time_labels_[i] <<  ": " <<
            (std::chrono::duration<double>(time_record.times_[i + 1] - time_record.times_[i])).count() << std::endl;
    }

    // Results
    yamlOut << "format:" << std::endl;
    yamlOut << "  shapes: " << std::endl;
	for(int i = 0; i < EXPECTED_SHAPES.size(); i++)
	{
		yamlOut << "    - " << EXPECTED_SHAPES[i] << std::endl;
	}

    yamlOut.close();
}

}  // end of namespace outputs
