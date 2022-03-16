#include "PointCloud.h"
#include "happly.h"
#include <MiscLib/Vector.h>
#include <iostream>
#include <iterator>
#include <fstream>
#include <limits>
#include <algorithm>
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#include <cmath>
#include "Plane.h"
#include <GfxTL/KdTree.h>
#include <GfxTL/CellRangeDataTreeStrategy.h>
#include <GfxTL/IndexedTreeDataKernels.h>
#include <GfxTL/VectorKernel.h>
#include <GfxTL/NullTreeStrategy.h>
#include <GfxTL/BBoxDistanceKdTreeStrategy.h>
#include <GfxTL/IncrementalDistanceKdTreeStrategy.h>
#include <GfxTL/MaxIntervalSplittingKdTreeStrategy.h>
#include <GfxTL/CellBBoxBuildInformationKdTreeStrategy.h>
#include <GfxTL/BBoxBuildInformationTreeStrategy.h>
#include <GfxTL/BucketSizeMaxLevelSubdivisionTreeStrategy.h>
#include <GfxTL/CellLevelTreeStrategy.h>
#include <GfxTL/L2Norm.h>
#include <GfxTL/Plane.h>
#include <GfxTL/VectorXD.h>
#include <GfxTL/IndexedIterator.h>
#include <MiscLib/AlignedAllocator.h>
#include <MiscLib/NoShrinkVector.h>

typedef GfxTL::KdTree
<
	GfxTL::IncrementalDistanceKdTreeStrategy
	<
		GfxTL::MaxIntervalSplittingKdTreeStrategy
		<
			GfxTL::CellBBoxBuildInformationKdTreeStrategy
			<
				GfxTL::BBoxBuildInformationTreeStrategy
				<
					GfxTL::BucketSizeMaxLevelSubdivisionTreeStrategy
					<
						GfxTL::CellLevelTreeStrategy
						<
							GfxTL::BaseKdTreeStrategy
							<
								GfxTL::CellRangeDataTreeStrategy
								<
									GfxTL::NullTreeStrategy,
									GfxTL::IndexedIteratorTreeDataKernel
									<
										PointCloud::const_iterator,
										MiscLib::Vector< size_t >
									>
								>
							>
						>
					>
				>
			>
		>
	>,
	GfxTL::L2Norm,
	GfxTL::VectorKernelD< 3 >::VectorKernelType
> KdTree3Df;

//#define LMS_NORMALS
#define PCA_NORMALS

using namespace std;

/*
 * * This Quickselect routine is based on the algorithm described in
 * * "Numerical recipes in C", Second Edition,
 * * Cambridge University Press, 1992, Section 8.5, ISBN 0-521-43108-5
 * * This code by Nicolas Devillard - 1998. Public domain.
 * */
#define ELEM_SWAP(a,b) { register float t=(a);(a)=(b);(b)=t; }
float quick_select(float arr[], int n)
{
	int low, high ;
	int median;
	int middle, ll, hh;
	low = 0 ; high = n-1 ; median = (low + high) / 2;
	for (;;) {
		if (high <= low) /* One element only */
			return arr[median] ;
		if (high == low + 1) { /* Two elements only */
			if (arr[low] > arr[high])
				ELEM_SWAP(arr[low], arr[high]) ;
			return arr[median] ;
		}
		/* Find median of low, middle and high items; swap into position low */
		middle = (low + high) / 2;
		if (arr[middle] > arr[high]) ELEM_SWAP(arr[middle], arr[high]) ;
		if (arr[low] > arr[high]) ELEM_SWAP(arr[low], arr[high]) ;
		if (arr[middle] > arr[low]) ELEM_SWAP(arr[middle], arr[low]) ;
		/* Swap low item (now in position middle) into position (low+1) */
		ELEM_SWAP(arr[middle], arr[low+1]) ;
		/* Nibble from each end towards middle, swapping items when stuck */
		ll = low + 1;
		hh = high;
		for (;;) {
			do ll++; while (arr[low] > arr[ll]) ;
			do hh--; while (arr[hh] > arr[low]) ;
			if (hh < ll)
				break;
			ELEM_SWAP(arr[ll], arr[hh]) ;
		}
		/* Swap middle item (in position low) back into correct position */
		ELEM_SWAP(arr[low], arr[hh]) ;
		/* Re-set active partition */
		if (hh <= median)
			low = ll;
		if (hh >= median)
		high = hh - 1;
	}
}
#undef ELEM_SWAP

PointCloud::PointCloud()
{
	float fmax = numeric_limits<float>::max();
	float fmin = -fmax;
	m_min = Vec3f(fmax, fmax, fmax);
	m_max = Vec3f(fmin, fmin, fmin);
}

PointCloud::PointCloud(Point *points, unsigned int s)
{
	float fmax = numeric_limits<float>::max();
	float fmin = -fmax;
	m_min = Vec3f(fmax, fmax, fmax);
	m_max = Vec3f(fmin, fmin, fmin);
	std::copy(points, points + s, std::back_inserter(*this));
}

PointCloud::PointCloud(std::string file_path)
{
    happly::PLYData plyIn(file_path);

    std::vector<std::array<double, 3>> vertices = plyIn.getVertexPositions();
    for (auto& vertex: vertices)
    {
        push_back(Point(Vec3f((float) vertex[0], (float) vertex[1], (float) vertex[2])));
    }
    for (size_t i = 0; i < size(); ++i)
        at(i).index = i;
    if (plyIn.hasElement("face"))
    {
        m_faces = plyIn.getElement("face").getListProperty<int>("vertex_indices");
    }
}

void PointCloud::reset(size_t s)
{
	resize(s);
	float fmax = numeric_limits<float>::max();
	float fmin = -fmax;
	m_min = Vec3f(fmax, fmax, fmax);
	m_max = Vec3f(fmin, fmin, fmin);
}

float *PointCloud::getBbox () const
{
	float *bbox = new float[6];
	m_min.getValue(bbox[0], bbox[2], bbox[4]);
	m_max.getValue(bbox[1], bbox[3], bbox[5]);

	return bbox;
}

void PointCloud::adjustBBoxToPoints()
{
    m_min = Vec3f(get_min(0), get_min(1), get_min(2));
    m_max = Vec3f(get_max(0), get_max(1), get_max(2));
}

float PointCloud::findOptimalBitmapEpsFromMeshes()
{
    float max_dist_sq = 0.;
    for (auto const& face: m_faces)
    {
        for (int i = 0; i < face.size() - 1; ++i)
        {
            float dist_sq = at(face[i]).pos.sqrDist(at(face[i + 1]).pos);
            max_dist_sq = std::max(max_dist_sq, dist_sq);
        }
    }
    return sqrt(max_dist_sq);
}

float PointCloud::findOptimalBitmapEpsFromNN(unsigned int kNN)
{
	KdTree3Df kd;
	kd.IndexedData(begin(), end());
	kd.Build();

	GfxTL::LimitedHeap< GfxTL::NN< float > > nn;
	KdTree3Df::NearestNeighborsAuxData< value_type > nnAux;

	float max_edge_length_sq = 0.;

	for ( unsigned int i = 0; i < size(); i ++ )
	{
		kd.NearestNeighbors(at(i), kNN, &nn, &nnAux);
        GfxTL::NN<float> nearest_neigh = *std::max_element(
           nn.begin(), nn.end(),
           [](const GfxTL::NN<float> n1, const GfxTL::NN<float> n2){return n1.sqrDist < n2.sqrDist;});
        max_edge_length_sq = std::max(max_edge_length_sq, nearest_neigh.sqrDist);
    }
    return sqrt(max_edge_length_sq);
}

void PointCloud::GetCurrentBBox(Vec3f *min, Vec3f *max) const
{
	*min = m_min;
	*max = m_max;
}

void PointCloud::Translate(const Vec3f &trans)
{
	for(size_t i = 0; i < size(); ++i)
		at(i).pos += trans;
	m_min += trans;
	m_max += trans;
}

void PointCloud::calcNormals ( float radius, unsigned int kNN, unsigned int maxTries )
{
//	cerr << "Begin calcNormals " << endl << flush;

	KdTree3Df kd;
	kd.IndexedData(begin(), end());
	kd.Build();

	GfxTL::LimitedHeap< GfxTL::NN< float > > nn;
	KdTree3Df::NearestNeighborsAuxData< value_type > nnAux;
	//GfxTL::AssumeUniqueLimitedHeap< GfxTL::NN< float > > nn;
	MiscLib::NoShrinkVector< float > weights;

	vector<int> stats(91, 0);
#ifdef PCA_NORMALS
	GfxTL::Plane< GfxTL::Vector3Df > plane;
#endif
	for ( unsigned int i = 0; i < size(); i ++ )
	{
		//kd.PointsInBall(at(i), radius, &nn);
		//if(nn.size() > kNN)
		//{
		//	std::sort(nn.begin(), nn.end());
		//	nn.resize(kNN);
		//}
		kd.NearestNeighbors(at(i), kNN, &nn, &nnAux);
		unsigned int num = (unsigned int)nn.size();

		//if(i%1000 == 0)
		//	cerr << num << " ";

		if ( num > kNN )
			num = kNN;

		at(i).normal = Vec3f(0,0,0);
		if ( num < 8 ) {

			continue;
		}
			
#ifdef PCA_NORMALS
		weights.resize(nn.size());
		if(nn.front().sqrDist > 0)
		{
			float h = nn.front().sqrDist / 2;
			for(unsigned int i = 0; i < nn.size(); ++i)
				weights[i] = std::exp(-nn[i].sqrDist / h);
		}
		else
			std::fill(weights.begin(), weights.end(), 1.f);
		plane.Fit(GfxTL::IndexIterate(nn.begin(), begin()),
			GfxTL::IndexIterate(nn.end(), begin()), weights.begin());
		at(i).normal = Vec3f(plane.Normal().Data());
#endif

#ifdef LMS_NORMALS
		float score, bestScore = -1.f;
		for (unsigned int tries = 0; tries < maxTries; tries++)
		{
			//choose candidate
			int i0, i1, i2;
			i0 = rand() % num;
			do 
				i1 = rand() % num;
			while (i1 == i0);
			do
				i2 = rand() % num;
			while (i2 == i0 || i2 == i1);

			Plane plane;
			if(!plane.Init(at(nn[i0]), at(nn[i1]), at(nn[i2])))
				continue;
			
			//evaluate metric
			float *dist = new float[num];
			for (unsigned int j = 0; j < num; j++)
			{
				dist[j] = plane.getDistance(at(nn[j]));
			}
	//		sort(dist, dist+num);
		//	score = dist[num/2]; // evaluate median
			score = quick_select(dist, num); // evaluate median
			delete[] dist;

			if (score < bestScore || bestScore < 0.f)
			{
				if ( tries > maxTries/2 ) {
					// let us see how good the first half of candidates are...
					int index = std::floor(180/M_PI*std::acos(std::min(1.f, abs(plane.getNormal().dot(at(i).normal)))));
					stats[index]++;
				}
				at(i).normal = plane.getNormal();
				bestScore = score;
			}

		}
			
#endif
		
	}

	//cerr << "End calcNormals " << endl << flush;
	//copy(stats.begin(), stats.end(), ostream_iterator<int>(cerr, " "));
}

float PointCloud::calcNormalsAndBEps ( float radius, unsigned int kNN, unsigned int maxTries, unsigned int kNN_BEps )
{
//	cerr << "Begin calcNormals " << endl << flush;

	KdTree3Df kd;
	kd.IndexedData(begin(), end());
	kd.Build();

	GfxTL::LimitedHeap< GfxTL::NN< float > > nn;
	KdTree3Df::NearestNeighborsAuxData< value_type > nnAux;
	//GfxTL::AssumeUniqueLimitedHeap< GfxTL::NN< float > > nn;
	MiscLib::NoShrinkVector< float > weights;

	float max_edge_length_sq = 0.;

	vector<int> stats(91, 0);
#ifdef PCA_NORMALS
	GfxTL::Plane< GfxTL::Vector3Df > plane;
#endif
	for ( unsigned int i = 0; i < size(); i ++ )
	{
		//kd.PointsInBall(at(i), radius, &nn);
		//if(nn.size() > kNN)
		//{
		//	std::sort(nn.begin(), nn.end());
		//	nn.resize(kNN);
		//}
		kd.NearestNeighbors(at(i), std::max(kNN, kNN_BEps), &nn, &nnAux);
		unsigned int num = (unsigned int)nn.size();

        // Compute BEps.
        std::sort(nn.begin(), nn.end(),
                  [](const GfxTL::NN<float> n1, const GfxTL::NN<float> n2){return n1.sqrDist < n2.sqrDist;});
        max_edge_length_sq = std::max(max_edge_length_sq, nn[kNN_BEps].sqrDist);

        nn.resize(kNN);  // if kNN_BEps > kNN, resize for the normal calculation

		//if(i%1000 == 0)
		//	cerr << num << " ";

		if ( num > kNN )
			num = kNN;

		at(i).normal = Vec3f(0,0,0);
		if ( num < 8 ) {

			continue;
		}
			
#ifdef PCA_NORMALS
		weights.resize(nn.size());
		if(nn.front().sqrDist > 0)
		{
			float h = nn.front().sqrDist / 2;
			for(unsigned int i = 0; i < nn.size(); ++i)
				weights[i] = std::exp(-nn[i].sqrDist / h);
		}
		else
			std::fill(weights.begin(), weights.end(), 1.f);
		plane.Fit(GfxTL::IndexIterate(nn.begin(), begin()),
			GfxTL::IndexIterate(nn.end(), begin()), weights.begin());
		at(i).normal = Vec3f(plane.Normal().Data());
#endif

#ifdef LMS_NORMALS
		float score, bestScore = -1.f;
		for (unsigned int tries = 0; tries < maxTries; tries++)
		{
			//choose candidate
			int i0, i1, i2;
			i0 = rand() % num;
			do 
				i1 = rand() % num;
			while (i1 == i0);
			do
				i2 = rand() % num;
			while (i2 == i0 || i2 == i1);

			Plane plane;
			if(!plane.Init(at(nn[i0]), at(nn[i1]), at(nn[i2])))
				continue;
			
			//evaluate metric
			float *dist = new float[num];
			for (unsigned int j = 0; j < num; j++)
			{
				dist[j] = plane.getDistance(at(nn[j]));
			}
	//		sort(dist, dist+num);
		//	score = dist[num/2]; // evaluate median
			score = quick_select(dist, num); // evaluate median
			delete[] dist;

			if (score < bestScore || bestScore < 0.f)
			{
				if ( tries > maxTries/2 ) {
					// let us see how good the first half of candidates are...
					int index = std::floor(180/M_PI*std::acos(std::min(1.f, abs(plane.getNormal().dot(at(i).normal)))));
					stats[index]++;
				}
				at(i).normal = plane.getNormal();
				bestScore = score;
			}

		}
			
#endif
		
	}
	return sqrt(max_edge_length_sq);
}
