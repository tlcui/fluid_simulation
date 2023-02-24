#pragma once
#ifndef POINT_HASH_GRID_SEARCHER3
#define POINT_HASH_GRID_SEARCHER3

#include "point_neighbor_searcher3.h"
#include "../common_utils.h"
#include <memory>

class Point_Hash_Grid_Searcher3 : public Point_Neighbor_Searcher3
{
public:
	// construct a hash grid with given resolution and grid spaceing
	// the grid spacing must be 2times or greater than the search radius
	Point_Hash_Grid_Searcher3(const Size3& resolution, double grid_spacing);
	Point_Hash_Grid_Searcher3(size_t resolution_x, size_t resolution_y, size_t resolution_z, double grid_spacing);

	Point_Hash_Grid_Searcher3(const Point_Hash_Grid_Searcher3& other);

	Point_Neighbor_Searcher3_Ptr clone() const override;

	void set(const Point_Hash_Grid_Searcher3& other);

	Point_Hash_Grid_Searcher3& operator = (const Point_Hash_Grid_Searcher3& other);

	// Builds internal acceleration structure for given points list.
	void build(const std::vector<Vector3D>& points) override;

	// Invokes the callback function for each nearby point if it's around the origin within given radius.
	void for_each_nearby_point(
		const Vector3D& origin, // the origin position
		double radius, // the search radius
		const For_Each_Nearby_Point_Func& callback) const override;

	//! Returns true if there exists any nearby points around given origin within radius
	bool has_nearby_points(const Vector3D& origin, double radius) const override;

	// add a point to the hash grid
	void add(const Vector3D& point);

	// return the internal buckets
	// a bucket is an array of points that have the same hash value
	const std::vector<std::vector<size_t>>& get_buckets() const;

	// a bucket index looks like (i,j,k)
	Point3I get_bucket_index(const Vector3D& position) const;

	// Returns the hash value for given 3-D bucket index.
	size_t get_hash_key_from_bucket_index(const Point3I& bucket_index) const;

private:
	double _grid_spacing = 1.0;
	Size3 _resolution = Size3(1, 1, 1);
	std::vector<Vector3D> _points;
	std::vector<std::vector<size_t>> _buckets; // a bucket stores the indiecs of points( in _points) with the same hash value;

	size_t get_hash_key_from_position(const Vector3D& position) const;

	// in 3-dimension space, bucket_indices is an array of size 8
	// given any point, we will have to search for 8 grids around it
	// this function writes the 8 hash keys of the grids into bucket_indices
	void get_nearby_keys(const Vector3D& position, size_t* bucket_indices) const;
};

typedef std::shared_ptr<Point_Hash_Grid_Searcher3> Point_Hash_Grid_Searcher3_Ptr;

#endif