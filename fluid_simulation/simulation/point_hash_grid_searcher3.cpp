#include "point_hash_grid_searcher3.h"
#include <algorithm>

Point_Hash_Grid_Searcher3::Point_Hash_Grid_Searcher3(const Size3& resolution, double grid_spacing):
	Point_Hash_Grid_Searcher3(resolution.x(), resolution.y(), resolution.z(), grid_spacing)
{

}

Point_Hash_Grid_Searcher3::Point_Hash_Grid_Searcher3(size_t resolution_x, size_t resolution_y, size_t resolution_z, double grid_spacing):
	_grid_spacing(grid_spacing)
{
	_resolution.x() = std::max(static_cast<ssize_t>(resolution_x), static_cast <ssize_t>(1));
	_resolution.y() = std::max(static_cast<ssize_t>(resolution_x), static_cast <ssize_t>(1));
	_resolution.z() = std::max(static_cast<ssize_t>(resolution_x), static_cast <ssize_t>(1));
}

Point_Hash_Grid_Searcher3::Point_Hash_Grid_Searcher3(const Point_Hash_Grid_Searcher3& other)
{
	set(other);
}

Point_Neighbor_Searcher3_Ptr Point_Hash_Grid_Searcher3::clone() const
{
	return std::make_shared<Point_Hash_Grid_Searcher3>(*this);
}

void Point_Hash_Grid_Searcher3::set(const Point_Hash_Grid_Searcher3& other)
{
	_grid_spacing = other._grid_spacing;
	_resolution = other._resolution;
	_points = other._points;
	_buckets = other._buckets;
}

Point_Hash_Grid_Searcher3& Point_Hash_Grid_Searcher3::operator=(const Point_Hash_Grid_Searcher3& other)
{
	set(other);
	return *this;
}

void Point_Hash_Grid_Searcher3::build(const std::vector<Vector3D>& points)
{
	_buckets.clear();
	_points.clear();

	_buckets.resize(_resolution.x() * _resolution.y() * _resolution.z());
	_points.resize(points.size(), Vector3D::Zero());

	if (points.empty())
	{
		return;
	}

	size_t points_size = points.size();
	for (size_t i = 0; i < points_size; ++i)
	{
		_points[i] = points[i];
		size_t key = get_hash_key_from_position(points[i]);
		_buckets[key].push_back(i);
	}
}

void Point_Hash_Grid_Searcher3::for_each_nearby_point(const Vector3D& origin, double radius, const For_Each_Nearby_Point_Func& callback) const
{
	if (_buckets.empty())
	{
		return;
	}

	size_t nearby_keys[8];
	get_nearby_keys(origin, nearby_keys);

	const double radius_squared = radius * radius;

	for (size_t i = 0; i < 8; ++i)
	{
		const auto& bucket = _buckets[nearby_keys[i]];
		size_t number_of_points_in_bucket = bucket.size();

		for (size_t j = 0; j < number_of_points_in_bucket; ++j)
		{
			size_t point_index = bucket[j];
			double distance_squared = (_points[point_index] - origin).squaredNorm();
			if (distance_squared <= radius_squared)
			{
				callback(point_index, _points[point_index]);
			}
		}
	}
}

bool Point_Hash_Grid_Searcher3::has_nearby_points(const Vector3D& origin, double radius) const
{
	if (_buckets.empty())
	{
		return false;
	}

	size_t nearby_keys[8];
	memset(nearby_keys, 0x00, sizeof(nearby_keys));
	get_nearby_keys(origin, nearby_keys);

	const double radius_squared = radius * radius;

	for (size_t i = 0; i < 8; ++i)
	{
		const auto& bucket = _buckets[nearby_keys[i]];
		size_t number_of_points_in_bucket = bucket.size();

		for (size_t j = 0; j < number_of_points_in_bucket; ++j)
		{
			size_t point_index = bucket[j];
			double distance_squared = (_points[point_index] - origin).squaredNorm();
			if (distance_squared <= radius_squared)
			{
				return true;
			}
		}
	}
	return false;
}

void Point_Hash_Grid_Searcher3::add(const Vector3D& point)
{
	if (_buckets.empty())
	{
		std::vector<Vector3D> arr = { point };
		build(arr);
	}
	else
	{
		size_t i = _points.size();
		_points.push_back(point);
		size_t key = get_hash_key_from_position(point);
		_buckets[key].push_back(i);
	}
}

const std::vector<std::vector<size_t>>& Point_Hash_Grid_Searcher3::get_buckets() const
{
	return _buckets;
}

Point3I Point_Hash_Grid_Searcher3::get_bucket_index(const Vector3D& position) const
{
	Point3I bucket_index = Point3I::Zero();
	bucket_index.x() = static_cast<ssize_t>(
		std::floor(position.x() / _grid_spacing));
	bucket_index.y() = static_cast<ssize_t>(
		std::floor(position.y() / _grid_spacing));
	bucket_index.z() = static_cast<ssize_t>(
		std::floor(position.z() / _grid_spacing));
	return bucket_index;
}

size_t Point_Hash_Grid_Searcher3::get_hash_key_from_bucket_index(const Point3I& bucket_index) const
{
	Point3I wrapped_index = bucket_index;
	wrapped_index.x() = bucket_index.x() % _resolution.x();
	wrapped_index.y() = bucket_index.y() % _resolution.y();
	wrapped_index.z() = bucket_index.z() % _resolution.z();
	if (wrapped_index.x() < 0)
	{
		wrapped_index.x() += _resolution.x();
	}
	if (wrapped_index.y() < 0)
	{
		wrapped_index.y() += _resolution.y();
	}
	if (wrapped_index.z() < 0)
	{
		wrapped_index.z() += _resolution.z();
	}
	return static_cast<size_t>(
		(wrapped_index.z() * _resolution.y() + wrapped_index.y()) * _resolution.x() + wrapped_index.x()
		);
}

size_t Point_Hash_Grid_Searcher3::get_hash_key_from_position(const Vector3D& position) const
{
	return get_hash_key_from_bucket_index(get_bucket_index(position));
}

void Point_Hash_Grid_Searcher3::get_nearby_keys(const Vector3D& position, size_t* bucket_indices) const
{
	Point3I originIndex = get_bucket_index(position);
	Point3I nearbyBucketIndices[8];

	for (int i = 0; i < 8; i++) {
		nearbyBucketIndices[i] = originIndex;
	}

	if ((originIndex.x() + 0.5) * _grid_spacing <= position.x()) 
	{
		nearbyBucketIndices[4].x() += 1;
		nearbyBucketIndices[5].x() += 1;
		nearbyBucketIndices[6].x() += 1;
		nearbyBucketIndices[7].x() += 1;
	}
	else
	{
		nearbyBucketIndices[4].x() -= 1;
		nearbyBucketIndices[5].x() -= 1;
		nearbyBucketIndices[6].x() -= 1;
		nearbyBucketIndices[7].x() -= 1;
	}

	if ((originIndex.y() + 0.5) * _grid_spacing <= position.y())
	{
		nearbyBucketIndices[2].y() += 1;
		nearbyBucketIndices[3].y() += 1;
		nearbyBucketIndices[6].y() += 1;
		nearbyBucketIndices[7].y() += 1;
	}
	else 
	{
		nearbyBucketIndices[2].y() -= 1;
		nearbyBucketIndices[3].y() -= 1;
		nearbyBucketIndices[6].y() -= 1;
		nearbyBucketIndices[7].y() -= 1;
	}

	if ((originIndex.z() + 0.5) * _grid_spacing <= position.z())
	{
		nearbyBucketIndices[1].z() += 1;
		nearbyBucketIndices[3].z() += 1;
		nearbyBucketIndices[5].z() += 1;
		nearbyBucketIndices[7].z() += 1;
	}
	else
	{
		nearbyBucketIndices[1].z() -= 1;
		nearbyBucketIndices[3].z() -= 1;
		nearbyBucketIndices[5].z() -= 1;
		nearbyBucketIndices[7].z() -= 1;
	}

	for (int i = 0; i < 8; i++) {
		bucket_indices[i] = get_hash_key_from_bucket_index(nearbyBucketIndices[i]);
	}
}
