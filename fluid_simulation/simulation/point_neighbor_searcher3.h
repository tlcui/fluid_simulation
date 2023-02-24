#pragma once
#ifndef POINT_NEIGHBOR_SEARCHER3_
#define POINT_NEIGHBOR_SEARCHER3_


#include <vector>
#include <functional>
#include <string>
#include <Eigen/Dense>
#include "../common_utils.h"

class Point_Neighbor_Searcher3
{
public:

    // !Callback function for nearby search query.The first parameter is the
    // !index of the nearby point, and the second is the position of the point.
    typedef std::function<void(size_t, const Vector3D&)>
        For_Each_Nearby_Point_Func;

    Point_Neighbor_Searcher3();
    virtual ~Point_Neighbor_Searcher3();

    //! Returns the type name of the derived class.
    // virtual std::string get_typename() const = 0;

    //! Builds internal acceleration structure for given points list.
    virtual void build(const std::vector<Vector3D>& points) = 0;

    //! Invokes the callback function for each nearby point if it's around the origin within given radius.
    virtual void for_each_nearby_point(
        const Vector3D& origin, // the origin position
        double radius, // the search radius
        const For_Each_Nearby_Point_Func& callback) const = 0;

    //! Returns true if there exists any nearby points around given origin within radius
    virtual bool has_nearby_points(const Vector3D& origin, double radius) const = 0;

    //! Creates a new instance of the object with the same properties as original
    virtual std::shared_ptr<Point_Neighbor_Searcher3> clone() const = 0;
};

typedef std::shared_ptr<Point_Neighbor_Searcher3> Point_Neighbor_Searcher3_Ptr;

#endif