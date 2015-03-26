#ifndef DEFS_H_INCLUDED
#define DEFS_H_INCLUDED

#include <cstdint>
#include <vector>
#include <memory>
#include <cmath>
#include "../../3rdParty/Eigen/Core"
#include "../../3rdParty/Eigen/Geometry"

/// coldet name space
namespace coldet {

	/// coldet default floating point
	typedef double float_type;

	/// 3 element vector class
	typedef Eigen::Translation <float_type, 3> Vec3;

	/// Homogeneous representation of SE(3) rigid body transformations
	typedef Eigen::Transform <double, 3, Eigen::Affine> Mat34;

	/// Quaternion representation of SO(3) group of rotations
    typedef Eigen::Quaternion<float_type> Quaternion;

	class CElevationMap{

	public:
		float_type size_X, size_Y;
		float_type raster_X, raster_Y;
		size_t  numRows, numCols;

	protected:
		std::vector < std::vector<float_type>>map;

	};

}

#endif // DEFS_H_INCLUDED