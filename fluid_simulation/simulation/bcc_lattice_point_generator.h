#pragma once
#ifndef BCC_LATTICE_POING_GENERATOR_H_
#define BCC_LATTICE_POING_GENERATOR_H_

#include "point_generator3.h"

//! \brief Body-centered lattice points generator.
class BccLatticePointGenerator final : public PointGenerator3 {
public:
    //!
    //! \brief Invokes \p callback function for each BCC-lattice points inside
    //! \p boundingBox.
    //!
    //! This function iterates every BCC-lattice points inside \p boundingBox
    //! where \p spacing is the size of the unit cell of BCC structure.
    //!
    void forEachPoint(
        const BoundingBox3D& boundingBox,
        double spacing,
        const std::function<bool(const Vector3D&)>& callback) const override;
};


#endif 