#pragma once

#include "cloth.hpp"
#include "geometry.hpp"
#include "simulation.hpp"
#include <vector>

namespace lyra {

    struct ForceCollection {
        Vec3 stretchForce;
        Vec3 bendingForce;
    };

	template<Space s>
	void add_internal_forces(const Cloth &cloth, SpMat<Mat3x3> &A,
							 std::vector<Vec3> &b, double dt);

	template<Space s>
	void add_internal_forces(const std::vector<Face *> &faces,
							 const std::vector<Edge *> &edges,
							 SpMat<Mat3x3> &A, std::vector<Vec3> &b, double dt);

	void add_constraint_forces(const Cloth &cloth,
							   const std::vector<Constraint *> &cons,
							   SpMat<Mat3x3> &A, std::vector<Vec3> &b, double dt);

	void add_external_forces(const Cloth &cloth, const Vec3 &gravity,
							 const Wind &wind, std::vector<Vec3> &fext,
							 std::vector<Mat3x3> &Jext);

	pair<SpMat<Mat3x3>, vector<Vec3> > ObtainImplicitEquation(Cloth &cloth, const vector<Vec3> &fext,
																const vector<Mat3x3> &Jext,
																const vector<Constraint *> &cons, double dt, ForceCollection forces = ForceCollection());
}
