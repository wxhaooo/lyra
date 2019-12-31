#pragma once

#include "mesh.hpp"
#include "vectors.hpp"
#include <map>

namespace lyra {
    typedef std::map<Node *, Vec3> MeshGrad;
    typedef std::map<std::pair<Node *, Node *>, Mat3x3> MeshHess;

    struct Constraint {
        virtual ~Constraint() {};

        virtual double value(int *sign = NULL) = 0;

        virtual MeshGrad gradient() = 0;

        virtual MeshGrad project() = 0;

        // energy function
        virtual double energy(double value) = 0;

        virtual double energy_grad(double value) = 0;

        virtual double energy_hess(double value) = 0;

        // frictional force
        virtual MeshGrad friction(double dt, MeshHess &jac) = 0;
    };

    struct EqCon : public Constraint {
        // n . (node->x - x) = 0
        Node *node;
        Vec3 x, n;
        double stiff;

        double value(int *sign = NULL);

        MeshGrad gradient();

        MeshGrad project();

        double energy(double value);

        double energy_grad(double value);

        double energy_hess(double value);

        MeshGrad friction(double dt, MeshHess &jac);
    };

    struct GlueCon : public Constraint {
        Node *nodes[2];
        Vec3 n;
        double stiff;

        double value(int *sign = NULL);

        MeshGrad gradient();

        MeshGrad project();

        double energy(double value);

        double energy_grad(double value);

        double energy_hess(double value);

        MeshGrad friction(double dt, MeshHess &jac);
    };

    struct IneqCon : public Constraint {
        // n . sum(w[i] verts[i]->x) >= 0
        Node *nodes[4];
        double w[4];
        bool free[4];
        Vec3 n;
        double a; // area
        double mu; // friction
        double stiff;

        double value(int *sign = NULL);

        MeshGrad gradient();

        MeshGrad project();

        double energy(double value);

        double energy_grad(double value);

        double energy_hess(double value);

        //MeshGrad friction(double dt, MeshHess &jac);
    };
}
