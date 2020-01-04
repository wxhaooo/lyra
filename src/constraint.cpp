

#include "constraint.hpp"

#include "magic.hpp"
using namespace std;

namespace lyra {

    double EqCon::value(int *sign) {
        if (sign) *sign = 0;
        return dot(n, node->x - x);
    }

    MeshGrad EqCon::gradient() {
        MeshGrad grad;
        grad[node] = n;
        return grad;
    }

    MeshGrad EqCon::project() { return MeshGrad(); }

    double EqCon::energy(double value) { return stiff * sq(value) / 2.; }

    double EqCon::energy_grad(double value) { return stiff * value; }

    double EqCon::energy_hess(double value) { return stiff; }

    MeshGrad EqCon::friction(double dt, MeshHess &jac) { return MeshGrad(); }

    double GlueCon::value(int *sign) {
        if (sign) *sign = 0;
        return dot(n, nodes[1]->x - nodes[0]->x);
    }

    MeshGrad GlueCon::gradient() {
        MeshGrad grad;
        grad[nodes[0]] = -n;
        grad[nodes[1]] = n;
        return grad;
    }

    MeshGrad GlueCon::project() { return MeshGrad(); }

    double GlueCon::energy(double value) { return stiff * sq(value) / 2.; }

    double GlueCon::energy_grad(double value) { return stiff * value; }

    double GlueCon::energy_hess(double value) { return stiff; }

    MeshGrad GlueCon::friction(double dt, MeshHess &jac) { return MeshGrad(); }

    double IneqCon::value(int *sign) {
        if (sign)
            *sign = 1;
        double d = 0;
        for (int i = 0; i < 4; i++)
            d += w[i] * dot(n, nodes[i]->x);
        d -= lyra::magic.repulsion_thickness;
        return d;
    }

    MeshGrad IneqCon::gradient() {
        MeshGrad grad;
        for (int i = 0; i < 4; i++)
            grad[nodes[i]] = w[i] * n;
        return grad;
    }

    MeshGrad IneqCon::project() {
        double d = value() + lyra::magic.repulsion_thickness - lyra::magic.projection_thickness;
        if (d >= 0)
            return MeshGrad();
        double inv_mass = 0;
        for (int i = 0; i < 4; i++)
            if (free[i])
                inv_mass += sq(w[i]) / nodes[i]->m;
        MeshGrad dx;
        for (int i = 0; i < 4; i++)
            if (free[i])
                dx[nodes[i]] = -(w[i] / nodes[i]->m) / inv_mass * n * d;
        return dx;
    }

    double violation(double value) { return std::max(-value, 0.); }

    double IneqCon::energy(double value) {
        double v = violation(value);
        return stiff * v * v * v / lyra::magic.repulsion_thickness / 6;
    }

    double IneqCon::energy_grad(double value) {
        return -stiff * sq(violation(value)) / lyra::magic.repulsion_thickness / 2;
    }

    double IneqCon::energy_hess(double value) {
        return stiff * violation(value) / lyra::magic.repulsion_thickness;
    }
}