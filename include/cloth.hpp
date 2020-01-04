#pragma once

#include "dde.hpp"
#include "mesh.hpp"

namespace lyra {
    struct Cloth {
        Mesh mesh;
        struct Material {
            double density; // area density
            StretchingSamples stretching;
            BendingData bending;
            double damping; //damping coefficient
            double yield_curv, weakening; // plasticity parameters
        };
        std::vector<Material *> materials;
        std::string texture;
    };

    void compute_masses(Cloth &cloth);
}
