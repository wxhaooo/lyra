#include "cloth.hpp"
using namespace std;


namespace lyra {

    void compute_masses(Cloth &cloth) {
        for (int v = 0; v < cloth.mesh.verts.size(); v++)
            cloth.mesh.verts[v]->m = 0;
        for (int n = 0; n < cloth.mesh.nodes.size(); n++)
            cloth.mesh.nodes[n]->m = 0;
        for (int f = 0; f < cloth.mesh.faces.size(); f++) {
            Face *face = cloth.mesh.faces[f];
            face->m = face->a * cloth.materials[face->label]->density;
            for (int v = 0; v < 3; v++) {
                face->v[v]->m += face->m / 3.;
                face->v[v]->node->m += face->m / 3.;
            }
        }
    }
}