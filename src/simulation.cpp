
#include "simulation.hpp"

#include "geometry.hpp"
#include "physics.hpp"

#include "constraint.hpp"

#include "sparse_solver.hpp"
#include <iostream>
#include <fstream>

using namespace std;

namespace lyra {

    static const bool verbose = false;
    static const int proximity = Simulation::Proximity,
            physics = Simulation::Physics,
            strainlimiting = Simulation::StrainLimiting,
            collision = Simulation::Collision,
            remeshing = Simulation::Remeshing,
            separation = Simulation::Separation,
            popfilter = Simulation::PopFilter,
            plasticity = Simulation::Plasticity;

    Simulation::~Simulation() {
        delete_simulation(*this);
    }

    void validate_handles(const Simulation &sim);

    void validate_handles(const Simulation &sim) {
        for (int h = 0; h < sim.handles.size(); h++) {
            vector<Node *> nodes = sim.handles[h]->get_nodes();
            for (int n = 0; n < nodes.size(); n++) {
                if (!nodes[n]->preserve) {
                    cout << "Constrained node " << nodes[n]->index << " will not be preserved by remeshing" << endl;
                    abort();
                }
            }
        }
    }

    vector<Constraint *> get_constraints(Simulation &sim, bool include_proximity);

    void delete_constraints(const vector<Constraint *> &cons);

    void delete_simulation(Simulation &sim) {
        for (int m = 0; m < sim.cloth_meshes.size(); m++) {
            Mesh *mesh = sim.cloth_meshes[m];
            delete_mesh(*mesh);
            // delete mesh; // don't do this since cloth_meshes was not created by new operator.
        }
       
        }

    vector<Constraint *> get_constraints(Simulation &sim, bool include_proximity) {
        vector<Constraint *> cons;
        for (int h = 0; h < sim.handles.size(); h++)
            append(cons, sim.handles[h]->get_constraints(sim.time));
        for (int m = 0; m < sim.cloth_meshes.size(); m++) {
            mark_nodes_to_preserve(*sim.cloth_meshes[m]);
        }
        return cons;
    }

    void delete_constraints(const vector<Constraint *> &cons) {
        for (int c = 0; c < cons.size(); c++)
            delete cons[c];
    }

// Steps

    void update_velocities(vector<Mesh *> &meshes, vector<Vec3> &xold, double dt);

    void compute_avg_velocities(std::vector<Mesh *> &meshes, double dt);

    void step_mesh(Mesh &mesh, double dt);


    void AddExternalForce(Simulation &sim, const vector<Constraint *> &cons, std::vector<Vec3> &fext,
                            std::vector<Mat3x3> &Jext, int c) {

        int nn = sim.cloths[c].mesh.nodes.size();
        vector<Vec3> fext2(nn, Vec3(0));
        vector<Mat3x3> Jext2(nn, Mat3x3(0));
        fext = fext2;
        Jext = Jext2;
        add_external_forces(sim.cloths[c], sim.gravity, sim.wind, fext, Jext);
    }

    enum CompareType {
        DIFFERENT,
        EQUAL,
        EARLIER,
        LATER
    };

    bool find_in_mesh3(const Node *node, const Mesh &mesh) {
        for (int n = 0; n < mesh.nodes.size(); n++) {
            if (node == mesh.nodes[n]) {
                return true;
            }
        }
        return false;
    }

    double smallest(Mesh &mesh) {
        double min = 100;
        Vec3 vi, vj;
        int di, dj;
        for (int i = 0; i < mesh.verts.size(); i++) {
            for (int j = i + 1; j < mesh.verts.size(); j++) {
                Vert *v1 = mesh.verts[i];
                Vert *v2 = mesh.verts[j];
                double dis = norm(v1->u - v2->u);
                if (dis < min) {
                    min = dis;
                    di = i;
                    dj = j;
                    vi = v1->node->x;
                    vj = v2->node->x;
                }
            }
        }
        // VisualDebugger *vd = VisualDebugger::getInstance();
        // vd->addVisualPoint3(mesh.verts[di]->node->x, Vec3(1, 0, 0), 'l');
        // vd->addVisualPoint3(mesh.verts[dj]->node->x, Vec3(1, 0, 0), 'l');
        return min;
    }

    double smallest_ve(Mesh &mesh) {
        double min = 100;
        for (int f = 0; f < mesh.faces.size(); f++) {
            Face *face = mesh.faces[f];
            for (int i = 0; i < 3; i++) {
                Vec2 proj;
                double d = material_ve_projection(face->v[i]->u, face->v[(i + 1) % 3]->u, face->v[(i + 2) % 3]->u,
                                                  proj);
                if (d >= 0 && d < min) {
                    min = d;
                }
            }
        }
        return min;
    }

    void print_mesh(Mesh &mesh) {
        for (int i = 0; i < mesh.nodes.size(); i++) {
            Node *node = mesh.nodes[i];
            cout << "x:" << node->x << endl;
            cout << "v:" << node->v << endl;
            cout << "u:" << node->verts[0]->u << endl;
            cout << "y:" << node->y << endl;
            cout << "temp:" << node->temp << endl;
            cout << "temp2:" << node->temp2 << endl;
            cout << "adje size:" << node->adje.size() << endl;
            for (int j = 0; j < node->adje.size(); j++) {
                Edge *e = node->adje[j];
                cout << "\t" << e->index << endl;
            }
            cout << "n:" << node->n << endl;
        }
        for (int i = 0; i < mesh.faces.size(); i++) {
            Face *face = mesh.faces[i];
            cout << "node index:";
            for (int i = 0; i < 3; i++) {
                cout << "\t" << face->v[i]->node->index;
            }
            cout << endl;
            for (int i = 0; i < 3; i++) {
                cout << "\t" << face->v[i]->index;
            }
            cout << endl;
            // cout << "adje:" << endl;
            // cout << "\t" << (face->adje[0] ? face->adje[0]->index : -1) << endl;
            // cout << "\t" << (face->adje[1] ? face->adje[1]->index : -1) << endl;
            // cout << "\t" << (face->adje[2] ? face->adje[2]->index : -1) << endl;
        }
        // for (int i = 0; i < mesh.edges.size(); i++) {
        // 	Edge* edge = mesh.edges[i];
        // 	cout << "adjf:" << endl;
        // 	cout << "\t" << (edge->adjf[0] ? edge->adjf[0]->index : -1) << endl;
        // 	cout << "\t" << (edge->adjf[1] ? edge->adjf[1]->index : -1) << endl;
        // }
        for (int i = 0; i < mesh.verts.size(); i++) {
            Vert *vert = mesh.verts[i];
            cout << "adjf size:" << vert->adjf.size() << endl;
            for (int j = 0; j < vert->adjf.size(); j++) {
                Face *f = vert->adjf[j];
                cout << "\t" << f->index << endl;
            }
        }
        cout << endl;
    }

    void step_mesh(Mesh &mesh, double dt) {
        for (int n = 0; n < mesh.nodes.size(); n++)
            mesh.nodes[n]->x += mesh.nodes[n]->v * dt;
    }

    void update_velocities(vector<Mesh *> &meshes, vector<Vec3> &xold, double dt) {
        double inv_dt = 1 / dt;
#pragma omp parallel for
        for (int n = 0; n < xold.size(); n++) {
            Node *node = get<Node>(n, meshes);
            node->v += (node->x - xold[n]) * inv_dt;
        }
    }

    void compute_avg_velocities(std::vector<Mesh *> &meshes, double dt) {

/*	double inv_dt = 1.0/dt;
#pragma omp parallel for
	for(int m = 0; m < meshes.size(); m++){
		for(int n = 0; n < meshes[m]->nodes.size(); n++){
			Node* node = meshes[m]->nodes[n];
			node->vMid = inv_dt * ( node->x - node->x0 );
		}
	}
	*/

    }

// Helper functions

    template<typename Prim>
    int size(const vector<Mesh *> &meshes) {
        int np = 0;
        for (int m = 0; m < meshes.size(); m++) np += get<Prim>(*meshes[m]).size();
        return np;
    }

    template int size<Vert>(const vector<Mesh *> &);

    template int size<Node>(const vector<Mesh *> &);

    template int size<Edge>(const vector<Mesh *> &);

    template int size<Face>(const vector<Mesh *> &);

    template<typename Prim>
    int get_index(const Prim *p,
                  const vector<Mesh *> &meshes) {
        int i = 0;
        for (int m = 0; m < meshes.size(); m++) {
            const vector<Prim *> &ps = get<Prim>(*meshes[m]);
            if (p->index < ps.size() && p == ps[p->index])
                return i + p->index;
            else i += ps.size();
        }
        return -1;
    }

    template int get_index(const Vert *, const vector<Mesh *> &);

    template int get_index(const Node *, const vector<Mesh *> &);

    template int get_index(const Edge *, const vector<Mesh *> &);

    template int get_index(const Face *, const vector<Mesh *> &);

    template<typename Prim>
    Prim *get(int i, const vector<Mesh *> &meshes) {
        for (int m = 0; m < meshes.size(); m++) {
            const vector<Prim *> &ps = get<Prim>(*meshes[m]);
            if (i < ps.size())
                return ps[i];
            else
                i -= ps.size();
        }
        return NULL;
    }

    template Vert *get(int, const vector<Mesh *> &);

    template Node *get(int, const vector<Mesh *> &);

    template Edge *get(int, const vector<Mesh *> &);

    template Face *get(int, const vector<Mesh *> &);

    vector<Vec3> node_positions(const vector<Mesh *> &meshes) {
        vector<Vec3> xs(size<Node>(meshes));
        for (int n = 0; n < xs.size(); n++)
            xs[n] = get<Node>(n, meshes)->x;
        return xs;
    }

    vector<Vec3> node_avg_velocities(const vector<Mesh *> &meshes, double dt) {
        vector<Vec3> vs(size<Node>(meshes));
        for (int n = 0; n < vs.size(); n++) {
            vs[n] = (get<Node>(n, meshes)->x - get<Node>(n, meshes)->x0) / dt;
        }
        return vs;
    }

}
