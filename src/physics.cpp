/*
  Copyright ©2013 The Regents of the University of California
  (Regents). All Rights Reserved. Permission to use, copy, modify, and
  distribute this software and its documentation for educational,
  research, and not-for-profit purposes, without fee and without a
  signed licensing agreement, is hereby granted, provided that the
  above copyright notice, this paragraph and the following two
  paragraphs appear in all copies, modifications, and
  distributions. Contact The Office of Technology Licensing, UC
  Berkeley, 2150 Shattuck Avenue, Suite 510, Berkeley, CA 94720-1620,
  (510) 643-7201, for commercial licensing opportunities.

  IN NO EVENT SHALL REGENTS BE LIABLE TO ANY PARTY FOR DIRECT,
  INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, INCLUDING
  LOST PROFITS, ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS
  DOCUMENTATION, EVEN IF REGENTS HAS BEEN ADVISED OF THE POSSIBILITY
  OF SUCH DAMAGE.

  REGENTS SPECIFICALLY DISCLAIMS ANY WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
  FOR A PARTICULAR PURPOSE. THE SOFTWARE AND ACCOMPANYING
  DOCUMENTATION, IF ANY, PROVIDED HEREUNDER IS PROVIDED "AS
  IS". REGENTS HAS NO OBLIGATION TO PROVIDE MAINTENANCE, SUPPORT,
  UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
*/

#include "physics.hpp"
#include "blockvectors.hpp"
#include "sparse.hpp"
#include "sparse_solver.hpp"

using namespace std;

namespace lyra {
    static const vector<Cloth::Material *> *materials;

    typedef Mat<9, 9> Mat9x9;
    typedef Mat<9, 6> Mat9x6;
    typedef Mat<6, 6> Mat6x6;
    typedef Mat<4, 6> Mat4x6;
    typedef Mat<3, 4> Mat3x4;
    typedef Mat<4, 9> Mat4x9;
    typedef Vec<9> Vec9;

// A kronecker B = [a11 B, a12 B, ..., a1n B;
//                  a21 B, a22 B, ..., a2n B;
//                   ... ,  ... , ...,  ... ;
//                  am1 B, am2 B, ..., amn B]
    template<int m, int n, int p, int q>
    Mat<m * p, n * q> kronecker(const Mat<m, n> &A, const Mat<p, q> &B) {
        Mat<m * p, n * q> C;
        for (int i = 0; i < m; i++)
            for (int j = 0; j < n; j++)
                for (int k = 0; k < p; k++)
                    for (int l = 0; l < q; l++)
                        C(i * p + k, j * q + l) = A(i, j) * B(k, l);
        return C;
    }

    template<int m>
    Mat<m, 1> colmat(const Vec<m> &v) {
        Mat<1, m> A;
        for (int i = 0; i < m; i++) A(i, 0) = v[i];
        return A;
    }

    template<int n>
    Mat<1, n> rowmat(const Vec<n> &v) {
        Mat<1, n> A;
        for (int i = 0; i < n; i++) A(0, i) = v[i];
        return A;
    }

    typedef Mat<12, 12> Mat12x12;
    typedef Vec<12> Vec12;

    double distance(const Vec3 &x, const Vec3 &a, const Vec3 &b) {
        Vec3 e = b - a;
        Vec3 xp = e * dot(e, x - a) / dot(e, e);
        // return norm((x-a)-xp);
        return std::max(norm((x - a) - xp), 1e-3 * norm(e));
    }

    Vec2 barycentric_weights(const Vec3 &x, const Vec3 &a, const Vec3 &b) {
        Vec3 e = b - a;
        double t = dot(e, x - a) / dot(e, e);
        return Vec2(1 - t, t);
    }

	template<Space s>
	pair<Mat9x9, Vec9> stretching_force(const Face* face) {
		Mat3x2 F = derivative(pos<s>(face->v[0]->node), pos<s>(face->v[1]->node),
			pos<s>(face->v[2]->node), face);
		Mat2x2 G = (F.t() * F - Mat2x2(1)) / 2.;
		Vec4 k = stretching_stiffness(G, (*materials)[face->label]->stretching);
		double weakening = (*materials)[face->label]->weakening;
		k *= 1 / (1 + weakening * face->damage);
		// eps = 1/2(F'F - I) = 1/2([x_u^2 & x_u x_v \\ x_u x_v & x_v^2] - I)
		// e is energy
		// e = 1/2 k0 eps00^2 + k1 eps00 eps11 + 1/2 k2 eps11^2 + k3 eps01^2
		// grad e = k0 eps00 grad eps00 + ...
		//        = k0 eps00 Du' x_u + ...
		Mat2x3 D = derivative(face);
		Vec3 du = D.row(0), dv = D.row(1);
		Mat<3, 9> Du = kronecker(rowmat(du), Mat3x3(1)),
			Dv = kronecker(rowmat(dv), Mat3x3(1));
		const Vec3& xu = F.col(0), & xv = F.col(1);
		Vec9 fuu = Du.t() * xu, fvv = Dv.t() * xv, fuv = (Du.t() * xv + Dv.t() * xu) / 2.;
		Vec9 grad_e = k[0] * G(0, 0) * fuu + k[2] * G(1, 1) * fvv
			+ k[1] * (G(0, 0) * fvv + G(1, 1) * fuu) + 2 * k[3] * G(0, 1) * fuv;
		Mat9x9 hess_e = k[0] * (outer(fuu, fuu) + std::max(G(0, 0), 0.) * Du.t() * Du)
			+ k[2] * (outer(fvv, fvv) + std::max(G(1, 1), 0.) * Dv.t() * Dv)
			+ k[1] * (outer(fuu, fvv) + std::max(G(0, 0), 0.) * Dv.t() * Dv
				+ outer(fvv, fuu) + std::max(G(1, 1), 0.) * Du.t() * Du)
			+ 2. * k[3] * (outer(fuv, fuv));
	
		return make_pair(-face->a * hess_e, -face->a * grad_e);
	}
	
    template<Space s>
    pair<Mat12x12, Vec12> bending_force(const Edge *edge) {
        const Face *face0 = edge->adjf[0], *face1 = edge->adjf[1];
        if (!face0 || !face1)
            return make_pair(Mat12x12(0), Vec12(0));
        double theta = dihedral_angle<s>(edge);
        double a = face0->a + face1->a;
        Vec3 x0 = pos<s>(edge->n[0]),
                x1 = pos<s>(edge->n[1]),
                x2 = pos<s>(edge_opp_vert(edge, 0)->node),
                x3 = pos<s>(edge_opp_vert(edge, 1)->node);
        double h0 = distance(x2, x0, x1), h1 = distance(x3, x0, x1);
        Vec3 n0 = nor<s>(face0), n1 = nor<s>(face1);
        Vec2 w_f0 = barycentric_weights(x2, x0, x1),
                w_f1 = barycentric_weights(x3, x0, x1);
        Vec12 dtheta = mat_to_vec(Mat3x4(-(w_f0[0] * n0 / h0 + w_f1[0] * n1 / h1),
                                         -(w_f0[1] * n0 / h0 + w_f1[1] * n1 / h1),
                                         n0 / h0,
                                         n1 / h1));
        const BendingData &bend0 = (*lyra::materials)[face0->label]->bending,
                &bend1 = (*lyra::materials)[face1->label]->bending;
        double ke = std::min(bending_stiffness(edge, 0, bend0),
                        bending_stiffness(edge, 1, bend1));
        double weakening = std::max((*lyra::materials)[face0->label]->weakening,
                               (*lyra::materials)[face1->label]->weakening);
        ke *= 1 / (1 + weakening * edge->damage);
        double shape = sq(edge->l) / (2 * a);
        return make_pair(-ke * shape * outer(dtheta, dtheta) / 2.,
                         -ke * shape * (theta - edge->theta_ideal) * dtheta / 2.);
    }

    template<int m, int n>
    Mat<3, 3> submat3(const Mat<m, n> &A, int i, int j) {
        Mat3x3 Asub;
        for (int k = 0; k < 3; k++)
            for (int l = 0; l < 3; l++)
                Asub(k, l) = A(i * 3 + k, j * 3 + l);
        return Asub;
    }

    template<int n>
    Vec<3> subvec3(const Vec<n> &b, int i) {
        Vec3 bsub;
        for (int k = 0; k < 3; k++)
            bsub[k] = b[i * 3 + k];
        return bsub;
    }

    template<int m>
    void add_submat(const Mat<m * 3, m * 3> &Asub, const Vec<m, int> &ix, SpMat<Mat3x3> &A) {
        for (int i = 0; i < m; i++) {
            if (ix[i] < 0) continue;
            for (int j = 0; j < m; j++) {
                if (ix[j] < 0) continue;
                A(ix[i], ix[j]) += submat3(Asub, i, j);
            }
        }
    }

    template<int m>
    void add_subvec(const Vec<m * 3> &bsub, const Vec<m, int> &ix, vector<Vec3> &b) {
        for (int i = 0; i < m; i++) {
            if (ix[i] < 0) continue;
            b[ix[i]] += subvec3(bsub, i);
        }
    }

    Vec<3, int> indices(const Node *n0, const Node *n1, const Node *n2) {
        Vec<3, int> ix;
        ix[0] = n0->active ? n0->index : -1;
        ix[1] = n1->active ? n1->index : -1;
        ix[2] = n2->active ? n2->index : -1;
        return ix;
    }

    Vec<4, int> indices(const Node *n0, const Node *n1,
                        const Node *n2, const Node *n3) {
        Vec<4, int> ix;
        ix[0] = n0->active ? n0->index : -1;
        ix[1] = n1->active ? n1->index : -1;
        ix[2] = n2->active ? n2->index : -1;
        ix[3] = n3->active ? n3->index : -1;
        return ix;
    }

//aa: use 3-compressed sparse matrix
#define USE_SPARSE3

    template<Space s>
    void add_internal_forces(const std::vector<Face *> &faces,
                             const std::vector<Edge *> &edges,
                             SpMat<Mat3x3> &A, std::vector<Vec3> &b, double dt) {
        lyra::materials = &faces[0]->v[0]->node->mesh->parent->materials;
        for (int f = 0; f < faces.size(); f++) {
            const Face *face = faces[f];
            const Node *n0 = face->v[0]->node, *n1 = face->v[1]->node,
                    *n2 = face->v[2]->node;
            Vec9 vs = mat_to_vec(Mat3x3(n0->v, n1->v, n2->v));
            pair<Mat9x9, Vec9> membF = stretching_force<s>(face);
            Mat9x9 J = membF.first;
            Vec9 F = membF.second;
            if (dt == 0) {
                add_submat(-J, indices(n0, n1, n2), A);
                add_subvec(F, indices(n0, n1, n2), b);
            } else {
                double damping = (*lyra::materials)[face->label]->damping;
                add_submat(-dt * (dt + damping) * J, indices(n0, n1, n2), A);
                add_subvec(dt * (F + (dt + damping) * J * vs), indices(n0, n1, n2), b);
            }
        }
        for (int e = 0; e < edges.size(); e++) {
            const Edge *edge = edges[e];
            if (!edge->adjf[0] || !edge->adjf[1])
                continue;
            pair<Mat12x12, Vec12> bendF = bending_force<s>(edge);
            const Node *n0 = edge->n[0],
                    *n1 = edge->n[1],
                    *n2 = edge_opp_vert(edge, 0)->node,
                    *n3 = edge_opp_vert(edge, 1)->node;
            Vec12 vs = mat_to_vec(Mat3x4(n0->v, n1->v, n2->v, n3->v));
            Mat12x12 J = bendF.first;
            Vec12 F = bendF.second;
            if (dt == 0) {
                add_submat(-J, indices(n0, n1, n2, n3), A);
                add_subvec(F, indices(n0, n1, n2, n3), b);
            } else {
                double damping = ((*lyra::materials)[edge->adjf[0]->label]->damping +
                                  (*lyra::materials)[edge->adjf[1]->label]->damping) / 2.;
                add_submat(-dt * (dt + damping) * J, indices(n0, n1, n2, n3), A);
                add_subvec(dt * (F + (dt + damping) * J * vs), indices(n0, n1, n2, n3), b);
            }
        }
    }

    template void add_internal_forces<PS>(const std::vector<Face *> &,
                                          const std::vector<Edge *> &,
                                          SpMat<Mat3x3> &, std::vector<Vec3> &,
                                          double);

    template void add_internal_forces<WS>(const std::vector<Face *> &,
                                          const std::vector<Edge *> &,
                                          SpMat<Mat3x3> &, std::vector<Vec3> &,
                                          double);

    template<Space s>
    void add_internal_forces(const Cloth &cloth, SpMat<Mat3x3> &A,
                             vector<Vec3> &b, double dt) {
        add_internal_forces<s>(cloth.mesh.faces, cloth.mesh.edges, A, b, dt);
    }

    template void add_internal_forces<PS>(const Cloth &, SpMat<Mat3x3> &,
                                          vector<Vec3> &, double);

    template void add_internal_forces<WS>(const Cloth &, SpMat<Mat3x3> &,
                                          vector<Vec3> &, double);

    bool contains(const Mesh &mesh, const Node *node) {
        return node->index < mesh.nodes.size() && mesh.nodes[node->index] == node;
    }

    void add_constraint_forces(const Cloth &cloth, const vector<Constraint *> &cons,
                               SpMat<Mat3x3> &A, vector<Vec3> &b, double dt) {
        const Mesh &mesh = cloth.mesh;
        for (int c = 0; c < cons.size(); c++) {
            double value = cons[c]->value();
            double g = cons[c]->energy_grad(value);
            double h = cons[c]->energy_hess(value);
            MeshGrad grad = cons[c]->gradient();
            // f = -g*grad
            // J = -h*outer(grad,grad)
            double v_dot_grad = 0;
            for (MeshGrad::iterator it = grad.begin(); it != grad.end(); it++) {
                const Node *node = it->first;
                v_dot_grad += dot(it->second, node->v);
            }
            for (MeshGrad::iterator it = grad.begin(); it != grad.end(); it++) {
                const Node *nodei = it->first;
                if (!contains(mesh, nodei) || !nodei->active)
                    continue;
                int ni = nodei->index;
                for (MeshGrad::iterator jt = grad.begin(); jt != grad.end(); jt++) {
                    const Node *nodej = jt->first;
                    if (!contains(mesh, nodej) || !nodej->active)
                        continue;
                    int nj = nodej->index;
                    if (dt == 0)
                        A(ni, nj) += h * outer(it->second, jt->second);
                    else
                        A(ni, nj) += dt * dt * h * outer(it->second, jt->second);
                }
                if (dt == 0)
                    b[ni] -= g * it->second;
                else
                    b[ni] -= dt * (g + dt * h * v_dot_grad) * it->second;
            }
        }
    }

    void get_supplementary_part(SpMat<Mat3x3> A, vector<Vec3> &b, vector<Vec3> v) {
        for (int i = 0; i < v.size(); i++) {
            const SpVec<Mat3x3> &row = A.rows[i];
            Vec3 block(0);
            for (size_t jj = 0; jj < row.indices.size(); jj++) {
                int j = row.indices[jj];
                block += A(i, j) * v[j];
            }
            b[i] += block;
        }
    }

    pair<SpMat<Mat3x3>, vector<Vec3> > ObtainImplicitEquation(Cloth &cloth, const vector<Vec3> &fext,
                                                                const vector<Mat3x3> &Jext,
                                                                const vector<Constraint *> &cons, double dt, ForceCollection forces) {
        Mesh &mesh = cloth.mesh;
        vector<Vert *>::iterator vert_it;
        vector<Face *>::iterator face_it;
        int nn = mesh.nodes.size();
      
        SpMat<Mat3x3> A(nn, nn);
        vector<Vec3> b(nn, Vec3(0));

        for (int n = 0; n < mesh.nodes.size(); n++) {
            const Node *node = mesh.nodes[n];
            A(n, n) += Mat3x3(node->m) - dt * dt * Jext[n];
            b[n] += dt * fext[n];
        }
        add_internal_forces<WS>(cloth, A, b, dt);
        add_constraint_forces(cloth, cons, A, b, dt);
    	
        vector<Vec3> v0(nn, Vec3(0));
        for (int n = 0; n < mesh.nodes.size(); n++) {
            const Node *node = mesh.nodes[n];
            v0[n] = node->v;
        }
    	
        get_supplementary_part(A, b, v0);
        return make_pair(A, b);
    }

    Vec3 wind_force(const Face *face, const Wind &wind) {
        Vec3 vface = (face->v[0]->node->v + face->v[1]->node->v
                      + face->v[2]->node->v) / 3.;
        Vec3 vrel = wind.velocity - vface;
        double vn = dot(face->n, vrel);
        Vec3 vt = vrel - vn * face->n;
        return wind.density * face->a * abs(vn) * vn * face->n + wind.drag * face->a * vt;
    }

    void add_external_forces(const Cloth &cloth, const Vec3 &gravity,
                             const Wind &wind, vector<Vec3> &fext,
                             vector<Mat3x3> &Jext) {
        const Mesh &mesh = cloth.mesh;

        for (int n = 0; n < mesh.nodes.size(); n++)
            fext[n] += mesh.nodes[n]->m * gravity;
        for (int f = 0; f < mesh.faces.size(); f++) {
            const Face *face = mesh.faces[f];
            Vec3 fw = wind_force(face, wind);
            for (int v = 0; v < 3; v++)
                fext[face->v[v]->node->index] += fw / 3.;
        }
    }
}
