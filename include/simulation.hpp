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

#ifndef SIMULATION_HPP
#define SIMULATION_HPP

#include "cloth.hpp"
#include "constraint.hpp"
#include "handle.hpp"
#include "spline.hpp"
#include <string>
#include <vector>

using namespace std;

namespace lyra {
    struct Wind {
        double density;
        Vec3 velocity;
        double drag;
    };

    struct Simulation {
        // variables
        double time;
        int frame, step;
        std::vector<Cloth> cloths;
        // constants
        int frame_steps;
		int motion_obstacle_num;
        double frame_time, step_time;
        double end_time, end_frame;
        std::vector<Motion> motions;
        std::vector<Handle *> handles;
        Vec3 gravity;
        Wind wind;
        double friction, obs_friction;
        enum {
            Proximity, Physics, StrainLimiting, Collision, Remeshing, Separation,
            PopFilter, Plasticity, nModules
        };
        bool enabled[nModules];
        // handy pointers
        std::vector<Mesh *> cloth_meshes, obstacle_meshes;
        Mesh *liveMesh;
        ~Simulation();

    };

    void delete_simulation(Simulation &sim);

// Helper functions

    template<typename Prim>
    int size(const std::vector<Mesh *> &meshes);

    template<typename Prim>
    int get_index(const Prim *p,
                  const std::vector<Mesh *> &meshes);

    template<typename Prim>
    Prim *get(int i, const std::vector<Mesh *> &meshes);

    std::vector<Vec3> node_positions(const std::vector<Mesh *> &meshes);

    std::vector<Vec3> node_avg_velocities(const vector<Mesh *> &meshes, double dt);

    void AddExternalForce(Simulation &sim, const vector<Constraint *> &cons, std::vector<Vec3> &fext,
                            std::vector<Mat3x3> &Jext, int c);
}
#endif
