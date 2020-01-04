#pragma once

#include "constraint.hpp"
#include "mesh.hpp"
#include <vector>

namespace lyra {
    struct Handle {
        double start_time, end_time, fade_time;

        virtual ~Handle() {};

        virtual std::vector<Constraint *> get_constraints(double t) = 0;

        virtual std::vector<Node *> get_nodes() = 0;

        bool active(double t) { return t >= start_time && t <= end_time; }

        double strength(double t) {
            if (t < start_time || t > end_time + fade_time) return 0;
            if (t <= end_time) return 1;
            double s = 1 - (t - end_time) / (fade_time + 1e-6);
            return sq(sq(s));
        }
    };

    struct NodeHandle : public Handle {
        Node *node;
        const Motion *motion;
        bool activated;
        Vec3 x0;

        NodeHandle() : activated(false) {}

        std::vector<Constraint *> get_constraints(double t);

        std::vector<Node *> get_nodes() { return std::vector<Node *>(1, node); }
    };

    struct CircleHandle : public Handle {
        Mesh *mesh;
        int label;
        const Motion *motion;
        double c; // circumference
        Vec2 u;
        Vec3 xc, dx0, dx1;

        std::vector<Constraint *> get_constraints(double t);

        std::vector<Node *> get_nodes() { return std::vector<Node *>(); }
    };

    struct GlueHandle : public Handle {
        Node *nodes[2];

        std::vector<Constraint *> get_constraints(double t);

        std::vector<Node *> get_nodes() {
            std::vector<Node *> ns;
            ns.push_back(nodes[0]);
            ns.push_back(nodes[1]);
            return ns;
        }
    };
}
