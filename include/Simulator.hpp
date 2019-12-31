#pragma once
#include <string>
#include "cloth.hpp"
#include "conf.hpp"
#include "physics.hpp"
#include "sparse_solver.hpp"

namespace lyra
{
	template<typename T>
	class Simulator
	{
	public:
		Simulator() = default;
		~Simulator() = default;

	public:
		void ParseConfigure(std::string& json_file);
		void Prepare();
		void Simulate();
		void PhysicStep(const vector<Constraint*>& cons);
		void InitSimulator(std::string& json_file);

	public:
		Simulation simulation_;

	private:
		void Step();
		vector<Constraint*> GetConstraints(bool include_proximity);
		void DeleteConstraints(const vector<Constraint* > & cons);
	};

	template <typename T>
	void Simulator<T>::ParseConfigure(std::string& json_file) {
		load_json(json_file, simulation_);
	}

	template <typename T>
	void Simulator<T>::Prepare() {

		simulation_.cloth_meshes.resize(simulation_.cloths.size());
		for (int c = 0; c < simulation_.cloths.size(); c++) {
			compute_masses(simulation_.cloths[c]);
			simulation_.cloth_meshes[c] = &simulation_.cloths[c].mesh;
			update_x0(*(simulation_.cloth_meshes)[c]);
			update_n0(*(simulation_.cloth_meshes)[c]);
		}

		simulation_.liveMesh = simulation_.cloth_meshes[0];
	}

	template <typename T>
	void Simulator<T>::Step() {
		activate_nodes(simulation_.cloth_meshes[0]->nodes);

		simulation_.cloth_meshes[0] = simulation_.liveMesh;
		simulation_.time += simulation_.step_time;
		simulation_.step++;

		vector<Constraint*> cons = GetConstraints(true);

		PhysicStep(cons);

	/*	if (simulation_.step % simulation_.frame_steps == 0) {
			simulation_.frame++;
		}*/

		DeleteConstraints(cons);
	}

	template<typename T>
	void Simulator<T>::InitSimulator(std::string& json_file) {
		ParseConfigure(json_file);
		Prepare();
	}

	template<typename T>
	vector<Constraint*> Simulator<T>::GetConstraints(bool include_proximity) {
		vector<Constraint*> cons;
		//add artificial handle
		for (int h = 0; h < simulation_.handles.size(); h++)
			append(cons, simulation_.handles[h]->get_constraints(simulation_.time));
		//stationary node
		for (int m = 0; m < simulation_.cloth_meshes.size(); m++) {
			mark_nodes_to_preserve(*simulation_.cloth_meshes[m]);
		}
		return cons;
	}

	template <typename T>
	void Simulator<T>::Simulate() {
		Step();
	}
	
	template<typename T>
	void Simulator<T>::DeleteConstraints(const vector<Constraint* >& cons) {
		for (int c = 0; c < cons.size(); c++)
			delete cons[c];
	}
	template<typename T>
	void Simulator<T>::PhysicStep(const vector<Constraint*>& cons) {
		SpMat<Mat3x3> A;
		vector<Vec3> b;
		vector<Vec3> predicted_vels;

		//for every cloth, collecting a big linear system to solve
		for (int c = 0; c < simulation_.cloths.size(); c++) {
			vector<Vec3> fext;
			vector<Mat3x3> Jext;
			//add external force[gravity and wind]
			AddExternalForce(simulation_, cons, fext, Jext, c);
			pair<SpMat<Mat3x3>, vector<Vec3> > equation;
			equation = ObtainImplicitEquation(simulation_.cloths[c], fext, Jext, cons, simulation_.step_time);
			A = equation.first;
			b = equation.second;
			predicted_vels = EigenLinearSolver(A, b);
			for (int n = 0; n < simulation_.cloths[c].mesh.nodes.size(); n++) {
				Node* node = simulation_.cloths[c].mesh.nodes[n];
				node->v = predicted_vels[n];
				node->x += (node->v) * simulation_.step_time;
			}
		}
	}

}


