#pragma once
#include "Simulator.hpp"

//#include<glad/glad.h>
#include <GL/glew.h>
#include<GLFW/glfw3.h>
//#include<glad/glad.c>
#include<string>
#include<memory>
#include<iostream>
#include<Shader.h>
#include<stb_image.h>
//#include <glm/gtc/type_ptr.hpp>

#include<camera.h>
#include<Shader.h>

static std::unique_ptr<lyra::Camera<float>> camera;

const static float frameColor[3] = { 0., 0., 0. };
const static float clothColor[3] = { 1., 0.49, 0.51 };
const static float backgroundColor[3] = { 1., 1., 1. };

namespace lyra
{
	template<typename T>
	class Render
	{
	public:
		Render() = default;
	public:
		//void ParseConfigureFile(std::string& json_file);
		//void InitSimulator(std::string& json_file);
		void InitRender(CameraSetting<T>& camera_setting, int w, int h);
		void Run(Simulator<T>& simulator);

	private:
		enum DataName {
			VERTEX, COORDINATE, COLOR, NORMAL, ELEMENT, numData
		};
		GLuint  vaoHandle;
		GLuint vboHandles[numData];
		GLuint texture;

	public:
		std::unique_ptr<T[]> vertices;
		std::unique_ptr<T[]> texCoords;
		std::unique_ptr<T[]> colors;
		std::unique_ptr<T[]> normals;
		std::unique_ptr<int[]> elements;
		int vSize, fSize;
		std::string texFile;

		GLFWwindow* window_;
		Shader<T> shader_;

		//初始化openGL
		static void GlfwReshapeCallbackFunc(GLFWwindow* window, int width, int height);
		static void GlfwKeyBoardCallbackFunc(GLFWwindow* window, int key, int scancode, int action, int mods);
		static void GlfwCursorPosCallbackFunc(GLFWwindow* window, double xpos, double ypos);
		static void GlfwMouseStateCallbackFunc(GLFWwindow* window, int button, int action, int mods);
		static void GlfwScrollCallback(GLFWwindow* window, double xoffset, double yoffset);

	private:
		void Prepare();
		void Bind();
		void Display();
		void Draw(bool show_mesh = true);
		void ExtractRenderData(Simulator<T>& simulator);
		void LoadTexture(Simulator<T>& simulator);

	};

	template<typename T>
	void Render<T>::Prepare() {
		//生成buffer
		glGenBuffers(numData, vboHandles);
		glGenVertexArrays(1, &vaoHandle);
		glBindVertexArray(vaoHandle);

		glEnableVertexAttribArray(0);
		glBindBuffer(GL_ARRAY_BUFFER, vboHandles[VERTEX]);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (GLubyte*)NULL);

		glEnableVertexAttribArray(1);
		glBindBuffer(GL_ARRAY_BUFFER, vboHandles[COORDINATE]);
		glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 0, (GLubyte*)NULL);

		glEnableVertexAttribArray(2);
		glBindBuffer(GL_ARRAY_BUFFER, vboHandles[COLOR]);
		glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 0, (GLubyte*)NULL);

		glEnableVertexAttribArray(3);
		glBindBuffer(GL_ARRAY_BUFFER, vboHandles[NORMAL]);
		glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, 0, (GLubyte*)NULL);
	}

	template <typename T>
	void Render<T>::Bind() {
		glBindVertexArray(vaoHandle);

		glBindBuffer(GL_ARRAY_BUFFER, vboHandles[VERTEX]);
		glBufferData(GL_ARRAY_BUFFER, vSize * 3 * sizeof(float), vertices.get(), GL_STATIC_DRAW);

		glBindBuffer(GL_ARRAY_BUFFER, vboHandles[COORDINATE]);
		glBufferData(GL_ARRAY_BUFFER, vSize * 2 * sizeof(float), texCoords.get(), GL_STATIC_DRAW);

		glBindBuffer(GL_ARRAY_BUFFER, vboHandles[COLOR]);
		glBufferData(GL_ARRAY_BUFFER, vSize * 3 * sizeof(float), colors.get(), GL_STATIC_DRAW);
		//
		//    glBindBuffer(GL_ARRAY_BUFFER, vboHandles[ALPHA]);
		//    glBufferData(GL_ARRAY_BUFFER, vSize*sizeof(float), &alpha[0], GL_STATIC_DRAW);

		glBindBuffer(GL_ARRAY_BUFFER, vboHandles[NORMAL]);
		glBufferData(GL_ARRAY_BUFFER, vSize * 3 * sizeof(float), normals.get(), GL_STATIC_DRAW);

		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vboHandles[ELEMENT]);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, (3 * fSize) * sizeof(int), elements.get(), GL_STATIC_DRAW);
	}

	template <typename T>
	void Render<T>::Draw(bool show_mesh) {
		if (show_mesh) {
			// Prevent z-fighting with mesh edges.
			glPushAttrib(GL_POLYGON_BIT);
			glEnable(GL_POLYGON_OFFSET_FILL);
			glPolygonOffset(1., 1.);
		}

		glBindVertexArray(vaoHandle);

		glBindTexture(GL_TEXTURE_2D, texture);
		//    glDrawArrays(GL_POINTS, vSize, pSize);
		//    glDrawArrays(GL_POINTS, 0, vSize);
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		glDrawElements(GL_TRIANGLES, fSize * 3, GL_UNSIGNED_INT, (void*)0);

		if (show_mesh) {
			float* frameColors = new float[vSize * 3];
			for (int v = 0; v < vSize; v++) {
				for (int i = 0; i < 3; i++) {
					frameColors[v * 3 + i] = frameColor[i];
				}
			}

			glBindBuffer(GL_ARRAY_BUFFER, vboHandles[COLOR]);
			glBufferData(GL_ARRAY_BUFFER, vSize * 3 * sizeof(T), frameColors, GL_STATIC_DRAW);

			glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
			glDrawElements(GL_TRIANGLES, fSize * 3, GL_UNSIGNED_INT, (void*)0);
		}

		//    glDrawElements(GL_LINES, lSize*2, GL_UNSIGNED_INT, (void*)(fSize*3*sizeof(int)));

		if (show_mesh) {
			// Revert z-fighting prevention enabled at beginning of this function.
			glPopAttrib();
		}
	}

	template <typename T>
	void Render<T>::Display() {
		glClearColor(backgroundColor[0], backgroundColor[1], backgroundColor[2], 0);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		//glEnable(GL_BLEND);
	   // glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glLightModeli(GL_LIGHT_MODEL_AMBIENT, 1);
		glEnable(GL_LIGHTING);
		glEnable(GL_DEPTH_TEST);
		glEnable(GL_COLOR_MATERIAL);
		glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
		glEnable(GL_NORMALIZE);
		glEnable(GL_POINT_SMOOTH);
		glEnable(GL_POLYGON_SMOOTH);

		shader_.setMat4("model", glm::mat4(1.0));
		shader_.setMat4("view", camera->GetViewMatrix());
		shader_.setMat4("projection", camera->GetProjectMatrix());
		shader_.setVec3("eyePos", camera->Position()[0], camera->Position()[1], camera->Position()[2]);
		shader_.setInt("Tex1", 0);
		if(texFile!="")
			shader_.setInt("useTex", 1);
		else
			shader_.setInt("useTex", 0);
		
		Bind();
		Draw();

		glfwSwapBuffers(window_);
	}

	template<typename T>
	void Render<T>::InitRender(CameraSetting<T>& camera_setting, int w, int h) {

		camera = std::make_unique<Camera<float>>();
		camera->Init(camera_setting);
		if (!glfwInit())
			exit(1);
		glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
		glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
		glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
		glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);

		//antialiasing
		glfwWindowHint(GLFW_SAMPLES, 4);

		window_ = glfwCreateWindow(w, h, "PDE Final Project", nullptr, nullptr);
		glfwMakeContextCurrent(window_);
		glfwSetFramebufferSizeCallback(window_, Render::GlfwReshapeCallbackFunc);
		glfwSetKeyCallback(window_, Render::GlfwKeyBoardCallbackFunc);
		glfwSetMouseButtonCallback(window_, Render::GlfwMouseStateCallbackFunc);
		glfwSetCursorPosCallback(window_, Render::GlfwCursorPosCallbackFunc);
		glfwSetScrollCallback(window_, Render::GlfwScrollCallback);

		glewExperimental = GL_TRUE;
		glewInit();

		/*if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
			std::cerr << "Failed to initialize GLAD" << std::endl;
			system("pause");
		}*/

		std::string shaderPrefix = "./shader/shader.";
		shader_.Init((shaderPrefix + "vert").c_str(), (shaderPrefix + "frag").c_str());
		shader_.use();

		Prepare();

		////准备shader,bind,Draw
		//glClearColor(0, 0, 0, 0);
		////glClearColor(1, 1, 1, 0);
		//glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		////glEnable(GL_BLEND);
	 //  // glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		//glLightModeli(GL_LIGHT_MODEL_AMBIENT, 1);
		//glEnable(GL_LIGHTING);
		//glEnable(GL_DEPTH_TEST);
		//glEnable(GL_COLOR_MATERIAL);
		//glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
		//glEnable(GL_NORMALIZE);
		//glEnable(GL_POINT_SMOOTH);
		//glEnable(GL_POLYGON_SMOOTH);

		//shader_.setMat4("model", glm::mat4(1.0));
		//shader_.setMat4("view", camera->GetViewMatrix());
		//shader_.setMat4("projection", camera->GetProjectMatrix());
		//shader_.setVec3("eyePos", camera->Position()[0], camera->Position()[1], camera->Position()[2]);
		//shader_.setInt("Tex1", 0);
		//shader_.setInt("useTex", 0);

		//Bind();
		//Draw();

		//glfwSwapBuffers(window_);
	}

	template <typename T>
	void Render<T>::Run(Simulator<T>& simulator) {
		glEnable(GL_MULTISAMPLE);
		LoadTexture(simulator);
		while (!glfwWindowShouldClose(window_)) {
			camera->Update();

			shader_.use();
			shader_.setMat4("model", glm::mat4(1.0));
			shader_.setMat4("view", camera->GetViewMatrix());
			shader_.setMat4("projection", camera->GetProjectMatrix());
			shader_.setVec3("eyePos", camera->Position()[0], camera->Position()[1], camera->Position()[2]);

			//simulation section
			simulator.Simulate();
			
			//rendering section
			ExtractRenderData(simulator);
			Display();
			glfwPollEvents();
		}
		glfwDestroyWindow(window_);
		glfwTerminate();
	}

	template<typename T>
	void Render<T>::GlfwReshapeCallbackFunc(GLFWwindow* window, int width, int height)
	{
		glViewport(0, 0, width, height);
	}

	template<typename T>
	void Render<T>::GlfwKeyBoardCallbackFunc(GLFWwindow* window, int key, int scancode, int action, int mods)
	{
		//WASDQE 用来控制移动
		switch (key) {
		case GLFW_KEY_W:
			camera->Move(CameraMovement::CAMERA_FRONT);
			break;
		case GLFW_KEY_S:
			camera->Move(CameraMovement::CAMERA_BACK);
			break;
		case GLFW_KEY_A:
			camera->Move(CameraMovement::CAMERA_LEFT);
			break;
		case GLFW_KEY_D:
			camera->Move(CameraMovement::CAMERA_RIGHT);
			break;
		case GLFW_KEY_Q:
			camera->Move(CameraMovement::CAMERA_DOWN);
			break;
		case GLFW_KEY_E:
			camera->Move(CameraMovement::CAMERA_UP);
			break;
		case GLFW_KEY_LEFT_CONTROL:
		{
			if (action == GLFW_PRESS) {
				camera->SetLeftControlPressState(true);
				//camera->SetRotateState(true);
				//std::cout << "GLFW_PRESS\n";
			}
			else if (action == GLFW_RELEASE) {
				camera->SetLeftControlPressState(false);
				//禁止绕target旋转
				camera->SetRotateState(false);
				//std::cout << "GLFW_RELEASE\n";
			}
		}
		break;
		case GLFW_KEY_ESCAPE:
			glfwTerminate();
			break;
		default:
			break;
		}

		/*if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS)
			clothDrop = true;

		if (glfwGetKey(window, GLFW_KEY_V) == GLFW_PRESS)
			drawVelocity = !drawVelocity;

		if (glfwGetKey(window, GLFW_KEY_B) == GLFW_PRESS && glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS)
			bodyLine = !bodyLine;
		if (glfwGetKey(window, GLFW_KEY_C) == GLFW_PRESS && glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS)
			clothLine = !clothLine;*/

		/*if (glfwGetKey(window, GLFW_KEY_P) == GLFW_PRESS)
			system("pause");*/


		/*if (glfwGetKey(window, GLFW_KEY_N) == GLFW_PRESS)
			drawNormal = !drawNormal;
		if (glfwGetKey(window, GLFW_KEY_L) == GLFW_PRESS)
			lineMode = !lineMode;
		if (glfwGetKey(window, GLFW_KEY_K) == GLFW_PRESS) {
			hasWind = !hasWind;
			if (hasWind)
				std::cout << "wind turn on\n";
			else
				std::cout << "wind turn off\n";
		}*/

	}

	template<typename T>
	void Render<T>::GlfwCursorPosCallbackFunc(GLFWwindow* window, double xpos, double ypos)
	{
		camera->SetMousePosition(xpos, ypos);
	}

	template<typename T>
	void Render<T>::GlfwMouseStateCallbackFunc(GLFWwindow* window, int button, int action, int mods)
	{
		if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
			camera->SetMouseMoveState(true);

			//左鼠标按下左ctrl按下，建立旋转标记并且初始化起始点的位置,同时禁止自身姿态调整
			if (camera->LeftControlState()) {
				camera->SetRotateState(true);

				camera->SetMouseMoveState(false);

				double xPos, yPos;
				glfwGetCursorPos(window, &xPos, &yPos);
				//记录鼠标点击时的位置
				camera->SetLastMousePosition(xPos, yPos);
			}
		}
		else if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_RELEASE) {
			camera->SetMouseMoveState(false);
			//左鼠标松开，禁止绕target旋转
			camera->SetRotateState(false);
		}
	}

	template<typename T>
	void Render<T>::GlfwScrollCallback(GLFWwindow* window, double xoffset, double yoffset)
	{
		//滚轮上滚
		if (yoffset > 0.0) {
			camera->Move(CameraMovement::CAMERA_UP);

			if (camera->LeftControlState())
				camera->Move(CameraMovement::CAMERA_FRONT);
		}
		//滚轮下滚
		else if (yoffset < 0.0) {
			camera->Move(CameraMovement::CAMERA_DOWN);
			if (camera->LeftControlState())
				camera->Move(CameraMovement::CAMERA_BACK);
		}
	}

	template<typename T>
	void Render<T>::ExtractRenderData(Simulator<T>& simulator) {
		texFile = simulator.simulation_.cloths[0].texture;

		vSize = 0;
		fSize = 0;
		for (int c = 0; c < simulator.simulation_.cloths.size(); c++) {
			vSize += simulator.simulation_.cloths[c].mesh.verts.size();
			fSize += simulator.simulation_.cloths[c].mesh.faces.size();
		}
	
		vertices = std::make_unique<float[]>(vSize * 3);
		texCoords = std::make_unique<float[]>(vSize * 2);
		colors = std::make_unique<float[]>(vSize * 3);
		normals = std::make_unique<float[]>(vSize * 3);
		elements = std::make_unique<int[]>(fSize * 3);

		int vtxOffset = 0, faceOffset = 0;
		for (int c = 0; c < simulator.simulation_.cloths.size(); c++) {
			Mesh& mesh = simulator.simulation_.cloths[c].mesh;
			compute_ws_data(mesh);
			for (int v = 0; v < mesh.verts.size(); v++) {
				for (int i = 0; i < 3; i++) {
					vertices[vtxOffset * 3 + v * 3 + i] = mesh.verts[v]->node->x[i];
					colors[vtxOffset * 3 + v * 3 + i] = clothColor[i];
					normals[vtxOffset * 3 + v * 3 + i] = mesh.verts[v]->node->n[i];
				}
				for (int i = 0; i < 2; i++) {
					texCoords[vtxOffset * 2 + v * 2 + i] = mesh.verts[v]->u[i];
				}
			}
			for (int f = 0; f < mesh.faces.size(); f++) {
				for (int i = 0; i < 3; i++) {
					elements[faceOffset * 3 + f * 3 + i] = mesh.faces[f]->v[i]->index + vtxOffset;
				}
			}
			vtxOffset += mesh.verts.size();
			faceOffset += mesh.faces.size();
		}
	}

	template<typename T>
	void Render<T>::LoadTexture(Simulator<T>& simulator) {

		texFile = simulator.simulation_.cloths[0].texture;
		if (texFile == "")
			return;
		
		glActiveTexture(GL_TEXTURE0);
		glGenTextures(1, &texture);

		glBindTexture(GL_TEXTURE_2D, texture);
		int width, height, nrChannels;
		unsigned char* data;
		if (texFile.empty()) {
		}
		else {
			//std::cout << texFile.c_str() << "\n";
			data = stbi_load(texFile.c_str(), &width, &height, &nrChannels, 0);
			glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0,
				GL_RGB, GL_UNSIGNED_BYTE, data);
			glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
			glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
			stbi_image_free(data);
		}

	}
}
