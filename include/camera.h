#pragma once

#include"Common.h"

//#include <glad/glad.h>
#include<GLFW/glfw3.h>
#include<GL/glew.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

#include <vector>

//usages: Camera使用CameraSetting进行配置
//functions: 建立MVP矩阵，留有glfw进行事件响应的接口

//camera 初始化示例:
//Camera<float> camera;
//CameraSetting<float> setting;
//glm::vec3 cameraPosition(0.f, 0.f, -200.f);
//glm::vec3 cameraTarget(0.f, 0.f, 0.f);
//setting.cameraMode = CameraMode::CAMERA_PROJECTION;
//setting.cameraPosition = cameraPosition;
//setting.cameraTarget = cameraTarget;
//setting.scrollSensitivity = 1.5f;
//setting.keyboardSensitivity = 0.1f;
//setting.mouseSensitivity = 0.05f;
//setting.maxPitchRate = 0.004f;
//setting.maxHeadingRate = 0.004f;
//setting.nearClipPlane = 0.1f;
//setting.farClipPlane = 1500.f;
//setting.viewportWidth = screenWidth;
//setting.viewportHeight = screenHeight;
//setting.screenWidth = screenWidth;
//setting.screenHeight = screenHeight;
//setting.FOV = 45.f;
//camera.Init(setting);

//glfw响应函数示例:
//void GlfwKeyBoardCallbackFunc(GLFWwindow* window, int key, int scancode, int action, int mods);
//void GlfwCursorPosCallbackFunc(GLFWwindow* window, double xpos, double ypos);
//void GlfwMouseStateCallbackFunc(GLFWwindow* window, int button, int action, int mods);
//void GlfwScrollCallback(GLFWwindow* window, double xoffset, double yoffset);
//..............................................................................................
//glfwSetKeyCallback(window, GlfwKeyBoardCallbackFunc);
//glfwSetCursorPosCallback(window, GlfwCursorPosCallbackFunc);
//glfwSetMouseButtonCallback(window, GlfwMouseStateCallbackFunc);
//glfwSetScrollCallback(window, GlfwScrollCallback);
//..............................................................................................
//void GlfwKeyBoardCallbackFunc(GLFWwindow* window, int key, int scancode, int action, int mods)
//{
//	//WASDQE 用来控制移动
//	switch (key) {
//	case GLFW_KEY_W:
//		camera.Move(CameraMovement::CAMERA_FRONT);
//		break;
//	case GLFW_KEY_S:
//		camera.Move(CameraMovement::CAMERA_BACK);
//		break;
//	case GLFW_KEY_A:
//		camera.Move(CameraMovement::CAMERA_LEFT);
//		break;
//	case GLFW_KEY_D:
//		camera.Move(CameraMovement::CAMERA_RIGHT);
//		break;
//	case GLFW_KEY_Q:
//		camera.Move(CameraMovement::CAMERA_DOWN);
//		break;
//	case GLFW_KEY_E:
//		camera.Move(CameraMovement::CAMERA_UP);
//		break;
//	case GLFW_KEY_LEFT_CONTROL:
//		{
//			if (action == GLFW_PRESS)
//				camera.SetLeftControlPressState(true);
//			else if (action == GLFW_RELEASE)
//				camera.SetLeftControlPressState(false);
//		}
//		break;
//	case GLFW_KEY_ESCAPE:
//		glfwTerminate();
//		break;
//	default:
//		break;
//	}
//}
//void GlfwCursorPosCallbackFunc(GLFWwindow* window, double xpos, double ypos)
//{
//	camera.SetMousePosition(xpos, ypos);
//}
//void GlfwMouseStateCallbackFunc(GLFWwindow* window, int button, int action, int mods)
//{
//	if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
//		camera.SetMouseMoveState(true);
//	} else if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_RELEASE) {
//		camera.SetMouseMoveState(false);
//	}
//}
//void GlfwScrollCallback(GLFWwindow* window, double xoffset, double yoffset)
//{
//	//滚轮上滚
//	if (yoffset > 0.0) {
//		camera.Move(CameraMovement::CAMERA_UP);
//
//		if (camera.LeftControlState())
//			camera.Move(CameraMovement::CAMERA_FRONT);
//	}
//	//滚轮下滚
//	else if (yoffset < 0.0) {
//		camera.Move(CameraMovement::CAMERA_DOWN);
//		if (camera.LeftControlState())
//			camera.Move(CameraMovement::CAMERA_BACK);
//	}
//}

namespace lyra
{
	//相机的模式
	enum CameraMode
	{
		CAMERA_PROJECTION,
		CAMERA_ORTHOGONALITY
	};

	//相机的移动方向
	enum CameraMovement
	{
		CAMERA_FRONT,
		CAMERA_BACK,
		CAMERA_LEFT,
		CAMERA_RIGHT,
		CAMERA_UP,
		CAMERA_DOWN
	};

	template<typename T>
	struct CameraSetting
	{
		//基本信息
		glm::vec<3, T> cameraPosition;
		glm::vec<3, T> cameraTarget;
		glm::vec<3, T> cameraUp;
		CameraMode cameraMode;

		//移动信息
		T mouseSensitivity;
		T keyboardSensitivity;
		T scrollSensitivity;
		T maxPitchRate;
		T maxHeadingRate;

		//视锥相关
		T FOV;
		T nearClipPlane;
		T farClipPlane;
		T viewportWidth;
		T viewportHeight;
		T screenWidth;
		T screenHeight;

		CameraSetting(){ cameraUp = glm::vec<3, T>(T(0), T(1), T(0));}

	};

	template<typename T>
	class Camera
	{
	private:
		//相机位置
		glm::vec<3, T> position;
		//上向量
		glm::vec<3, T> up;
		//目标位置
		glm::vec<3, T> target;

		//从相机位置指向目标
		glm::vec<3, T> front;
		//右向量
		glm::vec<3, T> right;

		//空间变换矩阵
		glm::mat<4, 4, T> modelMat;
		glm::mat<4, 4, T> viewMat;
		glm::mat<4, 4, T> projectMat;
		glm::mat<4, 4, T> MVPMat;

		//相机倾角,FPS Camera不适用roll
		T pitch;
		T heading;
		T roll;

		//视角大小
		T FOV;
		//宽长比
		T aspect;
		//近剪裁面离相机的距离
		T nearClipPlane;
		//远剪裁面离相机的距离
		T farClipPlane;

		//最大pitch heading rate
		T maxPitchRate;
		T maxHeadingRate;
		//鼠标、键盘和滑轮的灵敏度
		T mouseSensitivity;
		T keyboardSensitivity;
		T scrollSensitivity;

		//视口大小和屏幕大小可以不一致
		T viewportWidth;
		T viewportHeight;
		T screenWidth;
		T screenHeight;

		//相机自身调整姿态用
		bool mouseMove;
		glm::vec<2, T> mousePosition;
		glm::vec<2, T> deltaMouseDisplacement;

		//绕target旋转用
		bool rotate;
		glm::vec<2, T> lastMousePosition;

		//WASDQE 键盘用
		bool leftControlPress;
		glm::vec<3, T> deltaDisplacement;

		//Camera 的模式
		CameraMode mode;

	public:
		Camera() = default;
		~Camera() = default;

		glm::mat<4, 4, T>& GetMVPMatrix() { return MVPMat; }
		glm::mat<4, 4, T>& GetModelMatrix() { return modelMat; }
		glm::mat<4, 4, T>& GetViewMatrix() { return viewMat; }
		glm::mat<4, 4, T>& GetProjectMatrix() { return projectMat; }

		glm::vec<3, T>& Position() { return position; }

		//初始化
		bool Init(CameraSetting<T>& setting, glm::mat<4, 4, T> modelMat = glm::mat<4, 4, T>(T(1)));

		//相机移动，用于响应键盘事件
		void Move(CameraMovement direction);

		//改变Pitch的接口
		void ChangePitch(T degrees);

		//改变heading的接口
		void ChangeHeading(T degrees);

		//改变Roll的接口（待施工）
		void ChangeRoll(T degrees);

		//ArcBall的接口
		void Rotate();

		//水平旋转
		void RotateH(T deltaY, T angle);

		//垂直旋转
		void RotateV(T deltaX, T angle);

		//把平面的点映射到Arcball上去
		glm::vec<3, T> ProjectPointToArcBall(T x, T y);

		//根据外部事件更新camera
		void Update();

		//外部用来改变鼠标位置的接口，用于响应鼠标移动事件
		void SetMousePosition(T x, T y);

		//建立鼠标是否移动的标志
		void SetMouseMoveState(bool state) { mouseMove = state; }

		//建立左Control按下的标志
		void SetLeftControlPressState(bool state) { leftControlPress = state; }

		//建立旋转标记
		void SetRotateState(bool state) { rotate = state; }

		//查询左control的状态
		bool LeftControlState() const { return leftControlPress; }

		//查询旋转标记的状态
		bool RotateState() const { return rotate; }

		//用于记录上次鼠标左键按下时鼠标的位置
		void SetLastMousePosition(T x, T y);

	};

	template<typename T>
	bool Camera<T>::Init(CameraSetting<T>& setting, glm::mat<4, 4, T> modelMat)
	{
		//基本设置
		mode = setting.cameraMode;
		position = setting.cameraPosition;
		target = setting.cameraTarget;

		//灵敏度
		mouseSensitivity = setting.mouseSensitivity;
		keyboardSensitivity = setting.keyboardSensitivity;
		scrollSensitivity = setting.scrollSensitivity;

		//最大偏角
		maxPitchRate = setting.maxPitchRate;
		maxHeadingRate = setting.maxHeadingRate;

		//视锥
		nearClipPlane = setting.nearClipPlane;
		farClipPlane = setting.farClipPlane;
		screenWidth = setting.screenWidth;
		screenHeight = setting.screenHeight;
		viewportWidth = setting.viewportWidth;
		viewportHeight = setting.viewportHeight;
		aspect = viewportWidth / viewportHeight;
		FOV = setting.FOV;

		//其他需要初始化的量
		deltaDisplacement = glm::vec<3, T>(T(0), T(0), T(0));
		//up = glm::vec<3, T>(T(0), T(0), T(1));
		up = setting.cameraUp;
		mouseMove = false;
		leftControlPress = false;
		rotate = false;

		//外界可以对model先做变换，camera用来生成MVP矩阵
		this->modelMat = modelMat;
		projectMat = viewMat = glm::mat<4, 4, T>(T(1));

		roll = heading = pitch = T(0);
		return true;
	}

	template<typename T>
	void Camera<T>::Update()
	{
		//定义相机的前向
		front = glm::normalize(target - position);
		//不用这个会无法正常使用光照
		//glViewport(0, 0, screenWidth, screenHeight);
		//根据键鼠的输入来更新位置，角度等信息
		if (mode == CameraMode::CAMERA_ORTHOGONALITY) { std::cout << "施工中...\n"; }

		projectMat = glm::perspective(FOV, aspect, nearClipPlane, farClipPlane);

		if (mode == CameraMode::CAMERA_PROJECTION)
		{
			projectMat = glm::perspective(FOV, aspect, nearClipPlane, farClipPlane);
			//计算右向量,front和up都是单位向量
			right = glm::cross(front, up);
			//pitch是绕right来旋转的
			glm::quat pitchQuat = glm::angleAxis(pitch, right);
			//计算新的up
			up = glm::cross(right, front);
			//heading是绕up来旋转的
			glm::quat headingQuat = glm::angleAxis(heading, up);
			//综合两种变换
			glm::quat totalQuat = glm::cross(pitchQuat, headingQuat);
			//需要单位四元数
			totalQuat = glm::normalize(totalQuat);
			//四元数->旋转矩阵
			glm::mat<4, 4, T> rotation = glm::mat4_cast(totalQuat);
			//转变camera的方向，既对front向量进行变换
			front = rotation * glm::vec<4, T>(front, 1.0f);
			//更新camera位置
			position = position + deltaDisplacement;
			//新的target向量
			target = position + front * T(1);
			//ArcBall 的相机旋转
			if (rotate) {
				/*std::cout << "True" << "\n";*/
				viewMat = glm::lookAt(position, target, up);
				Rotate();
			}
			//smooth camera
			heading *= T(.5);
			pitch *= T(.5);

			deltaDisplacement = deltaDisplacement * T(.8);
		}

		viewMat = glm::lookAt(position, target, up);
		MVPMat = projectMat * viewMat * modelMat;
	}

	template<typename T>
	void Camera<T>::Move(CameraMovement direction)
	{
		switch (direction)
		{
		case CameraMovement::CAMERA_UP:
			deltaDisplacement += up * keyboardSensitivity;
			break;
		case CameraMovement::CAMERA_DOWN:
			deltaDisplacement -= up * keyboardSensitivity;
			break;
		case CameraMovement::CAMERA_LEFT:
			//注意这里算出的是左向量而不是右向量
			deltaDisplacement += glm::cross(up, front) * keyboardSensitivity;
			break;
		case CameraMovement::CAMERA_RIGHT:
			deltaDisplacement -= glm::cross(up, front) * keyboardSensitivity;
			break;
		case CameraMovement::CAMERA_FRONT:
		{
			deltaDisplacement += front * keyboardSensitivity;
			if (leftControlPress)
				deltaDisplacement += front * scrollSensitivity;
			break;
		}
		case CameraMovement::CAMERA_BACK:
		{
			deltaDisplacement -= front * keyboardSensitivity;
			if (leftControlPress)
				deltaDisplacement -= front * scrollSensitivity;
			break;
		}
		default:
			std::cerr << "direction is illegal\n";
			break;
		}
	}

	template<typename T>
	void Camera<T>::ChangePitch(T degrees)
	{
		//限制最大旋转
		if (degrees < -maxPitchRate)
			degrees = -maxPitchRate;
		else if (degrees > maxPitchRate)
			degrees = maxPitchRate;

		pitch += degrees;

		//限定在一圈内
		if (pitch > static_cast<T>(360.))
			pitch -= static_cast<T>(360.);
		else if (pitch < static_cast<T>(-360.))
			pitch += static_cast<T>(360.);
	}

	template<typename T>
	void Camera<T>::ChangeHeading(T degrees)
	{
		//printf_s("test heading\n");
		if (degrees < -maxHeadingRate)
			degrees = -maxHeadingRate;
		else if (degrees > maxHeadingRate)
			degrees = maxHeadingRate;

		//heading需要做一些额外的限制，pitch过大的时候要适当缩小heading
		if (pitch > static_cast<T>(90.) && pitch < static_cast<T>(270.)
			|| (pitch < static_cast<T>(-90.) && pitch > static_cast<T>(-270.)))
			heading -= degrees;
		else
			heading += degrees;

		if (heading > static_cast<T>(360.))
			heading -= static_cast<T>(360.);
		else if (heading < static_cast<T>(-360.))
			heading += static_cast<T>(360.);
	}

	template<typename T>
	void Camera<T>::ChangeRoll(T degrees)
	{

	}

	template<typename T>
	void Camera<T>::SetMousePosition(T x, T y)
	{
		glm::vec<2, T> deltaMouseDisplacement = mousePosition - glm::vec<2, T>(x, y);

		//printf_s("%f %f\n", deltaMouseDisplacement.x * mouseSensitivity, deltaMouseDisplacement.y * mouseSensitivity);

		if (mouseMove) {
			ChangeHeading(deltaMouseDisplacement.x * mouseSensitivity);
			ChangePitch(deltaMouseDisplacement.y * mouseSensitivity);
		}

		//printf_s("%f %f\n", pitch, heading);
		mousePosition = glm::vec<2, T>(x, y);
	}

	template<typename T>
	void Camera<T>::SetLastMousePosition(T x, T y)
	{
		lastMousePosition.x = x;
		lastMousePosition.y = y;
	}

	template<typename T>
	void Camera<T>::Rotate()
	{
		//旋转的角度
		T angle = T(0);
		if (mousePosition != lastMousePosition) {
			glm::vec<3, T> va = ProjectPointToArcBall(lastMousePosition.x, lastMousePosition.y);
			glm::vec<3, T> vb = ProjectPointToArcBall(mousePosition.x, mousePosition.y);

			//光滑的轨迹球
			va = glm::normalize(va);
			vb = glm::normalize(vb);
			//angle = std::acos(std::min(T(1), glm::dot(va, vb)));

			angle = std::acos(std::min(T(1), glm::dot(va, vb)));

			/*glm::vec<3, T> axisInCameraSpace = glm::cross(va, vb);

			glm::vec<3, T> axisInWorldSpace = glm::inverse(viewMat) * glm::vec<4, T>(axisInCameraSpace, T(1));

			axisInWorldSpace = glm::normalize(axisInWorldSpace);*/

			glm::vec<2, T> aux = mousePosition - lastMousePosition;

			//水平旋转
			RotateH(aux.y, angle);
			//垂直旋转
			RotateV(aux.x, angle);

			/*rotateMat = glm::rotate(rotateMat, angle, right);

			position = rotateMat * glm::vec<4, T>(position, T(1));*/

			lastMousePosition = mousePosition;
		}
	}

	template<typename T>
	void Camera<T>::RotateH(T delta, T angle)
	{
		glm::mat<4, 4, T> rotateMat(T(1));
		if (delta < 0)
			angle = -angle;

		rotateMat = glm::rotate(rotateMat, angle, right);
		modelMat = rotateMat * modelMat;
	}

	template<typename T>
	void Camera<T>::RotateV(T delta, T angle)
	{
		glm::mat<4, 4, T> rotateMat(T(1));
		if (delta < 0)
			angle = -angle;

		rotateMat = glm::rotate(rotateMat, angle, up);
		modelMat = rotateMat * modelMat;
	}

	template<typename T>
	glm::vec<3, T> Camera<T>::ProjectPointToArcBall(T x, T y)
	{
		//变换到[-1,1]区间
		T xx = x / (screenWidth / T(2)) - T(1);
		T yy = T(1) - y / (screenHeight / T(2));

		glm::vec<3, T> pointOnArcBall;

		T tmp = xx * xx + yy * yy;

		if (tmp <= T(1))
			pointOnArcBall = glm::vec<3, T>(xx, yy, std::sqrt(T(1) - tmp));
		else
			pointOnArcBall = glm::vec<3, T>(xx, yy, T(1) / (std::sqrt(1 + yy * yy)));

		return pointOnArcBall;
		////非光滑的轨迹球
		//glm::vec<3,T> pointOnArcBall = glm::vec<3,T>(x/(screenWidth / T(2)) -T(1),
		//	T(1) - y / (screenHeight / T(2)), T(0));

		////printf_s("%f %f %f\n", pointOnArcBall.x, pointOnArcBall.y, pointOnArcBall.z);

		//T tmp = pointOnArcBall.x * pointOnArcBall.x + pointOnArcBall.y * pointOnArcBall.y;

		//if (tmp <= T(1))
		//	pointOnArcBall.z = std::sqrt(T(1) - tmp);
		//else
		//	pointOnArcBall = glm::normalize(pointOnArcBall);

		//return pointOnArcBall;
	}
}




