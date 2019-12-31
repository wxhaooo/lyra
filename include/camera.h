#pragma once

#include"Common.h"

//#include <glad/glad.h>
#include<GLFW/glfw3.h>
#include<GL/glew.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

#include <vector>

//usages: Cameraʹ��CameraSetting��������
//functions: ����MVP��������glfw�����¼���Ӧ�Ľӿ�

//camera ��ʼ��ʾ��:
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

//glfw��Ӧ����ʾ��:
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
//	//WASDQE ���������ƶ�
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
//	//�����Ϲ�
//	if (yoffset > 0.0) {
//		camera.Move(CameraMovement::CAMERA_UP);
//
//		if (camera.LeftControlState())
//			camera.Move(CameraMovement::CAMERA_FRONT);
//	}
//	//�����¹�
//	else if (yoffset < 0.0) {
//		camera.Move(CameraMovement::CAMERA_DOWN);
//		if (camera.LeftControlState())
//			camera.Move(CameraMovement::CAMERA_BACK);
//	}
//}

namespace lyra
{
	//�����ģʽ
	enum CameraMode
	{
		CAMERA_PROJECTION,
		CAMERA_ORTHOGONALITY
	};

	//������ƶ�����
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
		//������Ϣ
		glm::vec<3, T> cameraPosition;
		glm::vec<3, T> cameraTarget;
		glm::vec<3, T> cameraUp;
		CameraMode cameraMode;

		//�ƶ���Ϣ
		T mouseSensitivity;
		T keyboardSensitivity;
		T scrollSensitivity;
		T maxPitchRate;
		T maxHeadingRate;

		//��׶���
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
		//���λ��
		glm::vec<3, T> position;
		//������
		glm::vec<3, T> up;
		//Ŀ��λ��
		glm::vec<3, T> target;

		//�����λ��ָ��Ŀ��
		glm::vec<3, T> front;
		//������
		glm::vec<3, T> right;

		//�ռ�任����
		glm::mat<4, 4, T> modelMat;
		glm::mat<4, 4, T> viewMat;
		glm::mat<4, 4, T> projectMat;
		glm::mat<4, 4, T> MVPMat;

		//������,FPS Camera������roll
		T pitch;
		T heading;
		T roll;

		//�ӽǴ�С
		T FOV;
		//����
		T aspect;
		//��������������ľ���
		T nearClipPlane;
		//Զ������������ľ���
		T farClipPlane;

		//���pitch heading rate
		T maxPitchRate;
		T maxHeadingRate;
		//��ꡢ���̺ͻ��ֵ�������
		T mouseSensitivity;
		T keyboardSensitivity;
		T scrollSensitivity;

		//�ӿڴ�С����Ļ��С���Բ�һ��
		T viewportWidth;
		T viewportHeight;
		T screenWidth;
		T screenHeight;

		//������������̬��
		bool mouseMove;
		glm::vec<2, T> mousePosition;
		glm::vec<2, T> deltaMouseDisplacement;

		//��target��ת��
		bool rotate;
		glm::vec<2, T> lastMousePosition;

		//WASDQE ������
		bool leftControlPress;
		glm::vec<3, T> deltaDisplacement;

		//Camera ��ģʽ
		CameraMode mode;

	public:
		Camera() = default;
		~Camera() = default;

		glm::mat<4, 4, T>& GetMVPMatrix() { return MVPMat; }
		glm::mat<4, 4, T>& GetModelMatrix() { return modelMat; }
		glm::mat<4, 4, T>& GetViewMatrix() { return viewMat; }
		glm::mat<4, 4, T>& GetProjectMatrix() { return projectMat; }

		glm::vec<3, T>& Position() { return position; }

		//��ʼ��
		bool Init(CameraSetting<T>& setting, glm::mat<4, 4, T> modelMat = glm::mat<4, 4, T>(T(1)));

		//����ƶ���������Ӧ�����¼�
		void Move(CameraMovement direction);

		//�ı�Pitch�Ľӿ�
		void ChangePitch(T degrees);

		//�ı�heading�Ľӿ�
		void ChangeHeading(T degrees);

		//�ı�Roll�Ľӿڣ���ʩ����
		void ChangeRoll(T degrees);

		//ArcBall�Ľӿ�
		void Rotate();

		//ˮƽ��ת
		void RotateH(T deltaY, T angle);

		//��ֱ��ת
		void RotateV(T deltaX, T angle);

		//��ƽ��ĵ�ӳ�䵽Arcball��ȥ
		glm::vec<3, T> ProjectPointToArcBall(T x, T y);

		//�����ⲿ�¼�����camera
		void Update();

		//�ⲿ�����ı����λ�õĽӿڣ�������Ӧ����ƶ��¼�
		void SetMousePosition(T x, T y);

		//��������Ƿ��ƶ��ı�־
		void SetMouseMoveState(bool state) { mouseMove = state; }

		//������Control���µı�־
		void SetLeftControlPressState(bool state) { leftControlPress = state; }

		//������ת���
		void SetRotateState(bool state) { rotate = state; }

		//��ѯ��control��״̬
		bool LeftControlState() const { return leftControlPress; }

		//��ѯ��ת��ǵ�״̬
		bool RotateState() const { return rotate; }

		//���ڼ�¼�ϴ�����������ʱ����λ��
		void SetLastMousePosition(T x, T y);

	};

	template<typename T>
	bool Camera<T>::Init(CameraSetting<T>& setting, glm::mat<4, 4, T> modelMat)
	{
		//��������
		mode = setting.cameraMode;
		position = setting.cameraPosition;
		target = setting.cameraTarget;

		//������
		mouseSensitivity = setting.mouseSensitivity;
		keyboardSensitivity = setting.keyboardSensitivity;
		scrollSensitivity = setting.scrollSensitivity;

		//���ƫ��
		maxPitchRate = setting.maxPitchRate;
		maxHeadingRate = setting.maxHeadingRate;

		//��׶
		nearClipPlane = setting.nearClipPlane;
		farClipPlane = setting.farClipPlane;
		screenWidth = setting.screenWidth;
		screenHeight = setting.screenHeight;
		viewportWidth = setting.viewportWidth;
		viewportHeight = setting.viewportHeight;
		aspect = viewportWidth / viewportHeight;
		FOV = setting.FOV;

		//������Ҫ��ʼ������
		deltaDisplacement = glm::vec<3, T>(T(0), T(0), T(0));
		//up = glm::vec<3, T>(T(0), T(0), T(1));
		up = setting.cameraUp;
		mouseMove = false;
		leftControlPress = false;
		rotate = false;

		//�����Զ�model�����任��camera��������MVP����
		this->modelMat = modelMat;
		projectMat = viewMat = glm::mat<4, 4, T>(T(1));

		roll = heading = pitch = T(0);
		return true;
	}

	template<typename T>
	void Camera<T>::Update()
	{
		//���������ǰ��
		front = glm::normalize(target - position);
		//����������޷�����ʹ�ù���
		//glViewport(0, 0, screenWidth, screenHeight);
		//���ݼ��������������λ�ã��Ƕȵ���Ϣ
		if (mode == CameraMode::CAMERA_ORTHOGONALITY) { std::cout << "ʩ����...\n"; }

		projectMat = glm::perspective(FOV, aspect, nearClipPlane, farClipPlane);

		if (mode == CameraMode::CAMERA_PROJECTION)
		{
			projectMat = glm::perspective(FOV, aspect, nearClipPlane, farClipPlane);
			//����������,front��up���ǵ�λ����
			right = glm::cross(front, up);
			//pitch����right����ת��
			glm::quat pitchQuat = glm::angleAxis(pitch, right);
			//�����µ�up
			up = glm::cross(right, front);
			//heading����up����ת��
			glm::quat headingQuat = glm::angleAxis(heading, up);
			//�ۺ����ֱ任
			glm::quat totalQuat = glm::cross(pitchQuat, headingQuat);
			//��Ҫ��λ��Ԫ��
			totalQuat = glm::normalize(totalQuat);
			//��Ԫ��->��ת����
			glm::mat<4, 4, T> rotation = glm::mat4_cast(totalQuat);
			//ת��camera�ķ��򣬼ȶ�front�������б任
			front = rotation * glm::vec<4, T>(front, 1.0f);
			//����cameraλ��
			position = position + deltaDisplacement;
			//�µ�target����
			target = position + front * T(1);
			//ArcBall �������ת
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
			//ע�������������������������������
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
		//���������ת
		if (degrees < -maxPitchRate)
			degrees = -maxPitchRate;
		else if (degrees > maxPitchRate)
			degrees = maxPitchRate;

		pitch += degrees;

		//�޶���һȦ��
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

		//heading��Ҫ��һЩ��������ƣ�pitch�����ʱ��Ҫ�ʵ���Сheading
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
		//��ת�ĽǶ�
		T angle = T(0);
		if (mousePosition != lastMousePosition) {
			glm::vec<3, T> va = ProjectPointToArcBall(lastMousePosition.x, lastMousePosition.y);
			glm::vec<3, T> vb = ProjectPointToArcBall(mousePosition.x, mousePosition.y);

			//�⻬�Ĺ켣��
			va = glm::normalize(va);
			vb = glm::normalize(vb);
			//angle = std::acos(std::min(T(1), glm::dot(va, vb)));

			angle = std::acos(std::min(T(1), glm::dot(va, vb)));

			/*glm::vec<3, T> axisInCameraSpace = glm::cross(va, vb);

			glm::vec<3, T> axisInWorldSpace = glm::inverse(viewMat) * glm::vec<4, T>(axisInCameraSpace, T(1));

			axisInWorldSpace = glm::normalize(axisInWorldSpace);*/

			glm::vec<2, T> aux = mousePosition - lastMousePosition;

			//ˮƽ��ת
			RotateH(aux.y, angle);
			//��ֱ��ת
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
		//�任��[-1,1]����
		T xx = x / (screenWidth / T(2)) - T(1);
		T yy = T(1) - y / (screenHeight / T(2));

		glm::vec<3, T> pointOnArcBall;

		T tmp = xx * xx + yy * yy;

		if (tmp <= T(1))
			pointOnArcBall = glm::vec<3, T>(xx, yy, std::sqrt(T(1) - tmp));
		else
			pointOnArcBall = glm::vec<3, T>(xx, yy, T(1) / (std::sqrt(1 + yy * yy)));

		return pointOnArcBall;
		////�ǹ⻬�Ĺ켣��
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




