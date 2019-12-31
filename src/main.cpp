#include<iostream>
#include<string>
#include"Render.hpp"

using Type = float;

int main(int argc, char const* argv[]) {

	if (argc < 2)
	{
		std::cerr << "Please indicate a simulator configuration file\n";
		exit(0);
	}

	std::string simulator_config_file = std::string(argv[1]);

	//std::cout << simulator_config_file << "\n";
	//std::string simulator_config_file = "./conf/sphere.json";

	int screenWidth = 1920;
	int screenHeight = 1080;

	lyra::CameraSetting<Type> camera_setting;

	{
		//glm::rotate(modelMat, 90.f/180*PI, glm::vec3(1.f, 0.f, 0.f));
		//如果相机位置正好和上向量(0.f,1.f,0.f)平行，那么就会出问题
		glm::vec<3, Type> cameraPosition(1.0f, 1.5f, 2.f);
		glm::vec<3, Type> cameraTarget(0.f, 0.f, 0.f);
		camera_setting.cameraMode = lyra::CameraMode::CAMERA_PROJECTION;
		camera_setting.cameraPosition = cameraPosition;
		camera_setting.cameraTarget = cameraTarget;
		camera_setting.cameraUp = glm::vec<3, Type>(Type(0), Type(0), Type(1));
		camera_setting.scrollSensitivity = 0.3f;
		camera_setting.keyboardSensitivity = 0.1f;
		camera_setting.mouseSensitivity = 0.05f;
		camera_setting.maxPitchRate = 0.004f;
		camera_setting.maxHeadingRate = 0.004f;
		camera_setting.nearClipPlane = 0.05f;
		camera_setting.farClipPlane = 100.f;
		camera_setting.viewportWidth = screenWidth;
		camera_setting.viewportHeight = screenHeight;
		camera_setting.screenWidth = screenWidth;
		camera_setting.screenHeight = screenHeight;
		camera_setting.FOV = 20.f;
	}

	lyra::Simulator<float> simulator;
	lyra::Render<float> render;
	simulator.InitSimulator(simulator_config_file);
	render.InitRender(camera_setting,screenWidth,screenHeight);
	//render.InitSimulator(simulator_config_file);
	render.Run(simulator);
	return 0;
}