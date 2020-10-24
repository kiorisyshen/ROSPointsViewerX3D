#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include "glad/glad.h"
#include "x3d_linux.hpp"

// Use imgui
#include "imgui.h"
#include "imgui_impl_glfw.h"

static X3D::X3DGate x3dGate;

static double cursorPosX_old;
static double cursorPosY_old;
static void cursor_position_callback(GLFWwindow *window, double xpos, double ypos) {
    if (ImGui::IsAnyWindowFocused()) {
        cursorPosX_old = xpos;
        cursorPosY_old = ypos;
        return;
    }
    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS) {
        x3dGate.Rotate(x3dGate.GetDefaultCam(), 0.5f * float(xpos - cursorPosX_old), 0.5f * float(ypos - cursorPosY_old));
    }

    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS) {
        x3dGate.Move(x3dGate.GetDefaultCam(), 0.03f * float(xpos - cursorPosX_old), 0.03f * float(ypos - cursorPosY_old));
    }

    cursorPosX_old = xpos;
    cursorPosY_old = ypos;
}

static void scroll_callback(GLFWwindow *window, double xoffset, double yoffset) {
    if (ImGui::IsAnyWindowFocused()) {
        return;
    }
    x3dGate.Move(x3dGate.GetDefaultCam(), float(yoffset));
}

void Pt2DataCbk(const sensor_msgs::PointCloud2ConstPtr &pt2_in) {
    ROS_INFO("Pt2 callback");
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pt2_X3D_viewer");
    ros::NodeHandle nh;

    // Read params from launch file
    int winWIDTH  = 800;
    int winHEIGHT = 600;
    int FPS       = 60;
    if (!ros::param::get("initWidth", winWIDTH) &&
        !ros::param::get("initHeight", winHEIGHT) &&
        !ros::param::get("FPS", FPS)) {
        std::cout << "Can not get the value from launch file" << std::endl;
        exit(1);
    }

    ros::Rate rosRate(FPS);

    ros::Subscriber sub_pt2 = nh.subscribe<sensor_msgs::PointCloud2>("/livox/lidar", 100, Pt2DataCbk);

    /* create window and GL context via GLFW */
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_RESIZABLE, GLFW_TRUE);
    GLFWwindow *w = glfwCreateWindow(winWIDTH, winHEIGHT, "X3D PointCloud2 Viewer", 0, 0);
    glfwMakeContextCurrent(w);
    glfwSwapInterval(1);

    glfwSetCursorPosCallback(w, cursor_position_callback);
    glfwSetScrollCallback(w, scroll_callback);

    X3D::X3DConfig x3dConfig;
    x3dConfig.logLvl     = X3D::LOG_LEVEL::trace;
    x3dConfig.viewWidth  = winWIDTH;
    x3dConfig.viewHeight = winHEIGHT;
    x3dConfig.fps        = FPS;

    x3dGate.Initialize(x3dConfig, (GLADloadproc)glfwGetProcAddress);

    // imgui glfw setup
    ImGui_ImplGlfw_InitForOpenGL(w, true);
    float cube_color[3] = {1.0f, 1.0f, 1.0f};  // for debug
    bool show_debug     = false;

    /* draw loop */
    while (!glfwWindowShouldClose(w)) {
        ros::spinOnce();  // Spin ros first for reading pt2

        int cur_width, cur_height;
        glfwGetFramebufferSize(w, &cur_width, &cur_height);

        if (cur_width != winWIDTH || cur_height != winHEIGHT) {
            // Resize view
            x3dGate.ResizeView(cur_width, cur_height);
            winWIDTH  = cur_width;
            winHEIGHT = cur_height;
        }

        // imgui glfw tick
        ImGui_ImplGlfw_NewFrame();
        x3dGate.newFrame_IMGUI();

        // A simple gui window
        {
            ImGui::Begin("X3D client imgui example");

#ifdef DEBUG_FUNC
            if (ImGui::Button("Toggle debug")) {  // Buttons return true when clicked (most widgets return true when edited/activated)
                x3dGate.ToggleDebug();
                show_debug = !show_debug;
            }
            if (show_debug) {
                bool cube_color_changed = ImGui::ColorEdit3("clear color", cube_color);  // Edit 3 floats representing a color
                if (cube_color_changed) {
                    x3dGate.debugUpdateObjColor(x3dGate.debugGetDefaultObj(), cube_color[0], cube_color[1], cube_color[2]);
                }
            }
#endif

            ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);

            ImGui::End();
        }

        x3dGate.RenderOnce();

        glfwSwapBuffers(w);
        glfwPollEvents();

        rosRate.sleep();
    }

    /* cleanup */
    ImGui_ImplGlfw_Shutdown();

    x3dGate.Finalize();
    glfwTerminate();

    return 0;
}