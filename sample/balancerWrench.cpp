/*  Copyright © 2018, Roboti LLC

    This file is licensed under the MuJoCo Resource License (the "License").
    You may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        https://www.roboti.us/resourcelicense.txt
*/


#include "mujoco.h"
#include "glfw3.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"

#include "eigenUtils.h"

#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <vector>


// MuJoCo data structures
mjModel* m = NULL;                  // MuJoCo model
mjData* d = NULL;                   // MuJoCo data
mjvCamera cam;                      // abstract camera
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right =  false;
double lastx = 0;
double lasty = 0;

bool firstLoop = true;
bool pushNow = true;


//Steps for switching tto balancer mode
int steps =0;

// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
    // backspace: reset simulation
    if( act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE )
    {
        mj_resetData(m, d);
        mj_forward(m, d);
        pushNow = true;
    }
}


// mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods)
{
    // update button state
    button_left =   (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
    button_right =  (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
}


// mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos)
{
    // no buttons down: nothing to do
    if( !button_left && !button_middle && !button_right )
        return;

    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if( button_right )
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if( button_left )
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    // move camera
    mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
}


// scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset)
{
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}

// controller callback funtcion
void controllerCallback(const mjModel* m, mjData* d)
{

    using namespace Eigen;
    using namespace std;
    //We only use that in order to place the Dodo in the first frame steps stabilized on the ground

    if(steps < 200){
        for(int i=0;i < m->nu; i++) {
                d->qfrc_applied[i+6]= 100.0*(m->qpos0[i+7] - d->qpos[i+7]) - 1.0*d->qvel[i+6] + d->qfrc_bias[i+6];
        }
    steps++;
    cout<<steps<<endl;
    }

    else {

        //Initialization
        //Constants
        const int number_contact_points = 2;
        const int contact_point_1 = mj_name2id(m,mjOBJ_BODY,"right_foot");
        const int contact_point_2 = mj_name2id(m,mjOBJ_BODY,"left_foot");
        const int torso_id = mj_name2id(m,mjOBJ_BODY,"torso");


        //1st step PD like controller

        const double gravity = 9.81;
        Vector3d gravity_vector(0,0, gravity);
        MatrixXd K_t = MatrixXd::Zero(3, 3);
        K_t << 100.0, 0, 0,
             0, 100.0, 0,
            0, 0, 100.0;
        MatrixXd D_t = MatrixXd::Zero(3, 3);
        D_t << 1.0, 0, 0,
             0, 1.0, 0,
            0, 0, 1.0;
        MatrixXd K_r = MatrixXd::Zero(3, 3);
        K_r << 100.0, 0, 0,
             0, 100.0, 0,
            0, 0, 100.0;
        MatrixXd D_r = MatrixXd::Zero(3, 3);
        D_r << 1.0, 0, 0,
             0, 1.0, 0,
            0, 0, 1.0;

        VectorXd x_COM_desired = VectorXd::Zero(3); //Desired COM position
        VectorXd x_COM = VectorXd::Zero(3); //Whole body COM position

        VectorXd v_COM = VectorXd::Zero(3); //Whole body COM velocity

        Quaterniond Q_b; //the current orientation of frame b
        Quaterniond Q_b_desired; //the desired orientation of frame b

        VectorXd w_b = VectorXd::Zero(3);
        VectorXd w_b_desired = VectorXd::Zero(3); //Is initialized as zero and stays zero as the angular velocity should be 0 for the quasi static case.
        VectorXd e_or = VectorXd::Zero(3);
        VectorXd edot_or = VectorXd::Zero(3);
        MatrixXd R_b = MatrixXd::Zero(3,3);

        VectorXd f_COM = VectorXd::Zero(3); //ground applied force
        VectorXd m_COM = VectorXd::Zero(3); //ground applied moment

        VectorXd F_COM = VectorXd::Zero(6); //ground applied wrench

        //Auxiliary variables for final equation 1
        VectorXd x_COMi = VectorXd::Zero(3); // COM position of body i
        VectorXd x_COMi_des = VectorXd::Zero(3); // COM position of body i
        VectorXd dq = VectorXd::Zero(m->nv); //velocity of the degrees of freedom


        //2nd step for force distribution with F_k= G_PI *F_COM
        MatrixXd G = MatrixXd::Zero(6, number_contact_points*6); //Contact map
        MatrixXd G_PI = MatrixXd::Zero(number_contact_points*6, 6); //Pseudo inversecontact map


        VectorXd f_k = VectorXd::Zero(3); //contact force
        VectorXd m_k = VectorXd::Zero(3); //contact moment

        VectorXd F_k = VectorXd::Zero(6); //contact wrench

        //Auxiliary variables for final equation 2
        VectorXd x_k = VectorXd::Zero(3); //Vector from W to the contact point k
        VectorXd x_COM_k = VectorXd::Zero(3); //Vector from COM to the contact point k
        MatrixXd x_COM_k_skew = MatrixXd::Zero(3, 3);
        MatrixXd GG_T= MatrixXd::Zero(3, 3);


        //3rd step wit tau=J_T*F_k

        MatrixXd J_b_COM = MatrixXd::Zero(3, m->nv); //Base to COM Jacobian in W frame with shape 3x number of degrees of freedom
        MatrixXd J_COM_K = MatrixXd::Zero(number_contact_points*6, m->nv); //Final Com to endeffector Jacobian in W frame with shape 6x number of degrees of freedom
        MatrixXd J_COM_K_T = MatrixXd::Zero(m->nv, number_contact_points*6); //Transpose final Jacobian

        VectorXd tau = VectorXd::Zero(m->nv); //contact wrench

        //Auxiliary variables for final equation 3
        MatrixXd J_COM = MatrixXd::Zero(3, m->nv); //COM Jacobian of all bodies with shape 3x number of degrees of freedom
        MatrixXd J_COMi = MatrixXd::Zero(3, m->nv); //COM Jacobian of body i with shape 3x number of degrees of freedom
        MatrixXd J_b = MatrixXd::Zero(3, m->nv); //Base Jacobian with shape 3x number of degrees of freedom

        MatrixXd J_bk_trans = MatrixXd::Zero(3, m->nv); //Translational part of Com to endeffector k Jacobian with shape 3x number of degrees of freedom
        MatrixXd J_bk_rot = MatrixXd::Zero(3, m->nv); //Rotational part of Com to endeffector k Jacobian with shape 3x number of degrees of freedom
        MatrixXd J_COM_k = MatrixXd::Zero(6, m->nv); //Com to endeffector k Jacobian in W frame with shape 6x number of degrees of freedom


        //Implementation

        //Calculating the desired variables

        //Desired COM position
        if(steps == 200){
            for (int i=1; i< m->nbody; i++) {//We start at i=1 since i=0 is the world frame
                //Calculate COM position
                x_COMi_des = Map<VectorXd> (d->subtree_com+3*i, 3);
                x_COM_desired += (m->body_mass[i] * x_COMi_des);
            }
            x_COM_desired /= mj_getTotalmass(m);

            steps++;
            cout<<x_COM_desired<<endl;
            cout<<steps<<endl;
        }

        //Desired base orientation
        Q_b_desired.w() = m->qpos0[3];
        Q_b_desired.x() = m->qpos0[4];
        Q_b_desired.y() = m->qpos0[5];
        Q_b_desired.z() = m->qpos0[6];

        mjMARKSTACK
        //Basics and variables for Equation 1

            //Computing the COM Jacobian and the COM
            mjtNum* J_COMi_temp = mj_stackAlloc(d, 3*m->nv);

            for (int i=1; i< m->nbody; i++) {//We start at i=1 since i=0 is the world frame
                //Calculate COM position
                x_COMi = Map<VectorXd> (d->subtree_com+3*i, 3);
                x_COM += (m->body_mass[i] * x_COMi);

                //Calculate J_COMi
                mj_jacBody(m, d, J_COMi_temp, NULL, i);
                J_COMi = Map<MatrixXd>(J_COMi_temp, 3, m->nv);
                J_COM += (m->body_mass[i] * J_COMi);
            }

            J_COM /= mj_getTotalmass(m);
            x_COM /= mj_getTotalmass(m);

            //Compute the COM veloxity v_COM

            dq = Map<VectorXd>(d->qvel,m->nv);
            v_COM = J_COM *dq;

            //Compute Quaternions for current base orientation
            Q_b.w() = d->qpos[3];
            Q_b.x() = d->qpos[4];
            Q_b.y() = d->qpos[5];
            Q_b.z() = d->qpos[6];
            Q_b.normalize();

            //Compute the current base velocity
            w_b = Map<VectorXd>(d->qvel+3, 3);

        //Variables for Equation 2
            //Contact point 1
            x_k = Map<VectorXd>(d->xpos+contact_point_1*3, 3);
            x_COM_k = x_k - x_COM;

            x_COM_k_skew = eigenUtils::skewSymmetric(x_COM_k);
            G.block(0,0,6,6) = MatrixXd::Identity(6,6);
            G.block(3,0,3,3) = x_COM_k_skew;

            //Contact point 2
            x_k = Map<VectorXd>(d->xpos+contact_point_1*3, 3);
            x_COM_k = x_k - x_COM;

            x_COM_k_skew = eigenUtils::skewSymmetric(x_COM_k);
            G.block(0,6,6,6) = MatrixXd::Identity(6,6);
            G.block(3,6,3,3) = x_COM_k_skew;

        //Variables for Equation 3
            // Compute the Jacobian J_b_COM from the base in "torso" b to COM.
            mjtNum* J_b_temp = mj_stackAlloc(d, 3*m->nv);
            mj_jacBody(m, d, J_b_temp, NULL, torso_id );
            J_b = Map<MatrixXd>(J_b_temp, 3, m->nv);
            J_b_COM = J_COM - J_b;


            //mjtNum* temp_J = mj_stackAlloc(d, 3*m->nv);
            mjtNum* J_bk_trans_temp = mj_stackAlloc(d, 3*m->nv);
            mjtNum* J_bk_rot_temp = mj_stackAlloc(d, 3*m->nv);

            //Contact point 1
            mj_jacBody(m, d, J_bk_trans_temp, J_bk_rot_temp, contact_point_1);
            J_bk_trans = Map<MatrixXd>(J_bk_trans_temp, 3, m->nv);
            J_bk_rot = Map<MatrixXd>(J_bk_rot_temp, 3, m->nv);

            J_COM_k.block(0,0,3, m->nv) = J_bk_trans - J_b_COM;
            J_COM_k.block(3,0,3, m->nv) = J_bk_rot;
            J_COM_K.block(0,0,6, m->nv) = J_COM_k;

            //Contact point _2
            mj_jacBody(m, d, J_bk_trans_temp, J_bk_rot_temp, contact_point_2);
            J_bk_trans = Map<MatrixXd>(J_bk_trans_temp, 3, m->nv);
            J_bk_rot = Map<MatrixXd>(J_bk_rot_temp, 3, m->nv);

            J_COM_k.block(0,0,3, m->nv) = J_bk_trans - J_b_COM;
            J_COM_k.block(3,0,3, m->nv) = J_bk_rot;
            J_COM_K.block(6,0,6, m->nv) = J_COM_k;

        mjFREESTACK
                
                
        //Final calculations

        //First step
        f_COM = mj_getTotalmass(m) * gravity_vector + K_t*(x_COM - x_COM_desired) + D_t * v_COM;

        eigenUtils::virtualSpringPD(m_COM, e_or, edot_or, Q_b, Q_b_desired, w_b,  w_b_desired, K_r, D_r);
        F_COM.head(3) = f_COM;
        F_COM.tail(3) = m_COM;

        //Second step
        GG_T = G* G.transpose();

        G_PI = G.transpose()* GG_T.inverse();
        F_k= G_PI * F_COM;

        //Third step
        J_COM_K_T= J_COM_K.transpose();
        tau = J_COM_K_T*F_k;

        for(int i=0;i < m->nu; i++) {
            d->qfrc_applied[i+6] = tau(i);
            }

    }


}


// main function
int main(int argc, const char** argv)
{
    // check command-line arguments
    if( argc!=2 )
    {
        printf(" USAGE:  basic modelfile\n");
        return 0;
    }

    // activate software
    mj_activate("mjkey.txt");

    // load and compile model
    char error[1000] = "Could not load binary model";
    if( strlen(argv[1])>4 && !strcmp(argv[1]+strlen(argv[1])-4, ".mjb") )
        m = mj_loadModel(argv[1], 0);
    else
        m = mj_loadXML(argv[1], 0, error, 1000);
    if( !m )
        mju_error_s("Load model error: %s", error);

    //Options for solver
    m->opt.jacobian = mjJAC_DENSE;
    m->opt.cone = mjCONE_ELLIPTIC;

    // make data
    d = mj_makeData(m);

    // init GLFW
    if( !glfwInit() )
        mju_error("Could not initialize GLFW");

    // create window, make OpenGL context current, request v-sync
    GLFWwindow* window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);

    // create scene and context
    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);



    // set initial values for the floating base
    m->qpos0[2] = 1.1;

    // set initial values for joints
    m->qpos0[6+mj_name2id(m, mjOBJ_JOINT, "right_hip_x")]     = 0.0;
    m->qpos0[6+mj_name2id(m, mjOBJ_JOINT, "right_hip_z")]     = 0.0*  M_PI/180.0;
    m->qpos0[6+mj_name2id(m, mjOBJ_JOINT, "right_hip_y")]     = 65.0*  M_PI/180.0;
    m->qpos0[6+mj_name2id(m, mjOBJ_JOINT, "right_knee")]      = 30.0*  M_PI/180.0;
    m->qpos0[6+mj_name2id(m, mjOBJ_JOINT, "right_ankle_y")]   = 65.0*  M_PI/180.0;
    m->qpos0[6+mj_name2id(m, mjOBJ_JOINT, "right_ankle_x")]   = 0.0;

    m->qpos0[6+mj_name2id(m, mjOBJ_JOINT, "left_hip_x")]      = 0.0;
    m->qpos0[6+mj_name2id(m, mjOBJ_JOINT, "left_hip_z")]      = 0.0*  M_PI/180.0;
    m->qpos0[6+mj_name2id(m, mjOBJ_JOINT, "left_hip_y")]      = 65.0*  M_PI/180.0;
    m->qpos0[6+mj_name2id(m, mjOBJ_JOINT, "left_knee")]       = 30.0*  M_PI/180.0;
    m->qpos0[6+mj_name2id(m, mjOBJ_JOINT, "left_ankle_y")]    = 65.0*  M_PI/180.0;
    m->qpos0[6+mj_name2id(m, mjOBJ_JOINT, "left_ankle_x")]    = 0.0;

    mj_resetData(m, d);

    mjcb_control = controllerCallback;



    // run main loop, target real-time simulation and 60 fps rendering
    while( !glfwWindowShouldClose(window) )
    {
        // advance interactive simulation for 1/60 sec
        //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
        //  this loop will finish on time for the next frame to be rendered at 60 fps.
        //  Otherwise add a cpu timer and exit this loop when it is time to render.
        mjtNum simstart = d->time;
        while( d->time - simstart < 1.0/60.0 )
            mj_step(m, d);

        // get framebuffer viewport
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        // update scene and render
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);

        // swap OpenGL buffers (blocking call due to v-sync)
        glfwSwapBuffers(window);

        // process pending GUI events, call GLFW callbacks
        glfwPollEvents();
    }

    //free visualization storage
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    // free MuJoCo model and data, deactivate
    mj_deleteData(d);
    mj_deleteModel(m);
    mj_deactivate();

    // terminate GLFW (crashes with Linux NVidia drivers)
#if defined(__APPLE__) || defined(_WIN32)
    glfwTerminate();
#endif

    return 1;
}
