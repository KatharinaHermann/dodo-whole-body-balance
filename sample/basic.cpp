/*  Copyright © 2018, Roboti LLC

    This file is licensed under the MuJoCo Resource License (the "License").
    You may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        https://www.roboti.us/resourcelicense.txt
*/


#include "mujoco.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"


#include "eigenUtils.h"

#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <vector>



// select EGL, OSMESA or GLFW
#if defined(MJ_EGL)
    #include <EGL/egl.h>
#elif defined(MJ_OSMESA)
    #include <GL/osmesa.h>
    OSMesaContext ctx;
    unsigned char buffer[10000000];
#else
    #include "glfw3.h"
#endif


//-------------------------------- global data ------------------------------------------

// MuJoCo model and data
mjModel* m = 0;
mjData* d = 0;

// MuJoCo visualization
mjvScene scn;
mjvCamera cam;
mjvOption opt;
mjrContext con;

//Steps for switching tto balancer mode
int steps =0;

//-------------------------------- utility functions ------------------------------------

// load model, init simulation and rendering
void initMuJoCo(const char* filename)
{
    // activate
    mj_activate("mjkey.txt");

    // load and compile
    char error[1000] = "Could not load binary model";
    if( strlen(filename)>4 && !strcmp(filename+strlen(filename)-4, ".mjb") )
        m = mj_loadModel(filename, 0);
    else
        m = mj_loadXML(filename, 0, error, 1000);
    if( !m )
        mju_error_s("Load model error: %s", error);

    // make data, run one computation to initialize all fields
    d = mj_makeData(m);
    mj_forward(m, d);

    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);

    // create scene and context
    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, 200);

    // center and scale view
    cam.lookat[0] = m->stat.center[0];
    cam.lookat[1] = m->stat.center[1];
    cam.lookat[2] = m->stat.center[2];
    cam.distance = 1.5 * m->stat.extent;
}


// deallocate everything and deactivate
void closeMuJoCo(void)
{
    mj_deleteData(d);
    mj_deleteModel(m);
    mjr_freeContext(&con);
    mjv_freeScene(&scn);
    mj_deactivate();
}


// create OpenGL context/window
void initOpenGL(void)
{
    //------------------------ EGL
#if defined(MJ_EGL)
    // desired config
    const EGLint configAttribs[] ={
        EGL_RED_SIZE,           8,
        EGL_GREEN_SIZE,         8,
        EGL_BLUE_SIZE,          8,
        EGL_ALPHA_SIZE,         8,
        EGL_DEPTH_SIZE,         24,
        EGL_STENCIL_SIZE,       8,
        EGL_COLOR_BUFFER_TYPE,  EGL_RGB_BUFFER,
        EGL_SURFACE_TYPE,       EGL_PBUFFER_BIT,
        EGL_RENDERABLE_TYPE,    EGL_OPENGL_BIT,
        EGL_NONE
    };

    // get default display
    EGLDisplay eglDpy = eglGetDisplay(EGL_DEFAULT_DISPLAY);
    if( eglDpy==EGL_NO_DISPLAY )
        mju_error_i("Could not get EGL display, error 0x%x\n", eglGetError());

    // initialize
    EGLint major, minor;
    if( eglInitialize(eglDpy, &major, &minor)!=EGL_TRUE )
        mju_error_i("Could not initialize EGL, error 0x%x\n", eglGetError());

    // choose config
    EGLint numConfigs;
    EGLConfig eglCfg;
    if( eglChooseConfig(eglDpy, configAttribs, &eglCfg, 1, &numConfigs)!=EGL_TRUE )
        mju_error_i("Could not choose EGL config, error 0x%x\n", eglGetError());

    // bind OpenGL API
    if( eglBindAPI(EGL_OPENGL_API)!=EGL_TRUE )
        mju_error_i("Could not bind EGL OpenGL API, error 0x%x\n", eglGetError());

    // create context
    EGLContext eglCtx = eglCreateContext(eglDpy, eglCfg, EGL_NO_CONTEXT, NULL);
    if( eglCtx==EGL_NO_CONTEXT )
        mju_error_i("Could not create EGL context, error 0x%x\n", eglGetError());

    // make context current, no surface (let OpenGL handle FBO)
    if( eglMakeCurrent(eglDpy, EGL_NO_SURFACE, EGL_NO_SURFACE, eglCtx)!=EGL_TRUE )
        mju_error_i("Could not make EGL context current, error 0x%x\n", eglGetError());

    //------------------------ OSMESA
#elif defined(MJ_OSMESA)
    // create context
    ctx = OSMesaCreateContextExt(GL_RGBA, 24, 8, 8, 0);
    if( !ctx )
        mju_error("OSMesa context creation failed");

    // make current
    if( !OSMesaMakeCurrent(ctx, buffer, GL_UNSIGNED_BYTE, 800, 800) )
        mju_error("OSMesa make current failed");

    //------------------------ GLFW
#else
    // init GLFW
    if( !glfwInit() )
        mju_error("Could not initialize GLFW");

    // create invisible window, single-buffered
    glfwWindowHint(GLFW_VISIBLE, 0);
    glfwWindowHint(GLFW_DOUBLEBUFFER, GLFW_FALSE);
    GLFWwindow* window = glfwCreateWindow(800, 800, "Invisible window", NULL, NULL);
    if( !window )
        mju_error("Could not create GLFW window");

    // make context current
    glfwMakeContextCurrent(window);
#endif
}


// close OpenGL context/window
void closeOpenGL(void)
{
    //------------------------ EGL
#if defined(MJ_EGL)
    // get current display
    EGLDisplay eglDpy = eglGetCurrentDisplay();
    if( eglDpy==EGL_NO_DISPLAY )
        return;

    // get current context
    EGLContext eglCtx = eglGetCurrentContext();

    // release context
    eglMakeCurrent(eglDpy, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);

    // destroy context if valid
    if( eglCtx!=EGL_NO_CONTEXT )
        eglDestroyContext(eglDpy, eglCtx);

    // terminate display
    eglTerminate(eglDpy);

    //------------------------ OSMESA
#elif defined(MJ_OSMESA)
    OSMesaDestroyContext(ctx);

    //------------------------ GLFW
#else
    // terminate GLFW (crashes with Linux NVidia drivers)
    #if defined(__APPLE__) || defined(_WIN32)
        glfwTerminate();
    #endif
#endif
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
        K_t << 80.0, 0, 0,
             0, 80.0, 0,
            0, 0, 10000.0;
        MatrixXd D_t = MatrixXd::Zero(3, 3);
        D_t << 5.0, 0, 0,
             0, 5.0, 0,
            0, 0, 400.0;
        MatrixXd K_r = MatrixXd::Zero(3, 3);
        K_r = 100.0f * MatrixXd::Identity(3,3);
        MatrixXd D_r = MatrixXd::Zero(3, 3);
        D_r = 1.0f * MatrixXd::Identity(3,3);

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

        VectorXd F_k = VectorXd::Zero(6); //contact wrench

        //Auxiliary variables for final equation 2
        VectorXd x_k = VectorXd::Zero(3); //Vector from W to the contact point k
        VectorXd x_COM_k = VectorXd::Zero(3); //Vector from COM to the contact point k
        MatrixXd x_COM_k_skew = MatrixXd::Zero(3, 3);
        MatrixXd G_T= MatrixXd::Zero(3, 3);


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

            cout<<x_COM_desired<<endl;
            cout<<steps<<endl;

            //Desired base orientation
            Q_b_desired.w() = d->qpos[3];
            Q_b_desired.x() = d->qpos[4];
            Q_b_desired.y() = d->qpos[5];
            Q_b_desired.z() = d->qpos[6];

            steps++;
        }

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
                J_COMi = Map<Matrix<double, Dynamic, Dynamic, RowMajor>>(J_COMi_temp, 3, m->nv);
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

            G.block(0,0,6,6) = eigenUtils::adjointTransformation(x_COM_k, MatrixXd::Identity(3,3));

            //x_COM_k_skew = eigenUtils::skewSymmetric(x_COM_k);
            //G.block(0,0,6,6) = MatrixXd::Identity(6,6);
            //G.block(3,0,3,3) = x_COM_k_skew;

            //Contact point 2
            x_k = Map<VectorXd>(d->xpos+contact_point_2*3, 3);
            x_COM_k = x_k - x_COM;

            G.block(0,6,6,6) = eigenUtils::adjointTransformation(x_COM_k, MatrixXd::Identity(3,3));
            //x_COM_k_skew = eigenUtils::skewSymmetric(x_COM_k);
            //G.block(0,6,6,6) = MatrixXd::Identity(6,6);
            //G.block(3,6,3,3) = x_COM_k_skew;

        //Variables for Equation 3
            // Compute the Jacobian J_b_COM from the base in "torso" b to COM.
            mjtNum* J_b_temp = mj_stackAlloc(d, 3*m->nv);
            mj_jacBody(m, d, J_b_temp, NULL, torso_id );
            J_b = Map<Matrix<double, Dynamic, Dynamic, RowMajor>>(J_b_temp, 3, m->nv);
            J_b_COM = J_COM - J_b;

            mjtNum* J_bk_trans_temp = mj_stackAlloc(d, 3*m->nv);
            mjtNum* J_bk_rot_temp = mj_stackAlloc(d, 3*m->nv);

            //Contact point 1
            mj_jacBody(m, d, J_bk_trans_temp, J_bk_rot_temp, contact_point_1);
            J_bk_trans = Map<Matrix<double, Dynamic, Dynamic, RowMajor>>(J_bk_trans_temp, 3, m->nv);
            J_bk_rot = Map<Matrix<double, Dynamic, Dynamic, RowMajor>>(J_bk_rot_temp, 3, m->nv);

            J_COM_k.block(0,0,3, m->nv) = J_bk_trans - J_b_COM;
            J_COM_k.block(3,0,3, m->nv) = J_bk_rot;
            J_COM_K.block(0,0,6, m->nv) = J_COM_k;

            //Contact point _2
            mj_jacBody(m, d, J_bk_trans_temp, J_bk_rot_temp, contact_point_2);
            J_bk_trans = Map<Matrix<double, Dynamic, Dynamic, RowMajor>>(J_bk_trans_temp, 3, m->nv);
            J_bk_rot = Map<Matrix<double, Dynamic, Dynamic, RowMajor>>(J_bk_rot_temp, 3, m->nv);

            J_COM_k.block(0,0,3, m->nv) = J_bk_trans - J_b_COM;
            J_COM_k.block(3,0,3, m->nv) = J_bk_rot;
            J_COM_K.block(6,0,6, m->nv) = J_COM_k;

        mjFREESTACK


        //Final calculations

        //First step
        f_COM = mj_getTotalmass(m) * gravity_vector - K_t* (x_COM - x_COM_desired) - D_t * v_COM;
        eigenUtils::virtualSpringPD(m_COM, e_or, edot_or, Q_b, Q_b_desired, w_b,  w_b_desired, K_r, D_r);

        F_COM.head(3) = f_COM;
        F_COM.tail(3) = m_COM;

        //Second step
        //Pseudo inverse
        F_k = G.bdcSvd(ComputeThinU | ComputeThinV).solve(F_COM);

        // GG_T = G* G.transpose();

        //G_PI = G.transpose()* GG_T.inverse();
        //F_k= G_PI * F_COM;

        //Third step
        J_COM_K_T= J_COM_K.transpose();
        tau = J_COM_K_T * F_k;

        for(int i=0;i < m->nu; i++) {
            d->qfrc_applied[i+6] = tau(i+6);
            }

    }


}

//-------------------------------- main function ----------------------------------------

int main(int argc, const char** argv)
{
    // check command-line arguments
    if( argc!=5 )
    {
        printf(" USAGE:  record modelfile duration fps rgbfile\n");
        return 0;
    }

    // parse numeric arguments
    double duration = 10, fps = 30;
    sscanf(argv[2], "%lf", &duration);
    sscanf(argv[3], "%lf", &fps);

    // initialize OpenGL and MuJoCo
    initOpenGL();
    initMuJoCo(argv[1]);

    // set rendering to offscreen buffer
    mjr_setBuffer(mjFB_OFFSCREEN, &con);
    if( con.currentBuffer!=mjFB_OFFSCREEN )
        printf("Warning: offscreen rendering not supported, using default/window framebuffer\n");

    // get size of active renderbuffer
    mjrRect viewport =  mjr_maxViewport(&con);
    int W = viewport.width;
    int H = viewport.height;

    // allocate rgb and depth buffers
    unsigned char* rgb = (unsigned char*)malloc(3*W*H);
    float* depth = (float*)malloc(sizeof(float)*W*H);
    if( !rgb || !depth )
        mju_error("Could not allocate buffers");

    // create output rgb file
    FILE* fp = fopen(argv[4], "wb");
    if( !fp )
        mju_error("Could not open rgbfile for writing");

    // main loop
    double frametime = 0;
    int framecount = 0;



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



    while( d->time<duration )
    {
        // render new frame if it is time (or first frame)
        if( (d->time-frametime)>1/fps || frametime==0 )
        {
            // update abstract scene
            mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);

            // render scene in offscreen buffer
            mjr_render(viewport, &scn, &con);

            // add time stamp in upper-left corner
            char stamp[50];
            sprintf(stamp, "Time = %.3f", d->time);
            mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport, stamp, NULL, &con);

            // read rgb and depth buffers
            mjr_readPixels(rgb, depth, viewport, &con);

            // insert subsampled depth image in lower-left corner of rgb image
            const int NS = 3;           // depth image sub-sampling
            for( int r=0; r<H; r+=NS )
                for( int c=0; c<W; c+=NS )
                {
                    int adr = (r/NS)*W + c/NS;
                    rgb[3*adr] = rgb[3*adr+1] = rgb[3*adr+2] =
                        (unsigned char)((1.0f-depth[r*W+c])*255.0f);
                }

            // write rgb image to file
            fwrite(rgb, 3, W*H, fp);

            // print every 10 frames: '.' if ok, 'x' if OpenGL error
            if( ((framecount++)%10)==0 )
            {
                if( mjr_getError() )
                    printf("x");
                else
                    printf(".");
            }

            // save simulation time
            frametime = d->time;
        }

        // advance simulation
        mj_step(m, d);
    }
    printf("\n");

    // close file, free buffers
    fclose(fp);
    free(rgb);
    free(depth);

    // close MuJoCo and OpenGL
    closeMuJoCo();
    closeOpenGL();

    return 1;
}
