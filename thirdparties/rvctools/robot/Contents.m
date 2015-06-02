% Robotics Toolbox.
% Version 9.0  2011
%
%
% Homogeneous transformations
%    angvec2r                   - angle/vector to RM
%    angvec2tr                  - angle/vector to HT
%    eul2r                      - Euler angles to RM
%    eul2tr                     - Euler angles to HT
%    oa2r                       - orientation and approach vector to RM
%    oa2tr                      - orientation and approach vector to HT
%    r2t                        - RM to HT
%    rt2tr                      - (R,t) to HT
%    rotx                       - RM for rotation about X-axis
%    roty                       - RM for rotation about Y-axis
%    rotz                       - RM for rotation about Z-axis
%    rpy2r                      - roll/pitch/yaw angles to RM
%    rpy2tr                     - roll/pitch/yaw angles to HT
%    se2                        - HT in SE(2)
%    se3                        - HT in SE(2)
%    t2r                        - HT to RM
%    tr2angvec                  - HT/RM to angle/vector form
%    tr2eul                     - HT/RM to Euler angles
%    tr2rpy                     - HT/RM to roll/pitch/yaw angles
%    tranimate                  - animate a coordinate frame
%    transl                     - set or extract the translational component of HT
%    trnorm                     - normalize HT
%    trplot                     - plot HT as a coordinate frame
%    trplot2                    - plot HT, SE(2), as a coordinate frame
%    trotx                      - HT for rotation about X-axis
%    troty                      - HT for rotation about Y-axis
%    trotz                      - HT for rotation about Z-axis
%
% Homogeneous points and lines
%    e2h                        - Euclidean coordinates to homogeneous
%    h2e                        - homogeneous coordinates to Euclidean
%    homline                    - create line from 2 points
%    homtrans                   - transform points
%
%  * HT = homogeneous transformation, a 4x4 matrix, belongs to the group SE(3).
%  * RM = RM, an orthonormal 3x3 matrix, belongs to the group SO(3).
%  * Functions of the form <b>tr2XX</b> will also accept a RM as the argument.
%
% Differential motion
%    delta2tr                   - differential motion vector to HT
%    eul2jac                    - Euler angles to Jacobian
%    rpy2jac                    - RPY angles to Jacobian
%    skew                       - vector to skew symmetric matrix
%    tr2delta                   - HT to differential motion vector
%    tr2jac                     - HT to Jacobian
%    vex                        - skew symmetric matrix to vector
%    wtrans                     - transform wrench between frames
%
% Trajectory generation
%    ctraj                      - Cartesian trajectory
%    jtraj                      - joint space trajectory
%    lspb                       - 1D trapezoidal trajectory
%    mtraj                      - multi-axis trajectory for arbitrary function
%    mstraj                     - multi-axis multi-segment trajectory
%    tpoly                      - 1D polynomial trajectory
%    trinterp                   - interpolate HT s
%
% Quaternion
%    Quaternion                 - constructor
%    /                          - divide quaternion by quaternion or scalar
%    *                          - multiply quaternion by a quaternion or vector
%    inv                        - invert a quaternion
%    norm                       - norm of a quaternion
%    plot                       - display a quaternion as a 3D rotation
%    unit                       - unitize a quaternion
%    interp                     - interpolate a quaternion
%
% Serial-link manipulator
%    SerialLink                 - construct a serial-link robot object
%    Link                       - construct a robot link object
%    *                          - compound two robots
%    friction                   - return joint friction torques
%    nofriction                 - return a robot object with no friction
%    perturb                    - return a robot object with perturbed parameters
%    plot                       - plot/animate robot
%    teach                      - drive a graphical  robot
%
%     Models
%        DHFactor               - convert elementary transformations to DH form
%        mdl_Fanuc10L           - Fanuc 10L (DH, kine)
%        mdl_MotomanHP6         - Motoman HP6 (DH, kine)
%        mdl_puma560            - Puma 560 data (DH, kine, dyn)
%        mdl_puma560akb         - Puma 560 data (MDH, kine, dyn)
%        mdl_p8                 - Puma 560 robot on an XY-translational base (DH, kine)
%        mdl_robot              - construct a robot object
%        mdl_stanford           - Stanford arm data (DH, kine, dyn)
%        mdl_S4ABB2p8           - ABB S4 2.8 (DH, kine)
%        mdl_twolink            - simple 2-link example (DH, kine)
%
%     Kinematic
%        DHFactor               - transform sequence to DH description
%        jsingu                 - find singular joints from Jacobian
%        fkine                  - forward kinematics
%        ikine                  - inverse kinematics (numeric)
%        ikine6s                - inverse kinematics for 6-axis arm with sph.wrist
%        jacob0                 - Jacobian in base coordinate frame
%        jacobn                 - Jacobian in end-effector coordinate frame
%        SerialLinkmaniplty     - compute manipulability
%
%     Dynamics
%        accel                  - forward dynamics
%        cinertia               - Cartesian manipulator inertia matrix
%        coriolis               - centripetal/coriolis torque
%        fdyn                   - forward dynamics
%        ftrans                 - transform a force/moment
%        gravload               - gravity loading
%        inertia                - manipulator inertia matrix
%        itorque                - inertia torque
%        rne                    - inverse dynamics
%
% Mobile robot
%    Map                        - point feature map object
%    RandomPath                 - driver for Vehicle object
%    RangeBearingSensor         - "laser scanner" object
%    Vehicle                    - construct a mobile robot object
%    sl_bicycle                 - Simulink "bicycle model" of non-holonomic wheeled vehicle
%    Navigation                 - Navigation superclass
%    Sensor                     - robot sensor superclass
%
%     Localization
%        EKF                    - extended Kalman filter object
%        ParticleFilter         - Monte Carlo estimator
%
%     Path planning
%        Bug2                   - bug navigation class
%        distancexform          - distance transform
%        DXform                 - distance transform planner class
%        Dstar                  - D* planner class
%        PRM                    - probabilistic roadmap planner class
%        RRT                    - rapidly exploring random tree class
%
% Graphics
%    plot2                      - plot trajectory
%    plotp                      - plot points
%    plot_box                   - draw a box
%    plot_arrow                 - draw an arrow
%    plot_circle                - draw a circle
%    plot_ellipse               - draw an ellipse
%    plot_homline               - plot homogeneous line
%    plot_point                 - plot points
%    plot_poly                  - plot polygon
%    plot_sphere                - draw a sphere
%    qplot                      - plot joint angle trajectories
%    plot2                      - Plot trajectories
%    plotp                      - Plot trajectories
%
% Utility
%    about                      - summary of object size and type
%    angdiff                    - subtract 2 angles modulo 2pi
%    circle                     - compute/draw points on a circle
%    colnorm                    - columnwise norm of matrix
%    diff2                      - elementwise diff
%    gauss2d                    - Gaussian distribution in 2D
%    ishomog                    - true if argument is a 4x4 matrix
%    ismatrix                   - true if non scalar
%    isrot                      - true if argument is a 3x3 matrix
%    isvec                      - true if argument is a 3-vector
%    numcols                    - number of columns in matrix
%    numrows                    - number of rows in matrix
%    PGraph                     - general purpose graph class
%    Polygon                    - general purpose polygon class
%    randinit                   - initialize random number generator
%    ramp                       - create linear ramp
%    unit                       - unitize a vector
%    tb_optparse                - toolbox argument parser
%
% Demonstrations
%    rtdemo                     - Serial-link manipulator demonstration
%
% Examples
%    sl_quadcopter              - Simulink model of a flying quadcopter
%    mdl_quadcopter             - Define dynamic parameters of a quadcopter
%    sl_braitenberg             - Simulink model a Braitenberg vehicle
%    sl_jspace                  - Puma robot joint space motion
%    movepoint                  - non-holonomic vehicle moving to a point
%    moveline                   - non-holonomic vehicle moving to a line
%    movepose                   - non-holonomic vehicle moving to a pose
%    walking                    - example of 4-legged walking robot
%    eg_inertia                 - joint 1 inertia I(q1,q2)
%    eg_inertia22               - joint 2 inertia I(q3)
%    eg_grav                    - joint 2 gravity load g(q2,q3)
%
%  *  located in the examples folder
%
% Copyright (C) 2011 Peter Corke
