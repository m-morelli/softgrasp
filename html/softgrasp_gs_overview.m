%% Product Overview
%

%% What is SoftGrasp?
%
% The SoftGrasp toolbox is a new open-source software library for use with
% MATLAB to model robotic grasping systems such as cooperating arms,
% closed-chain mechanisms and underactuated hands.

%%
% SoftGrasp includes models of generalized compliance at contacts and in
% the actuation mechanism, and enables kineto-static analysis of grasp that
% encompasses elastic motions, statically indeterminate configurations, and
% pre-loaded initial conditions. It is based on the _softly underactuated_
% model of hands described in [12], where the hand configuration driven by
% _synergies_ [1] modifies its posture according to the object shape and
% compliance.

%%
% SoftGrasp provides a clean object-oriented API that enforces the
% consistency of data structures, helps to manage software complexity
% easily and fosters code-reuse. The toolbox also offers a simple interface
% to OpenRAVE [10], so that contact points and normals can be computed for
% real-world grasp tasks by using the powerful planning tools this platform
% provides.

%% Related Software
%
% Differently from what happens in other field of robotics, e.g. industrial
% and mobile robotics, the variety of analysis and simulation tools
% available for robotic grasping is rather limited.
%
% GraspIt! [7] is an open-source virtual environment for planning and
% simulation of robotic grasping tasks. It features a contact determination
% system and a grasp planner, provides techniques for grasp analysis and
% includes eigengrasps information for many dexterous robotic hand models,
% according to the rigid synergy model described in [6]. However, GraspIt!
% is based on an analytical approach that abstracts the analysis from the
% specific physical characteristics of the grasping hand: it cannot
% distinguish contact forces that may be actively controlled by joint
% variables from those that cannot, as described in [8], and this may lead
% to misleading results. In addition, the MATLAB interface that allows
% script programming is no longer tested and maintained and its less
% modular architecture makes it difficult to improve and integrate with
% other tools and frameworks.
%
% OpenGRASP [9] is an open-source simulation toolkit for grasp planning and
% dexterous manipulation analysis. It takes advantage of strength points of
% OpenRAVE, such as the modular plugin-based architecture and scripting
% environments for Python, and adds specific features towards the
% realization of an advanced grasping simulator. OpenGRASP provides some
% methods for computing force closure metrics, such as the Volume Metric
% and the Grasp Wrench Space formulation, but does not implement any
% algorithm for computing the joint actuations that result in an optimal
% grasp force distribution to a specific external disturbance. It supports
% joint coupling modeling through mimic joints, but only underactuations
% that command the joint displacements directly can be described.
%
% SynGrasp [11] is a MATLAB toolbox recently developed within the Projects
% [2], [3] for grasp analysis of human and robotic hands. It enables
% modeling of joint coupling and underactuations according to the _soft_
% synergy model described in [4], [5], as opposed to the rigid synergy
% model of GraspIt! and OpenGRASP. However, it does not implement the
% analysis framework described in [12], where the grasp problem is studied
% as a whole and relevant properties, such as _pure_ and _spurious squeeze_
% and _kinematic grasp displacements_, are computed from a numerical
% decomposition of its solution space. In addition, the toolbox is not
% meant to be used also outside the THE context or in conjunction with
% other simulators, and its minimalistic application programming interface
% (API) does not enforce data encapsulation (being designed around MATLAB's
% structure arrays instead of classes).

%% SoftGrasp's Key Features
%
% GraspIt! and OpenGRASP are powerful simulators that perform great in
% grasp planning and contact detection. SoftGrasp focuses on, and is more
% suitable with, the study of effects of postural synergies on the quality
% of the grasp and the optimization of the distribution of grasp forces,
% once a grasp is established.
%
% SynGrasp and SoftGrasp are similar in scope. However, SoftGrasp offers
% the following unique features:
%
% * the possibility of using a screw-theory-based description of the grasp
% problem, according to [13];
% * a modular composition of the _Fundamental Grasp Matrix_ (FGM) [12],
% [14], with the possibility of introducing _compliant joints_, _variable
% stiffness actuators_ (VSA), _hard_ and _soft synergies_, and/or other
% user defined features;
% * a collection of algorithms performing a numerical decomposition of the
% solution space and returning relevant properties of grasp, according to
% [12], [14];
% * a clean _object-oriented API_ that enforces the consistency of data
% structures and fosters code-reuse;
% * a (simple) _interface to OpenRAVE/OpenGRASP_.

%% Acknowledgments
%
% In Progress.
%
% Authors, EU FP7 Projects (THE, SoftHands, ...)