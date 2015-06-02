%% SoftGrasp Concepts and Workflow
%

%% Basic Assumptions
%
% SoftGrasp relies on the analytical framework introduced in [12] and
% extensively explained in [14], which draws upon previous works on
% whole-body defective grasps [8], [15], [16] and assumes the _contact
% information is known, by either planning (from e.g., OpenRAVE/OpenGRASP)
% or sensing_.

%%
% A basic assumption is that all bodies are modelled as rigid. Thanks to
% this, it is possible to use the wide variety of mathematical tools
% developed to describe rigid body motions. Compliance, e.g. the softness
% of the contact fingertips and/or the grasped object, is modeled through
% virtual springs.

%%
% SoftGrasp deals with modeling, analysis and optimization of
% _quasi-static_ grasps and dynamic effects such as viscous damping, are
% disregarded in the model. Note that dynamics do not typically play a
% major role in the control of grasping, where motions are usually slow.

%%
% The investigation of the system capabilities, e.g. the contact forces on
% the object actually controllable by the hand, is separate to the
% restriction imposed by the friction limits, and it is assumed that a
% subsequent classical force optimization procedure can lead to a feasible
% solution. In other words, the grasp analysis is performed in the
% assumption that the system of forces grasping the object is _force
% closure_, ensuring that grasping forces can always be exerted on the
% object such that both balance equations and contact force constraints are
% not violated. SoftGrasp provides methods for checking force-closure and
% for controlling grasping forces so as to avoid the violation of contact
% constraints in general cooperating mechanisms.

%% Synergistic Underactuated Robotic Hands Grasping Objects
%
% <html><ul><a name="surhgo-1"></a><table cellspacing="0" class="note"
% summary="Note" cellpadding="5" border="1"><tr width="90%"
% style="background-color:#e7ebf7"><td><p><b>Note</b>&nbsp;&nbsp;An
% in-depth description of the mathematics behind SoftGrasp is out of the
% scope of this documentation. Readers are strongly encouraged to refer to
% [14] for a rigorous exposition of the math underlying the Underactuated
% Robotic Grasping.</p></td></tr></table></ul></html>
%
% <html> <IMG ALIGN="CENTERED" src="./system_complete.png" width="500"
% height="auto"><DIV><B>Fig. 1</B> Compliant grasp by an underactuated
% robotic hand</DIV><p></p> </html>
%
% With reference to Fig. 1, a hand is modeled as a collection of serial
% robots called _limbs_ (simple chains of links connected through revolute
% or prismatic joints) manipulating an _object_. An inertial frame $\{A\} =
% \{O_a; x_a, y_a, z_a\}$ is attached to the palm. A reference frame $\{B\}
% = \{O_b; x_b, y_b, z_b\}$, referred to as the body frame, is fixed on the
% grasped object. The object is in contact with all or some of the links of
% the limbs and, eventually, the palm. No distinction is made whether the
% contacting links are at the extremities of the limb or not.

%%
% On the i- _th_ of the _p_ contact points, a frame $\{C_i^h\} =
% \{O_{c_i^h}; x_{c_i^h}, y_{c_i^h}, z_{c_i^h}\}$ is attached to the finger
% link and a frame $\{C_i^o\} = \{O_{c_i^o}; x_{c_i^o}, y_{c_i^o},
% z_{c_i^o}\}$ is attached to the object. These frames play a key role in
% the mathematical description of the hand/object interaction, more
% specifically, in the computation of the Grasp matrix, the hand Jacobian
% matrix and the wrench bases, that themselves play a key role in the
% definition of the equilibrium and congruence equations of hand and
% object.

%%
% The _contact_ between the hand and the object can be interpreted as a
% motion constraint. Typically, depending on the nature of materials
% involved, not all the directions of motions are limited. For the case of
% contact point without friction, only the direction normal to the contact
% surface is forbidden; the _hard finger_, allows the presence of three
% components of force, on the contact, but no moment; the _soft finger_
% adds the possibility to transmit a moment around the normal vector to the
% contact surface. The hard–finger model is used when there is significant
% contact friction, but the contact patch is small, so that no appreciable
% friction moment exists. The soft–finger model is used in situations in
% which the surface friction and the contact patch are large enough to
% generate significant friction forces and a friction moment about the
% contact normal.
%
% <html> In general, there are more motion constraints than degrees of
% freedom and this makes the grasping system statically indeterminate (or
% <i>hyperstatic</i>). To address this problem, a contact force model is
% adopted which is linear in the interference between the hand and the
% object. A <i>virtual contact spring</i> is considered as interposed
% between the link and the object at each contact point. For the
% i-<i>th</i> contact, the characteristics of the virtual spring are
% described by a constant <i>stiffness matrix</i> <img
% src="tex2png-2-5-a.png" width="auto" height="17" alt="$K_{c_i} \in
% \mathbb{R}^{c_i \times c_i}$"> where <img
% src="softgrasp_gs_workflow_eq34929.png" alt="$c_i$"> is the number of
% (motion) directions constrained by the contact.</p></html>

%%
% For the i- _th_ joint an elastic model is adopted, that relates the real
% configuration with a reference value, $q_{r_i}$, described by a torsional
% spring with constant stiffness $k_{q_i}$
%
% $$\tau_i = k_{q_i}(q_{r_i} - q_i).$$
%
% <html> Underactuation of the Hand is modeled by imposing that the joint
% reference positions <img src="softgrasp_gs_workflow_eq39339.png"
% alt="$q_r$"> evolve on a manifold <img src="tex2png-2-7-a.png"
% width="auto" height="17" alt="$\mathbb{S} \subseteq \mathbb{R}^{\sharp
% q}$"> as a function of some postural synergy values <img
% src="tex2png-2-7-b.png" width="auto" height="17" alt="$\sigma \in
% \mathbb{R}^{\sharp \sigma}$">, where <img
% src="softgrasp_gs_workflow_eq19590.png" alt="$\sharp \sigma \leq \sharp
% q$">, with <img src="softgrasp_gs_workflow_eq54361.png"
% alt="$\sharp\sigma$"> being the number of synergy variables and <img
% src="softgrasp_gs_workflow_eq68392.png" alt="$\sharp q$"> the number of
% joint variables. Mathematically, this is expressed by <img
% src="softgrasp_gs_workflow_eq09804.png" alt="$q_r = f(\sigma)$">. By
% differentiation, it follows that <img
% src="softgrasp_gs_workflow_eq13689.png" alt="$\delta q_r =
% S(\sigma)\delta\sigma$">, where <img src="tex2png-2-7-c.png" width="auto"
% height="30" alt="$S(\sigma)=\frac{\partial f(\sigma)}{\partial\sigma} \in
% \mathbb{R}^{\sharp q \times \sharp\sigma}$">. The generalized forces at
% synergy level are related to the joint torques by <img
% src="softgrasp_gs_workflow_eq60034.png" alt="$\eta = S^T(\sigma)\tau$">.
% Similarly to the joint actuation model, an elastic model is adopted also
% for the synergistic underactuation, where synergy reference <img
% src="softgrasp_gs_workflow_eq43541.png" alt="$\sigma_{r_i}$"> is related
% to the postural synergy <img src="softgrasp_gs_workflow_eq22141.png"
% alt="$\sigma_i$"> through a constant synergy stiffness <img
% src="softgrasp_gs_workflow_eq16986.png" alt="$k_{\sigma_i}$">, i.e., <img
% src="softgrasp_gs_workflow_eq82100.png"
% alt="$\delta\eta=K_\sigma(\delta\sigma_r - \delta\sigma)$">.</p> </html>

%%
% The above equations provides a quasi-static description of the grasping
% problem. They all can be casted together obtaining a _linear_ and
% _homogeneous_ system of equations, called _Fundamental Grasp Equation_
% (FGE). The matrix form of the system is composed by a coefficient matrix,
% called _Fundamental Grasp Matrix_ (FGM), and a vector describing the
% perturbation of all the force/configuration variables, such as joint
% position, joint torques and contact forces, called _augmented
% configuration_.
%
% Some considerations about the dimensions of the system bring to define
% the _dependent_ and the _independent_ variables, that is, to find which
% variables of the system are sufficient (the independent ones) to compute
% all the others (the dependent ones). This relationship can be written
% again in matrix form, and it defines the _canonical form of the
% Fundamental Grasp Matrix_ (cFGM).
%
% From the cFGM it is possible to extract some matrix blocks with relevant
% physical meaning, e.g. the space of the controllable internal forces, or
% the grasp compliance matrix. Note that The cFGM can be computed also for
% user-defined (under-)actuations, e.g., variable stiffness actuation
% (VSA).

%% Closed–Chain Mechanisms With Unactuated Joints
%

%% SoftGrasp Modeling Concepts
%
% <html><ul><a name="sgcmtgsd-1"></a><table cellspacing="0" class="note"
% summary="Note" cellpadding="5" border="1"><tr width="90%"
% style="background-color:#e7ebf7"><td><p><b>Note</b>&nbsp;&nbsp;SoftGrasp
% classes follow the handle semantics. See <a
% href="matlab:doc('handle')">handle</a> and the references therein for an
% in-depth explaination of this
% semantics.</p></td></tr></table></ul></html>

%%
% The above concepts of limb, hand and object have a direct counterpart in
% SoftGrasp. They are represented by instances of the classes |Limb|,
% |Hand| and |ManipulatedObject|, respectively.

%%
% A |Limb| object contains the kinematic description of a limb. An
% extensible design approach known as Factory Method Pattern [17] allows to
% add support for arbitrary kinematic-description conventions, without
% requiring any modification to the |Limb| class. Support for
% screw-theory-, Denavit-Hartenberg- and
% <http://wiki.ros.org/urdf/XML/model URDF>-based (serial) robot
% descriptions is already provided by SoftGrasp.

%%
% A |Hand| is made up of a collection of |Limb| objects. An homogeneous
% transformation represents the inertial frame $\{A\}$ attached to the
% palm. Hence, a |Hand| object encapsulates the complete geometrical
% description of the hand model.

%%
% A |ManipulatedObject| instance contains an homogeneous transformation
% that represents the body frame $\{B\}$.

%%
% Note that, since SoftGrasp assumes that the contact information is known,
% the information about the actual shape of links and/or objects (that play
% a key role in software simulations such as collision detection and
% contact determination) is not required.

%%
% To represent the contact information, different concepts are defined. One
% that plays a basic role in the contact definition procedure is the _Point
% Of Interest_, or POI. A POI is represented by an instance of the class
% |POI|, which embeds information about position and orientation of a
% normalized Gauss frame attached to the point itself and expressed in
% local coordinates, i.e., with respect to the fingerpad or the manipulated
% object, depending on where the point is placed. When placed onto a link,
% the POI can be used to model the _location of a contact_ with an object
% in a grasp and the normalized Gauss frame represents $\{C_i^h\}$. When
% placed onto the object, the POI can be used to locate a point of contact
% with a hand in a grasp and the normalized Gauss frame represents
% $\{C_i^o\}$.

%%
% |Glove| and |Cover| classes encapsulate the information that specifies
% how a collection of |POI| instances is associated to a given |Hand| or
% |ManipulatedObject| instance, respectively. A |Glove| instance matches
% the concept of a _coating_ to be applied to the hand and decorated with a
% number of POIs drawn at specific locations on the finger phalanges (Fig.
% 2). A |Cover| instance matches the concept of a coating to be applied to
% the object and decorated with a number of POIs drawn at desired locations
% (Fig. 3).
%
% The (conceptual) gloved hand in Fig. 2 is represented by a
% |HandWithGlove| instance. In other words, the aggregation of a |Hand|
% instance with a |Glove| instance is specified by an object of type
% |HandWithGlove|. Similarly, the aggregation of a |ManipulatedObject|
% instance with a |Cover| instance is specified by an object of type
% |ObjectWithCover|, which represents the (conceptual) covered object in
% Fig. 3.
%
% <html> <IMG ALIGN="CENTERED" src="./HandAndGlove.png" width="800"
% height="auto"><DIV><B>Fig. 2</B> Robotic hand (left). The robotic hand
% wearing a yellow coating decorated with a number of POIs (right). In
% evidence, the POI drawn on the 2nd link of 3rd limb (assuming the 3rd
% limb is the rightmost one). The decorated yellow coating is the
% conceptual representation of a <tt>Glove</tt> instance. The gloved hand
% matches the concept of a <tt>HandWithGlove</tt> instance.</DIV><p></p>
% </html>
%
% <html> <IMG ALIGN="CENTERED" src="./ObjectAndCover.png" width="800"
% height="auto"><DIV><B>Fig. 3</B> Manipulated object (left). The
% manipulated object covered by a yellow coating decorated with a number of
% POIs (right). In evidence, a POI drawn at a desired location on the body.
% The decorated yellow coating is the conceptual representation of a
% <tt>Cover</tt> instance. The covered object matches the concept of a
% <tt>ObjectWithCover</tt> instance.</DIV><p></p> </html>

%%
% The information on the contact locations expressed by |POI|, |Glove| and
% |Cover| instances must then be complemented by contact properties such
% as, e.g., constrained motion directions, contact stiffness and frictional
% attributes. These properties are encapsulated in concrete specializations
% of the |Contact| class. The classes |HardFingerContact|,
% |SoftFingerContact| and |CompleteConstraintContact| are already included
% in SoftGrasp to enable modeling of hard/soft finger and
% complete-constraint concepts, respectively.

%%
% To complete the specification of the contact information, |POI| objects
% of both the |Glove| and |Cover| instances must be suitably paired and
% each pair must be associated to a kind of |Contact|. This is done by
% using one instance of the |ContactStructure| class. This class
% encapsulates the information of which |POI| object on the |Glove|
% instance is paired to which |POI| object on the |Cover| instance, and
% which contact properties are assigned to each pair of points (Fig. 4).
%
% <html> <IMG ALIGN="CENTERED" src="./contactStructure.png" width="800"
% height="auto"><DIV><B>Fig. 4</B> Instances of <tt>HandWithGlove</tt> and
% <tt>ObjectWithCover</tt> (left). Conceptual representation of pairing
% POIs, with a specification of an hard finger contact model (exploded view
% on the right).</DIV><p></p> </html>

%%
% |Glove|, |Cover| and |ContactStructure| classes decouple the hand and
% object model from the complete specification of contact information,
% which is specific to a grasp task. In this way it is possible to use the
% same |Hand| and |ManipulatedObject| instances to model any number of
% grasping configurations (Fig. 5).
%
% <html> <IMG ALIGN="CENTERED" src="./PISAIITSoftHandGraspingCellPhone.png"
% width="600" height="auto"><DIV><B>Fig. 5</B> The same instances of
% <tt>Hand</tt> and <tt>ManipulatedObject</tt>, representing the PISA-IIT
% SoftHand [18] and a cell phone (top), can be used to model different
% grasping configurations such as the four-finger precision grasping and
% the whole-hand grasping (below left and right, respectively). Only the
% <tt>Glove</tt>, <tt>Cover</tt> and <tt>ContactStructure</tt> instances
% need to be modified.</DIV><p></p> </html>

%% SoftGrasp Tools for Grasp Analysis and Optimization
%
% The concepts and the modeling procedures above enable the specification
% of the models and data needed to perform analysis and control of
% grasping. Analysis and optimization functionalities are not provided by a
% single-class implementation. Rather, they are organized in composable
% solvers, in order to avoid too deep class hierarchies and hence enhance
% extensibility, maintainability and foster code reuse.

%%
% Fig. 6 shows a (simplified) UML class diagram view that summarizes the
% available solvers in SoftGrasp and the relationships among them.
%
% <html> <IMG ALIGN="CENTERED" src="./graspSolvers.png" width="600"
% height="auto"><DIV><B>Fig. 6</B> Available solvers in SoftGrasp to
% perform classification, analysis and optimization of cooperating arms,
% closed-chain mechanisms and underactuated hands (UML class diagram of
% <tt>GraspSolver</tt>).</DIV><p></p> </html>

%%
% The |GraspSolver| class provides essential functionalities common to all
% the analysis solvers, such as the computation of the Grasp matrix, the
% hand Jacobian matrix and the wrench bases. |GraspSolver| is an astract
% class and hence cannot be instantiated directly; three concrete
% specializations are provided by SoftGrasp.

%%
% |ClosedChainsSolver| is a kind of |GraspSolver| that computes the above
% manipulability subspaces, both in the kinematic and in force domain, of
% rigid-body closed-chain mechanisms with passive joints.

%%
% |GraspAnalysisSolver| is a kind of |GraspSolver| that TODO

%%
% |GraspClassificator| is a kind of |GraspSolver| that provides a basic
% classification of grasping systems, as in [19]. The classification
% provides insight into the physical meaning of the null spaces of the
% Grasp matrix and the hand Jacobian matrix. It can be used as a basis to
% implement further grasp classification criteria for advanced planning
% and/or reasoning.

%%
% The |GraspForceOptimizer| provides the functionality to compute an
% optimal distribution of the grasping forces according to the
% unconstrained minimization approach with barrier strategy described
% above. Instances of this solver are initialized with two results from the
% |GraspAnalysisSolver|, i.e., the matrices representing the subspace of
% _controllable internal forces_ and the _contact force transmission_ of
% the external wrench. The |ContactStructure| instance stored in the
% |GraspAnalysisSolver| object is also required as input parameter by the
% |GraspForceOptimizer| constructor. In this way, all the contact
% information is accessible to the internal optimization routines of the
% |GraspForceOptimizer|.


%% Typical Workflow Using SoftGrasp
%