# softgrasp - Matlab toolbox for modeling, analysis and optimization of grasps with synergistic underactuated robotic hands

SoftGrasp is a cross-platform free-software toolbox for use with Matlab. It enables the modeling, analysis and optimization of complex manipulation systems, ranging from cooperating multi-arm systems to closed-chain mechanisms and underactuated robotic hands grasping objects. For more information about its unique features and a (preliminary, incomplete) description of the expected workflow using SoftGrasp please refer to:  
http://rawgit.com/m-morelli/softgrasp/master/html/softgrasp_gs_overview.html  
http://rawgit.com/m-morelli/softgrasp/master/html/workflow/softgrasp_gs_workflow.html

*SoftGrasp is still in the early phases of development and its API is under revision. Most of the documentation is absent or incomplete (in progress). The currently available documentation is integrated with the Matlab Help System. There are no read-to-use demos to get started with SoftGrasp; demos using an older version of the toolbox API are available [here] (http://www.thehandembodied.eu/pdf/tools/RTSS.zip), but some work (in progress) is required to adapt them to the current API.*

SoftGrasp makes heavy use of the Matlab's OOP framework and the `matlab.mixin.Heterogeneous` abstract class is used to define hierarchies of classes whose instances can be combined into heterogeneous arrays. A number of Matlab functions are also used that replace deprecated functionalities (e.g., `nargchk` and `nargoutchk`). As result, *SoftGrasp requires Matlab R2011b or above.*

# Install

To install SoftGrasp simply adjust the `MATLABPATH` to include this directory _and_ all its subdirectories.

Assuming `path/to/soft/grasp` is the path to the directory that contains this `README.md` file, simply give the following command at the Matlab prompt:
```
>> addpath(genpath('path/to/soft/grasp'))
```

For more information about the Matlab Search Path and on how to use the modified Search Path in future sessions, please refer to the following web links:  
http://www.mathworks.com/help/matlab/matlab_env/what-is-the-matlab-search-path.html  
http://www.mathworks.com/help/matlab/ref/savepath.html

SoftGrasp relies on the [Matlab's Robotics Toolbox] (http://www.petercorke.com/Robotics_Toolbox.html) for some minor functionalities and optional features, such as the modeling of the topology of the robotic fingers according to the Standard/Modified Denavit-Hartenberg convention. For user convenience, a (patched) version of the RTB package is already included in the SoftGrasp distribution. Note that any prior installation of the RTB package should be removed from the Search Path to avoid name conflicts. For further details on this topic, please refer to [this link] (http://www.mathworks.com/help/matlab/matlab_env/files-and-folders-that-matlab-accesses.html#br8hqz8-1).

# Note on licensing

SoftGrasp is free-software, released under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version. Third-party modules [RTB] (http://www.petercorke.com/Robotics_Toolbox.html) and the [Kinematics Toolbox] (http://fr.mathworks.com/matlabcentral/fileexchange/24589-kinematics-toolbox) are included for users convenience, come with their own licenses and are copyright to their respective owners.
