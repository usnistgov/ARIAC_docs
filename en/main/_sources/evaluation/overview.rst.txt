.. _AUTOMATED_EVALUATION_OVERVIEW:

=============================
Automated Evaluation Overview
=============================

An automated system is used to score each team's CCS. A docker container is created for each team from the ARIAC image. The competitors ROS package(s) are built along with any dependencies on the container. After the competitors package is built, scripts are used to run individual or multiple trials. The results for these trials along with ROS logs and gazebo state logs are copied from the container to the host machine. 

To properly build and install the competitor's code onto the container each team must submit a configuration file. The yaml configuration file should be submitted to the Google Drive folder for your team. :numref:`evaluation-config` shows an example of a team evaluation configuration file.

.. code-block:: yaml
  :caption: Example of a team evaluation configuration file
  :name: evaluation-config

  team_name: "nist_competitor"

  github:
    repository: "github.com/usnistgov/nist_competitor.git"
    tag: "2024.2.2"
    personal_access_token: ""

  build:
    pre_build_scripts: ["nist_competitor_pre_build.sh"]

  competition:
    package_name: "nist_competitor"
    launch_file: "competitor.launch.py"

  
.. note::

  The configuration file should be named :file:`team_name.yaml`.

------------------------------------------------
Instructions for creating the configuration file
------------------------------------------------

1. Competitors must upload their ROS package to a private github repository. Please ensure that only your ROS package is including in this repository. If you include the entire ROS workspace it will not work properly. If you have multiple ROS packages they can all be included in a single folder. The ARIAC repository is an example of this setup. 

  * For the :yamlname:`repository` link ensure that it is structured exactly like the example above with the :yaml:`https://` excluded. 

2. To ensure that the repository is not updated after submission, competitors will be required to create a release of their software (`instructions <https://docs.github.com/en/repositories/releasing-projects-on-github/managing-releases-in-a-repository>`_). Competitors should use the :yamlname:`tag` field with the tag for the desired release. This field will be required for qualifiers and finals but if you are testing your system and want to quickly make changes you can comment out the :yamlname:`tag` field and the main branch will be used. 

3. For the docker container to clone the environment teams will need to create a `personal_access_token for the repository <https://docs.github.com/en/authentication/keeping-your-account-and-data-secure/creating-a-personal-access-token>`_. We suggest creating a fine-grain token and only giving read access permissions for the competition repository. 

.. note::

  Do not include your token anywhere inside of your repository. Doing so will deactivate the token when you push it to github. 
  
4. The build scripts for the docker container will run rosdep automatically to ensure that any ROS packages you have included in your package manifest (:file:`package.xml`) will be installed.

5. If your team needs to install dependencies that are not included in rosdep you should create a custom build script, add the script to the competitor_build_scripts directory and include the file name in the :yamlname:`pre_build_scripts` section of the configuration file. These scripts will also need to be added to the Google Drive folder. For example of this, see the :file:`nist_competitor_pre_build.sh` script in the :file:`competitor_build_scripts` folder. 

6. Inside the competition section, competitors should specify the :yamlname:`package_name` and :yamlname:`launch_file`. For the automated evaluation to work properly competitors must create a launch file that starts the environment and any nodes that are necessary for the CCS. The instructions for creating this launch file can be found in :ref:`LAUNCH_FILE_SETUP`.
  
.. note::
    
  If your team has multiple ROS packages ensure that the :yamlname:`package_name` is set for the package that includes the :yamlname:`launch_file`

----------------------------------------------
Instructions for Testing the Evaluation System
----------------------------------------------

Competitors can test the evaluation system on their setup with the following steps.

.. note::

  Currently the evaluation system only runs on Ubuntu

1. Install `Docker Engine <https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository>`_.
   
    .. warning::
  
      The automated evaluation does not work with docker-desktop. If you already have docker-desktop installed you must run the following command to switch from the desktop-linux context to the default context. Images and containers will not show up in the docker-desktop GUI.

      .. code-block:: sh

        docker context use default


2. If your machine has an Nvidia GPU and you want to enable GPU acceleration, install the `nvidia container toolkit <https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker>`_.

.. note::

  The final evaluation will be run on a machine with an Nvidia 3070 

3. Pull the ARIAC docker image from docker hub with the command:

  .. code-block:: sh

    docker image pull nistariac/ariac2024:latest

4. Clone the ARIAC_evaluation repository:

  .. code-block:: sh

    git clone https://github.com/usnistgov/ARIAC_evaluation.git

5. Add your team's evaluation configuration file to the folder:
  
    :file:`/automated_evaluation/competitors_configs`

6. If necessary, add any build scripts to the folder:
    
    :file:`/automated_evaluation/competitors_configs/competitor_build_scripts`

7. Add any trials you want to test with to the folder: 

    :file:`/automated_evaluation/trials`

8. Navigate to the automated evaluation folder in a terminal.

  .. code-block:: sh

    cd ARIAC_evaluation/automated_evaluation

9. Run the following command to allow scripts to run as executables.

  .. code-block:: sh

    chmod +x build_container.sh run_trial.sh

10. Check that docker daemon is running by either opening docker desktop or running:

  .. code-block:: sh

    sudo systemctl start docker

11. Run the build container script with your team name as an argument. Passing a second argument of 'nvidia' will allow the container to be setup for GPU acceleration. For example to run the nist_competitor with GPU acceleration:

  .. code-block:: sh

    ./build_container.sh nist_competitor nvidia

  If you do not have nvidia graphics cards or do not want to use gpu acceleration you can run the script without the nvidia argument:

  .. code-block:: sh

    ./build_container.sh nist_competitor

  .. warning::

    If you make changes to your source code and want to update the container you first need to remove the existing container:

    .. code-block:: sh

      docker container rm {container_name} --force

12. To run a trial use the :file:`run_trial.sh` script. The first argument is the team name which should also be the name of the container. The second argument is the name of the trial to be run. For example to run the nist_competitor with trial :file:`kitting.yaml` use the command:

  .. code-block:: sh

    ./run_trial.sh nist_competitor kitting

  To run a specific trial multiple times pass a third argument for the number of iterations for that trial. For example to run the kitting trial three times:
  
  .. code-block:: sh

    ./run_trial.sh nist_competitor kitting 3

  .. note::

    You can only run trials that were added to the folder :file:`/automated_evaluation/trials` before running the build container script.

  To run all the trials added to the :file:`trials` folder replace the second argument with run-all, for example:

  .. code-block:: sh

    ./run_trial.sh nist_competitor run-all

  By default the run-all will run each trial once. To run each trial multiple times pass a third argument for the number of iterations for each trial. For example to run each trial three times:

  .. code-block:: sh

    ./run_trial.sh nist_competitor run-all 3

13. View the results of the trial in the folder :file:`/automated_evaluation/logs`. The output will include the sensor cost calculation, the scoring log, ROS logs, and a gazebo state log.

---------------------------------
Playing back the Gazebo State Log
---------------------------------

To view a interactive replay of the trial after completition, competitors can use the :file:`playback_trial.sh` script. The script takes two arguments, :code:`team_name` and :code:`trial_run`. The :code:`trial_run` argument should match the name of the log folder created for that trial. For example the second run of a trial named :file:`kitting.yaml` would be :code:`kitting_2`. To playback this trial use the command:

.. code-block:: sh

    ./playback_trial.sh nist_competitor kitting_2

