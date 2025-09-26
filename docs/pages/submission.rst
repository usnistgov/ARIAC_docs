.. _SUBMISSION:

==========
Submission
==========

.. TODO: ADD INTRODUCTION

----------
Repository
----------

For this year's competition, each team will be invited to edit a private GitHub repository under the ``usnistgov`` organization. In that repository, teams will need to place their code and the Dockerfile used to build the team's submission. 

----------
Dockerfile
----------

The Dockerfile is used to avoid any potential setup issues with dependencies and to have the commands needed to run a submission loaded correctly. In this Dockerfile, the FROM must be the official ARIAC image with the tag smoketest or final, depending on the submission type. Also, two environment variables must be set: 

* ``TEAM_CONFIG``: Path to the team config in the container.

* ``TEAM_COMMAND``: Command used to run the submission.

.. note::

  The command can either be a ROS 2 command (run/launch) or a shell command (./run.sh). If it is a shell command, be sure to change the permissions in the Dockerfile so it can be run easily.
 
Here is an example:

.. code-block:: dockerfile

  FROM nistariac/ariac2025:final

  ENV TEAM_COMMAND="ros2 launch example_team example_team.launch.py"
  ENV TEAM_CONFIG="/team_ws/src/example_team/config/example_team_config.yaml"

  # Create a new overlay workspace
  ENV TEAM_WS=/team_ws
  RUN mkdir -p $TEAM_WS/src
  WORKDIR $TEAM_WS

  # Copy team packages into the overlay workspace
  COPY ./ $TEAM_WS/src/

  # Update apt and install any OS dependencies needed for rosdep
  RUN apt-get update && \
      rosdep update && \
      rosdep install --from-paths src --ignore-src -r -y

  # RUN pip3 install -r $TEAM_WS/src/requirements.txt --break-system-packages

  # Build the workspace (symlink install is fine for overlays)
  RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && \
                    source /ariac_ws/install/setup.bash && \
                    colcon build --symlink-install"

  # Source automatically in container
  RUN echo "source /team_ws/install/setup.bash" >> /root/.bashrc

  WORKDIR $TEAM_WS
  CMD ["bash"]

In your Dockerfile, it is important that the solution is built with dependencies, and that all ROS 2 packages are sourced in the .bashrc. For the smoketest and the finals, new ARIAC tags will be generated for the `official ARIAC image <https://hub.docker.com/r/nistariac/ariac2025>`_. The smoketest tag will be public as soon as it's released. There will be a finals tag for testing, but the actual tag used for evaluation will be a local image with modified defects and trials. This image will be publicly released after evaluation is complete.
