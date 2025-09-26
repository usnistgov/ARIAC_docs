.. _SUBMISSION:

==========
Submission
==========

For ARIAC 2025, there are two types of submissions: smoketests and finals. Both will use the same methods of using a Dockerfile and a private GitHub Repository to submit teams' code. 

----------------
Submission Types
----------------

Smoketests
==========

The smoketests are a way for teams to submit their code early for the organizers to test. We will provide results and help to get teams' systems working on our side. This will prevent potential issues for finals submissions. We will also provide a new tag on the Docker image so testing can be done by the teams as well.

Finals
======

The finals are the part of ARIAC which is actually scored so teams can be compared against each other. A finals tag will be provided to teams for testing, but it will not include the actual defects and trials that will be used in the finals. A local private tag will be used for the actual finals. This local tag will be published after evalutation of the submissions have completed.

----------
Repository
----------

For this year's competition, each team will be invited to edit a private GitHub repository under the ``usnistgov`` organization. In that repository, teams will need to place their code and the Dockerfile used to build the team's submission. The organizers will send out a message prior to the smoketests requesting the team name and the emails of your team members which should be added as collaborators to the GitHub repository. 

----------
Dockerfile
----------

The Dockerfile is used to avoid any potential setup issues with dependencies and to have the commands needed to run a submission loaded correctly. In this Dockerfile, the FROM must be the official ARIAC image with the tag smoketest or final, depending on the submission type. Also, two environment variables must be set: 

* ``TEAM_CONFIG``: Path to the team config in the container.

* ``TEAM_COMMAND``: Command used to run the submission.

.. note::

  The command can either be a ROS 2 command (run/launch) or a shell command (./run.sh). If it is a shell command, be sure to change the permissions in the Dockerfile so it can be run easily.
 
Here is an example:

.. literalinclude:: /_static/files/dockerfile_example
  :language: docker

.. important::
  The ``RUN echo "source /team_ws/install/setup.bash" >> /root/.bashrc`` is required to run the competition properly.

In your Dockerfile, it is important that the solution is built with dependencies, and that all ROS 2 packages are sourced in the .bashrc. For the smoketest and the finals, new ARIAC tags will be generated for the `official ARIAC image <https://hub.docker.com/r/nistariac/ariac2025>`_. The smoketest tag will be public as soon as it's released. There will be a finals tag for testing, but the actual tag used for evaluation will be a local image with modified defects and trials. This image will be publicly released after evaluation is complete.
