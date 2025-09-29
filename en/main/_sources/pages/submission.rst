.. _SUBMISSION:

==========
Submission
==========

This page provides a step-by-step guide for submitting your code for the competition. See the :ref:`schedule <SCHEDULE>` for submission dates.

**Smoke Test (Optional)**

The smoke test is an optional preliminary evaluation to:

- Validate your submission process
- Ensure your code runs on the evaluation infrastructure
- Help identify issues before the final submission

.. note::

   Teams are **not expected** to have fully operational submissions at this stage. The goal is to verify your code runs without crashing.

**Finals (Required)**

The finals submission is the scored evaluation that determines your final team ranking.

----------------
Submission Steps
----------------

1. **Get Repository Access**

   a. Submit team information: Complete the `Google form <https://forms.gle/aiaySnEiS8vna2op8>`_ before the smoke test deadline. You'll receive an email invitation within 24-48 hours.

   b. Accept the GitHub invitation: Check your email and accept the invitation to your team's private repository.

   .. important::
  
     **Set up SSH key** (if not already done): Follow GitHub's guide to `generate an SSH key <https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent>`_ and `add it to your GitHub account <https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account>`_.

2. **Upload Your Code**

   Choose the option that matches your current setup:

   Option 1: You already use Git

   Add the submission repository as a remote and push your existing code:

   .. code-block:: bash

      # Navigate to your existing repository
      cd your-existing-repo

      # Add the new repository as a remote
      git remote add submission git@github.com:usnistgov/ariac2025_{team_name}.git

      # Push your existing code
      git push submission main

   Option 2: You don't use Git

   Clone the empty repository and copy your files:

   .. code-block:: bash

      # Clone the empty repository
      git clone git@github.com:usnistgov/ariac2025_{team_name}.git
      cd ariac2025_{team_name}

      # Copy your existing files into this directory
      # Replace {path_to_existing_files} with your actual file path
      cp -r {path_to_existing_files}/* .

      # Add and commit your files
      git add .
      git commit -m "Initial commit with team code"
      git push origin main


3. **Create a Dockerfile**

   Create a Dockerfile in your repository's root directory. This file tells the evaluation system how to build and run your code. Your Dockerfile must include the base image, environment variables, and any setup needed for your solution.

   a. Use the correct base image:

      .. code-block:: docker

         # For smoke test
         FROM nistariac/ariac2025:smoke_test

         # For finals
         FROM nistariac/ariac2025:final

   b. Set required environment variables:

      .. code-block:: docker

         ENV TEAM_CONFIG=/team_ws/config/team_config.yaml
         ENV TEAM_COMMAND="ros2 launch your_package your_launch.py"

      .. important::
         ``TEAM_COMMAND`` can be a ROS 2 command or shell script. For shell scripts, set proper permissions with ``RUN chmod +x /path/to/script.sh``.

   In addition the Dockerfile should:
   
   - Install all dependencies for your solution
   - Build your ROS workspace
   - Source the workspace in the container's .bashrc  

   
   **Example Dockerfile**:

   .. literalinclude:: /_static/files/dockerfile_example
     :language: docker

4. **Test Your Submission**

   a. Add a docker compose configuration to your repository:

      Example:

      .. literalinclude:: /_static/files/docker-compose_example.yaml
         :language: yaml

   b. Start the container using docker compose:

      If you have an NVIDIA graphics card:

      .. code-block:: bash

         docker compose up ariac_nvidia

      Otherwise:

      .. code-block:: bash

         docker compose up ariac

   c. Launch the environment (in a new terminal):

      Open a new terminal in the container and launch the environment with your team config:

      .. code-block:: bash

         docker exec -it your_container_name bash
         ros2 launch ariac_gz ariac.launch.py user_config:=$TEAM_CONFIG trial_config:=/path/to/trial/config.yaml

   d. Run your team command (in another new terminal):

      Open another terminal in the container and execute your team's solution:

      .. code-block:: bash

         docker exec -it your_container_name bash
         $TEAM_COMMAND

5. **Create Tagged Release**

   NIST will evaluate your code using specific tagged versions. Create the appropriate tag for each submission:

   a. Create and push a git tag:

      .. code-block:: bash

         # For smoke test
         git tag smoketest
         git push submission smoketest

         # For finals
         git tag final
         git push submission final

   .. important::

      Ensure all your code and Dockerfile are committed and pushed before creating a tag. Tags must be created before the submission deadline. 
