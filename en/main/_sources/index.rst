=======
Welcome
=======



The Agile Robotics for Industrial Automation Competition (ARIAC), put on by the National Institute of Standards and Technology (NIST), is an annual robotics competition that tests the ability of robots to perform complex manufacturing tasks in a dynamic environment. ARIAC 2025 focuses specifically on agility in manufacturing through tasks involving perception, task planning, motion planning, and reasoning with emphasis on adaptability, autonomy, and efficiency.

The competition serves as a testbed for developing algorithms that can be applied in real-world manufacturing settings, with results guiding standards efforts in robot agility. NIST uses the insights gained from the contest to shape standard metrics and test methods for future robotic agility in manufacturing.

.. figure:: /_static/images/environment.png
   :width: 100%
   :alt: View of the 2025 environment

   ARIAC environment

Getting Started
===============

* **Competition Information**: The official `ARIAC competition page <https://www.nist.gov/el/intelligent-systems-division-73500/agile-robotics-industrial-automation-competition>`_ provides competition history, previous results, and cash prize eligibility requirements. Teams from universities and research institutions worldwide participate in developing agile robotic systems for industrial automation.

* **Team Registration**: `Register for ARIAC 2025 <https://bit.ly/ariac2025>`_ to receive competition updates and access team resources. Registration is open to both cash prize eligible and non-eligible teams.

* **ARIAC GitHub Repo**: The `full source code <https://www.github.com/usnistgov/ARIAC/>`_ is available on GitHub, including simulation environments and ROS interfaces.

* **Installation**: See the :ref:`startup page <STARTUP>` for detailed installation instructions using Docker or local setup.

* **Example Implementation and Demos**: The `example team repository <https://github.com/usnistgov/ariac_example_team>`_ provides reference implementations and configuration examples. Visit the :ref:`demos page <DEMOS>` for step-by-step instructions on running these examples and testing your setup.

* **Technical Support**: For technical issues and questions, visit the `ARIAC GitHub issues page <https://github.com/usnistgov/ARIAC/issues>`_.

.. note::

   Official competition rules will be available on challenge.gov at a later date.


.. toctree::
   :caption: Competition
   :maxdepth: 3
   :hidden:

   pages/scenario
   pages/sensors
   pages/challenges
   pages/evaluation

.. toctree::
   :caption: Development
   :maxdepth: 3
   :hidden:

   pages/schedule
   pages/startup
   pages/config_files
   pages/demos
   pages/api

.. toctree::
   :caption: ROS Interfaces
   :maxdepth: 3
   :hidden:

   interfaces/message_definitions
   interfaces/service_definitions
   interfaces/action_definitions