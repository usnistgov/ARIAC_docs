.. _EVALUATION:

==========
Evaluation
==========

This page describes the scoring system for the competition. Teams are evaluated on their performance across multiple runs and trials, with scores calculated based on task completion, efficiency bonuses, and penalty deductions. The evaluation process covers individual run scores, which are aggregated into an execution score (80%) and combined with human judging (20%) to determine final team rankings.

---------
Run Score
---------

.. container::

    1. **Base Score**: How well was the assigned task completed?
    2. **Bonus Score**: Various factors; how efficient and accurate is the system?
    3. **Penalties**: Various sources; how often did the team perform an illegal action?

.. container:: formula-highlight

  .. math::

    R = B + \sum \beta_i - \sum \rho_i * o_i

.. list-table:: Variables
   :widths: 25 75
   :class: centered-table
   :width: 80%

   * - :math:`R`
     - Total run score
   * - :math:`B`
     - Base score
   * - :math:`\beta_i`
     - Bonus scores
   * - :math:`\rho_i`
     - Deduction for each penalty type
   * - :math:`o_i`
     - Number of occurrences for the associated penalty


Scoring Weights
===============

The scoring system uses predefined weights to balance the importance of different performance aspects. These weights determine how much each component contributes to the overall score.

.. list-table:: Scoring Weights
   :header-rows: 1
   :widths: 25 50 25
   :class: centered-table
   :width: 80%

   * - Weight
     - Description
     - Value
   * - :math:`\omega_1`
     - Kit completion weight
     - 500
   * - :math:`\omega_2`
     - Module completion weight
     - 800
   * - :math:`\omega_3`
     - Trial time bonus weight
     - 175
   * - :math:`\omega_4`
     - Inspection speed bonus weight
     - 125
   * - :math:`\omega_5`
     - High priority order speed bonus weight
     - 150
   * - :math:`\omega_6`
     - Sensor cost bonus weight
     - 500
   * - :math:`\omega_7`
     - Inspection classification bonus weight
     - 100

Base Score
==========

The base score is calculated from the submission of kitting orders and module orders.

.. container:: formula-highlight

  .. math::

    B = \omega_1 \cdot \frac{k_c}{k_d} + \omega_2 \cdot \frac{m_c}{m_d}

.. list-table:: Variables
   :widths: 25 75
   :class: centered-table
   :width: 80%

   * - :math:`B`
     - Base score for the run
   * - :math:`k_c`
     - Number of kits successfully submitted
   * - :math:`k_d`
     - Number of kits desired
   * - :math:`m_c`
     - Number of modules successfully submitted
   * - :math:`m_d`
     - Number of modules desired

.. important::

  High priority kits are included in :math:`k_d`

Kit Acceptance Criteria
^^^^^^^^^^^^^^^^^^^^^^^

* Four good cells (not defective and within voltage specification)
* Total kit voltage within tolerance
* Kit delivered to shipping area on an AGV

Module Acceptance Criteria
^^^^^^^^^^^^^^^^^^^^^^^^^^

* Four good cells with individual voltages in specification
* Total module voltage within tolerance
* Bottom and top shells properly installed and secured
* All six welds completed successfully:

  * Four top welds 
  * Two bottom welds

* Module delivered to submission area on assembly conveyor

Bonus Points
============

Teams can earn bonus points from several sources:

.. note::

  All bonus values are clamped to zero - bonuses cannot be negative.

.. important::

  Teams are only eligible for bonuses if all desired kits and modules were submitted

Trial Time
^^^^^^^^^^

The trial time bonus is awarded for completing the competition in less time than allocated.

.. container:: formula-highlight

  .. math::

    \beta_1 = \omega_3 \cdot (1 - \frac{t_e}{t_m})

.. list-table:: Variables
   :widths: 25 75
   :class: centered-table
   :width: 80%

   * - :math:`\beta_1`
     - Trial time bonus
   * - :math:`t_e`
     - Run execution duration
   * - :math:`t_m`
     - Run time limit

Inspection Speed
^^^^^^^^^^^^^^^^

The inspection speed bonus is awarded for submitting inspection reports faster than the target time.

.. container:: formula-highlight

  .. math::

    \beta_2 = \omega_4 \cdot (1 - \frac{\gamma}{\gamma_d})

.. list-table:: Variables
   :widths: 25 75
   :class: centered-table
   :width: 80%

   * - :math:`\beta_2`
     - Inspection speed bonus
   * - :math:`\gamma`
     - Average inspection duration
   * - :math:`\gamma_d`
     - Desired inspection duration (target: 8 seconds)

High Priority Order Speed
^^^^^^^^^^^^^^^^^^^^^^^^^^

The high priority order speed bonus is awarded for completing high priority orders faster than the time limit.

.. container:: formula-highlight

  .. math::

    \beta_3 = \omega_5 \cdot (1 - \frac{\tau}{\tau_d})

.. list-table:: Variables
   :widths: 25 75
   :class: centered-table
   :width: 80%

   * - :math:`\beta_3`
     - High priority order speed bonus
   * - :math:`\tau`
     - Average high priority kit execution duration
   * - :math:`\tau_d`
     - Desired high priority kit execution duration (target: 130 seconds)

Sensor Cost
^^^^^^^^^^^

The sensor cost bonus is awarded for using sensors below the allocated budget. Exceeding the budget results in a penalty instead.

.. container:: formula-highlight

  .. math::

    \beta_4 = \omega_6 \cdot (1 - \frac{\sigma}{\sigma_b})

.. list-table:: Variables
   :widths: 25 75
   :class: centered-table
   :width: 80%

   * - :math:`\beta_4`
     - Sensor cost bonus
   * - :math:`\sigma`
     - Team sensor cost
   * - :math:`\sigma_b`
     - Sensor budget ($7000)

Inspection Classification
^^^^^^^^^^^^^^^^^^^^^^^^^

The inspection classification bonus is awarded for correctly identifying defect types and locations. All elements of the defect report must be correct.

.. container:: formula-highlight

  .. math::

    \beta_5 = \omega_7 \cdot (1 - \frac{\nu}{\delta})

.. list-table:: Variables
   :widths: 25 75
   :class: centered-table
   :width: 80%

   * - :math:`\beta_5`
     - Inspection classification bonus
   * - :math:`\nu`
     - Number of correctly classified defect reports
   * - :math:`\delta`
     - Total number of defective cells


Penalties
=========

The competition applies penalties for certain events during runs. Penalties are calculated per occurrence - each penalty has an associated value that is deducted from the total score when the event occurs. Penalties come from the following sources:

* Non-defective cell placed in inspection bin
* Cell falls into conveyor bin
* Cell comes into contact with an invalid surface
* Two AGVs collide
* A robot collides, either with an object in the environment, or another robot
* The sensor cost used is over the provided budget

The following table shows the symbols for each penalty and their associated values.

.. list-table:: Penalty Values
   :header-rows: 1
   :widths: 25 50 25
   :class: centered-table

   * - Penalty Symbol
     - Description
     - Value
   * - :math:`\rho_{0}`
     - Non-defective cell placed in inspection bin
     - 20
   * - :math:`\rho_{1}`
     - Cell falls into conveyor bin
     - 20
   * - :math:`\rho_{2}`
     - Object on invalid surface
     - 20
   * - :math:`\rho_{3}`
     - AGV collision
     - 40
   * - :math:`\rho_{4}`
     - Robot collision
     - 50
   * - :math:`\rho_{5}`
     - Sensor cost over budget
     - 0.0715

.. warning::

  For the robot collision, for every 5 seconds the robot is in collision, another occurrence of the penalty will be recorded.
  In addition, if a robot is in collision with another robot, this penalty will be counted twice.

.. note::

  For the sensor cost penalty, the penalty is calculated for each dollar over the allocated budget. Using less than the allocated budget will result in a bonus being applied to the score.

---------------
Execution Score
---------------

The execution score aggregates individual run scores across all trials to determine the final performance ranking. Teams complete five runs per trial, with the two best scores from each trial being averaged together. These trial averages are then summed to create the total execution score.

.. container:: formula-highlight

  .. math::

    E = \sum_{i=1}^{n} \frac{R_1 + R_2}{2}

.. list-table:: Variables
   :widths: 25 75
   :class: centered-table
   :width: 80%

   * - :math:`E`
     - Total execution score
   * - :math:`n`
     - Number of trials
   * - :math:`R_1`
     - Best run score for trial i
   * - :math:`R_2`
     - Second best run score for trial i

----------------
Human Evaluation
----------------

In addition to the execution score, teams are evaluated by human judges who assess the overall approach and innovation demonstrated during the competition. Judges independently review videos of trial runs and evaluate teams across three categories.

Each team receives scores from 1 to 5 in the following categories:

* **Novelty/Innovation**: Creative and original approaches to solving competition challenges
* **Feasibility of Approach**: Practicality and robustness of the implemented solution
* **Alignment with Spirit of Competition**: How well the approach embodies the goals and values of the competition

The human evaluation score is the sum of all individual judge scores across the three categories:

.. container:: formula-highlight

  .. math::

    H = \sum_{j=1}^{n} (\eta_j + \phi_j + \alpha_j)

.. list-table:: Variables
   :widths: 25 75
   :class: centered-table
   :width: 90%

   * - :math:`H`
     - Human evaluation score
   * - :math:`n`
     - Number of judges
   * - :math:`\eta_j`
     - Judge j's score for Novelty/Innovation (1-5 scale)
   * - :math:`\phi_j`
     - Judge j's score for Feasibility of Approach (1-5 scale)
   * - :math:`\alpha_j`
     - Judge j's score for Alignment with Spirit of Competition (1-5 scale)

-----------------------
Final Competition Score
-----------------------

The final competition ranking is determined by converting both execution scores and human evaluation scores to standardized rankings, then combining them with 80% weight for execution performance and 20% weight for human evaluation.

**Ranking Process:**

1. **Execution Ranking**: Teams are ranked by execution score (highest to lowest)
2. **Human Evaluation Ranking**: Teams are ranked by human evaluation score (highest to lowest)
3. **Combined Ranking**: Rankings are weighted and combined to determine final standings

.. container:: formula-highlight

  .. math::

    F_{rank} = 0.8 \cdot R_E + 0.2 \cdot R_H

.. list-table:: Variables
   :widths: 25 75
   :class: centered-table
   :width: 80%

   * - :math:`F_{rank}`
     - Final weighted ranking score (lower is better)
   * - :math:`R_E`
     - Execution score ranking (1 = best execution score)
   * - :math:`R_H`
     - Human evaluation ranking (1 = best human score)