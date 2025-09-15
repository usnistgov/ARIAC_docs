Evaluation System
=================

The ARIAC 2025 evaluation system provides comprehensive assessment of team performance across multiple dimensions including task completion, efficiency, accuracy, and adaptability. The scoring system balances base performance with bonuses and penalties to reward well-rounded, agile manufacturing systems.

Run Score Calculation
---------------------

The overall run score combines three main components using the following formula:

**R = Β + Σβᵢ - Σρᵢ × oᵢ**

Where:

.. list-table:: Score Components
   :header-rows: 1
   :widths: 15 85

   * - Symbol
     - Description
   * - **R**
     - Total run score
   * - **Β**
     - Base score for task completion
   * - **βᵢ**
     - Score for bonus type i
   * - **ρᵢ**
     - Deduction amount for penalty type i  
   * - **oᵢ**
     - Number of occurrences of penalty type i

Base Score
----------

The base score measures fundamental task completion performance:

**Base = ω₁ × (kc/kd) + ω₂ × (mc/md)**

Base Score Components
~~~~~~~~~~~~~~~~~~~~

.. list-table:: Base Score Variables
   :header-rows: 1
   :widths: 15 85

   * - Symbol
     - Description
   * - **ω₁**
     - Scoring weight for kit completion (value TBD)
   * - **ω₂**
     - Scoring weight for module completion (value TBD)
   * - **kc**
     - Number of kits completed and accepted
   * - **kd**
     - Number of kits desired/required by the trial
   * - **mc**
     - Number of modules completed and accepted
   * - **md**
     - Number of modules desired/required by the trial

.. note::
   Scoring weights (ω values) are currently being evaluated and will be included in the final documentation release.

Acceptance Criteria
~~~~~~~~~~~~~~~~~~

For kits and modules to count toward the base score, they must meet strict acceptance criteria:

**Kit Acceptance Requirements:**

* Four good cells (not defective and within voltage specification)
* Total kit voltage within specified tolerance:
  
  * Li-Ion kits: 14.4V ± 0.36V (14.04V to 14.76V)
  * NiMH kits: 4.8V ± 0.12V (4.68V to 4.92V)

* Proper cell placement in AGV tray slots
* Successful submission at shipping station

**Module Acceptance Requirements:**

* Four good cells with individual voltages in specification
* Total module voltage within tolerance
* Bottom and top shells properly installed and secured
* All six welds completed successfully:
  
  * Four top welds connecting upper weld plates
  * Two bottom welds connecting lower weld plates

* Module delivered to submission area without damage

Cell Quality Standards
~~~~~~~~~~~~~~~~~~~~~

A cell is considered "good" if it meets all of the following criteria:

1. **Physical Integrity**: No defects detected during inspection
2. **Voltage Specification**: Individual voltage within ±0.2V of nominal:
   
   * Li-Ion: 3.6V ± 0.2V (3.4V to 3.8V)
   * NiMH: 1.2V ± 0.2V (1.0V to 1.4V)

3. **Proper Handling**: Not damaged during manipulation or placement

Bonus Scoring System
-------------------

Five bonus categories reward efficiency, accuracy, and cost-effectiveness:

Bonus Formulas
~~~~~~~~~~~~~

.. list-table:: Bonus Calculations
   :header-rows: 1
   :widths: 15 40 30 15

   * - Bonus
     - Description
     - Formula
     - Notes
   * - **β₁**
     - Trial Time Bonus
     - ω₃ × (1 - tₑ/tₘ)
     - ¹
   * - **β₂**
     - Inspection Speed Bonus
     - ω₄ × (1 - γ/γₐ)
     - ¹²
   * - **β₃**
     - High Priority Speed Bonus
     - ω₅ × (1 - τ/τₐ)
     - ²
   * - **β₄**
     - Sensor Cost Bonus
     - ω₆ × (1 - σ/σᵦ)
     - ²
   * - **β₅**
     - Inspection Classification
     - ω₇ × (ν/δ)
     - ¹

**Notes:**
¹ All kits and modules must be submitted to be eligible for this bonus
² Cannot be negative (minimum value is 0)

Bonus Variable Definitions
~~~~~~~~~~~~~~~~~~~~~~~~~

.. list-table:: Bonus Variables
   :header-rows: 1
   :widths: 15 85

   * - Symbol
     - Description
   * - **ω₃₋₇**
     - Scoring weights for bonuses (values TBD)
   * - **tₑ**
     - Execution time from competition start to end
   * - **tₘ**
     - Maximum time limit for the trial
   * - **γ**
     - Average time per cell inspection (actual)
   * - **γₐ**
     - Desired time per cell inspection (target)
   * - **τ**
     - Actual high priority order execution duration
   * - **τₐ**
     - Desired high priority order execution duration
   * - **σ**
     - Team's total sensor cost
   * - **σᵦ**
     - Available sensor budget
   * - **ν**
     - Number of correct defect classifications
   * - **δ**
     - Total number of defects present in trial

Detailed Bonus Descriptions
~~~~~~~~~~~~~~~~~~~~~~~~~~

**Trial Time Bonus (β₁)**
  Rewards teams that complete all required tasks quickly while maintaining quality. Only available if all kits and modules are successfully submitted.

**Inspection Speed Bonus (β₂)**  
  Encourages efficient inspection processes. Measures average time spent inspecting each cell compared to target inspection duration.

**High Priority Speed Bonus (β₃)**
  Rewards rapid response to high priority orders. Measures time from order receipt to successful submission of NiMH kit.

**Sensor Cost Bonus (β₄)**
  Incentivizes cost-effective sensor strategies. Teams using fewer sensors or lower-cost configurations receive higher bonuses.

**Inspection Classification Bonus (β₅)**
  Rewards accurate defect detection and classification. Based on ratio of correctly identified defects to total defects present.

Penalty System
--------------

Penalties discourage unsafe practices, errors, and inefficient resource usage:

Penalty Schedule
~~~~~~~~~~~~~~~

.. list-table:: Penalty Structure
   :header-rows: 1
   :widths: 15 15 70

   * - Penalty
     - Deduction
     - Description
   * - **ρ₁**
     - 20 points
     - Non-defective cell placed in inspection bin 1
   * - **ρ₂**
     - 20 points
     - Any cell placed in inspection bin 2
   * - **ρ₃**
     - 20 points
     - Object placed on invalid surface
   * - **ρ₄**
     - 40 points
     - AGV collision with environment or robots
   * - **ρ₅**
     - 50 points
     - Robot collision with environment, AGVs, or other robots
   * - **ρ₆**
     - 0.07 points
     - Per dollar of sensor cost over budget

Penalty Details
~~~~~~~~~~~~~~

**Cell Misplacement Penalties (ρ₁, ρ₂)**
  * Inspection bin 1 is intended for defective cells only
  * Inspection bin 2 should remain empty during normal operations
  * Each incorrectly placed cell incurs the full penalty
  * Accumulates for multiple violations

**Invalid Placement Penalty (ρ₃)**
  * Objects must be placed only on designated surfaces
  * Includes cells, shells, tools, and completed assemblies
  * Prevents damage to environment and components

**Collision Penalties (ρ₄, ρ₅)**
  * AGV collisions typically result from navigation errors
  * Robot collisions indicate motion planning failures
  * Higher penalty for robot collisions reflects greater safety risk
  * Includes collisions between multiple robots

**Sensor Budget Penalty (ρ₆)**
  * Applied per dollar over the established sensor budget
  * Encourages strategic sensor selection and placement
  * Small per-dollar penalty allows minor budget overruns
  * Accumulates quickly for significant budget violations

Final Evaluation Process
-----------------------

Trial Design and Execution
~~~~~~~~~~~~~~~~~~~~~~~~~~

**NIST Trial Development**
  NIST designs trials to comprehensively benchmark system performance across various manufacturing scenarios. Trials vary in:
  
  * Required number of kits and modules
  * Defect rates in incoming cells
  * Challenge frequency and timing
  * Time limits and complexity

**Containerization Process**
  Each team's submission is packaged into standardized Docker containers built from the ARIAC base image. This ensures:
  
  * Consistent execution environment across all teams
  * Reproducible results independent of hardware variations
  * Isolated execution preventing interference between teams
  * Standardized resource allocation and monitoring

**Multiple Run Strategy**
  Each team performs five runs per trial with scoring based on the average of the two best runs. This approach:
  
  * Accounts for stochastic variations in system performance
  * Reduces impact of random equipment failures
  * Rewards consistent performance over lucky single runs
  * Provides sufficient data for statistical analysis

**Data Collection and Storage**
  A comprehensive database captures all data necessary for run score calculation:
  
  * Timestamped event logs for all robot actions
  * Sensor data and inspection results
  * Challenge occurrences and system responses
  * Final kit and module acceptance status
  * Performance metrics for bonus calculations

**Video Documentation**
  All runs are recorded for detailed analysis and verification:
  
  * Multi-angle coverage of work environment
  * High-resolution capture of critical operations
  * Synchronization with sensor data and event logs
  * Available for post-competition review and dispute resolution

Scoring Methodology
~~~~~~~~~~~~~~~~~~

**Automated Scoring System**
  The evaluation system automatically calculates scores using:
  
  * Real-time monitoring of task completion
  * Automated detection of penalty-incurring events
  * Precise timing measurements for bonus calculations
  * Quality assessment of submitted kits and modules

**Quality Verification Process**
  Each submitted kit and module undergoes automated verification:
  
  * Voltage measurements for all cells
  * Visual inspection for proper assembly
  * Weld quality assessment through sensor feedback
  * Dimensional accuracy checks for proper fit

**Challenge Response Evaluation**
  System responses to agility challenges are assessed for:
  
  * Detection speed and accuracy
  * Appropriateness of response actions
  * Recovery time and effectiveness
  * Maintenance of quality during disruptions

Performance Benchmarking
------------------------

Evaluation Criteria
~~~~~~~~~~~~~~~~~~

The evaluation focuses on six key dimensions of manufacturing agility:

**Completeness**
  * Ability to finish all assigned tasks within time limits
  * Success rate for complex multi-step operations
  * Handling of varying workload demands

**Efficiency**
  * Speed of task execution without sacrificing quality
  * Resource utilization and waste minimization
  * Optimization of robot and equipment usage

**Accuracy**
  * Precision in defect detection and classification
  * Quality of assembly operations and weld placement
  * Consistency of voltage measurements and matching

**Adaptability**
  * Response effectiveness to unexpected challenges
  * Recovery speed from equipment malfunctions
  * Ability to handle priority changes and interruptions

**Cost-Effectiveness**
  * Strategic sensor deployment within budget constraints
  * Balance between capability and cost
  * Efficient use of available resources

**Robustness**
  * Reliability under various operating conditions
  * Graceful degradation during partial system failures
  * Consistency across multiple trial runs

Scoring Interpretation
~~~~~~~~~~~~~~~~~~~~~

**Score Ranges and Interpretation**

* **Excellent Performance (90-100% of maximum)**: Complete task execution with high efficiency, minimal penalties, and strong bonus achievement
* **Good Performance (70-89% of maximum)**: Solid task completion with moderate efficiency and some bonus achievement
* **Acceptable Performance (50-69% of maximum)**: Basic task completion with room for improvement in efficiency or quality
* **Poor Performance (<50% of maximum)**: Incomplete task execution or significant quality/safety issues

**Competitive Ranking**
Teams are ranked based on their average performance across all trials, with consideration for:

* Consistency of performance across different trial types
* Adaptability to varying challenge conditions
* Innovation in approach and problem-solving
* Overall system robustness and reliability

Statistical Analysis
~~~~~~~~~~~~~~~~~~~

**Performance Metrics**
The evaluation system generates comprehensive statistics:

* **Central Tendency**: Mean, median scores across trials
* **Variability**: Standard deviation, range of performance
* **Reliability**: Consistency metrics and failure rates
* **Efficiency**: Throughput rates and cycle times

**Comparative Analysis**
Teams receive detailed performance comparisons:

* **Peer Benchmarking**: Performance relative to other teams
* **Historical Comparison**: Performance trends across trials
* **Component Analysis**: Breakdown by task, bonus, and penalty categories
* **Improvement Opportunities**: Identification of performance gaps

Result Reporting
---------------

Team Performance Reports
~~~~~~~~~~~~~~~~~~~~~~~

Each team receives comprehensive performance reports including:

**Executive Summary**
* Overall ranking and score summary
* Key strengths and improvement areas
* Comparison to top-performing teams

**Detailed Breakdown**
* Trial-by-trial performance analysis
* Base score, bonus, and penalty breakdowns
* Task completion rates and quality metrics
* Challenge response effectiveness

**Technical Analysis**
* Sensor utilization and cost efficiency
* Robot performance and coordination metrics
* Error patterns and recovery strategies
* Timing analysis for all major operations

**Video Review**
* Access to recorded runs for detailed analysis
* Synchronized data overlays for performance correlation
* Comparison videos showing best practices

Competition Results
~~~~~~~~~~~~~~~~~~

**Public Rankings**
* Final team rankings with scores
* Award categories and recognition
* Anonymized performance statistics
* Best practice highlights

**Technical Insights**
* Analysis of successful strategies and approaches
* Common failure modes and lessons learned
* Technology trends and innovations observed
* Recommendations for future competition improvements

**Standards Impact**
* Insights for robotics agility standards development
* Performance benchmarks for industry reference
* Validation of evaluation metrics and methodologies
* Guidelines for real-world manufacturing applications

Key Success Factors
-------------------

To achieve high scores in ARIAC 2025, teams should focus on:

**System Integration**
* Seamless coordination between perception, planning, and control
* Robust communication and data flow between subsystems
* Effective error handling and recovery mechanisms

**Quality Assurance**
* Reliable defect detection with minimal false positives/negatives
* Precise manipulation and assembly operations
* Consistent voltage measurement and matching algorithms

**Efficiency Optimization**
* Streamlined workflows and motion planning
* Parallel processing and resource utilization
* Minimal waste and rework cycles

**Adaptive Intelligence**
* Quick detection and response to challenges
* Flexible task prioritization and resource allocation
* Learning and improvement from failures

**Strategic Planning**
* Cost-effective sensor deployment strategies
* Risk assessment and mitigation planning
* Performance optimization across all evaluation criteria

The ARIAC 2025 evaluation system is designed to identify and reward manufacturing systems that demonstrate true agility—the ability to perform complex tasks efficiently while adapting intelligently to unexpected challenges and changing requirements. Success requires not just technical capability, but the sophisticated integration and intelligence that characterize next-generation manufacturing systems.