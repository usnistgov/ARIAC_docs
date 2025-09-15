Agility Challenges
==================

ARIAC 2025 includes four agility challenges designed to test system adaptability, resilience, and ability to handle unexpected situations. These challenges simulate real-world manufacturing disruptions that require intelligent responses.

Challenge Categories
-------------------

The challenges are organized into three categories that test different aspects of system agility:

**Disruption**
  System must handle unexpected equipment failures and continue operations

**Adaptation** 
  System must adjust to changing conditions and requirements

**Continuation**
  System must resume operations after interruption with minimal impact

Challenge Overview
-----------------

Four specific challenges test team agility:

1. **Conveyor Malfunction**: Tests response to inspection system failures
2. **Voltage Tester Malfunction**: Tests adaptation to reduced testing capacity
3. **Vacuum Tool Malfunction**: Tests handling of manipulation failures
4. **High Priority Order**: Tests dynamic task switching and prioritization

Malfunction Challenges
---------------------

Conveyor Malfunction
~~~~~~~~~~~~~~~~~~~

**Scenario**: The inspection conveyor and cell feeder experience unexpected stoppage during normal operations.

Challenge Sequence
^^^^^^^^^^^^^^^^^

.. code-block:: text

   1. Normal Operation
      ├── Conveyor running at normal speed
      ├── Cells being fed onto conveyor
      └── Inspection system processing cells
   
   2. Malfunction Trigger  
      ├── Conveyor stops moving
      ├── Cell feed halts
      └── Status topic publishes "malfunction"
   
   3. Required Response
      ├── Pause inspection activities
      ├── Stop attempting cell pickup
      ├── Monitor status topic for recovery
      └── Maintain system state
   
   4. Recovery Phase
      ├── Status topic publishes "operational"
      ├── Conveyor resumes movement  
      ├── Cell feed restarts
      └── Resume inspection operations

**Detection Method**: Monitor conveyor status topic for malfunction flag

**Recovery Strategy**: 
* Implement watchdog timers for status monitoring
* Maintain inspection queue state during downtime
* Restart operations smoothly without losing progress

**Scoring Impact**: 
* No penalties for appropriate pause response
* Penalties for attempting operations during malfunction
* Bonus for quick recovery and minimal disruption

Voltage Tester Malfunction  
~~~~~~~~~~~~~~~~~~~~~~~~~~

**Scenario**: One of the two voltage testers experiences malfunction and stops providing reliable data.

Challenge Sequence
^^^^^^^^^^^^^^^^^

.. code-block:: text

   1. Normal Operation
      ├── Both testers operational
      ├── Load balancing between testers
      └── Normal voltage readings published
   
   2. Malfunction Trigger
      ├── Affected tester stops publishing data
      ├── OR publishes malfunction status
      └── Other tester continues normally
   
   3. Required Response  
      ├── Detect tester failure immediately
      ├── Stop using malfunctioned tester
      ├── Route all cells to operational tester
      └── Adjust workflow for single tester
   
   4. Recovery Phase
      ├── Malfunctioned tester resumes operation
      ├── Status indicates "operational"
      └── Resume using both testers

**Detection Method**: Monitor voltage reading topics and tester status messages

**Adaptation Strategies**:

* **Single Tester Operation**: Modify cell routing to use only operational tester
* **Queue Management**: Handle increased wait times for single tester
* **Throughput Adjustment**: Adapt inspection rate to available capacity
* **Load Balancing**: Efficiently utilize remaining resources

**Recovery Considerations**:
* Verify tester calibration after recovery
* Gradually reintroduce failed tester to workflow
* Monitor for recurring failures

Vacuum Tool Malfunction
~~~~~~~~~~~~~~~~~~~~~~~

**Scenario**: Vacuum gripper grasp attempts fail unexpectedly during shell or module handling.

Challenge Sequence  
^^^^^^^^^^^^^^^^^

.. code-block:: text

   1. Normal Operation
      ├── Vacuum tool functioning properly
      ├── Successful grasping operations
      └── Reliable object manipulation
   
   2. Malfunction Trigger
      ├── Grasp attempt fails
      ├── Object not properly attached
      └── Vacuum system malfunction
   
   3. Required Response
      ├── Detect failed grasp immediately  
      ├── Move tool away from object safely
      ├── Attempt alternative grasp strategy
      └── Re-attempt grasp operation
   
   4. Success Criteria
      ├── Successful object pickup
      ├── Continue with task execution
      └── Monitor for recurring issues

**Detection Methods**:
* Monitor vacuum pressure sensors
* Verify object attachment before movement
* Use force feedback for grasp confirmation
* Implement timeout detection for grasp attempts

**Recovery Strategies**:

**Immediate Response**:
1. **Safety First**: Move away from object to prevent damage
2. **Diagnosis**: Determine cause of grasp failure  
3. **Retry Logic**: Attempt different grasp positions or approaches
4. **Tool Inspection**: Check for debris or mechanical issues

**Alternative Approaches**:
* **Position Adjustment**: Try different grasp points on object
* **Angle Variation**: Approach object from different orientations  
* **Surface Preparation**: Clean object or tool surfaces if needed
* **Tool Change**: Switch to alternative gripper if available

**Failure Handling**:
* **Multiple Attempts**: Allow several retry attempts before escalation
* **Manual Intervention**: Request human assistance if automated recovery fails
* **Task Modification**: Adapt task sequence to work around tool limitations
* **Graceful Degradation**: Continue with reduced capability if necessary

High Priority Challenge
-----------------------

**Scenario**: A high priority order requiring NiMH cells is issued during normal Li-Ion production operations.

Challenge Sequence
^^^^^^^^^^^^^^^^^

.. code-block:: text

   1. Normal Li-Ion Production
      ├── Li-Ion cells being fed
      ├── Regular kits in production
      └── Normal workflow active
   
   2. High Priority Trigger
      ├── High priority order message received
      ├── Order specifies NiMH cell requirement
      └── Associated order ID provided
   
   3. Required Response  
      ├── Switch cell feed to NiMH immediately
      ├── Pause or complete current Li-Ion tasks
      ├── Build NiMH kit with required specs
      └── Prioritize high priority order processing
   
   4. Order Completion
      ├── Move AGV with NiMH kit to shipping
      ├── Submit high priority order with ID
      ├── Wait for acceptance confirmation
      └── Resume normal Li-Ion operations
   
   5. Normal Operations Resume
      ├── Switch cell feed back to Li-Ion
      ├── Continue with regular production
      └── Monitor for additional priority orders

**Message Interface**: 
* Topic: ``/high_priority_orders``
* Type: ``ariac_interfaces/msg/HighPriorityOrder``
* Content: Order ID and specifications

**Submission Interface**:
* Service: ``/submit_high_priority_order``
* Type: ``ariac_interfaces/srv/SubmitHighPriorityOrder``
* Required: Order ID matching the request

Implementation Strategy
~~~~~~~~~~~~~~~~~~~~~~

**Task Prioritization**:

1. **Immediate Assessment**: Evaluate current system state
2. **Graceful Interruption**: Complete critical operations before switching
3. **Resource Reallocation**: Direct system resources to high priority task
4. **Context Preservation**: Save state of interrupted tasks for later resumption

**Cell Feed Management**:

.. code-block:: python

   # Pseudocode for cell feed switching
   def handle_high_priority_order(order):
       # Save current state
       current_state = save_system_state()
       
       # Switch to NiMH production
       switch_cell_feed('NiMH')
       
       # Build high priority kit
       nimh_kit = build_kit_with_cells('NiMH', order.requirements)
       
       # Submit order
       submit_high_priority_order(nimh_kit, order.id)
       
       # Resume normal operations
       switch_cell_feed('Li-Ion')
       restore_system_state(current_state)

**Quality Assurance**:
* Verify NiMH cell specifications (1.2V nominal)
* Ensure kit meets high priority requirements
* Confirm successful order submission before resuming

**Performance Metrics**:
* **Response Time**: Time from order receipt to cell feed switch
* **Execution Time**: Total time to complete high priority order
* **Recovery Time**: Time to resume normal operations
* **Quality**: Accuracy of high priority kit specifications

Challenge Response Strategies
----------------------------

General Principles
~~~~~~~~~~~~~~~~~

**Monitoring and Detection**:
* Implement comprehensive system health monitoring
* Use multiple detection methods for redundancy
* Set appropriate timeout thresholds for failure detection
* Maintain real-time status awareness across all subsystems

**Response Planning**:
* Develop predetermined response protocols for each challenge type
* Create decision trees for different failure scenarios  
* Implement escalation procedures for unresolvable issues
* Design graceful degradation strategies for partial system failures

**Recovery Implementation**:
* Verify system readiness before resuming operations
* Implement gradual ramp-up procedures after recovery
* Monitor for recurring issues or cascading failures
* Maintain operation logs for post-incident analysis

Robust System Design
~~~~~~~~~~~~~~~~~~~~

**Fault Tolerance**:

* **Redundancy**: Design backup systems for critical components
* **Isolation**: Prevent failures from cascading to other subsystems
* **Graceful Degradation**: Maintain partial functionality during failures
* **Error Boundaries**: Contain failures within specific system modules

**State Management**:

* **Checkpointing**: Save system state at regular intervals
* **Rollback Capability**: Restore to known good states after failures
* **State Consistency**: Maintain coherent system state across all components
* **Recovery Validation**: Verify system integrity after recovery

**Adaptive Control**:

* **Dynamic Reconfiguration**: Adjust system parameters based on current conditions
* **Load Balancing**: Redistribute tasks when resources become unavailable
* **Priority Management**: Handle competing demands for system resources
* **Performance Optimization**: Adapt strategies based on system performance

Testing and Validation
----------------------

Challenge Simulation
~~~~~~~~~~~~~~~~~~~

Teams should test their systems against all challenge scenarios:

**Development Testing**:
* Create isolated test environments for each challenge type
* Implement challenge injection mechanisms for controlled testing
* Validate detection and response timing requirements
* Test recovery procedures under various system states

**Integration Testing**:
* Test challenges during normal production workflows
* Verify system behavior with multiple simultaneous challenges
* Validate performance under stress conditions
* Ensure no interference between challenge response mechanisms

**Performance Benchmarking**:
* Measure response times for each challenge type
* Assess impact on overall system throughput
* Evaluate quality metrics during challenge handling
* Compare performance before, during, and after challenges

Success Metrics
~~~~~~~~~~~~~~

**Challenge Response Evaluation**:

* **Detection Speed**: Time to identify challenge occurrence
* **Response Appropriateness**: Correctness of immediate response actions
* **Recovery Efficiency**: Speed and effectiveness of returning to normal operation
* **Quality Maintenance**: Preservation of output quality during challenges
* **Throughput Impact**: Minimization of productivity losses

**Scoring Considerations**:
* Bonuses for rapid and appropriate challenge responses
* Penalties for inappropriate actions during malfunctions
* Evaluation of overall system resilience and adaptability
* Assessment of learning and improvement across multiple challenge instances

The agility challenges in ARIAC 2025 are designed to test not just technical capability, but the intelligence and adaptability that distinguish truly agile manufacturing systems. Success requires robust system design, intelligent monitoring, and sophisticated response strategies that can handle the unexpected while maintaining high performance and quality standards.