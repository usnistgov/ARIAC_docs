Competition Schedule
===================

ARIAC 2025 follows a structured timeline designed to provide teams with adequate preparation time while maintaining competitive momentum. The schedule includes key milestones for code release, testing, submission, and final evaluation.

Important Dates
---------------

.. list-table:: ARIAC 2025 Timeline
   :header-rows: 1
   :widths: 25 75

   * - Date
     - Event
   * - **September 4, 2025**
     - Kick-off Presentation
   * - **September 12, 2025**
     - Code Release
   * - **December 8, 2025**
     - Smoke Test Submission Deadline
   * - **December 12, 2025**
     - Smoke Test Results Released
   * - **January 2, 2026**
     - Finals Submission Deadline
   * - **February 2, 2026**
     - Finals Results Announced

Phase 1: Kick-off and Preparation
---------------------------------

Kick-off Presentation (September 4, 2025)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Completed Event**: The competition officially launched with a comprehensive presentation covering:

* Competition overview and objectives
* Technical specifications and requirements
* Task descriptions and evaluation criteria
* Interface documentation and API references
* Challenge scenarios and response strategies
* Timeline and submission requirements

**Available Resources**:
* Presentation slides and materials
* Initial documentation and specifications
* Community forums for questions and discussion
* Contact information for technical support

Code Release (September 12, 2025)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Deliverables**:
The complete ARIAC 2025 software package will be released, including:

* **Simulation Environment**: Gazebo simulation with complete EV battery factory
* **ROS Packages**: All interface definitions and example implementations
* **Sensor Models**: Configurable sensor packages with different grades and costs
* **Robot Models**: UR5e robots with Robotiq grippers and vacuum tools
* **Example Code**: Reference implementations and tutorials
* **Docker Base Image**: Standardized container for final submissions

**Installation Requirements**:
* Ubuntu 24.04 Noble (required)
* ROS Jazzy Jalisco
* Gazebo Sim (Harmonic)
* Minimum 16GB RAM, 8-core CPU recommended
* NVIDIA GPU recommended for enhanced graphics and sensor simulation

**Initial Setup**:
Teams should immediately focus on:

1. **Environment Setup**: Install and configure the simulation environment
2. **Basic Familiarization**: Run provided tutorials and examples
3. **Interface Testing**: Verify ROS topic and service communications
4. **Sensor Configuration**: Experiment with different sensor placements and grades
5. **Team Organization**: Establish development workflows and responsibilities

Development Phase (September 12 - December 8, 2025)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Duration**: Nearly 3 months for full system development

**Recommended Milestones**:

**Month 1 (September 12 - October 12)**:
* Complete environment setup and basic tutorials
* Develop fundamental robot control and perception systems
* Implement basic Task 1 (inspection and kit building) functionality
* Begin sensor strategy development and testing

**Month 2 (October 12 - November 12)**:
* Complete Task 1 implementation with challenge handling
* Develop Task 2 (module construction) capabilities
* Implement tool changing and vacuum gripper control
* Integrate multi-robot coordination systems

**Month 3 (November 12 - December 8)**:
* System integration and optimization
* Comprehensive testing of all challenge scenarios
* Performance tuning and sensor optimization
* Smoke test preparation and submission packaging

**Development Resources**:
* **GitHub Repository**: Access to latest code updates and bug fixes
* **Documentation**: Comprehensive guides at pages.nist.gov/ARIAC_docs
* **Community Support**: Issue tracking and team collaboration forums
* **Tutorials**: Step-by-step guides for common implementation patterns
* **Web Application**: New ARIAC tool for configuration and testing

Phase 2: Smoke Test
-------------------

Smoke Test Submission (December 8, 2025)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Purpose**: The smoke test serves as a preliminary evaluation to:
* Verify team systems can run in the standardized Docker environment
* Identify and resolve technical issues before final submission
* Provide teams with performance feedback and improvement guidance
* Test the evaluation infrastructure and scoring systems

**Submission Requirements**:
* **Docker Container**: Complete system packaged in standardized format
* **Configuration Files**: Sensor placement and system parameter specifications
* **Documentation**: Brief system description and operating instructions
* **Source Code**: Complete source code for verification and analysis

**Submission Format**:
Teams must submit via the official ARIAC submission system:

1. **Container Registry**: Docker image uploaded to specified repository
2. **Configuration Package**: YAML files defining sensor placement and parameters
3. **System Documentation**: Technical description of approach and algorithms
4. **Verification Checklist**: Completed testing and validation checklist

**Technical Requirements**:
* Container must derive from official ARIAC base image
* All dependencies must be included in container
* System must run without external network access
* Maximum container size and resource limits apply
* Automated testing interface must be properly implemented

Smoke Test Evaluation (December 8-12, 2025)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Evaluation Process**:
* Automated deployment and testing of submitted containers
* Execution of standardized test scenarios
* Performance measurement and scoring calculation
* Error detection and diagnostic report generation

**Test Scenarios**:
Smoke test includes simplified versions of final evaluation trials:
* Basic kit building without challenges
* Simple module construction tasks
* Limited sensor configurations
* Reduced time pressure and complexity

Smoke Test Results (December 12, 2025)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Feedback Provided**:
Teams receive comprehensive feedback including:

* **Performance Scores**: Preliminary scores across all evaluation categories
* **System Functionality**: Success/failure status for major system components
* **Technical Issues**: Detailed error reports and diagnostic information
* **Improvement Recommendations**: Specific suggestions for enhanced performance
* **Benchmarking Data**: Anonymous comparison with other team performance

**Use of Feedback**:
Teams should use smoke test results to:
* Identify and fix critical system failures
* Optimize performance in low-scoring areas
* Refine sensor strategies based on cost-effectiveness analysis
* Improve challenge response mechanisms
* Enhance system robustness and reliability

Phase 3: Final Preparation
--------------------------

Final Development Period (December 12, 2025 - January 2, 2026)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Duration**: 3 weeks for final improvements and optimization

**Recommended Focus Areas**:

**Week 1 (December 12-19)**:
* Address critical issues identified in smoke test
* Implement performance improvements based on feedback
* Enhance error handling and recovery mechanisms

**Week 2 (December 19-26)**:
* System optimization and performance tuning
* Comprehensive testing of all scenarios and challenges
* Final sensor strategy refinement and validation

**Week 3 (December 26 - January 2)**:
* Final integration testing and validation
* Submission preparation and documentation
* Last-minute bug fixes and stability improvements

**Holiday Considerations**:
Teams should plan development schedules accounting for holiday periods and reduced availability during late December.

Finals Submission (January 2, 2026)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Final Submission Requirements**:
* **Complete System**: Fully functional robot control system
* **Docker Container**: Production-ready container with all optimizations
* **Comprehensive Documentation**: Detailed technical documentation and user guide
* **Sensor Configuration**: Final sensor placement strategy with cost analysis
* **Performance Validation**: Evidence of testing and performance verification

**Submission Completeness Checklist**:
* All required interfaces properly implemented
* Challenge response mechanisms tested and validated
* Multi-robot coordination systems functional
* Sensor integration and data processing optimized
* Error handling and recovery systems robust
* Documentation complete and accurate

Phase 4: Final Evaluation
-------------------------

Evaluation Period (January 2 - February 2, 2026)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Evaluation Process**:
* **Container Deployment**: Automated deployment of all team submissions
* **Trial Execution**: Multiple trials with varying difficulty and challenge types
* **Performance Measurement**: Comprehensive scoring across all evaluation categories
* **Data Collection**: Detailed logging and video recording of all runs
* **Quality Assurance**: Verification of results and dispute resolution

**Trial Characteristics**:
Final evaluation includes diverse trials designed to test:
* Complete system functionality under normal conditions
* Adaptability to various challenge scenarios
* Performance under time pressure and resource constraints
* Robustness across different environmental conditions
* Consistency and reliability over multiple runs

**Evaluation Infrastructure**:
* Standardized computing environment for fair comparison
* Automated trial execution and data collection
* Real-time monitoring and performance measurement
* Comprehensive logging for post-evaluation analysis

Finals Results (February 2, 2026)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Results Announcement**:
* **Team Rankings**: Final competition standings with detailed scores
* **Performance Analysis**: Comprehensive breakdown of team performance
* **Award Recognition**: Announcement of various award categories
* **Technical Insights**: Analysis of successful strategies and innovations

**Awards and Recognition**:
Multiple award categories recognize different aspects of excellence:
* **Overall Winner**: Highest total score across all evaluation criteria
* **Innovation Award**: Most creative or novel technical approach
* **Efficiency Award**: Best performance in speed and resource utilization
* **Robustness Award**: Most reliable system across varying conditions
* **Quality Award**: Highest accuracy in inspection and assembly tasks

**Post-Competition Activities**:
* **Technical Presentations**: Opportunity for teams to present their approaches
* **Standards Development**: Integration of insights into robotics agility standards
* **Academic Publications**: Collaborative papers on competition results and insights
* **Industry Engagement**: Sharing of results with manufacturing industry partners

Preparation Recommendations
---------------------------

For Teams
~~~~~~~~~

**Technical Preparation**:
* **Early Start**: Begin development immediately after code release
* **Iterative Development**: Implement and test components incrementally
* **Regular Testing**: Frequent testing against challenge scenarios
* **Performance Monitoring**: Continuous measurement and optimization

**Team Organization**:
* **Role Definition**: Clear assignment of responsibilities and expertise areas
* **Communication**: Regular team meetings and progress reviews
* **Version Control**: Proper code management and collaboration tools
* **Backup Plans**: Contingency strategies for technical issues

**Timeline Management**:
* **Milestone Planning**: Break development into manageable phases
* **Buffer Time**: Account for unexpected issues and debugging
* **Holiday Planning**: Adjust schedules for holiday periods
* **Final Testing**: Adequate time for comprehensive system validation

For Mentors and Advisors
~~~~~~~~~~~~~~~~~~~~~~~

**Student Guidance**:
* **Technical Mentoring**: Guidance on robotics, AI, and systems integration
* **Project Management**: Help with timeline and milestone planning
* **Resource Allocation**: Assistance with computing and development resources
* **Career Development**: Connection to industry opportunities and research directions

**Institutional Support**:
* **Computing Resources**: Access to development and testing environments
* **Collaboration Tools**: Platforms for team communication and code sharing
* **Academic Integration**: Connection to coursework and research projects
* **Industry Connections**: Networking with potential employers and partners

The ARIAC 2025 schedule is designed to provide teams with sufficient time for thorough preparation while maintaining competitive momentum. Success requires early engagement, consistent development progress, and strategic use of feedback from the smoke test to optimize final performance.