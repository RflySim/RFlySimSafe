# Purpose and audience
The primary purpose of this user manual is to provide researchers, developers, and practitioners in the field of unmanned aerial vehicles (UAVs) with a detailed operational guide and reference for the RflySimSaT UAV Safety Assessment Platform. The operational guidelines aim to assist users in comprehensively understanding and utilizing the platform for assessing the safety of UAVs, mitigating risks, and ensuring system stability and performance. Specifically, the objectives of this user manual are:

+ **Comprehensive Feature Description**: Help users become familiar with the various modules, functionalities, and features of the RflySimSaT platform, enabling them to leverage these tools effectively for safety assessments.；

+ **Detailed Operating Instructions**: Provide users with step-by-step operational procedures to ensure they can correctly configure, run, and analyze the safety of UAVs；

+ **Diverse Analysis Methods**: Introduce users to how the platform can be used for analyzing UAV performance in different scenarios, including fault simulation, risk assessment, and computation of safety metrics.

**_Target Audience_**

The intended audience for this user manual includes the following categories of individuals:
+ **UAV Researchers**: UAV manufacturers and software developers aiming to ensure the excellent performance of their products. They need to assess the safety, performance, and stability of UAVs. The manual offers effective tools and methods to meet their research needs；
+ **Industry Practitioners**: Professionals in various industries where UAVs are widely utilized, such as agriculture, logistics, and aviation. Industry practitioners need to understand how to assess the safety of UAVs to ensure their safety in daily operations；
+ **Educational Institutions**: Universities, research institutions, and training centers that may use the RflySimSaT platform for teaching UAV-related courses. The manual provides rich cases and operational examples suitable for teaching and research；
+ **Government Regulatory Authorities**: Government agencies require an understanding of UAV performance and safety to formulate appropriate regulations and policies. The manual can help them grasp methods and tools for UAV safety assessment.

This user manual is designed to follow a style typical of usage manuals, providing clear and concise instructions for effective utilization of the RflySimSaT UAV Safety Assessment Platform.

# File structure
Figure below shows the file structure of the automated testing platform. The "_RflySimSDK_" file is a dedicated proprietary library created for the RflySim platform, containing key interfaces and library files for control, communication, data, vision, etc. The "_include_" branch library files are related to automatic safety testing management, encompassing crucial communication interfaces for automated testing and assessment. Key components include:

+ _AutoMAVDB_：Common library for test case and data management, featuring automated operations on databases and files.
+ _AutoMAVCmd_：File for parsing test sequences, mapping different test sequences to corresponding control commands to drive unmanned vehicle movement.
+ _AutoMAVCtrl_：File for unmanned vehicle process management, covering test environment management, test task management, vehicle configuration, etc.
+ _PX4MavCtrlV4_：File for communication management of unmanned vehicles, mapping key user commands to corresponding Mavlink messages, enabling communication initialization, control, and communication with other components.
+ _CameraCtrlApi_ and VisionCaptureApi: Files for visual management of unmanned vehicles, handling visual data configuration, management, and other operations.

The "src" branch library include:
+ _autotest_: Main entry file for automated testing, allowing for one-click execution of automated tests for a specific vehicle by configuring its information in this file.
+ _openSHA_：Common library files for health and safety assessment. The RflySimSaT platform provides an example of safety assessment based on the rate model reliability. Users can build their own safety and health assessment algorithms and perform online verification on this platform.
+ _model_：This file contains model files and configuration information for different unmanned vehicles. The "db.json" folder in this directory can be configured to set up test cases for a specific model.
+ _example_：This folder contains a rich history, including custom fault modeling, public template usage, automatic test interface, and other custom functions

Other branch library include:
+ _data_：Data management files for automated collection of test data from unmanned vehicles. The directory has four structures corresponding to four scenarios: 1) data/xITL/single/sInstance for single vehicle type and single instance; 2) data/xITL/single/mInstance for single vehicle type and multiple instances; 3) data/xITL/multi/sInstance for multiple vehicle types and single instance; 4) data/xITL/multi/mInstance for multiple vehicle types and multiple instances. Here, xITL represents both SITL and HITL simulation data.
+ _case_：Database files. Due to the relatively complex nature of database file operations, each unmanned vehicle file in src/model provides a "db.json" file for configuring test cases. Test case configuration can also be done through a database visualization tool (in the tools folder).
+ _docs_: Documentation folder providing a user manual for the platform, detailing principles, and usage methods.
