# Flexbotics Tool Exchanger Manager

This package contains the node for Tool Exchanger & IO management in Flexbotics. This is the list of services provided by Flexbotics Tool Exchanger Manager:

* **LoadToolExchangerInfo**: Load the necessary tool exchange informations from a specific XML file. 
* **GrabTool**: Service used to grab a specific tool with a specific robot/group_name.
* **ReleaseTool**: Service used to release a specific tool with a specific robot/group_name.
* **SetMovementParameters**: Movement parameters such as velocity_ratio, translation_velocity, rotation_velocity, timeout are initialized with default values but can be changed through this service.