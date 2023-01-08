-add these things in locosim/ros_impedance_controller/worlds/models
-change in locosim/robot_control/lab_exericses/lab_palopoli/ur5_generic.py the file .world with lego.world
-to add new lego manually copy and paste 
```
<include>
      <name>**yourblockname**</name>
      <uri>model://**yourblockname**</uri>
      <!-- Spawn at the following position on the table (x, y, z)-->
      <pose>0.4 0.4 0.9 0 0 0</pose>
    </include>
```
