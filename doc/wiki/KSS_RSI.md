## KSS driver (RSI)

### Setup

#### Client side
It is recommended to use the driver on a real-time capable client machine (further information about setting up the PREEMPT_RT patch can be found [here](https://github.com/kroshu/kuka_drivers/wiki/Realtime)).

To set up the controller with WorkVisual (which is necessary if RSI is not yet installed), a Windows machine is also required.

##### Client IP configuration
- Set a fixed IP in the subnet of the KLI interface for the Windows machine, which is required to connect to WorkVisual and transfer the project
- Set a fixed IP in the subnet of the RSI interface for the real-time machine, which is required to send commands via the RSI interface.

#### Controller side
These instructions were tested with RSI 4.1.3 (on KSS8.6) and RSI 5.0.2 (on KSS8.7)

##### Controller network configuration

Windows runs behind the SmartHMI on the teach pad. Make sure that the **Windows interface** of the controller and the **PC with ROS** is connected to the same subnet.

1. Log in as **Expert** or **Administrator** on the teach pad and navigate to **Network configuration** (**Start-up > Network configuration > Activate advanced configuration**).
   There should already be an interface checked out as the **Windows interface**.
   - **Windows interface checkbox** should be checked.
2. Add a new network for RSI:

   **KRC4:**
   - Minimize the SmartHMI (**Start-up > Service > Minimize HMI**).
   - Run **RSI-Network** from the Windows Start menu (**All Programs > RSI-Network**).
   - Check that the **Network - Kuka User Interface** shows the Windows interface with the specified IP address.
   - Add a new IP address on another subnet for the **RSI interface**.
     - Select the entry **New** under **RSI Ethernet** in the tree structure and press **Edit**.
     - Enter the IP address and confirm with **OK**.
     - Close **RSI-Network** and maximize the SmartHMI.

   **KRC5:**
   - Press the **Advanced** button and **New interface**.
   - Select **Mixed IP address** and keep the default **Receiving task: Target subnet** and **Real-time receiving Task: UDP**
   - Set the IP address to a different subnet then the **KLI interface**.
     - **Default gateway**: leave it empty
     - **Windows interface checkbox** should NOT be checked

3. Reboot the controller with a cold restart (**Shutdown > Check *Force cold start* and *Reload files* > Reboot control PC**).

##### Update and upload configuration files
There are 3 files necessary for RSI that are available in the `krl` directory:

- `ros_rsi_ethernet.xml`: specifies the data transferred via RSI and contains the IP configuration of the client machine:
  - The `IP_NUMBER` tag should be modified so that it corresponds to the IP address previously added for your (real-time) PC.
  - The `PORT` might be left as it is (59152), but can be also changed if a different port is to be used on the client machine.

- `ros_rsi.src`: This contains the KRL program that starts external control and should not be modified.
- `ros_rsi.rsix`: This contains the RSI context (can be visualized with **RSIVisual**). It can be modified for example to add filtering behaviour, but this is not recommended and should be implemented on the client side instead.
  - For older RSI versions (<=4.0.3), the context can only be defined in 3 different files: `ros_rsi.rsi.xml`, `ros_rsi.rsi.diagram` and `ros_rsi.rsi`, these can be found under `krl/deprecated`. In this case, these 3 files should be copied to the controller instead of the `ros_rsi.rsix`.

There are two options to upload these files to the controller:

Method 1:
1. Copy the files to a USB-stick.
2. Plug it into the teach pad or controller.
3. Log in as **Expert** or **Administrator** on the controller.
4. Copy the `ros_rsi.src` file to `KRC:\R1\Program`.
5. Copy the rest of the files to `C:\KRC\ROBOTER\Config\User\Common\SensorInterface`.

Method 2:
1. Connect to the KRC with WorkVisual
2. Log in as **Expert** or **Administrator** on the controller.
4. Copy the `ros_rsi.src` file to `KRC:\R1\Program` in WorkVisual
5. Copy the rest of the files to `C:\KRC\ROBOTER\Config\User\Common\SensorInterface` in WorkVisual
6. Deploy the project

### Configuration

#### Startup configuration

The following configuration files are available in the `config` directory of the package:
- `driver_config.yaml`: contains the IP address of the client machine
- `ros2_controller_config.yaml`: contains the controller types for every controller name. Should be only modified if a different controller is to be used. The `configure_components_on_start` parameter should never be modified, which ensures that the hardware interface is not activated at startup.
- configuration files for specific controllers, for further information, see the documentation of the given controller

##### IP configuration
The following parameters must be set in the driver configuration file:
- `client_ip`: IP address of the client machine, should be identical to the one set in `ros_rsi_ethernet.xml`
- `client_port`: port of the real-time communication on the client machine, should be identical to the one set in `ros_rsi_ethernet.xml`

#### Runtime parameters

The KSS driver currently does not have runtime parameters. Control mode cannot be changed if the driver is running, as that also requires modifying the RSI context on the controller.

### Usage

#### Starting the driver

1. To start the driver, two launch file are available, with and without `rviz`. To launch (without `rviz`), run:
    ```
    ros2 launch kuka_kss_rsi_driver startup.launch.py
    ```
     - This starts the 3 core components of every driver (described in the *Non-real-time interface* section of the [project overview](Project%20overview.md)) and the following controllers:
       - `joint_state_broadcaster` (no configuration file, all state interfaces are published)
       - `joint_trajectory_controller` ([configuration file](../../kuka_kss_rsi_driver/config/joint_trajectory_controller_config.yaml))

     -  After successful startup, the `robot_manager` node has to be activated to start the cyclic communication with the robot controller, see further steps (before this only a collapsed robot is visible in `rviz`):

2. Configure and activate all components the driver:
    ```
    ros2 lifecycle set robot_manager configure
    ros2 lifecycle set robot_manager activate
    ```
   - The hardware interface is now waiting for the robot controller to connect, the timeout for this is currently 10 seconds
3. Start the `KRC:\R1\Program\ros_rsi.src` program on the controller and execute the line of `RSI_MOVECORR()`
   - in T1, a warning (*!!! Attention - Sensor correction goes active !!!*) should be visible after reaching `RSI_MOVECORR()`, which should be confirmed to start this step

On successful activation the brakes of the robot will be released and external control is started. To test moving the robot, the `rqt_joint_trajectory_controller` is not recommended, use the launch file in the `iiqka_moveit_example` package instead (usage is described in the *Additional packages* section of the [project overview](Project%20overview.md)).


##### Launch arguments

Both launch files support the following arguments:
- `robot_model` and `robot_family`: defines which robot to use. The available options for the valid model and family combinations can be found in the [readme](https://github.com/kroshu/kuka_robot_descriptions?tab=readme-ov-file#what-data-is-verified) of the `kuka_robot_descriptions` repository.
- `use_fake_hardware`: if true, the `mock_components/GenericSystem` will be used instead of the `KukaRSIHardwareInterface`. This enables trying out the driver without actual hardware.

#### Stopping external control

To stop external control, all components have to be deactivated with `ros2 lifecycle set robot_manager deactivate`

BEWARE, that this is a non-realtime process including lifecycle management, so the connection is not terminated immediately, in cases where an abrupt stop is needed, the safety stop of the Teach Pendant should be used!


### Simulation

To try out the driver with an open-loop simulation, the driver and the `kuka_rsi_simulator` must be started, (before activation only a "collapsed" robot will be visible in `rviz`):

```
ros2 launch kuka_kss_rsi_driver startup_with_rviz.launch.py
```

```
ros2 launch kuka_rsi_simulator kuka_rsi_simulator_launch.py
```

After all components have started successfully, the system needs to be configured and activated to start the simulation. The robot will be visible in rviz after activation:

```
ros2 lifecycle set robot_manager configure
ros2 lifecycle set robot_manager activate
```

### Known issues and limitations

- There are currently heap allocations in the control loop (hardware interface `read()` and `write()` functions), therefore the driver is not real-time safe
- In case of an error on the controller side, the driver is not deactivated
- Cartesian position control mode and I/O-s not yet supported
