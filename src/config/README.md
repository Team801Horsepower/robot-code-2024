# TOML-based config files for the robot
---
Robot-specific configuration is deployed through the use of a TOML file called `config.toml` in the root directory of the RoboRIO. Each robot must have a unique config file, since the file typically contains vital information about CAN IDs, motor positions, dimensions, drive types, and others. 

### Instructions for deploying the file to the RoboRIO
Deploys work by placing the config.toml file in the root directory of the RoboRIO's filesystem. This can be accomplished in many ways, but should be done using the following steps:
1. Use SSH to access the RIO. All future commands must be completed from within the SSH session. Command: `ssh admin@10.8.01.2`
2. Check for the file, and read its current contents: `cat /config.toml`
3. Copy the desired config file from the directory in which it is deployed to the robot (`/home/lvuser/py/config/`) by running `cp`: `cp /home/lvuser/py/config/config-2022-swerve-chassis.toml /config.toml`
4. Repeat Step 2. If the changes are successful, proceed. Otherwise, Google your specific problems. Try `touch /config.toml && sudo chmod 777 /config.toml` if you encounter issues.
5. Exit the SSH session: `exit`