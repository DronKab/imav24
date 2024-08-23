# imav24

# Installation of controls PID

1. Clonning just controls package

    You need to be at your current src of your workspace

    '''sh
    git clone https://github.com/DronKab/imav24/controlsPID.git
    '''

2. Compile the project

    '''sh
    cd ..
    source /opt/ros/humble/setup.bash
    colcon build
    '''

3. Run the code
    
    At this moment, control and image node are on test and they are not recognized as executable files

    '''sh
    source ~/.bashrc
    cd src/controlsPID/controlsPID/
    python3 cameraImage.py
    '''