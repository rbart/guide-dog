guide-dog
=========

Project Details
---------------
Project details can be found at http://rbart.github.com/guide-dog/

Install and Run
---------------
-   Guide-Dog requires Ubuntu Linux version 12.04.1 LTS
-   Open a Terminal and execute (these instructions taken from
    http://pointclouds.org/downloads/linux.html ):

        sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl
        sudo apt-get update
        sudo apt-get install libpcl-all

-   Download and extract a copy of the OpenCV source code from
    http://opencv.willowgarage.com/wiki/
-   Change directory into the 'OpenCV-2.4.2' directory.
-   Exectue 'mkdir build'.
-   Change directory to 'build'.
-   Execute 'cmake ..'.
-   After cmake has configured successfully, execute `sudo make install`.
-   Download and extract a copy of the PCL source code from
    http://www.pointclouds.org/assets/files/1.6.0/PCL-1.6.0-Source.tar.bz2
-   Download Guide-Dog to the same parent directory as the PCL source code by
    running `git clone https://github.com/rbart/guide-dog.git`
-   From the parent directory that contains the `guide-dog` directory and
    `PCL-1.6.0-Source` directory execute
    `cp -R guide-dog/sonic-dog/ PCL-1.6.0-Source/; cp -R guide-dog/combined/visualization/ PCL-1.6.0-Source/`
-   Change directory to the `PCL-1.6.0-Source` directory.
-   Execute `mkdir build`.
-   Change directory to `build`.
-   Execute `cmake ..` and make sure that the visualization package will be built.
-   Execute `make openni_viewer`.
-   Copy the .wav files under guide-dog/sonic-dog to 'build'.
-   Execute `./bin/openni_viewer` to run Guide-Dog.
