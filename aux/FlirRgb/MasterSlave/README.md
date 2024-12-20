Where the Master is *camera ip 29   sn 20039932* that triggers the signal to the **Slave** *camera ip 28   sn 20243318*, using SpinView.



1. Power both cameras with GPIO (both Green wires -» 12v, both Brown and Blue wires -» GND), 
2. Connect trigger in GPIO cameras according to image  MasterSlaveBFS.JPG


In **Spinview** modify the settings as follows:  (See images in this folder as well)


	*For the primary camera 20039932*

* Select the camera.

* Features tab -» Digital IO Control.

  * Set the output line, **Line 1** 
    * select **Line1** from the Line Selection dropdown 
    * set Line Mode to Output.
  * Enable the 3.3V line (**Line 2**)
    *     From the line selection drop-down select Line2 and 
    *     check the checkbox for 3.3V Enable.

* (Optional) Save the settings in a user set:

  *     Click User Set Control.
  *     From the User Set Selector drop-down, select User Set 0.
  *     Click User Set Save.
  *     (Optional) From the User Set Default drop-down, select User Set 0. This will ensure that this userset is loaded when the camera is booted up.

  

	*For each secondary camera 20243318, etc*

* Select the camera.
* Features tab -» Digital IO Control
  * Set the input line, **Line 2** 
    * select **Line2** from the Line Selection dropdown 
    * set Line Mode to Input.
* In Acquisition control /  GPIO tab
  *     Trigger Source -»  **Line 2**
  *     Trigger Overlap-»  Read Out
  *     Trigger Mode -»  On
* (Optional) Save the settings in a user set:
  *     Click User Set Control
  *     From the User Set Selector drop-down, select User Set 0 
  *     Click User Set Save
  *     (Optional) From the User Set Default drop-down, select User Set 0. This will ensure that this userset is loaded when the camera is booted up.



In **Spinview**, to visualize both cameras

* Adjust Linux settings (MTU)  (`sudo ifconfig eno1 mtu 9000`)
* Allow incomplete Frames (right click on both image frames)
* Initialize first Master camera and then Slave camera



In **Spinview**, reduce frame size


* Use max binning 4 x 4 instead of 1 x 1

  * Image Format Control -» Set Binning value for both Vertical and Horizontal to **4** 



In **Spinview**, set frame rate (fps)


* Acquisition control -» Acquisition Frame Rate



