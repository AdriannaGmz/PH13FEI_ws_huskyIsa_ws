### Multiple cameras, *slaves triggered by arduino signal*

Trigger both cameras  as "slave" from an external trigger output (from an arduino) , using  5v with pull down resistors

* Select the camera, have both cameras with Line2 as input
  * Features tab -» Digital IO Control
  * Set the input line, **Line 2** 
    * select **Line2** from the Line Selection dropdown 
    * set Line Mode to Input.
* In Acquisition control /  GPIO tab
  * Trigger Source -»  **Line 2**
  * Trigger Mode -»  On
  * *Trigger Overlap-»  Read Out*  



