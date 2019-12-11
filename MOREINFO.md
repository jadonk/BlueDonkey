## Information for Advanced Configurations

* Should the Pause/Unpause switch on the dashboard be out of sync, toggling it once will reset it.
  * Using the button on a BeagleBone-Blue does not trigger the button on the dashboard.
* Location to modify /path/to/python/code/
  * Car_control.py: car_control __init__ method: self.location. 
    * This requires a trailing ‘\’
  * Node-RED change node at the top of the flow named ‘Set Global Path’
    * Done in JSON
    * This requires a trailing ‘\’
* More settings are currently listed and available than are necessarily useful to edit or view.
* We are not 100% certain on whether the physical button will properly work when on BeagleBone-Blue, we had difficulty testing the difference between button and dashboard.
* The code has not been tested with servos enabled.
* Node-RED does not store the python variables anywhere, they are only accessed and passed through to the destination.
* Client.py must be executable (Chmod +x)
* Changing either the Node-RED dashboard or the python code may cause the need to reload the Node-RED dashboard UI webpage.
* Node-RED Specific:
  * Several debug nodes were created that can be toggled
  * The Node-RED Editor does not automatically update dashboard.json, it must be exported and saved manually.
  * When copying the Node-RED configuration to clipboard (Exporting to update dashboard.json), ui_groups and spacers are not copied unless you copy all flows. Ui_groups will be recreated upon import, but spacers will not, leading to some unformatted dashboard elements.
