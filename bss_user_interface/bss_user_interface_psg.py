#!/usr/bin/python3
# bss_user_interface_psg.py

import PySimpleGUI
import os
import signal
import subprocess

class metadata:
    clicked = False

_buttonStart = PySimpleGUI.Button(button_text="START", tooltip="Startup and run", disabled=False, enable_events=True, size=(10,3), key="_buttonStart", metadata=metadata())
_buttonStop = PySimpleGUI.Button(button_text="STOP", tooltip="Shutdown", disabled=True, enable_events=True, size=(10,3), key="_buttonStop", metadata=metadata())

_buttonCounterReturn = PySimpleGUI.Button(button_text="RETURN COUNTER", tooltip="Want to return a book?", enable_events=True, size=(30,10), key="_buttonCounterReturn", metadata=metadata())
_buttonCounterWithdraw = PySimpleGUI.Button(button_text="BORROW COUNTER", tooltip="Want to borrow a book?", enable_events=True, size=(30,10), key="_buttonCounterWithdraw", metadata=metadata())

_buttonShelfTopReturn = PySimpleGUI.Button(button_text="RETURN SHELF TOP", tooltip="Want to return a book?", enable_events=True, size=(30,10), key="_buttonShelfTopReturn", metadata=metadata())
_buttonShelfTopWithdraw = PySimpleGUI.Button(button_text="BORROW SHELF TOP", tooltip="Want to borrow a book?", enable_events=True, size=(30,10), key="_buttonShelfTopWithdraw", metadata=metadata())

_buttonShelfBottomReturn = PySimpleGUI.Button(button_text="RETURN SHELF BOTTOM", tooltip="Want to return a book?", enable_events=True, size=(30,10), key="_buttonShelfBottomReturn", metadata=metadata())
_buttonShelfBottomWithdraw = PySimpleGUI.Button(button_text="BORROW SHELF BOTTOM", tooltip="Want to borrow a book?", enable_events=True, size=(30,10), key="_buttonShelfBottomWithdraw", metadata=metadata())

def disableAllButton():
    _buttonCounterReturn.update(disabled=True)
    _buttonCounterWithdraw.update(disabled=True)
    
    _buttonShelfTopReturn.update(disabled=True)
    _buttonShelfTopWithdraw.update(disabled=True)
    
    _buttonShelfBottomReturn.update(disabled=True)
    _buttonShelfBottomWithdraw.update(disabled=True)
    
    return
    
def enableAllButton():
    _buttonCounterReturn.update(disabled=False)
    _buttonCounterWithdraw.update(disabled=False)
    
    _buttonShelfTopReturn.update(disabled=False)
    _buttonShelfTopWithdraw.update(disabled=False)
    
    _buttonShelfBottomReturn.update(disabled=False)
    _buttonShelfBottomWithdraw.update(disabled=False)
    
    return

def main():
    PySimpleGUI.theme('light')
    PySimpleGUI.set_options(font = 'Calibri 20', button_element_size=(6,10))

    layout = [
        [PySimpleGUI.Text("Status: "), _buttonStart, _buttonStop], 
        [_buttonCounterReturn, _buttonShelfTopReturn, _buttonShelfBottomReturn],
        [_buttonCounterWithdraw, _buttonShelfTopWithdraw, _buttonShelfBottomWithdraw],
    ]

    # Create the window
    window = PySimpleGUI.Window(title="BSS User Interface", layout=layout)

    # Create an event loop
    while True:
        event, values = window.read()
        
        if event == "_buttonStart":
            _buttonStart.update(disabled=True)
            _buttonStart.metadata.clicked = True
            _buttonStop.update(disabled=False)
            _buttonStop.metadata.clicked = False
            
            # os.system("ros2 launch ur10_ros2_moveit2 ur10_interface.launch.py")
            
            ur10launch = ["ros2", "launch", "ur_bringup", "ur10.launch.py"]
            ur10moveit = ["ros2", "launch", "ur_bringup", "ur_moveit.launch.py"]

            ur10 = subprocess.Popen(
                ur10launch,
                # stdout=subprocess.PIPE,
                # shell=True,
                start_new_session=True,
            )
            
            rviz = subprocess.Popen(
                ur10moveit,
                # stdout=subprocess.PIPE,
                # shell=True,
                start_new_session=True,
            )
            
        if event == "_buttonStop":
            _buttonStart.update(disabled=False)
            _buttonStart.metadata.clicked = False
            _buttonStop.update(disabled=True)
            _buttonStop.metadata.clicked = True
            
            # Stop the process
            rviz.kill()
            ur10.kill()
            
            # Terminate process
            os.killpg(os.getpgid(rviz.pid), signal.SIGTERM)
            os.killpg(os.getpgid(ur10.pid), signal.SIGTERM)
            
        if event == "_buttonCounterReturn":
            if _buttonStart.metadata.clicked == True:
                disableAllButton()
                
                os.system("ros2 launch hello_moveit_ur counter_return_launch.py")
                PySimpleGUI.popup("Returning Book!!", title="BUSY!!!", auto_close=True, auto_close_duration=20, keep_on_top=True)
                
                enableAllButton()
                
            else:
                PySimpleGUI.popup("System is Off!!", title="ERROR!!!", auto_close=True, auto_close_duration=10, keep_on_top=True)
                
        if event == "_buttonCounterWithdraw":
            if _buttonStart.metadata.clicked == True:
                disableAllButton()
                
                os.system("ros2 launch hello_moveit_ur counter_withdraw_launch.py")
                PySimpleGUI.popup("Withdrawing Book!!", title="BUSY!!!", auto_close=True, auto_close_duration=20, keep_on_top=True)
                
                enableAllButton()
                
            else:
                PySimpleGUI.popup("System is Off!!", title="ERROR!!!", auto_close=True, auto_close_duration=10, keep_on_top=True)
            
        if event == "_buttonShelfTopReturn":
            if _buttonStart.metadata.clicked == True:
                disableAllButton()
                
                os.system("ros2 launch hello_moveit_ur shelf_top_return_launch.py")
                PySimpleGUI.popup("Returning Book!!", title="BUSY!!!", auto_close=True, auto_close_duration=20, keep_on_top=True)
                
                enableAllButton()
                
            else:
                PySimpleGUI.popup("System is Off!!", title="ERROR!!!", auto_close=True, auto_close_duration=10, keep_on_top=True)
                
        if event == "_buttonShelfTopWithdraw":
            if _buttonStart.metadata.clicked == True:
                disableAllButton()
                
                os.system("ros2 launch hello_moveit_ur shelf_top_withdraw_launch.py")
                PySimpleGUI.popup("Withdrawing Book!!", title="BUSY!!!", auto_close=True, auto_close_duration=20, keep_on_top=True)
                
                enableAllButton()
                
            else:
                PySimpleGUI.popup("System is Off!!", title="ERROR!!!", auto_close=True, auto_close_duration=10, keep_on_top=True)
            
        if event == "_buttonShelfBottomReturn":
            if _buttonStart.metadata.clicked == True:
                disableAllButton()
                
                os.system("ros2 launch hello_moveit_ur shelf_bottom_return_launch.py")
                PySimpleGUI.popup("Returning Book!!", title="BUSY!!!", auto_close=True, auto_close_duration=20, keep_on_top=True)
                
                enableAllButton()
                
            else:
                PySimpleGUI.popup("System is Off!!", title="ERROR!!!", auto_close=True, auto_close_duration=10, keep_on_top=True)
            
        if event == "_buttonShelfBottomWithdraw":
            if _buttonStart.metadata.clicked == True:
                disableAllButton()
                
                os.system("ros2 launch hello_moveit_ur shelf_bottom_withdraw_launch.py")
                PySimpleGUI.popup("Withdrawing Book!!", title="BUSY!!!", auto_close=True, auto_close_duration=20, keep_on_top=True)
                
                enableAllButton()
                
            else:
                PySimpleGUI.popup("System is Off!!", title="ERROR!!!", auto_close=True, auto_close_duration=10, keep_on_top=True)
        
        # End program if user closes window
        if event == PySimpleGUI.WIN_CLOSED or event == "EXIT":
            break

    # Close window before exiting
    window.close()
    
    return

if __name__ == "__main__":
    main()