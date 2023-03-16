#!/usr/bin/python3
# bss_user_interface_psg.py

import PySimpleGUI
import os
import signal
import subprocess

class metadata:
    clicked = False

_buttonTest = PySimpleGUI.Button(button_text="TEST", tooltip="Run TurtleSim", enable_events=True, size=(10,3), key="_buttonTest")

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
    PySimpleGUI.theme('dark')
    PySimpleGUI.set_options(font = 'Calibri 20', button_element_size=(6,10))

    layout = [
        [_buttonTest],
        [PySimpleGUI.Text("Status: "), _buttonStart, _buttonStop], 
        [_buttonCounterReturn, _buttonShelfTopReturn, _buttonShelfBottomReturn],
        [_buttonCounterWithdraw, _buttonShelfTopWithdraw, _buttonShelfBottomWithdraw],
    ]

    # Create the window
    window = PySimpleGUI.Window(title="BSS User Interface", layout=layout)
    
    # ROS2 Run TurtleSim
    turtlesim = ["ros2", "run", "turtlesim", "turtlesim_node"]
    
    # ROS2 Launch UR10 MoveIt 2
    ur10launch = ["ros2", "launch", "ur_bringup", "ur10.launch.py"]
    ur10moveit = ["ros2", "launch", "ur_bringup", "ur_moveit.launch.py"]
    
    # ROS2 Launch Send Waypoints
    counter_return = ["ros2", "launch", "hello_moveit_ur", "counter_return_launch.py"]
    counter_withdraw = ["ros2", "launch", "hello_moveit_ur", "counter_withdraw_launch.py"]
    shelf_top_return = ["ros2", "launch", "hello_moveit_ur", "shelf_top_return_launch.py"]
    shelf_top_withdraw = ["ros2", "launch", "hello_moveit_ur", "shelf_top_withdraw_launch.py"]
    shelf_bottom_return = ["ros2", "launch", "hello_moveit_ur", "shelf_bottom_return_launch.py"]
    shelf_bottom_withdraw = ["ros2", "launch", "hello_moveit_ur", "shelf_bottom_withdraw_launch.py"]

    # Create an event loop
    while True:
        event, values = window.read()
        
        if event == "_buttonTest":
            _turtlesim = subprocess.Popen(counter_return, start_new_session=True)
            PySimpleGUI.popup_ok("TurtleSim is Running!", title="BUSY!!!", auto_close=True, auto_close_duration=20, non_blocking=False, keep_on_top=True)
            _turtlesim.send_signal(signal.SIGINT)
            _turtlesim.terminate()
            _turtlesim.kill()
        
        if event == "_buttonStart":
            _buttonStart.update(disabled=True)
            _buttonStart.metadata.clicked = True
            _buttonStop.update(disabled=False)
            _buttonStop.metadata.clicked = False
            
            # os.system("ros2 launch ur10_ros2_moveit2 ur10_interface.launch.py")

            _ur10 = subprocess.Popen(
                ur10launch,
                # stdout=subprocess.PIPE,
                # shell=True,
                start_new_session=True,
            )
            
            _rviz = subprocess.Popen(
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
            
            
        if event == "_buttonCounterReturn":
            if _buttonStart.metadata.clicked == True:
                disableAllButton()
                
                _counterReturn = subprocess.Popen(counter_return, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, start_new_session=True)
                PySimpleGUI.popup_ok("Returning Book!!", title="BUSY!!!", auto_close=True, auto_close_duration=20, non_blocking=False, keep_on_top=True)
                _counterReturn.send_signal(signal.SIGINT)
                _counterReturn.terminate()
                
                enableAllButton()
                
            else:
                PySimpleGUI.popup_ok("System is Off!!", title="ERROR!!!", auto_close=True, auto_close_duration=10, non_blocking=False, keep_on_top=True)
                
        if event == "_buttonCounterWithdraw":
            if _buttonStart.metadata.clicked == True:
                disableAllButton()
                
                _counterWithdraw = subprocess.Popen(counter_withdraw, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, start_new_session=True)
                PySimpleGUI.popup_ok("Withdrawing Book!!", title="BUSY!!!", auto_close=True, auto_close_duration=20, non_blocking=False, keep_on_top=True)
                _counterWithdraw.send_signal(signal.SIGINT)
                _counterWithdraw.terminate()
                
                enableAllButton()
                
            else:
                PySimpleGUI.popup_ok("System is Off!!", title="ERROR!!!", auto_close=True, auto_close_duration=10, non_blocking=False, keep_on_top=True)
            
        if event == "_buttonShelfTopReturn":
            if _buttonStart.metadata.clicked == True:
                disableAllButton()
                
                _shelfTopReturn = subprocess.Popen(shelf_top_return, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, start_new_session=True)
                PySimpleGUI.popup_ok("Returning Book!!", title="BUSY!!!", auto_close=True, auto_close_duration=20, non_blocking=False, keep_on_top=True)
                _shelfTopReturn.send_signal(signal.SIGINT)
                _shelfTopReturn.terminate()
                
                enableAllButton()
                
            else:
                PySimpleGUI.popup_ok("System is Off!!", title="ERROR!!!", auto_close=True, auto_close_duration=10, non_blocking=False, keep_on_top=True)
                
        if event == "_buttonShelfTopWithdraw":
            if _buttonStart.metadata.clicked == True:
                disableAllButton()
                
                _shelfTopWithdraw = subprocess.Popen(shelf_top_withdraw, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, start_new_session=True)
                PySimpleGUI.popup_ok("Withdrawing Book!!", title="BUSY!!!", auto_close=True, auto_close_duration=20, non_blocking=False, keep_on_top=True)
                _shelfTopWithdraw.send_signal(signal.SIGINT)
                _shelfTopWithdraw.terminate()
                
                enableAllButton()
                
            else:
                PySimpleGUI.popup_ok("System is Off!!", title="ERROR!!!", auto_close=True, auto_close_duration=10, non_blocking=False, keep_on_top=True)
            
        if event == "_buttonShelfBottomReturn":
            if _buttonStart.metadata.clicked == True:
                disableAllButton()
                
                _shelfBottomReturn = subprocess.Popen(shelf_bottom_return, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, start_new_session=True)
                PySimpleGUI.popup_ok("Returning Book!!", title="BUSY!!!", auto_close=True, auto_close_duration=20, non_blocking=False, keep_on_top=True)
                _shelfBottomReturn.send_signal(signal.SIGINT)
                _shelfBottomReturn.terminate()
                
                enableAllButton()
                
            else:
                PySimpleGUI.popup_ok("System is Off!!", title="ERROR!!!", auto_close=True, auto_close_duration=10, non_blocking=False, keep_on_top=True)
            
        if event == "_buttonShelfBottomWithdraw":
            if _buttonStart.metadata.clicked == True:
                disableAllButton()
                
                _shelfBottomWithdraw = subprocess.Popen(shelf_bottom_withdraw, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, start_new_session=True)
                PySimpleGUI.popup_ok("Withdrawing Book!!", title="BUSY!!!", auto_close=True, auto_close_duration=20, non_blocking=False, keep_on_top=True)
                _shelfBottomWithdraw.send_signal(signal.SIGINT)
                _shelfBottomWithdraw.terminate()
                
                enableAllButton()
                
            else:
                PySimpleGUI.popup_ok("System is Off!!", title="ERROR!!!", auto_close=True, auto_close_duration=10, non_blocking=False, keep_on_top=True)
        
        # End program if user closes window
        if event == PySimpleGUI.WIN_CLOSED or event == "EXIT":
            break
    
    # Interrupt the ROS2 windows
    _rviz.send_signal(signal.SIGINT)
    _ur10.send_signal(signal.SIGINT)
    
    # Terminate the ROS2 windows
    _rviz.terminate()
    _ur10.terminate()
    
    # Kill the ROS2 windows
    _rviz.kill()
    _ur10.kill()

    # Close window before exiting
    window.close()
    
    return

if __name__ == "__main__":
    main()