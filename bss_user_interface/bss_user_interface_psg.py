# bss_user_interface_psg.py

import PySimpleGUI

class metadata:
    clicked = False

def main():
    PySimpleGUI.theme('light')
    PySimpleGUI.set_options(font = 'Calibri 20', button_element_size=(6,10))

    _buttonStart = PySimpleGUI.Button(button_text="START", tooltip="Startup and run", disabled=False, enable_events=True, size=(10,3), key="_buttonStart", metadata=metadata())
    _buttonStop = PySimpleGUI.Button(button_text="STOP", tooltip="Shutdown", disabled=True, enable_events=True, size=(10,3), key="_buttonStop", metadata=metadata())

    _buttonReturn = PySimpleGUI.Button(button_text="RETURN", tooltip="Want to return a book?", enable_events=True, size=(30,10), key="_buttonReturn", metadata=metadata())
    _buttonWithdraw = PySimpleGUI.Button(button_text="BORROW", tooltip="Want to borrow a book?", enable_events=True, size=(30,10), key="_buttonWithdraw", metadata=metadata())

    layout = [
        [PySimpleGUI.Text("Status: "), _buttonStart, _buttonStop], 
        [_buttonReturn, _buttonWithdraw],
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
        
        if event == "_buttonStop":
            _buttonStart.update(disabled=False)
            _buttonStart.metadata.clicked = False
            _buttonStop.update(disabled=True)
            _buttonStop.metadata.clicked = True
            
        if event == "_buttonWithdraw":
            if _buttonStart.metadata.clicked == True:
                _buttonWithdraw.update(disabled=True)
                _buttonReturn.update(disabled=True)
                PySimpleGUI.popup("Withdrawing Book!!", title="BUSY!!!", auto_close=True, auto_close_duration=20, keep_on_top=True)
                _buttonWithdraw.update(disabled=False)
                _buttonReturn.update(disabled=False)
            else:
                PySimpleGUI.popup("System is Off!!", title="ERROR!!!", auto_close=True, auto_close_duration=10, keep_on_top=True)
            
        if event == "_buttonReturn":
            if _buttonStart.metadata.clicked == True:
                _buttonWithdraw.update(disabled=True)
                _buttonReturn.update(disabled=True)
                PySimpleGUI.popup("Returning Book!!", title="BUSY!!!", auto_close=True, auto_close_duration=20, keep_on_top=True)
                _buttonWithdraw.update(disabled=False)
                _buttonReturn.update(disabled=False)
            else:
                PySimpleGUI.popup("System is Off!!", title="ERROR!!!", auto_close=True, auto_close_duration=10, keep_on_top=True)
        
        # End program if user closes window
        if event == PySimpleGUI.WIN_CLOSED or event == "EXIT":
            break

    # Close window before exiting
    window.close()

if __name__ == "__main__":
    main()