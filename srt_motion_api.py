'''

For use in the McGill University physics.
Written by Brandon Ruffolo in 2021-22.
Email: brandon.ruffolo@mcgill.ca
'''

import mcphysics as _mp
import numpy as _n
import time as _time


_serial_left_marker  = '<'
_serial_right_marker = '>'  

_debug_enabled       = True 


class srt_motion_api():
    """
    Commands-only object for interacting with an Arduino
    temperature controller.
    
    Parameters
    ----------
    port='COM3' : str
        Name of the port to connect to.
        
    baudrate=115200 : int
        Baud rate of the connection. Must match the instrument setting.
        
    timeout=3000 : number
        How long to wait for responses before giving up (ms). 
        
        
    """
    def __init__(self, port='COM3', baudrate=115200, timeout=3000):

        # Check for installed libraries
        if not _mp._serial:
            print('You need to install pyserial to use the Arduino based PID temperature controller.')
            self.simulation = True
            _debug('Simulation enabled.')

        # Assume everything will work for now
        else: self.simulation = False

        # If the port is "Simulation"
        if port=='Simulation': 
            self.simulation      = True
            self.simulation_mode = "OPEN_LOOP" 
            _debug('Simulation enabled.')

        # If we have all the libraries, try connecting.
        if not self.simulation:
            _debug("Attempting serial communication with following parameters:\nPort    : "+port+"\nBaudrate: "+str(baudrate)+" BPS\nTimeout : "+str(timeout)+" ms\n")
            
            try:
                # Create the instrument and ensure the settings are correct.
                self.serial = _mp._serial.Serial(port=port, baudrate=baudrate, timeout=timeout/1000)
                
                _debug("Serial communication to port %s enabled.\n"%port)
                

            # Something went wrong. Go into simulation mode.
            except Exception as e:
                print('Could not open connection to '+port+' at baudrate '+str(baudrate)+' BPS. Entering simulation mode.')
                print(e)
                self.serial = None
                self.simulation = True
        
        # Give the arduino time to run setup loop!
        _time.sleep(2)
                                
    def disconnect(self):
        """
        Disconnects.
        """
        if not self.simulation: 
            self.serial.close()
            _debug('Serial port closed.')

        
    def get_params(self):
        """
        Gets the current temperature in Celcius.
        """

        self.write('get_params')
        raw_params = self.read().split(',')
        
        # Convert to floating point numbers
        temperature = float(raw_params[0])
        a_x   = float(raw_params[1])
        a_y   = float(raw_params[2]) 
        a_z   = float(raw_params[3])
        r_x   = float(raw_params[4]) 
        r_y   = float(raw_params[5])
        r_z   = float(raw_params[6]) 
        b_x   = float(raw_params[7]) 
        b_y   = float(raw_params[8]) 
        b_z   = float(raw_params[9]) 
        
        return temperature, a_x, a_y, a_z, r_x, r_y, r_z, b_x, b_y, b_z
         
    def set_motors(self, m1_en=False, m2_en=False, m1_dir=False, m2_dir=False):
        self.write('set_motors,%d,%d,%d,%d'%(int(m1_en),int(m2_en),int(m1_dir),int(m2_dir)))
        return
         
    def write(self,raw_data):
        """
        Writes data to the serial line, formatted appropriately to be read by the arduino temperature controller.        
        
        Parameters
        ----------
        raw_data : str
            Raw data string to be sent to the arduino.
        
        Returns
        -------
        None.
        
        """
        encoded_data = (_serial_left_marker + raw_data + _serial_right_marker).encode()
        self.serial.write(encoded_data) 
    
    def read(self):
        """
        Reads data from the serial line.
        
        Returns
        -------
        str
            Raw data string read from the serial line.
        """
        return self.serial.read_until(expected = '\r\n'.encode()).decode().strip('\r\n')
    

        
def _debug(*a):
    if _debug_enabled:
        s = []
        for x in a: s.append(str(x))
        print(', '.join(s))