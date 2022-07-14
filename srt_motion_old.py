'''
For use in the McGill University physics department.
Written by Brandon Ruffolo in 2021-22.
Email: brandon.ruffolo@mcgill.ca
'''

import mcphysics   as _mp
import numpy       as _n
import spinmob.egg as _egg
import traceback   as _traceback
import spinmob     as _s
import time        as _time

from srt_motion_api import srt_motion_api

try: from serial.tools.list_ports import comports as _comports
except: _comports = None

_p = _traceback.print_last
_g = _egg.gui

RAD_TO_DEGREES = 180./_n.pi

_debug_enabled = True

# Dark theme
_s.settings['dark_theme_qt'] = True

## Fonts ##
style_1    = 'font-size: 17pt; font-weight: bold; color: ' +('lavender'              if _s.settings['dark_theme_qt'] else 'royalblue')
style_2    = 'font-size: 17pt; font-weight: bold; color: ' +('mediumspringgreen'  if _s.settings['dark_theme_qt'] else 'mediumspringgreen')
style_3    = 'font-size: 14pt; font-weight: bold; color: ' +('lightcoral'         if _s.settings['dark_theme_qt'] else 'lightcoral')
style_4    = 'font-size: 14pt; font-weight: bold; color: ' +('paleturquoise'      if _s.settings['dark_theme_qt'] else 'lightcoral')
style_5    = 'font-size: 14pt; font-weight: bold; color: ' +('paleturquoise'      if _s.settings['dark_theme_qt'] else 'lightcoral')
style_6    = 'font-size: 14pt; font-weight: bold; color: ' +('mediumspringgreen'  if _s.settings['dark_theme_qt'] else 'mediumspringgreen')



class rtd_monitor(_g.BaseObject):
    """
    Graphical interface for the Arduino based PID temperature controller.
    
    Parameters
    ----------
    name='Arduino_PID' : str
        Unique name to give this instance, so that its settings will not
        collide with other egg objects.
    
    temperature_limit=80 : float
        Upper limit on the temperature setpoint (C).
    
    show=True : bool
        Whether to show the window after creating.
        
    block=False : bool
        Whether to block the console when showing the window.
        
    window_size=[1,1] : list
        Dimensions of the window.
    """

    def __init__(self, name='Arduino_PID', api_class = srt_motion_api, show=True, block=False, window_size=None):
        
        if not _mp._serial: _s._warn('You need to install pyserial to use the Arduino based PID temperature controller.')
        
        # Remebmer the name.
        self.name = name

        # Checks periodically for the last exception
        self.timer_exceptions = _g.TimerExceptions()
        self.timer_exceptions.signal_new_exception.connect(self._new_exception)

        # Where the actual api will live after we connect.
        self.api = None
        self._api_class = api_class

        # Create GUI window
        self.window   = _g.Window(self.name, size = [400,300], autosettings_path=name+'.window',event_close = self._window_close)
        
       # Get all the available ports
        self._ports = [] # Actual port names for connecting
        ports       = [] # Pretty port names for combo box
        
        default_port = None
        
        if _comports:
            for inx, p in enumerate(_comports()):
                self._ports.append(p.device)
                ports      .append(p.description)
                
                if 'Arduino' in p.description:
                    default_port = inx
                    

        # Append simulation port
        ports      .append('Simulation')
        self._ports.append('Simulation')
        
        # Append refresh port
        ports      .append('Refresh - Update Ports List')
        self._ports.append('Refresh - Update Ports List')
        
        # Populate the GUI window 
        self.populate_window(ports, default_port, show, block)
        
        # Create Timer for collecting data 
        self.timer = _g.Timer(interval_ms=500, single_shot=False)
        self.timer.signal_tick.connect(self._timer_tick)
        
        self.r_x_offset,self.r_y_offset, self.r_z_offset = 0,0,0

        # Show the GUI!
        self.window.show(block)
        
        
    def _timer_tick(self, *a):
        """
        Called whenever the timer ticks. 
        Updates all parameters and the plot and saves the latest data.
        
        """
        
        t = _time.time()-self.t0
        T, a_x, a_y, a_z, r_x, r_y, r_z, b_x, b_y, b_z = self.api.get_params()
        
        # Convert to degrees/s
        r_x, r_y, r_z = r_x*RAD_TO_DEGREES, r_y*RAD_TO_DEGREES, r_z*RAD_TO_DEGREES
        
        r_x -= self.r_x_offset
        r_y -= self.r_y_offset
        r_z -= self.r_z_offset
        
        self.number_temperature.set_value(T)
        
        self.ax.set_value(a_x)
        self.ay.set_value(a_y)
        self.az.set_value(a_z)
        self.acc_tot.set_value(_n.sqrt(a_x**2+a_y**2 +a_z**2))
        
        self.rx.set_value(r_x)
        self.ry.set_value(r_y)
        self.rz.set_value(r_z)
        self.gyro_tot.set_value(_n.sqrt(r_x**2+r_y**2 +r_z**2))
        
        self.bx.set_value(b_x)
        self.by.set_value(b_y)
        self.bz.set_value(b_z)
        self.mag_tot.set_value(_n.sqrt(b_x**2+b_y**2 +b_z**2))
                
        # Append this to the databox
        self.plot_acceleration.append_row([t, a_x, a_y, a_z], ckeys=['Time (s)', 'a_x [g]','a_y [g]','a_z [g]'])
        
        self.plot_rotation.append_row([t, r_x, r_y, r_z], ckeys=['Time (s)', 'r_x [dps]','r_y [dps]','r_z [dps]'])

        #self.plot_temperature.append_row([t, T], ckeys=['Time (s)', 'Temperature (°C)'])
        
        self.plot_bfield.append_row([t, b_x, b_y, b_z], ckeys=['Time (s)', 'B_x [Gauss]','B_y [Gauss]','B_z [Gauss]'])


        # Update the plot
        self.plot_acceleration.plot()  
        self.plot_rotation.plot()
        #self.plot_temperature.plot()
        self.plot_bfield.plot()
        

        # Update GUI
        self.window.process_events()
        

    def _button_connect_toggled(self, *a):
        """
        Called when the connect button is toggled in the GUI. 
        Creates the API and imports data from the arduino.
        """
        if self._api_class is None:
            raise Exception('You need to specify an api_class when creating a serial GUI object.')

        # If we checked it, open the connection and start the timer.
        if self.button_connect.is_checked():
            port = self.get_selected_port()
            self.api = self._api_class(
                    port=port,
                    baudrate=int(self.combo_baudrates.get_text()),
                    timeout=self.number_timeout.get_value())
            
            # If we're in simulation mode
            if self.api.simulation:
                self.label_status.set_text('*** Simulation ***')
                self.label_status.set_colors('pink' if _s.settings['dark_theme_qt'] else 'red')
                self.button_connect.set_colors(background='pink')
            else:
                # Display connection status to user
                self.label_status.set_text('Connected').set_colors('white' if _s.settings['dark_theme_qt'] else 'blue')


            # Record the time if it's not already there.
            if self.t0 is None: self.t0 = _time.time()

            # Enable the grid
            self.grid_bot.enable()

            # Disable other controls
            self.combo_baudrates.disable()
            self.combo_ports.disable()
            self.number_timeout.disable()
            
            # Change the button color to indicate we are connected
            self.button_connect.set_colors(background = 'blue')
            
            
            self.timer.start()

        # Otherwise, shut it down
        else:
            
            # Disconnect the API
            self.api.disconnect()
            
            #
            self.label_status.set_text('')
            self.button_connect.set_colors()
            
            # Disable plotting
            self.grid_bot.disable()

            # Re-enable other controls
            self.combo_baudrates.enable()
            self.combo_ports.enable()
            self.number_timeout.enable()
            
            # Display connection status to user
            self.label_status.set_text('Disconnected').set_colors('white' if _s.settings['dark_theme_qt'] else 'blue')
            
            self.timer.stop()
    
    def _ports_changed(self):
        """
        Refreshes the list of availible serial ports in the GUI.

        """
        if self.get_selected_port() == 'Refresh - Update Ports List':
            
            len_ports = len(self.combo_ports.get_all_items())
            
            # Clear existing ports
            if(len_ports > 1): # Stop recursion!
                for n in range(len_ports):
                    self.combo_ports.remove_item(0)
            else:
                return
                self.combo_ports.remove_item(0)
                 
            self._ports = [] # Actual port names for connecting
            ports       = [] # Pretty port names for combo box
                
            default_port = 0
             
            # Get all the available ports
            if _comports:
                for inx, p in enumerate(_comports()):
                    self._ports.append(p.device)
                    ports      .append(p.description)
                    
                    if 'Arduino' in p.description:
                        default_port = inx
                        
            # Append simulation port
            ports      .append('Simulation')
            self._ports.append('Simulation')
            
            # Append refresh port
            ports      .append('Refresh - Update Ports List')
            self._ports.append('Refresh - Update Ports List')
             
            # Add the new list of ports
            for item in ports:
                self.combo_ports.add_item(item)
             
            # Set the new default port
            self.combo_ports.set_index(default_port)
        
    def _new_exception(self, a):
        """
        Just updates the status with the exception.
        """
        self.label_message(str(a)).set_colors('red')

    def _window_close(self):
        """
        Disconnects. When you close the window.
        """
        print('Window closed but not destroyed. Use show() to bring it back.')
        if self.button_connect():
            print('  Disconnecting...')
            self.button_connect(False)

    def get_selected_port(self):
        """
        Returns the actual port string from the combo box.
        """
        return self._ports[self.combo_ports.get_index()]
    
    def populate_window(self, ports, default_port, show, block):
        """
        All the GUI window populating shoved into one convenient (and out of site) place. 
        """
        
        ## Create partitions in the GUI window ##
        
        self.grid_top = self.window.place_object(
            _g.GridLayout(margins=False),0,0,alignment=1,column_span=2)
        
 
        self.window.new_autorow()
        self.grid_1= self.window.place_object(_g.GridLayout(margins=False),0,3,alignment=1,column_span=1)
        
        self.window.new_autorow()
        
        self.grid_location = self.grid_1.place_object(_g.GridLayout(margins=False),alignment=0)
        self.grid_1.new_autorow()
        self.grid_position = self.grid_1.place_object(_g.GridLayout(margins=False),alignment=0)
        self.grid_buttons = self.grid_1.place_object(_g.GridLayout(margins=False),alignment=0)
        
        #self.grid_1.set_column_stretch()
 
        self.window.new_autorow()
        self.grid_bot = self.window.place_object(_g.GridLayout(margins=False),row=4,column=5,alignment=0)
        self.grid_bot1 = self.window.place_object(_g.GridLayout(margins=False),row=4,column=6,alignment=0)
        ## Add widgets (buttons, selectors, ect..) to the GUI ##
        
        # Add port selector to GUI 
        self._label_port = self.grid_top.add(_g.Label('Port:'))
        self.combo_ports = self.grid_top.add(_g.ComboBox(ports, default_index = default_port, autosettings_path=self.name+'.combo_ports'))
        self.combo_ports.signal_changed.connect(self._ports_changed)
        
        # Add BAUD selector to GUI 
        self.grid_top.add(_g.Label('Baud:'))
        self.combo_baudrates = self.grid_top.add(
            _g.ComboBox(['1200','2400','4800', '9600', '19200', '38400', '57600', '115200'],default_index=7,autosettings_path=
                        self.name+'.combo_baudrates'))

        # Add Timeout selector to GUI 
        self.grid_top.add(_g.Label('Timeout:'))
        self.number_timeout = self.grid_top.add(
            _g.NumberBox(500, dec=True, bounds=(1, None), suffix=' ms',
                         tip='How long to wait for an answer before giving up (ms).', autosettings_path=self.name+'.number_timeout')).set_width(100)

        # Add a button to connect to serial port to GUI
        self.button_connect  = self.grid_top.add(_g.Button('Connect', checkable=True,tip='Connect to the selected serial port.'))
        self.button_connect.signal_toggled.connect(self._button_connect_toggled)
        
        # Status
        self.label_status = self.grid_top.add(_g.Label(''))

        # Error
        self.grid_top.new_autorow()
        self.label_message = self.grid_top.add(_g.Label(''), column_span=1).set_colors('pink' if _s.settings['dark_theme_qt'] else 'red')
        
        # By default the bottom grid is disabled
        self.grid_bot.disable()


        # Other data
        self.t0 = None

        # Run the base object stuff and autoload settings
        _g.BaseObject.__init__(self, autosettings_path=self.name)

        # Show the window.
        if show: self.window.show(block)
          
        
        # Data box width
        box_width = 175
 
        self.tabs = self.grid_bot.add(_g.TabArea(self.name+'.tabs'), alignment=0)
        self.tabs1 = self.grid_bot1.add(_g.TabArea(self.name+'.tabs'), alignment=0)
        
        self.tab_motors       = self.tabs.add_tab('Sensor Monitor')
        self.tab_accelration  = self.tabs.add_tab('Acceleration')
        self.tab_rotation     = self.tabs.add_tab('Rotation')
        self.tab_temperature  = self.tabs.add_tab('Temperature')
        self.tab_bfield       = self.tabs1.add_tab('Magnetic Field')
        
        
        ## Make the plotter ##
        self.plot_acceleration = self.tab_accelration.add(_g.DataboxPlot(
            file_type='*.csv',
            autosettings_path=self.name+'.plot_acceleration',
            delimiter=',', show_logger=True), alignment=0, column_span=10)
        
        self.plot_rotation = self.tab_rotation.add(_g.DataboxPlot(
            file_type='*.csv',
            autosettings_path=self.name+'.plot_rotation',
            delimiter=',', show_logger=True), alignment=0, column_span=10)
        
        self.plot_temperature = self.tab_temperature.add(_g.DataboxPlot(
            file_type='*.csv',
            autosettings_path=self.name+'.plot_temperature',
            delimiter=',', show_logger=True), alignment=0, column_span=10)
        
        self.plot_bfield = self.tab_bfield.add(_g.DataboxPlot(
            file_type='*.csv',
            autosettings_path=self.name+'.plot_bfield',
            delimiter=',', show_logger=True), alignment=0, column_span=10)
        
        #self.tab_motors.
        
        self.grid_controls = self.tab_motors.place_object(_g.GridLayout())
        #self.grid_buttons = self.grid_params.place_object(_g.GridLayout())
        
        self.grid_acc1 = self.grid_controls.place_object(_g.GridLayout())
        self.grid_controls.new_autorow()
        self.grid_acc2 = self.grid_controls.place_object(_g.GridLayout())
        self.grid_controls.new_autorow()
        self.grid_gyro1 = self.grid_controls.place_object(_g.GridLayout())
        self.grid_controls.new_autorow()
        self.grid_gyro2 = self.grid_controls.place_object(_g.GridLayout())
        self.grid_controls.new_autorow()
        self.grid_mag1 = self.grid_controls.place_object(_g.GridLayout())
        self.grid_controls.new_autorow()
        self.grid_mag2 = self.grid_controls.place_object(_g.GridLayout())
        self.grid_controls.new_autorow()
        self.grid_temp = self.grid_controls.place_object(_g.GridLayout())
        
        
        
        self.grid_location.add(_g.Label('Location:'), alignment=1).set_style(style_1)
         
        self.grid_position.add(_g.Label('α:'), alignment=1, column = 0).set_style(style_5)
        self.number_ra = self.grid_position.add(_g.TextBox(
            '00h 00m 00s', tip='Right Ascension.'), alignment=1, column = 1).set_width(200).disable().set_style(style_6)
    
        self.grid_position.add(_g.Label('Altitude:'), alignment=1, column = 2).set_style(style_5)
        self.number_dec = self.grid_position.add(_g.NumberBox(
            0.0, suffix='°', tip='Declination.'), alignment=1, column = 3).set_width(125).disable().set_style(style_6)
        
        self.grid_position.new_autorow()
        
        self.grid_position.add(_g.Label('δ:'), alignment=1, column = 0).set_style(style_5)
        self.number_dec = self.grid_position.add(_g.NumberBox(
            0.0, suffix='°', tip='Declination.'), alignment=1, column = 1).set_width(125).disable().set_style(style_6)
        
        self.grid_position.add(_g.Label('Azimuth:'), alignment=1, column = 2).set_style(style_5)
        self.number_dec = self.grid_position.add(_g.NumberBox(
            0.0, suffix='°', tip='Declination.'), alignment=1, column = 3).set_width(125).disable().set_style(style_6)

        self.a1 = self.grid_buttons.add(_g.Button('↖', checkable=True).set_style(style_1),alignment=0, row_span=1)
        self.a2 = self.grid_buttons.add(_g.Button('↑', checkable=True).set_style(style_1),alignment=0, row_span=1)
        self.a3 = self.grid_buttons.add(_g.Button('↗', checkable=True).set_style(style_1),alignment=0, column_span=1)
        
        self.grid_buttons.new_autorow()
        
        self.b1 = self.grid_buttons.add(_g.Button('←', checkable=True).set_style(style_1),alignment=0, column_span=1)
        self.b2 = self.grid_buttons.add(_g.Button('■').set_style(style_1),alignment=0, column_span=1)
        self.b3 = self.grid_buttons.add(_g.Button('→', checkable=True).set_style(style_1),alignment=0, column_span=1)

        self.grid_buttons.new_autorow()
        
        self.c1 = self.grid_buttons.add(_g.Button('↙', checkable=True).set_style(style_1),alignment=0, column_span=1)
        self.c2 = self.grid_buttons.add(_g.Button('↓', checkable=True).set_style(style_1),alignment=0, column_span=1)
        self.c3 = self.grid_buttons.add(_g.Button('↘', checkable=True).set_style(style_1),alignment=0, column_span=1)
        
        #self.grid_buttons.new_autorow()
        
        
        self.grid_buttons.set_row_stretch(0,5)
        self.grid_buttons.set_row_stretch(1,0)
        self.grid_buttons.set_row_stretch(2,0)
        #self.grid_buttons.set_row_stretch(3,0)
        
        self.a1.signal_toggled.connect(self._button_up_left_toggled)
        self.a2.signal_toggled.connect(self._button_up_toggled)
        self.a3.signal_toggled.connect(self._button_up_right_toggled)
        
        self.b1.signal_toggled.connect(self._button_left_toggled)
        self.b3.signal_toggled.connect(self._button_right_toggled)
        
        self.c1.signal_toggled.connect(self._button_down_left_toggled)
        self.c2.signal_toggled.connect(self._button_down_toggled)
        self.c3.signal_toggled.connect(self._button_down_right_toggled)
        
        self.grid_acc1.add(_g.Label('Acceleration:'), alignment=1).set_style(style_1)
        self.acc_tot = self.grid_acc1.add(_g.NumberBox(suffix='m/s', decimals = 4), alignment=1).set_width(200).set_style(style_5)
        self.grid_acc2.add(_g.Label('x:'), alignment=1).set_style(style_5)
        self.ax = self.grid_acc2.add(_g.NumberBox(suffix='m/s'), alignment=1).set_width(200).set_style(style_6)
        self.grid_acc2.add(_g.Label('y:'), alignment=1).set_style(style_5)
        self.ay = self.grid_acc2.add(_g.NumberBox(suffix='m/s'), alignment=1).set_width(200).set_style(style_6)
        self.grid_acc2.add(_g.Label('z:'), alignment=1).set_style(style_5)
        self.az = self.grid_acc2.add(_g.NumberBox(suffix='m/s'), alignment=1).set_width(200).set_style(style_6)
        
        self.grid_gyro1.add(_g.Label('Rotation:'), alignment=1).set_style(style_1)
        self.gyro_tot = self.grid_gyro1.add(_g.NumberBox(suffix='°/s', dec=True,minStep=.001,decimals=4), alignment=1).set_width(200).set_style(style_5)
        self.button_zero_gyro = self.grid_gyro1.add(_g.Button('Zero'))
        self.grid_gyro2.add(_g.Label('x:'), alignment=1).set_style(style_5)
        self.rx = self.grid_gyro2.add(_g.NumberBox(suffix='°/s',dec=True,minStep=.001,decimals=4), alignment=1).set_width(200).set_style(style_6)
        self.grid_gyro2.add(_g.Label('y:'), alignment=1).set_style(style_5)
        self.ry = self.grid_gyro2.add(_g.NumberBox(suffix='°/s',dec=True,minStep=.001,decimals=4), alignment=1).set_width(200).set_style(style_6)
        self.grid_gyro2.add(_g.Label('z:'), alignment=1).set_style(style_5)
        self.rz = self.grid_gyro2.add(_g.NumberBox(suffix='°/s',decimals =4), alignment=1).set_width(200).set_style(style_6)
        
        self.button_zero_gyro.signal_clicked.connect(self._button_zero_gyro_clicked)
        
        
        self.grid_mag1.add(_g.Label('Magnetic field:'), alignment=1).set_style(style_1)
        self.mag_tot = self.grid_mag1.add(_g.NumberBox(suffix='uT', decimals = 4), alignment=1).set_width(200).set_style(style_5)
        self.grid_mag2.add(_g.Label('x:'), alignment=1).set_style(style_5)
        self.bx = self.grid_mag2.add(_g.NumberBox(suffix='uT'), alignment=1).set_width(200).set_style(style_6)
        self.grid_mag2.add(_g.Label('y:'), alignment=1).set_style(style_5)
        self.by = self.grid_mag2.add(_g.NumberBox(suffix='uT'), alignment=1).set_width(200).set_style(style_6)
        self.grid_mag2.add(_g.Label('z:'), alignment=1).set_style(style_5)
        self.bz = self.grid_mag2.add(_g.NumberBox(suffix='uT'), alignment=1).set_width(200).set_style(style_6)
        
        # Tab for monitoring measured temperature
        self.grid_temp.add(_g.Label('Temperature:'), alignment=2).set_style(style_1)
        self.number_temperature = self.grid_temp.add(_g.NumberBox(
            value=-273.16, suffix='°C', tip='Last recorded temperature value.'), alignment=2).set_width(250).disable().set_style(style_6)
        
        
        '''
        self.grid_fault.add(_g.Label('Azimuth:').set_style(style_5))
        self.grid_fault.add(_g.TextBox('Idle').set_width(150).set_style(style_6), alignment=0)
        
        self.grid_fault.new_autorow()
        self.grid_fault.add(_g.Label('Altitude:').set_style(style_5))
        self.grid_fault.add(_g.TextBox('Idle').set_width(150).set_style(style_6), alignment=0)'''
        
        self.motor_buttons = [self.a1,self.a2,self.a3,self.b1,self.b3,self.c1,self.c2,self.c3]
        
        # Bottom log file controls
        self.grid_bot.new_autorow()
        
    def _button_zero_gyro_clicked(self):
        r_xs = []
        r_ys = []
        r_zs = []
        

        
        
        for i in range(500):
            T, a_x, a_y, a_z, r_x, r_y, r_z, b_x, b_y, b_z = self.api.get_params()
                
            # Convert to degrees/s
            r_x, r_y, r_z = r_x*RAD_TO_DEGREES, r_y*RAD_TO_DEGREES, r_z*RAD_TO_DEGREES
            
            r_xs.append(r_x)
            r_ys.append(r_y)
            r_zs.append(r_z)
            
            _time.sleep(.01)
            
        self.r_x_offset = _n.average(r_xs)
        self.r_y_offset = _n.average(r_ys)
        self.r_z_offset = _n.average(r_zs)
        
        
    def _button_up_left_toggled(self):
        
        if self.a1.is_checked():
            for button in self.motor_buttons:
                if(button.is_checked() and button != self.a1):
                    button.click()
                    
            self.a1.set_colors(background = 'mediumspringgreen')
            self.api.set_motors(True, True, True, True)
        
        else:
            self.a1.set_colors(background = None)
            self.api.set_motors()
        
    def _button_up_toggled(self):
        
        if self.a2.is_checked():
            for button in self.motor_buttons:
                if(button.is_checked() and button != self.a2):
                    button.click()
                    
            self.a2.set_colors(background = 'mediumspringgreen')
            self.api.set_motors(True, False, True, False)
        
        else:
            self.a2.set_colors(background = None)
            self.api.set_motors() 
            
    def _button_up_right_toggled(self):
        
        if self.a3.is_checked():
            for button in self.motor_buttons:
                if(button.is_checked() and button != self.a3):
                    button.click()
                    
            self.a3.set_colors(background = 'mediumspringgreen')
            self.api.set_motors(True, True, True, False)

        else:
            self.a3.set_colors(background = None)
            self.api.set_motors() 
            
    def _button_left_toggled(self):
        
        if self.b1.is_checked():
            for button in self.motor_buttons:
                if(button.is_checked() and button != self.b1):
                    button.click()
                
            self.b1.set_colors(background = 'mediumspringgreen')
            self.api.set_motors(False, True, False, True)
        
        else:
            self.b1.set_colors(background = None)
            self.api.set_motors() 

    def _button_right_toggled(self):
        
        if self.b3.is_checked():
            
            for button in self.motor_buttons:
                if(button.is_checked() and button != self.b3):
                    button.click()
                    
            self.b3.set_colors(background = 'mediumspringgreen')
            self.api.set_motors(False, True, False, False)
            
        else:
            self.b3.set_colors(background = None)
            self.api.set_motors()  
            
    def _button_down_left_toggled(self):
        
        if self.c1.is_checked():
            
            for button in self.motor_buttons:
                if(button.is_checked() and button != self.c1):
                    button.click()
                    
            self.c1.set_colors(background = 'mediumspringgreen')
            self.api.set_motors(True, True, False, True)
            
        else:
            self.c1.set_colors(background = None)
            self.api.set_motors()             
        
    def _button_down_toggled(self):
        
        if self.c2.is_checked():
            
            for button in self.motor_buttons:
                if(button.is_checked() and button != self.c2):
                    button.click()
                    
            self.c2.set_colors(background = 'mediumspringgreen')
            self.api.set_motors(True, False, False, False)
            
        else:
            self.c2.set_colors(background = None)
            self.api.set_motors()        
            
    def _button_down_right_toggled(self):
        
        if self.c3.is_checked():
            
            for button in self.motor_buttons:
                if(button.is_checked() and button != self.c3):
                    button.click()
                    
            self.c3.set_colors(background = 'mediumspringgreen')
            self.api.set_motors(True, True, False, False)
            
        else:
            self.c3.set_colors(background = None)
            self.api.set_motors()        
            
            

def _debug(*a):
    if _debug_enabled:
        s = []
        for x in a: s.append(str(x))
        print(', '.join(s))

if __name__ == '__main__':
    _egg.clear_egg_settings()
    self = rtd_monitor()