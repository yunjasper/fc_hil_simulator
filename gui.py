"""
gui.py

GUI for simulator.

author: jasper yun
"""

import PySimpleGUI as sg
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, FigureCanvasAgg
from matplotlib.figure import Figure
from matplotlib import pyplot as plt
import pandas as pd

# local libraries
import rocket
import utils

FONT = 'Helvetica'
DO_LIVE_PLOTTING = False
sim_is_running = False


# sim configuration options tab
sim_config_text_size = (50, 1)
sim_config_input_size = (8, 1)
layout_sim_config_options = [
    [sg.Text('Simulator Configuration Options', font=(FONT, 15), justification='left')],
    # radio buttons
    [sg.Text('Target DUT: ', size=(15, 1)), sg.Radio('Hardware', 'target', size=(12, 1), key='radio_target_hw'), sg.Radio('Software', 'target', size=(12, 1), default=True, key='radio_target_sw')],
    [sg.Text('Thrust source: ', size=(15, 1)), sg.Radio('.rse file', 'thrust', size=(12, 1), default=True, key='radio_thrust_rse'), sg.Radio('Altus Metrum', 'thrust', size=(12, 1), key='radio_thrust_altos')],
    # checkboxes
    [sg.Checkbox('Mach dip emulation', size=(20, 1), default=True, key='box_mach_dip')],
    [sg.Checkbox('Noisy altitude data', size=(20, 1), default=True, key='box_noisy_altitude')],
    [sg.Checkbox('Send altitude instead of pressure', size=(30, 1), default=False, key='box_send_alt_pressure')],
    [sg.Text('Altitude data noise max amplitude (m): ', size=sim_config_text_size), sg.Input(focus=True, key='input_noisy_altitude_amplitude', default_text=str(utils.Settings.ALTITUDE_NOISE_MAX_AMPLITUDE_M), size=sim_config_input_size)],
    [sg.Text('Ground altitude (m): ', size=sim_config_text_size), sg.Input(focus=True, key='input_ground_altitude', default_text=str(utils.Settings.GROUND_ALTITUDE_M), size=sim_config_input_size)],

    # simulator time options
    [sg.Text('Simulator Timestep Options', font=(FONT, 15), justification='left')],
    [sg.Text('Simulation timestep (ms): ', size=sim_config_text_size), sg.Input(focus=True, key='input_sim_timestep', default_text=str(utils.Settings.SIMULATION_TIMESTEP_MS), size=sim_config_input_size)],
    [sg.Text('Hardware update timestep (ms): ', size=sim_config_text_size), sg.Input(focus=True, key='input_hw_update_timestep', default_text=str(utils.Settings.HARDWARE_UPDATE_TIMESTEP_MS), size=sim_config_input_size)],
    [sg.Text('Print update timestep (ms): ', size=sim_config_text_size), sg.Input(focus=True, key='input_print_timestep', default_text=str(utils.Settings.PRINT_UPDATE_TIMESTEP_MS), size=sim_config_input_size)],
    [sg.Text('Simulation launch time (software target) (ms): ', size=sim_config_text_size), sg.Input(focus=True, key='input_sim_launch_time_sw_target', default_text=str(utils.Settings.SIMULATION_SW_TARGET_LAUNCH_TIME_MS), size=sim_config_input_size)],
    [sg.Text('Simulation time allowed for hardware to detect landing (ms): ', size=sim_config_text_size), sg.Input(focus=True, key='input_sim_hw_landing_detect_time', default_text=str(utils.Settings.SIMULATION_TIME_ALLOWED_FOR_HW_TO_DETECT_LANDING_MS), size=sim_config_input_size)],

    # buttons
    [sg.Submit(key='button_submit_sim_config'), sg.Button('Restore Defaults', key='button_restore_defaults_sim_config')],
]

# rocket settings tab
rkt_options_text_size = (25, 1)
rkt_options_input_size = (6, 1)
layout_rocket_options = [
    [sg.Text('Rocket Configuration', font=(FONT, 15), justification='left')],
    [sg.Text('Rocket mass (kg): ', size=rkt_options_text_size), sg.Input(focus=True, key='input_rkt_mass', default_text=('%.3f' % utils.Settings.RKT_MASS_KG), size=rkt_options_input_size)],
    [sg.Text('Drogue parachute Cd: ', size=rkt_options_text_size), sg.Input(focus=True, key='input_drogue_cd', default_text=('%.3f' % utils.Settings.RKT_DROGUE_DRAG_COEFF), size=rkt_options_input_size)],
    [sg.Text('Drogue parachute area (m2): ', size=rkt_options_text_size), sg.Input(focus=True, key='input_drogue_area', default_text=('%.3f' % utils.Settings.RKT_DROGUE_AREA), size=rkt_options_input_size)],
    [sg.Text('Main parachute Cd: ', size=rkt_options_text_size), sg.Input(focus=True, key='input_main_cd', default_text=('%.3f' % utils.Settings.RKT_MAIN_DRAG_COEFF), size=rkt_options_input_size)],
    [sg.Text('Main parachute area (m2): ', size=rkt_options_text_size), sg.Input(focus=True, key='input_main_area', default_text=('%.3f' % utils.Settings.RKT_MAIN_AREA), size=rkt_options_input_size)],
    [sg.Text('Launch rail angle (deg): ', size=rkt_options_text_size), sg.Input(focus=True, key='input_launch_angle', default_text=('%.3f' % utils.Settings.RKT_LAUNCH_ANGLE), size=rkt_options_input_size)],
    [sg.Checkbox('Disable parachutes', size=(20, 1), default=False, key='box_disable_parachutes')],
    [sg.Submit(key='button_rkt_options_submit'), sg.Button('Restore Defaults', key='button_rkt_options_restore_defaults')],
]

# simulation results tab
sim_results_label_size = (18, 1)
sim_results_data_size = (20, 1)
CANVAS_WIDTH = 300
CANVAS_HEIGHT = 200
layout_sim_results = [
    [sg.Text('Simulation Results', font=(FONT, 15), justification='left')],
    [sg.Text('Simulation status: ', size=sim_results_label_size), sg.Text('Stopped', key='text_sim_running_status', size=sim_results_label_size)],
    [sg.Text('Simulator state: ', size=sim_results_label_size), sg.Text('Stopped', size=sim_results_data_size, key='text_sim_state'), 
     sg.Text('X position (m): ', size=sim_results_label_size), sg.Text('0', size=sim_results_data_size, key='text_x_pos'), 
     sg.Text('Z position (m): ', size=sim_results_label_size), sg.Text('0', size=sim_results_data_size, key='text_z_pos')],
    [sg.Text('Target state: ', size=sim_results_label_size), sg.Text('PAD', size=sim_results_data_size, key='text_target_state'), 
     sg.Text('X velocity (m/s): ', size=sim_results_label_size), sg.Text('0', size=sim_results_data_size, key='text_x_vel'),
     sg.Text('Z velocity (m/s): ', size=sim_results_label_size), sg.Text('0', size=sim_results_data_size, key='text_z_vel')],
    [sg.Text('Simulation time (ms): ', size=sim_results_label_size), sg.Text('0', size=sim_results_data_size, key='text_sim_time_ms'), 
     sg.Text('X acceleration (m/s2): ', size=sim_results_label_size), sg.Text('0', size=sim_results_data_size, key='text_x_acc'),
     sg.Text('Z acceleration (m/s2): ', size=sim_results_label_size), sg.Text('0', size=sim_results_data_size, key='text_z_acc')],
     # plot window
     [sg.Canvas(size=(CANVAS_WIDTH, CANVAS_HEIGHT), expand_x=True, expand_y=True, key='canvas')],

     # buttons
     [sg.Button('Start Simulation', key='button_start_sim'), sg.Button('Stop Simulation', key='button_stop_sim')],
]

# top-level layout
layout = [
    [sg.TabGroup([[sg.Tab('Configuration', layout_sim_config_options), sg.Tab('Rocket', layout_rocket_options), sg.Tab('Results', layout_sim_results)]], expand_x=True, expand_y=True)]
]

def update_sim_config(values, window):
    if values is None:
        # restore to defaults
        window['radio_target_hw'].update(False)
        window['radio_target_sw'].update(True)
        window['radio_thrust_rse'].update(True)
        window['radio_thrust_altos'].update(False)
        window['box_mach_dip'].update(True)
        window['box_noisy_altitude'].update(True)
        window['box_send_alt_pressure'].update(False)
        window['input_noisy_altitude_amplitude'].update(utils.Settings.ALTITUDE_NOISE_MAX_AMPLITUDE_M)
        window['input_ground_altitude'].update(utils.Settings.GROUND_ALTITUDE_M)
        window['input_sim_timestep'].update(utils.Settings.SIMULATION_TIMESTEP_MS)
        window['input_hw_update_timestep'].update(utils.Settings.HARDWARE_UPDATE_TIMESTEP_MS)
        window['input_print_timestep'].update(utils.Settings.PRINT_UPDATE_TIMESTEP_MS)
        window['input_sim_launch_time_sw_target'].update(utils.Settings.SIMULATION_SW_TARGET_LAUNCH_TIME_MS)
        window['input_sim_hw_landing_detect_time'].update(utils.Settings.SIMULATION_TIME_ALLOWED_FOR_HW_TO_DETECT_LANDING_MS)
    else:
        # do nothing. we will grab the values when starting simulation :)
        pass

def update_rkt_options(values, window):
    if values is None:
        window['input_rkt_mass'].update('%.3f' % utils.Settings.RKT_MASS_KG)
        window['input_drogue_cd'].update('%.3f' % utils.Settings.RKT_DROGUE_DRAG_COEFF)
        window['input_drogue_area'].update('%.3f' % utils.Settings.RKT_DROGUE_AREA)
        window['input_main_cd'].update('%.3f' % utils.Settings.RKT_MAIN_DRAG_COEFF)
        window['input_main_area'].update('%.3f' % utils.Settings.RKT_MAIN_AREA)
        window['input_launch_angle'].update('%.3f' % utils.Settings.RKT_LAUNCH_ANGLE)
        window['box_disable_parachutes'].update(False)
    else:
        # do nothing. we will grab the values when starting simulation :)
        pass

def parse_input(value):
    try:
        value = float(value)
    except:
        value = None
    return value

def get_rocket_options(values, window):
    # get inputs and sanitize. if inputs are invalid, defaults are used from utils.Settings.*
    rkt_mass = parse_input(values['input_rkt_mass'])
    if rkt_mass is None:
        # invalid input, use default and reset the input field to the default value
        rkt_mass = utils.Settings.RKT_MASS_KG
        window['input_rkt_mass'].update(utils.Settings.RKT_MASS_KG)
    
    rkt_drogue_cd = parse_input(values['input_drogue_cd'])
    if rkt_drogue_cd is None:
        rkt_drogue_cd = utils.Settings.RKT_DROGUE_DRAG_COEFF
        window['input_drogue_cd'].update(utils.Settings.RKT_DROGUE_DRAG_COEFF)
    
    rkt_drogue_area = parse_input(values['input_drogue_area'])
    if rkt_drogue_area is None:
        rkt_drogue_area = utils.Settings.RKT_DROGUE_AREA
        window['input_drogue_area'].update(utils.Settings.RKT_DROGUE_AREA)
    
    rkt_main_cd = parse_input(values['input_main_cd'])
    if rkt_main_cd is None:
        rkt_main_cd = utils.Settings.RKT_MAIN_DRAG_COEFF
        window['input_main_cd'].update(utils.Settings.RKT_MAIN_DRAG_COEFF)
    
    rkt_main_area = parse_input(values['input_main_area'])
    if rkt_main_area is None:
        rkt_main_area = utils.Settings.RKT_MAIN_AREA
        window['input_main_area'].update(utils.Settings.RKT_MAIN_AREA)
    
    rkt_launch_angle = parse_input(values['input_launch_angle'])
    if rkt_launch_angle is None:
        rkt_launch_angle = utils.Settings.RKT_LAUNCH_ANGLE
        window['input_launch_angle'].update(utils.Settings.RKT_LAUNCH_ANGLE)
    
    if values['box_disable_parachutes'] == True:
        rkt_drogue_cd = 0
        rkt_drogue_area = 0
        rkt_main_cd = 0
        rkt_main_area = 0
    
    rkt = rocket.Rocket(
        rkt_mass,
        rkt_drogue_cd,
        rkt_drogue_area,
        rkt_main_cd,
        rkt_main_area,
        rkt_launch_angle
    )

    return rkt

def get_sim_timestep(values, window):
    timestep = parse_input(values['input_sim_timestep'])
    if timestep is None:
        timestep = utils.Settings.SIMULATION_TIMESTEP_MS
        window['input_sim_timestep'].update(utils.Settings.SIMULATION_TIMESTEP_MS)
    return timestep

def get_hw_timestep(values, window):
    timestep = parse_input(values['input_hw_update_timestep'])
    if timestep is None:
        timestep = utils.Settings.HARDWARE_UPDATE_TIMESTEP_MS
        window['input_hw_update_timestep'].update(utils.Settings.HARDWARE_UPDATE_TIMESTEP_MS)
    return timestep

def get_print_timestep(values, window):
    timestep = parse_input(values['input_print_timestep'])
    if timestep is None:
        timestep = utils.Settings.PRINT_UPDATE_TIMESTEP_MS
        window['input_print_timestep'].update(utils.Settings.PRINT_UPDATE_TIMESTEP_MS)
    return timestep

def get_sw_sim_launch_time(values, window):
    timestep = parse_input(values['input_sim_launch_time_sw_target'])
    if timestep is None:
        timestep = utils.Settings.SIMULATION_SW_TARGET_LAUNCH_TIME_MS
        window['input_sim_launch_time_sw_target'].update(utils.Settings.SIMULATION_SW_TARGET_LAUNCH_TIME_MS)
    return timestep

def get_hw_landing_detect_time(values, window):
    timestep = parse_input(values['input_sim_hw_landing_detect_time'])
    if timestep is None:
        timestep = utils.Settings.SIMULATION_TIME_ALLOWED_FOR_HW_TO_DETECT_LANDING_MS
        window['input_sim_hw_landing_detect_time'].update(utils.Settings.SIMULATION_TIME_ALLOWED_FOR_HW_TO_DETECT_LANDING_MS)
    return timestep

def get_ground_altitude(values, window):
    alt = parse_input(values['input_ground_altitude'])
    if alt is None:
        alt = utils.Settings.GROUND_ALTITUDE_M
        window['input_ground_altitude'].update(utils.Settings.GROUND_ALTITUDE_M)
    return alt

def get_altitude_noise_amplitude(values, window):
    amplitude = parse_input(values['input_noisy_altitude_amplitude'])
    if amplitude is None:
        amplitude = utils.Settings.ALTITUDE_NOISE_MAX_AMPLITUDE_M
        window['input_noisy_altitude_amplitude'].update(utils.Settings.ALTITUDE_NOISE_MAX_AMPLITUDE_M)
    return amplitude

def get_sim_target(values, window):
    return values['radio_target_hw']

def get_mach_dip_emulation(values, window):
    return values['box_mach_dip']

def get_noisy_altitude_option(values, window):
    return values['box_noisy_altitude']

def get_send_alt_pressure_option(values, window):
    return values['box_send_alt_pressure']

def get_altos_fd_option(values, window):
    return values['radio_thrust_altos']

def update_gui(event, values, window):
    global sim_is_running, df_plot
    if event == 'button_submit_sim_config':
        print('pressed submit button in sim config!')
        update_sim_config(values, window)
    elif event == 'button_restore_defaults_sim_config':
        print('pressed restore defaults button in sim config')
        update_sim_config(None, window)
    elif event == 'button_rkt_options_submit':
        print('pressed submit button rocket options')
        update_rkt_options(values, window)
    elif event == 'button_rkt_options_restore_defaults':
        print('pressed button rocket options restore defaults')
        update_rkt_options(None, window)
    elif event == 'button_start_sim':
        print('pressed start sim')
        set_sim_running_status(window, 'Running')
        df_plot = pd.DataFrame(columns=utils.Settings.data_column_names)
    elif event == 'button_stop_sim':
        print('pressed stop sim')
        set_sim_running_status(window, 'Stopped')
    else:
        pass

def set_sim_running_status(window, value):
    global sim_is_running
    window['text_sim_running_status'].update(value)
    if value == 'Stopped':
        sim_is_running = False
    elif value == 'Running':
        sim_is_running = True

def update_sim_results(window, dp : utils.Sim_DataPoint, sim_state):
    window['text_x_pos'].update('%.3f' % (dp.rkt_pos_x))
    window['text_x_vel'].update('%.3f' % (dp.rkt_vel_x))
    window['text_x_acc'].update('%.3f' % (dp.rkt_acc_x))
    window['text_z_pos'].update('%.3f' % (dp.rkt_pos_z))
    window['text_z_vel'].update('%.3f' % (dp.rkt_vel_z))
    window['text_z_acc'].update('%.3f' % (dp.rkt_acc_z))
    window['text_sim_time_ms'].update('%.3f' % (dp.time))
    window['text_target_state'].update(dp.rkt_flight_state.name)
    window['text_sim_state'].update(sim_state.name)

window = sg.Window('MRT FC HIL Simulator', layout, font=("Helvetica", 12), resizable=True, finalize=True)
window.maximize()

# # --- everything related to live plotting ---
def draw_figure(canvas, figure, loc=(0, 0)):
        figure_canvas_agg = FigureCanvasTkAgg(figure, canvas)
        figure_canvas_agg.draw()
        figure_canvas_agg.get_tk_widget().pack(side='top', fill='both', expand=1)
        return figure_canvas_agg

def update_figure(dp):
    if DO_LIVE_PLOTTING:
        global fig_agg, df_plot, ax
        df_plot.loc[len(df_plot)] = dp.unwrap()
        for i in range(3):
            for j in range(2):
                ax[i][j].cla()      # clear the subplot
                ax[i][j].grid()     # draw the grid
        # update plots
        time = df_plot.iloc[:, utils.DATA_INDEX.TIME.value] / 1000
        ax[0][0].plot(time, df_plot.iloc[:, utils.DATA_INDEX.POS_X.value])
        ax[0][1].plot(time, df_plot.iloc[:, utils.DATA_INDEX.POS_Z.value])
        ax[1][0].plot(time, df_plot.iloc[:, utils.DATA_INDEX.VEL_X.value])
        ax[1][1].plot(time, df_plot.iloc[:, utils.DATA_INDEX.VEL_Z.value])
        ax[2][0].plot(time, df_plot.iloc[:, utils.DATA_INDEX.ACC_X.value])
        ax[2][1].plot(time, df_plot.iloc[:, utils.DATA_INDEX.ACC_Z.value])
        fig_agg.draw()
        for i in range(3):
            for j in range(2):
                fig.canvas.blit(ax[i][j].bbox)
    else:
        pass


if DO_LIVE_PLOTTING:
    df_plot = pd.DataFrame(columns=utils.Settings.data_column_names)


    canvas_elem = window['canvas']
    canvas = canvas_elem.TKCanvas

    plt.rc('lines', linewidth=2)
    plt.rc('axes', grid=True)
    plt.rc('grid', linestyle='--')

    # draw the initial plot in the window
    fig, ax = plt.subplots(3, 2)
    plt.tight_layout()
    ax[0][0].set_ylabel('Pos X (m)')
    ax[0][1].set_ylabel('Pos Z (m)')

    ax[1][0].set_ylabel('Vel X (m/s)')
    ax[1][1].set_ylabel('Vel Z (m/s)')

    ax[2][0].set_ylabel('Acc X (m/s2)')
    ax[2][1].set_ylabel('Acc Z (m/s2)')

    ax[2][0].set_xlabel('Time (s)')
    ax[2][1].set_xlabel('Time (s)')

    fig_agg = draw_figure(canvas, fig)

