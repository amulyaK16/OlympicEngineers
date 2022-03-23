import numpy as np
import matplotlib.pyplot as plt
import csv
import matlab.engine

'''
Chase Badalato

This script is to be used from the server side
to call digital processing functions from Matlab
'''


'''
connect_MATLAB
----------------------------
Connect to the Matlab Engine

param:
    None

return: matlab engine object
'''
def connect_Matlab():
    try:
        eng = matlab.engine.start_matlab()
        eng.addpath(os.path.realpath(os.path.join(os.getcwd(), os.path.dirname("EMGCode.m"))),           nargout=0)
        eng.addpath(os.path.realpath(os.path.join(os.getcwd(), os.path.dirname("PulseCode.m"))),         nargout=0)
        eng.addpath(os.path.realpath(os.path.join(os.getcwd(), os.path.dirname("LoadcellDSP.m"))),       nargout=0)
        eng.addpath(os.path.realpath(os.path.join(os.getcwd(), os.path.dirname("AccelerometerCode.m"))), nargout=0)
        eng.addpath(os.path.realpath(os.path.join(os.getcwd(), os.path.dirname("matlab_functions.m"))),  nargout=0)
        return eng

    except:
        return None

'''
disconnect_MATLAB
----------------------------
Disonnect to the Matlab Engine

param:
    eng: matlab engine

return: None
'''
def disconnect_Matlab(eng):
    eng.quit()

'''
DSP_ECG
----------------------------
Digitally Process raw ECG data

param:
    raw_array: array of the raw ECG data
    eng: matlab engine
return: processed array
'''
#{'ECG'}
def DSP_ECG(raw_array, eng):
    matlab_ecg = matlab.uint16(raw_array)
    dsp_array = []
    dsp_array = eng.PulseCode(matlab_ecg, nargout = 1)
    return dsp_array

'''
DSP_EMG
----------------------------
Digitally Process raw ECG data
param:
    raw_array: array of the raw ECG data
    eng: matlab engine
return: processed array
'''
#{'EMG'}
def DSP_EMG(raw_array, eng):
    matlab_ecg = matlab.uint16(raw_array)
    dsp_array = []
    dsp_array = eng.EMGCode(matlab_ecg, nargout = 1)
    return dsp_array

'''
DSP_Calculate_BPM
----------------------------
Find the BPM of the ECG signal
param:
    raw_array: array of the raw Load Cell data
    eng: matlab engine
return: the BPM
'''
def DSP_Calculate_BPM(raw_array, eng):
    matlab_ecg = matlab.uint16(raw_array)
    dsp_array = []
    freq, time, BPM = eng.matlab_functions(matlab_ecg, nargout = 3)

    '''
DSP_callback
----------------------------
A callback function to be used with threading (not used)
param:
    raw packet dict
return: processed packet dict
'''
def DSP_callback(packet, eng):
    packet['ECG'] = DSP_ECG(packet['ECG'], eng)
    packet['EMG'] = DSP_EMG(packet['EMG'], eng)
    return packet 