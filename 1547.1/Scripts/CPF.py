# coding: shift-jis
####################################################################################################
# This script was tuned specifically for the AIST FREA environment (Fixed in 2019)
#     AIST:National Institute of Advanced Industrial Science and Technology 
#     FREA:Fukushima Renewable Energy Institute
#
# What is the AIST FREA environment
#   Communication with SunSpecSVP is middleware called ExCon, and ExCon is
#   a mechanism to communicate with inverters and simulators.
####################################################################################################
"""
Copyright (c) 2018, Sandia National Labs, SunSpec Alliance and CanmetENERGY
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the names of the Sandia National Labs and SunSpec Alliance nor the names of its
contributors may be used to endorse or promote products derived from
this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Questions can be directed to support@sunspec.org
"""

import sys
import os
import traceback
from svpelab import gridsim
from svpelab import loadsim
from svpelab import pvsim
from svpelab import das
###from svpelab import der         <- Commented out because middleware is communicated using gridsim
from svpelab import hil
import script
from svpelab import result as rslt
from datetime import datetime, timedelta

import numpy as np
import collections
import cmath
import math

import time                        # WT3000 compatible
import subprocess                  # WT3000 compatible
from subprocess import PIPE        # WT3000 compatible
import re                          # WT3000 compatible
import csv                         # WT3000 compatible

import ctypes                      # WT3000 compatible
from ctypes import *               # WT3000 compatible
import threading                   # WT3000 compatible


### TEST Logic
### <START>

def print_varsize():
    #print "{}{: >25}{}{: >10}{}".format('|','Variable Name','|','Memory','|')
    #print " ------------------------------------ "
    ts.log("{}{: >25}{}{: >10}{}".format('|','Variable Name','|','Memory','|'))
    ts.log(" ------------------------------------ ")
    for var_name in dir():
        if not var_name.startswith("_"):
            #print "{}{: >25}{}{: >10}{}".format('|',var_name,'|',sys.getsizeof(eval(var_name)),'|')
            ts.log("{}{: >25}{}{: >10}{}".format('|',var_name,'|',sys.getsizeof(eval(var_name)),'|'))
    pass

### <END>

### WT3000 compatible
### <START>
def wt3000_format_set(dll, device_id):

    ts.log('--------------WT3000 FORMAT SET command Start----------------')

    rtn_up = {}
    msg = ""

    #------------------------
    #SET Format
    #------------------------
    msg = ":NUMERIC:FORMAT ASCII"
    rtn = dll.TmSend(device_id, msg)
    if rtn == 0:
        ts.log('@@@(WT3000) SET Format OK: %s' % (rtn))
    else:
        ts.log('@@@(WT3000) SET Format NG: %s' % (rtn))

    #------------------------
    #SET Number of Items
    #------------------------
    msg = ":NUMERIC:NORMAL:NUMBER 6"
    rtn = dll.TmSend(device_id, msg)
    if rtn == 0:
        ts.log('@@@(WT3000) SET Number of Items OK: %s' % (rtn))
    else:
        ts.log('@@@(WT3000) SET Number of Items NG: %s' % (rtn))

    #------------------------
    #SET Items
    #------------------------
    msg = ":NUMERIC:NORMAL:"
    msg = msg + "ITEM1 P,SIGMA,Total;"
    msg = msg + "ITEM2 Q,SIGMA,Total;"
    msg = msg + "ITEM3 S,SIGMA,Total;"
    msg = msg + "ITEM4 VRMS,SIGMA,Total;"
    msg = msg + "ITEM5 IRMS,SIGMA,Total;"
    msg = msg + "ITEM6 LAMBDA,SIGMA,Total;"
    rtn = dll.TmSend(device_id, msg)
    if rtn == 0:
        ts.log('@@@(WT3000) SET Items OK: %s' % (rtn))
    else:
        ts.log('@@@(WT3000) SET Items NG: %s' % (rtn))


    ts.log('--------------WT3000 FORMATSET command End-----------------')

    pass

def wt3000_data_read(dll, device_id):

    ts.log('--------------WT3000 command Start----------------')

    rtn_up = {}
    msg = ""
    tmp_data = {}

    #------------------------
    #SET VALUE?
    #------------------------
    msg = ":NUMERIC:NORMAL:VALUE?"
    rtn = dll.TmSend(device_id, msg)
    if rtn == 0:
        ts.log('@@@(WT3000) SET VALUE? OK: %s' % (rtn))
    else:
        ts.log('@@@(WT3000) SET VALUE? NG: %s' % (rtn))


    buf = ctypes.create_string_buffer(1024,1024)
    bufsize = ctypes.c_int(1024)
    length = ctypes.c_int()
    rtn = dll.TmReceive(device_id, buf, bufsize, ctypes.pointer(length))
    if rtn == 0:
        ts.log('@@@(WT3000) RECEIVE OK: %s' % (rtn))
        tmp_data = buf.value.split(",")
        ts.log('@@@(WT3000) split: %s' % (tmp_data))
    else:
        ts.log('@@@(WT3000) RECEIVE NG: %s' % (rtn))
        tmp_data[0] = "NAN"
        tmp_data[1] = "NAN"
        tmp_data[2] = "NAN"
        tmp_data[3] = "NAN"
        tmp_data[4] = "NAN"
        tmp_data[5] = "NAN"

    ts.log('buf.value: %s' % (buf.value))
    ts.log('length: %s' % (length))


    #------------------------
    #Return Data Set
    #------------------------
###    tmp_data = buf.value.split(",")
###    ts.log('@@@(WT3000) split: %s' % (tmp_data))

    # Active power(P)
    if tmp_data[0] == "NAN" or tmp_data[0] == "INF":
        rtn_up['AC_P_1'] = 0
        rtn_up['AC_P_2'] = 0
        rtn_up['AC_P_3'] = 0
    else:
        rtn_up['AC_P_1'] = float(tmp_data[0])
        rtn_up['AC_P_2'] = float(tmp_data[0])
        rtn_up['AC_P_3'] = float(tmp_data[0])
    ts.log('@@@(WT3000) Active power(P): %s' % (rtn_up['AC_P_1']))

    # Reactive power(Q)
    if tmp_data[1] == "NAN" or tmp_data[1] == "INF":
        rtn_up['AC_Q_1'] = 0
        rtn_up['AC_Q_2'] = 0
        rtn_up['AC_Q_3'] = 0
    else:
        rtn_up['AC_Q_1'] = float(tmp_data[1])
        rtn_up['AC_Q_2'] = float(tmp_data[1])
        rtn_up['AC_Q_3'] = float(tmp_data[1])
    ts.log('@@@(WT3000) Reactive power(Q): %s' % (rtn_up['AC_Q_1']))

    # Apparent power(S)
    if tmp_data[2] == "NAN" or tmp_data[2] == "INF":
        rtn_up['AC_S_1'] = 0
        rtn_up['AC_S_2'] = 0
        rtn_up['AC_S_3'] = 0
    else:
        rtn_up['AC_S_1'] = float(tmp_data[2])
        rtn_up['AC_S_2'] = float(tmp_data[2])
        rtn_up['AC_S_3'] = float(tmp_data[2])
    ts.log('@@@(WT3000) Apparent power(S): %s' % (rtn_up['AC_S_1']))

    # Voltage(U)
    if tmp_data[3] == "NAN" or tmp_data[3] == "INF":
        rtn_up['AC_VRMS_1'] = 0
        rtn_up['AC_VRMS_2'] = 0
        rtn_up['AC_VRMS_3'] = 0
    else:
        rtn_up['AC_VRMS_1'] = float(tmp_data[3])
        rtn_up['AC_VRMS_2'] = float(tmp_data[3])
        rtn_up['AC_VRMS_3'] = float(tmp_data[3])
    ts.log('@@@(WT3000) Voltage(U): %s' % (rtn_up['AC_VRMS_1']))

    # Current(I)
    if tmp_data[4] == "NAN" or tmp_data[4] == "INF":
        rtn_up['AC_IRMS_1'] = 0
        rtn_up['AC_IRMS_2'] = 0
        rtn_up['AC_IRMS_3'] = 0
    else:
        rtn_up['AC_IRMS_1'] = float(tmp_data[4])
        rtn_up['AC_IRMS_2'] = float(tmp_data[4])
        rtn_up['AC_IRMS_3'] = float(tmp_data[4])
    ts.log('@@@(WT3000) Current(I): %s' % (rtn_up['AC_IRMS_1']))

    # Power factor(lambda)
    if tmp_data[5] == "NAN" or tmp_data[5] == "INF":
        rtn_up['AC_PF_1'] = 0
        rtn_up['AC_PF_2'] = 0
        rtn_up['AC_PF_3'] = 0
    else:
        rtn_up['AC_PF_1'] = float(tmp_data[5])
        rtn_up['AC_PF_2'] = float(tmp_data[5])
        rtn_up['AC_PF_3'] = float(tmp_data[5])
    ts.log('@@@(WT3000) Power factor(lambda): %s' % (rtn_up['AC_PF_1']))


    """
    #------------------------
    #Local Test Code start
    #------------------------
    # Active power(P)
    rtn_up['AC_P_1'] = 3000
    rtn_up['AC_P_2'] = 3000
    rtn_up['AC_P_3'] = 3000

    # Apparent power(S)
    rtn_up['AC_S_1'] = 3100
    rtn_up['AC_S_2'] = 3100
    rtn_up['AC_S_3'] = 3100

    # Reactive power(Q)
    rtn_up['AC_Q_1'] = 100
    rtn_up['AC_Q_2'] = 100
    rtn_up['AC_Q_3'] = 100

    # Voltage(U)
    rtn_up['AC_VRMS_1'] = 190
    rtn_up['AC_VRMS_2'] = 190
    rtn_up['AC_VRMS_3'] = 190

    # Current(I)
    rtn_up['AC_IRMS_1'] = 205
    rtn_up['AC_IRMS_2'] = 205
    rtn_up['AC_IRMS_3'] = 205

    # Power factor(lambda)
    rtn_up['AC_PF_1'] = 0.98
    rtn_up['AC_PF_2'] = 0.98
    rtn_up['AC_PF_3'] = 0.98
    #------------------------
    #Local Test Code end
    #------------------------
    """

    ts.log('--------------WT3000 command End-----------------')

    return rtn_up

### <END>


### Add for Thread control
### <START>
def MeasureThread(e ,MeasurMachine ,writer_active ,writer_apparent ,writer_reactive ,writer_voltage ,writer_current ,writer_powerfactor ,m_time ,dll ,device_id ,phases):

    ts.log('--------------MeasureThread Start----------------')

    sv_time = time.time()

    wt3000_format_set(dll, device_id)
    ts.log('@@@wt3000_format_set()')

    MeasureData = wt3000_data_read(dll, device_id)
    ts.log('@@@wt3000_data_read()')

    MeasureTime = time.time() - sv_time

    #------------------------
    # Active power
    #------------------------
#    if phases == 'Single Phase':
#        grf_rec_active = [MeasureTime, (MeasureData.get('AC_P_1')/1000)]
#    else:
#        grf_rec_active = [MeasureTime, (MeasureData.get('AC_P_1')/1000)*3]
    grf_rec_active = [MeasureTime, (MeasureData.get('AC_P_1')/1000)]
    writer_active.writerow(grf_rec_active)
    ts.log('grf_rec_active: %s ' % (grf_rec_active))

    #------------------------
    # Apparent power
    #------------------------
#    if phases == 'Single Phase':
#        grf_rec_apparent = [MeasureTime, (MeasureData.get('AC_S_1')/1000)]
#    else:
#        grf_rec_apparent = [MeasureTime, (MeasureData.get('AC_S_1')/1000)*3]
    grf_rec_apparent = [MeasureTime, (MeasureData.get('AC_S_1')/1000)]
    writer_apparent.writerow(grf_rec_apparent)
    ts.log('grf_rec_apparent: %s ' % (grf_rec_apparent))

    #------------------------
    # Reactive power
    #------------------------
#    if phases == 'Single Phase':
#        grf_rec_reactive = [MeasureTime, (MeasureData.get('AC_Q_1')/1000)]
#    else:
#        grf_rec_reactive = [MeasureTime, (MeasureData.get('AC_Q_1')/1000)*3]
    grf_rec_reactive = [MeasureTime, (MeasureData.get('AC_Q_1')/1000)]
    writer_reactive.writerow(grf_rec_reactive)
    ts.log('grf_rec_reactive: %s ' % (grf_rec_reactive))

    #------------------------
    # Voltage
    #------------------------
    wk_voltage = round(MeasureData.get('AC_VRMS_1'),2)
    grf_rec_voltage = [MeasureTime, wk_voltage]
    writer_voltage.writerow(grf_rec_voltage)
    ts.log('grf_rec_voltage: %s ' % (grf_rec_voltage))

    #------------------------
    # Current
    #------------------------
    wk_current = round(MeasureData.get('AC_IRMS_1'),2)
    grf_rec_current = [MeasureTime, wk_current]
    writer_current.writerow(grf_rec_current)
    ts.log('grf_rec_current: %s ' % (grf_rec_current))

    #------------------------
    # Power factor
    #------------------------
    wk_powerfactor = round(MeasureData.get('AC_PF_1'),2)
    if wk_powerfactor >= 0:
        wk_powerfactor = (wk_powerfactor * -100) + 100
    else:
        wk_powerfactor = (wk_powerfactor * -100) - 100
    grf_rec_powerfactor = [MeasureTime, wk_powerfactor]
    writer_powerfactor.writerow(grf_rec_powerfactor)
    ts.log('grf_rec_powerfactor: %s ' % (grf_rec_powerfactor))

#    for i in range(99999):
    while not e.stop_event.is_set():
        time.sleep(m_time)
#        ts.sleep(m_time)
        #event_is_set = e.wait()
        #ts.log('@@@ Starting MeasureThread cnt = %s' % (i))

        MeasureData = wt3000_data_read(dll, device_id)
        ts.log('@@@wt3000_data_read()')

        MeasureTime = time.time() - sv_time

        #------------------------
        # Active power
        #------------------------
#        if phases == 'Single Phase':
#            grf_rec_active = [MeasureTime, (MeasureData.get('AC_P_1')/1000)]
#        else:
#            grf_rec_active = [MeasureTime, (MeasureData.get('AC_P_1')/1000)*3]
        grf_rec_active = [MeasureTime, (MeasureData.get('AC_P_1')/1000)]
        writer_active.writerow(grf_rec_active)
        ts.log('grf_rec_active: %s ' % (grf_rec_active))

        #------------------------
        # Apparent power
        #------------------------
#        if phases == 'Single Phase':
#            grf_rec_apparent = [MeasureTime, (MeasureData.get('AC_S_1')/1000)]
#        else:
#            grf_rec_apparent = [MeasureTime, (MeasureData.get('AC_S_1')/1000)*3]
        grf_rec_apparent = [MeasureTime, (MeasureData.get('AC_S_1')/1000)]
        writer_apparent.writerow(grf_rec_apparent)
        ts.log('grf_rec_apparent: %s ' % (grf_rec_apparent))

        #------------------------
        # Reactive power
        #------------------------
#        if phases == 'Single Phase':
#            grf_rec_reactive = [MeasureTime, (MeasureData.get('AC_Q_1')/1000)]
#        else:
#            grf_rec_reactive = [MeasureTime, (MeasureData.get('AC_Q_1')/1000)*3]
        grf_rec_reactive = [MeasureTime, (MeasureData.get('AC_Q_1')/1000)]
        writer_reactive.writerow(grf_rec_reactive)
        ts.log('grf_rec_reactive: %s ' % (grf_rec_reactive))

        #------------------------
        # Voltage
        #------------------------
        wk_voltage = round(MeasureData.get('AC_VRMS_1'),2)
        grf_rec_voltage = [MeasureTime, wk_voltage]
        writer_voltage.writerow(grf_rec_voltage)
        ts.log('grf_rec_voltage: %s ' % (grf_rec_voltage))

        #------------------------
        # Current
        #------------------------
        wk_current = round(MeasureData.get('AC_IRMS_1'),2)
        grf_rec_current = [MeasureTime, wk_current]
        writer_current.writerow(grf_rec_current)
        ts.log('grf_rec_current: %s ' % (grf_rec_current))

        #------------------------
        # Power factor
        #------------------------
        wk_powerfactor = round(MeasureData.get('AC_PF_1'),2)
        if wk_powerfactor >= 0:
            wk_powerfactor = (wk_powerfactor * -100) + 100
        else:
            wk_powerfactor = (wk_powerfactor * -100) - 100
        grf_rec_powerfactor = [MeasureTime, wk_powerfactor]
        writer_powerfactor.writerow(grf_rec_powerfactor)
        ts.log('grf_rec_powerfactor: %s ' % (grf_rec_powerfactor))

    ts.log('--------------MeasureThread End------------------')
    pass
### <END>


def q_p_criteria(pf, MSA_P, MSA_Q, daq, tr, step, q_initial, e, dll, device_id, start_time):
    """
    Determine Q(MSAs)
    :param pf:          power factor target
    :param MSA_P:       manufacturer's specified accuracy of active power (W)
    :param MSA_Q:       manufacturer's specified accuracy of reactive power (VAr)
    :param daq:         data acquisition object in order to manipulated
    :param tr:          response time (s)
    :param step:        test procedure step letter or number (e.g "Step G")
    :param q_initial:   dictionnary with timestamp and reactive value before step change
    :return:    dictionnary q_p_analysis that contains passfail of response time requirements ( q_p_analysis['Q_TR_PF'])
    and test result accuracy requirements ( q_p_analysis['Q_FINAL_PF'] )
    """
    ts.log('--------------q_p_criteria start------------------')

    tr_analysis = 'start'
    result_analysis = 'start'
    q_p_analysis = {}

    """
    Every time a parameter is stepped or ramped, 
    measure and record the time domain current and 
    voltage response for at least 4 times the maximum 
    expected response time after the stimulus, and measure or derive, 
    active power, apparent power, reactive power, and power factor.
    
    This is only for the response time requirements (5.14.3.3 Criteria)
    """
    first_tr = q_initial['timestamp']+timedelta(seconds = tr)
    ts.log('first_tr: %s ' % (first_tr))
    four_times_tr = q_initial['timestamp']+timedelta(seconds = 4*tr)
    ts.log('four_times_tr: %s ' % (four_times_tr))

    try:
        q_p_analysis['Q_INITIAL'] = 'No Data'           # ADD FREA
        q_p_analysis['Q_FINAL'] = 'No Data'             # ADD FREA
        q_p_analysis['Q_TR_PF'] = 'No Data'             # ADD FREA
        q_p_analysis['Q_FINAL_PF'] = 'No Data'          # ADD FREA

        while tr_analysis == 'start':
            ts.log('--------------while tr_analysis start------------------')
            time_to_sleep = first_tr - datetime.now()
            ts.sleep(time_to_sleep.total_seconds())
            now = datetime.now()
            ts.log('now: %s ' % (now))
            if first_tr <= now:
                ts.log('--------------(tr_analysis data_sample start)------------------')
                daq.data_sample()
                ts.log('--------------(tr_analysis data_sample end)------------------')
###                data = daq.data_capture_read()       # WT3000 compatible
                e.clear()                               # Add for Thread control
                ts.log('--------------(tr_analysis wt3000_data_read start)------------------')
                data = wt3000_data_read(dll, device_id) # WT3000 compatible
                ts.log('--------------(tr_analysis wt3000_data_read end)------------------')
                e.set()                                 # Add for Thread control
                daq.sc['TIME'] = time.time() - start_time # ADD FREA
                daq.sc['V_MEAS'] = measurement_total(data=data, type_meas='V',log=False)
                daq.sc['Q_MEAS'] = measurement_total(data=data, type_meas='Q',log=False)
                daq.sc['P_MEAS'] = measurement_total(data=data, type_meas='P',log=False)
#                ts.log('q_p_criteria(tr_analysis) V = %s%%  Q = %s%%  P = %s%%' % (daq.sc['V_MEAS'], daq.sc['Q_MEAS'], daq.sc['P_MEAS']))
                ts.log('TIME: %s ' % (daq.sc['TIME']))
                ts.log('V_MEAS: %s ' % (daq.sc['V_MEAS']))
                ts.log('Q_MEAS: %s ' % (daq.sc['Q_MEAS']))
                ts.log('P_MEAS: %s ' % (daq.sc['P_MEAS']))
                # The variable q_tr is the value use to verify the time response requirement.
                q_tr = daq.sc['Q_MEAS']
                daq.sc['event'] = "{}_tr_1".format(step)
                daq.data_sample()
                # This is to get out of the while loop. It provides the timestamp of tr_1
                tr_analysis = now

        while result_analysis == 'start':
            ts.log('--------------while result_analysis start------------------')
            time_to_sleep = four_times_tr - datetime.now()
            ts.sleep(time_to_sleep.total_seconds())
            now = datetime.now()
            ts.log('now: %s ' % (now))
            if four_times_tr <= now:
                ts.log('--------------(result_analysis data_sample start)------------------')
                daq.data_sample()
                ts.log('--------------(result_analysis data_sample end)------------------')
###                data = daq.data_capture_read()       # WT3000 compatible
                e.clear()                               # Add for Thread control
                ts.log('--------------(result_analysis wt3000_data_read start)------------------')
                data = wt3000_data_read(dll, device_id) # WT3000 compatible
                ts.log('--------------(result_analysis wt3000_data_read end)------------------')
                e.set()                                 # Add for Thread control
                daq.sc['TIME'] = time.time() - start_time # ADD FREA
                daq.sc['V_MEAS'] = measurement_total(data=data, type_meas='V',log=True)
                daq.sc['Q_MEAS'] = measurement_total(data=data, type_meas='Q',log=True)
                daq.sc['P_MEAS'] = measurement_total(data=data, type_meas='P',log=True)
                daq.sc['event'] = "{}_tr_4".format(step)
#                ts.log('q_p_criteria(result_analysis) V = %s%%  Q = %s%%  P = %s%%' % (daq.sc['V_MEAS'], daq.sc['Q_MEAS'], daq.sc['P_MEAS']))
                ts.log('TIME: %s ' % (daq.sc['TIME']))
                ts.log('V_MEAS: %s ' % (daq.sc['V_MEAS']))
                ts.log('Q_MEAS: %s ' % (daq.sc['Q_MEAS']))
                ts.log('P_MEAS: %s ' % (daq.sc['P_MEAS']))
                # To calculate the min/max, you need the measured value
                p_min = daq.sc['P_MEAS']+1.5*MSA_P
                p_max = daq.sc['P_MEAS']-1.5*MSA_P
                daq.sc['Q_TARGET_MIN'] = math.sqrt(pow(p_min, 2)*((1/pow(pf,2))-1))-1.5*MSA_Q  # reactive power target from the lower voltage limit
                daq.sc['Q_TARGET_MAX'] = math.sqrt(pow(p_max, 2)*((1/pow(pf,2))-1))+1.5*MSA_Q  # reactive power target from the upper voltage limit
                daq.data_sample()
                ts.log('        Q actual, min, max: %s, %s, %s' % (daq.sc['Q_MEAS'], daq.sc['Q_TARGET_MIN'], daq.sc['Q_TARGET_MAX']))

                """
                The variable q_tr is the value use to verify the time response requirement.
                |----------|----------|----------|----------|
                           1st tr     2nd tr     3rd tr     4th tr            
                |          |                                |
                q_initial  q_tr                             q_final    
                
                (1547.1)After each voltage, the open loop response time, Tr , is evaluated. 
                The expected reactive power output, Q(T r ) ,
                at one times the open loop response time , 
                is calculated as 90% x (Qfinal - Q initial ) + Q initial
                """

                q_p_analysis['Q_INITIAL'] = q_initial['value']
                q_p_analysis['Q_FINAL'] = daq.sc['Q_MEAS']
                q_tr_diff = q_p_analysis['Q_FINAL'] - q_p_analysis['Q_INITIAL']
                q_tr_target = ((0.9 * q_tr_diff) +  q_p_analysis['Q_INITIAL'])
                # This q_tr_diff < 0 has been added to tackle when Q_final - Q_initial is negative.
                if q_tr_diff < 0 :
                    if q_tr <= q_tr_target :
                        q_p_analysis['Q_TR_PF'] = 'Pass'
                    else:
                        q_p_analysis['Q_TR_PF'] = 'Fail'
                elif q_tr_diff >= 0:
                    if q_tr >= q_tr_target :
                        q_p_analysis['Q_TR_PF'] = 'Pass'
                    else:
                        q_p_analysis['Q_TR_PF'] = 'Fail'

                if daq.sc['Q_TARGET_MIN'] <= daq.sc['Q_MEAS'] <= daq.sc['Q_TARGET_MAX']:
                    q_p_analysis['Q_FINAL_PF'] = 'Pass'
                else:
                    q_p_analysis['Q_FINAL_PF'] = 'Fail'
                ts.log('        Q_TR Passfail: %s' % (q_p_analysis['Q_TR_PF']))
                ts.log('        Q_FINAL Passfail: %s' % (q_p_analysis['Q_FINAL_PF']))

                # This is to get out of the while loop. It provides the timestamp of tr_4
                result_analysis = now

                ts.log('--------------q_p_criteria end------------------')

    except:
        daq.sc['V_MEAS'] = 'No Data'
        daq.sc['P_MEAS'] = 'No Data'
        daq.sc['Q_MEAS'] = 'No Data'
        passfail = 'Fail'
        daq.sc['Q_TARGET_MIN'] = 'No Data'
        daq.sc['Q_TARGET_MAX'] = 'No Data'

    return q_p_analysis

###def get_q_initial(daq, step):
def get_q_initial(daq, step, dll, device_id, e):
    """
    Sum the EUT reactive power from all phases
    :param daq:         data acquisition object in order to manipulated
    :param step:        test procedure step letter or number (e.g "Step G")
    :return: returns a dictionnary with the timestamp, event and total EUT reactive power
    """
    # TODO : In a more sophisticated approach, q_initial['timestamp'] will come from a reliable secure thread or data acquisition timestamp
    q_initial={}
    q_initial['timestamp'] = datetime.now()
    daq.data_sample()
###    data = daq.data_capture_read()       # WT3000 compatible
    e.clear()                               # Add for Thread control
    data = wt3000_data_read(dll, device_id) # WT3000 compatible
    e.set()                                 # Add for Thread control
    daq.sc['event'] = step
    daq.sc['Q_MEAS'] = measurement_total(data=data, type_meas='Q', log=True)
    daq.data_sample()
    q_initial['value'] = daq.sc['Q_MEAS']
    return q_initial

def get_measurement_label(type_meas):
    """
    Sum the EUT reactive power from all phases
    :param type_meas:   Either V,P or Q
    :return:            List of labeled measurements
    """

    phases = ts.param_value('eut.phases')
    if type_meas == 'V':
        meas_root = 'AC_VRMS'
    elif type_meas == 'P':
        meas_root = 'AC_P'
    elif type_meas == 'PF':
        meas_root = 'AC_PF'
    elif type_meas == 'I':
        meas_root = 'AC_IRMS'
    else:
        meas_root = 'AC_Q'
    if phases == 'Single phase':
        meas_label = [meas_root+'_1']
    elif phases == 'Split phase':
        meas_label = [meas_root+'_1',meas_root+'_2']
    elif phases == 'Three phase':
        meas_label = [meas_root+'_1',meas_root+'_2',meas_root+'_3']

    return meas_label

def measurement_total(data, type_meas,log):
    """
    Sum the EUT reactive power from all phases
    :param data:        dataset from data acquistion object
    :param type_meas:   Either V,P or Q
    :param log:         Boolean variable to disable or enable logging
    :return: either total EUT reactive power, total EUT active power or average V
    """
    phases = ts.param_value('eut.phases')

    if phases == 'Single phase':
        value = data.get(get_measurement_label(type_meas)[0])
        if log:
            ts.log_debug('        %s are: %s' % (get_measurement_label(type_meas),value))
        nb_phases = 1

    elif phases == 'Split phase':
        value1 = data.get(get_measurement_label(type_meas)[0])
        value2 = data.get(get_measurement_label(type_meas)[1])
        if log:
            ts.log_debug('        %s are: %s, %s' % (get_measurement_label(type_meas),value1,value2))
        value = value1 + value2
        nb_phases = 2

    elif phases == 'Three phase':
        ts.log('get_measurement_label %s' % (get_measurement_label(type_meas)[0]))
        ts.log('get_measurement_label %s' % (get_measurement_label(type_meas)[1]))
        ts.log('get_measurement_label %s' % (get_measurement_label(type_meas)[2]))
        value1 = data.get(get_measurement_label(type_meas)[0])
        value2 = data.get(get_measurement_label(type_meas)[1])
        value3 = data.get(get_measurement_label(type_meas)[2])
        ts.log('get_measurement_label v1 = %s%%  v2 = %s%%  v3 = %s%%' % (value1, value2, value3))
        if log:
            ts.log_debug('        %s are: %s, %s, %s' % (get_measurement_label(type_meas),value1,value2,value3))
        value = value1 + value2 + value3
        nb_phases = 3

    else:
        ts.log_error('Inverter phase parameter not set correctly.')
        ts.log_error('phases=%s' % phases)
        raise

    if type_meas == 'V':
        # average value of V
        value = value/nb_phases

    elif type_meas == 'P':
        return abs(value)

    return value

def test_run():

    result = script.RESULT_FAIL
    grid = None
    pv = p_rated = None
    daq = None
    eut = None
    rs = None
    chil = None
    result_summary = None
    step = None
    q_initial = None

    grf_dat_file_active = None       # WT3000 compatible
    grf_dat_file_apparent = None     # WT3000 compatible
    grf_dat_file_reactive = None     # WT3000 compatible
    grf_dat_file_voltage = None      # WT3000 compatible
    grf_dat_file_curren = None       # WT3000 compatible
    grf_dat_file_powerfactor = None  # WT3000 compatible

    sv_time = -1                     # WT3000 compatible

    dll = None                       # WT3000 compatible
    tcp_control = ctypes.c_int(4)    # WT3000 compatible
    device_id = ctypes.c_int()       # WT3000 compatible
    e = None                         # WT3000 compatible

    start_time = time.time()         # FREA ADD

    #sc_points = ['PF_TARGET', 'PF_MAX', 'PF_MIN']



    try:

        cat = ts.param_value('eut.cat')
        cat2 = ts.param_value('eut.cat2')
        sink_power = ts.param_value('eut.sink_power')
        p_rated = ts.param_value('eut.p_rated')
        p_rated_prime = ts.param_value('eut.p_rated_prime')
        s_rated = ts.param_value('eut.s_rated')

        # DC voltages
        v_nom_in_enabled = ts.param_value('cpf.v_in_nom')
        v_min_in_enabled = ts.param_value('cpf.v_in_min')
        v_max_in_enabled = ts.param_value('cpf.v_in_max')

        v_nom_in = ts.param_value('eut.v_in_nom')
        v_min_in = ts.param_value('eut.v_in_min')
        v_max_in = ts.param_value('eut.v_in_max')

        # AC voltages
        v_nom = ts.param_value('eut.v_nom')
        v_min = ts.param_value('eut.v_low')
        v_max = ts.param_value('eut.v_high')
        p_min = ts.param_value('eut.p_min')
        p_min_prime = ts.param_value('eut.p_min_prime')
        phases = ts.param_value('eut.phases')
        pf_response_time = ts.param_value('eut.pf_response_time')
        #imbalance_resp = ts.param_value('eut.imbalance_resp')

        # Pass/fail accuracies
        pf_msa = ts.param_value('eut.pf_msa')

        #According to Table 3-Minimum requirements for manufacturers stated measured and calculated accuracy
        MSA_Q = 0.05 * s_rated
        MSA_P = 0.05 * s_rated
        MSA_V = 0.01 * v_nom
        a_v = MSA_V * 1.5

        # get target power factors
        pf_targets = {}
        if ts.param_value('cpf.pf_min_inj') == 'Enabled':
            pf_targets['cpf_min_ind'] = float(ts.param_value('cpf.pf_min_inj_value'))
        if ts.param_value('cpf.pf_mid_inj') == 'Enabled':
            pf_targets['cpf_mid_ind'] = float(ts.param_value('cpf.pf_mid_inj_value'))
        if ts.param_value('cpf.pf_min_ab') == 'Enabled':
            pf_targets['cpf_min_cap'] = float(ts.param_value('cpf.pf_min_ab_value'))
        if ts.param_value('cpf.pf_mid_ab') == 'Enabled':
            pf_targets['cpf_mid_cap'] = float(ts.param_value('cpf.pf_mid_ab_value'))

        v_in_targets = {}

        if v_nom_in_enabled == 'Enabled' :
            v_in_targets['v_nom_in'] = v_nom_in
        if v_min_in != v_nom_in and v_min_in_enabled == 'Enabled':
            v_in_targets['v_min_in'] = v_min_in
        if v_max_in != v_nom_in and v_max_in_enabled == 'Enabled':
            v_in_targets['v_max_in'] = v_max_in
        if not v_in_targets:
            ts.log_error('No V_in target specify. Please select a V_IN test')
            raise

        ip_addr = ts.param_value('cpf.ip_addr')              # WT3000 compatible
        m_time = ts.param_value('cpf.m_time')                # Add for Thread control
        p_ramp_rate = ts.param_value('eut.ramp_rate')        # Change because middleware is communicated using gridsim

### WT3000 compatible
### <START>
        dll = ctypes.WinDLL(r"C:\\Python27\\DLLs\\tmctl")
        ts.log('@@@(WT3000) DLL OK')

        tcp_address = ip_addr + ",anonymous,"
        rtn = dll.TmcInitialize(tcp_control, tcp_address, ctypes.pointer(device_id))
        if rtn == 0:
            ts.log('@@@(WT3000) CONNECT OK: %s' % (rtn))
        else:
            ts.log('@@@(WT3000) CONNECT NG: %s' % (tcp_address))

        wt3000_format_set(dll, device_id)
        ts.log('@@@wt3000_format_set()')
### <END>


        """
        a) Connect the EUT according to the instructions and specifications provided by the manufacturer.
        """
        # initialize HIL environment, if necessary
        chil = hil.hil_init(ts)
        if chil is not None:
            chil.config()

        # grid simulator is initialized with test parameters and enabled
        grid = gridsim.gridsim_init(ts)  # Turn on AC so the EUT can be initialized
        if grid is not None:
###            grid.voltage(v_nom)                                                                                              # Convert to line voltage
            grid.config_asymmetric_phase_angles(mag=[v_nom, v_nom, v_nom], angle=[0.0, 120.0, -120.0])                          # Convert to line voltage
            grid.volt_var(params={'Ena': False})                                                                                # Change because middleware is communicated using gridsim

        # pv simulator is initialized with test parameters and enabled
        pv = pvsim.pvsim_init(ts)
        if pv is not None:
            pv.power_set(p_rated)
            pv.power_on()  # Turn on DC so the EUT can be initialized

        # DAS soft channels
###        das_points = {'sc': ('V_MEAS', 'P_MEAS', 'Q_MEAS', 'Q_TARGET_MIN', 'Q_TARGET_MAX', 'PF_TARGET', 'event')}            # <- Since the graph is not displayed, it is added
        das_points = {'sc': ('TIME', 'V_MEAS', 'P_MEAS', 'Q_MEAS', 'Q_TARGET_MIN', 'Q_TARGET_MAX', 'PF_TARGET', 'event')}       # <- Since the graph is not displayed, it is added

        # initialize data acquisition
        daq = das.das_init(ts, sc_points=das_points['sc'])

        if daq:
            daq.sc['TIME'] = time.time()                            # <- Since the graph is not displayed, it is added
            daq.sc['V_MEAS'] = 100
            daq.sc['P_MEAS'] = 100
            daq.sc['Q_MEAS'] = 100
            daq.sc['Q_TARGET_MIN'] = 100
            daq.sc['Q_TARGET_MAX'] = 100
            daq.sc['PF_TARGET'] = 1
            daq.sc['event'] = 'None'

        ts.log('DAS device: %s' % daq.info())


        # initialize waveform data acquisition                      # DL850E compatible
        daq_wf = das.das_init(ts, 'das_wf')                         # DL850E compatible
        if daq_wf is not None:                                      # DL850E compatible
            ts.log('DAS Waveform device: %s' % (daq_wf.info()))     # DL850E compatible


        """
        b) Set all voltage trip parameters to the widest range of adjustability. Disable all reactive/active power
        control functions.
        """
        # it is assumed the EUT is on
### Commented out because middleware is communicated using gridsim
### <START>
###        eut = der.der_init(ts)
###        if eut is not None:
###            eut.config()
###            # disable volt/var curve
###            eut.volt_var(params={'Ena': False})
###            ts.log_debug('If not done already, set L/HVRT and trip parameters to the widest range of adjustability.')
### <END>

        """
        c) Set all AC test source parameters to the nominal operating voltage and frequency.
        """
        if grid is not None:
###            grid.voltage(v_nom)                                                                       # Convert to line voltage
            grid.config_asymmetric_phase_angles(mag=[v_nom, v_nom, v_nom], angle=[0.0, 120.0, -120.0])   # Convert to line voltage

        # open result summary file
        result_summary_filename = 'result_summary.csv'
        result_summary = open(ts.result_file_path(result_summary_filename), 'a+')
        ts.result_file(result_summary_filename)

        result_summary.write('RESULT_ACCURACY_REQUIREMENTS,RESPONSE_TIME_REQUIREMENTS,PF_TARGET,VRMS_ACT,P_ACT,Q_FINAL,Q_TARGET_MIN,Q_TARGET_MAX,STEP,FILENAME\n')

        """
        d) Adjust the EUT's available active power to Prated. For an EUT with an input voltage range, set the input
        voltage to Vin_nom. The EUT may limit active power throughout the test to meet reactive power requirements.

        s) For an EUT with an input voltage range, repeat steps d) through o) for Vin_min and Vin_max.
        """
        # TODO: Include step t)
        """
        t) Steps d) through q) may be repeated to test additional communication protocols - Run with another test.
        """


### Graph drawing for FREA original gnuplot
### <START>
        e = threading.Event()
        e.stop_event = threading.Event()

        # Active power
        grf_dat_file_active = ts.results_dir() + "\C_power_factor_active.csv"
        grf_dat_file_active = re.sub(r'\\', "/", grf_dat_file_active)
        ts.log('grf_dat_file_active = %s' % (grf_dat_file_active))
        grf_dat_active = open(grf_dat_file_active, mode='w')
        writer_active = csv.writer(grf_dat_active, lineterminator='\n')

        # Apparent power
        grf_dat_file_apparent = ts.results_dir() + "\C_power_factor_apparent.csv"
        grf_dat_file_apparent = re.sub(r'\\', "/", grf_dat_file_apparent)
        ts.log('grf_dat_file_apparent = %s' % (grf_dat_file_apparent))
        grf_dat_apparent = open(grf_dat_file_apparent, mode='w')
        writer_apparent = csv.writer(grf_dat_apparent, lineterminator='\n')

        # Reactive power
        grf_dat_file_reactive = ts.results_dir() + "\C_power_factor_reactive.csv"
        grf_dat_file_reactive = re.sub(r'\\', "/", grf_dat_file_reactive)
        ts.log('grf_dat_file_reactive = %s' % (grf_dat_file_reactive))
        grf_dat_reactive = open(grf_dat_file_reactive, mode='w')
        writer_reactive = csv.writer(grf_dat_reactive, lineterminator='\n')

        # Voltage
        grf_dat_file_voltage = ts.results_dir() + "\C_power_factor_voltage.csv"
        grf_dat_file_voltage = re.sub(r'\\', "/", grf_dat_file_voltage)
        ts.log('grf_dat_file_voltage = %s' % (grf_dat_file_voltage))
        grf_dat_voltage = open(grf_dat_file_voltage, mode='w')
        writer_voltage = csv.writer(grf_dat_voltage, lineterminator='\n')

        # Current
        grf_dat_file_current = ts.results_dir() + "\C_power_factor_current.csv"
        grf_dat_file_current = re.sub(r'\\', "/", grf_dat_file_current)
        ts.log('grf_dat_file_current = %s' % (grf_dat_file_current))
        grf_dat_current = open(grf_dat_file_current, mode='w')
        writer_current = csv.writer(grf_dat_current, lineterminator='\n')

        # Power factor
        grf_dat_file_powerfactor = ts.results_dir() + "\C_power_factor_powerfactor.csv"
        grf_dat_file_powerfactor = re.sub(r'\\', "/", grf_dat_file_powerfactor)
        ts.log('grf_dat_file_powerfactor = %s' % (grf_dat_file_powerfactor))
        grf_dat_powerfactor = open(grf_dat_file_powerfactor, mode='w')
        writer_powerfactor = csv.writer(grf_dat_powerfactor, lineterminator='\n')

        thread = threading.Thread(target=MeasureThread, args=(e ,grid ,writer_active ,writer_apparent ,writer_reactive ,writer_voltage ,writer_current ,writer_powerfactor ,m_time ,dll, device_id, phases,))
        thread.start()
        time.sleep(1)
#        ts.sleep(1)
### <END>


        # For PV systems, this requires that Vmpp = Vin_nom and Pmpp = Prated.
        for v_in_label, v_in in v_in_targets.iteritems():
            ts.log('Starting test %s at v_in = %s' % (v_in_label, v_in))
            if pv is not None:
                pv.iv_curve_config(pmp=p_rated, vmp=v_in)
                pv.irradiance_set(1000.)

            """
            e) Enable constant power factor mode and set the EUT power factor to PFmin,inj.
            r) Repeat steps d) through o) for additional power factor settings: PFmin,ab, PFmid,inj, PFmid,ab.

            Only the user-selected PF setting will be tested.
            """
            for pf_test_name, pf_target in pf_targets.iteritems():
                # Start acquisition
                daq.data_capture(True)
                # Configure the data acquisition system
                ts.log('Starting data capture for pf = %s' % pf_target)
                dataset_filename = ('{0}_{1}'.format(v_in_label.upper(), pf_test_name.upper()))
                ts.log('------------{}------------'.format(dataset_filename))
                daq.sc['PF_TARGET'] = pf_target


###                if eut is not None:                             # Change because middleware is communicated using gridsim
                if grid is not None:                               # Change because middleware is communicated using gridsim
                    parameters = {'Ena': True, 'PF': pf_target}
                    ts.log('PF set: %s' % parameters)
###                    eut.fixed_pf(params=parameters)             # Change because middleware is communicated using gridsim
                    grid.fixed_pf(params=parameters)               # Change because middleware is communicated using gridsim
###                    pf_setting = eut.fixed_pf()                 # Change because middleware is communicated using gridsim
                    pf_setting = grid.fixed_pf()                   # Change because middleware is communicated using gridsim
                    ts.log('PF setting read: %s' % pf_setting)

                """
                f) Wait for steady state to be reached.

                Every time a parameter is stepped or ramped, measure and record the time domain current and voltage
                response for at least 4 times the maximum expected response time after the stimulus, and measure or
                derive, active power, apparent power, reactive power, and power factor.
                """
                step = 'Step F'
                daq.sc['event'] = step
                daq.data_sample()
                ts.log('Wait for steady state to be reached')
                ts.sleep(4*pf_response_time)

                """
                g) Step the EUT's active power to Pmin.
                """
                if pv is not None:
                    ts.log('Power step: setting PV simulator power to %s' % p_min)
                    step = 'Step G'
###                    q_initial = get_q_initial(daq=daq,step=step)
                    q_initial = get_q_initial(daq=daq, step=step,dll=dll,device_id=device_id,e=e)
                    pv.power_set(p_min)
###                    q_p_analysis = q_p_criteria(pf=pf_target, MSA_P=MSA_P, MSA_Q=MSA_Q, daq=daq, tr=pf_response_time, step=step, q_initial=q_initial)
                    q_p_analysis = q_p_criteria(pf=pf_target, MSA_P=MSA_P, MSA_Q=MSA_Q, daq=daq, tr=pf_response_time, step=step, q_initial=q_initial, e=e, dll=dll, device_id=device_id, start_time=start_time)
                    result_summary.write('%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n' %
                                         (q_p_analysis['Q_FINAL_PF'], q_p_analysis['Q_TR_PF'], pf_target,
                                          daq.sc['V_MEAS'], daq.sc['P_MEAS'], q_p_analysis['Q_FINAL'],
                                          daq.sc['Q_TARGET_MIN'], daq.sc['Q_TARGET_MAX'], step, dataset_filename))
                """
                h) Step the EUT's available active power to Prated.
                """
                if pv is not None:
                    ts.log('Power step: setting PV simulator power to %s' % p_rated)
                    step = 'Step H'
######                    q_initial = get_q_initial(daq=daq,step=step)
                    q_initial = get_q_initial(daq=daq,step=step,dll=dll,device_id=device_id,e=e)
                    pv.power_set(p_rated)
###                    q_p_analysis = q_p_criteria(pf=pf_target, MSA_P=MSA_P, MSA_Q=MSA_Q, daq=daq, tr=pf_response_time,
###                                                step=step, q_initial=q_initial)
                    q_p_analysis = q_p_criteria(pf=pf_target, MSA_P=MSA_P, MSA_Q=MSA_Q, daq=daq, tr=pf_response_time,
                                                step=step, q_initial=q_initial, e=e, dll=dll, device_id=device_id, start_time=start_time)
                    result_summary.write('%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n' %
                                         (q_p_analysis['Q_FINAL_PF'], q_p_analysis['Q_TR_PF'], pf_target,
                                          daq.sc['V_MEAS'], daq.sc['P_MEAS'], q_p_analysis['Q_FINAL'],
                                          daq.sc['Q_TARGET_MIN'], daq.sc['Q_TARGET_MAX'], step, dataset_filename))

                if grid is not None:

                    #   i) Step the AC test source voltage to (VL + av)
                    ts.log('Voltage step: setting Grid simulator voltage to %s' % (v_min + a_v))
                    step = 'Step I'
###                    q_initial = get_q_initial(daq=daq,step=step)
                    q_initial = get_q_initial(daq=daq,step=step,dll=dll,device_id=device_id,e=e)
###                    grid.voltage(v_min + a_v)                                                                                   # Convert to line voltage
                    grid.config_asymmetric_phase_angles(mag=[v_min + a_v, v_min + a_v, v_min + a_v], angle=[0.0, 120.0, -120.0])   # Convert to line voltage
                    grid.power_setVV(p_rated * 1, s_rated, p_ramp_rate)                                                            # <- Change to control from grid
###                    q_p_analysis = q_p_criteria(pf=pf_target, MSA_P=MSA_P, MSA_Q=MSA_Q, daq=daq, tr=pf_response_time,
###                                                step=step, q_initial=q_initial)
                    q_p_analysis = q_p_criteria(pf=pf_target, MSA_P=MSA_P, MSA_Q=MSA_Q, daq=daq, tr=pf_response_time,
                                                step=step, q_initial=q_initial, e=e, dll=dll, device_id=device_id, start_time=start_time)
                    result_summary.write('%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n' %
                                         (q_p_analysis['Q_FINAL_PF'], q_p_analysis['Q_TR_PF'], pf_target,
                                          daq.sc['V_MEAS'], daq.sc['P_MEAS'], q_p_analysis['Q_FINAL'],
                                          daq.sc['Q_TARGET_MIN'], daq.sc['Q_TARGET_MAX'], step, dataset_filename))

                    #   j) Step the AC test source voltage to (VH - av)
                    ts.log('Voltage step: setting Grid simulator voltage to %s' % (v_max - a_v))
                    step = 'Step J'
###                    q_initial = get_q_initial(daq=daq, step=step)
                    q_initial = get_q_initial(daq=daq,step=step,dll=dll,device_id=device_id,e=e)
###                    grid.voltage(v_max - a_v)                                                                                   # Convert to line voltage
                    grid.config_asymmetric_phase_angles(mag=[v_max - a_v, v_max - a_v, v_max - a_v], angle=[0.0, 120.0, -120.0])   # Convert to line voltage
###                    q_p_analysis = q_p_criteria(pf=pf_target, MSA_P=MSA_P, MSA_Q=MSA_Q, daq=daq, tr=pf_response_time,
###                                                step=step, q_initial=q_initial)
                    q_p_analysis = q_p_criteria(pf=pf_target, MSA_P=MSA_P, MSA_Q=MSA_Q, daq=daq, tr=pf_response_time,
                                                step=step, q_initial=q_initial, e=e, dll=dll, device_id=device_id, start_time=start_time)
                    result_summary.write('%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n' %
                                         (q_p_analysis['Q_FINAL_PF'], q_p_analysis['Q_TR_PF'], pf_target,
                                          daq.sc['V_MEAS'], daq.sc['P_MEAS'], q_p_analysis['Q_FINAL'],
                                          daq.sc['Q_TARGET_MIN'], daq.sc['Q_TARGET_MAX'], step, dataset_filename))

                    #   k) Step the AC test source voltage to (VL + av)
                    #   STD_CHANGE : We think at CanmetENERGY that this should be v_nom and not (v_min + a_v) before doing imbalance testing
                    ts.log('Voltage step: setting Grid simulator voltage to %s' % (v_nom))
                    step = 'Step K'
###                    q_initial = get_q_initial(daq=daq, step=step)
                    q_initial = get_q_initial(daq=daq,step=step,dll=dll,device_id=device_id,e=e)
###                    grid.voltage(v_nom)                                                                                         # Convert to line voltage
                    grid.config_asymmetric_phase_angles(mag=[v_nom, v_nom, v_nom], angle=[0.0, 120.0, -120.0])                     # Convert to line voltage
###                    q_p_analysis = q_p_criteria(pf=pf_target, MSA_P=MSA_P, MSA_Q=MSA_Q, daq=daq, tr=pf_response_time,
###                                                step=step, q_initial=q_initial)
                    q_p_analysis = q_p_criteria(pf=pf_target, MSA_P=MSA_P, MSA_Q=MSA_Q, daq=daq, tr=pf_response_time,
                                                step=step, q_initial=q_initial, e=e, dll=dll, device_id=device_id, start_time=start_time)
                    result_summary.write('%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n' %
                                         (q_p_analysis['Q_FINAL_PF'], q_p_analysis['Q_TR_PF'], pf_target,
                                          daq.sc['V_MEAS'], daq.sc['P_MEAS'], q_p_analysis['Q_FINAL'],
                                          daq.sc['Q_TARGET_MIN'], daq.sc['Q_TARGET_MAX'], step, dataset_filename))

                """
                l) For multiphase units, step the AC test source voltage to Case A from Table 23.

                                            Table 23 - Imbalanced Voltage Test Cases 12
                        +----------------------------------------------+-----------------------------------------------+
                        | Symmetrical Components                       | Phasor Components                             |
                        +----------------------------------------------+-----------------------------------------------+
                        | Zero Sequence | Positive Seq | Negative Seq  | Phase A      | Phase B       | Phase C        |
                        | Mag | Angle   | Mag | Angle  | Mag   | Angle | Mag   | Angle| Mag   | Angle | Mag   | Angle  |
                +-------+-----+---------+-----+--------+-------+-------+-------+------+-------+-------+-------+--------+
                |Case A | 0.0 | 0.0     | 1.0 | 0.0    | 0.07  | 0     | 1.070 | 0.0  | 0.967 | 123.6 | 0.967 | -123.6 |
                +-------+-----+---------+-----+--------+-------+-------+-------+------+-------+-------+-------+--------+
                |Case B | 0.0 | 0.0     | 1.0 | 0.0    | 0.09  | 180   | 0.910 | 0.0  | 1.048 | 115.7 | 1.048 | -115.7 |
                +-------+-----+---------+-----+--------+-------+-------+-------+------+-------+-------+-------+--------+
                |Case C | 0.0 | 0.0     | 1.0 | 0.0    | 0.05  | 0     | 1.050 | 0.0  | 0.976 | 122.5 | 0.976 | -122.5 |
                +-------+-----+---------+-----+--------+-------+-------+-------+------+-------+-------+-------+--------+
                |Case D | 0.0 | 0.0     | 1.0 | 0.0    | 0.05  | 180   | 0.950 | 0.0  | 1.026 | 117.6 | 1.026 | -117.6 |
                +-------+-----+---------+-----+--------+-------+-------+-------+------+-------+-------+-------+--------+

                For tests with imbalanced, three-phase voltages, the manufacturer shall state whether the EUT responds
                to individual phase voltages, or the average of the three-phase effective (RMS) values or the positive
                sequence of voltages. For EUTs that respond to individual phase voltages, the response of each
                individual phase shall be evaluated. For EUTs that response to the average of the three-phase effective
                (RMS) values mor the positive sequence of voltages, the total three-phase reactive and active power
                shall be evaluated.
                """
                if grid is not None:
                    ts.log('Voltage step: setting Grid simulator to case A (IEEE 1547.1-Table 23)')
                    step = 'Step L'
###                    q_initial = get_q_initial(daq=daq, step=step)
                    q_initial = get_q_initial(daq=daq,step=step,dll=dll,device_id=device_id,e=e)
                    grid.config_asymmetric_phase_angles(mag=[1.07*v_nom, 0.967*v_nom, 0.967*v_nom],
                                                        angle=[0., 123.6, -123.6])
###                    q_p_analysis = q_p_criteria(pf=pf_target, MSA_P=MSA_P, MSA_Q=MSA_Q, daq=daq, tr=pf_response_time,
###                                                step=step, q_initial=q_initial)
                    q_p_analysis = q_p_criteria(pf=pf_target, MSA_P=MSA_P, MSA_Q=MSA_Q, daq=daq, tr=pf_response_time,
                                                step=step, q_initial=q_initial, e=e, dll=dll, device_id=device_id, start_time=start_time)
                    result_summary.write('%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n' %
                                         (q_p_analysis['Q_FINAL_PF'], q_p_analysis['Q_TR_PF'], pf_target,
                                          daq.sc['V_MEAS'], daq.sc['P_MEAS'], q_p_analysis['Q_FINAL'],
                                          daq.sc['Q_TARGET_MIN'], daq.sc['Q_TARGET_MAX'], step, dataset_filename))



                """
                m) For multiphase units, step the AC test source voltage to VN.
                """

                if grid is not None:
                    ts.log('Voltage step: setting Grid simulator voltage to %s' % v_nom)
                    step = 'Step M'
###                    q_initial = get_q_initial(daq=daq,step=step)
                    q_initial = get_q_initial(daq=daq,step=step,dll=dll,device_id=device_id,e=e)
###                    grid.voltage(v_nom)                                                                        # Convert to line voltage
                    grid.config_asymmetric_phase_angles(mag=[v_nom, v_nom, v_nom], angle=[0.0, 120.0, -120.0])    # Convert to line voltage
###                    q_p_analysis = q_p_criteria(pf=pf_target, MSA_P=MSA_P, MSA_Q=MSA_Q, daq=daq, tr=pf_response_time,
###                                                step=step, q_initial=q_initial)
                    q_p_analysis = q_p_criteria(pf=pf_target, MSA_P=MSA_P, MSA_Q=MSA_Q, daq=daq, tr=pf_response_time,
                                                step=step, q_initial=q_initial, e=e, dll=dll, device_id=device_id, start_time=start_time)
                    result_summary.write('%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n' %
                                         (q_p_analysis['Q_FINAL_PF'], q_p_analysis['Q_TR_PF'], pf_target,
                                          daq.sc['V_MEAS'], daq.sc['P_MEAS'], q_p_analysis['Q_FINAL'],
                                          daq.sc['Q_TARGET_MIN'], daq.sc['Q_TARGET_MAX'], step, dataset_filename))

                """
                n) For multiphase units, step the AC test source voltage to Case B from Table 23.
                """
                if grid is not None:
                    ts.log('Voltage step: setting Grid simulator to case B (IEEE 1547.1-Table 23)')
                    step = 'Step N'
###                    q_initial = get_q_initial(daq=daq,step=step)
                    q_initial = get_q_initial(daq=daq,step=step,dll=dll,device_id=device_id,e=e)
                    grid.config_asymmetric_phase_angles(mag=[0.91*v_nom, 1.048*v_nom, 1.048*v_nom],
                                                        angle=[0., 115.7, -115.7])
###                    q_p_analysis = q_p_criteria(pf=pf_target, MSA_P=MSA_P, MSA_Q=MSA_Q, daq=daq, tr=pf_response_time,
###                                                step=step, q_initial=q_initial)
                    q_p_analysis = q_p_criteria(pf=pf_target, MSA_P=MSA_P, MSA_Q=MSA_Q, daq=daq, tr=pf_response_time,
                                                step=step, q_initial=q_initial, e=e, dll=dll, device_id=device_id, start_time=start_time)
                    result_summary.write('%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n' %
                                         (q_p_analysis['Q_FINAL_PF'], q_p_analysis['Q_TR_PF'], pf_target,
                                          daq.sc['V_MEAS'], daq.sc['P_MEAS'], q_p_analysis['Q_FINAL'],
                                          daq.sc['Q_TARGET_MIN'], daq.sc['Q_TARGET_MAX'], step, dataset_filename))

                """
                o) For multiphase units, step the AC test source voltage to VN
                """
                if grid is not None:
                    ts.log('Voltage step: setting Grid simulator voltage to %s' % v_nom)
                    step = 'Step O'
###                    q_initial = get_q_initial(daq=daq, step=step)
                    q_initial = get_q_initial(daq=daq,step=step,dll=dll,device_id=device_id,e=e)
###                    grid.voltage(v_nom)                                                                       # Convert to line voltage
                    grid.config_asymmetric_phase_angles(mag=[v_nom, v_nom, v_nom], angle=[0.0, 120.0, -120.0])   # Convert to line voltage
###                    q_p_analysis = q_p_criteria(pf=pf_target, MSA_P=MSA_P, MSA_Q=MSA_Q, daq=daq, tr=pf_response_time,
###                                                step=step, q_initial=q_initial)
                    q_p_analysis = q_p_criteria(pf=pf_target, MSA_P=MSA_P, MSA_Q=MSA_Q, daq=daq, tr=pf_response_time,
                                                step=step, q_initial=q_initial, e=e, dll=dll, device_id=device_id, start_time=start_time)
                    result_summary.write('%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n' %
                                         (q_p_analysis['Q_FINAL_PF'], q_p_analysis['Q_TR_PF'], pf_target,
                                          daq.sc['V_MEAS'], daq.sc['P_MEAS'], q_p_analysis['Q_FINAL'],
                                          daq.sc['Q_TARGET_MIN'], daq.sc['Q_TARGET_MAX'], step, dataset_filename))
                """
                p) Disable constant power factor mode. Power factor should return to unity.
                """
###                if eut is not None:                                # Change because middleware is communicated using gridsim
                if grid is not None:                                  # Change because middleware is communicated using gridsim
                    parameters = {'Ena': False, 'PF': 1.0}
                    ts.log('PF set: %s' % parameters)
###                    eut.fixed_pf(params=parameters)                # Change because middleware is communicated using gridsim
                    grid.fixed_pf(params=parameters)                  # Change because middleware is communicated using gridsim
###                    pf_setting = eut.fixed_pf()                    # Change because middleware is communicated using gridsim
                    pf_setting = grid.fixed_pf()                      # Change because middleware is communicated using gridsim
                    ts.log('PF setting read: %s' % pf_setting)
                    daq.sc['event'] = 'Step P'
                    daq.data_sample()
                    ts.sleep(4*pf_response_time)
                    daq.sc['event'] = 'T_settling_done'
                    daq.data_sample()


                """
                q) Verify all reactive/active power control functions are disabled.
                """
###                if eut is not None:                                # Change because middleware is communicated using gridsim
                if grid is not None:                                  # Change because middleware is communicated using gridsim
                    ts.log('Reactive/active power control functions are disabled.')
                    # TODO Implement ts.prompt functionality?
                    #meas = eut.measurements()
                    #ts.log('EUT PF is now: %s' % (data.get('AC_PF_1')))
                    #ts.log('EUT Power: %s, EUT Reactive Power: %s' % (meas['W'], meas['VAr']))

                # result params
                result_params = {
                    'plot.title': ts.name,
                    'plot.x.title': 'Time (sec)',
                    'plot.x.points': 'TIME',
                    'plot.y.points': '{}, PF_TARGET'.format(','.join(str(x) for x in get_measurement_label('PF'))),
                    'plot.y.title': 'Power Factor',
                    'plot.y2.points': '{}'.format(','.join(str(x) for x in get_measurement_label('I'))),
                    'plot.y2.title': 'Current (A)'
                }

                ts.log('Sampling complete')
                dataset_filename = dataset_filename + ".csv"
                daq.data_capture(False)
                ds = daq.data_capture_dataset()
                ts.log('Saving file: %s' % dataset_filename)
                ds.to_csv(ts.result_file_path(dataset_filename))
                result_params['plot.title'] = dataset_filename.split('.csv')[0]
                ts.result_file(dataset_filename, params=result_params)

                result = script.RESULT_COMPLETE

### Graph drawing for FREA original gnuplot
### <START>
        e.stop_event.set()               # WT3000 compatible
        thread.join()                    # WT3000 compatible

        grf_dat_active.close()           # <- Since the graph is not displayed, it is added
        grf_dat_apparent.close()         # <- Since the graph is not displayed, it is added
        grf_dat_reactive.close()         # <- Since the graph is not displayed, it is added
        grf_dat_voltage.close()          # <- Since the graph is not displayed, it is added
        grf_dat_current.close()          # <- Since the graph is not displayed, it is added
        grf_dat_powerfactor.close()      # <- Since the graph is not displayed, it is added

        rtn = dll.TmFinish(device_id)                                  # WT3000 compatible
        if rtn == 0:                                                   # WT3000 compatible
            ts.log('@@@(WT3000) DISCONNECT OK: %s' % (rtn))            # WT3000 compatible
        else:                                                          # WT3000 compatible
            ts.log('@@@(WT3000) DISCONNECT NG: %s' % (rtn))            # WT3000 compatible
### <END>

    except script.ScriptFail, e:
        reason = str(e)
        if reason:
            ts.log_error(reason)
    finally:
        ts.log('--------------Finally START----------------')

        # TEST Logic start
        print_varsize()
        # TEST Logic end

        if grid is not None:
            grid.fixed_pf(params={'Ena': False, 'PF': 1.0})        # Change because middleware is communicated using gridsim
            grid.close()
        if pv is not None:
            if p_rated is not None:
                pv.power_set(p_rated)
            pv.close()
        if daq is not None:
            daq.close()
        if daq_wf is not None:                                     # DL850E compatible
            daq_wf.data_capture(False)                             # DL850E compatible
            time.sleep(1)                                          # DL850E compatible
            daq_wf.close()                                         # DL850E compatible
####        if eut is not None:                                    # Change because middleware is communicated using gridsim
####            eut.fixed_pf(params={'Ena': False, 'PF': 1.0})     # Change because middleware is communicated using gridsim
####            eut.close()                                        # Change because middleware is communicated using gridsim
        if rs is not None:                                         # Change because middleware is communicated using gridsim
            rs.close()
        if chil is not None:
            chil.close()

        if result_summary is not None:
            result_summary.close()

        # create result workbook
        excelfile = ts.config_name() + '.xlsx'
        rslt.result_workbook(excelfile, ts.results_dir(), ts.result_dir())
        ts.result_file(excelfile)





### Graph drawing for FREA original gnuplot
### <START>
        ### C_power_factor.png
        gnuplot =  subprocess.Popen('gnuplot', shell=True, stdin=PIPE, stdout=PIPE, stderr=PIPE, universal_newlines=True)

        gnuplot.stdin.write('set xlabel "Time (seconds)"\n')
        gnuplot.stdin.write('set ylabel "Active Power(kW), Apparent power(kVA), Reactive power(kvar)"\n')

###        set_over = round(float(s_rated)/1000) + (round(float(s_rated)/1000) * 0.1)
###        set_under = ((round(float(s_rated)/1000)) * -1) - ((round(float(s_rated)/1000)) * -0.1)
###        set_over = round(float(s_rated)/1000)
###        set_under = ((round(float(s_rated)/1000)) * -1)
        set_over = (round(float(s_rated)/1000)) * 1.25
        set_under = (((round(float(s_rated)/1000)) * -1)) * 1.25

        set_cmd = "set yrange [" + str(set_under) + ":" + str(set_over) + "]\n"
        gnuplot.stdin.write(set_cmd)

#        gnuplot.stdin.write('set ytics nomirror\n')
        gnuplot.stdin.write('set y2label "Power factor(%)"\n')
        gnuplot.stdin.write('set y2range [-6.25:6.25]\n')
        gnuplot.stdin.write('set y2tics\n')
#        gnuplot.stdin.write('set y2tics 0, 0.25\n')
#        gnuplot.stdin.write('set y2tics "+0.1" 1.1, "0.0" 1.0, "-0.1" 0.9\n')

        gnuplot.stdin.write('set term png size 1000, 1000\n')
        gnuplot.stdin.write('set grid lw 1\n')
###        gnuplot.stdin.write('set key box\n')

        graph_out = ts.results_dir() + "\C_power_factor.png"
        ts.log('graph_out = %s' % (graph_out))
        graph_cmd = "set output " + "'" + graph_out + "'\n"
        ts.log('graph_cmd = %s' % (graph_cmd))
        gnuplot.stdin.write(graph_cmd)

        graph_cmd = "set datafile separator ','\n"
        gnuplot.stdin.write(graph_cmd)

        # Active power
        graph_cmd = "plot " + "'" + grf_dat_file_active + "' ti 'Active power Point' with linespoints pt 7 lc rgb 'orange' axis x1y1"
        ts.log('graph_cmd = %s' % (graph_cmd))

        # Apparent power
        graph_cmd = graph_cmd + ", " + "'" + grf_dat_file_apparent + "' ti 'Apparent power Point' with linespoints pt 7 lc rgb 'gray' axis x1y1"
        ts.log('graph_cmd = %s' % (graph_cmd))

        # Reactive power
        graph_cmd = graph_cmd + ", " + "'" + grf_dat_file_reactive + "' ti 'Reactive power Point' with linespoints pt 7 lc rgb 'royalblue' axis x1y1"
        ts.log('graph_cmd = %s' % (graph_cmd))

        # Power factor
        graph_cmd = graph_cmd + ", " + "'" + grf_dat_file_powerfactor + "' ti 'Power factor Point' with linespoints pt 7 lc rgb 'gold' axis x1y2\n"
        ts.log('graph_cmd = %s' % (graph_cmd))

        gnuplot.stdin.write(graph_cmd)

        ### Return setting
        gnuplot.stdin.write('set terminal windows\n')
        gnuplot.stdin.write('set output\n')
### <END>





    return result

def run(test_script):

    try:
        global ts
        ts = test_script
        rc = 0
        result = script.RESULT_COMPLETE

        ts.log_debug('')
        ts.log_debug('**************  Starting %s  **************' % (ts.config_name()))
        ts.log_debug('Script: %s %s' % (ts.name, ts.info.version))
        ts.log_active_params()

        result = test_run()

        ts.result(result)
        if result == script.RESULT_FAIL:
            rc = 1

    except Exception, e:
        ts.log_error('Test script exception: %s' % traceback.format_exc())
        rc = 1

    sys.exit(rc)

info = script.ScriptInfo(name=os.path.basename(__file__), run=run, version='1.1.2')

# Power factor parameters
# PF - the commanded power factor
# PFmin,inj - minimum injected power factor, 0.90 for both Category A and B equipment
# PFmin,ab - minimum absorbed power factor, 0.97 for Catergory A, 0.90 for Catergory B
# PFmid,inj - a power factor setting chosen to be less than 1 and greater than PFmin,inj
# PFmid,ab - a power factor setting chosen to be less than 1 and greater than PFmin,ab

### FREA ADD
### <START>
info.param_group('aist', label='AIST Parameters', glob=True)
info.param('aist.script_version', label='Script Version', default='3.0.0')
info.param('aist.library_version', label='Library Version (gridsim_frea_simulator)', default='5.0.0')
### <END>

info.param_group('cpf', label='Test Parameters')
info.param('cpf.pf_min_inj', label='PFmin,inj activation', default='Enabled', values=['Disabled', 'Enabled'])
info.param('cpf.pf_min_inj_value', label='PFmin,inj (Overexcited) (negative value, for SunSpec sign convention)',
           default=-0.90, active='cpf.pf_min_inj', active_value=['Enabled'])

info.param('cpf.pf_mid_inj', label='PFmid,inj activation', default='Enabled', values=['Disabled', 'Enabled'])
info.param('cpf.pf_mid_inj_value', label='PFmid,inj value (-1.00 < PFmid,inj < PFmin,inj):', default=-0.95,
           active='cpf.pf_mid_inj', active_value=['Enabled'])

info.param('cpf.pf_min_ab', label='PFmin,ab activation', default='Enabled', values=['Disabled', 'Enabled'])
info.param('cpf.pf_min_ab_value', label='PFmin,ab (Underexcited)', default=0.90,
           active='cpf.pf_min_ab', active_value=['Enabled'])

info.param('cpf.pf_mid_ab', label='PFmid,ab', default='Enabled', values=['Disabled', 'Enabled'])
info.param('cpf.pf_mid_ab_value', label='PFmid,ab value (PFmin,ab < PFmid,ab < 1.00):', default=0.95,
           active='cpf.pf_mid_ab', active_value=['Enabled'])
info.param('cpf.v_in_nom', label='Test V_in_nom', default='Enabled', values=['Disabled', 'Enabled'])
info.param('cpf.v_in_min', label='Test V_in_min', default='Enabled', values=['Disabled', 'Enabled'])
info.param('cpf.v_in_max', label='Test V_in_max', default='Enabled', values=['Disabled', 'Enabled'])

info.param('cpf.ip_addr', label='WT3000 IP Address for NonExCon', default='192.168.127.200')      # WT3000 compatible
info.param('cpf.m_time', label='Measurement time interval (secs)', default=0.5)                   # WT3000 compatible

# EUT parameters
# Prated - output power rating (W)
# P'rated - for EUT's that can sink power, output power rating while sinking power (W)
# Srated - apparent power rating (VA)
# Vin_nom - for an EUT with an electrical input, nominal input voltage (V)
# Vin_min - for an EUT with an electrical input, minimum input voltage (V)
# Vin_max - for an EUT with an electrical input, maximum input voltage (V)
# VN - nominal output voltage (V)
# VL - minimum output voltage in the continous operating region (V)
# VH - maximum output voltage in the continous operating region (V)
# Pmin - minimum active power (W)
# P'min - for EUT's that can sink power, minimum active power while sinking power(W)
# Qmax,inj - maximum absorbed reactive power (VAr)
# Qmax,inj - minimum absorbed reactive power (VAr)

info.param_group('eut', label='CPF - EUT Parameters', glob=True)
info.param('eut.cat', label='DER Category (Distribution System Stability)', default='Category III (inverter-based)',
           values=['Category I (synchronous generator)', 'Category II (fuel cell)', 'Category III (inverter-based)'])

info.param('eut.cat2', label='DER Category (Bulk System Stability)', default='Category B',
           values=['Category A', 'Category B'],
           active='eut.cat', active_value=['Category II (fuel cell)'])

info.param('eut.sink_power', label='Can the EUT sink power, e.g., is it a battery system', default='No',
           values=['No', 'Yes'])

info.param('eut.p_rated', label='Prated: Output power rating (W)', default=3000.0)
info.param('eut.p_rated_prime', label='P\'rated: Output power rating while sinking power (W) (negative)',
           default=-3000.0, active='eut.sink_power', active_value=['Yes'])

info.param('eut.s_rated', label='Srated: apparent power rating (VA)', default=3000.0)

info.param('eut.v_in_nom', label='V_in_nom: Nominal input voltage (Vdc)', default=400)
info.param('eut.v_in_min', label='V_in_min: Nominal input voltage (Vdc)', default=200)
info.param('eut.v_in_max', label='V_in_max: Nominal input voltage (Vdc)', default=600)
info.param('eut.v_nom', label='V_nom: Nominal voltage output (V)', default=240.0)
info.param('eut.v_low', label='Minimum output voltage in the continous operating region (V)', default=0.88*240)
info.param('eut.v_high', label='Maximum output voltage in the continous operating region (V)', default=1.1*240)

info.param('eut.p_min', label='Pmin: Minimum active power (W)', default=0.2*3000.0)
info.param('eut.p_min_prime', label='P\'min: minimum active power while sinking power(W) (negative)',
           default=-0.2*3000.0, active='eut.sink_power', active_value=['Yes'])
#info.param('eut.imbalance_resp', label='Imbalance response. EUT responds to:', default='individual phase voltages',
#           values=['individual phase voltages', 'average of the three-phase effective (RMS)',
#                   'the positive sequence of voltages'])

info.param('eut.phases', label='Phases', values=['Single phase', 'Split phase', 'Three phase'], default='Three phase')

info.param('eut.pf_response_time', label='PF Response Time (secs)', default=1.0)
info.param('eut.ramp_rate', label='Power Ramp Rate (0.01%/s)', default=0)          # Change because middleware is communicated using gridsim

###der.params(info)
das.params(info)
das.params(info, 'das_wf', 'Data Acquisition (Waveform)')
gridsim.params(info)
loadsim.params(info)
pvsim.params(info)
hil.params(info)


def script_info():
    
    return info


if __name__ == "__main__":

    # stand alone invocation
    config_file = None
    if len(sys.argv) > 1:
        config_file = sys.argv[1]

    params = None

    test_script = script.Script(info=script_info(), config_file=config_file, params=params)
    test_script.log('log it')

    run(test_script)


