import os,sys,inspect
import labrad
import labrad.units as U
import numpy as np
import matplotlib.pyplot as plt

NUM_DAC_CARDS = 2
NUM_ADC_CARDS = 2

currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0,parentdir)

def test1():
    cxn = labrad.connect()
    ###TEST 1 -- connect to instrument###
    try:
        testname = "CONNECT TO DEVICE"
        print("TEST 1: " + testname)
        da = cxn.dac_adc_giga
        da.select_device()
        print("TEST 1: " + testname + " PASSED\n")
        return da
    except Exception as e:
        print("COULD NOT CONNECT TO GIGADAQ, ERROR:", e)
        exit()

def test2(da):
    ###Test 2 --- check serial communication###
    try:
        testname = "TEST SERIAL COMMUNICATION"
        print("TEST 2: " + testname)
        nop = da.query('NOP')
        if nop == 'NOP':
            print("TEST 2: " + testname + " PASSED\n")
        else:
            print("serial not working, test 2 failed")
            exit()
    except Exception as e:
        print("Error:",e)
        exit()
        
def test3(da):     
    ###Test 3 --- get instrument ID###
    try:
        testname = "GET INSTRUMENT ID"
        print("TEST 3: " + testname)
        id = da.id()
        serial = da.query('SERIAL_NUMBER')
        env = da.query('GET_ENVIRONMENT')
        if id == 'DAC-ADC_AD7734-AD5791':
            print("DEVICE SERIAL NUMBER: ", serial)
            print("DEVICE ENVIRONMENT: ", env)
            print("TEST 3: ID PASSED\n")
        else:
            print("ID is set incorrectly, test 3 failed")
            exit()
    except Exception as e:
        print("Error:",e)
        exit()
        
def test4(da):       
    ###Test 4 --- test SPI ADC###
    try:
        testname = "ADC SPI"
        print("TEST 4: " + testname)
        revisionRegisterIdeal = 34
        for i in range(NUM_ADC_CARDS):
            revReg = int(da.query('GET_REVISION_REG,'+str(i)))
            if revReg != revisionRegisterIdeal:
                print("TEST 4 " + testname + " FAILED")
                exit()
        print("TEST 4 " + testname + " PASSED\n")
    except Exception as e:
        print("Error: ", e)
        exit()

def test5(da):        
    ###Test 5 --- test set/read functionality ###
    try:
        testname = "SET/READ"
        print("TEST 5: " + testname)
        output = da.initialize()
        if output != 'INITIALIZATION COMPLETE':
            print("TEST 5: " + testname + " FAILED")
            exit()
        
        #Check if DACs are all within 5mV of 0V
        for i in range(4*NUM_ADC_CARDS):
            voltage = float(da.read_voltage(i))
            if abs(voltage) >= 0.1:
                print(voltage)
                print("TEST 5: " + testname + " FAILED")
                exit()
         
        #Check if DACs are within 10mV of setpt
        for val in np.linspace(-9.5, 9.5, 21):
            for j in range(4*NUM_DAC_CARDS):
                da.set_voltage(j, val)
                output = float(da.read_voltage(j))
                #print(abs(output - val))
                if abs(output - val) >= 0.2:
                    print(output)
                    print("TEST 5: " + testname + " FAILED")
                    exit()
        print("TEST 5 " + testname + " PASSED\n")
    except Exception as e:
        print("Error:", e)
        exit()

def test6(da):        
    ###Test 6 --- test ADC calibration functionality###
    try:
        testname = "CALIBRATE"
        print("TEST 6: " + testname)
        output = da.initialize()
        if output != 'INITIALIZATION COMPLETE':
            print("TEST 6: " + testname + " FAILED")
            exit()
        
        #Check if DACs are all within 5mV of 0V
        for i in range(4*NUM_ADC_CARDS):
            voltage = float(da.read_voltage(i))
            if abs(voltage) >= 0.005:
                print("TEST 6: " + testname + " FAILED")
                exit()
         
        #change the conversion time for each ADC
        for i in range(4*NUM_ADC_CARDS):
            convTime = int(da.set_conversiontime(i, 2600))
            if convTime != 2602:
                print("TEST 6: " + testname + " FAILED")
                exit()
                
        calib = da.adc_ch_zero_sc_calibration()
        if calib != "CALIBRATION_FINISHED":
            print("TEST 6: " + testname + " FAILED")
            exit()
         
        #Check if DACs are all within 50uV of 0V
        for i in range(4*NUM_ADC_CARDS):
            voltage = float(da.read_voltage(i))
            if abs(voltage) >= 0.00005:
                print("TEST 6: " + testname + " FAILED")
                exit()
         
        #Check if DACs are within 10mV of 10V -- includes negative values from non clipped overrange 
        for i in range(4*NUM_DAC_CARDS):
            da.set_voltage(i, 10.0)
            output = float(da.read_voltage(i))
            if abs(abs(output) - 10) >= 0.1:
                print("TEST 6: " + testname + " FAILED")
                exit()
        
        ### Full scale calibration ###
        calib = da.adc_ch_full_sc_calibration()
        if calib != "CALIBRATION_FINISHED":
            print("TEST 6: " + testname + " FAILED")
            exit()
                
        #Check if DACs are within 100uV of 10V -- includes negative values from non clipped overrange 
        for i in range(4*NUM_DAC_CARDS):
            da.set_voltage(i, 10.0)
            output = float(da.read_voltage(i))
            if abs(abs(output) - 10) >= 0.0001:
                print("TEST 6: " + testname + " FAILED")
                exit()
        print("TEST 6: " + testname + " PASSED\n")
    except Exception as e:
        print("Error:", e)
        exit()

    ### Test Basic Buffer Ramp functionality ###

def test7(da):    
    try: 
        testname = "SINGLE CHANNEL DAC LED BUFFER RAMP"
        print("TEST 7: " + testname)
        
        for dacNum in range(NUM_DAC_CARDS*4):
            fig, axes = plt.subplots(2,4, figsize=(12,6))
            fig2,axes2 = plt.subplots(2,4, figsize=(12,6))
            
            axes = axes.flatten()
            axes2 = axes2.flatten()
            
            convTimes = np.linspace(82,2600,8)
            rampTimes = [convTimes[i] + 200 + 100 for i in range(len(convTimes))]
            
            x_t = np.linspace(-9.999, 9.999, 1001)
            lsb = 20.0/2**16
            
            x_errs = []
            x_dat = []
            
            for j, convTime in enumerate(convTimes):
                print("Running ramp for DAC"+str(dacNum)+" with conversion time "+str(convTime))
                for i in range(4*NUM_ADC_CARDS):
                    da.set_conversiontime(i, convTime)
                
                print(f"da.dac_led_buffer_ramp([{dacNum}], [{dacNum}], [-9.999], [9.999], 1001, {rampTimes[j]}, 100)")
                
                x = da.dac_led_buffer_ramp([dacNum], [dacNum], [-9.999], [9.999], 1001, rampTimes[j], 100)
                x_dat.append(x)
                
                x_err = (x - x_t)/lsb
                x_errs.append(x_err)
                
            for k, ax in enumerate(axes):
                ax.plot(x_errs[k][0])
                convTimeString = format(convTimes[k], "0.3f")
                ax.set_title("Conversion Time: " + str(convTimeString) + "$\mu$s", size = 12)
                ax.set_xlabel("Data Index [i]", size = 12)
                ax.set_ylabel("Error [16-bit LSB]", size=12)
                
            for k, ax in enumerate(axes2):
                ax.plot(x_dat[k][0])
                convTimeString = format(convTimes[k], "0.3f")
                ax.set_title("Conversion Time: " + str(convTimeString) + "$\mu$s", size = 12)
                ax.set_xlabel("Data Index [i]", size = 12)
                ax.set_ylabel("ADC0 [V]", size=12)
            
            fig.tight_layout()
            fig2.tight_layout()
            
            #plt.show()
            fig.savefig(currentdir+"\\Unit Test Plots\\dac_led_single_channel_ramp_errors_DAC"+str(dacNum)+".png")
            fig2.savefig(currentdir+"\\Unit Test Plots\\dac_led_single_channel_ramp_data_DAC"+str(dacNum)+".png")
        print("TEST 7: " + testname + " PASSED")
    except Exception as e:
        print("Error: ", e)
        exit()
  
def test8(da):  
    ### Test Basic Buffer Ramp functionality ###
    try: 
        testname = "MULTI CHANNEL DAC LED BUFFER RAMP"
        print("TEST 8: " + testname)
        
        fig, axes = plt.subplots(2,3, figsize=(18,10))
        
        axes = axes.flatten()
        
        convTimes = np.linspace(82,2600,6)
        realConvTimes = []
        delays = []
        
        datas = []
        
        for j, convTime in enumerate(convTimes):
            realConvTime = 0
            for i in range(4*NUM_ADC_CARDS):
                if i == 0:
                    realConvTime = float(da.set_conversiontime(i, convTime))
                    realConvTimes.append(realConvTime)
                else:
                    da.set_conversiontime(i, convTime)
            
            rampTime = 4.0*realConvTime + 181 + 100
            delays.append(rampTime)
            
            data = da.dac_led_buffer_ramp([0,1,2,3,4,5,6,7], [0,1,2,3,4,5,6,7], [-9, -8, -7, -6, -5, -4, -3, -2], [9, 8, 7, 6, 5, 4, 3, 2], 201, rampTime, 100)
            datas.append(data)
            
        for k, ax in enumerate(axes):
            for l, dataset in enumerate(datas[k]):
                ax.plot(dataset, label = 'ADC'+str(l))
            rampTimeString = format(delays[k], "0.3f")
            convTimeString = format(realConvTimes[k], "0.1f")
            ax.set_title("DAC Delay: " + str(rampTimeString) + "$\mu$s, Conversion Time: " + str(convTimeString) + "$\mu$s", size = 12)
            ax.set_xlabel("Data Index [i]", size = 12)
            ax.set_ylabel("ADC [V]", size=12)
            ax.legend()
                 
        fig.tight_layout()
        
        #plt.show()
        fig.savefig(currentdir+"\\Unit Test Plots\\dac_led_multi_channel_ramp.png")
        print("TEST 8: " + testname + " PASSED\n")
    except Exception as e:
        print("Error: ", e)
        exit()

def test9(da):        
    ### Test Basic Buffer Ramp functionality ###
    
    try: 
        testname = "SINGLE CHANNEL TIME SERIES  BUFFER RAMP"
        print("TEST 9: " + testname)
        
        for dacNum in range(NUM_DAC_CARDS*4):
            fig, axes = plt.subplots(2,4, figsize=(18,10))
            
            axes = axes.flatten()
            
            convTimes = np.linspace(82,2600,8)
            realConvTimes = []
              
            datas = []
            
            for j, convTime in enumerate(convTimes):
                realConvTime = 0
                for i in range(4*NUM_ADC_CARDS):
                    if i == 0:
                        realConvTime = float(da.set_conversiontime(i, convTime))
                        realConvTimes.append(realConvTime)
                    else:
                        da.set_conversiontime(i, convTime)
                
                print("Running ramp with DAC"+str(dacNum)+" with conversion time " + str(realConvTime))
                data = da.time_series_buffer_ramp([dacNum], [dacNum], [-9], [9], 10001, 60, int(realConvTime) + 310)
                datas.append(data)
                
                
            for k, ax in enumerate(axes):
                ax.plot(datas[k][0])
                convTimeString = format(realConvTimes[k], "0.3f")
                ax.set_title("Conversion Time: " + str(convTimeString) + "$\mu$s", size = 12)
                ax.set_xlabel("Data Index [i]", size = 12)
                ax.set_ylabel("ADC [V]", size=12)
                ax.set_ylim(-9.5,9.5)
                
            fig.suptitle("DAC Update Period: 60$\mu$s, Total Ramp Time = 600ms, $\Delta V$ = 1.8mV", fontsize=18)
            fig.tight_layout()
            
            #plt.show()
            fig.savefig(currentdir+"\\Unit Test Plots\\time_series_single_channel_ramp_DAC"+str(dacNum)+".png")
        print("TEST 9: " + testname + " PASSED\n")
    except Exception as e:
        print("Error: ", e)
        exit()
    
def test10(da):    
    ### Test Basic Buffer Ramp functionality ###
    try: 
        testname = "MULTI CHANNEL TIME SERIES BUFFER RAMP"
        print("TEST 10: " + testname)
        
        fig, axes = plt.subplots(2,3, figsize=(18,10))
        
        axes = axes.flatten()
        
        convTimes = np.linspace(82,2600,6)
        realConvTimes = []
        
        datas = []
        
        for j, convTime in enumerate(convTimes):
            realConvTime = 0
            for i in range(4*NUM_ADC_CARDS):
                if i == 0:
                    realConvTime = float(da.set_conversiontime(i, convTime))
                    realConvTimes.append(realConvTime)
                else:
                    da.set_conversiontime(i, convTime)
            
            data = da.time_series_buffer_ramp([0,1,2,3,4,5,6,7], [0,1,2,3,4,5,6,7], [-9, -8, -7, -6, -5, -4, -3, -2], [9, 8, 7, 6, 5, 4, 3, 2], 10001, 250, 4*round(realConvTime) + 310)
            datas.append(data)
            
        for k, ax in enumerate(axes):
            for l, dataset in enumerate(datas[k]):
                ax.plot(dataset, label = 'ADC'+str(l))
            convTimeString = format(realConvTimes[k], "0.1f")
            ax.set_title("Conversion Time: " + str(convTimeString) + "$\mu$s", size = 12)
            ax.set_xlabel("Data Index [i]", size = 12)
            ax.set_ylabel("ADC [V]", size=12)
            ax.legend()
          
        fig.suptitle("DAC Update Period: 250$\mu$s, Total Ramp Time = 2.5s, $\Delta V$ = 1.8mV", fontsize=18)  
        fig.tight_layout()
        #plt.show()
       
        fig.savefig(currentdir+"\\Unit Test Plots\\time_series_multi_channel_ramp.png")
        print("TEST 10: " + testname + " PASSED\n")
    except Exception as e:
        print("Error: ", e)
        exit()
    
def test11(da):    
    ### Test Basic Buffer Ramp functionality ###
    try: 
        testname = "2D DAC LED BUFFER RAMP NO RETRACE"
        print("TEST 11: " + testname)
        
        fig, axes = plt.subplots(2,3, figsize=(18,8))
        
        axes = axes.flatten()
        
        convTimes = np.linspace(82,2600,6)
        realConvTimes = []
        delays = []
        
        datas = []
        
        for j, convTime in enumerate(convTimes):
            realConvTime = 0
            for i in range(4*NUM_ADC_CARDS):
                if i == 0:
                    realConvTime = float(da.set_conversiontime(i, convTime))
                    realConvTimes.append(realConvTime)
                else:
                    da.set_conversiontime(i, convTime)
            
            rampTime = 2*realConvTime + 181 + 100
            delays.append(rampTime)
            
            data = da.dac_led_buffer_ramp_2d([0, 4], [1, 5], [-2.0, -1.0], [4.0, 2.0], [4.0, 2.0], 201, 11, False, False, 1, rampTime, 100.0)
            datas.append(data)
            
        for k, ax in enumerate(axes):
            adc_nums = [0, 1, 4, 5]
            for l, dataset in enumerate(datas[k]):
                ax.plot(dataset, label = 'ADC'+str(adc_nums[l]))
            rampTimeString = format(delays[k], "0.3f")
            convTimeString = format(realConvTimes[k], "0.1f")
            ax.set_title("DAC Delay: " + str(rampTimeString) + "$\mu$s, Conversion Time: " + str(convTimeString) + "$\mu$s", size = 12)
            ax.set_xlabel("Data Index [i]", size = 12)
            ax.set_ylabel("ADC [V]", size=12)
            ax.legend()
                 
        fig.tight_layout()
        #fig.suptitle("DAC Led 2D Buffer Ramps -- NO RETRACE", fontsize=18)  
        
        #plt.show()
        fig.savefig(currentdir+"\\Unit Test Plots\\dac_led_multi_channel_2d_ramp.png")
        print("TEST 11: " + testname + " PASSED\n")
    except Exception as e:
        print("Error: ", e)
        exit()
 
def test12(da): 
    ### Test Basic Buffer Ramp functionality ###
    try: 
        testname = "2D DAC LED BUFFER RAMP WITH RETRACE"
        print("TEST 12: " + testname)
        
        fig, axes = plt.subplots(2,3, figsize=(18,8))
        
        axes = axes.flatten()
        
        convTimes = np.linspace(82,2600,6)
        realConvTimes = []
        delays = []
        
        datas = []
        
        for j, convTime in enumerate(convTimes):
            realConvTime = 0
            for i in range(4*NUM_ADC_CARDS):
                if i == 0:
                    realConvTime = float(da.set_conversiontime(i, convTime))
                    realConvTimes.append(realConvTime)
                else:
                    da.set_conversiontime(i, convTime)
            
            rampTime = 2*realConvTime + 181 + 100
            delays.append(rampTime)
            
            data = da.dac_led_buffer_ramp_2d([0,4], [1,5], [-2, -1], [4, 2], [4, 2], 201, 11, True, False, 1, rampTime, 100)
            datas.append(data)
            
        for k, ax in enumerate(axes):
            adc_nums = [0, 1, 4, 5]
            for l, dataset in enumerate(datas[k]):
                ax.plot(dataset, label = 'ADC'+str(adc_nums[l]))
            rampTimeString = format(delays[k], "0.3f")
            convTimeString = format(realConvTimes[k], "0.1f")
            ax.set_title("DAC Delay: " + str(rampTimeString) + "$\mu$s, Conversion Time: " + str(convTimeString) + "$\mu$s", size = 12)
            ax.set_xlabel("Data Index [i]", size = 12)
            ax.set_ylabel("ADC [V]", size=12)
            ax.legend()
                 
        fig.tight_layout()
        #fig.suptitle("DAC Led 2D Buffer Ramps -- WITH RETRACE", fontsize=18)  
        
        #plt.show()
        fig.savefig(currentdir+"\\Unit Test Plots\\dac_led_multi_channel_2d_ramp_retrace.png")
        print("TEST 12: " + testname + " PASSED\n")
    except Exception as e:
        print("Error: ", e)
        exit()

def test13(da):    
    ### Test Basic Buffer Ramp functionality ###
    try: 
        testname = "2D DAC LED BUFFER RAMP WITH SNAKE"
        print("TEST 13: " + testname)
        
        fig, axes = plt.subplots(2,3, figsize=(18,8))
        
        axes = axes.flatten()
        
        convTimes = np.linspace(82,2600,6)
        realConvTimes = []
        delays = []
        
        datas = []
        
        for j, convTime in enumerate(convTimes):
            realConvTime = 0
            for i in range(4*NUM_ADC_CARDS):
                if i == 0:
                    realConvTime = float(da.set_conversiontime(i, convTime))
                    realConvTimes.append(realConvTime)
                else:
                    da.set_conversiontime(i, convTime)
            
            rampTime = 2*realConvTime + 181 + 100
            delays.append(rampTime)
            
            data = da.dac_led_buffer_ramp_2d([0,4], [1,5], [-2, -1], [4, 2], [4, 2], 201, 11, False, True, 1, rampTime, 100)
            datas.append(data)
            
        for k, ax in enumerate(axes):
            adc_nums = [0, 1, 4, 5]
            for l, dataset in enumerate(datas[k]):
                ax.plot(dataset, label = 'ADC'+str(adc_nums[l]))
            rampTimeString = format(delays[k], "0.3f")
            convTimeString = format(realConvTimes[k], "0.1f")
            ax.set_title("DAC Delay: " + str(rampTimeString) + "$\mu$s, Conversion Time: " + str(convTimeString) + "$\mu$s", size = 12)
            ax.set_xlabel("Data Index [i]", size = 12)
            ax.set_ylabel("ADC [V]", size=12)
            ax.legend()
                 
        fig.tight_layout()
        #fig.suptitle("DAC Led 2D Buffer Ramps -- WITH RETRACE", fontsize=18)  
        
        #plt.show()
        fig.savefig(currentdir+"\\Unit Test Plots\\dac_led_multi_channel_2d_ramp_snake.png")
        print("TEST 13: " + testname + " PASSED\n")
    except Exception as e:
        print("Error: ", e)
        exit()
        
def test14(da):    
    ### Test Basic Buffer Ramp functionality ###
    try: 
        testname = "2D TIME SERIES BUFFER RAMP NO RETRACE"
        print("TEST 14: " + testname)
        
        fig, axes = plt.subplots(2,3, figsize=(18,8))
        
        axes = axes.flatten()
        
        convTimes = np.linspace(82,2600,6)
        realConvTimes = []
        delays = []
        
        datas = []
        
        for j, convTime in enumerate(convTimes):
            realConvTime = 0
            for i in range(4*NUM_ADC_CARDS):
                if i == 0:
                    realConvTime = float(da.set_conversiontime(i, convTime))
                    realConvTimes.append(realConvTime)
                else:
                    da.set_conversiontime(i, convTime)
            
            sampleTime = 2*int(realConvTime) + 310
            delays.append(sampleTime)
            
            data = da.time_series_buffer_ramp_2d([0,4], [1,5], [-2, -1], [4, 2], [4, 2], 10000, 11, False, False, 150, sampleTime)
            datas.append(data)
            
        for k, ax in enumerate(axes):
            adc_nums = [0, 1, 4, 5]
            for l, dataset in enumerate(datas[k]):
                ax.plot(dataset, label = 'ADC'+str(adc_nums[l]))
            rampTimeString = format(delays[k], "0.3f")
            convTimeString = format(realConvTimes[k], "0.1f")
            ax.set_title("Conversion Time: " + str(convTimeString) + "$\mu$s", size = 12)
            ax.set_xlabel("Data Index [i]", size = 12)
            ax.set_ylabel("ADC [V]", size=12)
            ax.legend()
                 
        fig.tight_layout()
        #fig.suptitle("DAC Led 2D Buffer Ramps -- NO RETRACE", fontsize=18)  
        
        #plt.show()
        fig.savefig(currentdir+"\\Unit Test Plots\\time_series_multi_channel_2d_ramp.png")
        print("TEST 14: " + testname + " PASSED\n")
    except Exception as e:
        print("Error: ", e)
        exit()

def test15(da):    
    ### Test Basic Buffer Ramp functionality ###
    try: 
        testname = "2D TIME SERIES BUFFER RAMP WITH RETRACE"
        print("TEST 15: " + testname)
        
        fig, axes = plt.subplots(2,3, figsize=(18,8))
        
        axes = axes.flatten()
        
        convTimes = np.linspace(82,2600,6)
        realConvTimes = []
        delays = []
        
        datas = []
        
        for j, convTime in enumerate(convTimes):
            realConvTime = 0
            for i in range(4*NUM_ADC_CARDS):
                if i == 0:
                    realConvTime = float(da.set_conversiontime(i, convTime))
                    realConvTimes.append(realConvTime)
                else:
                    da.set_conversiontime(i, convTime)
            
            sampleTime = 2*int(realConvTime) + 310
            delays.append(sampleTime)
            
            data = da.time_series_buffer_ramp_2d([0,4], [1,5], [-2, -1], [4, 2], [4, 2], 10000, 11, True, False, 150, sampleTime)
            datas.append(data)
            
        for k, ax in enumerate(axes):
            adc_nums = [0, 1, 4, 5]
            for l, dataset in enumerate(datas[k]):
                ax.plot(dataset, label = 'ADC'+str(adc_nums[l]))
            rampTimeString = format(delays[k], "0.3f")
            convTimeString = format(realConvTimes[k], "0.1f")
            ax.set_title("Conversion Time: " + str(convTimeString) + "$\mu$s", size = 12)
            ax.set_xlabel("Data Index [i]", size = 12)
            ax.set_ylabel("ADC [V]", size=12)
            ax.legend()
                 
        fig.tight_layout()
        #fig.suptitle("DAC Led 2D Buffer Ramps -- NO RETRACE", fontsize=18)  
        
        #plt.show()
        fig.savefig(currentdir+"\\Unit Test Plots\\time_series_multi_channel_2d_ramp_with_retrace.png")
        print("TEST 15: " + testname + " PASSED\n")
    except Exception as e:
        print("Error: ", e)
        exit()

def test16(da):    
    ### Test Basic Buffer Ramp functionality ###
    try: 
        testname = "2D TIME SERIES BUFFER RAMP WITH SNAKE"
        print("TEST 16: " + testname)
        
        fig, axes = plt.subplots(2,3, figsize=(18,8))
        
        axes = axes.flatten()
        
        convTimes = np.linspace(82,2600,6)
        realConvTimes = []
        delays = []
        
        datas = []
        
        for j, convTime in enumerate(convTimes):
            realConvTime = 0
            for i in range(4*NUM_ADC_CARDS):
                if i == 0:
                    realConvTime = float(da.set_conversiontime(i, convTime))
                    realConvTimes.append(realConvTime)
                else:
                    da.set_conversiontime(i, convTime)
            
            sampleTime = 2*int(realConvTime) + 310
            delays.append(sampleTime)
            
            data = da.time_series_buffer_ramp_2d([0,4], [1,5], [-2, -1], [4, 2], [4, 2], 10000, 11, False, True, 150, sampleTime)
            datas.append(data)
            
        for k, ax in enumerate(axes):
            adc_nums = [0, 1, 4, 5]
            for l, dataset in enumerate(datas[k]):
                ax.plot(dataset, label = 'ADC'+str(adc_nums[l]))
            rampTimeString = format(delays[k], "0.3f")
            convTimeString = format(realConvTimes[k], "0.1f")
            ax.set_title("Conversion Time: " + str(convTimeString) + "$\mu$s", size = 12)
            ax.set_xlabel("Data Index [i]", size = 12)
            ax.set_ylabel("ADC [V]", size=12)
            ax.legend()
                 
        fig.tight_layout()
        #fig.suptitle("DAC Led 2D Buffer Ramps -- NO RETRACE", fontsize=18)  
        
        #plt.show()
        fig.savefig(currentdir+"\\Unit Test Plots\\time_series_multi_channel_2d_ramp_with_snake.png")
        print("TEST 16: " + testname + " PASSED\n")
    except Exception as e:
        print("Error: ", e)
        exit()
       
def test17(da):
    testname = "DAC READ BACK"
    print("TEST 17: " + testname)
    try:
        for i in range(NUM_ADC_CARDS*4):
            da.set_conversiontime(i, 2600)
        
        test_voltages = np.linspace(-9.9, 9.9, 101)
        
        for voltage in test_voltages:
            for channel in range(NUM_DAC_CARDS*4):
                da.set_voltage(channel, voltage)
                dac_set = float(da.read_dac_voltage(channel))
                if abs(dac_set - voltage) >= 1e-4:
                    print(dac_set)
                    print(voltage)
                    print("TEST 17: " + testname + " FAILED\n")
                    exit()
                    
                actual_voltage = float(da.read_voltage(channel))
                if abs(actual_voltage - dac_set) >= 0.001:
                    print(actual_voltage)
                    print(dac_set)
                    print("TEST 17: " + testname + " FAILED\n")
                    exit()
        print("TEST 17: " + testname + " PASSED\n")
    except Exception as e:
        print("Error: ", e)
        exit()

def test18(da):
    try: 
        testname = "ADC NOISE TEST"
        print("TEST 18: " + testname)
        
        for i in range(4*NUM_DAC_CARDS):
            da.set_voltage(i, 0)
        
        for adcNum in range(NUM_ADC_CARDS*4):
            fig, axes = plt.subplots(2,4, figsize=(15,12))
            axes = axes.flatten()
            
            convTimes = np.linspace(82,2600,8)
            
            adc_spectra = []
            
            for j, convTime in enumerate(convTimes):
                print("Sampling ADC"+str(adcNum)+" with conversion time "+str(convTime))
                
                x = da.time_series_adc_read([adcNum], convTime, 3e6)
                adc_spectra.append(x)
                
            for k, ax in enumerate(axes):
                std = 1e6 * np.std(adc_spectra[k][0])
                ax.plot(1e6*adc_spectra[k][0])
                convTimeString = format(convTimes[k], "0.3f")
                stdString = format(std, "0.3f")
                ax.set_title("Conversion Time: " + str(convTimeString) + "$\mu$s, $\sigma$ = "+str(stdString)+"$\mu$V" , size = 12)
                ax.set_xlabel("Data Index [i]", size = 12)
                ax.set_ylabel("ADC"+str(adcNum)+" [V]", size=12)
                
            
            fig.tight_layout()
            
            #plt.show()
            fig.savefig(currentdir+"\\Unit Test Plots\\adc_spectra_"+str(adcNum)+".png")
        print("TEST 18: " + testname + " PASSED")
    except Exception as e:
        print("Error: ", e)
        exit()            
        
    '''
    ### Testing Noise Analysis Capabilities ###
    try: 
        testname = "Spectrum Analyzer"
        print("TEST 15: " + testname)
        
        for dacNum in range(NUM_DAC_CARDS*4):
            fig, axes = plt.subplots(2,2, figsize=(12,8))
            
            axes = axes.flatten()
            
            convTimes = np.linspace(82,500,4)
            realConvTimes = []
              
            datas = []
            freqs = []
            
            for j, convTime in enumerate(convTimes):
                realConvTime = 0
                for i in range(8):
                    if i == 0:
                        realConvTime = float(da.set_conversiontime(i, convTime))
                        realConvTimes.append(realConvTime)
                    else:
                        da.set_conversiontime(i, convTime)
                
                for i in range(100):
                    if i == 0:
                        out = da.time_series_buffer_ramp([dacNum], [dacNum], [0.0], [0.0], 3000, 600, realConvTime + 310)
                        #print(out)
                        data = out[0]
                    else:
                        out = da.time_series_buffer_ramp([dacNum], [dacNum], [0.0], [0.0], 3000, 600, realConvTime + 310)
                        data += out[0]
                        
                data = data/100.0
                T = (realConvTime + 310)*1e-6
                fft_vals = np.abs(np.fft.rfft(data**2)/(len(data) / T))
                fft_freqs = np.fft.rfftfreq(len(data), d=T)
                
                #fft_vals = np.fft.fftshift(fft_vals)
                #fft_freqs = np.fft.fftshift(fft_freqs)
                
                datas.append(1e9 * np.sqrt(fft_vals))
                freqs.append(fft_freqs)
                
                print(np.mean(datas[j]))
                
            for k, ax in enumerate(axes):
                ax.plot(freqs[k],datas[k])
                convTimeString = format(realConvTimes[k], "0.3f")
                ax.set_title("Conversion Time: " + str(convTimeString) + "$\mu$s", size = 12)
                ax.set_xlabel("Frequency [Hz]", size = 12)
                ax.set_ylabel("Voltage Spectral Density [nV/Sqrt(Hz)]", size=12)
                #ax.set_xlim(0.2,3000)
                
            #fig.suptitle("DAC Update Period: 50$\mu$s, Total Ramp Time = 500ms, $\Delta V$ = 1.8mV", fontsize=18)
            fig.tight_layout()
            
            #plt.show()
            fig.savefig(currentdir+"\\Unit Test Plots\\time_series_single_channel_ramp_DAC"+str(dacNum)+".png")
        print("TEST 15: " + testname + " PASSED\n")
    except Exception as e:
        print("Error: ", e)
        exit()
    '''

def main():
    da = test1()
    test2(da)
    test3(da)
    test4(da)
    test5(da)
    test6(da)
    test7(da)
    test8(da)
    test9(da)
    test10(da)
    test11(da)
    test12(da)
    test13(da)
    test14(da)
    test15(da)
    test16(da)
    test17(da)
    #test18(da)


if __name__ == '__main__':
    main()

    