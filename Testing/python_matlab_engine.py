import matlab.engine
import os, sys
import csv

'''
Chase Badalato
10172570
2022-02-22

Connect to the matlab engine and run analysis
'''
print("Retrieving Values ...")
ecg_vals = []
with open('ECG_SAMPLES.csv') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    line_count = 0
    for row in csv_reader:
        if line_count == 0:
            #print(f'Column names are {", ".join(row)}')
            line_count += 1
        elif not row:
        	line_count += 1
        else:
            #print('Time: ' + str(row[0][:-2]) + '    Value: ' + str(row[1]))
            ecg_vals.append(int(row[1]))
            line_count += 1

    print(f'Processed {line_count} lines.')

print("Sucess!")
print("")
print("Calling Matlab Function ...")

eng = matlab.engine.start_matlab()
eng.addpath(os.path.realpath(os.path.join(os.getcwd(), os.path.dirname("matlab_functions.m"))),nargout=0)

matlab_ecg = matlab.uint16(ecg_vals)
res = eng.matlab_functions(matlab_ecg, nargout = 1)
print("Matlab Result: " + str(res))

# x = 4.0
# eng.workspace['y'] = x
# a = eng.eval('sqrt(y)')
# print(a)
eng.quit()