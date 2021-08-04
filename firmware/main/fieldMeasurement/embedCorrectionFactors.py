import subprocess, csv, sys, os

def show_help():
    print 'embedCorrectionFactors.py - Generates headers with PM sensor adjustment factors, for the connected device'
    print '  Usage: python embedCorrectionFactors.py [device port]'
    print '  Examples:'
    print '    python embedCorrectionFactors.py /dev/ttyUSB0'
    print '    python embedCorrectionFactors.py /dev/ttyUSB1'

# Read Base MAC address of connected device
def get_mac(device_port):
    substr = 'MAC: '
    
    process = subprocess.Popen(f'esptool -p {device_port} read_mac'.split(), 
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    stdout, stderr = process.communicate()
    split_stdout = stdout.decode().split('\n')
    filtered = [line for line in split_stdout if substr in line]
    
    if len(filtered) == 0:
        return -1
    else: 
        mac = filtered[0].replace(substr, '')
    return mac

#Retrieve correction factors from CSV
def retrieve_correction_factors(mac_address, csv_path):
    found_mac = False

    try:
    with open(path, mode='r') as csv_file:
        csv_reader = csv.DictReader(csv_file)
        for row in csv_reader:
            if mac_address in row.keys():
                found_mac = True
            if 'intercept' in row.values():
                intercept = row[mac_address]
            if 'slope' in row.values():
                slope = row[mac_address]
        if found_mac == False:
            intercept = 0.0
            slope = 1.0
    except:
        print("Unable to source correction factors file")
    finally:
        return slope, intercept

# Write code into file within the particulate sensor driver folder
def generate_correction_factors_hpp(slope, intercept, hpp_path):
    code =  'namespace externalHardwareSubsystem\n'
            '{\n'
            '    namespace particulateSensor\n'
            '    {\n'
           f'        constexpr float slope = {slope};\n'
           f'        constexpr float intercept = {intercept};\n'
            '    }\n'
            '}\n'
  
  with open(hpp_path,'w') as output_file:
        output_file.write(code)

if __name__ == "__main__":
    # we need at least the port name
    if (len(sys.argv) < 2):
        show_help()
        sys.exit()
    
    file_dir = os.path.dirname(os.path.abspath(__file__))  
    csv_path = 
    ( 
    f'{file_dir}/../../../'
    'software/particulateSensorNormalization/results/correction_factors.csv'
    )
    hpp_path = 
    ( 
    f'{file_dir}/../../'
    'components/hwSubsystem_particulateSensor/include/correction_factors.hpp'
    )
  
    device_port = sys.argv[1]
    mac_result = get_mac(device_port)
    if mac_result == -1:
        print('Error obtaining device base MAC address')
    else:
        factors = retrieve_correction_factors(mac_result, csv_path)
        slope = factors[0]
        intercept = factors[1]
        generate_correction_factors_hpp(slope, intercept, hpp_path)
