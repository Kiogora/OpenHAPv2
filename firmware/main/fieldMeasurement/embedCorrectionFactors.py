import subprocess
import csv
import os

# Read Base MAC address of connected device
def get_mac():
  process = subprocess.Popen('esptool -p /dev/ttyUSB0 read_mac'.split(), 
                              stdout=subprocess.PIPE, 
                              stderr=subprocess.PIPE)
  substr = 'MAC: '
  stdout, stderr = process.communicate()
  split_stdout = stdout.decode().split('\n')
  filtered = [line for line in split_stdout if substr in line]
  if len(filtered) == 0:
    return -1
  else: 
    mac = filtered[0].replace(substr, '')
    return mac

#Retrieve correction factors from CSV
def retrieve_correction_factors(mac_address):
  file_dir = os.path.dirname(os.path.abspath(__file__))
  path = ( 
          f'{file_dir}/../../../'
          'software/particulateSensorNormalization/results/correction_factors.csv'
  )
  try:
    with open(path, mode='r') as csv_file:
      found_mac = False
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
def generate_source(slope, intercept):
  obrace = '{'
  cbrace = '}'

  code = fr'''
namespace externalHardwareSubsystem
{obrace}
  namespace particulateSensor
  {obrace}
  constexpr float slope = {slope};
  constexpr float intercept = {intercept};
  {cbrace}
{cbrace}
  '''
  file_dir = os.path.dirname(os.path.abspath(__file__))
  path = ( 
          f'{file_dir}/../../'
          'components/hwSubsystem_particulateSensor/include/correction_factors.hpp'
  )
  with open(path,'w') as output_file:
      output_file.write(code)

if __name__ == "__main__":
  result = get_mac()
  if result != -1:
    factors = retrieve_correction_factors(result)
    slope = factors[0]
    intercept = factors[1]
    generate_source(slope, intercept)
  else:
    print('Error obtaining base MAC address')
