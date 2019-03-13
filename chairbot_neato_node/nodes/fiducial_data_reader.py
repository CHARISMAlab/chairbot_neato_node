import subprocess;

class F:
    def __init__(self):
        # FIDUCIAL MARKER variables
        self.fiducial_marker_file_path = '/home/charisma/processing-3.5.3';
        self.fiducial_marker_file_name = 'output1.txt'
        self.fiducial_marker_data = {'time':None,'x':None,'y':None,'angle':None};
        self.fiducial_marker_file=None;

        try:
          #get the filename
          self.fiducial_marker_file_name =self.fiducial_marker_file_path+self.fiducial_marker_file_name
          #get the last line from the file by running tail
          string = subprocess.check_output(['tail','-n1','/home/charisma/processing-3.5.3/output1.txt'])
          #remove line-endings from that last line
          string = string.splitlines()[0]
          #unpack the data in variables
          data = string.split(',')
          #sample line: 42881,3.0,3.0,3.0
          self.fiducial_marker_data['time'] = float(data[0])
          self.fiducial_marker_data['x'] = float(data[1])
          self.fiducial_marker_data['y'] = float(data[2])
          self.fiducial_marker_data['angle'] = float(data[3])
          print("Hey I read %s", string)
          print("Hey we got ", self.fiducial_marker_data)
        except IOError:
          print("Fidcuial marker file " + self.fiducial_marker_file_name + "cannot be opened")

x = F()
