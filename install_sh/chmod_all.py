#!/usr/bin/env python3

import os #for folder find
import subprocess #for popen


def find_all_files( directory ):
    for root, dirs, files in os.walk( directory ):
        yield root
#        print (dirs)
        for file in files:
#            print (file)
            yield os.path.join( root, file )



current_directory_path = str(os.getcwd())
print ("current_directory_path: " , current_directory_path)
up_num_to_home_dir = current_directory_path.count('/') - 2
print ("up_num_to_home_dir: " , up_num_to_home_dir)
target_dir_path = "catkin_ws/src"
for i in range(up_num_to_home_dir):
    target_dir_path = "../" + target_dir_path
print ("target_dir_path: " , target_dir_path , "\n\n")



for file in find_all_files( target_dir_path ):
#    print (file)
    if file.find( ".py" ) > 0 or file.find( ".sh" ) > 0:
        print (file)
        cmd = "chmod 755 " + file
        subprocess.Popen(cmd, shell=True)

print ("\nAll catkin_ws/src-files are now executable")
