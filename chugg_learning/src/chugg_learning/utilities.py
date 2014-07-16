import subprocess
from os import path

def get_log_dir():
    pkg_dir = subprocess.check_output(['rospack', 'find', 'chugg_learning'])
    pkg_dir = pkg_dir[:-1]      # remove newline
    
    return path.join(pkg_dir, 'logs')

def get_log_path():
    
    return path.join(get_log_dir(), '{domain}/{agent}/{representation}/')
