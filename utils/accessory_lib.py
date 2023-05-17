import platform 
import sys 


def system_info(): 
    print('\n', "----- Environment Status -----") 
    print("Operating System Type :", platform.architecture()) 
    print('Python Version :', sys.version, end='\n'+'\n')
