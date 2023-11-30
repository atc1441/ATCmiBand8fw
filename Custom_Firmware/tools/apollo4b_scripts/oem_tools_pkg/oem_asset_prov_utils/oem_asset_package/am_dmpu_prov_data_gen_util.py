

import sys
import os 

# Definitions for paths
if sys.platform != "win32" :
    path_div = "//"    
else : #platform = win32
    path_div = "\\"

HBK1_SIZE   = 16
AES_KEY_SIZE_PLAIN = 16
AES_KEY_SIZE_ASSET_PKG  = 64
OEM_DATA_SIZE_MAX = 2096

AES_KEY_NO_KEY = 0
AES_KEY_PLAIN = 1
AES_KEY_ASSET_PKG = 2


CURRENT_PATH = sys.path[0]
# In case the scripts were run from current directory
CURRENT_PATH_SCRIPTS = path_div +  ".." + path_div + "common"
# this is the scripts local path, from where the program was called
sys.path.append(CURRENT_PATH+CURRENT_PATH_SCRIPTS)

import configparser
from dmpu_util_helper import *
import sys

UTILITY_LIB_DIR = "lib"
UTILITY_LIB_Name = SBU_CRYPTO_LIB_DIR + "/" + "lib_oem_asset_pkg.so"

ASSET_TYPE_OEM_DATA = 3

# Parse given test configuration file and return test attributes as dictionary
def parse_config_file (config, log_file):
    local_dict = {}
    section_name = "DMPU-DATA-CFG"

    if not config.has_section(section_name):
        log_sync(log_file, "section " + section_name + " wasn't found in cfg file\n")
        return None

    # Get HBK1 file name
    if config.has_option(section_name, 'hbk-1-filename'): 
        hbkFilePath_str = config.get(section_name, 'hbk-1-filename')
        if os.path.isfile(hbkFilePath_str):
            fileSize = os.path.getsize(hbkFilePath_str)
            if fileSize != HBK1_SIZE:
                log_sync(log_file,"hbk-1-filename size not Correct \n") 
                return None         
            local_dict['hbk-1-filename'] = hbkFilePath_str
            log_sync(log_file,"hbk-1-filename: " + str(local_dict['hbk-1-filename']) + "\n")    
        else:
            log_sync(log_file,"hbk-1-filename: " + str(hbkFilePath_str) + " File Not Found !!! \n")      
            return None              
    else:
        log_sync(log_file,"HBK 1 not provided \n")
        return None

    # Get kcp type
    if config.has_option(section_name, 'kcp-data-type'): 
        local_dict['kcp-data-type'] = int(config.get(section_name, 'kcp-data-type'))
        if( (local_dict['kcp-data-type'] == AES_KEY_NO_KEY) or (local_dict['kcp-data-type'] == AES_KEY_ASSET_PKG) or (local_dict['kcp-data-type'] == AES_KEY_PLAIN) ):
            log_sync(log_file,"kcp-data-type: " + str(local_dict['kcp-data-type']) + "\n")
        else:
            log_sync(log_file," Invalid kcp-data-type \n")
            return None           
    else:
        return None

    # Get kce type
    if config.has_option(section_name, 'kce-data-type'): 
        local_dict['kce-data-type'] = int(config.get(section_name, 'kce-data-type'))
        if((local_dict['kce-data-type'] == AES_KEY_NO_KEY) or (local_dict['kce-data-type'] == AES_KEY_ASSET_PKG) or (local_dict['kce-data-type'] == AES_KEY_PLAIN)):
            log_sync(log_file,"kce-data-type: " + str(local_dict['kce-data-type']) + "\n")
        else:
             log_sync(log_file," Invalid kce-data-type \n")
             return None           
    else:
        return None 

    # Get kcp file name
    if(local_dict['kcp-data-type'] != AES_KEY_NO_KEY):
        if config.has_option(section_name, 'kcp-filename'): 
            filePath_str = config.get(section_name, 'kcp-filename')
            if os.path.isfile(filePath_str):
                fileSize = os.path.getsize(filePath_str)
                if ( ( (local_dict['kcp-data-type'] == AES_KEY_ASSET_PKG) and (fileSize == AES_KEY_SIZE_ASSET_PKG) ) or
                    ( (local_dict['kcp-data-type'] == AES_KEY_PLAIN) and (fileSize == AES_KEY_SIZE_PLAIN) ) ):
                    local_dict['kcp-filename'] = filePath_str
                    log_sync(log_file,"kcp-filename: " + str(local_dict['kcp-filename']) + "\n")
                else: 
                    log_sync(log_file,"kcp key size Invalid \n") 
                    return None
            else:
                log_sync(log_file,"kcp-filename: " + str(filePath_str) + " File Not Found !!! \n")      
                return None              
        else:
            log_sync(log_file,"kcp key not provided \n")
            return None
    else:
        print("\n NO KCP KEY IN PROVISIONING DATA !!! \n")        


    # Get kce file name
    if(local_dict['kce-data-type'] != AES_KEY_NO_KEY):
        if config.has_option(section_name, 'kce-filename'): 
            filePath_str = config.get(section_name, 'kce-filename')
            if os.path.isfile(filePath_str):
                fileSize = os.path.getsize(filePath_str)
                if ( ( (local_dict['kce-data-type'] == AES_KEY_ASSET_PKG) and (fileSize == AES_KEY_SIZE_ASSET_PKG) ) or
                    ( (local_dict['kce-data-type'] == AES_KEY_PLAIN) and (fileSize == AES_KEY_SIZE_PLAIN) ) ):

                    local_dict['kce-filename'] = filePath_str
                    log_sync(log_file,"kce-filename: " + str(local_dict['kce-filename']) + "\n")
                else: 
                    log_sync(log_file,"kce key size Invalid \n") 
                    return None         
            else:
                log_sync(log_file,"kce-filename: " + str(filePath_str) + " File Not Found !!! \n")      
                return None              
        else:
            log_sync(log_file,"kce key not provided \n")
            return None
    else:
        print("NO KCE KEY IN PROVISIONING DATA !!! \n") 


    # Get oem-min-version
    if config.has_option(section_name, 'oem-min-version'): 
        verData_str = config.get(section_name, 'oem-min-version')
        local_dict['oem-min-version'] = int(verData_str,16)
        log_sync(log_file,"oem-min-version: " + str(verData_str) + "\n")        
    else:        
        log_sync(log_file,"oem min version not provided \n")
        return None

    # Get oem DCU default data
    if config.has_option(section_name, 'oem-dcu-default_0'): 
        dcuW0_str = config.get(section_name, 'oem-dcu-default_0')
        local_dict['oem-dcu-default_0'] = int(dcuW0_str,16)
        log_sync(log_file,"oem-dcu-default_0: " + str(dcuW0_str) + "\n")        
    else:       
        log_sync(log_file,"oem dcu default word 0 not provided \n")
        return None 

    if config.has_option(section_name, 'oem-dcu-default_1'): 
        dcuW1_str = config.get(section_name, 'oem-dcu-default_1')
        local_dict['oem-dcu-default_1'] = int(dcuW1_str,16)
        log_sync(log_file,"oem-dcu-default_1: " + str(dcuW1_str) + "\n")        
    else:       
        log_sync(log_file,"oem dcu default word 1 not provided \n")
        return None 

    if config.has_option(section_name, 'oem-dcu-default_2'): 
        dcuW2_str = config.get(section_name, 'oem-dcu-default_2')
        local_dict['oem-dcu-default_2'] = int(dcuW2_str,16)
        log_sync(log_file,"oem-dcu-default_2: " + str(dcuW2_str) + "\n")     
    else:       
        log_sync(log_file,"oem dcu default word 2 not provided \n")
        return None 

    if config.has_option(section_name, 'oem-dcu-default_3'): 
        dcuW3_str = config.get(section_name, 'oem-dcu-default_3')
        local_dict['oem-dcu-default_3'] = int(dcuW3_str,16)
        log_sync(log_file,"oem-dcu-default_3: " + str(dcuW3_str) + "\n")       
    else:       
        log_sync(log_file,"oem dcu default word 3 not provided \n")
        return None

    # OEM GPC Configuration    
    if config.has_option(section_name, 'oem-gpc-cfg-bits'): 
        oemGpcCfg_str = config.get(section_name, 'oem-gpc-cfg-bits')
        oemGpcCfg = int(oemGpcCfg_str,16)
        # Ignore the byte 0, that is reserved for ICV
        local_dict['oem-gpc-cfg-bits'] = (oemGpcCfg & 0xFFFFFF00)
        log_sync(log_file,"oem-gpc-cfg-bits: " + str(local_dict['oem-gpc-cfg-bits']) + "\n")       
    else:       
        log_sync(log_file,"oem GPC config data not provided \n")
        return None      

    # Get oem Prop Data file name
    if config.has_option(section_name, 'oem-prop-data-filename'): 
        filePath_str = config.get(section_name, 'oem-prop-data-filename')
        print(filePath_str)
        if os.path.isfile(filePath_str):
            fileSize = os.path.getsize(filePath_str)
            print("fileSize \n")
            print(fileSize)

            if fileSize != OEM_DATA_SIZE_MAX:
                log_sync(log_file,"Oem Prop Data size more than max limit \n") 
                return None                                
            local_dict['oem-prop-data-filename'] = filePath_str
            log_sync(log_file,"oem-prop-data-filename: " + str(local_dict['oem-prop-data-filename']) + "\n") 
        else:
            log_sync(log_file,"oem-prop-data-filename: " + str(filePath_str) + " File Not Found !!! \n")      
            return None                  
    else:
        log_sync(log_file,"OEM PROP KEYS DATA NOT PROVIDED !!! \n")


    # Get the output file name
    if config.has_option(section_name, 'dmpu-data-filename'): 
        local_dict['dmpu-data-filename'] = config.get(section_name, 'dmpu-data-filename')
        log_sync(log_file,"dmpu-data-filename: " + str(local_dict['dmpu-data-filename']) + "\n")        
    else:
        log_sync(log_file,"Output File name not provided !!! \n")
        return None 

    return local_dict

# Parse script parameters
def parse_shell_arguments ():
    len_arg =  len(sys.argv)
    if len_arg < 2:
        print_sync("len " + str(len_arg) + " invalid. Usage:" + sys.argv[0] + "<test configuration file>\n")
        for i in range(1,len_arg):
            print_sync("i " + str(i) + " arg " + sys.argv[i] + "\n")
        sys.exit(1)
    config_fname = sys.argv[1]
    if len_arg == 3:
        log_fname = sys.argv[2]
    else:
        log_fname = "asset_prov.log"
    return config_fname, log_fname


# close files and exit script
def exit_main_func(log_file, config_file, rc):
    log_file.close()
    config_file.close()
    sys.exit(rc)


def main():

    config_fname, log_fname = parse_shell_arguments()
    log_file = create_log_file(log_fname)
    print_and_log(log_file, str(datetime.now()) + ": Dmpu provisioning data generation utility started (Logging to " + log_fname + ")\n")

    print("Processing config file \n")

    DLLHandle = LoadDLLGetHandle(UTILITY_LIB_Name)
    
    try:
        config_file = open(config_fname, 'r')
    except IOError as e:
        print_and_log(log_file,"Failed opening " + config_fname + " (" + e.strerror + ")\n")
        log_file.close()
        sys.exit(e.errno)



    config = configparser.ConfigParser()
    config.read(config_fname)
    data_dict = {}

    data_dict = parse_config_file(config, log_file)

    if (data_dict != None):
        print("Parser Done \n")

        # Get assets and encrypted key from files
        if(data_dict['kcp-data-type'] != AES_KEY_NO_KEY):
            kcpAssetSize, kcpAsset = GetDataFromBinFile(log_file, data_dict['kcp-filename'])

        if(data_dict['kce-data-type'] != AES_KEY_NO_KEY):    
            kceAssetSize, kceAsset = GetDataFromBinFile(log_file, data_dict['kce-filename'])

        #check if we have OEM Prop Data 
        OemPropAssetSize, OemPropAsset = GetDataFromBinFile(log_file, data_dict['oem-prop-data-filename'])

        hbk1Size, hbk1Data =  GetDataFromBinFile(log_file, data_dict['hbk-1-filename'])

        print(OemPropAssetSize)

        result = DLLHandle.build_dmpu_data_blob(1, hbk1Data,  data_dict['kcp-data-type'], kcpAsset,
                        data_dict['kce-data-type'], kceAsset, data_dict['oem-min-version'], 
                        data_dict['oem-dcu-default_0'], data_dict['oem-dcu-default_1'],
                        data_dict['oem-dcu-default_2'], data_dict['oem-dcu-default_3'], 
                        OemPropAsset, OemPropAssetSize, data_dict['oem-gpc-cfg-bits'], str.encode(data_dict['dmpu-data-filename']))
        print(result)                 
                        
        if result != 0:
            raise NameError

        cmd = "xxd -i " +  data_dict['dmpu-data-filename'] + " > dmpu_prov_data.h"
        os.system(cmd)    

        print_and_log(log_file, "**** DMPU Data Blob Generate completed successfully ****\n")
        exit_main_func(log_file, config_file, 0)       


    else:
        print_and_log(log_file, "**** Invalid config file ****\n")
        exit_main_func(log_file, config_file, 1)

    FreeDLLGetHandle(DLLHandle)

#############################
if __name__ == "__main__":
    main()



