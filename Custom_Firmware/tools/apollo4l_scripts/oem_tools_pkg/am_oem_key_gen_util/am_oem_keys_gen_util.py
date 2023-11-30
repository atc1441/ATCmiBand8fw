#!/usr/local/bin/python3

import sys
import configparser
import os 
from pathlib import Path
import pdb
import subprocess
import binascii

# Definitions for paths
#######################
if sys.platform != "win32" :
    path_div = "/"    
else : #platform = win32
    path_div = "\\"

# Adding the utility python scripts to the python PATH
CURRENT_PATH = sys.path[0]
print(CURRENT_PATH)

# In case the scripts were run from current directory
CURRENT_PATH_SCRIPTS = path_div + 'common_utils'
print(CURRENT_PATH_SCRIPTS)
    
sys.path.append(str(Path(CURRENT_PATH).parent)+CURRENT_PATH_SCRIPTS)
print(str(Path(CURRENT_PATH).parent)+CURRENT_PATH_SCRIPTS)


from cert_cfg_parser_util import *

BITS_WITHIN_WORD = 32
BYTES_WITHIN_WORD = 4

##################################################
#check if the val is in the list
###################################################
def check (list, val):
    for i in list:
        if i == val:
            return True
    return False 
##################################################
# The function creates words list out of a string that contains hex
# resrepesentation
###################################################
def CreateWordList(buff, buffWordSize, endiannes):

    wordList = list()

    for i in range(int(buffWordSize)):
        word = 0
        if endiannes == 'L':
            #int(buff[i].encode('hex'),16)
            word = buff[i*4]
            word = word+(buff[i*4+1]<<8)
            word = word+(buff[i*4+2]<<16)
            word = word+(buff[i*4+3]<<24)
        else:
            word = buff[i*4+3]
            word = word+(buff[i*4+2]<<8)
            word = word+(buff[i*4+1]<<16)
            word = word+(buff[i*4]<<24)
        wordList.append(format(word, 'x'))

    return wordList 



####################################################
# The function calculates the number of Zeroes for a given data
####################################################
def CalculateZeroBitCountOnData(HASHwordList, size):    
#total number of zeros, start value
    zero_sum = int(0)
    
    for word in HASHwordList:
        if size == 0:
            break
        #1's bit count start value
        count = 0
        intWord = int(word, 16)        
        while (intWord):      
            count += (intWord & 1)
            intWord >>= 1                      
        zero_sum += (BITS_WITHIN_WORD - count)
        size -=1
    
    return zero_sum


##################################################################
# This function parse the config file and check for errors. The 
# keys generation section / options, which are error free, added 
# to the dictionary which are processed later to generate applicable 
# keys 
###################################################################
def key_gen_config_file_parser(config_fname, log_file):

    try:
        config_file = open(config_fname, 'r')
    except IOError as e:
        print_and_log(log_file,"Failed opening " + config_fname + " (" + e.strerror + ")\n")
        sys.exit(e.errno)

    config = configparser.ConfigParser()
    #Read the config file 
    config.readfp(config_file)
    print(config.sections())
    config_file.close()

    # Local temp Variables
    local_dict = dict()
    temp_list = list()

    # Check for the config file section
    section_name = "OEM-KEY-GEN"
    if not config.has_section(section_name):
        print_and_log(log_file, "section " + section_name + " wasn't found in cfg file\n")        
        return None, None

    rsaKeyStrengthList = ['1024','2048','3072','4096']
    aesKeyStrengthList = ['128', '256']

    local_dict['rsa_key_strength'] = '3072'
    local_dict['aes_key_strength'] = '128'

    # check for RSA keys path 
    if config.has_option(section_name, 'rsa_keys_path'):
        rsaKeysPath = config.get(section_name, 'rsa_keys_path')
        if len(rsaKeysPath):
            local_dict['rsa_keys_path'] = config.get(section_name, 'rsa_keys_path')
        else:
            local_dict['rsa_keys_path'] = ' '
            print_and_log(log_file, "Path for RSA keys not provided\n")
    else:
        print_and_log(log_file, "The RSA keys path parameter is mandatory \n")
        return None, None

    # check for AES keys path
    if config.has_option(section_name, 'aes_keys_path'):
        aesKeyPath = config.get(section_name, 'aes_keys_path')
        if len(aesKeyPath):
            local_dict['aes_keys_path'] = config.get(section_name, 'aes_keys_path')
        else:
            local_dict['aes_keys_path'] = ' '
            print_and_log(log_file, "Path for AES keys not provided \n")
    else:
        print_and_log(log_file, "The AES keys path parameter is mandatory \n")
        return None, None  

###############Start processing RSA key options ################################

    # Check for the OEM Root certificate RSA key option    
    if config.has_option(section_name, 'pwd_key_oemRootCert_rsa'):
        # Get the key Cert rsa key pwd
        passWord = config.get(section_name, 'pwd_key_oemRootCert_rsa')
        # length of the passphrase should be in the required range set by openssl
        if len(passWord) > 4 and len(passWord) < 1023 :
        # Add passphase to the Dict
            local_dict['pwd_key_oem_root_cert_rsa'] = passWord
        else:
            # passphrase is not good
            print_and_log(log_file, "section " + 'pwd_key_oemRootCert_rsa' + " passphase should be greater than 4 and less than 1023 characters \n")        
            return None, None
    else:
        print_and_log(log_file, "Passphrase for OEM Root-Certificate RSA key not provided \n") 


    # Check for the OEM Key certificate RSA key option    
    if config.has_option(section_name, 'pwd_key_oemKeyCert_rsa'):
        # Get the key Cert rsa key pwd
        passWord = config.get(section_name, 'pwd_key_oemKeyCert_rsa')
        # length of the passphrase should be in the required range set by openssl
        if len(passWord) > 4 and len(passWord) < 1023 :
        # Add passphase to the Dict
            local_dict['pwd_key_oem_key_cert_rsa'] = passWord
        else:
            # passphrase is not good
            print_and_log(log_file, "section " + 'pwd_key_oemKeyCert_rsa' + " passphase should be greater than 4 and less than 1023 characters \n")        
            return None, None
    else:
        print_and_log(log_file, "Passphrase for OEM Root-Certificate RSA key not provided \n")


    # Check for the content certificate OEM key option    
    if config.has_option(section_name, 'pwd_key_contentCertOem_rsa'):
        # Get the key Cert rsa key pwd
        passWord = config.get(section_name, 'pwd_key_contentCertOem_rsa')
        # length of the passphrase should be in the required range set by openssl
        if len(passWord) > 4 and len(passWord) < 1023 :
        # Add passphase to the Dict
            local_dict['pwd_key_oem_content_cert_rsa'] = passWord
        else:
            # passphrase is not good
            print_and_log(log_file, "section " + 'pwd_key_contentCertOem_rsa' + " passphase should be greater than 4 and less than 1023 characters \n")        
            return None, None
    else:
        print_and_log(log_file, "Passphrase for OEM Content-Certificate RSA key not provided \n")


    # Check for the debug key certificate RSA key option 
    if config.has_option(section_name, 'pwd_key_dbgKeyCert_rsa'):
        # Get the key Cert rsa key pwd
        passWord = config.get(section_name, 'pwd_key_dbgKeyCert_rsa')
        # length of the passphrase should be in the required range set by openssl
        if len(passWord) > 4 and len(passWord) < 1023 :
        # Add passphase to the Dict
            local_dict['pwd_key_dbg_key_cert_rsa'] = passWord
        else:
            # passphrase is not good
            print_and_log(log_file, "section " + 'pwd_key_dbgKeyCert_rsa' + " passphase should be greater than 4 and less than 1023 characters \n")        
            return None, None
    else:
        print_and_log(log_file, "Passphrase for ICV Debug Key Certificate RSA key not provided \n")


    # Check for the Debug developer certificate RSA key option    
    if config.has_option(section_name, 'pwd_key_dbgDevlpCert_rsa'):
        # Get the key Cert rsa key pwd
        passWord = config.get(section_name, 'pwd_key_dbgDevlpCert_rsa')
        # length of the passphrase should be in the required range set by openssl
        if len(passWord) > 4 and len(passWord) < 1023 :
        # Add passphase to the Dict
            local_dict['pwd_key_dbg_devlpr_Cert_rsa'] = passWord
        else:
            # passphrase is not good
            print_and_log(log_file, "section " + 'pwd_key_dbgDevlpCert_rsa' + " passphase should be greater than 4 and less than 1023 characters \n")        
            return None, None
    else:
        print_and_log(log_file, "Passphrase for OEM Debug Developer RSA key not provided \n")

    # Check for the Debug Enabler certificate RSA key option 
    if config.has_option(section_name, 'pwd_key_dbgEnablerCert_rsa'):
        # Get the key Cert rsa key pwd
        passWord = config.get(section_name, 'pwd_key_dbgEnablerCert_rsa')
        # length of the passphrase should be in the required range set by openssl
        if len(passWord) > 4 and len(passWord) < 1023 :
        # Add passphase to the Dict
            local_dict['pwd_key_dbg_enblr_cert_rsa'] = passWord
        else:
            # passphrase is not good
            print_and_log(log_file, "section " + 'pwd_key_dbgEnablerCert_rsa' + " passphase should be greater than 4 and less than 1023 characters \n")        
            return None, None
    else:
        print_and_log(log_file, "Passphrase for OEM Debug Enabler Certificate RSA key not provided \n")


    # Check for the OEM Key request RSA key pair  
    if config.has_option(section_name, 'pwd_key_oemKrtlKeyReq_rsa'):
        # Get the key Cert rsa key pwd
        passWord = config.get(section_name, 'pwd_key_oemKrtlKeyReq_rsa')
        # length of the passphrase should be in the required range set by openssl
        if len(passWord) > 4 and len(passWord) < 1023 :
        # Add passphase to the Dict
            local_dict['pwd_key_oem_Krtl_Key_Req_rsa'] = passWord
        else:
            # passphrase is not good
            print_and_log(log_file, "section " + 'pwd_key_oemKrtlKeyReq_rsa' + " passphase should be greater than 4 and less than 1023 characters \n")        
            return None, None
    else:
        print_and_log(log_file, "Passphrase for OEM Krtl key Request not provided \n")         


############################ Process AES keys ######################################

    # Check for the OEM provisioning AES key option    
    if config.has_option(section_name, 'pwd_key_oem_prov_aes'):
        # Get the key pwd
        passWord = config.get(section_name, 'pwd_key_oem_prov_aes')
        # length of the passphrase should be in the required range set by openssl
        if len(passWord) > 4 and len(passWord) < 1023 :
        # Add passphase to the Dict
            local_dict['pwd_oem_prov_aes'] = passWord
        else:
            # passphrase is not good
            print_and_log(log_file, "section " + 'pwd_key_oem_prov_aes' + " passphase should be greater than 4 and less than 1023 characters \n")        
            return None, None
    else:
        print_and_log(log_file, "Passphrase for OEM provisioning key not provided \n") 


    # Check for the OEM code encryption AES key option    
    if config.has_option(section_name, 'pwd_key_oem_code_encpt_aes'):
        # Get the key pwd
        passWord = config.get(section_name, 'pwd_key_oem_code_encpt_aes')
        # length of the passphrase should be in the required range set by openssl
        if len(passWord) > 4 and len(passWord) < 1023 :
        # Add passphase to the Dict
            local_dict['pwd_oem_code_encpt_aes'] = passWord
        else:
            # passphrase is not good
            print_and_log(log_file, "section " + 'pwd_key_oem_code_encpt_aes' + " passphase should be greater than 4 and less than 1023 characters \n")        
            return None, None
    else:
        print_and_log(log_file, "Passphrase for OEM code encryption key not provided \n")             


    return local_dict, config



# Parse script parameters
def parse_shell_arguments ():
    len_arg =  len(sys.argv)
    if len_arg < 2:
        print("len " + str(len_arg) + " invalid. Usage:" + sys.argv[0] + "<test configuration file>\n")
        for i in range(1,len_arg):
            print("i " + str(i) + " arg " + sys.argv[i] + "\n")
        sys.exit(1)
    config_fname = sys.argv[1]
    if len_arg == 3:
        log_fname = sys.argv[2]
    else:
        log_fname = "sb_key_cert.log"
    return config_fname, log_fname

##################################################################
# This function Parse the dictionary for valid enteries in the 
# input config file and generate the keys
###################################################################
def CreateKeys(sysArgsList):        
    try:          
        config_fname, log_fname =  parse_shell_arguments()
             
        log_file = create_log_file(log_fname)             
        # Check the input parameters and save it to list
        ArgsDict, config = key_gen_config_file_parser(config_fname, log_file)
        if ArgsDict == None:
               log_file.close()
               exit(1)
          
        print_and_log(log_file, "**** Generating keys **** \n \n")

                    # check for the out put path directory. If it is not there, then create it             
        if not os.path.exists(ArgsDict['rsa_keys_path']):
            os.makedirs(ArgsDict['rsa_keys_path'])


        # Generate OEM key for Root cert
        if 'pwd_key_oem_root_cert_rsa' in ArgsDict.keys():
            print_and_log(log_file, "\nGenerating OEM Root Certificate RSA key pair, Pass Phrase file and public key \n")

            # write the passphrase to a txt file 
            passPhrasePath = ArgsDict['rsa_keys_path'] + 'pwdOemRootCertKey_Rsa.txt'
            pwdFile = open(passPhrasePath, 'w+')       
            pwdFile.write(ArgsDict['pwd_key_oem_root_cert_rsa'])
            pwdFile.close()
        
            # Generate the rsa key pair
            keyPairPath = ArgsDict['rsa_keys_path'] + 'oemRootCertKeyPair.pem'
            cmd = "openssl genrsa -aes256 -passout pass:" +  ArgsDict['pwd_key_oem_root_cert_rsa'] + " -out " + keyPairPath + ' ' + ArgsDict['rsa_key_strength']
            os.system(cmd)

            # extract the public key from the key pair
            pubKeyPath = ArgsDict['rsa_keys_path'] + 'oemRootCertPublicKey.pem'
            cmd = "openssl rsa -in " + keyPairPath + " -out " +  pubKeyPath + " -passin file:"  +  passPhrasePath + " -pubout"
            os.system(cmd)

        # Generate OEM key for key cert
        if 'pwd_key_oem_key_cert_rsa' in ArgsDict.keys():
            print_and_log(log_file, "\nGenerating OEM Key Certificate RSA key pair, Pass Phrase file and public key \n")

            # write the passphrase to a txt file 
            passPhrasePath = ArgsDict['rsa_keys_path'] + 'pwdOemKeyCertKey_Rsa.txt'
            pwdFile = open(passPhrasePath, 'w+')       
            pwdFile.write(ArgsDict['pwd_key_oem_key_cert_rsa'])
            pwdFile.close()
        
            # Generate the rsa key pair
            keyPairPath = ArgsDict['rsa_keys_path'] + 'oemKeyCertKeyPair.pem'
            cmd = "openssl genrsa -aes256 -passout pass:" +  ArgsDict['pwd_key_oem_key_cert_rsa'] + " -out " + keyPairPath + ' ' + ArgsDict['rsa_key_strength']
            os.system(cmd)

            # extract the public key from the key pair
            pubKeyPath = ArgsDict['rsa_keys_path'] + 'oemKeyCertPublicKey.pem'
            cmd = "openssl rsa -in " + keyPairPath + " -out " +  pubKeyPath + " -passin file:"  +  passPhrasePath + " -pubout"
            os.system(cmd)

        # Generate OEM key for content cert
        if 'pwd_key_oem_content_cert_rsa' in ArgsDict.keys():
            print_and_log(log_file, "\nGenerating OEM content Certificate RSA key pair, Pass Phrase file and public key \n")

            # write the passphrase to a txt file 
            passPhrasePath = ArgsDict['rsa_keys_path'] + 'pwdOemContentCertKey_Rsa.txt'
            pwdFile = open(passPhrasePath, 'w+')       
            pwdFile.write(ArgsDict['pwd_key_oem_content_cert_rsa'])
            pwdFile.close()
        
            # Generate the rsa key pair
            keyPairPath = ArgsDict['rsa_keys_path'] + 'oemContentCertKeyPair.pem'
            cmd = "openssl genrsa -aes256 -passout pass:" +  ArgsDict['pwd_key_oem_content_cert_rsa'] + " -out " + keyPairPath + ' ' + ArgsDict['rsa_key_strength']
            os.system(cmd)

            # extract the public key from the key pair
            pubKeyPath = ArgsDict['rsa_keys_path'] + 'oemContentCertPublicKey.pem'
            cmd = "openssl rsa -in " + keyPairPath + " -out " +  pubKeyPath + " -passin file:"  +  passPhrasePath + " -pubout"
            os.system(cmd)
            

        # Generate Debug Key certificate key
        if 'pwd_key_dbg_key_cert_rsa' in ArgsDict.keys():
            print_and_log(log_file, "\nGenerating OEM Debug Key Certificate RSA key pair, Pass Phrase file and public key \n")

            # write the passphrase to a txt file 
            passPhrasePath = ArgsDict['rsa_keys_path'] + 'pwdOemDbgKeyCertKey_Rsa.txt'
            pwdFile = open(passPhrasePath, 'w+')       
            pwdFile.write(ArgsDict['pwd_key_dbg_key_cert_rsa'])
            pwdFile.close()
        
            # Generate the rsa key pair
            keyPairPath = ArgsDict['rsa_keys_path'] + 'dbgOemKeyCertKeyPair.pem'
            cmd = "openssl genrsa -aes256 -passout pass:" +  ArgsDict['pwd_key_dbg_key_cert_rsa'] + " -out " + keyPairPath + ' ' + ArgsDict['rsa_key_strength']
            os.system(cmd)

            # extract the public key from the key pair
            pubKeyPath = ArgsDict['rsa_keys_path'] + 'dbgOemKeyCertPublicKey.pem'
            cmd = "openssl rsa -in " + keyPairPath + " -out " +  pubKeyPath + " -passin file:"  +  passPhrasePath + " -pubout" 
            os.system(cmd)


        # Generate Debug developer Certificate key
        if 'pwd_key_dbg_devlpr_Cert_rsa' in ArgsDict.keys():
            print_and_log(log_file, "\nGenerating Debug developer Certificate RSA key pair, Pass Phrase file and public key \n")

            # write the passphrase to a txt file 
            passPhrasePath = ArgsDict['rsa_keys_path'] + 'pwdOemDbgDevlprCert_Rsa.txt'
            pwdFile = open(passPhrasePath, 'w+')       
            pwdFile.write(ArgsDict['pwd_key_dbg_devlpr_Cert_rsa'])
            pwdFile.close()
        
            # Generate the rsa key pair
            keyPairPath = ArgsDict['rsa_keys_path'] + 'dbgOemDevlprCertKeyPair.pem'
            cmd = "openssl genrsa -aes256 -passout pass:" +  ArgsDict['pwd_key_dbg_devlpr_Cert_rsa'] + " -out " + keyPairPath + ' ' + ArgsDict['rsa_key_strength']
            os.system(cmd)

            # extract the public key from the key pair
            pubKeyPath = ArgsDict['rsa_keys_path'] + 'dbgOemDevlprCertPublicKey.pem'
            cmd = "openssl rsa -in " + keyPairPath + " -out " +  pubKeyPath + " -passin file:"  +  passPhrasePath + " -pubout"
            os.system(cmd)


        # Generate Debug enabler certificate key
        if 'pwd_key_dbg_enblr_cert_rsa' in ArgsDict.keys():
            print_and_log(log_file, "\nGenerating OEM Debug Enabler Certificate RSA key pair, Pass Phrase file and public key \n")

            # write the passphrase to a txt file 
            passPhrasePath = ArgsDict['rsa_keys_path'] + 'pwdOemDbgEnblrCertKey_Rsa.txt'
            pwdFile = open(passPhrasePath, 'w+')       
            pwdFile.write(ArgsDict['pwd_key_dbg_enblr_cert_rsa'])
            pwdFile.close()
        
            # Generate the rsa key pair
            keyPairPath = ArgsDict['rsa_keys_path'] + 'dbgOemEnablerCertKeyPair.pem'
            cmd = "openssl genrsa -aes256 -passout pass:" +  ArgsDict['pwd_key_dbg_enblr_cert_rsa'] + " -out " + keyPairPath + ' ' + ArgsDict['rsa_key_strength']
            os.system(cmd)

            # extract the public key from the key pair
            pubKeyPath = ArgsDict['rsa_keys_path'] + 'dbgOemEnablerCertPublicKey.pem'
            cmd = "openssl rsa -in " + keyPairPath + " -out " +  pubKeyPath + " -passin file:"  +  passPhrasePath + " -pubout" 
            os.system(cmd)


        # Generate krtl key request Certificate key
        if 'pwd_key_oem_Krtl_Key_Req_rsa' in ArgsDict.keys():
            print_and_log(log_file, "\nGenerating OEM Krtl key request RSA key pair, Pass Phrase file and public key \n")

            # write the passphrase to a txt file 
            passPhrasePath = ArgsDict['rsa_keys_path'] + 'pwdOemKrtlKeyRequest_Rsa.txt'
            pwdFile = open(passPhrasePath, 'w+')       
            pwdFile.write(ArgsDict['pwd_key_oem_Krtl_Key_Req_rsa'])
            pwdFile.close()
        
            # Generate the rsa key pair
            keyPairPath = ArgsDict['rsa_keys_path'] + 'oemKrtlKeyReqKeyPair.pem'
            cmd = "openssl genrsa -aes256 -passout pass:" +  ArgsDict['pwd_key_oem_Krtl_Key_Req_rsa'] + " -out " + keyPairPath + ' ' + ArgsDict['rsa_key_strength']
            os.system(cmd)

            # extract the public key from the key pair
            pubKeyPath = ArgsDict['rsa_keys_path'] + 'oemKrtlKeyReqPublicKey.pem'
            cmd = "openssl rsa -in " + keyPairPath + " -out " +  pubKeyPath + " -passin file:"  +  passPhrasePath + " -pubout"
            os.system(cmd)


#####################################  AES KEYS ###############################################

        # calculate AES key strength in bytes
        aesKeyStrength = int(int(ArgsDict['aes_key_strength']) / 8)

        # check for the out put path directory. If it is not there, then create it             
        if not os.path.exists(ArgsDict['aes_keys_path']):
            os.makedirs(ArgsDict['aes_keys_path'])


        # Generate OEM provisioning key
        if 'pwd_oem_prov_aes' in ArgsDict.keys():
            print_and_log(log_file, "\nGenerating OEM provisioning AES key and Pass Phrase file \n")

            # write the passphrase to a txt file 
            passPhrasePath = ArgsDict['aes_keys_path'] + 'pwdKcp.txt'
            pwdFile = open(passPhrasePath, 'w+')       
            pwdFile.write(ArgsDict['pwd_oem_prov_aes'])
            pwdFile.close()
        
            # Generate the aes key random number
            cmd = "openssl rand -hex -out temp.txt " + str(aesKeyStrength)
            os.system(cmd)

            file = open("temp.txt", 'r')
            keyData = file.read()
            file.close()

            cmd = "rm temp.txt"
            os.system(cmd)

            binary_string = binascii.unhexlify(keyData[0:32])                
            keyFilePath = ArgsDict['aes_keys_path'] + "kcp.bin"

            file = open(keyFilePath, 'wb')
            keyData = file.write(binary_string)
            file.close()
           
            # Encrypt the AES key using pass-phrase with no salt
            aesKeyPath = ArgsDict['aes_keys_path'] + 'Kcp_enc.bin'
            cmd = "openssl enc -e -nosalt -aes-128-cbc" + " -k " + ArgsDict['pwd_oem_prov_aes'] + " -in " + keyFilePath + " -out " + aesKeyPath
            os.system(cmd)

           
        # Generate OEM code encryption key
        if 'pwd_oem_code_encpt_aes' in ArgsDict.keys():
            print_and_log(log_file, "\nGenerating OEM code encryption AES key and Pass Phrase file \n")

            # write the passphrase to a txt file 
            passPhrasePath = ArgsDict['aes_keys_path'] + 'pwdKce.txt'
            pwdFile = open(passPhrasePath, 'w+')       
            pwdFile.write(ArgsDict['pwd_oem_code_encpt_aes'])
            pwdFile.close()
        
            # Generate the aes key random number
            cmd = "openssl rand -hex -out temp.txt " + str(aesKeyStrength)
            os.system(cmd)

            file = open("temp.txt", 'r')
            keyData = file.read()
            file.close()

            cmd = "rm temp.txt"
            os.system(cmd)

            binary_string = binascii.unhexlify(keyData[0:32])
            keyFilePath = ArgsDict['aes_keys_path'] + "kce.bin"

            file = open(keyFilePath, 'wb')
            keyData = file.write(binary_string)
            file.close()
           
            # Encrypt the AES key using pass-phrase with no salt
            aesKeyPath = ArgsDict['aes_keys_path'] + 'Kce_enc.bin'
            cmd = "openssl enc -e -nosalt -aes-128-cbc" + " -k " + ArgsDict['pwd_oem_code_encpt_aes'] + " -in " +  keyFilePath + " -out " + aesKeyPath
            os.system(cmd)

   
    except IOError as Error8: 
        (errno, strerror) = Error8.args 
        print_and_log(log_file, "I/O error(%s): %s" % (errno, strerror))
        raise
    except NameError:
        print_and_log(log_file, "Unexpected error, exiting program")
        raise  # Debug info
    except ValueError:
        print_and_log(log_file, "Illegal variable type")
        raise # Debug info


##################################
#       Main function
##################################
        
if __name__ == "__main__":

    import sys
    if sys.version_info<(3,0,0):
        print("You need python 3.0 or later to run this script")
        exit(1)

    CreateKeys(sys.argv)



