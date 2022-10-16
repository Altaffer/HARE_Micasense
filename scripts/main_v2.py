#!/usr/bin/env python3


## Micasense Firmware for Plant Disease Detection Using Hyperspectral Imagery
## Written by Matt Rochford (mrochfor@ucsc.edu) and Nick Bender (nibender@ucsc.edu) 


# Import Libraries
import cv2
import requests
import numpy as np
#from PIL import Image
import time
from cv_bridge import CvBridge
import rospy
from micasense_wedge.msg import micasense
from multiprocessing import Process,cpu_count


# Define global variables
wifi = False # If connecting through ethernet set to false
if wifi:
    host = "http://192.168.10.254/"
else:
    host = "http://192.168.1.83/"

# Initialize ROS Parameters
mica_pub = rospy.Publisher('micasense_data', micasense)
rospy.init_node('micasense_wedge', anonymous=True)
bridge = CvBridge()

# Initialize persistent session
s = requests.Session() 

def main():
    """
    Making HTTP requests to micasense and returning image data

    Documentation available here:
    https://micasense.github.io/rededge-api/api/http.html

    Available functions for checking Micasense statuses and configs:

    clear_sd_storage() # NOT WORKING
    config() 
    config('POST', streaming_enable=False)
    status()
    network_status()
    """

    # Get start time for testing
    start = time.time() 
    i = 0

    # Collect data while ROS is not shutdown
    while not rospy.is_shutdown():

        try:
            # Initiate capture and return paths to cached files
            paths = initiate_capture()

            # Download files and create ROS message object
            mica_msg = load_data_from_files(paths)

            # Publish full ROS message    
            mica_pub.publish(mica_msg)
            i += 1

            if i%100 == 0:
            	print(f'{i} messages published')
            

            if i%1000 == 0:

                end = time.time()

                script_time = round((end-start)/60,2)

                print(f'Script has been running for {script_time} minutes')

        except Exception as e: 

            print(e)

            #exit()


def load_data_from_files(paths):
    """
    This function continually loads all files specified in the input

    Inputs:
    paths - dictionary of file names

    Outputs:
    mica_msg - ROS custom message for micasense data
    """   

    # Initialize micasense message
    mica_msg = micasense()

    # Load image from each band and update micasense message
    for key,value in paths.items():
        
        file_path = host[:-1]+value
        #print(file_path)

        img_arr = retrieve_data_from_file(file_path)

        # Convert numpy array to ROS array
        ros_img = bridge.cv2_to_imgmsg(img_arr,encoding="passthrough")

        # Update micasense message (Note: Not 100% on these but I assumed images are ordered by wavelength)
        if key == '1': 
            mica_msg.img_b = ros_img
        elif key == '2':
            mica_msg.img_g = ros_img
        elif key == '3':
            mica_msg.img_r = ros_img
        elif key == '4':
            mica_msg.img_nir = ros_img
        elif key == '5': 
            mica_msg.img_swir = ros_img

    return mica_msg


def capture_process():
    """
    Simple function to be used with multiprocessing module to continually make capture requests to the micasense. 
    
    This function will continue making capture requests until the micasense loses power.
    """

    # Collect data while ROS is not shutdown
    while not rospy.is_shutdown():

        initiate_capture()


def data_load_process(i):
    """
    This function continually loads all files from the cache in order (FIFO).

    Inputs:
    i - first file number to start downloading

    Outputs:
    none (ROS messages will continually be published)
    """
        
    # Initialize ROS Parameters
    mica_pub = rospy.Publisher('micasense_data', micasense,queue_size=10)
    rospy.init_node('micasense_wedge', anonymous=True)
    bridge = CvBridge()

    # Initialize micasense message
    mica_msg = micasense()

    # Download data while ROS is not shutdown
    while not rospy.is_shutdown():

        # Format file path
        file_path = '{}images/tmp{}.tif'.format(host,i)

        # Try to load file
        try:

            # If this fails, loop will keep trying until it succeeds 
            img_arr = retrieve_data_from_file(file_path)
          
            # Convert numpy array to ROS array
            ros_img = bridge.cv2_to_imgmsg(img_arr,encoding="passthrough")

            # Update micasense message (Note: Not 100% on these but I assumed images are ordered by wavelength)
            if i%5 == 0: 
                mica_msg.img_b = ros_img
            elif i%5 == 1:
                mica_msg.img_g = ros_img
            elif i%5 == 2:
                mica_msg.img_r = ros_img
            elif i%5 == 3:
                mica_msg.img_nir = ros_img
            elif i%5 == 4: 
                mica_msg.img_swir = ros_img

                # Publish full ROS message after loading last band
                mica_pub.publish(mica_msg)

                print('message published') # Optional print message

                # Initialize micasense message for next round (This could potentially be removed)
                mica_msg = micasense()

            # Increment counter for loading next file
            i += 1 

        except Exception as e:
            
            #print(e) # For debugging
            #print(file_path) # For debugging
            
            continue # Skip rest of loop


def initiate_capture(store_capture=False,block=True,bitmask=31,cache_type='raw'):
    """
    This function intiates a capture and returns the file paths to the captured images
    First image taken upon power up will be /images/tmp0.tif

    Inputs:
    store_capture = boolean - determines whether or not to store images to sd card on micasense (default is false)
    block = boolean - When 'true', the HTTP request will not return until the capture is complete (default is True)
    bitmask = int(1-31) - determines which bands are captured. Use 31 for all 5 bands or 8 for red edge only (default is 31)
    cache_type = string - acceptable values are either 'raw' or 'jpeg' (default is raw)

    Outputs:
    paths = dictionary of file paths on micasense (using cache)
    """

    # Create request string
    request_string = f"capture?cache_{cache_type}={bitmask}&block={block}&store_capture={store_capture}" # For cache
    
    # Make capture request to Micasense

    #print('Initiating capture request') # Optional print message
    
    r = s.get(host+request_string)

    data = r.json() # Convert to json

    # Grab paths from JSON data
    path_type = '{}_cache_path'.format(cache_type)
    paths = data[path_type]

    return paths


def retrieve_data_from_file(file_path):
    """
    Retrieve data from micasense and convert it to usable data

    Inputs:
    file_path = full file path on micasense (string)

    Outputs:
    img_data = numpy array of pixel data
    """

    # Get file object from Micasense
    r = s.get(file_path,stream=True,timeout=0.5)

    # Convert bytes to numpy array and reshape to image dimensions
    img_data = np.frombuffer(r.content,dtype=np.dtype(np.uint16)) #,offset=5,count=1228800) # Can try messing around with this for marginal speed improvements
    img_data = img_data[4:1228804] # Magic numbers to make it work
    img_data = img_data.reshape(960,1280) # This is the resolution provided by micasense
    
    return img_data


def get_cache_number():
    """
    Initiates the first capture and returns the file number to begin the download process with.
    When this script is first executed it will be 0. But each capture will increase the number.
    This function is used to enable the script to run without a global counter or file list.

    Inputs:
    none

    Outputs:
    num = int - number of the first file captured
    """

    print('Getting starting number for cache')

    # Initiate the first capture and return the paths
    paths = initiate_capture()

    # Get path to first capture
    last_file = paths['1'] 

    # Split the path twice to get the number
    last_file = last_file.split('tmp')[-1]
    num = last_file.split('.')[0]

    # Convert from string to int
    num = int(num)
    
    return num


def config(request_type='GET',**kwargs):
    """
    This function handles the Micasense configuration options.
    If request_type='GET' then 

    Inputs:
    request_type = 'GET' or 'POST'
    **kwargs = dictionary of values to change (only used for POST requests)

    Ouputs:
    none
    """

    # Define config path
    config_path = host+'config/'
    #print(config_path)

    # Handle GET requests
    if request_type == 'GET':

        r = requests.get(config_path)
        
        # Print config information
        for key,val in r.json().items():
            print(str(key)+'='+str(val))

    # Handle POST requests
    elif request_type == 'POST':

        r = requests.post(config_path,data=kwargs) 
        
        print('Config updated, printing new config:')
        config()
    
    else:
        raise ValueError("config request_type must be 'GET' or 'POST'")


def status():
    """
    This function returns the status of the camera

    Inputs:
    none

    Ouputs:
    none (status printed to terminal)
    """

    # Define config path
    status_path = host+'status'

    # Make get requests
    r = requests.get(status_path)

    # Print status information
    for key,val in r.json().items():
        print(str(key)+'='+str(val))


def network_status(storage=False):
    """
    This function returns the status of the camera

    Inputs:
    storage = boolean - If true will only print the storage available on sd card. Otherwise will print everything

    Ouputs:
    none (network status printed to terminal)
    """

    # Define network status path
    network_status_path = host+'networkstatus'

    # Make get requests
    r = requests.get(network_status_path)

    data = r.json()['network_map']

    if storage:

        key = 'sd_gb_free'

        for device in data:
            print(key+'='+str(device[key]))

    else:

        for device in data:
            for key,val in device.items():
                print(str(key)+'='+str(val))


def clear_sd_storage(): # Not working right now
    """
    Clears all saved files on the Micasense SD card

    Inputs:
    none

    Outputs:
    none (statistics printed to terminal)
    """

    print('Clearing SD card storage on Micasense')
    print('Storage before reformatting')
    network_status(storage=True)

    # Define request path
    request_path = host+'reformatsdcard'
    #print(request_path)

    # Make post request
    r = s.post(request_path,data={"erase_all_data":True})

    # Print result
    print(r.json()['message'])

    print('Storage after reformatting')
    network_status(storage=True)


if __name__ == '__main__':
    main()
