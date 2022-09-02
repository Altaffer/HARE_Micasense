#!/usr/bin/env python3


## Micasense Firmware for Plant Disease Detection Using Hyperspectral Imagery
## Written by Matt Rochford (mrochfor@ucsc.edu) and Nick Bender (nibender@ucsc.edu) 


# Import Libraries
import requests
import numpy as np
from PIL import Image
import time
from cv_bridge import CvBridge
import rospy
from micasense_wedge.msg import micasense


# Define global variables
wifi = False # If connecting through ethernet set to false
if wifi:
    host = "http://192.168.10.254/"
else:
    host = "http://192.168.1.83/"


def main():
    """
    Making HTTP requests to micasense and returning image data

    Documentation available here:
    https://micasense.github.io/rededge-api/api/http.html

    GET request arguments (argument - type - default value)
    anti_sat - bool - false (use anti-saturation rules for image capture)
    block - bool - false (if true, HTTP request will not return until capture is complete)
    detect_panel - false (if true, camera will not return image until a reflectance panel is detected)
    preview - bool - false (if true, current preview image is updated)
    cache_jpeg - int - /config file (bitmask for bands to capture, use 31 for all 5 bands)
    cache_raw - int - /config file
    store_capture - bool - true (store image to SD card)
    """
    
    # Available functions for checking Micasense statuses and configs
    #clear_sd_storage() # NOT WORKING
    #config() 
    #config('POST',raw_format='TIFF')
    #status()
    #network_status()

    # Initialize ROS Parameters
    mica_pub = rospy.Publisher('micasense_data', micasense)
    rospy.init_node('micasense_wedge', anonymous=True)
    bridge = CvBridge()

    # Collect data while ROS is not shutdown
    while not rospy.is_shutdown():

        # Initiate capture and return paths to cached files
        paths = initiate_capture()
        
        # Initialize micasense message
        mica_msg = micasense()

        # Load image from each band and update micasense message
        for key,value in paths.items():

            file_path = host[:-1]+value

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
                mica_msg.img_swi = ros_img

            #print('Array shape: '+str(img_arr.shape))
            #input('press enter to continue')
        
        # Publish full ROS message    
        mica_pub.publish(mica_msg)
            

def initiate_capture(store_capture=False):
    """
    This function intiates a capture and returns the file paths to the captured images

    Inputs:
    store_capture = boolean - determines whether or not to store images to sd card on micasense (default is false)

    Outputs:
    paths = dictionary of file paths on micasense (using cache)
    """

    # Define capture parameters
    block = True
    bitmask = 31 # Capture all 5 bands
    cache_type = 'raw'

    # Create request string
    request_string = f"capture?cache_{cache_type}={bitmask}&block={block}&store_capture={store_capture}" # For cache
    
    # Make capture request to Micasense
    #print('Initiating capture request')
    r = requests.get(host+request_string)
    
    data = r.json()

    # Grab paths from JSON data
    paths = data['raw_cache_path']

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
    r = requests.get(file_path,stream=True)

    # Convert bytes to numpy array and reshape to image dimensions
    img_data = np.frombuffer(r.content,dtype=np.dtype(np.uint16))#,offset=5,count=1228800)
    img_data = img_data[4:1228804]
    img_data = img_data.reshape(960,1280)
    
    return img_data


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
    request_string = 'reformatsdcard'

    # Make post request
    r = requests.post(host+request_string,data={'erase_all_data':True})

    # Print result
    print(r.json()['message'])

    print('Storage after reformatting')
    network_status(storage=True)


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


if __name__ == '__main__':
    main()