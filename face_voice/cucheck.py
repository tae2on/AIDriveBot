""" import cv2

print('Is OpenCV built with CUDA:', cv2.ocl.haveOpenCL())

if cv2.ocl.haveOpenCL():
    cv2.ocl.setUseOpenCL(True)
    print('Using CUDA.')
else:
    print('Not using CUDA.')
 """

import cv2

# Get OpenCV build information
build_info = cv2.getBuildInformation()

# Check if the build information includes CUDA support
if 'CUDA' in build_info:
    print('OpenCV is built with CUDA support.')
else:
    print('OpenCV is not built with CUDA support.')
