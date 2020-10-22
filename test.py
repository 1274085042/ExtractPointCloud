from ctypes import *
preprocess_point=cdll.LoadLibrary("lib/libextract_point_cloud.so")

#ep=preprocess_point.extract_point("../data/000001.pcd","../output")
#ep.extract_time()