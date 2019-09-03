#!/usr/bin/env python
# -*- coding: utf-8 -*-

## Created on 23.06.2016
## author: Simon T.
## maintainer: Y. K.

import sys

sys.path.append("/usr/local/lib/python2.7/dist-packages") # for exiftool

import Metashape as PS
import os
import glob
import configparser
import exiftool
import operator
import csv
import shutil
import zipfile
import logging
import datetime
#import yaml

LOG_DIR="/home/ankommen/logs/metashape/"
LOG_FILE=datetime.datetime.now().isoformat()+".log"
RGB_IMAGE_INTERVAL=6.0
THERMAL_IMAGE_INTERVAL=0.0

if not os.path.exists(os.path.dirname(LOG_DIR)):
		os.makedirs(os.path.dirname(LOG_DIR))

logging.basicConfig(filename=LOG_DIR+LOG_FILE, filemode='w', format='%(asctime)s - %(message)s', datefmt='%d-%b-%y %H:%M:%S', level=logging.INFO)
logging.info("Starting Metashape script.")

def createPSImageList(image_path, interval):
	'''Creates a list of images to be processed by PS'''
	logging.info("######## Creating Lists for processing Started  ############")
	list_file = open(image_path+'/list.csv','w')
	file_list_sorted=os.listdir(image_path)
	file_list_sorted.sort()
	img_count = 0
	with exiftool.ExifTool() as et:
		for file in file_list_sorted:
			if file.endswith(".jpg"):
				file_path = os.path.join(image_path, file)
				dateTimeString = et.get_tag("DateTimeOriginal", file_path)
				if dateTimeString:
					tmp_lst=dateTimeString.split(' ')[1].split(':')
					seconds=float(tmp_lst[0])*3600+float(tmp_lst[1])*60+float(tmp_lst[2])
					list_file.write(file_path+','+str(seconds)+'\n')
				else: # A fall-back option for images with no time stamp.
					img_count = img_count + 1
					list_file.write(file_path+','+str(img_count)+'\n')
	list_file.close()

	data = csv.reader(open(image_path+'/list.csv'),delimiter=',')
	sortedlist = sorted(data,key=operator.itemgetter(1))

	initTime=float(sortedlist[0][1])
	final_list_file = open(image_path+'/list.txt','w')
	final_list_file.write(sortedlist[0][0]+'\n')
	for element in sortedlist:
		second=float(element[1])
		if second - initTime >= float(interval):
			final_list_file.write(element[0]+'\n')
			initTime = second
	final_list_file.close()
	logging.info("Total Images in directory: " + image_path + "  are:  " + str(len(file_list_sorted)))
	logging.info("######## Creating Lists for processing  Completed ############")

def sync_ankommen(image_path, quality):
	logging.info("######## Sync Ankommen Started.  ############")
	home = os.path.join(os.path.expanduser("~"), "ankdata_gs")
	dir_name = image_path.replace("_rgb", "")
	data = [{'type': 'images',      'sub_type': "dsm",      'inp': 'dem_',         'out': '_rgb_photoscan',     'band': 'rgb', 	'ext': '.tif'},
			{'type': 'images',      'sub_type': "dsm",      'inp': 'dem_',         'out': '_rgb_photoscan',     'band': 'rgb', 	'ext': '.tfw'},
			{'type': 'images',      'sub_type': "dsm",      'inp': 'dem_',         'out': '_rgb_photoscan',     'band': 'rgb', 	'ext': '.zip'},
			{'type': 'images',      'sub_type': "ortho",    'inp': 'ortho_',       'out': '_rgb_photoscan', 	'band': 'rgb', 	'ext': '.tif'},
			{'type': 'images',      'sub_type': "ortho",    'inp': 'ortho_',       'out': '_rgb_photoscan', 	'band': 'rgb', 	'ext': '.tfw'},
			{'type': 'images',      'sub_type': "ortho",    'inp': 'ortho_',       'out': '_rgb_photoscan', 	'band': 'rgb', 	'ext': '.zip'},
			{'type': 'images',      'sub_type': "ortho",    'inp': 'ortho_ir_',    'out': '_th_photoscan',  	'band': 'th', 	'ext': '.tif'},
			{'type': 'images',      'sub_type': "ortho",    'inp': 'ortho_ir_',    'out': '_th_photoscan',  	'band': 'th', 	'ext': '.tfw'},
			{'type': 'images',      'sub_type': "ortho",    'inp': 'ortho_ir_',    'out': '_th_photoscan',  	'band': 'th', 	'ext': '.zip'},
			{'type': 'pointclouds', 'sub_type': "photoscan",'inp': 'pw_ascii_',    'out': 'pc_photoscan', 		'band': '', 	'ext': '.ply'}]
	for el in data:
		src = os.path.join(image_path, 'PSResults', el['inp']+quality+el['ext'])
		dst_dir = os.path.basename(dir_name.replace("raw", el['sub_type']))
		if el['type'] == 'pointclouds':
			dst_dir = os.path.basename(dir_name.replace("raw", ""))
		dst_path = os.path.join(home, el['type'], el['sub_type'])
		dst = os.path.join(dst_path, dst_dir+el['out'], dst_dir+el['out']+el['ext']) # mainfolder, folder, file

		if not os.path.exists(os.path.dirname(dst)):
			os.makedirs(os.path.dirname(dst))
		shutil.copy(src, dst)

		if el['ext'] == '.zip':
			zip_ref = zipfile.ZipFile(src, 'r')
			zip_ref.extractall(os.path.dirname(dst))
	logging.info("######## Sync Ankommen Completed.  ############")

def processing(dwg, quality):
	'''Contains process-steps that are called in PhotoScan'''
	logging.info("####### Processing Started.   ########")
	config = configparser.ConfigParser()
	cwd = os.path.dirname(os.path.abspath(__file__))
	config_file_path = cwd+'/config_cor.ini'
	config.read(config_file_path)

	############ Multi-Spectral things are changed here ########
	input_dir_path = config["PATHS"]["input_images_path_manta"]
	input_dir_path_thermal = config["PATHS"]["input_images_path_thermal"]

	save_path = input_dir_path +"/PSResults/"+config["MISC"]["pro_name"]

	types = (input_dir_path+"/*.jpg",input_dir_path+"/*.JPG")
	all_fotos = []
	for files in types:
		all_fotos.extend(glob.glob(files))

	if not os.path.exists(os.path.dirname(save_path)):
		os.makedirs(os.path.dirname(save_path))

	doc = PS.app.document
	doc.save(path = save_path +".psx")

	chunk = doc.addChunk ()
	chunk.label = config["MISC"]["pro_name"] +"_"+ quality

	input_list_file = input_dir_path + '/list.txt'
	input_list_file_thermal = input_dir_path_thermal + '/list.txt'

	input_fotos = [line.rstrip('\n') for line in open(input_list_file)]
	input_fotos_thermal = [line.rstrip('\n') for line in open(input_list_file_thermal)]

	#logging.info(input_fotos)
	#logging.info(input_fotos_thermal)
	multi_photos = []
	for i in range(len(input_fotos)):
		index_thermal = int(input_fotos[i].split("_")[-1].split(".")[0])
		logging.info("Thermal Index: "+str(index_thermal))
		multi_photos.append([input_fotos[i], input_fotos_thermal[index_thermal]])

	#logging.info(multi_photos)
	chunk.addPhotos(multi_photos, Metashape.MultiplaneLayout)

	logging.info(chunk.cameras)
	logging.info("######### SENSORS   ###########")
	logging.info(chunk.sensors)
	############# Multi-Spectral things are changed here ########
	if config["MISC"]["calibfile"] == "true":
		calib = PS.Calibration()
		calib.load(config["PATHS"]["calibfile_path"])

	doc.save(path = save_path +".psx")
	
	#The 2 sensors in Ankommen are: [<Sensor 'A65X (13mm)'>, <Sensor 'Manta G917C (10.4mm)'>]
	for s in chunk.sensors:
		if "Manta" in s.label:
			new_master = s
	for s in chunk.sensors:
		s.master = None
		s.fixed_rotation = False
	for s in chunk.sensors:
		s.master = new_master
	new_master.fixed_rotation = True

	for s in chunk.sensors:
		print(s.master, s.label, s.key, s.bands)
		logging.info("Sensor Details: " + str(s.master) + "  ,  " + str(s.label)+ "  ,  " +str(s.key)+ "  ,  " + str(s.bands))

	logging.info("##### Master Sensor is: " + str(new_master))
	logging.info("##### The Cameras start: ")

	camera_list = chunk.cameras
	logging.info("Cameras: ", camera_list)
	logging.info("The Cameras end: ")

	chunk.matchPhotos(accuracy = dwg["accuracy"],
					preselection = dwg["preselection"],
					filter_mask = dwg["filter_mask"], keypoint_limit = dwg["keypoint_limit"],
			 tiepoint_limit = dwg["tiepoint_limit"])

	chunk.alignCameras()
	chunk.optimizeCameras(fit_f=True, fit_cxcy=True, fit_b1=True, fit_b2=True,
						  fit_k1k2k3=True, fit_p1p2=True, fit_k4=True,
						  fit_p3=True, fit_p4=True)
	doc.save(path = save_path +".psx")

	###########################################################################
	if dwg["no_DenseCloud"] == False:
		chunk.buildDepthMaps(quality = dwg["quality"], filter = dwg["filter"], reuse_depth=False)
		chunk.buildDenseCloud()
	###########################################################################

	doc.save(path = save_path +".psx")

	#chunk.buildModel(dwg["surface_M"], dwg["interpolation"], dwg["face_count"])

	#doc.save(path = save_path +".psx")

	#chunk.buildUV(PS.MappingMode.AdaptiveOrthophotoMapping,count=1)
	#chunk.buildTexture(PS.BlendingMode.MosaicBlending)

	#doc.save(path = save_path +".psx")

	chunk.buildDem(dwg["source_cloud"], dwg["interpolation"])

	doc.save(path = save_path +".psx")

	chunk.buildOrthomosaic(surface = dwg["surface_O"], blending = dwg["blending"])

	chunk.buildContours(source_data = dwg["source_data"], interval = dwg["interval"])

	doc.save(path = save_path +".psx")
	logging.info("####### Processing Completed.   ########")

	return


def export(qual):
	'''exports the results from the project, into desired formats'''
	#print ("########Exporting############")
	logging.info("######## Exporting the results.  ############")

	doc = PS.app.document

	config = configparser.ConfigParser()
	cwd = os.path.dirname(os.path.abspath(__file__))
	config_file_path = cwd+'/config_cor.ini'
	config.read(config_file_path)
	save_path = config["PATHS"]["input_images_path_manta"] +"/PSResults/"
	crs = Metashape.CoordinateSystem("EPSG::32632")

	for chunk in doc.chunks:
		if qual == chunk.label.split("_")[-1]:
			logging.info(chunk.label.split("_")[-1])

			#orhtopfadjpg = save_path +"ortho_"+qual +".jpg"
			#chunk.exportOrthomosaic(path = orhtopfadjpg, image_format = PS.ImageFormat.ImageFormatJPEG)

			#orhtopfadtiff = save_path +"ortho_"+qual +"_high_res.tif"
			#chunk.exportOrthomosaic(path = orhtopfadtiff, image_format = PS.ImageFormat.ImageFormatTIFF, projection=crs)

			orhtopfadtiff_low = save_path +"ortho_"+qual +".tif"
			chunk.exportOrthomosaic(path = orhtopfadtiff_low, image_format = PS.ImageFormat.ImageFormatTIFF, write_world=True, 
									tiff_compression = PS.TiffCompression.TiffCompressionJPEG, jpeg_quality=70)

			orthotiles_rgb = save_path + "ortho_"+ qual +".zip"
			chunk.exportOrthomosaic( path = orthotiles_rgb, format = PS.RasterFormatTMS, image_format = PS.ImageFormat.ImageFormatPNG,
									 projection = crs)

			dempfad = save_path +"dem_"+qual +".tif"
			chunk.exportDem(path = dempfad, image_format = PS.ImageFormat.ImageFormatTIFF, write_world=True, nodata =-32767)

			demtilespfad = save_path +"dem_"+qual +".zip"
			chunk.exportDem(path = demtilespfad, format = PS.RasterFormatTMS, raster_transform= PS.RasterTransformType.RasterTransformPalette,
							image_format = PS.ImageFormat.ImageFormatPNG, nodata =-32767)

			# Now export in the infra red
			##raster_transform = Metashape.RasterTransformType.RasterTransformPalette
			chunk.raster_transform.formula = ["B4"]
			chunk.raster_transform.calibrateRange()
			chunk.raster_transform.enabled = True
			chunk.raster_transform.palette = {0.0: (0, 0, 0),
												1.0: (255, 255, 255)}
			#orhtopfadtiff1 = save_path +"ortho_ir_"+qual +"_high_res.tif"
			#chunk.exportOrthomosaic(path = orhtopfadtiff1, image_format = PS.ImageFormat.ImageFormatTIFF,
			#							raster_transform = PS.RasterTransformType.RasterTransformPalette, projection=crs)

			orhtopfadtiff1_low = save_path +"ortho_ir_"+qual +".tif"
			chunk.exportOrthomosaic(path = orhtopfadtiff1_low, image_format = PS.ImageFormat.ImageFormatTIFF, write_world=True,
										raster_transform = PS.RasterTransformType.RasterTransformPalette, 
										tiff_compression = PS.TiffCompression.TiffCompressionJPEG, jpeg_quality=70)

			orthotiles_th = save_path + "ortho_ir_"+ qual +".zip"
			chunk.exportOrthomosaic( path = orthotiles_th, format = PS.RasterFormatTMS, image_format = PS.ImageFormat.ImageFormatPNG,
									 raster_transform = PS.RasterTransformType.RasterTransformPalette, projection = crs)

			#modelpfad = save_path +"model_"+qual +".ply"
			#chunk.exportModel(path = modelpfad, projection=crs)

			pointcloudfad = save_path +"pw_"+qual +".ply"
			chunk.exportPoints(path = pointcloudfad, projection=crs)

			pointcloudasciifad = save_path +"pw_ascii_"+qual +".ply"
			chunk.exportPoints(path = pointcloudasciifad, binary=False, precision=3, normals=False, projection=crs)

			reppfad = save_path +"rep_"+qual +".pdf"

			chunk.exportReport(reppfad,chunk.label)

			doc.save(path = save_path + config["MISC"]["pro_name"] +".psx")

	logging.info("######## Exporting the results Completed.  ############")


def choose_profile(quality):
	'''creates a dict with options for the desired quality param in PhotoScan,
	depending on what quality-profile was choosen'''

	dwg = {
		   # for matchPhotos:
		   "accuracy":      PS.Accuracy.MediumAccuracy,
		   "preselection":  PS.Preselection.GenericPreselection,
		   "filter_mask":   True,
		   "keypoint_limit": 40000,
		   "tiepoint_limit": 4000,
		   #for builddensecloud
		   "quality":       PS.Quality.LowQuality,
		   "filter" :       PS.FilterMode.AggressiveFiltering,
		   # for buildModel
		   "surface_M":     PS.SurfaceType.HeightField,
		   "interpolation": PS.Interpolation.EnabledInterpolation,
		   "face_count":    PS.FaceCount.LowFaceCount,
		   # for DEM
		   "source_cloud":  PS.DataSource.PointCloudData,
		   #for Ortho
		   "surface_O":     PS.DataSource.ElevationData,
		   "blending":      PS.BlendingMode.MosaicBlending,
		   #for Contour
		   "source_data":   PS.DataSource.ElevationData,
		   "interval":      1,
		   "no_DenseCloud": True }

	if quality == "low":
		return dwg

	elif quality == "opt":
		dwg["preselection"] = PS.Preselection.ReferencePreselection
		dwg["source_cloud"] = PS.DataSource.DenseCloudData
		return dwg

	elif quality == "medium":
		dwg["preselection"] = PS.Preselection.ReferencePreselection
		dwg["source_cloud"] = PS.DataSource.DenseCloudData
		dwg["quality"] = PS.Quality.MediumQuality
		return dwg

	elif quality == "high":
		dwg["preselection"] = PS.Preselection.ReferencePreselection
		dwg["source_cloud"] = PS.DataSource.DenseCloudData
		dwg["quality"] = PS.Quality.UltraQuality
		dwg["blending"] = PS.BlendingMode.AverageBlending
		return dwg


	elif quality == "lowest":
		dwg["quality"] = PS.Quality.LowestQuality
		dwg["blending"] = PS.BlendingMode.AverageBlending
		return dwg

	else:
		print ("Not implemented profile.")
		return

def main():
	'''reads out the desired quality-profiles from config.ini. For each Profile
	the a automatisiation-run is called'''
	settings = PS.app.Settings(log_enable = True, log_path = "/home/ankommen/log.log")
	config = configparser.ConfigParser()

	cwd = os.path.dirname(os.path.abspath(__file__))
	config_file_path = cwd+'/config_cor.ini'
	logging.info('Reading config file for processing: ' + config_file_path)
	config.read(config_file_path)

	config.set("PATHS","input_images_path_manta",sys.argv[1])
	config.set("PATHS","input_images_path_thermal",sys.argv[2])

	with open(config_file_path,'w') as cfgfile:
		config.write(cfgfile)

	logging.info("Image Save Folders are: " + str(sys.argv[1])+" and: " +  str(sys.argv[2]))
	createPSImageList(sys.argv[1], RGB_IMAGE_INTERVAL)
	createPSImageList(sys.argv[2], THERMAL_IMAGE_INTERVAL)

	qual_ls = config.get("QUALITY", "qual")
	qual_list = qual_ls.replace("'","").replace(" ","").split(",")
	logging.info("Quality List is: " + str(qual_list))

	try:
		for qual in qual_list:
			dict_with_quality = choose_profile(qual)
			processing(dict_with_quality, qual)
			export(qual)
			sync_ankommen(sys.argv[1], qual)
	except Exception as e:
  		logging.error("Exception occurred", exc_info=True)
	

	logging.info("######## Metashape is now quitting after successful processing.  ############")
	#PS.app.quit()


main()