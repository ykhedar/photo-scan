#!/usr/bin/env python
# -*- coding: utf-8 -*-

## Created on 23.06.2016
## author: Simon T.
## maintainer: Y. K.

import sys

sys.path.append("/usr/local/lib/python2.7/dist-packages") # for exiftool

import PhotoScan as PS
import os
import glob
import configparser
import re
import exiftool
import operator
import csv

def createPSImageList(image_path):
    '''Creates a list of images to be processed by PS'''
    list_file = open(image_path+'/list.csv','w')
    file_list_sorted=os.listdir(image_path)
    file_list_sorted.sort()
    img_count = 0
    with exiftool.ExifTool() as et:
        for file in file_list_sorted:
            if file.endswith(".jpg"):
                file_path = os.path.join(image_path, file)
                imgNumber = et.get_tag("ImageNumber", file_path)
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
        if second - initTime > 3:
            final_list_file.write(element[0]+'\n')
            initTime = second
    final_list_file.close()


def processing(dwg, quality):
    '''Contains process-steps that are called in PhotoScan'''

    config = configparser.ConfigParser()
    cwd = os.path.dirname(os.path.abspath(__file__))
    config_file_path = cwd+'/config_cor.ini'
    config.read(config_file_path)

    input_dir_path = config["PATHS"]["input_images_path"]
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
    input_fotos = [line.rstrip('\n') for line in open(input_list_file)]

    chunk.addPhotos(input_fotos)

    if config["MISC"]["calibfile"] == "true":
        calib = PS.Calibration()
        calib.load(config["PATHS"]["calibfile_path"])

    doc.save(path = save_path +".psx")
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

    chunk.buildModel(dwg["surface_M"], dwg["interpolation"], dwg["face_count"])

    doc.save(path = save_path +".psx")

    chunk.buildUV(PS.MappingMode.AdaptiveOrthophotoMapping,count=1)
    chunk.buildTexture(PS.BlendingMode.MosaicBlending)

    doc.save(path = save_path +".psx")

    chunk.buildDem(dwg["source_cloud"], dwg["interpolation"])

    doc.save(path = save_path +".psx")

    chunk.buildOrthomosaic(surface = dwg["surface_O"], blending = dwg["blending"])

    chunk.buildContours(source_data = dwg["source_data"], interval = dwg["interval"])

    doc.save(path = save_path +".psx")

    return


def export(qual):
    '''exports the results from the project, into desired formats'''
    print ("#############################")
    print ("########Exporting############")
    print ("#############################")

    doc = PS.app.document

    config = configparser.ConfigParser()
    cwd = os.path.dirname(os.path.abspath(__file__))
    config_file_path = cwd+'/config_cor.ini'
    config.read(config_file_path)
    save_path = config["PATHS"]["input_images_path"] +"/PSResults/"
    crs = PhotoScan.CoordinateSystem("EPSG::32632")
    for chunk in doc.chunks:
        if qual == chunk.label.split("_")[-1]:
            print(chunk.label.split("_")[-1])

            orhtopfadjpg = save_path +"ortho_"+qual +".jpg"
            chunk.exportOrthomosaic(path = orhtopfadjpg, image_format = PS.ImageFormat.ImageFormatJPEG,
                            raster_transform = PS.RasterTransformType.RasterTransformNone,
                            write_kml = False, write_world = False,
                            tiff_compression = PS.TiffCompression.TiffCompressionNone, tiff_big = False)

            orhtopfadtiff = save_path +"ortho_"+qual +".tif"
            chunk.exportOrthomosaic(path = orhtopfadtiff, image_format = PS.ImageFormat.ImageFormatTIFF,
                            raster_transform = PS.RasterTransformType.RasterTransformNone,
                            write_kml = False, write_world = False,
                            tiff_compression = PS.TiffCompression.TiffCompressionNone, tiff_big = False, projection=crs)

            dempfad = save_path +"dem_"+qual +".tif"
            chunk.exportDem(path = dempfad, image_format = PS.ImageFormat.ImageFormatTIFF, nodata =-32767,
                        write_kml = False, write_world = False, tiff_big=False)

            modelpfad = save_path +"model_"+qual +".ply"
            chunk.exportModel(path = modelpfad, projection=crs)

            pointcloudfad = save_path +"ptcloud_"+qual +".ply"
            chunk.exportPoints(path = pointcloudfad, binary=True, precision=6,
            	          normals=True,colors=True, projection=crs)

            pointcloudasciifad = save_path +"ptcloudascii_"+qual +".ply"
            chunk.exportPoints(path = pointcloudasciifad, binary=False, precision=6,
                        normals=True,colors=True, projection=crs)

            reppfad = save_path +"rep_"+qual +".pdf"

            chunk.exportReport(reppfad,chunk.label)

def choose_profile(quality):
    '''creates a dict with options for the desired quality param in PhotoScan,
    depending on what quality-profile was choosen'''

    dwg = {
           # for matchPhotos:
           "accuracy":      PS.Accuracy.LowAccuracy,
           "preselection":  PS.Preselection.GenericPreselection,
           "filter_mask":   True,
           "keypoint_limit": 20000,
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
           "no_DenseCloud": False }

    if quality == "low":
        return dwg

    elif quality == "opt":
        dwg["accuracy"] = PS.Accuracy.LowestAccuracy
        dwg["preselection"] = PS.Preselection.ReferencePreselection
        dwg["source_cloud"] = PS.DataSource.DenseCloudData
        return dwg

    elif quality == "medium":
        dwg["accuracy"] = PS.Accuracy.MediumAccuracy
        dwg["preselection"] = PS.Preselection.ReferencePreselection
        dwg["source_cloud"] = PS.DataSource.DenseCloudData
        dwg["keypoint_limit"] = 40000
        dwg["quality"] = PS.Quality.MediumQuality
        return dwg
        
    elif quality == "high":
        dwg["accuracy"] = PS.Accuracy.HighAccuracy
        dwg["preselection"] = PS.Preselection.ReferencePreselection
        dwg["source_cloud"] = PS.DataSource.DenseCloudData
        dwg["keypoint_limit"] = 40000
        dwg["quality"] = PS.Quality.UltraQuality
        dwg["blending"] = PS.BlendingMode.AverageBlending
        return dwg


    elif quality == "lowest":
        dwg["accuracy"] = PS.Accuracy.LowestAccuracy
        dwg["keypoint_limit"] = 20000
        dwg["quality"] = PS.Quality.LowestQuality
        dwg["blending"] = PS.BlendingMode.AverageBlending
        return dwg

    else:
        print ("Not implemented profile.")
        return

def main():
    '''reads out the desired quality-profiles from config.ini. For each Profile
    the a automatisiation-run is called'''

    config = configparser.ConfigParser()

    cwd = os.path.dirname(os.path.abspath(__file__))
    config_file_path = cwd+'/config_cor.ini'

    config.read(config_file_path)

    config.set("PATHS","input_images_path",sys.argv[1])
    with open(config_file_path,'w') as cfgfile:
        config.write(cfgfile)
    createPSImageList(sys.argv[1])

    qual_ls = config.get("QUALITY", "qual")
    qual_list = qual_ls.replace("'","").replace(" ","").split(",")

    for qual in qual_list:
        dict_with_quality = choose_profile(qual)
        processing(dict_with_quality, qual)
        export(qual)
    PS.app.quit()


main()