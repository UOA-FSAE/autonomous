# ZED SDK - Object Detection

This sample shows how to detect custom objects using the official Pytorch implementation of YOLOv7 from a ZED camera and ingest them into the ZED SDK to extract 3D informations and tracking for each objects.

## Getting Started

 - Get the latest [ZED SDK](https://www.stereolabs.com/developers/release/) and [pyZED Package](https://www.stereolabs.com/docs/app-development/python/install/)
 - Check the [Documentation](https://www.stereolabs.com/docs/object-detection/custom-od/)

## Setting up

 - YoloV7 folder is already a submodule of the repo, so we just need to install dependencies

```sh
cd yolov7
pip install -r requirements.txt
```
## Run the ZED-Part program 

*NOTE: The ZED v1 is not compatible with this module*

```
python detector-clean-worked.py --weights yolov7m.pt
or 
python detector-clean-worked.py --weights yolov7m.pt # [--img_size 512 --conf_thres 0.1 --svo path/to/file.svo]
```

## Support

If you need assistance go to our Community site at https://community.stereolabs.com/
